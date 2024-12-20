// Copyright 2013 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

#include "pointer_delegate.h"

#include <lib/trace/event.h>
#include <zircon/status.h>
#include <zircon/types.h>

#include <limits>

#include "flutter/fml/logging.h"
#include "flutter/fml/trace_event.h"

// TODO(fxbug.dev/87076): Add MouseSource tests.

namespace fuchsia::ui::pointer {
// For using TouchInteractionId as a map key.
bool operator==(const fuchsia::ui::pointer::TouchInteractionId& a,
                const fuchsia::ui::pointer::TouchInteractionId& b) {
  return a.device_id == b.device_id && a.pointer_id == b.pointer_id &&
         a.interaction_id == b.interaction_id;
}
}  // namespace fuchsia::ui::pointer

namespace flutter_runner {

using fup_EventPhase = fuchsia::ui::pointer::EventPhase;
using fup_MouseDeviceInfo = fuchsia::ui::pointer::MouseDeviceInfo;
using fup_MouseEvent = fuchsia::ui::pointer::MouseEvent;
using fup_TouchEvent = fuchsia::ui::pointer::TouchEvent;
using fup_TouchIxnStatus = fuchsia::ui::pointer::TouchInteractionStatus;
using fup_TouchResponse = fuchsia::ui::pointer::TouchResponse;
using fup_TouchResponseType = fuchsia::ui::pointer::TouchResponseType;
using fup_ViewParameters = fuchsia::ui::pointer::ViewParameters;

namespace {
void IssueTouchTraceEvent(const fup_TouchEvent& event) {
  FML_DCHECK(event.has_trace_flow_id()) << "API guarantee";
  TRACE_FLOW_END("input", "dispatch_event_to_client", event.trace_flow_id());
}

void IssueMouseTraceEvent(const fup_MouseEvent& event) {
  FML_DCHECK(event.has_trace_flow_id()) << "API guarantee";
  TRACE_FLOW_END("input", "dispatch_event_to_client", event.trace_flow_id());
}

bool HasValidatedTouchSample(const fup_TouchEvent& event) {
  if (!event.has_pointer_sample()) {
    return false;
  }
  FML_DCHECK(event.pointer_sample().has_interaction()) << "API guarantee";
  FML_DCHECK(event.pointer_sample().has_phase()) << "API guarantee";
  FML_DCHECK(event.pointer_sample().has_position_in_viewport())
      << "API guarantee";
  return true;
}

bool HasValidatedMouseSample(const fup_MouseEvent& event) {
  if (!event.has_pointer_sample()) {
    return false;
  }
  const auto& sample = event.pointer_sample();
  FML_DCHECK(sample.has_device_id()) << "API guarantee";
  FML_DCHECK(sample.has_position_in_viewport()) << "API guarantee";
  FML_DCHECK(!sample.has_pressed_buttons() || !sample.pressed_buttons().empty())
      << "API guarantee";

  return true;
}

std::array<float, 2> ViewportToViewCoordinates(
    std::array<float, 2> viewport_coordinates,
    const std::array<float, 9>& viewport_to_view_transform) {
  // The transform matrix is a FIDL array with matrix data in column-major
  // order. For a matrix with data [a b c d e f g h i], and with the viewport
  // coordinates expressed as homogeneous coordinates, the logical view
  // coordinates are obtained with the following formula:
  //   |a d g|   |x|   |x'|
  //   |b e h| * |y| = |y'|
  //   |c f i|   |1|   |w'|
  // which we then normalize based on the w component:
  //   if z' not zero: (x'/w', y'/w')
  //   else (x', y')
  const auto& M = viewport_to_view_transform;
  const float x = viewport_coordinates[0];
  const float y = viewport_coordinates[1];
  const float xp = M[0] * x + M[3] * y + M[6];
  const float yp = M[1] * x + M[4] * y + M[7];
  const float wp = M[2] * x + M[5] * y + M[8];
  if (wp != 0) {
    return {xp / wp, yp / wp};
  } else {
    return {xp, yp};
  }
}

flutter::PointerData::Change GetChangeFromTouchEventPhase(
    fup_EventPhase phase) {
  switch (phase) {
    case fup_EventPhase::ADD:
      return flutter::PointerData::Change::kAdd;
    case fup_EventPhase::CHANGE:
      return flutter::PointerData::Change::kMove;
    case fup_EventPhase::REMOVE:
      return flutter::PointerData::Change::kRemove;
    case fup_EventPhase::CANCEL:
      return flutter::PointerData::Change::kCancel;
    default:
      return flutter::PointerData::Change::kCancel;
  }
}

std::array<float, 2> ClampToViewSpace(const float x,
                                      const float y,
                                      const fup_ViewParameters& p) {
  const float min_x = p.view.min[0];
  const float min_y = p.view.min[1];
  const float max_x = p.view.max[0];
  const float max_y = p.view.max[1];
  if (min_x <= x && x < max_x && min_y <= y && y < max_y) {
    return {x, y};  // No clamping to perform.
  }

  // View boundary is [min_x, max_x) x [min_y, max_y). Note that min is
  // inclusive, but max is exclusive - so we subtract epsilon.
  const float max_x_inclusive = max_x - std::numeric_limits<float>::epsilon();
  const float max_y_inclusive = max_y - std::numeric_limits<float>::epsilon();
  const float& clamped_x = std::clamp(x, min_x, max_x_inclusive);
  const float& clamped_y = std::clamp(y, min_y, max_y_inclusive);
  FML_LOG(INFO) << "Clamped (" << x << ", " << y << ") to (" << clamped_x
                << ", " << clamped_y << ").";
  return {clamped_x, clamped_y};
}

flutter::PointerData::Change ComputePhase(
    bool any_button_down,
    std::unordered_set<uint32_t>& mouse_down,
    uint32_t id) {
  if (!mouse_down.count(id) && !any_button_down) {
    return flutter::PointerData::Change::kHover;
  } else if (!mouse_down.count(id) && any_button_down) {
    mouse_down.insert(id);
    return flutter::PointerData::Change::kDown;
  } else if (mouse_down.count(id) && any_button_down) {
    return flutter::PointerData::Change::kMove;
  } else if (mouse_down.count(id) && !any_button_down) {
    mouse_down.erase(id);
    return flutter::PointerData::Change::kUp;
  }

  FML_UNREACHABLE();
  return flutter::PointerData::Change::kCancel;
}

// Flutter's PointerData.device field is 64 bits and is expected to be unique
// for each pointer. We pack Fuchsia's device ID (hi) and pointer ID (lo) into
// 64 bits to retain uniqueness across multiple touch devices.
uint64_t PackFuchsiaDeviceIdAndPointerId(uint32_t fuchsia_device_id,
                                         uint32_t fuchsia_pointer_id) {
  return (((uint64_t)fuchsia_device_id) << 32) | fuchsia_pointer_id;
}

// It returns a "draft" because the coordinates are logical. Later, view pixel
// ratio is applied to obtain physical coordinates.
//
// The flutter pointerdata state machine has extra phases, which this function
// synthesizes on the fly. Hence the return data is a flutter pointerdata, and
// optionally a second one.
// For example: <ADD, DOWN>, <MOVE, nullopt>, <UP, REMOVE>.
// TODO(fxbug.dev/87074): Let PointerDataPacketConverter synthesize events.
//
// Flutter gestures expect a gesture to start within the logical view space, and
// is not tolerant of floating point drift. This function coerces just the DOWN
// event's coordinate to start within the logical view.
std::pair<flutter::PointerData, std::optional<flutter::PointerData>>
CreateTouchDraft(const fup_TouchEvent& event,
                 const fup_ViewParameters& view_parameters) {
  FML_DCHECK(HasValidatedTouchSample(event)) << "precondition";
  const auto& sample = event.pointer_sample();
  const auto& ixn = sample.interaction();

  flutter::PointerData ptr;
  ptr.Clear();
  ptr.time_stamp = event.timestamp() / 1000;  // in microseconds
  ptr.change = GetChangeFromTouchEventPhase(sample.phase());
  ptr.kind = flutter::PointerData::DeviceKind::kTouch;
  // Load Fuchsia's pointer ID onto Flutter's |device| field, and not the
  // |pointer_identifier| field. The latter is written by
  // PointerDataPacketConverter, to track individual gesture interactions.
  ptr.device = PackFuchsiaDeviceIdAndPointerId(ixn.device_id, ixn.pointer_id);
  // View parameters can change mid-interaction; apply transform on the fly.
  auto logical =
      ViewportToViewCoordinates(sample.position_in_viewport(),
                                view_parameters.viewport_to_view_transform);
  ptr.physical_x = logical[0];  // Not yet physical; adjusted in PlatformView.
  ptr.physical_y = logical[1];  // Not yet physical; adjusted in PlatformView.

  // Match Flutter pointer's state machine with synthesized events.
  if (ptr.change == flutter::PointerData::Change::kAdd) {
    flutter::PointerData down;
    memcpy(&down, &ptr, sizeof(flutter::PointerData));
    down.change = flutter::PointerData::Change::kDown;
    {  // Ensure gesture recognition: DOWN starts in the logical view space.
      auto [x, y] =
          ClampToViewSpace(down.physical_x, down.physical_y, view_parameters);
      down.physical_x = x;
      down.physical_y = y;
    }
    return {std::move(ptr), std::move(down)};
  } else if (ptr.change == flutter::PointerData::Change::kRemove) {
    flutter::PointerData up;
    memcpy(&up, &ptr, sizeof(flutter::PointerData));
    up.change = flutter::PointerData::Change::kUp;
    return {std::move(up), std::move(ptr)};
  } else {
    return {std::move(ptr), std::nullopt};
  }
}

// It returns a "draft" because the coordinates are logical. Later, view pixel
// ratio is applied to obtain physical coordinates.
//
// Phase data is computed before this call; it involves state tracking based on
// button-down state.
//
// Button data, if available, gets packed into the |buttons| field, in flutter
// button order (kMousePrimaryButton, etc). The device-assigned button IDs are
// provided in priority order in MouseEvent.device_info (at the start of channel
// connection), and maps from device button ID (given in fup_MouseEvent) to
// flutter button ID (flutter::PointerData).
//
// Scroll data, if available, gets packed into the |scroll_delta_x| or
// |scroll_delta_y| fields, and the |signal_kind| field is set to kScroll.
// The PointerDataPacketConverter reads this field to synthesize events to match
// Flutter's expected pointer stream.
// TODO(fxbug.dev/87073): PointerDataPacketConverter should synthesize a
// discrete scroll event on kDown or kUp, to match engine expectations.
//
// Flutter gestures expect a gesture to start within the logical view space, and
// is not tolerant of floating point drift. This function coerces just the DOWN
// event's coordinate to start within the logical view.
flutter::PointerData CreateMouseDraft(const fup_MouseEvent& event,
                                      const flutter::PointerData::Change phase,
                                      const fup_ViewParameters& view_parameters,
                                      const fup_MouseDeviceInfo& device_info) {
  FML_DCHECK(HasValidatedMouseSample(event)) << "precondition";
  const auto& sample = event.pointer_sample();

  flutter::PointerData ptr;
  ptr.Clear();
  ptr.time_stamp = event.timestamp() / 1000;  // in microseconds
  ptr.change = phase;
  ptr.kind = flutter::PointerData::DeviceKind::kMouse;
  ptr.device = sample.device_id();

  // View parameters can change mid-interaction; apply transform on the fly.
  auto logical =
      ViewportToViewCoordinates(sample.position_in_viewport(),
                                view_parameters.viewport_to_view_transform);
  ptr.physical_x = logical[0];  // Not yet physical; adjusted in PlatformView.
  ptr.physical_y = logical[1];  // Not yet physical; adjusted in PlatformView.

  // Ensure gesture recognition: DOWN starts in the logical view space.
  if (ptr.change == flutter::PointerData::Change::kDown) {
    auto [x, y] =
        ClampToViewSpace(ptr.physical_x, ptr.physical_y, view_parameters);
    ptr.physical_x = x;
    ptr.physical_y = y;
  }

  if (sample.has_pressed_buttons()) {
    int64_t flutter_buttons = 0;
    const auto& pressed = sample.pressed_buttons();
    for (size_t idx = 0; idx < pressed.size(); ++idx) {
      const uint8_t button_id = pressed[idx];
      FML_DCHECK(device_info.has_buttons()) << "API guarantee";
      // Priority 0 maps to kPrimaryButton, and so on.
      for (uint8_t prio = 0; prio < device_info.buttons().size(); ++prio) {
        if (button_id == device_info.buttons()[prio]) {
          flutter_buttons |= (1 << prio);
        }
      }
    }
    FML_DCHECK(flutter_buttons != 0);
    ptr.buttons = flutter_buttons;
  }

  // Fuchsia previously only provided scroll data in "ticks", not physical
  // pixels. On legacy platforms, since Flutter expects scroll data in physical
  // pixels, to compensate for lack of guidance, we make up a "reasonable
  // amount".
  // TODO(fxbug.dev/103443): Remove the tick based scrolling after the
  // transition.
  const int kScrollOffsetMultiplier = 20;

  double dy = 0;
  double dx = 0;
  bool is_scroll = false;

  if (sample.has_scroll_v_physical_pixel()) {
    dy = -sample.scroll_v_physical_pixel();
    is_scroll = true;
  } else if (sample.has_scroll_v()) {
    dy = -sample.scroll_v() *
         kScrollOffsetMultiplier;  // logical amount, not yet physical; adjusted
                                   // in Platform View.
    is_scroll = true;
  }

  if (sample.has_scroll_h_physical_pixel()) {
    dx = sample.scroll_h_physical_pixel();
    is_scroll = true;
  } else if (sample.has_scroll_h()) {
    dx = sample.scroll_h() * kScrollOffsetMultiplier;  // logical amount
    is_scroll = true;
  }

  if (is_scroll) {
    ptr.signal_kind = flutter::PointerData::SignalKind::kScroll;
    ptr.scroll_delta_y = dy;
    ptr.scroll_delta_x = dx;
  }

  return ptr;
}

// Helper to insert one or two events into a vector buffer.
void InsertIntoBuffer(
    std::pair<flutter::PointerData, std::optional<flutter::PointerData>> events,
    std::vector<flutter::PointerData>* buffer) {
  FML_DCHECK(buffer);
  buffer->emplace_back(std::move(events.first));
  if (events.second.has_value()) {
    buffer->emplace_back(std::move(events.second.value()));
  }
}
}  // namespace

PointerDelegate::PointerDelegate(
    fuchsia::ui::pointer::TouchSourceHandle touch_source,
    fuchsia::ui::pointer::MouseSourceHandle mouse_source)
    : touch_source_(touch_source.Bind()), mouse_source_(mouse_source.Bind()) {
  if (touch_source_) {
    touch_source_.set_error_handler([](zx_status_t status) {
      FML_LOG(ERROR) << "TouchSource channel error: << "
                     << zx_status_get_string(status);
    });
  }
  if (mouse_source_) {
    mouse_source_.set_error_handler([](zx_status_t status) {
      FML_LOG(ERROR) << "MouseSource channel error: << "
                     << zx_status_get_string(status);
    });
  }
}
// Core logic of this class.
// Aim to keep state management in this function.
void PointerDelegate::WatchLoop(
    std::function<void(std::vector<flutter::PointerData>)> callback) {
  FML_LOG(INFO) << "Flutter - PointerDelegate started.";
  if (touch_responder_) {
    FML_LOG(ERROR) << "PointerDelegate::WatchLoop() must be called once.";
    return;
  }

  touch_responder_ = [this, callback](std::vector<fup_TouchEvent> events) {
    TRACE_EVENT0("flutter", "PointerDelegate::TouchHandler");
    FML_DCHECK(touch_responses_.empty()) << "precondition";
    std::vector<flutter::PointerData> to_client;
    for (const fup_TouchEvent& event : events) {
      IssueTouchTraceEvent(event);
      fup_TouchResponse
          response;  // Response per event, matched on event's index.
      if (event.has_view_parameters()) {
        touch_view_parameters_ = std::move(event.view_parameters());
      }
      if (HasValidatedTouchSample(event)) {
        const auto& sample = event.pointer_sample();
        const auto& ixn = sample.interaction();
        if (sample.phase() == fup_EventPhase::ADD &&
            !event.has_interaction_result()) {
          touch_buffer_.emplace(ixn, std::vector<flutter::PointerData>());
        }

        FML_DCHECK(touch_view_parameters_.has_value()) << "API guarantee";
        auto events = CreateTouchDraft(event, touch_view_parameters_.value());
        if (touch_buffer_.count(ixn) > 0) {
          InsertIntoBuffer(std::move(events), &touch_buffer_[ixn]);
        } else {
          InsertIntoBuffer(std::move(events), &to_client);
        }
        // For this simple client, always claim we want the gesture.
        response.set_response_type(fup_TouchResponseType::YES);
      }
      if (event.has_interaction_result()) {
        const auto& result = event.interaction_result();
        const auto& ixn = result.interaction;
        if (result.status == fup_TouchIxnStatus::GRANTED &&
            touch_buffer_.count(ixn) > 0) {
          FML_DCHECK(to_client.empty()) << "invariant";
          to_client.insert(to_client.end(), touch_buffer_[ixn].begin(),
                           touch_buffer_[ixn].end());
        }
        touch_buffer_.erase(ixn);  // Result seen, delete the buffer.
      }
      touch_responses_.push_back(std::move(response));
    }
    callback(std::move(to_client));  // Notify client of touch events, if any.

    touch_source_->Watch(std::move(touch_responses_),
                         /*copy*/ touch_responder_);
    touch_responses_.clear();
  };

  mouse_responder_ = [this, callback](std::vector<fup_MouseEvent> events) {
    TRACE_EVENT0("flutter", "PointerDelegate::MouseHandler");
    std::vector<flutter::PointerData> to_client;
    for (fup_MouseEvent& event : events) {
      IssueMouseTraceEvent(event);
      if (event.has_device_info()) {
        const auto& id = event.device_info().id();
        mouse_device_info_[id] = std::move(*event.mutable_device_info());
      }
      if (event.has_view_parameters()) {
        mouse_view_parameters_ = std::move(event.view_parameters());
      }
      if (HasValidatedMouseSample(event)) {
        const auto& sample = event.pointer_sample();
        const auto& id = sample.device_id();
        const bool any_button_down = sample.has_pressed_buttons();
        FML_DCHECK(mouse_view_parameters_.has_value()) << "API guarantee";
        FML_DCHECK(mouse_device_info_.count(id) > 0) << "API guarantee";

        const auto phase = ComputePhase(any_button_down, mouse_down_, id);
        flutter::PointerData data =
            CreateMouseDraft(event, phase, mouse_view_parameters_.value(),
                             mouse_device_info_[id]);
        to_client.emplace_back(std::move(data));
      }
    }
    callback(std::move(to_client));
    mouse_source_->Watch(/*copy*/ mouse_responder_);
  };

  // Start watching both channels.
  touch_source_->Watch(std::move(touch_responses_), /*copy*/ touch_responder_);
  touch_responses_.clear();
  if (mouse_source_) {
    mouse_source_->Watch(/*copy*/ mouse_responder_);
  }
}

}  // namespace flutter_runner
