// Copyright 2013 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

#include "pointer_event_utility.h"

namespace flutter_runner::testing {

using fup_EventPhase = fuchsia::ui::pointer::EventPhase;
using fup_TouchEvent = fuchsia::ui::pointer::TouchEvent;
using fup_TouchIxnId = fuchsia::ui::pointer::TouchInteractionId;
using fup_TouchIxnResult = fuchsia::ui::pointer::TouchInteractionResult;
using fup_TouchPointerSample = fuchsia::ui::pointer::TouchPointerSample;
using fup_ViewParameters = fuchsia::ui::pointer::ViewParameters;
using fup_MouseEvent = fuchsia::ui::pointer::MouseEvent;
using fup_MousePointerSample = fuchsia::ui::pointer::MousePointerSample;
using fup_MouseDeviceInfo = fuchsia::ui::pointer::MouseDeviceInfo;

namespace {

fup_ViewParameters CreateViewParameters(
    std::array<std::array<float, 2>, 2> view,
    std::array<std::array<float, 2>, 2> viewport,
    std::array<float, 9> transform) {
  fup_ViewParameters params;
  fuchsia::ui::pointer::Rectangle view_rect;
  view_rect.min = view[0];
  view_rect.max = view[1];
  params.view = view_rect;
  fuchsia::ui::pointer::Rectangle viewport_rect;
  viewport_rect.min = viewport[0];
  viewport_rect.max = viewport[1];
  params.viewport = viewport_rect;
  params.viewport_to_view_transform = transform;
  return params;
}

}  // namespace

TouchEventBuilder TouchEventBuilder::New() {
  return TouchEventBuilder();
}

TouchEventBuilder& TouchEventBuilder::AddTime(zx_time_t time) {
  time_ = time;
  return *this;
}

TouchEventBuilder& TouchEventBuilder::AddSample(fup_TouchIxnId id,
                                                fup_EventPhase phase,
                                                std::array<float, 2> position) {
  sample_ = std::make_optional<fup_TouchPointerSample>();
  sample_->set_interaction(id);
  sample_->set_phase(phase);
  sample_->set_position_in_viewport(position);
  return *this;
}

TouchEventBuilder& TouchEventBuilder::AddViewParameters(
    std::array<std::array<float, 2>, 2> view,
    std::array<std::array<float, 2>, 2> viewport,
    std::array<float, 9> transform) {
  params_ = CreateViewParameters(std::move(view), std::move(viewport),
                                 std::move(transform));
  return *this;
}

TouchEventBuilder& TouchEventBuilder::AddResult(fup_TouchIxnResult result) {
  result_ = result;
  return *this;
}

fup_TouchEvent TouchEventBuilder::Build() {
  fup_TouchEvent event;
  if (time_) {
    event.set_timestamp(time_.value());
  }
  if (params_) {
    event.set_view_parameters(std::move(params_.value()));
  }
  if (sample_) {
    event.set_pointer_sample(std::move(sample_.value()));
  }
  if (result_) {
    event.set_interaction_result(std::move(result_.value()));
  }
  return event;
}

std::vector<fup_TouchEvent> TouchEventBuilder::BuildAsVector() {
  std::vector<fup_TouchEvent> events;
  events.emplace_back(Build());
  return events;
}

MouseEventBuilder MouseEventBuilder::New() {
  return MouseEventBuilder();
}

MouseEventBuilder& MouseEventBuilder::AddTime(zx_time_t time) {
  time_ = time;
  return *this;
}

MouseEventBuilder& MouseEventBuilder::AddSample(
    uint32_t id,
    std::array<float, 2> position,
    std::vector<uint8_t> pressed_buttons,
    std::array<int64_t, 2> scroll,
    std::array<int64_t, 2> scroll_in_physical_pixel,
    bool is_precision_scroll) {
  sample_ = std::make_optional<fup_MousePointerSample>();
  sample_->set_device_id(id);
  if (!pressed_buttons.empty()) {
    sample_->set_pressed_buttons(pressed_buttons);
  }
  sample_->set_position_in_viewport(position);
  if (scroll[0] != 0) {
    sample_->set_scroll_h(scroll[0]);
  }
  if (scroll[1] != 0) {
    sample_->set_scroll_v(scroll[1]);
  }
  if (scroll_in_physical_pixel[0] != 0) {
    sample_->set_scroll_h_physical_pixel(scroll_in_physical_pixel[0]);
  }
  if (scroll_in_physical_pixel[1] != 0) {
    sample_->set_scroll_v_physical_pixel(scroll_in_physical_pixel[1]);
  }
  sample_->set_is_precision_scroll(is_precision_scroll);
  return *this;
}

MouseEventBuilder& MouseEventBuilder::AddViewParameters(
    std::array<std::array<float, 2>, 2> view,
    std::array<std::array<float, 2>, 2> viewport,
    std::array<float, 9> transform) {
  params_ = CreateViewParameters(std::move(view), std::move(viewport),
                                 std::move(transform));
  return *this;
}

MouseEventBuilder& MouseEventBuilder::AddMouseDeviceInfo(
    uint32_t id,
    std::vector<uint8_t> buttons) {
  device_info_ = std::make_optional<fup_MouseDeviceInfo>();
  device_info_->set_id(id);
  device_info_->set_buttons(buttons);
  return *this;
}

fup_MouseEvent MouseEventBuilder::Build() {
  fup_MouseEvent event;
  if (time_) {
    event.set_timestamp(time_.value());
  }
  if (params_) {
    event.set_view_parameters(std::move(params_.value()));
  }
  if (sample_) {
    event.set_pointer_sample(std::move(sample_.value()));
  }
  if (device_info_) {
    event.set_device_info(std::move(device_info_.value()));
  }
  event.set_trace_flow_id(123);
  return event;
}

std::vector<fup_MouseEvent> MouseEventBuilder::BuildAsVector() {
  std::vector<fup_MouseEvent> events;
  events.emplace_back(Build());
  return events;
}

}  // namespace flutter_runner::testing
