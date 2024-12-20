// Copyright 2013 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

#include "pointer_injector_delegate.h"
#include "flutter/fml/logging.h"

namespace flutter_runner {

using fup_Config = fuchsia::ui::pointerinjector::Config;
using fup_Context = fuchsia::ui::pointerinjector::Context;
using fup_Data = fuchsia::ui::pointerinjector::Data;
using fup_DeviceType = fuchsia::ui::pointerinjector::DeviceType;
using fup_DispatchPolicy = fuchsia::ui::pointerinjector::DispatchPolicy;
using fup_Event = fuchsia::ui::pointerinjector::Event;
using fup_EventPhase = fuchsia::ui::pointerinjector::EventPhase;
using fup_PointerSample = fuchsia::ui::pointerinjector::PointerSample;
using fup_Target = fuchsia::ui::pointerinjector::Target;
using fup_Viewport = fuchsia::ui::pointerinjector::Viewport;
using fuv_ViewRef = fuchsia::ui::views::ViewRef;
const auto fup_MAX_INJECT = fuchsia::ui::pointerinjector::MAX_INJECT;

namespace {

// clang-format off
  static constexpr std::array<float, 9> kIdentityMatrix = {
    1, 0, 0, // column one
    0, 1, 0, // column two
    0, 0, 1, // column three
  };
// clang-format on

}  // namespace

bool PointerInjectorDelegate::HandlePlatformMessage(
    rapidjson::Value request,
    fml::RefPtr<flutter::PlatformMessageResponse> response) {
  if (!registry_->is_bound()) {
    FML_LOG(WARNING)
        << "Lost connection to fuchsia.ui.pointerinjector.Registry";
    return false;
  }

  auto method = request.FindMember("method");
  if (method == request.MemberEnd() || !method->value.IsString()) {
    FML_LOG(ERROR) << "No method found in platform message.";
    return false;
  }

  if (method->value != kPointerInjectorMethodPrefix) {
    FML_LOG(ERROR) << "Unexpected platform message method, expected "
                      "View.pointerinjector.inject.";
    return false;
  }

  auto args_it = request.FindMember("args");
  if (args_it == request.MemberEnd() || !args_it->value.IsObject()) {
    FML_LOG(ERROR) << "No arguments found in platform message's method";
    return false;
  }

  const auto& args = args_it->value;

  auto view_id = args.FindMember("viewId");
  if (!view_id->value.IsUint64()) {
    FML_LOG(ERROR) << "Argument 'viewId' is not a uint64";
    return false;
  }

  auto id = view_id->value.GetUint64();
  if (valid_views_.count(id) == 0) {
    // A child view can get destroyed bottom-up, so the parent view may continue
    // injecting until all view state processing "catches up". Until then, it's
    // okay to accept a request to inject into a view that no longer exists.
    // Doing so avoids log pollution regarding "MissingPluginException".
    Complete(std::move(response), "[0]");
    return true;
  }

  auto phase = args.FindMember("phase");
  if (!phase->value.IsInt()) {
    FML_LOG(ERROR) << "Argument 'phase' is not a int";
    return false;
  }

  auto pointer_x = args.FindMember("x");
  if (!pointer_x->value.IsFloat() && !pointer_x->value.IsInt()) {
    FML_LOG(ERROR) << "Argument 'Pointer.X' is not a float";
    return false;
  }

  auto pointer_y = args.FindMember("y");
  if (!pointer_y->value.IsFloat() && !pointer_y->value.IsInt()) {
    FML_LOG(ERROR) << "Argument 'Pointer.Y' is not a float";
    return false;
  }

  auto pointer_id = args.FindMember("pointerId");
  if (!pointer_id->value.IsUint()) {
    FML_LOG(ERROR) << "Argument 'pointerId' is not a uint32";
    return false;
  }

  auto trace_flow_id = args.FindMember("traceFlowId");
  if (!trace_flow_id->value.IsInt()) {
    FML_LOG(ERROR) << "Argument 'traceFlowId' is not a int";
    return false;
  }

  auto width = args.FindMember("logicalWidth");
  if (!width->value.IsFloat() && !width->value.IsInt()) {
    FML_LOG(ERROR) << "Argument 'logicalWidth' is not a float";
    return false;
  }

  auto height = args.FindMember("logicalHeight");
  if (!height->value.IsFloat() && !height->value.IsInt()) {
    FML_LOG(ERROR) << "Argument 'logicalHeight' is not a float";
    return false;
  }

  auto timestamp = args.FindMember("timestamp");
  if (!timestamp->value.IsInt() && !timestamp->value.IsUint64()) {
    FML_LOG(ERROR) << "Argument 'timestamp' is not a int";
    return false;
  }

  PointerInjectorRequest event = {
      .x = pointer_x->value.GetFloat(),
      .y = pointer_y->value.GetFloat(),
      .pointer_id = pointer_id->value.GetUint(),
      .phase = static_cast<fup_EventPhase>(phase->value.GetInt()),
      .trace_flow_id = trace_flow_id->value.GetUint64(),
      .logical_size = {width->value.GetFloat(), height->value.GetFloat()},
      .timestamp = timestamp->value.GetInt()};

  // Inject the pointer event if the view has been created.
  valid_views_.at(id).InjectEvent(std::move(event));
  Complete(std::move(response), "[0]");
  return true;
}

void PointerInjectorDelegate::OnCreateView(
    uint64_t view_id,
    std::optional<fuv_ViewRef> view_ref) {
  FML_CHECK(valid_views_.count(view_id) == 0);

  auto [_, success] = valid_views_.try_emplace(
      view_id, registry_, host_view_ref_, std::move(view_ref));

  FML_CHECK(success);
}

fup_Event PointerInjectorDelegate::ExtractPointerEvent(
    PointerInjectorRequest request) {
  fup_Event event;
  event.set_timestamp(request.timestamp);
  event.set_trace_flow_id(request.trace_flow_id);

  fup_PointerSample pointer_sample;
  pointer_sample.set_pointer_id(request.pointer_id);
  pointer_sample.set_phase(request.phase);
  pointer_sample.set_position_in_viewport({request.x, request.y});

  fup_Data data;
  data.set_pointer_sample(std::move(pointer_sample));

  event.set_data(std::move(data));
  return event;
}

void PointerInjectorDelegate::Complete(
    fml::RefPtr<flutter::PlatformMessageResponse> response,
    std::string value) {
  if (response) {
    response->Complete(std::make_unique<fml::DataMapping>(
        std::vector<uint8_t>(value.begin(), value.end())));
  }
}

void PointerInjectorDelegate::PointerInjectorEndpoint::InjectEvent(
    PointerInjectorRequest request) {
  if (!registered_) {
    RegisterInjector(request);
  }

  auto event = ExtractPointerEvent(std::move(request));

  // Add the event to |injector_events_| and dispatch it to the view.
  EnqueueEvent(std::move(event));

  DispatchPendingEvents();
}

void PointerInjectorDelegate::PointerInjectorEndpoint::DispatchPendingEvents() {
  // Return if there is already a |fuchsia.ui.pointerinjector.Device.Inject|
  // call in flight. The new pointer events will be dispatched once the
  // in-progress call terminates.
  if (injection_in_flight_) {
    return;
  }

  // Dispatch the events present in |injector_events_|. Note that we recursively
  // call |DispatchPendingEvents| in the callback passed to the
  // |f.u.p.Device.Inject| call. This ensures that there is only one
  // |f.u.p.Device.Inject| call at a time. If a new pointer event comes when
  // there is a |f.u.p.Device.Inject| call in progress, it gets buffered in
  // |injector_events_| and is picked up later.
  if (!injector_events_.empty()) {
    auto events = std::move(injector_events_.front());
    injector_events_.pop();
    injection_in_flight_ = true;

    FML_CHECK(device_.is_bound());
    FML_CHECK(events.size() <= fup_MAX_INJECT);

    device_->Inject(std::move(events), [weak = weak_factory_.GetWeakPtr()] {
      if (!weak) {
        FML_LOG(WARNING) << "Use after free attempted.";
        return;
      }
      weak->injection_in_flight_ = false;
      weak->DispatchPendingEvents();
    });
  }
}

void PointerInjectorDelegate::PointerInjectorEndpoint::EnqueueEvent(
    fup_Event event) {
  // Add |event| in |injector_events_| keeping in mind that the vector size does
  // not exceed |fup_MAX_INJECT|.
  if (!injector_events_.empty() &&
      injector_events_.back().size() < fup_MAX_INJECT) {
    injector_events_.back().push_back(std::move(event));
  } else {
    std::vector<fup_Event> vec;
    vec.reserve(fup_MAX_INJECT);
    vec.push_back(std::move(event));
    injector_events_.push(std::move(vec));
  }
}

void PointerInjectorDelegate::PointerInjectorEndpoint::RegisterInjector(
    const PointerInjectorRequest& request) {
  if (registered_) {
    return;
  }

  fup_Config config;
  config.set_device_id(1);
  config.set_device_type(fup_DeviceType::TOUCH);
  config.set_dispatch_policy(fup_DispatchPolicy::EXCLUSIVE_TARGET);

  fup_Context context;
  fuv_ViewRef context_clone;
  fidl::Clone(*host_view_ref_, &context_clone);
  context.set_view(std::move(context_clone));
  config.set_context(std::move(context));

  FML_CHECK(view_ref_.has_value());
  fup_Target target;
  fuv_ViewRef target_clone;

  fidl::Clone(*view_ref_, &target_clone);
  target.set_view(std::move(target_clone));
  config.set_target(std::move(target));

  fup_Viewport viewport;
  viewport.set_viewport_to_context_transform(kIdentityMatrix);
  std::array<std::array<float, 2>, 2> extents{
      {/*min*/ {0, 0},
       /*max*/ {request.logical_size[0], request.logical_size[1]}}};
  viewport.set_extents(std::move(extents));
  config.set_viewport(std::move(viewport));

  FML_CHECK(registry_->is_bound());

  (*registry_)->Register(std::move(config), device_.NewRequest(), [] {});

  registered_ = true;
}

void PointerInjectorDelegate::PointerInjectorEndpoint::Reset() {
  injection_in_flight_ = false;
  registered_ = false;
  injector_events_ = {};
}

}  // namespace flutter_runner
