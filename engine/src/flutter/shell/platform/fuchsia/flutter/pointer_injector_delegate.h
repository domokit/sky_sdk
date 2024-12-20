// Copyright 2013 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

#ifndef FLUTTER_SHELL_PLATFORM_FUCHSIA_FLUTTER_POINTER_INJECTOR_DELEGATE_H_
#define FLUTTER_SHELL_PLATFORM_FUCHSIA_FLUTTER_POINTER_INJECTOR_DELEGATE_H_

#include <fuchsia/ui/pointerinjector/cpp/fidl.h>
#include <fuchsia/ui/views/cpp/fidl.h>

#include <queue>
#include <unordered_map>
#include <vector>

#include "flutter/fml/macros.h"
#include "flutter/fml/memory/weak_ptr.h"
#include "flutter/lib/ui/window/platform_message.h"
#include "third_party/rapidjson/include/rapidjson/document.h"

namespace flutter_runner {

// This class is responsible for handling the platform messages related to
// pointer events and managing the lifecycle of
// |fuchsia.ui.pointerinjector.Device| client side endpoint for embedded views.
class PointerInjectorDelegate {
 public:
  static constexpr auto kPointerInjectorMethodPrefix =
      "View.pointerinjector.inject";

  PointerInjectorDelegate(fuchsia::ui::pointerinjector::RegistryHandle registry,
                          fuchsia::ui::views::ViewRef host_view_ref)
      : registry_(std::make_shared<fuchsia::ui::pointerinjector::RegistryPtr>(
            registry.Bind())),
        host_view_ref_(std::make_shared<fuchsia::ui::views::ViewRef>(
            std::move(host_view_ref))) {}

  // Handles the following pointer event related platform message requests:
  // View.Pointerinjector.inject
  //  - Attempts to dispatch a pointer event to the given viewRef. Completes
  //    with [0] when the pointer event is sent to the given viewRef.
  bool HandlePlatformMessage(
      rapidjson::Value request,
      fml::RefPtr<flutter::PlatformMessageResponse> response);

  // Adds an endpoint for |view_id| in |valid_views_| for lifecycle management.
  // Called in |PlatformView::OnChildViewViewRef()|.
  void OnCreateView(
      uint64_t view_id,
      std::optional<fuchsia::ui::views::ViewRef> view_ref = std::nullopt);

  // Closes the |fuchsia.ui.pointerinjector.Device| channel for |view_id| and
  // cleans up resources.
  void OnDestroyView(uint64_t view_id) { valid_views_.erase(view_id); }

 private:
  using ViewId = int64_t;

  struct PointerInjectorRequest {
    // The position of the pointer event in viewport's coordinate system.
    float x = 0.f, y = 0.f;

    // |fuchsia.ui.pointerinjector.PointerSample.pointer_id|.
    uint32_t pointer_id = 0;

    // |fuchsia.ui.pointerinjector.PointerSample.phase|.
    fuchsia::ui::pointerinjector::EventPhase phase =
        fuchsia::ui::pointerinjector::EventPhase::ADD;

    // |fuchsia.ui.pointerinjector.Event.trace_flow_id|.
    uint64_t trace_flow_id = 0;

    // Logical size of the view's coordinate system.
    std::array<float, 2> logical_size = {0.f, 0.f};

    // |fuchsia.ui.pointerinjector.Event.timestamp|.
    zx_time_t timestamp = 0;
  };

  // This class is responsible for dispatching pointer events to a view by first
  // registering the injector device using
  // |fuchsia.ui.pointerinjector.Registry.Register| and then injecting the
  // pointer event using |fuchsia.ui.pointerinjector.Device.Inject|.
  class PointerInjectorEndpoint {
   public:
    PointerInjectorEndpoint(
        std::shared_ptr<fuchsia::ui::pointerinjector::RegistryPtr> registry,
        std::shared_ptr<fuchsia::ui::views::ViewRef> host_view_ref,
        std::optional<fuchsia::ui::views::ViewRef> view_ref)
        : registry_(std::move(registry)),
          host_view_ref_(std::move(host_view_ref)),
          view_ref_(std::move(view_ref)),
          weak_factory_(this) {
      // Try to re-register the |device_| if the |device_| gets closed due to
      // some error.
      device_.set_error_handler(
          [weak = weak_factory_.GetWeakPtr()](auto status) {
            FML_LOG(WARNING)
                << "fuchsia.ui.pointerinjector.Device closed " << status;
            if (!weak) {
              return;
            }

            // Clear all the stale pointer events in |injector_events_| and
            // reset the state of |weak| so that any future calls do not inject
            // any stale pointer events.
            weak->Reset();
          });
    }

    // Registers |device_| if it has not been registered and calls
    // |DispatchPendingEvents()| to dispatch |request| to the view.
    void InjectEvent(PointerInjectorRequest request);

   private:
    // Registers with the pointer injector service.
    //
    // Sets |registered_| to true immediately after submitting the registration
    // request. This means that the registration request may still be in-flight
    // on the server side when the function returns. Events can safely be
    // injected into the channel while registration is pending ("feed forward").
    void RegisterInjector(const PointerInjectorRequest& request);

    // Recursively calls |fuchsia.ui.pointerinjector.Device.Inject| to dispatch
    // the pointer events in |injector_events_| to the view.
    void DispatchPendingEvents();

    void EnqueueEvent(fuchsia::ui::pointerinjector::Event event);

    // Resets |registered_|, |injection_in_flight_| and |injector_events_| so
    // that |device_| can be re-registered and future calls to
    // |fuchsia.ui.pointerinjector.Device.Inject| do not include any stale
    // pointer events.
    void Reset();

    // Set to true if there is a |fuchsia.ui.pointerinjector.Device.Inject| call
    // in progress. If true, the |fuchsia.ui.pointerinjector.Event| is buffered
    // in |injector_events_|.
    bool injection_in_flight_ = false;

    // Set to true if |device_| has been registered using
    // |fuchsia.ui.pointerinjector.Registry.Register|. False otherwise.
    bool registered_ = false;

    std::shared_ptr<fuchsia::ui::pointerinjector::RegistryPtr> registry_;

    // ViewRef for the main flutter app launching the embedded child views.
    std::shared_ptr<fuchsia::ui::views::ViewRef> host_view_ref_;

    // ViewRef for a flatland view.
    // Set in |OnCreateView|.
    std::optional<fuchsia::ui::views::ViewRef> view_ref_;

    fuchsia::ui::pointerinjector::DevicePtr device_;

    // A queue containing all the pending |fuchsia.ui.pointerinjector.Event|s
    // which have to be dispatched to the view.
    // Note: The size of a vector inside |injector_events_| should not exceed
    // |fuchsia.ui.pointerinjector.MAX_INJECT|.
    std::queue<std::vector<fuchsia::ui::pointerinjector::Event>>
        injector_events_;

    fml::WeakPtrFactory<PointerInjectorEndpoint>
        weak_factory_;  // Must be the last member.

    FML_DISALLOW_COPY_AND_ASSIGN(PointerInjectorEndpoint);
  };

  void Complete(fml::RefPtr<flutter::PlatformMessageResponse> response,
                std::string value);

  // Generates a |fuchsia.ui.pointerinjector.Event| from |request| by extracting
  // information like timestamp, trace flow id and pointer sample from
  // |request|.
  static fuchsia::ui::pointerinjector::Event ExtractPointerEvent(
      PointerInjectorRequest request);

  // A map of valid views keyed by its view id. A view can receive pointer
  // events only if it is present in |valid_views_|.
  std::unordered_map<ViewId, PointerInjectorEndpoint> valid_views_;

  std::shared_ptr<fuchsia::ui::pointerinjector::RegistryPtr> registry_;

  // ViewRef for the main flutter app launching the embedded child views.
  std::shared_ptr<fuchsia::ui::views::ViewRef> host_view_ref_;

  FML_DISALLOW_COPY_AND_ASSIGN(PointerInjectorDelegate);
};

}  // namespace flutter_runner
#endif  // FLUTTER_SHELL_PLATFORM_FUCHSIA_FLUTTER_POINTER_INJECTOR_DELEGATE_H_
