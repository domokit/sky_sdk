// Copyright 2013 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

#ifndef FLUTTER_SHELL_PLATFORM_FUCHSIA_FLUTTER_POINTER_DELEGATE_H_
#define FLUTTER_SHELL_PLATFORM_FUCHSIA_FLUTTER_POINTER_DELEGATE_H_

#include <fuchsia/ui/pointer/cpp/fidl.h>

#include <array>
#include <functional>
#include <optional>
#include <unordered_map>
#include <unordered_set>
#include <vector>

#include "flutter/lib/ui/window/pointer_data.h"

namespace flutter_runner {

// Helper class for keying into a map.
struct IxnHasher {
  std::size_t operator()(
      const fuchsia::ui::pointer::TouchInteractionId& ixn) const {
    return std::hash<uint32_t>()(ixn.device_id) ^
           std::hash<uint32_t>()(ixn.pointer_id) ^
           std::hash<uint32_t>()(ixn.interaction_id);
  }
};

// Channel processors for fuchsia.ui.pointer.TouchSource and MouseSource
// protocols. It manages the channel state, collects touch and mouse events, and
// surfaces them to PlatformView as flutter::PointerData events for further
// processing and dispatch.
class PointerDelegate {
 public:
  PointerDelegate(fuchsia::ui::pointer::TouchSourceHandle touch_source,
                  fuchsia::ui::pointer::MouseSourceHandle mouse_source);

  // This function collects Fuchsia's TouchPointerSample and MousePointerSample
  // data and transforms them into flutter::PointerData structs. It then calls
  // the supplied callback with a vector of flutter::PointerData, which (1) does
  // last processing (applies metrics), and (2) packs these flutter::PointerData
  // in a flutter::PointerDataPacket for transport to the Engine.
  void WatchLoop(
      std::function<void(std::vector<flutter::PointerData>)> callback);

 private:
  /***** TOUCH STATE *****/

  // Channel for touch events from Scenic.
  fuchsia::ui::pointer::TouchSourcePtr touch_source_;

  // Receive touch events from Scenic. Must be copyable.
  std::function<void(std::vector<fuchsia::ui::pointer::TouchEvent>)>
      touch_responder_;

  // Per-interaction buffer of touch events from Scenic. When an interaction
  // starts with event.pointer_sample.phase == ADD, we allocate a buffer and
  // store samples. When interaction ownership becomes
  // event.interaction_result.status == GRANTED, we flush the buffer to client,
  // delete the buffer, and all future events in this interaction are flushed
  // direct to client. When interaction ownership becomes DENIED, we delete the
  // buffer, and the client does not get any previous or future events in this
  // interaction.
  //
  // There are three basic interaction forms that we need to handle, and the API
  // guarantees we see only these three forms. S=sample, R(g)=result-granted,
  // R(d)=result-denied, and + means packaged in the same table. Time flows from
  // left to right. Samples start with ADD, and end in REMOVE or CANCEL. Each
  // interaction receives just one ownership result.
  //   (1) Late grant. S S S R(g) S S S
  //   (1-a) Combo.    S S S+R(g) S S S
  //   (2) Early grant. S+R(g) S S S S S
  //   (3) Late deny. S S S R(d)
  //   (3-a) Combo.   S S S+R(d)
  //
  // This results in the following high-level algorithm to correctly deal with
  // buffer allocation and deletion, and event flushing or event dropping based
  // on ownership.
  //   if event.sample.phase == ADD && !event.result
  //     allocate buffer[event.sample.interaction]
  //   if buffer[event.sample.interaction]
  //     buffer[event.sample.interaction].push(event.sample)
  //   else
  //     flush_to_client(event.sample)
  //   if event.result
  //     if event.result == GRANTED
  //       flush_to_client(buffer[event.result.interaction])
  //     delete buffer[event.result.interaction]
  std::unordered_map<fuchsia::ui::pointer::TouchInteractionId,
                     std::vector<flutter::PointerData>,
                     IxnHasher>
      touch_buffer_;

  // The fuchsia.ui.pointer.TouchSource protocol allows one in-flight
  // hanging-get Watch() call to gather touch events, and the client is expected
  // to respond with consumption intent on the following hanging-get Watch()
  // call. Store responses here for the next call.
  std::vector<fuchsia::ui::pointer::TouchResponse> touch_responses_;

  // The fuchsia.ui.pointer.TouchSource protocol issues channel-global view
  // parameters on connection and on change. Events must apply these view
  // parameters to correctly map to logical view coordinates. The "nullopt"
  // state represents the absence of view parameters, early in the protocol
  // lifecycle.
  std::optional<fuchsia::ui::pointer::ViewParameters> touch_view_parameters_;

  /***** MOUSE STATE *****/

  // Channel for mouse events from Scenic.
  fuchsia::ui::pointer::MouseSourcePtr mouse_source_;

  // Receive mouse events from Scenic. Must be copyable.
  std::function<void(std::vector<fuchsia::ui::pointer::MouseEvent>)>
      mouse_responder_;

  // The set of mouse devices that are currently interacting with the UI.
  // A mouse is considered flutter::PointerData::Change::kDown if any button is
  // pressed. This set is used to correctly set the phase in
  // flutter::PointerData.change, with this high-level algorithm:
  //   if !mouse_down[id] && !button then: change = kHover
  //   if !mouse_down[id] &&  button then: change = kDown; mouse_down.add(id)
  //   if  mouse_down[id] &&  button then: change = kMove
  //   if  mouse_down[id] && !button then: change = kUp; mouse_down.remove(id)
  std::unordered_set</*mouse device ID*/ uint32_t> mouse_down_;

  // For each mouse device, its device-specific information, such as mouse
  // button priority order.
  std::unordered_map</*mouse device ID*/ uint32_t,
                     fuchsia::ui::pointer::MouseDeviceInfo>
      mouse_device_info_;

  // The fuchsia.ui.pointer.MouseSource protocol issues channel-global view
  // parameters on connection and on change. Events must apply these view
  // parameters to correctly map to logical view coordinates. The "nullopt"
  // state represents the absence of view parameters, early in the protocol
  // lifecycle.
  std::optional<fuchsia::ui::pointer::ViewParameters> mouse_view_parameters_;
};

}  // namespace flutter_runner
#endif  // FLUTTER_SHELL_PLATFORM_FUCHSIA_FLUTTER_POINTER_DELEGATE_H_
