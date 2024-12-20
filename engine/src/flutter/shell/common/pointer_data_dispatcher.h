// Copyright 2013 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

#ifndef FLUTTER_SHELL_COMMON_POINTER_DATA_DISPATCHER_H_
#define FLUTTER_SHELL_COMMON_POINTER_DATA_DISPATCHER_H_

#include "flutter/runtime/runtime_controller.h"
#include "flutter/shell/common/animator.h"

namespace flutter {

class PointerDataDispatcher;

//------------------------------------------------------------------------------
/// The `Engine` pointer data dispatcher that forwards the packet received from
/// `PlatformView::DispatchPointerDataPacket` on the platform thread, to
/// `Window::DispatchPointerDataPacket` on the UI thread.
///
/// This class is used to filter the packets so the Flutter framework on the UI
/// thread will receive packets with some desired properties. See
/// `SmoothPointerDataDispatcher` for an example which filters irregularly
/// delivered packets, and dispatches them in sync with the VSYNC signal.
///
/// This object will be owned by the engine because it relies on the engine's
/// `Animator` (which owns `VsyncWaiter`) and `RuntimeController` to do the
/// filtering. This object is currently designed to be only called from the UI
/// thread (no thread safety is guaranteed).
///
/// The `PlatformView` decides which subclass of `PointerDataDispatcher` is
/// constructed by sending a `PointerDataDispatcherMaker` to the engine's
/// constructor in `Shell::CreateShellOnPlatformThread`. This is needed because:
///   (1) Different platforms (e.g., Android, iOS) have different dispatchers
///       so the decision has to be made per `PlatformView`.
///   (2) The `PlatformView` can only be accessed from the PlatformThread while
///       this class (as owned by engine) can only be accessed in the UI thread.
///       Hence `PlatformView` creates a `PointerDataDispatchMaker` on the
///       platform thread, and sends it to the UI thread for the final
///       construction of the `PointerDataDispatcher`.
class PointerDataDispatcher {
 public:
  /// The interface for Engine to implement.
  class Delegate {
   public:
    /// Actually dispatch the packet using Engine's `animator_` and
    /// `runtime_controller_`.
    virtual void DoDispatchPacket(std::unique_ptr<PointerDataPacket> packet,
                                  uint64_t trace_flow_id) = 0;

    //--------------------------------------------------------------------------
    /// @brief    Schedule a secondary callback to be executed right after the
    ///           main `VsyncWaiter::AsyncWaitForVsync` callback (which is added
    ///           by `Animator::RequestFrame`).
    ///
    ///           Like the callback in `AsyncWaitForVsync`, this callback is
    ///           only scheduled to be called once per |id|, and it will be
    ///           called in the UI thread. If there is no AsyncWaitForVsync
    ///           callback (`Animator::RequestFrame` is not called), this
    ///           secondary callback will still be executed at vsync.
    ///
    ///           This callback is used to provide the vsync signal needed by
    ///           `SmoothPointerDataDispatcher`, and for `Animator` input flow
    ///           events.
    virtual void ScheduleSecondaryVsyncCallback(
        uintptr_t id,
        const fml::closure& callback) = 0;
  };

  //----------------------------------------------------------------------------
  /// @brief      Signal that `PlatformView` has a packet to be dispatched.
  ///
  /// @param[in]  packet             The `PointerDataPacket` to be dispatched.
  /// @param[in]  trace_flow_id      The id for `Animator::EnqueueTraceFlowId`.
  virtual void DispatchPacket(std::unique_ptr<PointerDataPacket> packet,
                              uint64_t trace_flow_id) = 0;

  //----------------------------------------------------------------------------
  /// @brief      Default destructor.
  virtual ~PointerDataDispatcher();
};

//------------------------------------------------------------------------------
/// The default dispatcher that forwards the packet without any modification.
///
class DefaultPointerDataDispatcher : public PointerDataDispatcher {
 public:
  explicit DefaultPointerDataDispatcher(Delegate& delegate)
      : delegate_(delegate) {}

  // |PointerDataDispatcer|
  void DispatchPacket(std::unique_ptr<PointerDataPacket> packet,
                      uint64_t trace_flow_id) override;

  virtual ~DefaultPointerDataDispatcher();

 protected:
  Delegate& delegate_;

  FML_DISALLOW_COPY_AND_ASSIGN(DefaultPointerDataDispatcher);
};

//------------------------------------------------------------------------------
/// A dispatcher that may temporarily store and defer the last received
/// PointerDataPacket if multiple packets are received in one VSYNC. The
/// deferred packet will be sent in the next vsync in order to smooth out the
/// events. This filters out irregular input events delivery to provide a smooth
/// scroll on iPhone X/Xs.
///
/// It works as follows:
///
/// When `DispatchPacket` is called while a previous pointer data dispatch is
/// still in progress (its frame isn't finished yet), it means that an input
/// event is delivered to us too fast. That potentially means a later event will
/// be too late which could cause the missing of a frame. Hence we'll cache it
/// in `pending_packet_` for the next frame to smooth it out.
///
/// If the input event is sent to us regularly at the same rate of VSYNC (say
/// at 60Hz), this would be identical to `DefaultPointerDataDispatcher` where
/// `runtime_controller_->DispatchPointerDataPacket` is always called right
/// away. That's because `is_pointer_data_in_progress_` will always be false
/// when `DispatchPacket` is called since it will be cleared by the end of a
/// frame through `ScheduleSecondaryVsyncCallback`. This is the case for all
/// Android/iOS devices before iPhone X/XS.
///
/// If the input event is irregular, but with a random latency of no more than
/// one frame, this would guarantee that we'll miss at most 1 frame. Without
/// this, we could miss half of the frames.
///
/// If the input event is delivered at a higher rate than that of VSYNC, this
/// would at most add a latency of one event delivery. For example, if the
/// input event is delivered at 120Hz (this is only true for iPad pro, not even
/// iPhone X), this may delay the handling of an input event by 8ms.
///
/// The assumption of this solution is that the sampling itself is still
/// regular. Only the event delivery is allowed to be irregular. So far this
/// assumption seems to hold on all devices. If it's changed in the future,
/// we'll need a different solution.
///
/// See also input_events_unittests.cc where we test all our claims above.
class SmoothPointerDataDispatcher : public DefaultPointerDataDispatcher {
 public:
  explicit SmoothPointerDataDispatcher(Delegate& delegate);

  // |PointerDataDispatcer|
  void DispatchPacket(std::unique_ptr<PointerDataPacket> packet,
                      uint64_t trace_flow_id) override;

  virtual ~SmoothPointerDataDispatcher();

 private:
  void DispatchPendingPacket();
  void ScheduleSecondaryVsyncCallback();

  // If non-null, this will be a pending pointer data packet for the next frame
  // to consume. This is used to smooth out the irregular drag events delivery.
  // See also `DispatchPointerDataPacket` and input_events_unittests.cc.
  std::unique_ptr<PointerDataPacket> pending_packet_;
  int pending_trace_flow_id_ = -1;
  bool is_pointer_data_in_progress_ = false;

  // WeakPtrFactory must be the last member.
  fml::WeakPtrFactory<SmoothPointerDataDispatcher> weak_factory_;
  FML_DISALLOW_COPY_AND_ASSIGN(SmoothPointerDataDispatcher);
};

//--------------------------------------------------------------------------
/// @brief      Signature for constructing PointerDataDispatcher.
///
/// @param[in]  delegate      the `Flutter::Engine`
///
using PointerDataDispatcherMaker =
    std::function<std::unique_ptr<PointerDataDispatcher>(
        PointerDataDispatcher::Delegate&)>;

}  // namespace flutter

#endif  // FLUTTER_SHELL_COMMON_POINTER_DATA_DISPATCHER_H_
