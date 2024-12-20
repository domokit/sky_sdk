// Copyright 2013 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

#include "flutter/fml/platform/fuchsia/message_loop_fuchsia.h"

#include <lib/async-loop/default.h>
#include <lib/async/cpp/task.h>
#include <lib/async/default.h>
#include <lib/zx/time.h>
#include <zircon/status.h>

#include "flutter/fml/platform/fuchsia/task_observers.h"

namespace fml {

namespace {

// See comment on `ExecuteAfterTaskObservers` for explanation.
static void LoopEpilogue(async_loop_t*, void*) {
  ExecuteAfterTaskObservers();
}

constexpr async_loop_config_t kLoopConfig = {
    .make_default_for_current_thread = false,
    .epilogue = &LoopEpilogue,
};

}  // namespace

MessageLoopFuchsia::MessageLoopFuchsia() : loop_(&kLoopConfig) {
  async_set_default_dispatcher(loop_.dispatcher());

  zx_status_t timer_status =
      zx::timer::create(ZX_TIMER_SLACK_LATE, ZX_CLOCK_MONOTONIC, &timer_);
  FML_CHECK(timer_status == ZX_OK)
      << "MessageLoopFuchsia failed to create timer; status="
      << zx_status_get_string(timer_status);
}

MessageLoopFuchsia::~MessageLoopFuchsia() {
  // It is only safe to unset the current thread's default dispatcher if it is
  // already pointing to this loop.
  if (async_get_default_dispatcher() == loop_.dispatcher()) {
    async_set_default_dispatcher(nullptr);
  }
}

void MessageLoopFuchsia::Run() {
  timer_wait_ = std::make_unique<async::Wait>(
      timer_.get(), ZX_TIMER_SIGNALED, 0,
      [this](async_dispatcher_t* dispatcher, async::Wait* wait,
             zx_status_t status, const zx_packet_signal_t* signal) {
        if (status == ZX_ERR_CANCELED) {
          return;
        }
        FML_CHECK(signal->observed & ZX_TIMER_SIGNALED);

        // Cancel the timer now, because `RunExpiredTasksNow` might not re-arm
        // the timer.  That would leave the timer in a signalled state and it
        // would trigger the async::Wait again immediately, creating a busy
        // loop.
        //
        // NOTE: It is not neccesary to synchronize this with the timer_.set()
        // call below, even though WakeUp() can be called from any thread and
        // thus timer_.set() can run in parallel with this timer_.cancel().
        //
        // Zircon will synchronize the 2 syscalls internally, and the Wait loop
        // here is resilient to cancel() and set() being called in any order.
        timer_.cancel();

        // Run the tasks, which may or may not re-arm the timer for the future.
        RunExpiredTasksNow();

        // Kick off the next iteration of the timer wait loop.
        zx_status_t wait_status = wait->Begin(loop_.dispatcher());
        FML_CHECK(wait_status == ZX_OK)
            << "MessageLoopFuchsia::WakeUp failed to wait for timer; status="
            << zx_status_get_string(wait_status);
      });

  // Kick off the first iteration of the timer wait loop.
  zx_status_t wait_status = timer_wait_->Begin(loop_.dispatcher());
  FML_CHECK(wait_status == ZX_OK)
      << "MessageLoopFuchsia::WakeUp failed to wait for timer; status="
      << zx_status_get_string(wait_status);

  // Kick off the underlying async loop that services the timer wait in addition
  // to other tasks and waits queued on its `async_dispatcher_t`.
  loop_.Run();

  // Ensure any pending waits on the timer are properly canceled.
  if (timer_wait_->is_pending()) {
    timer_wait_->Cancel();
    timer_.cancel();
  }
}

void MessageLoopFuchsia::Terminate() {
  loop_.Quit();
}

void MessageLoopFuchsia::WakeUp(fml::TimePoint time_point) {
  constexpr zx::duration kZeroSlack(0);
  zx::time due_time(time_point.ToEpochDelta().ToNanoseconds());

  zx_status_t timer_status = timer_.set(due_time, kZeroSlack);
  FML_CHECK(timer_status == ZX_OK)
      << "MessageLoopFuchsia::WakeUp failed to set timer; status="
      << zx_status_get_string(timer_status);
}

}  // namespace fml
