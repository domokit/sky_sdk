// Copyright 2013 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

#ifndef FLUTTER_SHELL_PLATFORM_FUCHSIA_FLUTTER_TESTS_FAKES_TOUCH_SOURCE_H_
#define FLUTTER_SHELL_PLATFORM_FUCHSIA_FLUTTER_TESTS_FAKES_TOUCH_SOURCE_H_

#include <fuchsia/ui/pointer/cpp/fidl.h>

#include <optional>
#include <vector>

#include "flutter/fml/logging.h"

namespace flutter_runner::testing {

// A test stub to act as the protocol server. A test can control what is sent
// back by this server implementation, via the ScheduleCallback call.
class FakeTouchSource : public fuchsia::ui::pointer::TouchSource {
 public:
  // |fuchsia.ui.pointer.TouchSource|
  void Watch(std::vector<fuchsia::ui::pointer::TouchResponse> responses,
             TouchSource::WatchCallback callback) override {
    responses_ = std::move(responses);
    callback_ = std::move(callback);
  }

  // Have the server issue events to the client's hanging-get Watch call.
  void ScheduleCallback(std::vector<fuchsia::ui::pointer::TouchEvent> events) {
    FML_CHECK(callback_) << "require a valid WatchCallback";
    callback_(std::move(events));
  }

  // Allow the test to observe what the client uploaded on the next Watch call.
  std::optional<std::vector<fuchsia::ui::pointer::TouchResponse>>
  UploadedResponses() {
    auto responses = std::move(responses_);
    responses_.reset();
    return responses;
  }

 private:
  // |fuchsia.ui.pointer.TouchSource|
  void UpdateResponse(fuchsia::ui::pointer::TouchInteractionId ixn,
                      fuchsia::ui::pointer::TouchResponse response,
                      TouchSource::UpdateResponseCallback callback) override {
    FML_UNREACHABLE();
  }

  // Client uploads responses to server.
  std::optional<std::vector<fuchsia::ui::pointer::TouchResponse>> responses_;

  // Client-side logic to invoke on Watch() call's return. A test triggers it
  // with ScheduleCallback().
  TouchSource::WatchCallback callback_;
};

}  // namespace flutter_runner::testing

#endif  // FLUTTER_SHELL_PLATFORM_FUCHSIA_FLUTTER_TESTS_FAKES_TOUCH_SOURCE_H_
