// Copyright 2013 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

#ifndef FLUTTER_SHELL_PLATFORM_FUCHSIA_FLUTTER_TESTS_FAKES_MOUSE_SOURCE_H_
#define FLUTTER_SHELL_PLATFORM_FUCHSIA_FLUTTER_TESTS_FAKES_MOUSE_SOURCE_H_

#include <fuchsia/ui/pointer/cpp/fidl.h>

#include "flutter/fml/logging.h"

namespace flutter_runner::testing {

// A test stub to act as the protocol server. A test can control what is sent
// back by this server implementation, via the ScheduleCallback call.
class FakeMouseSource : public fuchsia::ui::pointer::MouseSource {
 public:
  // |fuchsia.ui.pointer.MouseSource|
  void Watch(MouseSource::WatchCallback callback) override {
    callback_ = std::move(callback);
  }

  // Have the server issue events to the client's hanging-get Watch call.
  void ScheduleCallback(std::vector<fuchsia::ui::pointer::MouseEvent> events) {
    FML_CHECK(callback_) << "require a valid WatchCallback";
    callback_(std::move(events));
  }

 private:
  // Client-side logic to invoke on Watch() call's return. A test triggers it
  // with ScheduleCallback().
  MouseSource::WatchCallback callback_;
};

}  // namespace flutter_runner::testing

#endif  // FLUTTER_SHELL_PLATFORM_FUCHSIA_FLUTTER_TESTS_FAKES_MOUSE_SOURCE_H_
