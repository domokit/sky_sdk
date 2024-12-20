// Copyright 2013 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

#ifndef FLUTTER_SHELL_PLATFORM_FUCHSIA_FLUTTER_TESTS_FAKES_FOCUSER_H_
#define FLUTTER_SHELL_PLATFORM_FUCHSIA_FLUTTER_TESTS_FAKES_FOCUSER_H_

#include <fuchsia/ui/views/cpp/fidl.h>
#include <fuchsia/ui/views/cpp/fidl_test_base.h>

#include <string>

#include "flutter/fml/logging.h"

using Focuser = fuchsia::ui::views::Focuser;

namespace flutter_runner::testing {

class FakeFocuser : public fuchsia::ui::views::testing::Focuser_TestBase {
 public:
  bool request_focus_called() const { return request_focus_called_; }

  void fail_request_focus(bool fail_request = true) {
    fail_request_focus_ = fail_request;
  }

 private:
  void RequestFocus(fuchsia::ui::views::ViewRef view_ref,
                    RequestFocusCallback callback) override {
    request_focus_called_ = true;
    auto result =
        fail_request_focus_
            ? fuchsia::ui::views::Focuser_RequestFocus_Result::WithErr(
                  fuchsia::ui::views::Error::DENIED)
            : fuchsia::ui::views::Focuser_RequestFocus_Result::WithResponse(
                  fuchsia::ui::views::Focuser_RequestFocus_Response());
    callback(std::move(result));
  }

  void NotImplemented_(const std::string& name) {
    FML_LOG(FATAL) << "flutter_runner::Testing::FakeFocuser does not implement "
                   << name;
  }

  bool request_focus_called_ = false;
  bool fail_request_focus_ = false;
};

}  // namespace flutter_runner::testing

#endif  // FLUTTER_SHELL_PLATFORM_FUCHSIA_FLUTTER_TESTS_FAKES_FOCUSER_H_
