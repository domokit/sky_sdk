// Copyright 2013 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

#ifndef FLUTTER_SHELL_PLATFORM_FUCHSIA_FLUTTER_TESTS_FAKES_VIEW_REF_FOCUSED_H_
#define FLUTTER_SHELL_PLATFORM_FUCHSIA_FLUTTER_TESTS_FAKES_VIEW_REF_FOCUSED_H_

#include <fuchsia/ui/views/cpp/fidl.h>

using ViewRefFocused = fuchsia::ui::views::ViewRefFocused;

namespace flutter_runner::testing {

class FakeViewRefFocused : public ViewRefFocused {
 public:
  using WatchCallback = ViewRefFocused::WatchCallback;
  std::size_t times_watched = 0;

  void Watch(WatchCallback callback) override {
    callback_ = std::move(callback);
    ++times_watched;
  }

  void ScheduleCallback(bool focused) {
    fuchsia::ui::views::FocusState focus_state;
    focus_state.set_focused(focused);
    callback_(std::move(focus_state));
  }

 private:
  WatchCallback callback_;
};

}  // namespace flutter_runner::testing

#endif  // FLUTTER_SHELL_PLATFORM_FUCHSIA_FLUTTER_TESTS_FAKES_VIEW_REF_FOCUSED_H_
