// Copyright 2013 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

#ifndef FLUTTER_LIB_UI_WINDOW_VIEW_FOCUS_H_
#define FLUTTER_LIB_UI_WINDOW_VIEW_FOCUS_H_

#include <cstdint>

namespace flutter {

// Focus state of a View.
// Must match ViewFocusState in ui/platform_dispatcher.dart.
enum class ViewFocusState : int64_t {
  kUnfocused = 0,
  kFocused,
};

// Represents the direction of which the focus transitioned over
// a FlutterView.
// Must match ViewFocusDirection in ui/platform_dispatcher.dart.
enum class ViewFocusDirection : int64_t {
  kUndefined = 0,
  kForward,
  kBackward,
};

// Event sent by the embedder to the engine indicating that native view focus
// state has changed.
class ViewFocusEvent {
 public:
  ViewFocusEvent(int64_t view_id,
                 ViewFocusState state,
                 ViewFocusDirection direction)
      : view_id_(view_id), state_(state), direction_(direction) {}

  int64_t view_id() const { return view_id_; }
  ViewFocusState state() const { return state_; }
  ViewFocusDirection direction() const { return direction_; }

 private:
  int64_t view_id_;
  ViewFocusState state_;
  ViewFocusDirection direction_;
};

// Request sent by the engine to the embedder indicating that the FlutterView
// focus state has changed and the native view should be updated.
class ViewFocusChangeRequest {
 public:
  ViewFocusChangeRequest(int64_t view_id,
                         ViewFocusState state,
                         ViewFocusDirection direction)
      : view_id_(view_id), state_(state), direction_(direction) {}

  int64_t view_id() const { return view_id_; }
  ViewFocusState state() const { return state_; }
  ViewFocusDirection direction() const { return direction_; }

 private:
  int64_t view_id_;
  ViewFocusState state_;
  ViewFocusDirection direction_;
};

}  // namespace flutter

#endif  // FLUTTER_LIB_UI_WINDOW_VIEW_FOCUS_H_
