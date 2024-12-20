// Copyright 2013 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

#ifndef FLUTTER_LIB_UI_SEMANTICS_CUSTOM_ACCESSIBILITY_ACTION_H_
#define FLUTTER_LIB_UI_SEMANTICS_CUSTOM_ACCESSIBILITY_ACTION_H_

#include "third_party/tonic/dart_library_natives.h"
#include "third_party/tonic/dart_wrappable.h"
#include "third_party/tonic/typed_data/typed_list.h"

namespace flutter {

/// A custom accessibility action is used to indicate additional semantics
/// actions that a user can perform on a semantics node beyond the
/// preconfigured options.
struct CustomAccessibilityAction {
  CustomAccessibilityAction();
  ~CustomAccessibilityAction();

  int32_t id = 0;
  int32_t overrideId = -1;
  std::string label;
  std::string hint;
};

// Contains custom accessibility actions that need to be updated.
//
// The keys in the map are stable action IDs, and the values contain
// semantic information for the action corresponding to that id.
using CustomAccessibilityActionUpdates =
    std::unordered_map<int32_t, CustomAccessibilityAction>;

}  // namespace flutter

#endif  // FLUTTER_LIB_UI_SEMANTICS_CUSTOM_ACCESSIBILITY_ACTION_H_
