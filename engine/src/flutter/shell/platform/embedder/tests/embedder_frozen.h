// Copyright 2013 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

#ifndef FLUTTER_SHELL_PLATFORM_EMBEDDER_TESTS_EMBEDDER_FROZEN_H_
#define FLUTTER_SHELL_PLATFORM_EMBEDDER_TESTS_EMBEDDER_FROZEN_H_

#include "flutter/shell/platform/embedder/embedder.h"

// NO CHANGES ARE PERMITTED TO THE STRUCTS IN THIS FILE.
//
// This file contains the subset of the embedder API that is "frozen".
// "Frozen" APIs are locked and must not be modified to guarantee
// forward and backward ABI compatibility. The order, type, and size
// of the struct members below must remain the same, and members must
// not be added nor removed.
//
// This file serves as a "golden test" for structs in embedder.h
// and contains a snapshot of the expected "frozen" API. Tests
// then verify that these structs match the actual embedder API.

namespace flutter {
namespace testing {

// New members must not be added to `FlutterTransformation`
// as it would break the ABI of `FlutterSemanticsNode`.
// See: https://github.com/flutter/flutter/issues/121176
typedef struct {
  double scaleX;
  double skewX;
  double transX;
  double skewY;
  double scaleY;
  double transY;
  double pers0;
  double pers1;
  double pers2;
} FrozenFlutterTransformation;

// New members must not be added to `FlutterRect` as it would
// break the ABI of `FlutterSemanticsNode` and `FlutterDamage`.
// See: https://github.com/flutter/flutter/issues/121176
// See: https://github.com/flutter/flutter/issues/121347
typedef struct {
  double left;
  double top;
  double right;
  double bottom;
} FrozenFlutterRect;

// New members must not be added to `FlutterPoint` as it would
// break the ABI of `FlutterLayer`.
typedef struct {
  double x;
  double y;
} FrozenFlutterPoint;

// New members must not be added to `FlutterDamage` as it would
// break the ABI of `FlutterPresentInfo`.
typedef struct {
  size_t struct_size;
  size_t num_rects;
  FrozenFlutterRect* damage;
} FrozenFlutterDamage;

// New members must not be added to `FlutterSemanticsNode`
// as it would break the ABI of `FlutterSemanticsUpdate`.
// See: https://github.com/flutter/flutter/issues/121176
typedef struct {
  size_t struct_size;
  int32_t id;
  FlutterSemanticsFlag flags;
  FlutterSemanticsAction actions;
  int32_t text_selection_base;
  int32_t text_selection_extent;
  int32_t scroll_child_count;
  int32_t scroll_index;
  double scroll_position;
  double scroll_extent_max;
  double scroll_extent_min;
  double elevation;
  double thickness;
  const char* label;
  const char* hint;
  const char* value;
  const char* increased_value;
  const char* decreased_value;
  FlutterTextDirection text_direction;
  FrozenFlutterRect rect;
  FrozenFlutterTransformation transform;
  size_t child_count;
  const int32_t* children_in_traversal_order;
  const int32_t* children_in_hit_test_order;
  size_t custom_accessibility_actions_count;
  const int32_t* custom_accessibility_actions;
  FlutterPlatformViewIdentifier platform_view_id;
  const char* tooltip;
} FrozenFlutterSemanticsNode;

// New members must not be added to `FlutterSemanticsCustomAction`
// as it would break the ABI of `FlutterSemanticsUpdate`.
// See: https://github.com/flutter/flutter/issues/121176
typedef struct {
  size_t struct_size;
  int32_t id;
  FlutterSemanticsAction override_action;
  const char* label;
  const char* hint;
} FrozenFlutterSemanticsCustomAction;

}  // namespace testing
}  // namespace flutter

#endif  // FLUTTER_SHELL_PLATFORM_EMBEDDER_TESTS_EMBEDDER_FROZEN_H_
