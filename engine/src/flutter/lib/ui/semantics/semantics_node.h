// Copyright 2013 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

#ifndef FLUTTER_LIB_UI_SEMANTICS_SEMANTICS_NODE_H_
#define FLUTTER_LIB_UI_SEMANTICS_SEMANTICS_NODE_H_

#include <cstdint>
#include <string>
#include <unordered_map>
#include <vector>

#include "third_party/skia/include/core/SkM44.h"
#include "third_party/skia/include/core/SkRect.h"

#include "flutter/lib/ui/semantics/string_attribute.h"

namespace flutter {

// Must match the SemanticsAction enum in semantics.dart and in each of the
// embedders.
enum class SemanticsAction : int32_t {
  kTap = 1 << 0,
  kLongPress = 1 << 1,
  kScrollLeft = 1 << 2,
  kScrollRight = 1 << 3,
  kScrollUp = 1 << 4,
  kScrollDown = 1 << 5,
  kIncrease = 1 << 6,
  kDecrease = 1 << 7,
  kShowOnScreen = 1 << 8,
  kMoveCursorForwardByCharacter = 1 << 9,
  kMoveCursorBackwardByCharacter = 1 << 10,
  kSetSelection = 1 << 11,
  kCopy = 1 << 12,
  kCut = 1 << 13,
  kPaste = 1 << 14,
  kDidGainAccessibilityFocus = 1 << 15,
  kDidLoseAccessibilityFocus = 1 << 16,
  kCustomAction = 1 << 17,
  kDismiss = 1 << 18,
  kMoveCursorForwardByWord = 1 << 19,
  kMoveCursorBackwardByWord = 1 << 20,
  kSetText = 1 << 21,
  kFocus = 1 << 22,
  kScrollToOffset = 1 << 23,
};

const int kVerticalScrollSemanticsActions =
    static_cast<int32_t>(SemanticsAction::kScrollUp) |
    static_cast<int32_t>(SemanticsAction::kScrollDown);

const int kHorizontalScrollSemanticsActions =
    static_cast<int32_t>(SemanticsAction::kScrollLeft) |
    static_cast<int32_t>(SemanticsAction::kScrollRight);

const int kScrollableSemanticsActions =
    kVerticalScrollSemanticsActions | kHorizontalScrollSemanticsActions;

/// C/C++ representation of `SemanticsFlags` defined in
/// `lib/ui/semantics.dart`.
///\warning This must match the `SemanticsFlags` enum in
///         `lib/ui/semantics.dart`.
/// See also:
///   - file://./../../../lib/ui/semantics.dart
enum class SemanticsFlags : int32_t {
  kHasCheckedState = 1 << 0,
  kIsChecked = 1 << 1,
  kIsSelected = 1 << 2,
  kIsButton = 1 << 3,
  kIsTextField = 1 << 4,
  kIsFocused = 1 << 5,
  kHasEnabledState = 1 << 6,
  kIsEnabled = 1 << 7,
  kIsInMutuallyExclusiveGroup = 1 << 8,
  kIsHeader = 1 << 9,
  kIsObscured = 1 << 10,
  kScopesRoute = 1 << 11,
  kNamesRoute = 1 << 12,
  kIsHidden = 1 << 13,
  kIsImage = 1 << 14,
  kIsLiveRegion = 1 << 15,
  kHasToggledState = 1 << 16,
  kIsToggled = 1 << 17,
  kHasImplicitScrolling = 1 << 18,
  kIsMultiline = 1 << 19,
  kIsReadOnly = 1 << 20,
  kIsFocusable = 1 << 21,
  kIsLink = 1 << 22,
  kIsSlider = 1 << 23,
  kIsKeyboardKey = 1 << 24,
  kIsCheckStateMixed = 1 << 25,
  kHasExpandedState = 1 << 26,
  kIsExpanded = 1 << 27,
  kHasSelectedState = 1 << 28,
};

const int kScrollableSemanticsFlags =
    static_cast<int32_t>(SemanticsFlags::kHasImplicitScrolling);

struct SemanticsNode {
  SemanticsNode();

  SemanticsNode(const SemanticsNode& other);

  ~SemanticsNode();

  bool HasAction(SemanticsAction action) const;
  bool HasFlag(SemanticsFlags flag) const;

  // Whether this node is for embedded platform views.
  bool IsPlatformViewNode() const;

  int32_t id = 0;
  int32_t flags = 0;
  int32_t actions = 0;
  int32_t maxValueLength = -1;
  int32_t currentValueLength = -1;
  int32_t textSelectionBase = -1;
  int32_t textSelectionExtent = -1;
  int32_t platformViewId = -1;
  int32_t scrollChildren = 0;
  int32_t scrollIndex = 0;
  double scrollPosition = std::nan("");
  double scrollExtentMax = std::nan("");
  double scrollExtentMin = std::nan("");
  double elevation = 0.0;
  double thickness = 0.0;
  std::string identifier;
  std::string label;
  StringAttributes labelAttributes;
  std::string hint;
  StringAttributes hintAttributes;
  std::string value;
  StringAttributes valueAttributes;
  std::string increasedValue;
  StringAttributes increasedValueAttributes;
  std::string decreasedValue;
  StringAttributes decreasedValueAttributes;
  std::string tooltip;
  int32_t textDirection = 0;  // 0=unknown, 1=rtl, 2=ltr

  SkRect rect = SkRect::MakeEmpty();  // Local space, relative to parent.
  SkM44 transform = SkM44{};          // Identity
  std::vector<int32_t> childrenInTraversalOrder;
  std::vector<int32_t> childrenInHitTestOrder;
  std::vector<int32_t> customAccessibilityActions;
  int32_t headingLevel = 0;

  std::string linkUrl;
};

// Contains semantic nodes that need to be updated.
//
// The keys in the map are stable node IDd, and the values contain
// semantic information for the node corresponding to the ID.
using SemanticsNodeUpdates = std::unordered_map<int32_t, SemanticsNode>;

}  // namespace flutter

#endif  // FLUTTER_LIB_UI_SEMANTICS_SEMANTICS_NODE_H_
