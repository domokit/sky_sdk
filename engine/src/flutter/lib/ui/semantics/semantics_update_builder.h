// Copyright 2013 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

#ifndef FLUTTER_LIB_UI_SEMANTICS_SEMANTICS_UPDATE_BUILDER_H_
#define FLUTTER_LIB_UI_SEMANTICS_SEMANTICS_UPDATE_BUILDER_H_

#include <any>
#include <list>

#include "flutter/lib/ui/dart_wrapper.h"
#include "flutter/lib/ui/semantics/semantics_update.h"
#include "flutter/lib/ui/ui_dart_state.h"
#include "third_party/tonic/typed_data/typed_list.h"

namespace flutter {

class SemanticsUpdateBuilder
    : public RefCountedDartWrappable<SemanticsUpdateBuilder> {
  DEFINE_WRAPPERTYPEINFO();
  FML_FRIEND_MAKE_REF_COUNTED(SemanticsUpdateBuilder);

 public:
  static void Create(Dart_Handle wrapper) {
    UIDartState::ThrowIfUIOperationsProhibited();
    auto res = fml::MakeRefCounted<SemanticsUpdateBuilder>();
    res->AssociateWithDartWrapper(wrapper);
  }

  ~SemanticsUpdateBuilder() override;

  void updateNode(
      int id,
      int flags,
      int actions,
      int maxValueLength,
      int currentValueLength,
      int textSelectionBase,
      int textSelectionExtent,
      int platformViewId,
      int scrollChildren,
      int scrollIndex,
      double scrollPosition,
      double scrollExtentMax,
      double scrollExtentMin,
      double left,
      double top,
      double right,
      double bottom,
      double elevation,
      double thickness,
      std::string identifier,
      std::string label,
      const std::vector<NativeStringAttribute*>& labelAttributes,
      std::string value,
      const std::vector<NativeStringAttribute*>& valueAttributes,
      std::string increasedValue,
      const std::vector<NativeStringAttribute*>& increasedValueAttributes,
      std::string decreasedValue,
      const std::vector<NativeStringAttribute*>& decreasedValueAttributes,
      std::string hint,
      const std::vector<NativeStringAttribute*>& hintAttributes,
      std::string tooltip,
      int textDirection,
      const tonic::Float64List& transform,
      const tonic::Int32List& childrenInTraversalOrder,
      const tonic::Int32List& childrenInHitTestOrder,
      const tonic::Int32List& customAccessibilityActions,
      int headingLevel,
      std::string linkUrl);

  void updateCustomAction(int id,
                          std::string label,
                          std::string hint,
                          int overrideId);

  void build(Dart_Handle semantics_update_handle);

 private:
  explicit SemanticsUpdateBuilder();
  SemanticsNodeUpdates nodes_;
  CustomAccessibilityActionUpdates actions_;
};

}  // namespace flutter

#endif  // FLUTTER_LIB_UI_SEMANTICS_SEMANTICS_UPDATE_BUILDER_H_
