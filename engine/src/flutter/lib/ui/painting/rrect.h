// Copyright 2013 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

#ifndef FLUTTER_LIB_UI_PAINTING_RRECT_H_
#define FLUTTER_LIB_UI_PAINTING_RRECT_H_

#include "flutter/display_list/geometry/dl_geometry_types.h"
#include "third_party/dart/runtime/include/dart_api.h"
#include "third_party/skia/include/core/SkRRect.h"
#include "third_party/tonic/converter/dart_converter.h"

namespace flutter {

class RRect {
 public:
  DlRoundRect rrect;
  bool is_null;
};

}  // namespace flutter

namespace tonic {

template <>
struct DartConverter<flutter::RRect> {
  using NativeType = flutter::RRect;
  using FfiType = Dart_Handle;
  static constexpr const char* kFfiRepresentation = "Handle";
  static constexpr const char* kDartRepresentation = "Object";
  static constexpr bool kAllowedInLeafCall = false;

  static NativeType FromDart(Dart_Handle handle);
  static NativeType FromArguments(Dart_NativeArguments args,
                                  int index,
                                  Dart_Handle& exception);

  static NativeType FromFfi(FfiType val) { return FromDart(val); }
  static const char* GetFfiRepresentation() { return kFfiRepresentation; }
  static const char* GetDartRepresentation() { return kDartRepresentation; }
  static bool AllowedInLeafCall() { return kAllowedInLeafCall; }
};

}  // namespace tonic

#endif  // FLUTTER_LIB_UI_PAINTING_RRECT_H_
