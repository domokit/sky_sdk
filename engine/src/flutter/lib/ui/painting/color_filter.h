// Copyright 2013 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

#ifndef FLUTTER_LIB_UI_PAINTING_COLOR_FILTER_H_
#define FLUTTER_LIB_UI_PAINTING_COLOR_FILTER_H_

#include "flutter/display_list/effects/dl_color_filter.h"
#include "flutter/lib/ui/dart_wrapper.h"
#include "third_party/tonic/typed_data/typed_list.h"

namespace flutter {

// A handle to an SkCodec object.
//
// Doesn't mirror SkCodec's API but provides a simple sequential access API.
class ColorFilter : public RefCountedDartWrappable<ColorFilter> {
  DEFINE_WRAPPERTYPEINFO();
  FML_FRIEND_MAKE_REF_COUNTED(ColorFilter);

 public:
  static void Create(Dart_Handle wrapper);

  void initMode(int color, int blend_mode);
  void initMatrix(const tonic::Float32List& color_matrix);
  void initSrgbToLinearGamma();
  void initLinearToSrgbGamma();

  ~ColorFilter() override;

  const std::shared_ptr<const DlColorFilter> filter() const { return filter_; }

 private:
  std::shared_ptr<const DlColorFilter> filter_;
};

}  // namespace flutter

#endif  // FLUTTER_LIB_UI_PAINTING_COLOR_FILTER_H_
