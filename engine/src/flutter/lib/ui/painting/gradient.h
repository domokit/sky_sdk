// Copyright 2013 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

#ifndef FLUTTER_LIB_UI_PAINTING_GRADIENT_H_
#define FLUTTER_LIB_UI_PAINTING_GRADIENT_H_

#include "flutter/display_list/effects/dl_color_source.h"
#include "flutter/lib/ui/painting/matrix.h"
#include "flutter/lib/ui/painting/shader.h"
#include "third_party/tonic/typed_data/typed_list.h"

namespace flutter {

class CanvasGradient : public Shader {
  DEFINE_WRAPPERTYPEINFO();
  FML_FRIEND_MAKE_REF_COUNTED(CanvasGradient);

 public:
  ~CanvasGradient() override;
  static void Create(Dart_Handle wrapper);

  void initLinear(const tonic::Float32List& end_points,
                  const tonic::Float32List& colors,
                  const tonic::Float32List& color_stops,
                  DlTileMode tile_mode,
                  const tonic::Float64List& matrix4);

  void initRadial(double center_x,
                  double center_y,
                  double radius,
                  const tonic::Float32List& colors,
                  const tonic::Float32List& color_stops,
                  DlTileMode tile_mode,
                  const tonic::Float64List& matrix4);

  void initSweep(double center_x,
                 double center_y,
                 const tonic::Float32List& colors,
                 const tonic::Float32List& color_stops,
                 DlTileMode tile_mode,
                 double start_angle,
                 double end_angle,
                 const tonic::Float64List& matrix4);

  void initTwoPointConical(double start_x,
                           double start_y,
                           double start_radius,
                           double end_x,
                           double end_y,
                           double end_radius,
                           const tonic::Float32List& colors,
                           const tonic::Float32List& color_stops,
                           DlTileMode tile_mode,
                           const tonic::Float64List& matrix4);

  std::shared_ptr<DlColorSource> shader(DlImageSampling sampling) override {
    // Gradient color sources do not have image sampling variants...
    return dl_shader_;
  }

 private:
  CanvasGradient();
  std::shared_ptr<DlColorSource> dl_shader_;
};

}  // namespace flutter

#endif  // FLUTTER_LIB_UI_PAINTING_GRADIENT_H_
