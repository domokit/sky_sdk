// Copyright 2013 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

#ifndef FLUTTER_LIB_UI_PAINTING_CANVAS_H_
#define FLUTTER_LIB_UI_PAINTING_CANVAS_H_

#include "flutter/display_list/dl_blend_mode.h"
#include "flutter/display_list/dl_op_flags.h"
#include "flutter/lib/ui/dart_wrapper.h"
#include "flutter/lib/ui/painting/path.h"
#include "flutter/lib/ui/painting/picture.h"
#include "flutter/lib/ui/painting/picture_recorder.h"
#include "flutter/lib/ui/painting/rrect.h"
#include "flutter/lib/ui/painting/vertices.h"
#include "third_party/tonic/typed_data/typed_list.h"

namespace flutter {
class CanvasImage;

class Canvas : public RefCountedDartWrappable<Canvas>, DisplayListOpFlags {
  DEFINE_WRAPPERTYPEINFO();
  FML_FRIEND_MAKE_REF_COUNTED(Canvas);

 public:
  static void Create(Dart_Handle wrapper,
                     PictureRecorder* recorder,
                     double left,
                     double top,
                     double right,
                     double bottom);

  ~Canvas() override;

  void save();
  void saveLayerWithoutBounds(Dart_Handle paint_objects,
                              Dart_Handle paint_data);

  void saveLayer(double left,
                 double top,
                 double right,
                 double bottom,
                 Dart_Handle paint_objects,
                 Dart_Handle paint_data);

  void restore();
  int getSaveCount();
  void restoreToCount(int count);

  void translate(double dx, double dy);
  void scale(double sx, double sy);
  void rotate(double radians);
  void skew(double sx, double sy);
  void transform(const tonic::Float64List& matrix4);
  void getTransform(Dart_Handle matrix4_handle);

  void clipRect(double left,
                double top,
                double right,
                double bottom,
                DlCanvas::ClipOp clipOp,
                bool doAntiAlias = true);
  void clipRRect(const RRect& rrect, bool doAntiAlias = true);
  void clipPath(const CanvasPath* path, bool doAntiAlias = true);
  void getDestinationClipBounds(Dart_Handle rect_handle);
  void getLocalClipBounds(Dart_Handle rect_handle);

  void drawColor(uint32_t color, DlBlendMode blend_mode);

  void drawLine(double x1,
                double y1,
                double x2,
                double y2,
                Dart_Handle paint_objects,
                Dart_Handle paint_data);

  void drawPaint(Dart_Handle paint_objects, Dart_Handle paint_data);

  void drawRect(double left,
                double top,
                double right,
                double bottom,
                Dart_Handle paint_objects,
                Dart_Handle paint_data);

  void drawRRect(const RRect& rrect,
                 Dart_Handle paint_objects,
                 Dart_Handle paint_data);

  void drawDRRect(const RRect& outer,
                  const RRect& inner,
                  Dart_Handle paint_objects,
                  Dart_Handle paint_data);

  void drawOval(double left,
                double top,
                double right,
                double bottom,
                Dart_Handle paint_objects,
                Dart_Handle paint_data);

  void drawCircle(double x,
                  double y,
                  double radius,
                  Dart_Handle paint_objects,
                  Dart_Handle paint_data);

  void drawArc(double left,
               double top,
               double right,
               double bottom,
               double startAngle,
               double sweepAngle,
               bool useCenter,
               Dart_Handle paint_objects,
               Dart_Handle paint_data);

  void drawPath(const CanvasPath* path,
                Dart_Handle paint_objects,
                Dart_Handle paint_data);

  Dart_Handle drawImage(const CanvasImage* image,
                        double x,
                        double y,
                        Dart_Handle paint_objects,
                        Dart_Handle paint_data,
                        int filterQualityIndex);

  Dart_Handle drawImageRect(const CanvasImage* image,
                            double src_left,
                            double src_top,
                            double src_right,
                            double src_bottom,
                            double dst_left,
                            double dst_top,
                            double dst_right,
                            double dst_bottom,
                            Dart_Handle paint_objects,
                            Dart_Handle paint_data,
                            int filterQualityIndex);

  Dart_Handle drawImageNine(const CanvasImage* image,
                            double center_left,
                            double center_top,
                            double center_right,
                            double center_bottom,
                            double dst_left,
                            double dst_top,
                            double dst_right,
                            double dst_bottom,
                            Dart_Handle paint_objects,
                            Dart_Handle paint_data,
                            int bitmapSamplingIndex);

  void drawPicture(Picture* picture);

  // The paint argument is first for the following functions because Paint
  // unwraps a number of C++ objects. Once we create a view unto a
  // Float32List, we cannot re-enter the VM to unwrap objects. That means we
  // either need to process the paint argument first.

  void drawPoints(Dart_Handle paint_objects,
                  Dart_Handle paint_data,
                  DlCanvas::PointMode point_mode,
                  const tonic::Float32List& points);

  void drawVertices(const Vertices* vertices,
                    DlBlendMode blend_mode,
                    Dart_Handle paint_objects,
                    Dart_Handle paint_data);

  Dart_Handle drawAtlas(Dart_Handle paint_objects,
                        Dart_Handle paint_data,
                        int filterQualityIndex,
                        CanvasImage* atlas,
                        Dart_Handle transforms_handle,
                        Dart_Handle rects_handle,
                        Dart_Handle colors_handle,
                        DlBlendMode blend_mode,
                        Dart_Handle cull_rect_handle);

  void drawShadow(const CanvasPath* path,
                  uint32_t color,
                  double elevation,
                  bool transparentOccluder);

  void Invalidate();

  DisplayListBuilder* builder() { return display_list_builder_.get(); }

 private:
  explicit Canvas(sk_sp<DisplayListBuilder> builder);

  sk_sp<DisplayListBuilder> display_list_builder_;
};

}  // namespace flutter

#endif  // FLUTTER_LIB_UI_PAINTING_CANVAS_H_
