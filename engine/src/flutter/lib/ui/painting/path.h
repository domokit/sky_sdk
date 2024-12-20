// Copyright 2013 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

#ifndef FLUTTER_LIB_UI_PAINTING_PATH_H_
#define FLUTTER_LIB_UI_PAINTING_PATH_H_

#include "flutter/lib/ui/dart_wrapper.h"
#include "flutter/lib/ui/painting/rrect.h"
#include "flutter/lib/ui/ui_dart_state.h"
#include "third_party/skia/include/core/SkPath.h"
#include "third_party/skia/include/pathops/SkPathOps.h"
#include "third_party/tonic/typed_data/typed_list.h"

namespace flutter {

class CanvasPath : public RefCountedDartWrappable<CanvasPath> {
  DEFINE_WRAPPERTYPEINFO();
  FML_FRIEND_MAKE_REF_COUNTED(CanvasPath);

 public:
  ~CanvasPath() override;

  static void CreateFrom(Dart_Handle path_handle, const SkPath& src) {
    auto path = fml::MakeRefCounted<CanvasPath>();
    path->AssociateWithDartWrapper(path_handle);
    path->sk_path_ = src;
  }

  static fml::RefPtr<CanvasPath> Create(Dart_Handle wrapper) {
    UIDartState::ThrowIfUIOperationsProhibited();
    auto res = fml::MakeRefCounted<CanvasPath>();
    res->AssociateWithDartWrapper(wrapper);
    return res;
  }

  int getFillType();
  void setFillType(int fill_type);

  void moveTo(double x, double y);
  void relativeMoveTo(double x, double y);
  void lineTo(double x, double y);
  void relativeLineTo(double x, double y);
  void quadraticBezierTo(double x1, double y1, double x2, double y2);
  void relativeQuadraticBezierTo(double x1, double y1, double x2, double y2);
  void cubicTo(double x1,
               double y1,
               double x2,
               double y2,
               double x3,
               double y3);
  void relativeCubicTo(double x1,
                       double y1,
                       double x2,
                       double y2,
                       double x3,
                       double y3);
  void conicTo(double x1, double y1, double x2, double y2, double w);
  void relativeConicTo(double x1, double y1, double x2, double y2, double w);
  void arcTo(double left,
             double top,
             double right,
             double bottom,
             double startAngle,
             double sweepAngle,
             bool forceMoveTo);
  void arcToPoint(double arcEndX,
                  double arcEndY,
                  double radiusX,
                  double radiusY,
                  double xAxisRotation,
                  bool isLargeArc,
                  bool isClockwiseDirection);
  void relativeArcToPoint(double arcEndDeltaX,
                          double arcEndDeltaY,
                          double radiusX,
                          double radiusY,
                          double xAxisRotation,
                          bool isLargeArc,
                          bool isClockwiseDirection);
  void addRect(double left, double top, double right, double bottom);
  void addOval(double left, double top, double right, double bottom);
  void addArc(double left,
              double top,
              double right,
              double bottom,
              double startAngle,
              double sweepAngle);
  void addPolygon(const tonic::Float32List& points, bool close);
  void addRRect(const RRect& rrect);
  void addPath(CanvasPath* path, double dx, double dy);

  void addPathWithMatrix(CanvasPath* path,
                         double dx,
                         double dy,
                         Dart_Handle matrix4_handle);

  void extendWithPath(CanvasPath* path, double dx, double dy);

  void extendWithPathAndMatrix(CanvasPath* path,
                               double dx,
                               double dy,
                               Dart_Handle matrix4_handle);

  void close();
  void reset();
  bool contains(double x, double y);
  void shift(Dart_Handle path_handle, double dx, double dy);

  void transform(Dart_Handle path_handle, Dart_Handle matrix4_handle);

  tonic::Float32List getBounds();
  bool op(CanvasPath* path1, CanvasPath* path2, int operation);
  void clone(Dart_Handle path_handle);

  const DlPath& path() const;

 private:
  CanvasPath();

  SkPath sk_path_;
  mutable std::optional<const DlPath> dl_path_;

  // Must be called whenever the path is created or mutated.
  void resetVolatility();
};

}  // namespace flutter

#endif  // FLUTTER_LIB_UI_PAINTING_PATH_H_
