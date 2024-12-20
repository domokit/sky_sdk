// Copyright 2013 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

#ifndef FLUTTER_LIB_UI_PAINTING_PATH_MEASURE_H_
#define FLUTTER_LIB_UI_PAINTING_PATH_MEASURE_H_

#include <vector>

#include "flutter/lib/ui/dart_wrapper.h"
#include "flutter/lib/ui/painting/path.h"
#include "third_party/skia/include/core/SkContourMeasure.h"
#include "third_party/skia/include/core/SkPath.h"
#include "third_party/tonic/typed_data/typed_list.h"

// Be sure that the client doesn't modify a path on us before Skia finishes
// See AOSP's reasoning in PathMeasure.cpp

namespace flutter {

class CanvasPathMeasure : public RefCountedDartWrappable<CanvasPathMeasure> {
  DEFINE_WRAPPERTYPEINFO();
  FML_FRIEND_MAKE_REF_COUNTED(CanvasPathMeasure);

 public:
  ~CanvasPathMeasure() override;
  static void Create(Dart_Handle wrapper,
                     const CanvasPath* path,
                     bool forceClosed);

  void setPath(const CanvasPath* path, bool isClosed);
  double getLength(int contour_index);
  tonic::Float32List getPosTan(int contour_index, double distance);
  void getSegment(Dart_Handle path_handle,
                  int contour_index,
                  double start_d,
                  double stop_d,
                  bool start_with_move_to);
  bool isClosed(int contour_index);
  bool nextContour();

  const SkContourMeasureIter& pathMeasure() const { return *path_measure_; }

 private:
  CanvasPathMeasure();

  std::unique_ptr<SkContourMeasureIter> path_measure_;
  std::vector<sk_sp<SkContourMeasure>> measures_;
};

}  // namespace flutter

#endif  // FLUTTER_LIB_UI_PAINTING_PATH_MEASURE_H_
