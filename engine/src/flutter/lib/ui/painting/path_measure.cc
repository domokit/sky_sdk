// Copyright 2013 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

#include "flutter/lib/ui/painting/path_measure.h"

#include <cmath>

#include "flutter/lib/ui/floating_point.h"
#include "flutter/lib/ui/painting/matrix.h"
#include "flutter/lib/ui/ui_dart_state.h"
#include "third_party/tonic/converter/dart_converter.h"
#include "third_party/tonic/dart_args.h"
#include "third_party/tonic/dart_binding_macros.h"
#include "third_party/tonic/dart_library_natives.h"

namespace flutter {

typedef CanvasPathMeasure PathMeasure;

IMPLEMENT_WRAPPERTYPEINFO(ui, PathMeasure);

void CanvasPathMeasure::Create(Dart_Handle wrapper,
                               const CanvasPath* path,
                               bool forceClosed) {
  UIDartState::ThrowIfUIOperationsProhibited();
  fml::RefPtr<CanvasPathMeasure> pathMeasure =
      fml::MakeRefCounted<CanvasPathMeasure>();
  if (path) {
    const SkPath& skPath = path->path().GetSkPath();
    SkScalar resScale = 1;
    pathMeasure->path_measure_ =
        std::make_unique<SkContourMeasureIter>(skPath, forceClosed, resScale);
  } else {
    pathMeasure->path_measure_ = std::make_unique<SkContourMeasureIter>();
  }
  pathMeasure->AssociateWithDartWrapper(wrapper);
}

CanvasPathMeasure::CanvasPathMeasure() {}

CanvasPathMeasure::~CanvasPathMeasure() {}

void CanvasPathMeasure::setPath(const CanvasPath* path, bool isClosed) {
  path_measure_->reset(path->path().GetSkPath(), isClosed);
}

double CanvasPathMeasure::getLength(int contour_index) {
  if (static_cast<std::vector<sk_sp<SkContourMeasure>>::size_type>(
          contour_index) < measures_.size()) {
    return measures_[contour_index]->length();
  }
  return -1;
}

tonic::Float32List CanvasPathMeasure::getPosTan(int contour_index,
                                                double distance) {
  tonic::Float32List posTan(Dart_NewTypedData(Dart_TypedData_kFloat32, 5));
  posTan[0] = 0;  // dart code will check for this for failure
  if (static_cast<std::vector<sk_sp<SkContourMeasure>>::size_type>(
          contour_index) >= measures_.size()) {
    return posTan;
  }

  SkPoint pos;
  SkVector tan;
  float fdistance = SafeNarrow(distance);
  bool success = measures_[contour_index]->getPosTan(fdistance, &pos, &tan);

  if (success) {
    posTan[0] = 1;  // dart code will check for this for success
    posTan[1] = pos.x();
    posTan[2] = pos.y();
    posTan[3] = tan.x();
    posTan[4] = tan.y();
  }

  return posTan;
}

void CanvasPathMeasure::getSegment(Dart_Handle path_handle,
                                   int contour_index,
                                   double start_d,
                                   double stop_d,
                                   bool start_with_move_to) {
  if (static_cast<std::vector<sk_sp<SkContourMeasure>>::size_type>(
          contour_index) >= measures_.size()) {
    CanvasPath::Create(path_handle);
  }
  SkPath dst;
  bool success = measures_[contour_index]->getSegment(
      SafeNarrow(start_d), SafeNarrow(stop_d), &dst, start_with_move_to);
  if (!success) {
    CanvasPath::Create(path_handle);
  } else {
    CanvasPath::CreateFrom(path_handle, dst);
  }
}

bool CanvasPathMeasure::isClosed(int contour_index) {
  if (static_cast<std::vector<sk_sp<SkContourMeasure>>::size_type>(
          contour_index) < measures_.size()) {
    return measures_[contour_index]->isClosed();
  }
  return false;
}

bool CanvasPathMeasure::nextContour() {
  auto measure = path_measure_->next();
  if (measure) {
    measures_.push_back(std::move(measure));
    return true;
  }
  return false;
}

}  // namespace flutter
