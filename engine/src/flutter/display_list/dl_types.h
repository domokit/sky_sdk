// Copyright 2013 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

#ifndef FLUTTER_DISPLAY_LIST_DL_TYPES_H_
#define FLUTTER_DISPLAY_LIST_DL_TYPES_H_

namespace flutter {

enum class DlClipOp {
  kDifference,
  kIntersect,
};

enum class DlPointMode {
  kPoints,   //!< draw each point separately
  kLines,    //!< draw each separate pair of points as a line segment
  kPolygon,  //!< draw each pair of overlapping points as a line segment
};

enum class DlSrcRectConstraint {
  kStrict,
  kFast,
};

}  // namespace flutter

#endif  // FLUTTER_DISPLAY_LIST_DL_TYPES_H_
