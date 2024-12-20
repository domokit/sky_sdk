// Copyright 2013 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

#ifndef FLUTTER_DISPLAY_LIST_DL_SAMPLING_OPTIONS_H_
#define FLUTTER_DISPLAY_LIST_DL_SAMPLING_OPTIONS_H_

namespace flutter {

enum class DlFilterMode {
  kNearest,  // single sample point (nearest neighbor)
  kLinear,   // interporate between 2x2 sample points (bilinear interpolation)

  kLast = kLinear,
};

enum class DlImageSampling {
  kNearestNeighbor,
  kLinear,
  kMipmapLinear,
  kCubic,
};

}  // namespace flutter

#endif  // FLUTTER_DISPLAY_LIST_DL_SAMPLING_OPTIONS_H_
