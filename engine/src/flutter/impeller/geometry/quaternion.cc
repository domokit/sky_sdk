// Copyright 2013 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

#include "quaternion.h"
#include <sstream>

namespace impeller {

Quaternion Quaternion::Slerp(const Quaternion& to, double time) const {
  double cosine = Dot(to);
  if (fabs(cosine) < 1.0 - 1e-3 /* epsilon */) {
    /*
     *  Spherical Interpolation.
     */
    auto sine = sqrt(1.0 - cosine * cosine);
    auto angle = atan2(sine, cosine);
    auto sineInverse = 1.0 / sine;
    auto c0 = sin((1.0 - time) * angle) * sineInverse;
    auto c1 = sin(time * angle) * sineInverse;
    return *this * c0 + to * c1;
  } else {
    /*
     *  Linear Interpolation.
     */
    return (*this * (1.0 - time) + to * time).Normalize();
  }
}

}  // namespace impeller
