// Copyright 2013 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

#ifndef FLUTTER_IMPELLER_GEOMETRY_TRIG_H_
#define FLUTTER_IMPELLER_GEOMETRY_TRIG_H_

#include <functional>
#include <vector>

#include "flutter/impeller/geometry/point.h"

namespace impeller {

/// @brief  A structure to store the sine and cosine of an angle.
struct Trig {
  /// Construct a Trig object from a given angle in radians.
  explicit Trig(Radians r)
      : cos(std::cos(r.radians)), sin(std::sin(r.radians)) {}

  /// Construct a Trig object from the given cosine and sine values.
  Trig(double cos, double sin) : cos(cos), sin(sin) {}

  double cos;
  double sin;

  /// @brief  Returns the corresponding point on a circle of a given |radius|.
  Vector2 operator*(double radius) const {
    return Vector2(static_cast<Scalar>(cos * radius),
                   static_cast<Scalar>(sin * radius));
  }

  /// @brief  Returns the corresponding point on an ellipse with the given size.
  Vector2 operator*(const Size& ellipse_radii) const {
    return Vector2(static_cast<Scalar>(cos * ellipse_radii.width),
                   static_cast<Scalar>(sin * ellipse_radii.height));
  }
};

}  // namespace impeller

#endif  // FLUTTER_IMPELLER_GEOMETRY_TRIG_H_
