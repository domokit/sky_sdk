// Copyright 2013 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

#include "flutter/impeller/geometry/round_rect.h"

namespace impeller {

static inline void NormalizeEmptyToZero(Size& radii) {
  if (radii.IsEmpty()) {
    radii = Size();
  }
}

static inline void AdjustScale(Scalar& radius1,
                               Scalar& radius2,
                               Scalar dimension,
                               Scalar& scale) {
  FML_DCHECK(radius1 >= 0.0f && radius2 >= 0.0f);
  FML_DCHECK(dimension > 0.0f);
  if (radius1 + radius2 > dimension) {
    scale = std::min(scale, dimension / (radius1 + radius2));
  }
}

RoundRect RoundRect::MakeRectRadii(const Rect& bounds,
                                   const RoundingRadii& in_radii) {
  if (bounds.IsEmpty() || !bounds.IsFinite() ||  //
      in_radii.AreAllCornersEmpty() || !in_radii.IsFinite()) {
    // preserve the empty bounds as they might be strokable
    return RoundRect(bounds, RoundingRadii());
  }

  // Copy the incoming radii so that we can work on normalizing them to the
  // particular rectangle they are paired with without disturbing the caller.
  RoundingRadii radii = in_radii;

  // If any corner is flat or has a negative value, normalize it to zeros
  // We do this first so that the unnecessary non-flat part of that radius
  // does not contribute to the global scaling below.
  NormalizeEmptyToZero(radii.top_left);
  NormalizeEmptyToZero(radii.top_right);
  NormalizeEmptyToZero(radii.bottom_left);
  NormalizeEmptyToZero(radii.bottom_right);

  // Now determine a global scale to apply to all of the radii to ensure
  // that none of the adjacent pairs of radius values sum to larger than
  // the corresponding dimension of the rectangle.
  Size size = bounds.GetSize();
  Scalar scale = 1.0f;
  // clang-format off
  AdjustScale(radii.top_left.width,    radii.top_right.width,     size.width,
              scale);
  AdjustScale(radii.bottom_left.width, radii.bottom_right.width,  size.width,
              scale);
  AdjustScale(radii.top_left.height,   radii.bottom_left.height,  size.height,
              scale);
  AdjustScale(radii.top_right.height,  radii.bottom_right.height, size.height,
              scale);
  // clang-format on
  if (scale < 1.0f) {
    radii = radii * scale;
  }

  return RoundRect(bounds, radii);
}

// Determine if p is inside the elliptical corner curve defined by the
// indicated corner point and the indicated radii.
// p - is the test point in absolute coordinates
// corner - is the location of the associated corner in absolute coordinates
// direction - is the sign of (corner - center), or the sign of coordinates
//             as they move in the direction of the corner from inside the
//             rect ((-1,-1) for the upper left corner for instance)
// radii - the non-negative X and Y size of the corner's radii.
static bool CornerContains(const Point& p,
                           const Point& corner,
                           const Point& direction,
                           const Size& radii) {
  FML_DCHECK(radii.width >= 0.0f && radii.height >= 0.0f);
  if (radii.IsEmpty()) {
    // This corner is not curved, therefore the containment is the same as
    // the previously checked bounds containment.
    return true;
  }

  // The positive X,Y distance between the corner and the point.
  Point corner_relative = (corner - p) * direction;

  // The distance from the "center" of the corner's elliptical curve.
  // If both numbers are positive then we need to do an elliptical distance
  // check to determine if it is inside the curve.
  // If either number is negative, then the point is outside this quadrant
  // and is governed by inclusion in the bounds and inclusion within other
  // corners of this round rect. In that case, we return true here to allow
  // further evaluation within other quadrants.
  Point quadrant_relative = radii - corner_relative;
  if (quadrant_relative.x <= 0.0f || quadrant_relative.y <= 0.0f) {
    // Not within the curved quadrant of this corner, therefore "inside"
    // relative to this one corner.
    return true;
  }

  // Dividing the quadrant_relative point by the radii gives a corresponding
  // location within a unit circle which can be more easily tested for
  // containment. We can use x^2 + y^2 and compare it against the radius
  // squared (1.0) to avoid the sqrt.
  Point quadrant_unit_circle_point = quadrant_relative / radii;
  return quadrant_unit_circle_point.GetLengthSquared() <= 1.0;
}

// The sign of the direction that points move as they approach the indicated
// corner from within the rectangle.
static constexpr Point kUpperLeftDirection(-1.0f, -1.0f);
static constexpr Point kUpperRightDirection(1.0f, -1.0f);
static constexpr Point kLowerLeftDirection(-1.0f, 1.0f);
static constexpr Point kLowerRightDirection(1.0f, 1.0f);

[[nodiscard]] bool RoundRect::Contains(const Point& p) const {
  if (!bounds_.Contains(p)) {
    return false;
  }
  if (!CornerContains(p, bounds_.GetLeftTop(), kUpperLeftDirection,
                      radii_.top_left) ||
      !CornerContains(p, bounds_.GetRightTop(), kUpperRightDirection,
                      radii_.top_right) ||
      !CornerContains(p, bounds_.GetLeftBottom(), kLowerLeftDirection,
                      radii_.bottom_left) ||
      !CornerContains(p, bounds_.GetRightBottom(), kLowerRightDirection,
                      radii_.bottom_right)) {
    return false;
  }
  return true;
}

}  // namespace impeller
