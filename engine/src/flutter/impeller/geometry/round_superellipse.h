// Copyright 2013 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

#ifndef FLUTTER_IMPELLER_GEOMETRY_ROUND_SUPERELLIPSE_H_
#define FLUTTER_IMPELLER_GEOMETRY_ROUND_SUPERELLIPSE_H_

#include "flutter/impeller/geometry/point.h"
#include "flutter/impeller/geometry/rect.h"
#include "flutter/impeller/geometry/size.h"

namespace impeller {

struct RoundSuperellipse {
  RoundSuperellipse() = default;

  static RoundSuperellipse MakeRectRadius(const Rect& rect,
                                          Scalar corner_radius);

  constexpr const Rect& GetBounds() const { return bounds_; }
  constexpr float GetCornerRadius() const { return corner_radius_; }

  [[nodiscard]] constexpr bool IsFinite() const {
    return bounds_.IsFinite() && std::isfinite(corner_radius_);
  }

  [[nodiscard]] constexpr bool IsEmpty() const { return bounds_.IsEmpty(); }

  [[nodiscard]] constexpr bool IsRect() const {
    return !bounds_.IsEmpty() && ScalarNearlyEqual(corner_radius_, 0);
  }

  [[nodiscard]] constexpr bool IsCircle() const {
    return !bounds_.IsEmpty() &&
           ScalarNearlyEqual(corner_radius_, bounds_.GetWidth() * 0.5f) &&
           ScalarNearlyEqual(corner_radius_, bounds_.GetHeight() * 0.5f);
  }

  /// @brief  Returns a new round rectangle translated by the given offset.
  [[nodiscard]] constexpr RoundSuperellipse Shift(Scalar dx, Scalar dy) const {
    // Just in case, use the factory rather than the internal constructor
    // as shifting the rectangle may increase/decrease its bit precision
    // so we should re-validate the radii to the newly located rectangle.
    return MakeRectRadius(bounds_.Shift(dx, dy), corner_radius_);
  }

  [[nodiscard]] constexpr bool operator==(const RoundSuperellipse& rr) const {
    return bounds_ == rr.bounds_ && corner_radius_ == rr.corner_radius_;
  }

  [[nodiscard]] constexpr bool operator!=(const RoundSuperellipse& r) const {
    return !(*this == r);
  }

  /// @brief  A conservative inner rectangle that is fully contained in this
  /// shape.
  ///
  ///         This is useful for certain optimizations.
  [[nodiscard]] Rect EstimateInner() const;

 private:
  constexpr RoundSuperellipse(const Rect& bounds, float corner_radius)
      : bounds_(bounds), corner_radius_(corner_radius) {}

  const Rect bounds_;
  const float corner_radius_ = 0;
};

}  // namespace impeller

namespace std {

inline std::ostream& operator<<(std::ostream& out,
                                const impeller::RoundSuperellipse& rr) {
  out << "("                                        //
      << "rect: " << rr.GetBounds() << ", "         //
      << "corner_radius: " << rr.GetCornerRadius()  //
      << ")";
  return out;
}

}  // namespace std

#endif  // FLUTTER_IMPELLER_GEOMETRY_ROUND_SUPERELLIPSE_H_
