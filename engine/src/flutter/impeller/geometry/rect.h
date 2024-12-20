// Copyright 2013 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

#ifndef FLUTTER_IMPELLER_GEOMETRY_RECT_H_
#define FLUTTER_IMPELLER_GEOMETRY_RECT_H_

#include <array>
#include <optional>
#include <ostream>
#include <vector>

#include "fml/logging.h"
#include "impeller/geometry/matrix.h"
#include "impeller/geometry/point.h"
#include "impeller/geometry/saturated_math.h"
#include "impeller/geometry/scalar.h"
#include "impeller/geometry/size.h"

namespace impeller {

#define ONLY_ON_FLOAT_M(Modifiers, Return) \
  template <typename U = T>                \
  Modifiers std::enable_if_t<std::is_floating_point_v<U>, Return>
#define ONLY_ON_FLOAT(Return) DL_ONLY_ON_FLOAT_M(, Return)

/// Templated struct for holding an axis-aligned rectangle.
///
/// Rectangles are defined as 4 axis-aligned edges that might contain
/// space. They can be viewed as 2 X coordinates that define the
/// left and right edges and 2 Y coordinates that define the top and
/// bottom edges; or they can be viewed as an origin and horizontal
/// and vertical dimensions (width and height).
///
/// When the left and right edges are equal or reversed (right <= left)
/// or the top and bottom edges are equal or reversed (bottom <= top),
/// the rectangle is considered empty. Considering the rectangle in XYWH
/// form, the width and/or the height would be negative or zero. Such
/// reversed/empty rectangles contain no space and act as such in the
/// methods that operate on them (Intersection, Union, IntersectsWithRect,
/// Contains, Cutout, etc.)
///
/// Rectangles cannot be modified by any method and a new value can only
/// be stored into an existing rect using assignment. This keeps the API
/// clean compared to implementations that might have similar methods
/// that produce the answer in place, or construct a new object with
/// the answer, or place the result in an indicated result object.
///
/// Methods that might fail to produce an answer will use |std::optional|
/// to indicate that success or failure (see |Intersection| and |CutOut|).
/// For convenience, |Intersection| and |Union| both have overloaded
/// variants that take |std::optional| arguments and treat them as if
/// the argument was an empty rect to allow chaining multiple such methods
/// and only needing to check the optional condition of the final result.
/// The primary methods also provide |...OrEmpty| overloaded variants that
/// translate an empty optional answer into a simple empty rectangle of the
/// same type.
///
/// Rounding instance methods are not provided as the return value might
/// be wanted as another floating point rectangle or sometimes as an integer
/// rectangle. Instead a |RoundOut| factory, defined only for floating point
/// input rectangles, is provided to provide control over the result type.
///
/// NaN and Infinity values
///
/// Constructing an LTRB rectangle using Infinity values should work as
/// expected with either 0 or +Infinity returned as dimensions depending on
/// which side the Infinity values are on and the sign.
///
/// Constructing an XYWH rectangle using Infinity values will usually
/// not work if the math requires the object to compute a right or bottom
/// edge from ([xy] -Infinity + [wh] +Infinity). Other combinations might
/// work.
///
/// The special factory |MakeMaximum| is provided to construct a rectangle
/// of the indicated coordinate type that covers all finite coordinates.
/// It does not use infinity values, but rather the largest finite values
/// to avoid math that might produce a NaN value from various getters.
///
/// Any rectangle that is constructed with, or computed to have a NaN value
/// will be considered the same as any empty rectangle.
///
/// Empty Rectangle canonical results summary:
///
/// Union will ignore any empty rects and return the other rect
/// Intersection will return nullopt if either rect is empty
/// IntersectsWithRect will return false if either rect is empty
/// Cutout will return the source rect if the argument is empty
/// Cutout will return nullopt if the source rectangle is empty
/// Contains(Point) will return false if the source rectangle is empty
/// Contains(Rect) will return false if the source rectangle is empty
/// Contains(Rect) will otherwise return true if the argument is empty
/// Specifically, EmptyRect.Contains(EmptyRect) returns false
///
/// ---------------
/// Special notes on problems using the XYWH form of specifying rectangles:
///
/// It is possible to have integer rectangles whose dimensions exceed
/// the maximum number that their coordinates can represent since
/// (MAX_INT - MIN_INT) overflows the representable positive numbers.
/// Floating point rectangles technically have a similar issue in that
/// overflow can occur, but it will be automatically converted into
/// either an infinity, or a finite-overflow value and still be
/// representable, just with little to no precision.
///
/// Secondly, specifying a rectangle using XYWH leads to cases where the
/// math for (x+w) and/or (y+h) are also beyond the maximum representable
/// coordinates. For N-bit integer rectangles declared as XYWH, the
/// maximum right coordinate will require N+1 signed bits which cannot be
/// stored in storage that uses N-bit integers.
///
/// Saturated math is used when constructing a rectangle from XYWH values
/// and when returning the dimensions of the rectangle. Constructing an
/// integer rectangle from values such that xy + wh is beyond the range
/// of the integer type will place the right or bottom edges at the maximum
/// value for the integer type. Similarly, constructing an integer rectangle
/// such that the distance from the left to the right (or top to bottom) is
/// greater than the range of the integer type will simply return the
/// maximum integer value as the dimension. Floating point rectangles are
/// naturally saturated by the rules of IEEE arithmetic.
template <class T>
struct TRect {
 private:
  using Type = T;

 public:
  constexpr TRect() : left_(0), top_(0), right_(0), bottom_(0) {}

  constexpr static TRect MakeLTRB(Type left,
                                  Type top,
                                  Type right,
                                  Type bottom) {
    return TRect(left, top, right, bottom);
  }

  constexpr static TRect MakeXYWH(Type x, Type y, Type width, Type height) {
    return TRect(x, y, saturated::Add(x, width), saturated::Add(y, height));
  }

  constexpr static TRect MakeWH(Type width, Type height) {
    return TRect(0, 0, width, height);
  }

  constexpr static TRect MakeOriginSize(const TPoint<Type>& origin,
                                        const TSize<Type>& size) {
    return MakeXYWH(origin.x, origin.y, size.width, size.height);
  }

  template <class U>
  constexpr static TRect MakeSize(const TSize<U>& size) {
    return TRect(0.0, 0.0, size.width, size.height);
  }

  /// Construct a floating point rect |Rect| from another Rect of a
  /// potentially different storage type (eg. |IRect|).
  template <class U, class FT = T>
  constexpr static std::enable_if_t<std::is_floating_point_v<FT>, TRect> Make(
      const TRect<U>& rect) {
    return MakeLTRB(
        static_cast<FT>(rect.GetLeft()), static_cast<FT>(rect.GetTop()),
        static_cast<FT>(rect.GetRight()), static_cast<FT>(rect.GetBottom()));
  }

  template <typename U>
  constexpr static std::optional<TRect> MakePointBounds(const U& value) {
    return MakePointBounds(value.begin(), value.end());
  }

  template <typename PointIter>
  constexpr static std::optional<TRect> MakePointBounds(const PointIter first,
                                                        const PointIter last) {
    if (first == last) {
      return std::nullopt;
    }
    auto left = first->x;
    auto top = first->y;
    auto right = first->x;
    auto bottom = first->y;
    for (auto it = first + 1; it < last; ++it) {
      left = std::min(left, it->x);
      top = std::min(top, it->y);
      right = std::max(right, it->x);
      bottom = std::max(bottom, it->y);
    }
    return TRect::MakeLTRB(left, top, right, bottom);
  }

  [[nodiscard]] constexpr static TRect MakeMaximum() {
    return TRect::MakeLTRB(std::numeric_limits<Type>::lowest(),
                           std::numeric_limits<Type>::lowest(),
                           std::numeric_limits<Type>::max(),
                           std::numeric_limits<Type>::max());
  }

  [[nodiscard]] constexpr bool operator==(const TRect& r) const {
    return left_ == r.left_ &&    //
           top_ == r.top_ &&      //
           right_ == r.right_ &&  //
           bottom_ == r.bottom_;
  }

  [[nodiscard]] constexpr bool operator!=(const TRect& r) const {
    return !(*this == r);
  }

  [[nodiscard]] constexpr TRect Scale(Type scale) const {
    return TRect(left_ * scale,   //
                 top_ * scale,    //
                 right_ * scale,  //
                 bottom_ * scale);
  }

  [[nodiscard]] constexpr TRect Scale(Type scale_x, Type scale_y) const {
    return TRect(left_ * scale_x,   //
                 top_ * scale_y,    //
                 right_ * scale_x,  //
                 bottom_ * scale_y);
  }

  [[nodiscard]] constexpr TRect Scale(TPoint<T> scale) const {
    return Scale(scale.x, scale.y);
  }

  [[nodiscard]] constexpr TRect Scale(TSize<T> scale) const {
    return Scale(scale.width, scale.height);
  }

  /// @brief  Returns true iff the provided point |p| is inside the
  ///         half-open interior of this rectangle.
  ///
  ///         For purposes of containment, a rectangle contains points
  ///         along the top and left edges but not points along the
  ///         right and bottom edges so that a point is only ever
  ///         considered inside one of two abutting rectangles.
  [[nodiscard]] constexpr bool Contains(const TPoint<Type>& p) const {
    return !this->IsEmpty() &&  //
           p.x >= left_ &&      //
           p.y >= top_ &&       //
           p.x < right_ &&      //
           p.y < bottom_;
  }

  /// @brief  Returns true iff the provided point |p| is inside the
  ///         closed-range interior of this rectangle.
  ///
  ///         Unlike the regular |Contains(TPoint)| method, this method
  ///         considers all points along the boundary of the rectangle
  ///         to be contained within the rectangle - useful for testing
  ///         if vertices that define a filled shape would carry the
  ///         interior of that shape outside the bounds of the rectangle.
  ///         Since both geometries are defining half-open spaces, their
  ///         defining geometry needs to consider their boundaries to
  ///         be equivalent with respect to interior and exterior.
  [[nodiscard]] constexpr bool ContainsInclusive(const TPoint<Type>& p) const {
    return !this->IsEmpty() &&  //
           p.x >= left_ &&      //
           p.y >= top_ &&       //
           p.x <= right_ &&     //
           p.y <= bottom_;
  }

  /// @brief  Returns true iff this rectangle is not empty and it also
  ///         contains every point considered inside the provided
  ///         rectangle |o| (as determined by |Contains(TPoint)|).
  ///
  ///         This is similar to a definition where the result is true iff
  ///         the union of the two rectangles is equal to this rectangle,
  ///         ignoring precision issues with performing those operations
  ///         and assuming that empty rectangles are never equal.
  ///
  ///         An empty rectangle can contain no other rectangle.
  ///
  ///         An empty rectangle is, however, contained within any
  ///         other non-empy rectangle as the set of points it contains
  ///         is an empty set and so there are no points to fail the
  ///         containment criteria.
  [[nodiscard]] constexpr bool Contains(const TRect& o) const {
    return !this->IsEmpty() &&                     //
           (o.IsEmpty() || (o.left_ >= left_ &&    //
                            o.top_ >= top_ &&      //
                            o.right_ <= right_ &&  //
                            o.bottom_ <= bottom_));
  }

  /// @brief  Returns true if all of the fields of this floating point
  ///         rectangle are finite.
  ///
  ///         Note that the results of |GetWidth()| and |GetHeight()| may
  ///         still be infinite due to overflow even if the fields themselves
  ///         are finite.
  ONLY_ON_FLOAT_M([[nodiscard]] constexpr, bool)
  IsFinite() const {
    return std::isfinite(left_) &&   //
           std::isfinite(top_) &&    //
           std::isfinite(right_) &&  //
           std::isfinite(bottom_);
  }

  /// @brief  Returns true if either of the width or height are 0, negative,
  ///         or NaN.
  [[nodiscard]] constexpr bool IsEmpty() const {
    // Computing the non-empty condition and negating the result causes any
    // NaN value to return true - i.e. is considered empty.
    return !(left_ < right_ && top_ < bottom_);
  }

  /// @brief  Returns true if width and height are equal and neither is NaN.
  [[nodiscard]] constexpr bool IsSquare() const {
    // empty rectangles can technically be "square", but would be
    // misleading to most callers. Using |IsEmpty| also prevents
    // "non-empty and non-overflowing" computations from happening
    // to be equal to "empty and overflowing" results.
    // (Consider LTRB(10, 15, MAX-2, MIN+2) which is empty, but both
    //  w/h subtractions equal "5").
    return !IsEmpty() && (right_ - left_) == (bottom_ - top_);
  }

  [[nodiscard]] constexpr bool IsMaximum() const {
    return *this == MakeMaximum();
  }

  /// @brief Returns the upper left corner of the rectangle as specified
  ///        by the left/top or x/y values when it was constructed.
  [[nodiscard]] constexpr TPoint<Type> GetOrigin() const {
    return {left_, top_};
  }

  /// @brief Returns the size of the rectangle which may be negative in
  ///        either width or height and may have been clipped to the
  ///        maximum integer values for integer rects whose size overflows.
  [[nodiscard]] constexpr TSize<Type> GetSize() const {
    return {GetWidth(), GetHeight()};
  }

  /// @brief Returns the X coordinate of the upper left corner, equivalent
  ///        to |GetOrigin().x|
  [[nodiscard]] constexpr Type GetX() const { return left_; }

  /// @brief Returns the Y coordinate of the upper left corner, equivalent
  ///        to |GetOrigin().y|
  [[nodiscard]] constexpr Type GetY() const { return top_; }

  /// @brief Returns the width of the rectangle, equivalent to
  ///        |GetSize().width|
  [[nodiscard]] constexpr Type GetWidth() const {
    return saturated::Sub(right_, left_);
  }

  /// @brief Returns the height of the rectangle, equivalent to
  ///        |GetSize().height|
  [[nodiscard]] constexpr Type GetHeight() const {
    return saturated::Sub(bottom_, top_);
  }

  [[nodiscard]] constexpr auto GetLeft() const { return left_; }

  [[nodiscard]] constexpr auto GetTop() const { return top_; }

  [[nodiscard]] constexpr auto GetRight() const { return right_; }

  [[nodiscard]] constexpr auto GetBottom() const { return bottom_; }

  [[nodiscard]] constexpr TPoint<T> GetLeftTop() const {  //
    return {left_, top_};
  }

  [[nodiscard]] constexpr TPoint<T> GetRightTop() const {
    return {right_, top_};
  }

  [[nodiscard]] constexpr TPoint<T> GetLeftBottom() const {
    return {left_, bottom_};
  }

  [[nodiscard]] constexpr TPoint<T> GetRightBottom() const {
    return {right_, bottom_};
  }

  /// @brief  Get the area of the rectangle, equivalent to |GetSize().Area()|
  [[nodiscard]] constexpr T Area() const {
    // TODO(141710): Use saturated math to avoid overflow.
    return IsEmpty() ? 0 : (right_ - left_) * (bottom_ - top_);
  }

  /// @brief  Get the center point as a |Point|.
  [[nodiscard]] constexpr Point GetCenter() const {
    return {saturated::AverageScalar(left_, right_),
            saturated::AverageScalar(top_, bottom_)};
  }

  [[nodiscard]] constexpr std::array<T, 4> GetLTRB() const {
    return {left_, top_, right_, bottom_};
  }

  /// @brief  Get the x, y coordinates of the origin and the width and
  ///         height of the rectangle in an array.
  [[nodiscard]] constexpr std::array<T, 4> GetXYWH() const {
    return {left_, top_, GetWidth(), GetHeight()};
  }

  /// @brief  Get a version of this rectangle that has a non-negative size.
  [[nodiscard]] constexpr TRect GetPositive() const {
    if (!IsEmpty()) {
      return *this;
    }
    return {
        std::min(left_, right_),
        std::min(top_, bottom_),
        std::max(left_, right_),
        std::max(top_, bottom_),
    };
  }

  /// @brief  Get the points that represent the 4 corners of this rectangle
  ///         in a Z order that is compatible with triangle strips or a set
  ///         of all zero points if the rectangle is empty.
  ///         The order is: Top left, top right, bottom left, bottom right.
  [[nodiscard]] constexpr std::array<TPoint<T>, 4> GetPoints() const {
    if (IsEmpty()) {
      return {};
    }
    return {
        TPoint{left_, top_},
        TPoint{right_, top_},
        TPoint{left_, bottom_},
        TPoint{right_, bottom_},
    };
  }

  [[nodiscard]] constexpr std::array<TPoint<T>, 4> GetTransformedPoints(
      const Matrix& transform) const {
    auto points = GetPoints();
    for (size_t i = 0; i < points.size(); i++) {
      points[i] = transform * points[i];
    }
    return points;
  }

  /// @brief  Creates a new bounding box that contains this transformed
  ///         rectangle, clipped against the near clipping plane if
  ///         necessary.
  [[nodiscard]] constexpr TRect TransformAndClipBounds(
      const Matrix& transform) const {
    if (!transform.HasPerspective2D()) {
      return TransformBounds(transform);
    }

    if (IsEmpty()) {
      return {};
    }

    auto ul = transform.TransformHomogenous({left_, top_});
    auto ur = transform.TransformHomogenous({right_, top_});
    auto ll = transform.TransformHomogenous({left_, bottom_});
    auto lr = transform.TransformHomogenous({right_, bottom_});

    // It can probably be proven that we only ever have 5 points at most
    // which happens when only 1 corner is clipped and we get 2 points
    // in return for it as we interpolate against its neighbors.
    Point points[8];
    int index = 0;

    // Process (clip and interpolate) each point against its 2 neighbors:
    //                                 left, pt, right
    index = ClipAndInsert(points, index, ll, ul, ur);
    index = ClipAndInsert(points, index, ul, ur, lr);
    index = ClipAndInsert(points, index, ur, lr, ll);
    index = ClipAndInsert(points, index, lr, ll, ul);

    auto bounds = TRect::MakePointBounds(points, points + index);
    return bounds.value_or(TRect{});
  }

  /// @brief  Creates a new bounding box that contains this transformed
  ///         rectangle.
  [[nodiscard]] constexpr TRect TransformBounds(const Matrix& transform) const {
    if (IsEmpty()) {
      return {};
    }
    auto points = GetTransformedPoints(transform);
    auto bounds = TRect::MakePointBounds(points.begin(), points.end());
    if (bounds.has_value()) {
      return bounds.value();
    }
    FML_UNREACHABLE();
  }

  /// @brief  Constructs a Matrix that will map all points in the coordinate
  ///         space of the rectangle into a new normalized coordinate space
  ///         where the upper left corner of the rectangle maps to (0, 0)
  ///         and the lower right corner of the rectangle maps to (1, 1).
  ///
  ///         Empty and non-finite rectangles will return a zero-scaling
  ///         transform that maps all points to (0, 0).
  [[nodiscard]] constexpr Matrix GetNormalizingTransform() const {
    if (!IsEmpty()) {
      Scalar sx = 1.0 / GetWidth();
      Scalar sy = 1.0 / GetHeight();
      Scalar tx = left_ * -sx;
      Scalar ty = top_ * -sy;

      // Exclude NaN and infinities and either scale underflowing to zero
      if (sx != 0.0 && sy != 0.0 && 0.0 * sx * sy * tx * ty == 0.0) {
        // clang-format off
        return Matrix(  sx, 0.0f, 0.0f, 0.0f,
                      0.0f,   sy, 0.0f, 0.0f,
                      0.0f, 0.0f, 1.0f, 0.0f,
                        tx,   ty, 0.0f, 1.0f);
        // clang-format on
      }
    }

    // Map all coordinates to the origin.
    return Matrix::MakeScale({0.0f, 0.0f, 1.0f});
  }

  [[nodiscard]] constexpr TRect Union(const TRect& o) const {
    if (IsEmpty()) {
      return o;
    }
    if (o.IsEmpty()) {
      return *this;
    }
    return {
        std::min(left_, o.left_),
        std::min(top_, o.top_),
        std::max(right_, o.right_),
        std::max(bottom_, o.bottom_),
    };
  }

  [[nodiscard]] constexpr std::optional<TRect> Intersection(
      const TRect& o) const {
    if (IntersectsWithRect(o)) {
      return TRect{
          std::max(left_, o.left_),
          std::max(top_, o.top_),
          std::min(right_, o.right_),
          std::min(bottom_, o.bottom_),
      };
    } else {
      return std::nullopt;
    }
  }

  [[nodiscard]] constexpr TRect IntersectionOrEmpty(const TRect& o) const {
    return Intersection(o).value_or(TRect());
  }

  [[nodiscard]] constexpr bool IntersectsWithRect(const TRect& o) const {
    return !IsEmpty() &&        //
           !o.IsEmpty() &&      //
           left_ < o.right_ &&  //
           top_ < o.bottom_ &&  //
           right_ > o.left_ &&  //
           bottom_ > o.top_;
  }

  /// @brief Returns the new boundary rectangle that would result from this
  ///        rectangle being cut out by the specified rectangle.
  [[nodiscard]] constexpr std::optional<TRect<T>> Cutout(const TRect& o) const {
    if (IsEmpty()) {
      // This test isn't just a short-circuit, it also prevents the concise
      // math below from returning the wrong answer on empty rects.
      // Once we know that this rectangle is not empty, the math below can
      // only succeed in computing a value if o is also non-empty and non-nan.
      // Otherwise, the method returns *this by default.
      return std::nullopt;
    }

    const auto& [a_left, a_top, a_right, a_bottom] = GetLTRB();  // Source rect.
    const auto& [b_left, b_top, b_right, b_bottom] = o.GetLTRB();  // Cutout.
    if (b_left <= a_left && b_right >= a_right) {
      if (b_top <= a_top && b_bottom >= a_bottom) {
        // Full cutout.
        return std::nullopt;
      }
      if (b_top <= a_top && b_bottom > a_top) {
        // Cuts off the top.
        return TRect::MakeLTRB(a_left, b_bottom, a_right, a_bottom);
      }
      if (b_bottom >= a_bottom && b_top < a_bottom) {
        // Cuts off the bottom.
        return TRect::MakeLTRB(a_left, a_top, a_right, b_top);
      }
    }
    if (b_top <= a_top && b_bottom >= a_bottom) {
      if (b_left <= a_left && b_right > a_left) {
        // Cuts off the left.
        return TRect::MakeLTRB(b_right, a_top, a_right, a_bottom);
      }
      if (b_right >= a_right && b_left < a_right) {
        // Cuts off the right.
        return TRect::MakeLTRB(a_left, a_top, b_left, a_bottom);
      }
    }

    return *this;
  }

  [[nodiscard]] constexpr TRect CutoutOrEmpty(const TRect& o) const {
    return Cutout(o).value_or(TRect());
  }

  /// @brief  Returns a new rectangle translated by the given offset.
  [[nodiscard]] constexpr TRect<T> Shift(T dx, T dy) const {
    return {
        saturated::Add(left_, dx),    //
        saturated::Add(top_, dy),     //
        saturated::Add(right_, dx),   //
        saturated::Add(bottom_, dy),  //
    };
  }

  /// @brief  Returns a new rectangle translated by the given offset.
  [[nodiscard]] constexpr TRect<T> Shift(TPoint<T> offset) const {
    return Shift(offset.x, offset.y);
  }

  /// @brief  Returns a rectangle with expanded edges. Negative expansion
  ///         results in shrinking.
  [[nodiscard]] constexpr TRect<T> Expand(T left,
                                          T top,
                                          T right,
                                          T bottom) const {
    return {
        saturated::Sub(left_, left),      //
        saturated::Sub(top_, top),        //
        saturated::Add(right_, right),    //
        saturated::Add(bottom_, bottom),  //
    };
  }

  /// @brief  Returns a rectangle with expanded edges in all directions.
  ///         Negative expansion results in shrinking.
  [[nodiscard]] constexpr TRect<T> Expand(T amount) const {
    return {
        saturated::Sub(left_, amount),    //
        saturated::Sub(top_, amount),     //
        saturated::Add(right_, amount),   //
        saturated::Add(bottom_, amount),  //
    };
  }

  /// @brief  Returns a rectangle with expanded edges in all directions.
  ///         Negative expansion results in shrinking.
  [[nodiscard]] constexpr TRect<T> Expand(T horizontal_amount,
                                          T vertical_amount) const {
    return {
        saturated::Sub(left_, horizontal_amount),   //
        saturated::Sub(top_, vertical_amount),      //
        saturated::Add(right_, horizontal_amount),  //
        saturated::Add(bottom_, vertical_amount),   //
    };
  }

  /// @brief  Returns a rectangle with expanded edges in all directions.
  ///         Negative expansion results in shrinking.
  [[nodiscard]] constexpr TRect<T> Expand(TPoint<T> amount) const {
    return Expand(amount.x, amount.y);
  }

  /// @brief  Returns a rectangle with expanded edges in all directions.
  ///         Negative expansion results in shrinking.
  [[nodiscard]] constexpr TRect<T> Expand(TSize<T> amount) const {
    return Expand(amount.width, amount.height);
  }

  /// @brief  Returns a new rectangle that represents the projection of the
  ///         source rectangle onto this rectangle. In other words, the source
  ///         rectangle is redefined in terms of the coordinate space of this
  ///         rectangle.
  [[nodiscard]] constexpr TRect<T> Project(TRect<T> source) const {
    if (IsEmpty()) {
      return {};
    }
    return source.Shift(-left_, -top_)
        .Scale(1.0 / static_cast<Scalar>(GetWidth()),
               1.0 / static_cast<Scalar>(GetHeight()));
  }

  ONLY_ON_FLOAT_M([[nodiscard]] constexpr static, TRect)
  RoundOut(const TRect<U>& r) {
    return TRect::MakeLTRB(saturated::Cast<U, Type>(floor(r.GetLeft())),
                           saturated::Cast<U, Type>(floor(r.GetTop())),
                           saturated::Cast<U, Type>(ceil(r.GetRight())),
                           saturated::Cast<U, Type>(ceil(r.GetBottom())));
  }

  ONLY_ON_FLOAT_M([[nodiscard]] constexpr static, TRect)
  RoundIn(const TRect<U>& r) {
    return TRect::MakeLTRB(saturated::Cast<U, Type>(ceil(r.GetLeft())),
                           saturated::Cast<U, Type>(ceil(r.GetTop())),
                           saturated::Cast<U, Type>(floor(r.GetRight())),
                           saturated::Cast<U, Type>(floor(r.GetBottom())));
  }

  ONLY_ON_FLOAT_M([[nodiscard]] constexpr static, TRect)
  Round(const TRect<U>& r) {
    return TRect::MakeLTRB(saturated::Cast<U, Type>(round(r.GetLeft())),
                           saturated::Cast<U, Type>(round(r.GetTop())),
                           saturated::Cast<U, Type>(round(r.GetRight())),
                           saturated::Cast<U, Type>(round(r.GetBottom())));
  }

  [[nodiscard]] constexpr static std::optional<TRect> Union(
      const TRect& a,
      const std::optional<TRect> b) {
    return b.has_value() ? a.Union(b.value()) : a;
  }

  [[nodiscard]] constexpr static std::optional<TRect> Union(
      const std::optional<TRect> a,
      const TRect& b) {
    return a.has_value() ? a->Union(b) : b;
  }

  [[nodiscard]] constexpr static std::optional<TRect> Union(
      const std::optional<TRect> a,
      const std::optional<TRect> b) {
    return a.has_value() ? Union(a.value(), b) : b;
  }

  [[nodiscard]] constexpr static std::optional<TRect> Intersection(
      const TRect& a,
      const std::optional<TRect> b) {
    return b.has_value() ? a.Intersection(b.value()) : a;
  }

  [[nodiscard]] constexpr static std::optional<TRect> Intersection(
      const std::optional<TRect> a,
      const TRect& b) {
    return a.has_value() ? a->Intersection(b) : b;
  }

  [[nodiscard]] constexpr static std::optional<TRect> Intersection(
      const std::optional<TRect> a,
      const std::optional<TRect> b) {
    return a.has_value() ? Intersection(a.value(), b) : b;
  }

 private:
  constexpr TRect(Type left, Type top, Type right, Type bottom)
      : left_(left), top_(top), right_(right), bottom_(bottom) {}

  Type left_;
  Type top_;
  Type right_;
  Type bottom_;

  static constexpr Scalar kMinimumHomogenous = 1.0f / (1 << 14);

  // Clip p against the near clipping plane (W = kMinimumHomogenous)
  // and interpolate a crossing point against the nearby neighbors
  // left and right if p is clipped and either of them is not.
  // This method can produce 0, 1, or 2 points per call depending on
  // how many of the points are clipped.
  // 0 - all points are clipped
  // 1 - p is unclipped OR
  //     p is clipped and exactly one of the neighbors is not
  // 2 - p is clipped and both neighbors are not
  static constexpr int ClipAndInsert(Point clipped[],
                                     int index,
                                     const Vector3& left,
                                     const Vector3& p,
                                     const Vector3& right) {
    if (p.z >= kMinimumHomogenous) {
      clipped[index++] = {p.x / p.z, p.y / p.z};
    } else {
      index = InterpolateAndInsert(clipped, index, p, left);
      index = InterpolateAndInsert(clipped, index, p, right);
    }
    return index;
  }

  // Interpolate (a clipped) point p against one of its neighbors
  // and insert the point into the array where the line between them
  // veers from clipped space to unclipped, if such a point exists.
  static constexpr int InterpolateAndInsert(Point clipped[],
                                            int index,
                                            const Vector3& p,
                                            const Vector3& neighbor) {
    if (neighbor.z >= kMinimumHomogenous) {
      auto t = (kMinimumHomogenous - p.z) / (neighbor.z - p.z);
      clipped[index++] = {
          (t * p.x + (1.0f - t) * neighbor.x) / kMinimumHomogenous,
          (t * p.y + (1.0f - t) * neighbor.y) / kMinimumHomogenous,
      };
    }
    return index;
  }
};

using Rect = TRect<Scalar>;
using IRect32 = TRect<int32_t>;
using IRect64 = TRect<int64_t>;
using IRect = IRect64;

#undef ONLY_ON_FLOAT
#undef ONLY_ON_FLOAT_M

}  // namespace impeller

namespace std {

template <class T>
inline std::ostream& operator<<(std::ostream& out,
                                const impeller::TRect<T>& r) {
  out << "(" << r.GetLeftTop() << " => " << r.GetRightBottom() << ")";
  return out;
}

}  // namespace std

#endif  // FLUTTER_IMPELLER_GEOMETRY_RECT_H_
