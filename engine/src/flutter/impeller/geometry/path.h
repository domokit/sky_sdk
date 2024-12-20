// Copyright 2013 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

#ifndef FLUTTER_IMPELLER_GEOMETRY_PATH_H_
#define FLUTTER_IMPELLER_GEOMETRY_PATH_H_

#include <functional>
#include <memory>
#include <optional>
#include <tuple>
#include <vector>

#include "impeller/geometry/path_component.h"
#include "impeller/geometry/rect.h"

namespace impeller {

enum class Cap {
  kButt,
  kRound,
  kSquare,
};

enum class Join {
  kMiter,
  kRound,
  kBevel,
};

enum class FillType {
  kNonZero,  // The default winding order.
  kOdd,
};

enum class Convexity {
  kUnknown,
  kConvex,
};

//------------------------------------------------------------------------------
/// @brief      Paths are lightweight objects that describe a collection of
///             linear, quadratic, or cubic segments. These segments may be
///             broken up by move commands, which are effectively linear
///             commands that pick up the pen rather than continuing to draw.
///
///             All shapes supported by Impeller are paths either directly or
///             via approximation (in the case of circles).
///
///             Paths are externally immutable once created, Creating paths must
///             be done using a path builder.
///
class Path {
 public:
  enum class ComponentType {
    kLinear,
    kQuadratic,
    kCubic,
    kContour,
  };

  static constexpr size_t VerbToOffset(Path::ComponentType verb) {
    switch (verb) {
      case Path::ComponentType::kLinear:
        return 2u;
      case Path::ComponentType::kQuadratic:
        return 3u;
      case Path::ComponentType::kCubic:
        return 4u;
      case Path::ComponentType::kContour:
        return 2u;
        break;
    }
    FML_UNREACHABLE();
  }

  struct PolylineContour {
    struct Component {
      size_t component_start_index;
      /// Denotes whether this component is a curve.
      ///
      /// This is set to true when this component is generated from
      /// QuadraticComponent or CubicPathComponent.
      bool is_curve;
    };
    /// Index that denotes the first point of this contour.
    size_t start_index;

    /// Denotes whether the last point of this contour is connected to the first
    /// point of this contour or not.
    bool is_closed;

    /// The direction of the contour's start cap.
    Vector2 start_direction;
    /// The direction of the contour's end cap.
    Vector2 end_direction;

    /// Distinct components in this contour.
    ///
    /// If this contour is generated from multiple path components, each
    /// path component forms a component in this vector.
    std::vector<Component> components;
  };

  /// One or more contours represented as a series of points and indices in
  /// the point vector representing the start of a new contour.
  ///
  /// Polylines are ephemeral and meant to be used by the tessellator. They do
  /// not allocate their own point vectors to allow for optimizations around
  /// allocation and reuse of arenas.
  struct Polyline {
    /// The signature of a method called when it is safe to reclaim the point
    /// buffer provided to the constructor of this object.
    using PointBufferPtr = std::unique_ptr<std::vector<Point>>;
    using ReclaimPointBufferCallback = std::function<void(PointBufferPtr)>;

    /// The buffer will be cleared and returned at the destruction of this
    /// polyline.
    Polyline(PointBufferPtr point_buffer, ReclaimPointBufferCallback reclaim);

    Polyline(Polyline&& other);
    ~Polyline();

    /// Points in the polyline, which may represent multiple contours specified
    /// by indices in |contours|.
    PointBufferPtr points;

    Point& GetPoint(size_t index) const { return (*points)[index]; }

    /// Contours are disconnected pieces of a polyline, such as when a MoveTo
    /// was issued on a PathBuilder.
    std::vector<PolylineContour> contours;

    /// Convenience method to compute the start (inclusive) and end (exclusive)
    /// point of the given contour index.
    ///
    /// The contour_index parameter is clamped to contours.size().
    std::tuple<size_t, size_t> GetContourPointBounds(
        size_t contour_index) const;

   private:
    ReclaimPointBufferCallback reclaim_points_;
  };

  Path();

  ~Path();

  size_t GetComponentCount(std::optional<ComponentType> type = {}) const;

  FillType GetFillType() const;

  bool IsConvex() const;

  bool IsEmpty() const;

  /// @brief Whether the line contains a single contour.
  bool IsSingleContour() const;

  bool GetLinearComponentAtIndex(size_t index,
                                 LinearPathComponent& linear) const;

  bool GetQuadraticComponentAtIndex(size_t index,
                                    QuadraticPathComponent& quadratic) const;

  bool GetCubicComponentAtIndex(size_t index, CubicPathComponent& cubic) const;

  bool GetContourComponentAtIndex(size_t index,
                                  ContourComponent& contour) const;

  /// Callers must provide the scale factor for how this path will be
  /// transformed.
  ///
  /// It is suitable to use the max basis length of the matrix used to transform
  /// the path. If the provided scale is 0, curves will revert to straight
  /// lines.
  Polyline CreatePolyline(
      Scalar scale,
      Polyline::PointBufferPtr point_buffer =
          std::make_unique<std::vector<Point>>(),
      Polyline::ReclaimPointBufferCallback reclaim = nullptr) const;

  void EndContour(
      size_t storage_offset,
      Polyline& polyline,
      size_t component_index,
      std::vector<PolylineContour::Component>& poly_components) const;

  std::optional<Rect> GetBoundingBox() const;

  std::optional<Rect> GetTransformedBoundingBox(const Matrix& transform) const;

  /// Generate a polyline into the temporary storage held by the [writer].
  ///
  /// It is suitable to use the max basis length of the matrix used to transform
  /// the path. If the provided scale is 0, curves will revert to straight
  /// lines.
  void WritePolyline(Scalar scale, VertexWriter& writer) const;

  /// Determine required storage for points and number of contours.
  std::pair<size_t, size_t> CountStorage(Scalar scale) const;

 private:
  friend class PathBuilder;

  // All of the data for the path is stored in this structure which is
  // held by a shared_ptr. Since they all share the structure, the
  // copy constructor for Path is very cheap and we don't need to deal
  // with shared pointers for Path fields and method arguments.
  //
  // PathBuilder also uses this structure to accumulate the path data
  // but the Path constructor used in |TakePath()| will clone the
  // structure to prevent sharing and future modifications within the
  // builder from affecting the existing taken paths.
  struct Data {
    Data() = default;

    Data(Data&& other) = default;

    Data(const Data& other) = default;

    ~Data() = default;

    FillType fill = FillType::kNonZero;
    Convexity convexity = Convexity::kUnknown;
    bool single_countour = true;
    std::optional<Rect> bounds;
    std::vector<Point> points;
    std::vector<ComponentType> components;
  };

  explicit Path(Data data);

  std::shared_ptr<const Data> data_;
};

static_assert(sizeof(Path) == sizeof(std::shared_ptr<struct Anonymous>));

}  // namespace impeller

#endif  // FLUTTER_IMPELLER_GEOMETRY_PATH_H_
