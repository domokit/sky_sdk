// Copyright 2014 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

/// @docImport 'package:flutter/rendering.dart';
library;

import 'package:flutter/foundation.dart';
import 'package:vector_math/vector_math_64.dart';

import 'basic_types.dart';

/// Utility functions for working with matrices.
abstract final class MatrixUtils {
  /// Returns the given [transform] matrix as an [Offset], if the matrix is
  /// nothing but a 2D translation.
  ///
  /// Otherwise, returns null.
  static Offset? getAsTranslation(Matrix4 transform) {
    // Values are stored in column-major order, so the appearance
    // of dx is transposed from the top-right to the bottom-left.
    if (transform.storage case [
      1.0,
      0.0,
      0.0,
      0.0,
      0.0,
      1.0,
      0.0,
      0.0,
      0.0,
      0.0,
      1.0,
      0.0,
      final double dx,
      final double dy,
      0.0,
      1.0,
    ]) {
      return Offset(dx, dy);
    }
    return null;
  }

  /// Returns the given [transform] matrix as a [double] describing a uniform
  /// scale, if the matrix is nothing but a symmetric 2D scale transform.
  ///
  /// Otherwise, returns null.
  static double? getAsScale(Matrix4 transform) {
    // Values are stored in column-major order
    // (but this symmetric matrix is unaffected).
    if (transform.storage case [
      final double diagonal1,
      0.0,
      0.0,
      0.0,
      0.0,
      final double diagonal2,
      0.0,
      0.0,
      0.0,
      0.0,
      1.0,
      0.0,
      0.0,
      0.0,
      0.0,
      1.0,
    ] when diagonal1 == diagonal2) {
      return diagonal1;
    }
    return null;
  }

  /// Returns true if the given matrices are exactly equal, and false
  /// otherwise. Null values are assumed to be the identity matrix.
  static bool matrixEquals(Matrix4? a, Matrix4? b) {
    if (identical(a, b)) {
      return true;
    }
    assert(a != null || b != null);
    if (a == null) {
      return isIdentity(b!);
    }
    if (b == null) {
      return isIdentity(a);
    }
    return a.storage[0] == b.storage[0] &&
        a.storage[1] == b.storage[1] &&
        a.storage[2] == b.storage[2] &&
        a.storage[3] == b.storage[3] &&
        a.storage[4] == b.storage[4] &&
        a.storage[5] == b.storage[5] &&
        a.storage[6] == b.storage[6] &&
        a.storage[7] == b.storage[7] &&
        a.storage[8] == b.storage[8] &&
        a.storage[9] == b.storage[9] &&
        a.storage[10] == b.storage[10] &&
        a.storage[11] == b.storage[11] &&
        a.storage[12] == b.storage[12] &&
        a.storage[13] == b.storage[13] &&
        a.storage[14] == b.storage[14] &&
        a.storage[15] == b.storage[15];
  }

  /// Whether the given matrix is the identity matrix.
  static bool isIdentity(Matrix4 a) {
    return a.storage[0] ==
            1.0 // col 1
            &&
        a.storage[1] == 0.0 &&
        a.storage[2] == 0.0 &&
        a.storage[3] == 0.0 &&
        a.storage[4] ==
            0.0 // col 2
            &&
        a.storage[5] == 1.0 &&
        a.storage[6] == 0.0 &&
        a.storage[7] == 0.0 &&
        a.storage[8] ==
            0.0 // col 3
            &&
        a.storage[9] == 0.0 &&
        a.storage[10] == 1.0 &&
        a.storage[11] == 0.0 &&
        a.storage[12] ==
            0.0 // col 4
            &&
        a.storage[13] == 0.0 &&
        a.storage[14] == 0.0 &&
        a.storage[15] == 1.0;
  }

  /// Applies the given matrix as a perspective transform to the given point.
  ///
  /// This function assumes the given point has a z-coordinate of 0.0. The
  /// z-coordinate of the result is ignored.
  ///
  /// While not common, this method may return (NaN, NaN), iff the given `point`
  /// results in a "point at infinity" in homogeneous coordinates after applying
  /// the `transform`. For example, a [RenderObject] may set its transform to
  /// the zero matrix to indicate its content is currently not visible. Trying
  /// to convert an `Offset` to its coordinate space always results in
  /// (NaN, NaN).
  static Offset transformPoint(Matrix4 transform, Offset point) {
    final Float64List storage = transform.storage;
    final double x = point.dx;
    final double y = point.dy;

    // Directly simulate the transform of the vector (x, y, 0, 1),
    // dropping the resulting Z coordinate, and normalizing only
    // if needed.

    final double rx = storage[0] * x + storage[4] * y + storage[12];
    final double ry = storage[1] * x + storage[5] * y + storage[13];
    final double rw = storage[3] * x + storage[7] * y + storage[15];
    if (rw == 1.0) {
      return Offset(rx, ry);
    } else {
      return Offset(rx / rw, ry / rw);
    }
  }

  /// Returns a rect that bounds the result of applying the given matrix as a
  /// perspective transform to the given rect.
  ///
  /// This version of the operation is slower than the regular transformRect
  /// method, but it avoids creating infinite values from large finite values
  /// if it can.
  static Rect _safeTransformRect(Matrix4 transform, Rect rect) {
    final Float64List storage = transform.storage;
    final bool isAffine = storage[3] == 0.0 && storage[7] == 0.0 && storage[15] == 1.0;

    _accumulate(storage, rect.left, rect.top, true, isAffine);
    _accumulate(storage, rect.right, rect.top, false, isAffine);
    _accumulate(storage, rect.left, rect.bottom, false, isAffine);
    _accumulate(storage, rect.right, rect.bottom, false, isAffine);

    return Rect.fromLTRB(_minMax[0], _minMax[1], _minMax[2], _minMax[3]);
  }

  static final Float64List _minMax = Float64List(4);
  static void _accumulate(Float64List m, double x, double y, bool first, bool isAffine) {
    final double w = isAffine ? 1.0 : 1.0 / (m[3] * x + m[7] * y + m[15]);
    final double tx = (m[0] * x + m[4] * y + m[12]) * w;
    final double ty = (m[1] * x + m[5] * y + m[13]) * w;
    if (first) {
      _minMax[0] = _minMax[2] = tx;
      _minMax[1] = _minMax[3] = ty;
    } else {
      if (tx < _minMax[0]) {
        _minMax[0] = tx;
      }
      if (ty < _minMax[1]) {
        _minMax[1] = ty;
      }
      if (tx > _minMax[2]) {
        _minMax[2] = tx;
      }
      if (ty > _minMax[3]) {
        _minMax[3] = ty;
      }
    }
  }

  /// Returns a rect that bounds the result of applying the given matrix as a
  /// perspective transform to the given rect.
  ///
  /// This function assumes the given rect is in the plane with z equals 0.0.
  /// The transformed rect is then projected back into the plane with z equals
  /// 0.0 before computing its bounding rect.
  static Rect transformRect(Matrix4 transform, Rect rect) {
    final Float64List storage = transform.storage;
    final double x = rect.left;
    final double y = rect.top;
    final double w = rect.right - x;
    final double h = rect.bottom - y;

    // We want to avoid turning a finite rect into an infinite one if we can.
    if (!w.isFinite || !h.isFinite) {
      return _safeTransformRect(transform, rect);
    }

    // Transforming the 4 corners of a rectangle the straightforward way
    // incurs the cost of transforming 4 points using vector math which
    // involves 48 multiplications and 48 adds and then normalizing
    // the points using 4 inversions of the homogeneous weight factor
    // and then 12 multiplies. Once we have transformed all of the points
    // we then need to turn them into a bounding box using 4 min/max
    // operations each on 4 values yielding 12 total comparisons.
    //
    // On top of all of those operations, using the vector_math package to
    // do the work for us involves allocating several objects in order to
    // communicate the values back and forth - 4 allocating getters to extract
    // the [Offset] objects for the corners of the [Rect], 4 conversions to
    // a [Vector3] to use [Matrix4.perspectiveTransform()], and then 4 new
    // [Offset] objects allocated to hold those results, yielding 8 [Offset]
    // and 4 [Vector3] object allocations per rectangle transformed.
    //
    // But the math we really need to get our answer is actually much less
    // than that.
    //
    // First, consider that a full point transform using the vector math
    // package involves expanding it out into a vector3 with a Z coordinate
    // of 0.0 and then performing 3 multiplies and 3 adds per coordinate:
    //
    //     xt = x*m00 + y*m10 + z*m20 + m30;
    //     yt = x*m01 + y*m11 + z*m21 + m31;
    //     zt = x*m02 + y*m12 + z*m22 + m32;
    //     wt = x*m03 + y*m13 + z*m23 + m33;
    //
    // Immediately we see that we can get rid of the 3rd column of multiplies
    // since we know that Z=0.0. We can also get rid of the 3rd row because
    // we ignore the resulting Z coordinate. Finally we can get rid of the
    // last row if we don't have a perspective transform since we can verify
    // that the results are 1.0 for all points. This gets us down to 16
    // multiplies and 16 adds in the non-perspective case and 24 of each for
    // the perspective case. (Plus the 12 comparisons to turn them back into
    // a bounding box.)
    //
    // But we can do even better than that.
    //
    // Under optimal conditions of no perspective transformation,
    // which is actually a very common condition, we can transform
    // a rectangle in as little as 3 operations:
    //
    // (rx,ry) = transform of upper left corner of rectangle
    // (wx,wy) = delta transform of the (w, 0) width relative vector
    // (hx,hy) = delta transform of the (0, h) height relative vector
    //
    // A delta transform is a transform of all elements of the matrix except
    // for the translation components. The translation components are added
    // in at the end of each transform computation so they represent a
    // constant offset for each point transformed. A delta transform of
    // a horizontal or vertical vector involves a single multiplication due
    // to the fact that it only has one non-zero coordinate and no addition
    // of the translation component.
    //
    // In the absence of a perspective transform, the transformed
    // rectangle will be mapped into a parallelogram with corners at:
    // corner1 = (rx, ry)
    // corner2 = corner1 + dTransformed width vector = (rx+wx, ry+wy)
    // corner3 = corner1 + dTransformed height vector = (rx+hx, ry+hy)
    // corner4 = corner1 + both dTransformed vectors = (rx+wx+hx, ry+wy+hy)
    // In all, this method of transforming the rectangle requires only
    // 8 multiplies and 12 additions (which we can reduce to 8 additions if
    // we only need a bounding box, see below).
    //
    // In the presence of a perspective transform, the above conditions
    // continue to hold with respect to the non-normalized coordinates so
    // we can still save a lot of multiplications by computing the 4
    // non-normalized coordinates using relative additions before we normalize
    // them and they lose their "pseudo-parallelogram" relationships. We still
    // have to do the normalization divisions and min/max all 4 points to
    // get the resulting transformed bounding box, but we save a lot of
    // calculations over blindly transforming all 4 coordinates independently.
    // In all, we need 12 multiplies and 22 additions to construct the
    // non-normalized vectors and then 8 divisions (or 4 inversions and 8
    // multiplies) for normalization (plus the standard set of 12 comparisons
    // for the min/max bounds operations).
    //
    // Back to the non-perspective case, the optimization that lets us get
    // away with fewer additions if we only need a bounding box comes from
    // analyzing the impact of the relative vectors on expanding the
    // bounding box of the parallelogram. First, the bounding box always
    // contains the transformed upper-left corner of the rectangle. Next,
    // each relative vector either pushes on the left or right side of the
    // bounding box and also either the top or bottom side, depending on
    // whether it is positive or negative. Finally, you can consider the
    // impact of each vector on the bounding box independently. If, say,
    // wx and hx have the same sign, then the limiting point in the bounding
    // box will be the one that involves adding both of them to the origin
    // point. If they have opposite signs, then one will push one wall one
    // way and the other will push the opposite wall the other way and when
    // you combine both of them, the resulting "opposite corner" will
    // actually be between the limits they established by pushing the walls
    // away from each other, as below:
    //
    //             +---------(originx,originy)--------------+
    //             |            -----^----                  |
    //             |       -----          ----              |
    //             |  -----                   ----          |
    //     (+hx,+hy)<                             ----      |
    //             |  ----                            ----  |
    //             |      ----                             >(+wx,+wy)
    //             |          ----                   -----  |
    //             |              ----          -----       |
    //             |                  ---- -----            |
    //             |                      v                 |
    //             +---------------(+wx+hx,+wy+hy)----------+
    //
    // In this diagram, consider that:
    //
    //  * wx would be a positive number
    //  * hx would be a negative number
    //  * wy and hy would both be positive numbers
    //
    // As a result, wx pushes out the right wall, hx pushes out the left wall,
    // and both wy and hy push down the bottom wall of the bounding box. The
    // wx,hx pair (of opposite signs) worked on opposite walls and the final
    // opposite corner had an X coordinate between the limits they established.
    // The wy,hy pair (of the same sign) both worked together to push the
    // bottom wall down by their sum.
    //
    // This relationship allows us to simply start with the point computed by
    // transforming the upper left corner of the rectangle, and then
    // conditionally adding wx, wy, hx, and hy to either the left or top
    // or right or bottom of the bounding box independently depending on sign.
    // In that case we only need 4 comparisons and 4 additions total to
    // compute the bounding box, combined with the 8 multiplications and
    // 4 additions to compute the transformed point and relative vectors
    // for a total of 8 multiplies, 8 adds, and 4 comparisons.
    //
    // An astute observer will note that we do need to do 2 subtractions at
    // the top of the method to compute the width and height. Add those to
    // all of the relative solutions listed above. The test for perspective
    // also adds 3 compares to the affine case and up to 3 compares to the
    // perspective case (depending on which test fails, the rest are omitted).
    //
    // The final tally:
    // basic method          = 60 mul + 48 add + 12 compare
    // optimized perspective = 12 mul + 22 add + 15 compare + 2 sub
    // optimized affine      =  8 mul +  8 add +  7 compare + 2 sub
    //
    // Since compares are essentially subtractions and subtractions are
    // the same cost as adds, we end up with:
    // basic method          = 60 mul + 60 add/sub/compare
    // optimized perspective = 12 mul + 39 add/sub/compare
    // optimized affine      =  8 mul + 17 add/sub/compare

    final double wx = storage[0] * w;
    final double hx = storage[4] * h;
    final double rx = storage[0] * x + storage[4] * y + storage[12];

    final double wy = storage[1] * w;
    final double hy = storage[5] * h;
    final double ry = storage[1] * x + storage[5] * y + storage[13];

    if (storage[3] == 0.0 && storage[7] == 0.0 && storage[15] == 1.0) {
      double left = rx;
      double right = rx;
      if (wx < 0) {
        left += wx;
      } else {
        right += wx;
      }
      if (hx < 0) {
        left += hx;
      } else {
        right += hx;
      }

      double top = ry;
      double bottom = ry;
      if (wy < 0) {
        top += wy;
      } else {
        bottom += wy;
      }
      if (hy < 0) {
        top += hy;
      } else {
        bottom += hy;
      }

      return Rect.fromLTRB(left, top, right, bottom);
    } else {
      final double ww = storage[3] * w;
      final double hw = storage[7] * h;
      final double rw = storage[3] * x + storage[7] * y + storage[15];

      final double ulx = rx / rw;
      final double uly = ry / rw;
      final double urx = (rx + wx) / (rw + ww);
      final double ury = (ry + wy) / (rw + ww);
      final double llx = (rx + hx) / (rw + hw);
      final double lly = (ry + hy) / (rw + hw);
      final double lrx = (rx + wx + hx) / (rw + ww + hw);
      final double lry = (ry + wy + hy) / (rw + ww + hw);

      return Rect.fromLTRB(
        _min4(ulx, urx, llx, lrx),
        _min4(uly, ury, lly, lry),
        _max4(ulx, urx, llx, lrx),
        _max4(uly, ury, lly, lry),
      );
    }
  }

  static double _min4(double a, double b, double c, double d) {
    final double e = (a < b) ? a : b;
    final double f = (c < d) ? c : d;
    return (e < f) ? e : f;
  }

  static double _max4(double a, double b, double c, double d) {
    final double e = (a > b) ? a : b;
    final double f = (c > d) ? c : d;
    return (e > f) ? e : f;
  }

  /// Returns a rect that bounds the result of applying the inverse of the given
  /// matrix as a perspective transform to the given rect.
  ///
  /// This function assumes the given rect is in the plane with z equals 0.0.
  /// The transformed rect is then projected back into the plane with z equals
  /// 0.0 before computing its bounding rect.
  static Rect inverseTransformRect(Matrix4 transform, Rect rect) {
    // As exposed by `unrelated_type_equality_checks`, this assert was a no-op.
    // Fixing it introduces a bunch of runtime failures; for more context see:
    // https://github.com/flutter/flutter/pull/31568
    // assert(transform.determinant != 0.0);
    if (isIdentity(transform)) {
      return rect;
    }
    transform = Matrix4.copy(transform)..invert();
    return transformRect(transform, rect);
  }

  /// Create a transformation matrix which mimics the effects of tangentially
  /// wrapping the plane on which this transform is applied around a cylinder
  /// and then looking at the cylinder from a point outside the cylinder.
  ///
  /// The `radius` simulates the radius of the cylinder the plane is being
  /// wrapped onto. If the transformation is applied to a 0-dimensional dot
  /// instead of a plane, the dot would translate by ± `radius` pixels
  /// along the `orientation` [Axis] when rotating from 0 to ±90 degrees.
  ///
  /// A positive radius means the object is closest at 0 `angle` and a negative
  /// radius means the object is closest at π `angle` or 180 degrees.
  ///
  /// The `angle` argument is the difference in angle in radians between the
  /// object and the viewing point. A positive `angle` on a positive `radius`
  /// moves the object up when `orientation` is vertical and right when
  /// horizontal.
  ///
  /// The transformation is always done such that a 0 `angle` keeps the
  /// transformed object at exactly the same size as before regardless of
  /// `radius` and `perspective` when `radius` is positive.
  ///
  /// The `perspective` argument is a number between 0 and 1 where 0 means
  /// looking at the object from infinitely far with an infinitely narrow field
  /// of view and 1 means looking at the object from infinitely close with an
  /// infinitely wide field of view. Defaults to a sane but arbitrary 0.001.
  ///
  /// The `orientation` is the direction of the rotation axis.
  ///
  /// Because the viewing position is a point, it's never possible to see the
  /// outer side of the cylinder at or past ±π/2 or 90 degrees and it's
  /// almost always possible to end up seeing the inner side of the cylinder
  /// or the back side of the transformed plane before π / 2 when perspective > 0.
  static Matrix4 createCylindricalProjectionTransform({
    required double radius,
    required double angle,
    double perspective = 0.001,
    Axis orientation = Axis.vertical,
  }) {
    assert(perspective >= 0 && perspective <= 1.0);

    // Pre-multiplied matrix of a projection matrix and a view matrix.
    //
    // Projection matrix is a simplified perspective matrix
    // http://web.iitd.ac.in/~hegde/cad/lecture/L9_persproj.pdf
    // in the form of
    // [[1.0, 0.0, 0.0, 0.0],
    //  [0.0, 1.0, 0.0, 0.0],
    //  [0.0, 0.0, 1.0, 0.0],
    //  [0.0, 0.0, -perspective, 1.0]]
    //
    // View matrix is a simplified camera view matrix.
    // Basically re-scales to keep object at original size at angle = 0 at
    // any radius in the form of
    // [[1.0, 0.0, 0.0, 0.0],
    //  [0.0, 1.0, 0.0, 0.0],
    //  [0.0, 0.0, 1.0, -radius],
    //  [0.0, 0.0, 0.0, 1.0]]
    Matrix4 result =
        Matrix4.identity()
          ..setEntry(3, 2, -perspective)
          ..setEntry(2, 3, -radius)
          ..setEntry(3, 3, perspective * radius + 1.0);

    // Model matrix by first translating the object from the origin of the world
    // by radius in the z axis and then rotating against the world.
    result =
        result *
                (switch (orientation) {
                      Axis.horizontal => Matrix4.rotationY(angle),
                      Axis.vertical => Matrix4.rotationX(angle),
                    } *
                    Matrix4.translationValues(0.0, 0.0, radius))
            as Matrix4;

    // Essentially perspective * view * model.
    return result;
  }

  /// Returns a matrix that transforms every point to [offset].
  static Matrix4 forceToPoint(Offset offset) {
    return Matrix4.identity()
      ..setRow(0, Vector4(0, 0, 0, offset.dx))
      ..setRow(1, Vector4(0, 0, 0, offset.dy));
  }
}

/// Returns a list of strings representing the given transform in a format
/// useful for [TransformProperty].
///
/// If the argument is null, returns a list with the single string "null".
List<String> debugDescribeTransform(Matrix4? transform) {
  if (transform == null) {
    return const <String>['null'];
  }
  return <String>[
    '[0] ${debugFormatDouble(transform.entry(0, 0))},${debugFormatDouble(transform.entry(0, 1))},${debugFormatDouble(transform.entry(0, 2))},${debugFormatDouble(transform.entry(0, 3))}',
    '[1] ${debugFormatDouble(transform.entry(1, 0))},${debugFormatDouble(transform.entry(1, 1))},${debugFormatDouble(transform.entry(1, 2))},${debugFormatDouble(transform.entry(1, 3))}',
    '[2] ${debugFormatDouble(transform.entry(2, 0))},${debugFormatDouble(transform.entry(2, 1))},${debugFormatDouble(transform.entry(2, 2))},${debugFormatDouble(transform.entry(2, 3))}',
    '[3] ${debugFormatDouble(transform.entry(3, 0))},${debugFormatDouble(transform.entry(3, 1))},${debugFormatDouble(transform.entry(3, 2))},${debugFormatDouble(transform.entry(3, 3))}',
  ];
}

/// Property which handles [Matrix4] that represent transforms.
class TransformProperty extends DiagnosticsProperty<Matrix4> {
  /// Create a diagnostics property for [Matrix4] objects.
  TransformProperty(
    String super.name,
    super.value, {
    super.showName,
    super.defaultValue,
    super.level,
  });

  @override
  String valueToString({TextTreeConfiguration? parentConfiguration}) {
    if (parentConfiguration != null && !parentConfiguration.lineBreakProperties) {
      // Format the value on a single line to be compatible with the parent's
      // style.
      final List<String> values = <String>[
        '${debugFormatDouble(value!.entry(0, 0))},${debugFormatDouble(value!.entry(0, 1))},${debugFormatDouble(value!.entry(0, 2))},${debugFormatDouble(value!.entry(0, 3))}',
        '${debugFormatDouble(value!.entry(1, 0))},${debugFormatDouble(value!.entry(1, 1))},${debugFormatDouble(value!.entry(1, 2))},${debugFormatDouble(value!.entry(1, 3))}',
        '${debugFormatDouble(value!.entry(2, 0))},${debugFormatDouble(value!.entry(2, 1))},${debugFormatDouble(value!.entry(2, 2))},${debugFormatDouble(value!.entry(2, 3))}',
        '${debugFormatDouble(value!.entry(3, 0))},${debugFormatDouble(value!.entry(3, 1))},${debugFormatDouble(value!.entry(3, 2))},${debugFormatDouble(value!.entry(3, 3))}',
      ];
      return '[${values.join('; ')}]';
    }
    return debugDescribeTransform(value).join('\n');
  }
}
