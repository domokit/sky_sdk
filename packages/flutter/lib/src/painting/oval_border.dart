// Copyright 2014 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

/// @docImport 'box_border.dart';
/// @docImport 'box_decoration.dart';
/// @docImport 'shape_decoration.dart';
library;

import 'dart:ui' as ui show lerpDouble;

import 'package:flutter/foundation.dart';

import 'borders.dart';
import 'circle_border.dart';

/// A border that fits an elliptical shape.
///
/// Typically used with [ShapeDecoration] to draw an oval. Instead of centering
/// the [Border] to a square, like [CircleBorder], it fills the available space,
/// such that it touches the edges of the box. There is no difference between
/// `CircleBorder(eccentricity = 1.0)` and `OvalBorder()`. [OvalBorder] works as
/// an alias for users to discover this feature.
///
/// See also:
///
///  * [CircleBorder], which draws a circle, centering when the box is rectangular.
///  * [Border], which, when used with [BoxDecoration], can also describe an oval.
class OvalBorder extends CircleBorder {
  /// Create an oval border.
  const OvalBorder({super.side, super.eccentricity = 1.0});

  @override
  ShapeBorder scale(double t) => OvalBorder(side: side.scale(t), eccentricity: eccentricity);

  @override
  OvalBorder copyWith({BorderSide? side, double? eccentricity}) {
    return OvalBorder(side: side ?? this.side, eccentricity: eccentricity ?? this.eccentricity);
  }

  @override
  ShapeBorder? lerpFrom(ShapeBorder? a, double t) {
    if (a is OvalBorder) {
      return OvalBorder(
        side: BorderSide.lerp(a.side, side, t),
        eccentricity: clampDouble(ui.lerpDouble(a.eccentricity, eccentricity, t)!, 0.0, 1.0),
      );
    }
    return super.lerpFrom(a, t);
  }

  @override
  ShapeBorder? lerpTo(ShapeBorder? b, double t) {
    if (b is OvalBorder) {
      return OvalBorder(
        side: BorderSide.lerp(side, b.side, t),
        eccentricity: clampDouble(ui.lerpDouble(eccentricity, b.eccentricity, t)!, 0.0, 1.0),
      );
    }
    return super.lerpTo(b, t);
  }

  @override
  String toString() {
    if (eccentricity != 1.0) {
      return '${objectRuntimeType(this, 'OvalBorder')}($side, eccentricity: $eccentricity)';
    }
    return '${objectRuntimeType(this, 'OvalBorder')}($side)';
  }
}
