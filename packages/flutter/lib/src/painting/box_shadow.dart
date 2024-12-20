// Copyright 2014 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

/// @docImport 'package:flutter/material.dart';
/// @docImport 'package:flutter_test/flutter_test.dart';
library;

import 'dart:math' as math;
import 'dart:ui' as ui show Shadow, lerpDouble;

import 'package:flutter/foundation.dart';

import 'basic_types.dart';
import 'debug.dart';

/// A shadow cast by a box.
///
/// [BoxShadow] can cast non-rectangular shadows if the box is non-rectangular
/// (e.g., has a border radius or a circular shape).
///
/// This class is similar to CSS box-shadow.
///
/// See also:
///
///  * [Canvas.drawShadow], which is a more efficient way to draw shadows.
///  * [PhysicalModel], a widget for showing shadows.
///  * [kElevationToShadow], for some predefined shadows used in Material
///    Design.
///  * [Shadow], which is the parent class that lacks [spreadRadius].
@immutable
class BoxShadow extends ui.Shadow {
  /// Creates a box shadow.
  ///
  /// By default, the shadow is solid black with zero [offset], zero [blurRadius],
  /// zero [spreadRadius], and [BlurStyle.normal].
  const BoxShadow({
    super.color,
    super.offset,
    super.blurRadius,
    this.spreadRadius = 0.0,
    this.blurStyle = BlurStyle.normal,
  });

  /// The amount the box should be inflated prior to applying the blur.
  final double spreadRadius;

  /// The [BlurStyle] to use for this shadow.
  ///
  /// Defaults to [BlurStyle.normal].
  ///
  /// When [debugDisableShadows] is true, [toPaint] ignores the [blurStyle] and
  /// acts as if [BlurStyle.normal] was used.
  final BlurStyle blurStyle;

  /// Create the [Paint] object that corresponds to this shadow description.
  ///
  /// The [offset] and [spreadRadius] are not represented in the [Paint] object.
  /// To honor those as well, the shape should be inflated by [spreadRadius] pixels
  /// in every direction and then translated by [offset] before being filled using
  /// this [Paint].
  ///
  /// The [blurStyle] is ignored if [debugDisableShadows] is true. This causes
  /// an especially significant change to the rendering when [BlurStyle.outer]
  /// is used; the caller is responsible for adjusting for that case if
  /// necessary. (This only matters when using [debugDisableShadows], e.g. in
  /// tests that use [matchesGoldenFile].)
  @override
  Paint toPaint() {
    final Paint result =
        Paint()
          ..color = color
          ..maskFilter = MaskFilter.blur(blurStyle, blurSigma);
    assert(() {
      if (debugDisableShadows) {
        result.maskFilter = null;
      }
      return true;
    }());
    return result;
  }

  /// Returns a new box shadow with its offset, blurRadius, and spreadRadius
  /// scaled by the given factor.
  @override
  BoxShadow scale(double factor) {
    return BoxShadow(
      color: color,
      offset: offset * factor,
      blurRadius: blurRadius * factor,
      spreadRadius: spreadRadius * factor,
      blurStyle: blurStyle,
    );
  }

  /// Creates a copy of this object but with the given fields replaced with the
  /// new values.
  BoxShadow copyWith({
    Color? color,
    Offset? offset,
    double? blurRadius,
    double? spreadRadius,
    BlurStyle? blurStyle,
  }) {
    return BoxShadow(
      color: color ?? this.color,
      offset: offset ?? this.offset,
      blurRadius: blurRadius ?? this.blurRadius,
      spreadRadius: spreadRadius ?? this.spreadRadius,
      blurStyle: blurStyle ?? this.blurStyle,
    );
  }

  /// Linearly interpolate between two box shadows.
  ///
  /// If either box shadow is null, this function linearly interpolates from
  /// a box shadow that matches the other box shadow in color but has a zero
  /// offset and a zero blurRadius.
  ///
  /// {@macro dart.ui.shadow.lerp}
  static BoxShadow? lerp(BoxShadow? a, BoxShadow? b, double t) {
    if (identical(a, b)) {
      return a;
    }
    if (a == null) {
      return b!.scale(t);
    }
    if (b == null) {
      return a.scale(1.0 - t);
    }
    return BoxShadow(
      color: Color.lerp(a.color, b.color, t)!,
      offset: Offset.lerp(a.offset, b.offset, t)!,
      blurRadius: ui.lerpDouble(a.blurRadius, b.blurRadius, t)!,
      spreadRadius: ui.lerpDouble(a.spreadRadius, b.spreadRadius, t)!,
      blurStyle: a.blurStyle == BlurStyle.normal ? b.blurStyle : a.blurStyle,
    );
  }

  /// Linearly interpolate between two lists of box shadows.
  ///
  /// If the lists differ in length, excess items are lerped with null.
  ///
  /// {@macro dart.ui.shadow.lerp}
  static List<BoxShadow>? lerpList(List<BoxShadow>? a, List<BoxShadow>? b, double t) {
    if (identical(a, b)) {
      return a;
    }
    a ??= <BoxShadow>[];
    b ??= <BoxShadow>[];
    final int commonLength = math.min(a.length, b.length);
    return <BoxShadow>[
      for (int i = 0; i < commonLength; i += 1) BoxShadow.lerp(a[i], b[i], t)!,
      for (int i = commonLength; i < a.length; i += 1) a[i].scale(1.0 - t),
      for (int i = commonLength; i < b.length; i += 1) b[i].scale(t),
    ];
  }

  @override
  bool operator ==(Object other) {
    if (identical(this, other)) {
      return true;
    }
    if (other.runtimeType != runtimeType) {
      return false;
    }
    return other is BoxShadow &&
        other.color == color &&
        other.offset == offset &&
        other.blurRadius == blurRadius &&
        other.spreadRadius == spreadRadius &&
        other.blurStyle == blurStyle;
  }

  @override
  int get hashCode => Object.hash(color, offset, blurRadius, spreadRadius, blurStyle);

  @override
  String toString() =>
      'BoxShadow($color, $offset, ${debugFormatDouble(blurRadius)}, ${debugFormatDouble(spreadRadius)}, $blurStyle)';
}
