// Copyright 2013 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

import 'dart:math' as math;
import 'dart:typed_data';

import 'package:ui/ui.dart' as ui;

import '../vector_math.dart';
import 'canvaskit_api.dart';
import 'path.dart';

/// An error related to the CanvasKit rendering backend.
class CanvasKitError extends Error {
  CanvasKitError(this.message);

  /// Describes this error.
  final String message;

  @override
  String toString() => 'CanvasKitError: $message';
}

/// Creates a new color array.
Float32List makeFreshSkColor(ui.Color color) {
  final Float32List result = Float32List(4);
  result[0] = color.red / 255.0;
  result[1] = color.green / 255.0;
  result[2] = color.blue / 255.0;
  result[3] = color.alpha / 255.0;
  return result;
}

ui.TextPosition fromPositionWithAffinity(SkTextPosition positionWithAffinity) {
  final ui.TextAffinity affinity =
      ui.TextAffinity.values[positionWithAffinity.affinity.value.toInt()];
  return ui.TextPosition(offset: positionWithAffinity.pos.toInt(), affinity: affinity);
}

/// Shadow flag constants derived from Skia's SkShadowFlags.h.
class SkiaShadowFlags {
  /// The occluding object is opaque, making the part of the shadow under the
  /// occluder invisible. This allows some optimizations because some parts of
  /// the shadow do not need to be accurate.
  static const int kNone_ShadowFlag = 0x00;

  /// The occluding object is not opaque, making the part of the shadow under the
  /// occluder visible. This requires that the shadow is rendered more accurately
  /// and therefore is slightly more expensive.
  static const int kTransparentOccluder_ShadowFlag = 0x01;

  /// Light position represents a direction, light radius is blur radius at
  /// elevation 1.
  ///
  /// This makes the shadow to have a fixed position relative to the shape that
  /// casts it.
  static const int kDirectionalLight_ShadowFlag = 0x04;

  /// Complete value for the `flags` argument for opaque occluder.
  static const int kDefaultShadowFlags = kDirectionalLight_ShadowFlag | kNone_ShadowFlag;

  /// Complete value for the `flags` argument for transparent occluder.
  static const int kTransparentOccluderShadowFlags =
      kDirectionalLight_ShadowFlag | kTransparentOccluder_ShadowFlag;
}

// These numbers have been chosen empirically to give a result closest to the
// material spec.
const double ckShadowAmbientAlpha = 0.039;
const double ckShadowSpotAlpha = 0.25;
const double ckShadowLightXOffset = 0;
const double ckShadowLightYOffset = -450;
const double ckShadowLightHeight = 600;
const double ckShadowLightRadius = 800;
const double ckShadowLightXTangent = ckShadowLightXOffset / ckShadowLightHeight;
const double ckShadowLightYTangent = ckShadowLightYOffset / ckShadowLightHeight;

/// Computes the smallest rectangle that contains the shadow.
// Most of this logic is borrowed from SkDrawShadowInfo.cpp in Skia.
// TODO(yjbanov): switch to SkDrawShadowMetrics::GetLocalBounds when available
//                See:
//                  - https://bugs.chromium.org/p/skia/issues/detail?id=11146
//                  - https://github.com/flutter/flutter/issues/73492
ui.Rect computeSkShadowBounds(
  CkPath path,
  double elevation,
  double devicePixelRatio,
  Matrix4 matrix,
) {
  ui.Rect pathBounds = path.getBounds();

  if (elevation == 0) {
    return pathBounds;
  }

  // For visual correctness the shadow offset and blur does not change with
  // parent transforms. Therefore, in general case we have to first transform
  // the shape bounds to device coordinates, then compute the shadow bounds,
  // then transform the bounds back to local coordinates. However, if the
  // transform is an identity or translation (a common case), we can skip this
  // step. With directional lighting translation does not affect the size or
  // shape of the shadow. Skipping this step saves us two transformRects and
  // one matrix inverse.
  final bool isComplex = !matrix.isIdentityOrTranslation();
  if (isComplex) {
    pathBounds = matrix.transformRect(pathBounds);
  }

  double left = pathBounds.left;
  double top = pathBounds.top;
  double right = pathBounds.right;
  double bottom = pathBounds.bottom;

  final double ambientBlur = ambientBlurRadius(elevation);
  final double spotBlur = ckShadowLightRadius * elevation;
  final double spotOffsetX = -elevation * ckShadowLightXTangent;
  final double spotOffsetY = -elevation * ckShadowLightYTangent;

  // The extra +1/-1 are to cover possible floating point errors.
  left = left - 1 + (spotOffsetX - ambientBlur - spotBlur) * devicePixelRatio;
  top = top - 1 + (spotOffsetY - ambientBlur - spotBlur) * devicePixelRatio;
  right = right + 1 + (spotOffsetX + ambientBlur + spotBlur) * devicePixelRatio;
  bottom = bottom + 1 + (spotOffsetY + ambientBlur + spotBlur) * devicePixelRatio;

  final ui.Rect shadowBounds = ui.Rect.fromLTRB(left, top, right, bottom);

  if (isComplex) {
    final Matrix4 inverse = Matrix4.zero();
    // The inverse only makes sense if the determinat is non-zero.
    if (inverse.copyInverse(matrix) != 0.0) {
      return inverse.transformRect(shadowBounds);
    } else {
      return shadowBounds;
    }
  } else {
    return shadowBounds;
  }
}

const double kAmbientHeightFactor = 1.0 / 128.0;
const double kAmbientGeomFactor = 64.0;
const double kMaxAmbientRadius = 300 * kAmbientHeightFactor * kAmbientGeomFactor;

double ambientBlurRadius(double height) {
  return math.min(height * kAmbientHeightFactor * kAmbientGeomFactor, kMaxAmbientRadius);
}

void drawSkShadow(
  SkCanvas skCanvas,
  CkPath path,
  ui.Color color,
  double elevation,
  bool transparentOccluder,
  double devicePixelRatio,
) {
  int flags =
      transparentOccluder
          ? SkiaShadowFlags.kTransparentOccluderShadowFlags
          : SkiaShadowFlags.kDefaultShadowFlags;
  flags |= SkiaShadowFlags.kDirectionalLight_ShadowFlag;

  final ui.Color inAmbient = color.withAlpha((color.alpha * ckShadowAmbientAlpha).round());
  final ui.Color inSpot = color.withAlpha((color.alpha * ckShadowSpotAlpha).round());

  final SkTonalColors inTonalColors = SkTonalColors(
    ambient: makeFreshSkColor(inAmbient),
    spot: makeFreshSkColor(inSpot),
  );

  final SkTonalColors tonalColors = canvasKit.computeTonalColors(inTonalColors);

  skCanvas.drawShadow(
    path.skiaObject,
    Float32List(3)..[2] = devicePixelRatio * elevation,
    Float32List(3)
      ..[0] = 0
      ..[1] = -1
      ..[2] = 1,
    ckShadowLightRadius / ckShadowLightHeight,
    tonalColors.ambient,
    tonalColors.spot,
    flags.toDouble(),
  );
}
