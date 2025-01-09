// Copyright 2013 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

import 'dart:typed_data';

import 'package:ui/src/engine/vector_math.dart';
import 'package:ui/ui.dart' as ui;

import '../color_filter.dart';
import '../util.dart';
import 'canvaskit_api.dart';
import 'image_filter.dart';
import 'native_memory.dart';

/// Owns a [SkColorFilter] and manages its lifecycle.
///
/// See also:
///
/// * [CkPaint.colorFilter], which uses a [ManagedSkColorFilter] to manage
///   the lifecycle of its [SkColorFilter].
class ManagedSkColorFilter {
  ManagedSkColorFilter(CkColorFilter ckColorFilter) : colorFilter = ckColorFilter {
    _ref = UniqueRef<SkColorFilter>(this, colorFilter._initRawColorFilter(), 'ColorFilter');
  }

  final CkColorFilter colorFilter;

  late final UniqueRef<SkColorFilter> _ref;

  SkColorFilter get skiaObject => _ref.nativeObject;

  @override
  int get hashCode => colorFilter.hashCode;

  @override
  bool operator ==(Object other) {
    if (runtimeType != other.runtimeType) {
      return false;
    }
    return other is ManagedSkColorFilter && other.colorFilter == colorFilter;
  }

  @override
  String toString() => colorFilter.toString();
}

/// CanvasKit implementation of [ui.ColorFilter].
abstract class CkColorFilter implements CkManagedSkImageFilterConvertible {
  const CkColorFilter();

  /// Converts this color filter into an image filter.
  ///
  /// Passes the ownership of the returned [SkImageFilter] to the caller. It is
  /// the caller's responsibility to manage the lifecycle of the returned value.
  SkImageFilter initRawImageFilter() {
    final SkColorFilter skColorFilter = _initRawColorFilter();
    final SkImageFilter result = canvasKit.ImageFilter.MakeColorFilter(skColorFilter, null);

    // The underlying SkColorFilter is now owned by the SkImageFilter, so we
    // need to drop the reference to allow it to be collected.
    skColorFilter.delete();
    return result;
  }

  /// Creates a Skia object based on the properties of this color filter.
  ///
  /// Passes the ownership of the returned [SkColorFilter] to the caller. It is
  /// the caller's responsibility to manage the lifecycle of the returned value.
  SkColorFilter _initRawColorFilter();

  @override
  void withSkImageFilter(
    SkImageFilterBorrow borrow, {
    ui.TileMode defaultBlurTileMode = ui.TileMode.clamp,
  }) {
    // Since ColorFilter has a const constructor it cannot store dynamically
    // created Skia objects. Therefore a new SkImageFilter is created every time
    // it's used. However, once used it's no longer needed, so it's deleted
    // immediately to free memory.
    final SkImageFilter skImageFilter = initRawImageFilter();
    borrow(skImageFilter);
    skImageFilter.delete();
  }

  /// The blur ImageFilter will override this and return the necessary
  /// value to hand to the saveLayer call. It is the only filter type that
  /// needs to pass along a tile mode so we just return a default value of
  /// clamp for color filters.
  @override
  ui.TileMode? get backdropTileMode => ui.TileMode.clamp;

  @override
  Matrix4 get transform => Matrix4.identity();
}

/// A reusable identity transform matrix.
///
/// WARNING: DO NOT MUTATE THIS MATRIX! It is a shared global singleton.
Float32List _identityTransform = _computeIdentityTransform();

Float32List _computeIdentityTransform() {
  final Float32List result = Float32List(20);
  const List<int> translationIndices = <int>[0, 6, 12, 18];
  for (final int i in translationIndices) {
    result[i] = 1;
  }
  _identityTransform = result;
  return result;
}

SkColorFilter createSkColorFilterFromColorAndBlendMode(ui.Color color, ui.BlendMode blendMode) {
  final SkColorFilter? filter = canvasKit.ColorFilter.MakeBlend(
    toSharedSkColor1(color),
    toSkBlendMode(blendMode),
  );
  if (filter == null) {
    // If CanvasKit returns null, then the ColorFilter with this combination of
    // color and blend mode is a no-op. So just return a dummy color filter that
    // does nothing.
    return canvasKit.ColorFilter.MakeMatrix(_identityTransform);
  }
  return filter;
}

class CkBlendModeColorFilter extends CkColorFilter {
  const CkBlendModeColorFilter(this.color, this.blendMode);

  final ui.Color color;
  final ui.BlendMode blendMode;

  @override
  SkColorFilter _initRawColorFilter() {
    return createSkColorFilterFromColorAndBlendMode(color, blendMode);
  }

  @override
  int get hashCode => Object.hash(color, blendMode);

  @override
  bool operator ==(Object other) {
    if (runtimeType != other.runtimeType) {
      return false;
    }
    return other is CkBlendModeColorFilter && other.color == color && other.blendMode == blendMode;
  }

  @override
  String toString() => 'ColorFilter.mode($color, $blendMode)';
}

class CkMatrixColorFilter extends CkColorFilter {
  const CkMatrixColorFilter(this.matrix);

  final List<double> matrix;

  /// Flutter documentation says the translation column of the color matrix
  /// is specified in unnormalized 0..255 space. CanvasKit expects the
  /// translation values to be normalized to 0..1 space.
  ///
  /// See [https://api.flutter.dev/flutter/dart-ui/ColorFilter/ColorFilter.matrix.html].
  Float32List get _normalizedMatrix {
    assert(matrix.length == 20, 'Color Matrix must have 20 entries.');
    final Float32List result = Float32List(20);
    const List<int> translationIndices = <int>[4, 9, 14, 19];
    for (int i = 0; i < 20; i++) {
      if (translationIndices.contains(i)) {
        result[i] = matrix[i] / 255.0;
      } else {
        result[i] = matrix[i];
      }
    }
    return result;
  }

  @override
  SkColorFilter _initRawColorFilter() {
    return canvasKit.ColorFilter.MakeMatrix(_normalizedMatrix);
  }

  @override
  int get hashCode => Object.hashAll(matrix);

  @override
  bool operator ==(Object other) {
    return runtimeType == other.runtimeType &&
        other is CkMatrixColorFilter &&
        listEquals<double>(matrix, other.matrix);
  }

  @override
  String toString() => 'ColorFilter.matrix($matrix)';
}

class CkLinearToSrgbGammaColorFilter extends CkColorFilter {
  const CkLinearToSrgbGammaColorFilter();
  @override
  SkColorFilter _initRawColorFilter() => canvasKit.ColorFilter.MakeLinearToSRGBGamma();

  @override
  bool operator ==(Object other) => runtimeType == other.runtimeType;

  @override
  int get hashCode => runtimeType.hashCode;

  @override
  String toString() => 'ColorFilter.linearToSrgbGamma()';
}

class CkSrgbToLinearGammaColorFilter extends CkColorFilter {
  const CkSrgbToLinearGammaColorFilter();
  @override
  SkColorFilter _initRawColorFilter() => canvasKit.ColorFilter.MakeSRGBToLinearGamma();

  @override
  bool operator ==(Object other) => runtimeType == other.runtimeType;

  @override
  int get hashCode => runtimeType.hashCode;

  @override
  String toString() => 'ColorFilter.srgbToLinearGamma()';
}

class CkComposeColorFilter extends CkColorFilter {
  const CkComposeColorFilter(this.outer, this.inner);
  final ManagedSkColorFilter? outer;
  final ManagedSkColorFilter inner;

  @override
  SkColorFilter _initRawColorFilter() =>
      canvasKit.ColorFilter.MakeCompose(outer?.skiaObject, inner.skiaObject);

  @override
  bool operator ==(Object other) {
    if (other is! CkComposeColorFilter) {
      return false;
    }
    final CkComposeColorFilter filter = other;
    return filter.outer == outer && filter.inner == inner;
  }

  @override
  int get hashCode => Object.hash(outer, inner);

  @override
  String toString() => 'ColorFilter.compose($outer, $inner)';
}

/// Convert the current [ColorFilter] to a CkColorFilter.
///
/// This workaround allows ColorFilter to be const constructbile and
/// efficiently comparable, so that widgets can check for ColorFilter equality to
/// avoid repainting.
CkColorFilter? createCkColorFilter(EngineColorFilter colorFilter) {
  switch (colorFilter.type) {
    case ColorFilterType.mode:
      if (colorFilter.color == null || colorFilter.blendMode == null) {
        return null;
      }
      return CkBlendModeColorFilter(colorFilter.color!, colorFilter.blendMode!);
    case ColorFilterType.matrix:
      if (colorFilter.matrix == null) {
        return null;
      }
      assert(colorFilter.matrix!.length == 20, 'Color Matrix must have 20 entries.');
      return CkMatrixColorFilter(colorFilter.matrix!);
    case ColorFilterType.linearToSrgbGamma:
      return const CkLinearToSrgbGammaColorFilter();
    case ColorFilterType.srgbToLinearGamma:
      return const CkSrgbToLinearGammaColorFilter();
  }
}
