// Copyright 2013 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

import 'dart:typed_data';

import 'package:ui/src/engine.dart';
import 'package:ui/src/engine/skwasm/skwasm_impl.dart';
import 'package:ui/ui.dart' as ui;

typedef ImageFilterHandleBorrow = void Function(ImageFilterHandle handle);

abstract class SkwasmImageFilter implements SceneImageFilter {
  const SkwasmImageFilter();

  factory SkwasmImageFilter.blur({
    double sigmaX = 0.0,
    double sigmaY = 0.0,
    ui.TileMode? tileMode,
  }) => SkwasmBlurFilter(sigmaX, sigmaY, tileMode);

  factory SkwasmImageFilter.dilate({double radiusX = 0.0, double radiusY = 0.0}) =>
      SkwasmDilateFilter(radiusX, radiusY);

  factory SkwasmImageFilter.erode({double radiusX = 0.0, double radiusY = 0.0}) =>
      SkwasmErodeFilter(radiusX, radiusY);

  factory SkwasmImageFilter.matrix(
    Float64List matrix4, {
    ui.FilterQuality filterQuality = ui.FilterQuality.low,
  }) => SkwasmMatrixFilter(matrix4, filterQuality);

  factory SkwasmImageFilter.fromColorFilter(SkwasmColorFilter filter) =>
      SkwasmColorImageFilter(filter);

  factory SkwasmImageFilter.fromUiFilter(ui.ImageFilter filter) {
    if (filter is ui.ColorFilter) {
      return SkwasmImageFilter.fromColorFilter(
        SkwasmColorFilter.fromEngineColorFilter(filter as EngineColorFilter),
      );
    } else {
      return filter as SkwasmImageFilter;
    }
  }

  factory SkwasmImageFilter.compose(ui.ImageFilter outer, ui.ImageFilter inner) =>
      SkwasmComposedImageFilter(
        SkwasmImageFilter.fromUiFilter(outer),
        SkwasmImageFilter.fromUiFilter(inner),
      );

  /// Creates a temporary [ImageFilterHandle] and passes it to the [borrow]
  /// function.
  ///
  /// If (and only if) the filter is a blur ImageFilter, then the indicated
  /// [defaultBlurTileMode] is used in place of a missing (null) tile mode.
  ///
  /// The handle is deleted immediately after [borrow] returns. The [borrow]
  /// function must not store the handle to avoid dangling pointer bugs.
  void withRawImageFilter(
    ImageFilterHandleBorrow borrow, {
    ui.TileMode defaultBlurTileMode = ui.TileMode.clamp,
  });

  ui.TileMode? get backdropTileMode => ui.TileMode.clamp;

  @override
  ui.Rect filterBounds(ui.Rect inputBounds) => withStackScope((StackScope scope) {
    final RawIRect rawRect = scope.convertIRectToNative(inputBounds);
    withRawImageFilter((handle) {
      imageFilterGetFilterBounds(handle, rawRect);
    });
    return scope.convertIRectFromNative(rawRect);
  });
}

class SkwasmBlurFilter extends SkwasmImageFilter {
  const SkwasmBlurFilter(this.sigmaX, this.sigmaY, this.tileMode);

  final double sigmaX;
  final double sigmaY;
  final ui.TileMode? tileMode;

  @override
  ui.TileMode? get backdropTileMode => tileMode;

  @override
  void withRawImageFilter(
    ImageFilterHandleBorrow borrow, {
    ui.TileMode defaultBlurTileMode = ui.TileMode.clamp,
  }) {
    final rawImageFilter = imageFilterCreateBlur(
      sigmaX,
      sigmaY,
      (tileMode ?? defaultBlurTileMode).index,
    );
    borrow(rawImageFilter);
    imageFilterDispose(rawImageFilter);
  }

  @override
  String toString() => 'ImageFilter.blur($sigmaX, $sigmaY, ${tileModeString(tileMode)})';

  @override
  Matrix4? get transform => null;
}

class SkwasmDilateFilter extends SkwasmImageFilter {
  const SkwasmDilateFilter(this.radiusX, this.radiusY);

  final double radiusX;
  final double radiusY;

  @override
  void withRawImageFilter(
    ImageFilterHandleBorrow borrow, {
    ui.TileMode defaultBlurTileMode = ui.TileMode.clamp,
  }) {
    final rawImageFilter = imageFilterCreateDilate(radiusX, radiusY);
    borrow(rawImageFilter);
    imageFilterDispose(rawImageFilter);
  }

  @override
  String toString() => 'ImageFilter.dilate($radiusX, $radiusY)';

  @override
  Matrix4? get transform => null;
}

class SkwasmErodeFilter extends SkwasmImageFilter {
  const SkwasmErodeFilter(this.radiusX, this.radiusY);

  final double radiusX;
  final double radiusY;

  @override
  void withRawImageFilter(
    ImageFilterHandleBorrow borrow, {
    ui.TileMode defaultBlurTileMode = ui.TileMode.clamp,
  }) {
    final rawImageFilter = imageFilterCreateErode(radiusX, radiusY);
    borrow(rawImageFilter);
    imageFilterDispose(rawImageFilter);
  }

  @override
  String toString() => 'ImageFilter.erode($radiusX, $radiusY)';

  @override
  Matrix4? get transform => null;
}

class SkwasmMatrixFilter extends SkwasmImageFilter {
  const SkwasmMatrixFilter(this.matrix4, this.filterQuality);

  final Float64List matrix4;
  final ui.FilterQuality filterQuality;

  @override
  void withRawImageFilter(
    ImageFilterHandleBorrow borrow, {
    ui.TileMode defaultBlurTileMode = ui.TileMode.clamp,
  }) {
    withStackScope((scope) {
      final rawImageFilter = imageFilterCreateMatrix(
        scope.convertMatrix4toSkMatrix(matrix4),
        filterQuality.index,
      );
      borrow(rawImageFilter);
      imageFilterDispose(rawImageFilter);
    });
  }

  @override
  String toString() => 'ImageFilter.matrix($matrix4, $filterQuality)';

  @override
  Matrix4? get transform => Matrix4.fromFloat32List(toMatrix32(matrix4));
}

class SkwasmColorImageFilter extends SkwasmImageFilter {
  const SkwasmColorImageFilter(this.filter);

  final SkwasmColorFilter filter;

  @override
  void withRawImageFilter(
    ImageFilterHandleBorrow borrow, {
    ui.TileMode defaultBlurTileMode = ui.TileMode.clamp,
  }) {
    filter.withRawColorFilter((colroFilterHandle) {
      final rawImageFilter = imageFilterCreateFromColorFilter(colroFilterHandle);
      borrow(rawImageFilter);
      imageFilterDispose(rawImageFilter);
    });
  }

  @override
  String toString() => filter.toString();

  @override
  Matrix4? get transform => null;
}

class SkwasmComposedImageFilter extends SkwasmImageFilter {
  const SkwasmComposedImageFilter(this.outer, this.inner);

  final SkwasmImageFilter outer;
  final SkwasmImageFilter inner;

  @override
  void withRawImageFilter(
    ImageFilterHandleBorrow borrow, {
    ui.TileMode defaultBlurTileMode = ui.TileMode.clamp,
  }) {
    outer.withRawImageFilter((outerHandle) {
      inner.withRawImageFilter((innerHandle) {
        final rawImageFilter = imageFilterCompose(outerHandle, innerHandle);
        borrow(rawImageFilter);
        imageFilterDispose(rawImageFilter);
      }, defaultBlurTileMode: defaultBlurTileMode);
    }, defaultBlurTileMode: defaultBlurTileMode);
  }

  @override
  String toString() => 'ImageFilter.compose($outer, $inner)';

  @override
  Matrix4? get transform {
    final outerTransform = outer.transform;
    final innerTransform = inner.transform;
    if (outerTransform != null && innerTransform != null) {
      return outerTransform.multiplied(innerTransform);
    }
    return outerTransform ?? innerTransform;
  }
}

typedef ColorFilterHandleBorrow = void Function(ColorFilterHandle handle);

abstract class SkwasmColorFilter {
  const SkwasmColorFilter();

  factory SkwasmColorFilter.fromEngineColorFilter(EngineColorFilter colorFilter) =>
      switch (colorFilter.type) {
        ColorFilterType.mode => SkwasmModeColorFilter(colorFilter.color!, colorFilter.blendMode!),
        ColorFilterType.linearToSrgbGamma => const SkwasmLinearToSrgbGammaColorFilter(),
        ColorFilterType.srgbToLinearGamma => const SkwasmSrgbToLinearGammaColorFilter(),
        ColorFilterType.matrix => SkwasmMatrixColorFilter(colorFilter.matrix!),
      };

  factory SkwasmColorFilter.composed(SkwasmColorFilter outer, SkwasmColorFilter inner) =>
      SkwasmComposedColorFilter(outer, inner);

  /// Creates a temporary [ColorFilterHandle] and passes it to the [borrow]
  /// function.
  ///
  /// The handle is deleted immediately after [borrow] returns. The [borrow]
  /// function must not store the handle to avoid dangling pointer bugs.
  void withRawColorFilter(ColorFilterHandleBorrow borrow);
}

class SkwasmModeColorFilter extends SkwasmColorFilter {
  const SkwasmModeColorFilter(this.color, this.blendMode);

  final ui.Color color;
  final ui.BlendMode blendMode;

  @override
  void withRawColorFilter(ColorFilterHandleBorrow borrow) {
    final rawColorFilter = colorFilterCreateMode(color.value, blendMode.index);
    borrow(rawColorFilter);
    colorFilterDispose(rawColorFilter);
  }

  @override
  String toString() => 'ColorFilter.mode($color, $blendMode)';
}

class SkwasmLinearToSrgbGammaColorFilter extends SkwasmColorFilter {
  const SkwasmLinearToSrgbGammaColorFilter();

  /// This filter does not need to be deleted, because the same instance can
  /// reused everywhere (it's not configurable).
  static final _rawColorFilter = colorFilterCreateLinearToSRGBGamma();

  @override
  void withRawColorFilter(ColorFilterHandleBorrow borrow) {
    borrow(_rawColorFilter);
  }

  @override
  String toString() => 'ColorFilter.linearToSrgbGamma()';
}

class SkwasmSrgbToLinearGammaColorFilter extends SkwasmColorFilter {
  const SkwasmSrgbToLinearGammaColorFilter();

  /// This filter does not need to be deleted, because the same instance can
  /// reused everywhere (it's not configurable).
  static final _rawColorFilter = colorFilterCreateSRGBToLinearGamma();

  @override
  void withRawColorFilter(ColorFilterHandleBorrow borrow) {
    borrow(_rawColorFilter);
  }

  @override
  String toString() => 'ColorFilter.srgbToLinearGamma()';
}

class SkwasmMatrixColorFilter extends SkwasmColorFilter {
  const SkwasmMatrixColorFilter(this.matrix);

  final List<double> matrix;

  @override
  void withRawColorFilter(ColorFilterHandleBorrow borrow) {
    withStackScope((scope) {
      final rawColorFilter = colorFilterCreateMatrix(scope.convertDoublesToNative(matrix));
      borrow(rawColorFilter);
      colorFilterDispose(rawColorFilter);
    });
  }

  @override
  String toString() => 'ColorFilter.matrix($matrix)';
}

class SkwasmComposedColorFilter extends SkwasmColorFilter {
  const SkwasmComposedColorFilter(this.outer, this.inner);

  final SkwasmColorFilter outer;
  final SkwasmColorFilter inner;

  @override
  void withRawColorFilter(ColorFilterHandleBorrow borrow) {
    outer.withRawColorFilter((outerHandle) {
      inner.withRawColorFilter((innerHandle) {
        final rawColorFilter = colorFilterCompose(outerHandle, innerHandle);
        borrow(rawColorFilter);
        colorFilterDispose(rawColorFilter);
      });
    });
  }

  @override
  String toString() => 'ColorFilter.compose($outer, $inner)';
}

class SkwasmMaskFilter extends SkwasmObjectWrapper<RawMaskFilter> {
  SkwasmMaskFilter._(MaskFilterHandle handle) : super(handle, _registry);

  factory SkwasmMaskFilter.fromUiMaskFilter(ui.MaskFilter maskFilter) => SkwasmMaskFilter._(
    maskFilterCreateBlur(maskFilter.webOnlyBlurStyle.index, maskFilter.webOnlySigma),
  );

  static final SkwasmFinalizationRegistry<RawMaskFilter> _registry =
      SkwasmFinalizationRegistry<RawMaskFilter>(maskFilterDispose);
}
