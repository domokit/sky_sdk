// Copyright 2013 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

import 'dart:ffi';

import 'package:ui/src/engine.dart';
import 'package:ui/src/engine/skwasm/skwasm_impl.dart';
import 'package:ui/ui.dart' as ui;

class SkwasmPaint implements ui.Paint {
  SkwasmPaint();

  /// Creates the C++ side paint object based on the current state of this
  /// paint object, and returns it with ownership.
  ///
  /// It is the responsibility of the caller to dispose of the returned handle
  /// when it's no longer needed.
  PaintHandle toRawPaint({ui.TileMode defaultBlurTileMode = ui.TileMode.decal}) {
    final rawPaint = paintCreate(
      isAntiAlias,
      blendMode.index,
      _colorValue,
      style.index,
      strokeWidth,
      strokeCap.index,
      strokeJoin.index,
      strokeMiterLimit,
    );

    _maybeSetEffectiveColorFilter(rawPaint);

    final shaderHandle = _shader?.handle;
    if (shaderHandle != null) {
      paintSetShader(rawPaint, shaderHandle);
    }

    final localMaskFilter = maskFilter;
    if (localMaskFilter != null) {
      final nativeFilter = SkwasmMaskFilter.fromUiMaskFilter(localMaskFilter);
      paintSetMaskFilter(rawPaint, nativeFilter.handle);
      nativeFilter.dispose();
    }

    final filter = imageFilter;
    if (filter != null) {
      final skwasmImageFilter = SkwasmImageFilter.fromUiFilter(filter);
      skwasmImageFilter.withRawImageFilter((nativeHandle) {
        paintSetImageFilter(rawPaint, nativeHandle);
      }, defaultBlurTileMode: defaultBlurTileMode);
    }

    return rawPaint;
  }

  /// If `invertColors` is true or `colorFilter` is not null, sets the
  /// appropriate Skia color filter. Otherwise, does nothing.
  void _maybeSetEffectiveColorFilter(Pointer<RawPaint> handle) {
    final nativeFilter =
        _colorFilter != null ? SkwasmColorFilter.fromEngineColorFilter(_colorFilter!) : null;
    if (invertColors) {
      if (nativeFilter != null) {
        final composedFilter = SkwasmColorFilter.composed(_invertColorFilter, nativeFilter);
        composedFilter.withRawColorFilter((composedFilterHandle) {
          paintSetColorFilter(handle, composedFilterHandle);
        });
      } else {
        _invertColorFilter.withRawColorFilter((invertFilterHandle) {
          paintSetColorFilter(handle, invertFilterHandle);
        });
      }
    } else if (nativeFilter != null) {
      nativeFilter.withRawColorFilter((nativeFilterHandle) {
        paintSetColorFilter(handle, nativeFilterHandle);
      });
    }
  }

  static final SkwasmColorFilter _invertColorFilter = SkwasmColorFilter.fromEngineColorFilter(
    const EngineColorFilter.matrix(<double>[
      -1.0, 0, 0, 1.0, 0, // row
      0, -1.0, 0, 1.0, 0, // row
      0, 0, -1.0, 1.0, 0, // row
      1.0, 1.0, 1.0, 1.0, 0,
    ]),
  );

  @override
  ui.BlendMode blendMode = _kBlendModeDefault;

  // Must be kept in sync with the default in paint.cc.
  static const ui.BlendMode _kBlendModeDefault = ui.BlendMode.srcOver;

  @override
  ui.PaintingStyle style = ui.PaintingStyle.fill;

  @override
  double strokeWidth = 0.0;

  @override
  ui.StrokeCap strokeCap = ui.StrokeCap.butt;

  @override
  ui.StrokeJoin strokeJoin = ui.StrokeJoin.miter;

  @override
  bool isAntiAlias = true;

  @override
  ui.Color get color => ui.Color(_colorValue);
  @override
  set color(ui.Color value) {
    _colorValue = value.value;
  }

  static const int _kColorDefault = 0xFF000000;
  int _colorValue = _kColorDefault;

  @override
  double strokeMiterLimit = _kStrokeMiterLimitDefault;
  static const double _kStrokeMiterLimitDefault = 4.0;

  @override
  ui.Shader? get shader => _shader;

  @override
  set shader(ui.Shader? uiShader) {
    uiShader as SkwasmShader?;
    _shader = uiShader;
  }

  SkwasmShader? _shader;

  @override
  ui.FilterQuality filterQuality = ui.FilterQuality.none;

  @override
  ui.ImageFilter? imageFilter;

  @override
  ui.ColorFilter? get colorFilter => _colorFilter;

  @override
  set colorFilter(ui.ColorFilter? filter) {
    _colorFilter = filter as EngineColorFilter?;
  }

  EngineColorFilter? _colorFilter;

  @override
  ui.MaskFilter? maskFilter;

  @override
  bool invertColors = false;

  @override
  String toString() {
    String resultString = 'Paint()';

    assert(() {
      final StringBuffer result = StringBuffer();
      String semicolon = '';
      result.write('Paint(');
      if (style == ui.PaintingStyle.stroke) {
        result.write('$style');
        if (strokeWidth != 0.0) {
          result.write(' ${strokeWidth.toStringAsFixed(1)}');
        } else {
          result.write(' hairline');
        }
        if (strokeCap != ui.StrokeCap.butt) {
          result.write(' $strokeCap');
        }
        if (strokeJoin == ui.StrokeJoin.miter) {
          if (strokeMiterLimit != _kStrokeMiterLimitDefault) {
            result.write(' $strokeJoin up to ${strokeMiterLimit.toStringAsFixed(1)}');
          }
        } else {
          result.write(' $strokeJoin');
        }
        semicolon = '; ';
      }
      if (!isAntiAlias) {
        result.write('${semicolon}antialias off');
        semicolon = '; ';
      }
      if (color != const ui.Color(_kColorDefault)) {
        result.write('$semicolon$color');
        semicolon = '; ';
      }
      if (blendMode.index != _kBlendModeDefault.index) {
        result.write('$semicolon$blendMode');
        semicolon = '; ';
      }
      if (colorFilter != null) {
        result.write('${semicolon}colorFilter: $colorFilter');
        semicolon = '; ';
      }
      if (maskFilter != null) {
        result.write('${semicolon}maskFilter: $maskFilter');
        semicolon = '; ';
      }
      if (filterQuality != ui.FilterQuality.none) {
        result.write('${semicolon}filterQuality: $filterQuality');
        semicolon = '; ';
      }
      if (shader != null) {
        result.write('${semicolon}shader: $shader');
        semicolon = '; ';
      }
      if (imageFilter != null) {
        result.write('${semicolon}imageFilter: $imageFilter');
        semicolon = '; ';
      }
      if (invertColors) {
        result.write('${semicolon}invert: $invertColors');
      }
      result.write(')');
      resultString = result.toString();
      return true;
    }());

    return resultString;
  }
}
