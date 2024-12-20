// Copyright 2013 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

import 'package:ui/ui.dart' as ui;

import '../color_filter.dart';
import '../util.dart';

/// Implementation of [ui.Paint] used by the HTML rendering backend.
class SurfacePaint implements ui.Paint {
  SurfacePaintData _paintData = SurfacePaintData();

  @override
  ui.BlendMode get blendMode => _paintData.blendMode ?? ui.BlendMode.srcOver;

  @override
  set blendMode(ui.BlendMode value) {
    if (_frozen) {
      _paintData = _paintData.clone();
      _frozen = false;
    }
    _paintData.blendMode = value;
  }

  @override
  ui.PaintingStyle get style => _paintData.style ?? ui.PaintingStyle.fill;

  @override
  set style(ui.PaintingStyle value) {
    if (_frozen) {
      _paintData = _paintData.clone();
      _frozen = false;
    }
    _paintData.style = value;
  }

  @override
  double get strokeWidth => _paintData.strokeWidth ?? 0.0;

  @override
  set strokeWidth(double value) {
    if (_frozen) {
      _paintData = _paintData.clone();
      _frozen = false;
    }
    _paintData.strokeWidth = value;
  }

  @override
  ui.StrokeCap get strokeCap => _paintData.strokeCap ?? ui.StrokeCap.butt;

  @override
  set strokeCap(ui.StrokeCap value) {
    if (_frozen) {
      _paintData = _paintData.clone();
      _frozen = false;
    }
    _paintData.strokeCap = value;
  }

  @override
  ui.StrokeJoin get strokeJoin => _paintData.strokeJoin ?? ui.StrokeJoin.miter;

  @override
  set strokeJoin(ui.StrokeJoin value) {
    if (_frozen) {
      _paintData = _paintData.clone();
      _frozen = false;
    }
    _paintData.strokeJoin = value;
  }

  @override
  bool get isAntiAlias => _paintData.isAntiAlias;

  @override
  set isAntiAlias(bool value) {
    if (_frozen) {
      _paintData = _paintData.clone();
      _frozen = false;
    }
    _paintData.isAntiAlias = value;
  }

  @override
  ui.Color get color => ui.Color(_paintData.color);

  @override
  set color(ui.Color value) {
    if (_frozen) {
      _paintData = _paintData.clone();
      _frozen = false;
    }
    _paintData.color = value.value;
  }

  @override
  bool invertColors = false;

  @override
  ui.Shader? get shader => _paintData.shader;

  @override
  set shader(ui.Shader? value) {
    if (_frozen) {
      _paintData = _paintData.clone();
      _frozen = false;
    }
    _paintData.shader = value;
  }

  @override
  ui.MaskFilter? get maskFilter => _paintData.maskFilter;

  @override
  set maskFilter(ui.MaskFilter? value) {
    if (_frozen) {
      _paintData = _paintData.clone();
      _frozen = false;
    }
    _paintData.maskFilter = value;
  }

  @override
  ui.FilterQuality get filterQuality => _paintData.filterQuality ?? ui.FilterQuality.none;

  @override
  set filterQuality(ui.FilterQuality value) {
    if (_frozen) {
      _paintData = _paintData.clone();
      _frozen = false;
    }
    _paintData.filterQuality = value;
  }

  @override
  ui.ColorFilter? get colorFilter => _paintData.colorFilter;

  @override
  set colorFilter(ui.ColorFilter? value) {
    if (_frozen) {
      _paintData = _paintData.clone();
      _frozen = false;
    }
    _paintData.colorFilter = value as EngineColorFilter?;
  }

  // TODO(ferhat): see https://github.com/flutter/flutter/issues/33605
  @override
  double strokeMiterLimit = 4.0;

  // TODO(ferhat): Implement ImageFilter, flutter/flutter#35156.
  @override
  ui.ImageFilter? imageFilter;

  // True if Paint instance has used in RecordingCanvas.
  bool _frozen = false;

  // Marks this paint object as previously used.
  SurfacePaintData get paintData {
    // Flip bit so next time object gets mutated we create a clone of
    // current paint data.
    _frozen = true;
    return _paintData;
  }

  // Must be kept in sync with the default in paint.cc.
  static const double _kStrokeMiterLimitDefault = 4.0;

  // Must be kept in sync with the default in paint.cc.
  static const int _kColorDefault = 0xFF000000;

  // Must be kept in sync with the default in paint.cc.
  static final int _kBlendModeDefault = ui.BlendMode.srcOver.index;

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
      if (blendMode.index != _kBlendModeDefault) {
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

/// Private Paint context data used for recording canvas commands allowing
/// Paint to be mutated post canvas draw operations.
class SurfacePaintData {
  ui.BlendMode? blendMode;
  ui.PaintingStyle? style;
  double? strokeWidth;
  ui.StrokeCap? strokeCap;
  ui.StrokeJoin? strokeJoin;
  bool isAntiAlias = true;
  int color = 0xFF000000;
  ui.Shader? shader;
  ui.MaskFilter? maskFilter;
  ui.FilterQuality? filterQuality;
  EngineColorFilter? colorFilter;

  // Internal for recording canvas use.
  SurfacePaintData clone() {
    return SurfacePaintData()
      ..blendMode = blendMode
      ..filterQuality = filterQuality
      ..maskFilter = maskFilter
      ..shader = shader
      ..isAntiAlias = isAntiAlias
      ..color = color
      ..colorFilter = colorFilter
      ..strokeWidth = strokeWidth
      ..style = style
      ..strokeJoin = strokeJoin
      ..strokeCap = strokeCap;
  }

  @override
  String toString() {
    String result = super.toString();
    assert(() {
      final StringBuffer buffer = StringBuffer('SurfacePaintData(');
      if (blendMode != null) {
        buffer.write('blendMode = $blendMode; ');
      }
      if (style != null) {
        buffer.write('style = $style; ');
      }
      if (strokeWidth != null) {
        buffer.write('strokeWidth = $strokeWidth; ');
      }
      if (strokeCap != null) {
        buffer.write('strokeCap = $strokeCap; ');
      }
      if (strokeJoin != null) {
        buffer.write('strokeJoin = $strokeJoin; ');
      }
      buffer.write('color = ${ui.Color(color).toCssString()}; ');
      if (shader != null) {
        buffer.write('shader = $shader; ');
      }
      if (maskFilter != null) {
        buffer.write('maskFilter = $maskFilter; ');
      }
      if (filterQuality != null) {
        buffer.write('filterQuality = $filterQuality; ');
      }
      if (colorFilter != null) {
        buffer.write('colorFilter = $colorFilter; ');
      }
      buffer.write('isAntiAlias = $isAntiAlias)');
      result = buffer.toString();

      return true;
    }());

    return result;
  }
}

class HtmlFragmentProgram implements ui.FragmentProgram {
  @override
  ui.FragmentShader fragmentShader() {
    throw UnsupportedError('FragmentProgram is not supported for the HTML renderer.');
  }
}

class HtmlFragmentShader implements ui.FragmentShader {
  @override
  void setFloat(int index, double value) {
    throw UnsupportedError('FragmentShader is not supported for the HTML renderer.');
  }

  @override
  void setImageSampler(int index, ui.Image image) {
    throw UnsupportedError('FragmentShader is not supported for the HTML renderer.');
  }

  @override
  void dispose() {
    throw UnsupportedError('FragmentShader is not supported for the HTML renderer.');
  }

  @override
  bool get debugDisposed {
    throw UnsupportedError('FragmentShader is not supported for the HTML renderer.');
  }
}
