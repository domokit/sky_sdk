// Copyright 2014 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

/// @docImport 'box_decoration.dart';
library;

import 'dart:math' as math;
import 'dart:ui' as ui show Gradient, TextBox, lerpDouble;

import 'package:flutter/foundation.dart';

import 'alignment.dart';
import 'basic_types.dart';
import 'box_fit.dart';
import 'colors.dart';
import 'decoration.dart';
import 'edge_insets.dart';
import 'image_provider.dart';
import 'text_painter.dart';
import 'text_span.dart';
import 'text_style.dart';

/// Possible ways to draw Flutter's logo.
enum FlutterLogoStyle {
  /// Show only Flutter's logo, not the "Flutter" label.
  ///
  /// This is the default behavior for [FlutterLogoDecoration] objects.
  markOnly,

  /// Show Flutter's logo on the left, and the "Flutter" label to its right.
  horizontal,

  /// Show Flutter's logo above the "Flutter" label.
  stacked,
}

/// An immutable description of how to paint Flutter's logo.
class FlutterLogoDecoration extends Decoration {
  /// Creates a decoration that knows how to paint Flutter's logo.
  ///
  /// The [style] controls whether and where to draw the "Flutter" label. If one
  /// is shown, the [textColor] controls the color of the label.
  const FlutterLogoDecoration({
    this.textColor = const Color(0xFF757575),
    this.style = FlutterLogoStyle.markOnly,
    this.margin = EdgeInsets.zero,
  }) : _position =
           identical(style, FlutterLogoStyle.markOnly)
               ? 0.0
               : identical(style, FlutterLogoStyle.horizontal)
               ? 1.0
               : -1.0,
       _opacity = 1.0;

  const FlutterLogoDecoration._(
    this.textColor,
    this.style,
    this.margin,
    this._position,
    this._opacity,
  );

  /// The color used to paint the "Flutter" text on the logo, if [style] is
  /// [FlutterLogoStyle.horizontal] or [FlutterLogoStyle.stacked].
  ///
  /// If possible, the default (a medium grey) should be used against a white
  /// background.
  final Color textColor;

  /// Whether and where to draw the "Flutter" text. By default, only the logo
  /// itself is drawn.
  // This property isn't actually used when painting. It's only really used to
  // set the internal _position property.
  final FlutterLogoStyle style;

  /// How far to inset the logo from the edge of the container.
  final EdgeInsets margin;

  // The following are set when lerping, to represent states that can't be
  // represented by the constructor.
  final double _position; // -1.0 for stacked, 1.0 for horizontal, 0.0 for no logo
  final double _opacity; // 0.0 .. 1.0

  bool get _inTransition =>
      _opacity != 1.0 || (_position != -1.0 && _position != 0.0 && _position != 1.0);

  @override
  bool debugAssertIsValid() {
    assert(_position.isFinite && _opacity >= 0.0 && _opacity <= 1.0);
    return true;
  }

  @override
  bool get isComplex => !_inTransition;

  /// Linearly interpolate between two Flutter logo descriptions.
  ///
  /// Interpolates both the color and the style in a continuous fashion.
  ///
  /// If both values are null, this returns null. Otherwise, it returns a
  /// non-null value. If one of the values is null, then the result is obtained
  /// by scaling the other value's opacity and [margin].
  ///
  /// {@macro dart.ui.shadow.lerp}
  ///
  /// See also:
  ///
  ///  * [Decoration.lerp], which interpolates between arbitrary decorations.
  static FlutterLogoDecoration? lerp(FlutterLogoDecoration? a, FlutterLogoDecoration? b, double t) {
    assert(a == null || a.debugAssertIsValid());
    assert(b == null || b.debugAssertIsValid());
    if (identical(a, b)) {
      return a;
    }
    if (a == null) {
      return FlutterLogoDecoration._(
        b!.textColor,
        b.style,
        b.margin * t,
        b._position,
        b._opacity * clampDouble(t, 0.0, 1.0),
      );
    }
    if (b == null) {
      return FlutterLogoDecoration._(
        a.textColor,
        a.style,
        a.margin * t,
        a._position,
        a._opacity * clampDouble(1.0 - t, 0.0, 1.0),
      );
    }
    if (t == 0.0) {
      return a;
    }
    if (t == 1.0) {
      return b;
    }
    return FlutterLogoDecoration._(
      Color.lerp(a.textColor, b.textColor, t)!,
      t < 0.5 ? a.style : b.style,
      EdgeInsets.lerp(a.margin, b.margin, t)!,
      a._position + (b._position - a._position) * t,
      clampDouble(a._opacity + (b._opacity - a._opacity) * t, 0.0, 1.0),
    );
  }

  @override
  FlutterLogoDecoration? lerpFrom(Decoration? a, double t) {
    assert(debugAssertIsValid());
    if (a is FlutterLogoDecoration?) {
      assert(a?.debugAssertIsValid() ?? true);
      return FlutterLogoDecoration.lerp(a, this, t);
    }
    return super.lerpFrom(a, t) as FlutterLogoDecoration?;
  }

  @override
  FlutterLogoDecoration? lerpTo(Decoration? b, double t) {
    assert(debugAssertIsValid());
    if (b is FlutterLogoDecoration?) {
      assert(b?.debugAssertIsValid() ?? true);
      return FlutterLogoDecoration.lerp(this, b, t);
    }
    return super.lerpTo(b, t) as FlutterLogoDecoration?;
  }

  @override
  // TODO(ianh): better hit testing
  bool hitTest(Size size, Offset position, {TextDirection? textDirection}) => true;

  @override
  BoxPainter createBoxPainter([VoidCallback? onChanged]) {
    assert(debugAssertIsValid());
    return _FlutterLogoPainter(this);
  }

  @override
  Path getClipPath(Rect rect, TextDirection textDirection) {
    return Path()..addRect(rect);
  }

  @override
  bool operator ==(Object other) {
    assert(debugAssertIsValid());
    if (identical(this, other)) {
      return true;
    }
    return other is FlutterLogoDecoration &&
        other.textColor == textColor &&
        other._position == _position &&
        other._opacity == _opacity;
  }

  @override
  int get hashCode {
    assert(debugAssertIsValid());
    return Object.hash(textColor, _position, _opacity);
  }

  @override
  void debugFillProperties(DiagnosticPropertiesBuilder properties) {
    super.debugFillProperties(properties);
    properties.add(ColorProperty('textColor', textColor));
    properties.add(EnumProperty<FlutterLogoStyle>('style', style));
    if (_inTransition) {
      properties.add(
        DiagnosticsNode.message(
          'transition ${debugFormatDouble(_position)}:${debugFormatDouble(_opacity)}',
        ),
      );
    }
  }
}

/// An object that paints a [BoxDecoration] into a canvas.
class _FlutterLogoPainter extends BoxPainter {
  _FlutterLogoPainter(this._config) : assert(_config.debugAssertIsValid()), super(null) {
    _prepareText();
  }

  final FlutterLogoDecoration _config;

  // these are configured assuming a font size of 100.0.
  late TextPainter _textPainter;
  late Rect _textBoundingRect;

  @override
  void dispose() {
    _textPainter.dispose();
    super.dispose();
  }

  void _prepareText() {
    const String kLabel = 'Flutter';
    _textPainter = TextPainter(
      text: TextSpan(
        text: kLabel,
        style: TextStyle(
          color: _config.textColor,
          fontFamily: 'Roboto',
          fontSize:
              100.0 *
              350.0 /
              247.0, // 247 is the height of the F when the fontSize is 350, assuming device pixel ratio 1.0
          fontWeight: FontWeight.w300,
          textBaseline: TextBaseline.alphabetic,
        ),
      ),
      textDirection: TextDirection.ltr,
    );
    _textPainter.layout();
    final ui.TextBox textSize =
        _textPainter
            .getBoxesForSelection(const TextSelection(baseOffset: 0, extentOffset: kLabel.length))
            .single;
    _textBoundingRect = Rect.fromLTRB(textSize.left, textSize.top, textSize.right, textSize.bottom);
  }

  // This class contains a lot of magic numbers. They were derived from the
  // values in the SVG files exported from the original artwork source.

  void _paintLogo(Canvas canvas, Rect rect) {
    // Our points are in a coordinate space that's 166 pixels wide and 202 pixels high.
    // First, transform the rectangle so that our coordinate space is a square 202 pixels
    // to a side, with the top left at the origin.
    canvas.save();
    canvas.translate(rect.left, rect.top);
    canvas.scale(rect.width / 202.0, rect.height / 202.0);
    // Next, offset it some more so that the 166 horizontal pixels are centered
    // in that square (as opposed to being on the left side of it). This means
    // that if we draw in the rectangle from 0,0 to 166,202, we are drawing in
    // the center of the given rect.
    canvas.translate((202.0 - 166.0) / 2.0, 0.0);

    // Set up the styles.
    final Paint lightPaint = Paint()..color = const Color(0xFF54C5F8);
    final Paint mediumPaint = Paint()..color = const Color(0xFF29B6F6);
    final Paint darkPaint = Paint()..color = const Color(0xFF01579B);

    final ui.Gradient triangleGradient = ui.Gradient.linear(
      const Offset(87.2623 + 37.9092, 28.8384 + 123.4389),
      const Offset(42.9205 + 37.9092, 35.0952 + 123.4389),
      <Color>[const Color(0x001A237E), const Color(0x661A237E)],
    );
    final Paint trianglePaint = Paint()..shader = triangleGradient;

    // Draw the basic shape.
    final Path topBeam =
        Path()
          ..moveTo(37.7, 128.9)
          ..lineTo(9.8, 101.0)
          ..lineTo(100.4, 10.4)
          ..lineTo(156.2, 10.4);
    canvas.drawPath(topBeam, lightPaint);

    final Path middleBeam =
        Path()
          ..moveTo(156.2, 94.0)
          ..lineTo(100.4, 94.0)
          ..lineTo(78.5, 115.9)
          ..lineTo(106.4, 143.8);
    canvas.drawPath(middleBeam, lightPaint);

    final Path bottomBeam =
        Path()
          ..moveTo(79.5, 170.7)
          ..lineTo(100.4, 191.6)
          ..lineTo(156.2, 191.6)
          ..lineTo(107.4, 142.8);
    canvas.drawPath(bottomBeam, darkPaint);

    // The overlap between middle and bottom beam.
    canvas.save();
    canvas.transform(
      Float64List.fromList(const <double>[
        // careful, this is in _column_-major order
        0.7071, -0.7071, 0.0, 0.0,
        0.7071, 0.7071, 0.0, 0.0,
        0.0, 0.0, 1.0, 0.0,
        -77.697, 98.057, 0.0, 1.0,
      ]),
    );
    canvas.drawRect(const Rect.fromLTWH(59.8, 123.1, 39.4, 39.4), mediumPaint);
    canvas.restore();

    // The gradients below the middle beam on top of the bottom beam.
    final Path triangle =
        Path()
          ..moveTo(79.5, 170.7)
          ..lineTo(120.9, 156.4)
          ..lineTo(107.4, 142.8);
    canvas.drawPath(triangle, trianglePaint);

    canvas.restore();
  }

  @override
  void paint(Canvas canvas, Offset offset, ImageConfiguration configuration) {
    offset += _config.margin.topLeft;
    final Size canvasSize = _config.margin.deflateSize(configuration.size!);
    if (canvasSize.isEmpty) {
      return;
    }
    final Size logoSize = switch (_config._position) {
      > 0.0 => const Size(820.0, 232.0), // horizontal style
      < 0.0 => const Size(252.0, 306.0), // stacked style
      _ => const Size(202.0, 202.0), // only the mark
    };
    final FittedSizes fittedSize = applyBoxFit(BoxFit.contain, logoSize, canvasSize);
    assert(fittedSize.source == logoSize);
    final Rect rect = Alignment.center.inscribe(fittedSize.destination, offset & canvasSize);
    final double centerSquareHeight = canvasSize.shortestSide;
    final Rect centerSquare = Rect.fromLTWH(
      offset.dx + (canvasSize.width - centerSquareHeight) / 2.0,
      offset.dy + (canvasSize.height - centerSquareHeight) / 2.0,
      centerSquareHeight,
      centerSquareHeight,
    );

    final Rect logoTargetSquare;
    if (_config._position > 0.0) {
      // horizontal style
      logoTargetSquare = Rect.fromLTWH(rect.left, rect.top, rect.height, rect.height);
    } else if (_config._position < 0.0) {
      // stacked style
      final double logoHeight = rect.height * 191.0 / 306.0;
      logoTargetSquare = Rect.fromLTWH(
        rect.left + (rect.width - logoHeight) / 2.0,
        rect.top,
        logoHeight,
        logoHeight,
      );
    } else {
      // only the mark
      logoTargetSquare = centerSquare;
    }
    final Rect logoSquare = Rect.lerp(centerSquare, logoTargetSquare, _config._position.abs())!;

    if (_config._opacity < 1.0) {
      canvas.saveLayer(
        offset & canvasSize,
        Paint()
          ..colorFilter = ColorFilter.mode(
            const Color(0xFFFFFFFF).withOpacity(_config._opacity),
            BlendMode.modulate,
          ),
      );
    }
    if (_config._position != 0.0) {
      if (_config._position > 0.0) {
        // horizontal style
        final double fontSize = 2.0 / 3.0 * logoSquare.height * (1 - (10.4 * 2.0) / 202.0);
        final double scale = fontSize / 100.0;
        final double finalLeftTextPosition = // position of text in rest position
            (256.4 / 820.0) *
                rect.width - // 256.4 is the distance from the left edge to the left of the F when the whole logo is 820.0 wide
            (32.0 / 350.0) * fontSize; // 32 is the distance from the text bounding box edge to the left edge of the F when the font size is 350
        final double initialLeftTextPosition = // position of text when just starting the animation
            rect.width / 2.0 - _textBoundingRect.width * scale;
        final Offset textOffset = Offset(
          rect.left +
              ui.lerpDouble(initialLeftTextPosition, finalLeftTextPosition, _config._position)!,
          rect.top + (rect.height - _textBoundingRect.height * scale) / 2.0,
        );
        canvas.save();
        if (_config._position < 1.0) {
          final Offset center = logoSquare.center;
          final Path path =
              Path()
                ..moveTo(center.dx, center.dy)
                ..lineTo(center.dx + rect.width, center.dy - rect.width)
                ..lineTo(center.dx + rect.width, center.dy + rect.width)
                ..close();
          canvas.clipPath(path);
        }
        canvas.translate(textOffset.dx, textOffset.dy);
        canvas.scale(scale, scale);
        _textPainter.paint(canvas, Offset.zero);
        canvas.restore();
      } else if (_config._position < 0.0) {
        // stacked style
        final double fontSize = 0.35 * logoTargetSquare.height * (1 - (10.4 * 2.0) / 202.0);
        final double scale = fontSize / 100.0;
        if (_config._position > -1.0) {
          // This limits what the drawRect call below is going to blend with.
          canvas.saveLayer(_textBoundingRect, Paint());
        } else {
          canvas.save();
        }
        canvas.translate(
          logoTargetSquare.center.dx - (_textBoundingRect.width * scale / 2.0),
          logoTargetSquare.bottom,
        );
        canvas.scale(scale, scale);
        _textPainter.paint(canvas, Offset.zero);
        if (_config._position > -1.0) {
          canvas.drawRect(
            _textBoundingRect.inflate(_textBoundingRect.width * 0.5),
            Paint()
              ..blendMode = BlendMode.modulate
              ..shader = ui.Gradient.linear(
                Offset(_textBoundingRect.width * -0.5, 0.0),
                Offset(_textBoundingRect.width * 1.5, 0.0),
                <Color>[
                  const Color(0xFFFFFFFF),
                  const Color(0xFFFFFFFF),
                  const Color(0x00FFFFFF),
                  const Color(0x00FFFFFF),
                ],
                <double>[
                  0.0,
                  math.max(0.0, _config._position.abs() - 0.1),
                  math.min(_config._position.abs() + 0.1, 1.0),
                  1.0,
                ],
              ),
          );
        }
        canvas.restore();
      }
    }
    _paintLogo(canvas, logoSquare);
    if (_config._opacity < 1.0) {
      canvas.restore();
    }
  }
}
