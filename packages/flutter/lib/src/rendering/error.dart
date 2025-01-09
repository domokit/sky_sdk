// Copyright 2014 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

import 'dart:ui'
    as ui
    show Paragraph, ParagraphBuilder, ParagraphConstraints, ParagraphStyle, TextStyle;

import 'package:flutter/foundation.dart';

import 'box.dart';
import 'object.dart';

const double _kMaxWidth = 100000.0;
const double _kMaxHeight = 100000.0;

/// A render object used as a placeholder when an error occurs.
///
/// The box will be painted in the color given by the
/// [RenderErrorBox.backgroundColor] static property.
///
/// A message can be provided. To simplify the class and thus help reduce the
/// likelihood of this class itself being the source of errors, the message
/// cannot be changed once the object has been created. If provided, the text
/// will be painted on top of the background, using the styles given by the
/// [RenderErrorBox.textStyle] and [RenderErrorBox.paragraphStyle] static
/// properties.
///
/// Again to help simplify the class, if the parent has left the constraints
/// unbounded, this box tries to be 100000.0 pixels wide and high, to
/// approximate being infinitely high but without using infinities.
class RenderErrorBox extends RenderBox {
  /// Creates a RenderErrorBox render object.
  ///
  /// A message can optionally be provided. If a message is provided, an attempt
  /// will be made to render the message when the box paints.
  RenderErrorBox([this.message = '']) {
    try {
      if (message != '') {
        // This class is intentionally doing things using the low-level
        // primitives to avoid depending on any subsystems that may have ended
        // up in an unstable state -- after all, this class is mainly used when
        // things have gone wrong.
        //
        // Generally, the much better way to draw text in a RenderObject is to
        // use the TextPainter class. If you're looking for code to crib from,
        // see the paragraph.dart file and the RenderParagraph class.
        final ui.ParagraphBuilder builder = ui.ParagraphBuilder(paragraphStyle);
        builder.pushStyle(textStyle);
        builder.addText(message);
        _paragraph = builder.build();
      } else {
        _paragraph = null;
      }
    } catch (error) {
      // If an error happens here we're in a terrible state, so we really should
      // just forget about it and let the developer deal with the already-reported
      // errors. It's unlikely that these errors are going to help with that.
    }
  }

  /// The message to attempt to display at paint time.
  final String message;

  late final ui.Paragraph? _paragraph;

  @override
  double computeMaxIntrinsicWidth(double height) {
    return _kMaxWidth;
  }

  @override
  double computeMaxIntrinsicHeight(double width) {
    return _kMaxHeight;
  }

  @override
  bool get sizedByParent => true;

  @override
  bool hitTestSelf(Offset position) => true;

  @override
  @protected
  Size computeDryLayout(covariant BoxConstraints constraints) {
    return constraints.constrain(const Size(_kMaxWidth, _kMaxHeight));
  }

  /// The distance to place around the text.
  ///
  /// This is intended to ensure that if the [RenderErrorBox] is placed at the top left
  /// of the screen, under the system's status bar, the error text is still visible in
  /// the area below the status bar.
  ///
  /// The padding is ignored if the error box is smaller than the padding.
  ///
  /// See also:
  ///
  ///  * [minimumWidth], which controls how wide the box must be before the
  ///    horizontal padding is applied.
  static EdgeInsets padding = const EdgeInsets.fromLTRB(64.0, 96.0, 64.0, 12.0);

  /// The width below which the horizontal padding is not applied.
  ///
  /// If the left and right padding would reduce the available width to less than
  /// this value, then the text is rendered flush with the left edge.
  static double minimumWidth = 200.0;

  /// The color to use when painting the background of [RenderErrorBox] objects.
  ///
  /// Defaults to red in debug mode, a light gray otherwise.
  static Color backgroundColor = _initBackgroundColor();

  static Color _initBackgroundColor() {
    Color result = const Color(0xF0C0C0C0);
    assert(() {
      result = const Color(0xF0900000);
      return true;
    }());
    return result;
  }

  /// The text style to use when painting [RenderErrorBox] objects.
  ///
  /// Defaults to a yellow monospace font in debug mode, and a dark gray
  /// sans-serif font otherwise.
  static ui.TextStyle textStyle = _initTextStyle();

  static ui.TextStyle _initTextStyle() {
    ui.TextStyle result = ui.TextStyle(
      color: const Color(0xFF303030),
      fontFamily: 'sans-serif',
      fontSize: 18.0,
    );
    assert(() {
      result = ui.TextStyle(
        color: const Color(0xFFFFFF66),
        fontFamily: 'monospace',
        fontSize: 14.0,
        fontWeight: FontWeight.bold,
      );
      return true;
    }());
    return result;
  }

  /// The paragraph style to use when painting [RenderErrorBox] objects.
  static ui.ParagraphStyle paragraphStyle = ui.ParagraphStyle(
    textDirection: TextDirection.ltr,
    textAlign: TextAlign.left,
  );

  @override
  void paint(PaintingContext context, Offset offset) {
    try {
      context.canvas.drawRect(offset & size, Paint()..color = backgroundColor);
      if (_paragraph != null) {
        double width = size.width;
        double left = 0.0;
        double top = 0.0;
        if (width > padding.left + minimumWidth + padding.right) {
          width -= padding.left + padding.right;
          left += padding.left;
        }
        _paragraph.layout(ui.ParagraphConstraints(width: width));
        if (size.height > padding.top + _paragraph.height + padding.bottom) {
          top += padding.top;
        }
        context.canvas.drawParagraph(_paragraph, offset + Offset(left, top));
      }
    } catch (error) {
      // If an error happens here we're in a terrible state, so we really should
      // just forget about it and let the developer deal with the already-reported
      // errors. It's unlikely that these errors are going to help with that.
    }
  }
}
