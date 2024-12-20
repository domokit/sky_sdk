// Copyright 2014 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

/// @docImport 'package:flutter/material.dart';
///
/// @docImport 'app.dart';
library;

import 'dart:math' as math;

import 'package:flutter/foundation.dart';

import 'basic.dart';
import 'debug.dart';
import 'framework.dart';

const double _kOffset = 40.0; // distance to bottom of banner, at a 45 degree angle inwards
const double _kHeight = 12.0; // height of banner
const double _kBottomOffset = _kOffset + math.sqrt1_2 * _kHeight;
const Rect _kRect = Rect.fromLTWH(-_kOffset, _kOffset - _kHeight, _kOffset * 2.0, _kHeight);
const BoxShadow _kShadow = BoxShadow(color: Color(0x7F000000), blurRadius: 6.0);

const Color _kColor = Color(0xA0B71C1C);
const TextStyle _kTextStyle = TextStyle(
  color: Color(0xFFFFFFFF),
  fontSize: _kHeight * 0.85,
  fontWeight: FontWeight.w900,
  height: 1.0,
);

const String _flutterWidgetsLibrary = 'package:flutter/widgets.dart';

/// Where to show a [Banner].
///
/// The start and end locations are relative to the ambient [Directionality]
/// (which can be overridden by [Banner.layoutDirection]).
enum BannerLocation {
  /// Show the banner in the top-right corner when the ambient [Directionality]
  /// (or [Banner.layoutDirection]) is [TextDirection.rtl] and in the top-left
  /// corner when the ambient [Directionality] is [TextDirection.ltr].
  topStart,

  /// Show the banner in the top-left corner when the ambient [Directionality]
  /// (or [Banner.layoutDirection]) is [TextDirection.rtl] and in the top-right
  /// corner when the ambient [Directionality] is [TextDirection.ltr].
  topEnd,

  /// Show the banner in the bottom-right corner when the ambient
  /// [Directionality] (or [Banner.layoutDirection]) is [TextDirection.rtl] and
  /// in the bottom-left corner when the ambient [Directionality] is
  /// [TextDirection.ltr].
  bottomStart,

  /// Show the banner in the bottom-left corner when the ambient
  /// [Directionality] (or [Banner.layoutDirection]) is [TextDirection.rtl] and
  /// in the bottom-right corner when the ambient [Directionality] is
  /// [TextDirection.ltr].
  bottomEnd,
}

/// Paints a [Banner].
class BannerPainter extends CustomPainter {
  /// Creates a banner painter.
  BannerPainter({
    required this.message,
    required this.textDirection,
    required this.location,
    required this.layoutDirection,
    this.color = _kColor,
    this.textStyle = _kTextStyle,
    this.shadow = _kShadow,
  }) : super(repaint: PaintingBinding.instance.systemFonts) {
    // TODO(polina-c): stop duplicating code across disposables
    // https://github.com/flutter/flutter/issues/137435
    if (kFlutterMemoryAllocationsEnabled) {
      FlutterMemoryAllocations.instance.dispatchObjectCreated(
        library: _flutterWidgetsLibrary,
        className: '$BannerPainter',
        object: this,
      );
    }
  }

  /// The message to show in the banner.
  final String message;

  /// The directionality of the text.
  ///
  /// This value is used to disambiguate how to render bidirectional text. For
  /// example, if the message is an English phrase followed by a Hebrew phrase,
  /// in a [TextDirection.ltr] context the English phrase will be on the left
  /// and the Hebrew phrase to its right, while in a [TextDirection.rtl]
  /// context, the English phrase will be on the right and the Hebrew phrase on
  /// its left.
  ///
  /// See also:
  ///
  ///  * [layoutDirection], which controls the interpretation of values in
  ///    [location].
  final TextDirection textDirection;

  /// Where to show the banner (e.g., the upper right corner).
  final BannerLocation location;

  /// The directionality of the layout.
  ///
  /// This value is used to interpret the [location] of the banner.
  ///
  /// See also:
  ///
  ///  * [textDirection], which controls the reading direction of the [message].
  final TextDirection layoutDirection;

  /// The color to paint behind the [message].
  ///
  /// Defaults to a dark red.
  final Color color;

  /// The text style to use for the [message].
  ///
  /// Defaults to bold, white text.
  final TextStyle textStyle;

  /// The shadow properties for the banner.
  ///
  /// Use a [BoxShadow] object to define the shadow's color, blur radius,
  /// and spread radius. These properties can be used to create different
  /// shadow effects.
  final BoxShadow shadow;

  bool _prepared = false;
  TextPainter? _textPainter;
  late Paint _paintShadow;
  late Paint _paintBanner;

  /// Release resources held by this painter.
  ///
  /// After calling this method, this object is no longer usable.
  void dispose() {
    // TODO(polina-c): stop duplicating code across disposables
    // https://github.com/flutter/flutter/issues/137435
    if (kFlutterMemoryAllocationsEnabled) {
      FlutterMemoryAllocations.instance.dispatchObjectDisposed(object: this);
    }
    _textPainter?.dispose();
    _textPainter = null;
  }

  void _prepare() {
    _paintShadow = shadow.toPaint();
    _paintBanner = Paint()..color = color;
    _textPainter?.dispose();
    _textPainter = TextPainter(
      text: TextSpan(style: textStyle, text: message),
      textAlign: TextAlign.center,
      textDirection: textDirection,
    );
    _prepared = true;
  }

  @override
  void paint(Canvas canvas, Size size) {
    if (!_prepared) {
      _prepare();
    }
    canvas
      ..translate(_translationX(size.width), _translationY(size.height))
      ..rotate(_rotation)
      ..drawRect(_kRect, _paintShadow)
      ..drawRect(_kRect, _paintBanner);
    const double width = _kOffset * 2.0;
    _textPainter!.layout(minWidth: width, maxWidth: width);
    _textPainter!.paint(
      canvas,
      _kRect.topLeft + Offset(0.0, (_kRect.height - _textPainter!.height) / 2.0),
    );
  }

  @override
  bool shouldRepaint(BannerPainter oldDelegate) {
    return message != oldDelegate.message ||
        location != oldDelegate.location ||
        color != oldDelegate.color ||
        textStyle != oldDelegate.textStyle;
  }

  @override
  bool hitTest(Offset position) => false;

  double _translationX(double width) {
    return switch ((layoutDirection, location)) {
      (TextDirection.rtl, BannerLocation.topStart) => width,
      (TextDirection.ltr, BannerLocation.topStart) => 0.0,
      (TextDirection.rtl, BannerLocation.topEnd) => 0.0,
      (TextDirection.ltr, BannerLocation.topEnd) => width,
      (TextDirection.rtl, BannerLocation.bottomStart) => width - _kBottomOffset,
      (TextDirection.ltr, BannerLocation.bottomStart) => _kBottomOffset,
      (TextDirection.rtl, BannerLocation.bottomEnd) => _kBottomOffset,
      (TextDirection.ltr, BannerLocation.bottomEnd) => width - _kBottomOffset,
    };
  }

  double _translationY(double height) {
    return switch (location) {
      BannerLocation.bottomStart || BannerLocation.bottomEnd => height - _kBottomOffset,
      BannerLocation.topStart || BannerLocation.topEnd => 0.0,
    };
  }

  double get _rotation {
    return math.pi /
        4.0 *
        switch ((layoutDirection, location)) {
          (TextDirection.rtl, BannerLocation.topStart || BannerLocation.bottomEnd) => 1,
          (TextDirection.ltr, BannerLocation.topStart || BannerLocation.bottomEnd) => -1,
          (TextDirection.rtl, BannerLocation.bottomStart || BannerLocation.topEnd) => -1,
          (TextDirection.ltr, BannerLocation.bottomStart || BannerLocation.topEnd) => 1,
        };
  }
}

/// Displays a diagonal message above the corner of another widget.
///
/// Useful for showing the execution mode of an app (e.g., that asserts are
/// enabled.)
///
/// See also:
///
///  * [CheckedModeBanner], which the [WidgetsApp] widget includes by default in
///    debug mode, to show a banner that says "DEBUG".
class Banner extends StatefulWidget {
  /// Creates a banner.
  const Banner({
    super.key,
    this.child,
    required this.message,
    this.textDirection,
    required this.location,
    this.layoutDirection,
    this.color = _kColor,
    this.textStyle = _kTextStyle,
    this.shadow = _kShadow,
  });

  /// The widget to show behind the banner.
  ///
  /// {@macro flutter.widgets.ProxyWidget.child}
  final Widget? child;

  /// The message to show in the banner.
  final String message;

  /// The directionality of the text.
  ///
  /// This is used to disambiguate how to render bidirectional text. For
  /// example, if the message is an English phrase followed by a Hebrew phrase,
  /// in a [TextDirection.ltr] context the English phrase will be on the left
  /// and the Hebrew phrase to its right, while in a [TextDirection.rtl]
  /// context, the English phrase will be on the right and the Hebrew phrase on
  /// its left.
  ///
  /// Defaults to the ambient [Directionality], if any.
  ///
  /// See also:
  ///
  ///  * [layoutDirection], which controls the interpretation of the [location].
  final TextDirection? textDirection;

  /// Where to show the banner (e.g., the upper right corner).
  final BannerLocation location;

  /// The directionality of the layout.
  ///
  /// This is used to resolve the [location] values.
  ///
  /// Defaults to the ambient [Directionality], if any.
  ///
  /// See also:
  ///
  ///  * [textDirection], which controls the reading direction of the [message].
  final TextDirection? layoutDirection;

  /// The color of the banner.
  final Color color;

  /// The style of the text shown on the banner.
  final TextStyle textStyle;

  /// The shadow properties for the banner.
  ///
  /// Use a [BoxShadow] object to define the shadow's color, blur radius,
  /// and spread radius. These properties can be used to create different
  /// shadow effects.
  final BoxShadow shadow;

  @override
  State<Banner> createState() => _BannerState();
}

class _BannerState extends State<Banner> {
  BannerPainter? _painter;

  @override
  void dispose() {
    _painter?.dispose();
    super.dispose();
  }

  @override
  Widget build(BuildContext context) {
    assert(
      (widget.textDirection != null && widget.layoutDirection != null) ||
          debugCheckHasDirectionality(context),
    );

    _painter?.dispose();
    _painter = BannerPainter(
      message: widget.message,
      textDirection: widget.textDirection ?? Directionality.of(context),
      location: widget.location,
      layoutDirection: widget.layoutDirection ?? Directionality.of(context),
      color: widget.color,
      textStyle: widget.textStyle,
      shadow: widget.shadow,
    );

    return CustomPaint(foregroundPainter: _painter, child: widget.child);
  }

  @override
  void debugFillProperties(DiagnosticPropertiesBuilder properties) {
    super.debugFillProperties(properties);
    properties.add(StringProperty('message', widget.message, showName: false));
    properties.add(
      EnumProperty<TextDirection>('textDirection', widget.textDirection, defaultValue: null),
    );
    properties.add(EnumProperty<BannerLocation>('location', widget.location));
    properties.add(
      EnumProperty<TextDirection>('layoutDirection', widget.layoutDirection, defaultValue: null),
    );
    properties.add(ColorProperty('color', widget.color, showName: false));
    widget.textStyle.debugFillProperties(properties, prefix: 'text ');
  }
}

/// Displays a [Banner] saying "DEBUG" when running in debug mode.
/// [MaterialApp] builds one of these by default.
///
/// Does nothing in release mode.
class CheckedModeBanner extends StatelessWidget {
  /// Creates a const debug mode banner.
  const CheckedModeBanner({super.key, required this.child});

  /// The widget to show behind the banner.
  ///
  /// {@macro flutter.widgets.ProxyWidget.child}
  final Widget child;

  @override
  Widget build(BuildContext context) {
    Widget result = child;
    assert(() {
      result = Banner(
        message: 'DEBUG',
        textDirection: TextDirection.ltr,
        location: BannerLocation.topEnd,
        child: result,
      );
      return true;
    }());
    return result;
  }

  @override
  void debugFillProperties(DiagnosticPropertiesBuilder properties) {
    super.debugFillProperties(properties);
    String message = 'disabled';
    assert(() {
      message = '"DEBUG"';
      return true;
    }());
    properties.add(DiagnosticsNode.message(message));
  }
}
