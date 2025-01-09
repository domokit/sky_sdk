// Copyright 2014 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

import 'dart:math';
import 'package:flutter/material.dart';
import 'metrics.dart';

class SettingsIcon extends StatelessWidget {
  const SettingsIcon(this.time, {super.key});

  final double time;

  @override
  Widget build(BuildContext context) {
    return CustomPaint(painter: _SettingsIconPainter(time: time, context: context));
  }
}

class _SettingsIconPainter extends CustomPainter {
  _SettingsIconPainter({required this.time, required this.context});

  final double time;
  final BuildContext context;

  late Offset _center;
  late double _scaling;
  late Canvas _canvas;

  /// Computes [_center] and [_scaling], parameters used to convert offsets
  /// and lengths in relative units into logical pixels.
  ///
  /// The icon is aligned to the bottom-start corner.
  void _computeCenterAndScaling(Size size) {
    _scaling = min(size.width / unitWidth, size.height / unitHeight);
    final double dx = switch (Directionality.of(context)) {
      TextDirection.rtl => size.width - unitWidth * _scaling / 2,
      TextDirection.ltr => unitWidth * _scaling / 2,
    };
    _center = Offset(dx, size.height - unitHeight * _scaling / 2);
  }

  /// Transforms an offset in relative units into an offset in logical pixels.
  Offset _transform(Offset offset) {
    return _center + offset * _scaling;
  }

  /// Transforms a length in relative units into a dimension in logical pixels.
  double _size(double length) {
    return length * _scaling;
  }

  /// A rectangle with a fixed location, used to locate gradients.
  Rect get _fixedRect {
    final Offset topLeft = Offset(-_size(stickLength / 2), -_size(stickWidth / 2));
    final Offset bottomRight = Offset(_size(stickLength / 2), _size(stickWidth / 2));
    return Rect.fromPoints(topLeft, bottomRight);
  }

  /// Black or white paint, depending on brightness.
  Paint get _monoPaint {
    final Color monoColor =
        Theme.of(context).colorScheme.brightness == Brightness.light ? Colors.black : Colors.white;
    return Paint()..color = monoColor;
  }

  /// Pink paint with horizontal gradient.
  Paint get _pinkPaint {
    const LinearGradient shader = LinearGradient(colors: <Color>[pinkLeft, pinkRight]);
    final Rect shaderRect = _fixedRect.translate(_size(-(stickLength - colorLength(time)) / 2), 0);

    return Paint()..shader = shader.createShader(shaderRect);
  }

  /// Teal paint with horizontal gradient.
  Paint get _tealPaint {
    const LinearGradient shader = LinearGradient(colors: <Color>[tealLeft, tealRight]);
    final Rect shaderRect = _fixedRect.translate(_size((stickLength - colorLength(time)) / 2), 0);

    return Paint()..shader = shader.createShader(shaderRect);
  }

  /// Paints a stadium-shaped stick.
  void _paintStick({
    required Offset center,
    required double length,
    required double width,
    double angle = 0,
    required Paint paint,
  }) {
    // Convert to pixels.
    center = _transform(center);
    length = _size(length);
    width = _size(width);

    // Paint.
    width = min(width, length);
    final double stretch = length / 2;
    final double radius = width / 2;

    _canvas.save();

    _canvas.translate(center.dx, center.dy);
    _canvas.rotate(angle);

    final Rect leftOval = Rect.fromCircle(center: Offset(-stretch + radius, 0), radius: radius);

    final Rect rightOval = Rect.fromCircle(center: Offset(stretch - radius, 0), radius: radius);

    _canvas.drawPath(
      Path()
        ..arcTo(leftOval, pi / 2, pi, false)
        ..arcTo(rightOval, -pi / 2, pi, false),
      paint,
    );

    _canvas.restore();
  }

  @override
  void paint(Canvas canvas, Size size) {
    _computeCenterAndScaling(size);
    _canvas = canvas;

    if (isTransitionPhase(time)) {
      _paintStick(
        center: upperColorOffset(time),
        length: colorLength(time),
        width: stickWidth,
        paint: _pinkPaint,
      );

      _paintStick(
        center: lowerColorOffset(time),
        length: colorLength(time),
        width: stickWidth,
        paint: _tealPaint,
      );

      _paintStick(
        center: upperMonoOffset(time),
        length: monoLength(time),
        width: knobDiameter,
        paint: _monoPaint,
      );

      _paintStick(
        center: lowerMonoOffset(time),
        length: monoLength(time),
        width: knobDiameter,
        paint: _monoPaint,
      );
    } else {
      _paintStick(
        center: upperKnobCenter,
        length: stickLength,
        width: knobDiameter,
        angle: -knobRotation(time),
        paint: _monoPaint,
      );

      _paintStick(
        center: knobCenter(time),
        length: stickLength,
        width: knobDiameter,
        angle: knobRotation(time),
        paint: _monoPaint,
      );
    }
  }

  @override
  bool shouldRepaint(CustomPainter oldDelegate) =>
      oldDelegate is! _SettingsIconPainter || oldDelegate.time != time;
}
