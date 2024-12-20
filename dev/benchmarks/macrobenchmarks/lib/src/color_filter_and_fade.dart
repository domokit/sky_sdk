// Copyright 2014 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

import 'dart:ui' as ui;

import 'package:flutter/material.dart';

// This tests whether the Opacity layer raster cache works with color filters.
// See https://github.com/flutter/flutter/issues/51975.
class ColorFilterAndFadePage extends StatefulWidget {
  const ColorFilterAndFadePage({super.key});

  @override
  State<ColorFilterAndFadePage> createState() => _ColorFilterAndFadePageState();
}

class _ColorFilterAndFadePageState extends State<ColorFilterAndFadePage>
    with TickerProviderStateMixin {
  @override
  Widget build(BuildContext context) {
    final Widget shadowWidget = _ShadowWidget(
      width: 24,
      height: 24,
      useColorFilter: _useColorFilter,
      shadow: const ui.Shadow(color: Colors.black45, offset: Offset(0.0, 2.0), blurRadius: 4.0),
    );

    final Widget row = Row(
      mainAxisAlignment: MainAxisAlignment.center,
      children: <Widget>[
        shadowWidget,
        const SizedBox(width: 12),
        shadowWidget,
        const SizedBox(width: 12),
        shadowWidget,
        const SizedBox(width: 12),
        shadowWidget,
        const SizedBox(width: 12),
        shadowWidget,
        const SizedBox(width: 12),
      ],
    );

    final Widget column = Column(
      mainAxisAlignment: MainAxisAlignment.center,
      children: <Widget>[
        row,
        const SizedBox(height: 12),
        row,
        const SizedBox(height: 12),
        row,
        const SizedBox(height: 12),
        row,
        const SizedBox(height: 12),
      ],
    );

    final Widget fadeTransition = FadeTransition(
      opacity: _opacityAnimation,
      // This RepaintBoundary is necessary to not let the opacity change
      // invalidate the layer raster cache below. This is necessary with
      // or without the color filter.
      child: RepaintBoundary(child: column),
    );

    return Scaffold(
      backgroundColor: Colors.lightBlue,
      body: Center(
        child: Column(
          mainAxisSize: MainAxisSize.min,
          children: <Widget>[
            fadeTransition,
            Container(height: 20),
            const Text('Use Color Filter:'),
            Checkbox(
              value: _useColorFilter,
              onChanged: (bool? value) {
                setState(() {
                  _useColorFilter = value ?? false;
                });
              },
            ),
          ],
        ),
      ),
    );
  }

  // Create a looping fade-in fade-out animation for opacity.
  void _initAnimation() {
    _controller = AnimationController(duration: const Duration(seconds: 3), vsync: this);
    _opacityAnimation = Tween<double>(begin: 0.0, end: 1.0).animate(_controller);
    _opacityAnimation.addStatusListener((AnimationStatus status) {
      if (status == AnimationStatus.completed) {
        _controller.reverse();
      } else if (status == AnimationStatus.dismissed) {
        _controller.forward();
      }
    });
    _controller.forward();
  }

  @override
  void initState() {
    super.initState();
    _initAnimation();
  }

  @override
  void dispose() {
    _controller.dispose();
    super.dispose();
  }

  late AnimationController _controller;
  late Animation<double> _opacityAnimation;
  bool _useColorFilter = true;
}

class _ShadowWidget extends StatelessWidget {
  const _ShadowWidget({
    required this.width,
    required this.height,
    required this.useColorFilter,
    required this.shadow,
  });

  final double width;
  final double height;
  final bool useColorFilter;
  final Shadow shadow;

  @override
  Widget build(BuildContext context) {
    return SizedBox(
      width: width,
      height: height,
      child: CustomPaint(
        painter: _ShadowPainter(useColorFilter: useColorFilter, shadow: shadow),
        size: Size(width, height),
        isComplex: true,
      ),
    );
  }
}

class _ShadowPainter extends CustomPainter {
  const _ShadowPainter({required this.useColorFilter, required this.shadow});

  final bool useColorFilter;
  final Shadow shadow;

  @override
  void paint(Canvas canvas, Size size) {
    final Rect rect = Offset.zero & size;

    final Paint paint = Paint();
    if (useColorFilter) {
      paint.colorFilter = ColorFilter.mode(shadow.color, BlendMode.srcIn);
    }

    canvas.saveLayer(null, paint);
    canvas.translate(shadow.offset.dx, shadow.offset.dy);
    canvas.drawRect(rect, Paint());
    canvas.drawRect(
      rect,
      Paint()..maskFilter = MaskFilter.blur(BlurStyle.normal, shadow.blurSigma),
    );
    canvas.restore();

    canvas.drawRect(rect, Paint()..color = useColorFilter ? Colors.white : Colors.black);
  }

  @override
  bool shouldRepaint(_ShadowPainter oldDelegate) => oldDelegate.useColorFilter != useColorFilter;
}
