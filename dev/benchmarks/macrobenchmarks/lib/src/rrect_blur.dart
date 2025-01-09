// Copyright 2014 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

import 'dart:math' show cos, sin;
import 'dart:typed_data';

import 'package:flutter/material.dart';

class RRectBlur extends StatefulWidget {
  const RRectBlur({super.key});

  @override
  State<RRectBlur> createState() => _RRectBlurPageState();
}

class _RRectBlurPageState extends State<RRectBlur> with SingleTickerProviderStateMixin {
  late final AnimationController controller;
  double tick = 0.0;

  @override
  void initState() {
    super.initState();
    controller = AnimationController(vsync: this, duration: const Duration(hours: 1));
    controller.addListener(() {
      setState(() {
        tick += 1;
      });
    });
    controller.forward(from: 0);
  }

  @override
  void dispose() {
    controller.dispose();
    super.dispose();
  }

  @override
  Widget build(BuildContext context) {
    return CustomPaint(
      size: const Size(500, 500),
      painter: PointsPainter(tick),
      child: Container(),
    );
  }
}

class PointsPainter extends CustomPainter {
  PointsPainter(this.tick);

  final double tick;

  final Float32List data = Float32List(8000);

  static const List<Color> kColors = <Color>[
    Colors.red,
    Colors.blue,
    Colors.green,
    Colors.yellow,
    Colors.orange,
    Colors.purple,
    Colors.pink,
    Colors.deepPurple,
  ];

  @override
  void paint(Canvas canvas, Size size) {
    if (size.width == 0) {
      return;
    }
    final double halfHeight = size.height / 2.0;
    const double freq = 0.25;
    const int circleCount = 40;
    for (int i = 0; i < circleCount; ++i) {
      final double radius = 25 * cos(i + (1.0 * 2.0 * 3.1415 * tick) / 60.0) + 25;
      final Paint paint =
          Paint()
            ..style = PaintingStyle.fill
            ..filterQuality = FilterQuality.low
            ..maskFilter = MaskFilter.blur(BlurStyle.normal, radius);
      final double yval = halfHeight * sin(i + (freq * 2.0 * 3.1415 * tick) / 60.0) + halfHeight;
      final double xval = (i.toDouble() / circleCount) * size.width;
      canvas.drawCircle(Offset(xval, yval), 50, paint..color = kColors[i % kColors.length]);
    }
  }

  @override
  bool shouldRepaint(covariant CustomPainter oldDelegate) {
    return true;
  }
}
