// Copyright 2014 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

import 'dart:typed_data';
import 'dart:ui';

import 'package:flutter/material.dart';

class DrawPointsPage extends StatefulWidget {
  const DrawPointsPage({super.key});

  @override
  State<DrawPointsPage> createState() => _DrawPointsPageState();
}

class _DrawPointsPageState extends State<DrawPointsPage> with SingleTickerProviderStateMixin {
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
    canvas.drawPaint(Paint()..color = Colors.white);
    for (int i = 0; i < 8; i++) {
      final double x = ((size.width / (i + 1)) + tick) % size.width;
      for (int j = 0; j < data.length; j += 2) {
        data[j] = x;
        data[j + 1] = (size.height / (j + 1)) + 200;
      }
      final Paint paint =
          Paint()
            ..color = kColors[i]
            ..strokeWidth = 5
            ..strokeCap = StrokeCap.round
            ..style = PaintingStyle.stroke;
      canvas.drawRawPoints(PointMode.points, data, paint);
    }
  }

  @override
  bool shouldRepaint(covariant CustomPainter oldDelegate) {
    return true;
  }
}
