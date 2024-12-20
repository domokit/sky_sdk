// Copyright 2014 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

import 'package:flutter/material.dart';

class _MultiplyPainter extends CustomPainter {
  _MultiplyPainter(this._color);

  final Color _color;

  @override
  void paint(Canvas canvas, Size size) {
    const int xDenominator = 2;
    const int yDenominator = 10;
    final double width = size.width / xDenominator;
    final double height = size.height / yDenominator;

    for (int y = 0; y < yDenominator; y++) {
      for (int x = 0; x < xDenominator; x++) {
        final Rect rect = Offset(x * width, y * height) & Size(width, height);
        final Paint basePaint =
            Paint()
              ..color = Color.fromARGB(
                (((x + 1) * width) / size.width * 255.0).floor(),
                (((y + 1) * height) / size.height * 255.0).floor(),
                255,
                127,
              );
        canvas.drawRect(rect, basePaint);

        final Paint multiplyPaint =
            Paint()
              ..color = _color
              ..blendMode = BlendMode.multiply;
        canvas.drawRect(rect, multiplyPaint);
      }
    }
  }

  @override
  bool shouldRepaint(CustomPainter oldDelegate) {
    return true;
  }
}

class AnimatedAdvancedBlend extends StatefulWidget {
  const AnimatedAdvancedBlend({super.key});

  @override
  State<AnimatedAdvancedBlend> createState() => _AnimatedAdvancedBlendState();
}

class _AnimatedAdvancedBlendState extends State<AnimatedAdvancedBlend>
    with SingleTickerProviderStateMixin {
  late final AnimationController controller = AnimationController(
    vsync: this,
    duration: const Duration(milliseconds: 5000),
  );
  late final Animation<double> animation = controller.drive(Tween<double>(begin: 0.0, end: 1.0));
  Color _color = const Color.fromARGB(255, 255, 0, 255);

  @override
  void initState() {
    super.initState();
    controller.repeat();
    animation.addListener(() {
      setState(() {
        _color = Color.fromARGB((animation.value * 255).floor(), 255, 0, 255);
      });
    });
  }

  @override
  void dispose() {
    controller.dispose();
    super.dispose();
  }

  @override
  Widget build(BuildContext context) {
    return MaterialApp(
      home: Scaffold(body: CustomPaint(painter: _MultiplyPainter(_color), child: Container())),
    );
  }
}
