// Copyright 2014 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

import 'dart:math';
import 'dart:ui' as ui;

import 'package:flutter/material.dart';

import '../common.dart';

Map<String, WidgetBuilder> gradientPerfRoutes = <String, WidgetBuilder>{
  kGradientPerfRecreateDynamicRouteName: (BuildContext _) => const RecreateDynamicPainterPage(),
  kGradientPerfRecreateConsistentRouteName:
      (BuildContext _) => const RecreateConsistentPainterPage(),
  kGradientPerfStaticConsistentRouteName: (BuildContext _) => const StaticConsistentPainterPage(),
};

typedef CustomPaintFactory = CustomPainter Function(double hue);

class GradientPerfHomePage extends StatelessWidget {
  const GradientPerfHomePage({super.key});

  @override
  Widget build(BuildContext context) {
    return Scaffold(
      appBar: AppBar(title: const Text('Gradient Perf')),
      body: ListView(
        key: const Key(kGradientPerfScrollableName),
        children: <Widget>[
          ElevatedButton(
            key: const Key(kGradientPerfRecreateDynamicRouteName),
            child: const Text('Recreate Dynamic Gradients'),
            onPressed: () {
              Navigator.pushNamed(context, kGradientPerfRecreateDynamicRouteName);
            },
          ),
          ElevatedButton(
            key: const Key(kGradientPerfRecreateConsistentRouteName),
            child: const Text('Recreate Same Gradients'),
            onPressed: () {
              Navigator.pushNamed(context, kGradientPerfRecreateConsistentRouteName);
            },
          ),
          ElevatedButton(
            key: const Key(kGradientPerfStaticConsistentRouteName),
            child: const Text('Static Gradients'),
            onPressed: () {
              Navigator.pushNamed(context, kGradientPerfStaticConsistentRouteName);
            },
          ),
        ],
      ),
    );
  }
}

class _PainterPage extends StatefulWidget {
  const _PainterPage({super.key, required this.title, required this.factory});

  final String title;
  final CustomPaintFactory factory;

  @override
  State<_PainterPage> createState() => _PainterPageState();
}

class RecreateDynamicPainterPage extends _PainterPage {
  const RecreateDynamicPainterPage({super.key})
    : super(title: 'Recreate Dynamic Gradients', factory: makePainter);

  static CustomPainter makePainter(double f) {
    return RecreatedDynamicGradients(baseFactor: f);
  }
}

class RecreateConsistentPainterPage extends _PainterPage {
  const RecreateConsistentPainterPage({super.key})
    : super(title: 'Recreate Same Gradients', factory: makePainter);

  static CustomPainter makePainter(double f) {
    return RecreatedConsistentGradients(baseFactor: f);
  }
}

class StaticConsistentPainterPage extends _PainterPage {
  const StaticConsistentPainterPage({super.key})
    : super(title: 'Reuse Same Gradients', factory: makePainter);

  static CustomPainter makePainter(double f) {
    return StaticConsistentGradients(baseFactor: f);
  }
}

class _PainterPageState extends State<_PainterPage> with SingleTickerProviderStateMixin {
  late AnimationController _controller;

  @override
  void initState() {
    super.initState();
    _controller = AnimationController(vsync: this);
    _controller.repeat(period: const Duration(seconds: 2));
  }

  @override
  void dispose() {
    _controller.dispose();
    super.dispose();
  }

  @override
  Widget build(BuildContext context) {
    return Scaffold(
      appBar: AppBar(title: Text(widget.title)),
      body: Center(
        child: AnimatedBuilder(
          animation: _controller,
          builder: (BuildContext context, Widget? child) {
            return CustomPaint(
              size: const Size(paintW, paintH),
              painter: widget.factory(_controller.value),
              willChange: true,
            );
          },
        ),
      ),
    );
  }
}

Color color(double factor) {
  int v = ((factor * 255 * 3) % (255 * 3)).round();
  if (v < 0) {
    v += 255 * 3;
  }
  int r = 0;
  int g = 0;
  int b = 0;
  if (v < 255) {
    r = 255 - v;
    g = v;
  } else {
    v -= 255;
    if (v < 255) {
      g = 255 - v;
      b = v;
    } else {
      v -= 255;
      b = 255 - v;
      r = v;
    }
  }
  return Color.fromARGB(255, r, g, b);
}

Shader rotatingGradient(double factor, double x, double y, double h) {
  final double s = sin(factor * 2 * pi) * h / 8;
  final double c = cos(factor * 2 * pi) * h / 8;
  final double cx = x;
  final double cy = y + h / 2;
  final Offset p0 = Offset(cx + s, cy + c);
  final Offset p1 = Offset(cx - s, cy - c);
  return ui.Gradient.linear(p0, p1, <Color>[color(factor), color(factor + 0.5)]);
}

const int nAcross = 12;
const int nDown = 16;
const double cellW = 20;
const double cellH = 20;
const double hGap = 5;
const double vGap = 5;
const double paintW = hGap + (cellW + hGap) * nAcross;
const double paintH = vGap + (cellH + vGap) * nDown;

double x(int i, int j) {
  return hGap + i * (cellW + hGap);
}

double y(int i, int j) {
  return vGap + j * (cellH + vGap);
}

Shader gradient(double baseFactor, int i, int j) {
  final double lineFactor = baseFactor + 1 / 3 + 0.5 * (j + 1) / (nDown + 1);
  final double cellFactor = lineFactor + 1 / 3 * (i + 1) / (nAcross + 1);
  return rotatingGradient(cellFactor, x(i, j) + cellW / 2, y(i, j), cellH);
}

class RecreatedDynamicGradients extends CustomPainter {
  RecreatedDynamicGradients({required this.baseFactor});

  final double baseFactor;

  @override
  void paint(Canvas canvas, Size size) {
    final Paint p = Paint();
    p.color = color(baseFactor);
    canvas.drawRect(Offset.zero & size, p);
    for (int j = 0; j < nDown; j++) {
      for (int i = 0; i < nAcross; i++) {
        p.shader = gradient(baseFactor, i, j);
        canvas.drawRect(Rect.fromLTWH(x(i, j), y(i, j), cellW, cellH), p);
      }
    }
  }

  @override
  bool shouldRepaint(CustomPainter oldDelegate) => true;
}

class RecreatedConsistentGradients extends CustomPainter {
  RecreatedConsistentGradients({required this.baseFactor});

  final double baseFactor;

  @override
  void paint(Canvas canvas, Size size) {
    final Paint p = Paint();
    p.color = color(baseFactor);
    canvas.drawRect(Offset.zero & size, p);
    for (int j = 0; j < nDown; j++) {
      for (int i = 0; i < nAcross; i++) {
        p.shader = gradient(0, i, j);
        canvas.drawRect(Rect.fromLTWH(x(i, j), y(i, j), cellW, cellH), p);
      }
    }
  }

  @override
  bool shouldRepaint(CustomPainter oldDelegate) => true;
}

class StaticConsistentGradients extends CustomPainter {
  StaticConsistentGradients({required this.baseFactor});

  final double baseFactor;

  static List<List<Shader>> gradients = <List<Shader>>[
    for (int j = 0; j < nDown; j++) <Shader>[for (int i = 0; i < nAcross; i++) gradient(0, i, j)],
  ];

  @override
  void paint(Canvas canvas, Size size) {
    final Paint p = Paint();
    p.color = color(baseFactor);
    canvas.drawRect(Offset.zero & size, p);
    for (int j = 0; j < nDown; j++) {
      for (int i = 0; i < nAcross; i++) {
        p.shader = gradients[j][i];
        canvas.drawRect(Rect.fromLTWH(x(i, j), y(i, j), cellW, cellH), p);
      }
    }
  }

  @override
  bool shouldRepaint(CustomPainter oldDelegate) => true;
}
