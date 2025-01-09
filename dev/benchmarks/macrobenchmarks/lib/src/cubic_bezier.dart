// Copyright 2014 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

import 'dart:math';

import 'package:flutter/material.dart';

// Based on https://github.com/eseidelGoogle/bezier_perf/blob/master/lib/main.dart
class CubicBezierPage extends StatelessWidget {
  const CubicBezierPage({super.key});

  @override
  Widget build(BuildContext context) {
    return const Center(
      child: Column(
        mainAxisAlignment: MainAxisAlignment.center,
        children: <Widget>[Bezier(Colors.amber, 1.0)],
      ),
    );
  }
}

class Bezier extends StatelessWidget {
  const Bezier(this.color, this.scale, {super.key, this.blur = 0.0, this.delay = 0.0});

  final Color color;
  final double scale;
  final double blur;
  final double delay;

  List<PathDetail> _getLogoPath() {
    final List<PathDetail> paths = <PathDetail>[];

    final Path path = Path();
    path.moveTo(100.0, 97.0);
    path.cubicTo(100.0, 97.0, 142.0, 59.0, 169.91, 41.22);
    path.cubicTo(197.82, 23.44, 249.24, 5.52, 204.67, 85.84);

    paths.add(PathDetail(path));

    // Path 2
    final Path bezier2Path = Path();
    bezier2Path.moveTo(0.0, 70.55);
    bezier2Path.cubicTo(0.0, 70.55, 42.0, 31.55, 69.91, 14.77);
    bezier2Path.cubicTo(97.82, -2.01, 149.24, -20.93, 104.37, 59.39);

    paths.add(PathDetail(bezier2Path, translate: <double>[29.45, 151.0], rotation: -1.5708));

    // Path 3
    final Path bezier3Path = Path();
    bezier3Path.moveTo(0.0, 69.48);
    bezier3Path.cubicTo(0.0, 69.48, 44.82, 27.92, 69.91, 13.7);
    bezier3Path.cubicTo(95.0, -0.52, 149.24, -22.0, 104.37, 58.32);

    paths.add(PathDetail(bezier3Path, translate: <double>[53.0, 200.48], rotation: -3.14159));

    // Path 4
    final Path bezier4Path = Path();
    bezier4Path.moveTo(0.0, 69.48);
    bezier4Path.cubicTo(0.0, 69.48, 43.82, 27.92, 69.91, 13.7);
    bezier4Path.cubicTo(96.0, -0.52, 149.24, -22.0, 104.37, 58.32);

    paths.add(PathDetail(bezier4Path, translate: <double>[122.48, 77.0], rotation: -4.71239));

    return paths;
  }

  @override
  Widget build(BuildContext context) {
    return Stack(
      children: <Widget>[
        CustomPaint(
          foregroundPainter: BezierPainter(Colors.grey, 0.0, _getLogoPath(), false),
          size: const Size(100.0, 100.0),
        ),
        AnimatedBezier(color, scale, blur: blur),
      ],
    );
  }
}

class PathDetail {
  PathDetail(this.path, {this.translate, this.rotation});

  Path path;
  List<double>? translate = <double>[];
  double? rotation;
}

class AnimatedBezier extends StatefulWidget {
  const AnimatedBezier(this.color, this.scale, {super.key, this.blur = 0.0});

  final Color color;
  final double scale;
  final double blur;

  @override
  State createState() => AnimatedBezierState();
}

class Point {
  Point(this.x, this.y);

  double x;
  double y;
}

class AnimatedBezierState extends State<AnimatedBezier> with SingleTickerProviderStateMixin {
  late AnimationController controller;
  late CurvedAnimation curve;
  bool isPlaying = false;
  List<List<Point>> pointList = <List<Point>>[<Point>[], <Point>[], <Point>[], <Point>[]];
  bool isReversed = false;

  List<PathDetail> _playForward() {
    final List<PathDetail> paths = <PathDetail>[];
    final double t = curve.value;
    final double b = controller.upperBound;
    double pX;
    double pY;

    final Path path = Path();

    if (t < b / 2) {
      pX = _getCubicPoint(t * 2, 100.0, 100.0, 142.0, 169.91);
      pY = _getCubicPoint(t * 2, 97.0, 97.0, 59.0, 41.22);
      pointList[0].add(Point(pX, pY));
    } else {
      pX = _getCubicPoint(t * 2 - b, 169.91, 197.80, 249.24, 204.67);
      pY = _getCubicPoint(t * 2 - b, 41.22, 23.44, 5.52, 85.84);
      pointList[0].add(Point(pX, pY));
    }

    path.moveTo(100.0, 97.0);

    for (final Point p in pointList[0]) {
      path.lineTo(p.x, p.y);
    }

    paths.add(PathDetail(path));

    // Path 2
    final Path bezier2Path = Path();

    if (t <= b / 2) {
      final double pX = _getCubicPoint(t * 2, 0.0, 0.0, 42.0, 69.91);
      final double pY = _getCubicPoint(t * 2, 70.55, 70.55, 31.55, 14.77);
      pointList[1].add(Point(pX, pY));
    } else {
      final double pX = _getCubicPoint(t * 2 - b, 69.91, 97.82, 149.24, 104.37);
      final double pY = _getCubicPoint(t * 2 - b, 14.77, -2.01, -20.93, 59.39);
      pointList[1].add(Point(pX, pY));
    }

    bezier2Path.moveTo(0.0, 70.55);

    for (final Point p in pointList[1]) {
      bezier2Path.lineTo(p.x, p.y);
    }

    paths.add(PathDetail(bezier2Path, translate: <double>[29.45, 151.0], rotation: -1.5708));

    // Path 3
    final Path bezier3Path = Path();
    if (t <= b / 2) {
      pX = _getCubicPoint(t * 2, 0.0, 0.0, 44.82, 69.91);
      pY = _getCubicPoint(t * 2, 69.48, 69.48, 27.92, 13.7);
      pointList[2].add(Point(pX, pY));
    } else {
      pX = _getCubicPoint(t * 2 - b, 69.91, 95.0, 149.24, 104.37);
      pY = _getCubicPoint(t * 2 - b, 13.7, -0.52, -22.0, 58.32);
      pointList[2].add(Point(pX, pY));
    }

    bezier3Path.moveTo(0.0, 69.48);

    for (final Point p in pointList[2]) {
      bezier3Path.lineTo(p.x, p.y);
    }

    paths.add(PathDetail(bezier3Path, translate: <double>[53.0, 200.48], rotation: -3.14159));

    // Path 4
    final Path bezier4Path = Path();

    if (t < b / 2) {
      final double pX = _getCubicPoint(t * 2, 0.0, 0.0, 43.82, 69.91);
      final double pY = _getCubicPoint(t * 2, 69.48, 69.48, 27.92, 13.7);
      pointList[3].add(Point(pX, pY));
    } else {
      final double pX = _getCubicPoint(t * 2 - b, 69.91, 96.0, 149.24, 104.37);
      final double pY = _getCubicPoint(t * 2 - b, 13.7, -0.52, -22.0, 58.32);
      pointList[3].add(Point(pX, pY));
    }

    bezier4Path.moveTo(0.0, 69.48);

    for (final Point p in pointList[3]) {
      bezier4Path.lineTo(p.x, p.y);
    }

    paths.add(PathDetail(bezier4Path, translate: <double>[122.48, 77.0], rotation: -4.71239));

    return paths;
  }

  List<PathDetail> _playReversed() {
    for (final List<Point> list in pointList) {
      if (list.isNotEmpty) {
        list.removeLast();
      }
    }

    final List<Point> points = pointList[0];
    final Path path = Path();

    path.moveTo(100.0, 97.0);

    for (final Point point in points) {
      path.lineTo(point.x, point.y);
    }

    final Path bezier2Path = Path();

    bezier2Path.moveTo(0.0, 70.55);

    for (final Point p in pointList[1]) {
      bezier2Path.lineTo(p.x, p.y);
    }

    final Path bezier3Path = Path();
    bezier3Path.moveTo(0.0, 69.48);

    for (final Point p in pointList[2]) {
      bezier3Path.lineTo(p.x, p.y);
    }

    final Path bezier4Path = Path();

    bezier4Path.moveTo(0.0, 69.48);

    for (final Point p in pointList[3]) {
      bezier4Path.lineTo(p.x, p.y);
    }

    return <PathDetail>[
      PathDetail(path),
      PathDetail(bezier2Path, translate: <double>[29.45, 151.0], rotation: -1.5708),
      PathDetail(bezier3Path, translate: <double>[53.0, 200.48], rotation: -3.14159),
      PathDetail(bezier4Path, translate: <double>[122.48, 77.0], rotation: -4.71239),
    ];
  }

  List<PathDetail> _getLogoPath() {
    if (!isReversed) {
      return _playForward();
    }

    return _playReversed();
  }

  //From http://wiki.roblox.com/index.php?title=File:Beziereq4.png
  double _getCubicPoint(double t, double p0, double p1, double p2, double p3) {
    return (pow(1 - t, 3) as double) * p0 +
        3 * pow(1 - t, 2) * t * p1 +
        3 * (1 - t) * pow(t, 2) * p2 +
        pow(t, 3) * p3;
  }

  void playAnimation() {
    isPlaying = true;
    isReversed = false;
    for (final List<Point> list in pointList) {
      list.clear();
    }
    controller.reset();
    controller.forward();
  }

  void stopAnimation() {
    isPlaying = false;
    controller.stop();
    for (final List<Point> list in pointList) {
      list.clear();
    }
  }

  void reverseAnimation() {
    isReversed = true;
    controller.reverse();
  }

  @override
  void initState() {
    super.initState();
    controller = AnimationController(vsync: this, duration: const Duration(milliseconds: 1000));
    // Animations are typically implemented using the AnimatedBuilder widget.
    // This code uses a manual listener for historical reasons and will remain
    // in order to preserve compatibility with the history of measurements for
    // this benchmark.
    curve =
        CurvedAnimation(parent: controller, curve: Curves.linear)
          ..addListener(() {
            setState(() {});
          })
          ..addStatusListener((AnimationStatus status) {
            if (status.isCompleted) {
              reverseAnimation();
            } else if (status.isDismissed) {
              playAnimation();
            }
          });

    playAnimation();
  }

  @override
  void dispose() {
    controller.dispose();
    super.dispose();
  }

  @override
  Widget build(BuildContext context) {
    return CustomPaint(
      foregroundPainter: BezierPainter(
        widget.color,
        curve.value * widget.blur,
        _getLogoPath(),
        isPlaying,
      ),
      size: const Size(100.0, 100.0),
    );
  }
}

class BezierPainter extends CustomPainter {
  BezierPainter(this.color, this.blur, this.path, this.isPlaying);

  Color color;
  final double blur;
  List<PathDetail> path;
  bool isPlaying;

  @override
  void paint(Canvas canvas, Size size) {
    final Paint paint = Paint();
    paint.strokeWidth = 18.0;
    paint.style = PaintingStyle.stroke;
    paint.strokeCap = StrokeCap.round;
    paint.color = color;
    canvas.scale(0.5, 0.5);

    for (int i = 0; i < path.length; i++) {
      if (path[i].translate != null) {
        canvas.translate(path[i].translate![0], path[i].translate![1]);
      }

      if (path[i].rotation != null) {
        canvas.rotate(path[i].rotation!);
      }

      if (blur > 0) {
        final MaskFilter blur = MaskFilter.blur(BlurStyle.normal, this.blur);
        paint.maskFilter = blur;
        canvas.drawPath(path[i].path, paint);
      }

      paint.maskFilter = null;
      canvas.drawPath(path[i].path, paint);
    }
  }

  @override
  bool shouldRepaint(BezierPainter oldDelegate) => true;

  @override
  bool shouldRebuildSemantics(BezierPainter oldDelegate) => false;
}
