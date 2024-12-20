// Copyright 2014 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

import 'dart:math' as math;

import 'package:flutter/material.dart';

class ExampleDragTarget extends StatefulWidget {
  const ExampleDragTarget({super.key});

  @override
  ExampleDragTargetState createState() => ExampleDragTargetState();
}

class ExampleDragTargetState extends State<ExampleDragTarget> {
  Color _color = Colors.grey;

  void _handleAccept(DragTargetDetails<Color> details) {
    setState(() {
      _color = details.data;
    });
  }

  @override
  Widget build(BuildContext context) {
    return DragTarget<Color>(
      onAcceptWithDetails: _handleAccept,
      builder: (BuildContext context, List<Color?> data, List<dynamic> rejectedData) {
        return Container(
          height: 100.0,
          margin: const EdgeInsets.all(10.0),
          decoration: BoxDecoration(
            color: data.isEmpty ? _color : Colors.grey.shade200,
            border: Border.all(width: 3.0, color: data.isEmpty ? Colors.white : Colors.blue),
          ),
        );
      },
    );
  }
}

class Dot extends StatefulWidget {
  const Dot({super.key, this.color, this.size, this.child, this.tappable = false});

  final Color? color;
  final double? size;
  final Widget? child;
  final bool tappable;

  @override
  DotState createState() => DotState();
}

class DotState extends State<Dot> {
  int taps = 0;

  @override
  Widget build(BuildContext context) {
    return GestureDetector(
      onTap:
          widget.tappable
              ? () {
                setState(() {
                  taps += 1;
                });
              }
              : null,
      child: Container(
        width: widget.size,
        height: widget.size,
        decoration: BoxDecoration(
          color: widget.color,
          border: Border.all(width: taps.toDouble()),
          shape: BoxShape.circle,
        ),
        child: widget.child,
      ),
    );
  }
}

class ExampleDragSource extends StatelessWidget {
  const ExampleDragSource({
    super.key,
    this.color,
    this.heavy = false,
    this.under = true,
    this.child,
  });

  final Color? color;
  final bool heavy;
  final bool under;
  final Widget? child;

  static const double kDotSize = 50.0;
  static const double kHeavyMultiplier = 1.5;
  static const double kFingerSize = 50.0;

  @override
  Widget build(BuildContext context) {
    double size = kDotSize;
    if (heavy) {
      size *= kHeavyMultiplier;
    }

    final Widget contents = DefaultTextStyle(
      style: Theme.of(context).textTheme.bodyMedium!,
      textAlign: TextAlign.center,
      child: Dot(color: color, size: size, child: Center(child: child)),
    );

    Widget feedback = Opacity(opacity: 0.75, child: contents);

    Offset feedbackOffset;
    DragAnchorStrategy dragAnchorStrategy;
    if (!under) {
      feedback = Transform(
        transform: Matrix4.identity()..translate(-size / 2.0, -(size / 2.0 + kFingerSize)),
        child: feedback,
      );
      feedbackOffset = const Offset(0.0, -kFingerSize);
      dragAnchorStrategy = pointerDragAnchorStrategy;
    } else {
      feedbackOffset = Offset.zero;
      dragAnchorStrategy = childDragAnchorStrategy;
    }

    if (heavy) {
      return LongPressDraggable<Color>(
        data: color,
        feedback: feedback,
        feedbackOffset: feedbackOffset,
        dragAnchorStrategy: dragAnchorStrategy,
        child: contents,
      );
    } else {
      return Draggable<Color>(
        data: color,
        feedback: feedback,
        feedbackOffset: feedbackOffset,
        dragAnchorStrategy: dragAnchorStrategy,
        child: contents,
      );
    }
  }
}

class DashOutlineCirclePainter extends CustomPainter {
  const DashOutlineCirclePainter();

  static const int segments = 17;
  static const double deltaTheta = math.pi * 2 / segments; // radians
  static const double segmentArc = deltaTheta / 2.0; // radians
  static const double startOffset = 1.0; // radians

  @override
  void paint(Canvas canvas, Size size) {
    final double radius = size.shortestSide / 2.0;
    final Paint paint =
        Paint()
          ..color = const Color(0xFF000000)
          ..style = PaintingStyle.stroke
          ..strokeWidth = radius / 10.0;
    final Path path = Path();
    final Rect box = Offset.zero & size;
    for (double theta = 0.0; theta < math.pi * 2.0; theta += deltaTheta) {
      path.addArc(box, theta + startOffset, segmentArc);
    }
    canvas.drawPath(path, paint);
  }

  @override
  bool shouldRepaint(DashOutlineCirclePainter oldDelegate) => false;
}

class MovableBall extends StatelessWidget {
  const MovableBall(this.position, this.ballPosition, this.callback, {super.key});

  final int position;
  final int ballPosition;
  final ValueChanged<int> callback;

  static final GlobalKey kBallKey = GlobalKey();
  static const double kBallSize = 50.0;

  @override
  Widget build(BuildContext context) {
    final Widget ball = DefaultTextStyle(
      style: Theme.of(context).primaryTextTheme.bodyMedium!,
      textAlign: TextAlign.center,
      child: Dot(
        key: kBallKey,
        color: Colors.blue.shade700,
        size: kBallSize,
        tappable: true,
        child: const Center(child: Text('BALL')),
      ),
    );
    const Widget dashedBall = SizedBox(
      width: kBallSize,
      height: kBallSize,
      child: CustomPaint(painter: DashOutlineCirclePainter()),
    );
    if (position == ballPosition) {
      return Draggable<bool>(
        data: true,
        childWhenDragging: dashedBall,
        feedback: ball,
        maxSimultaneousDrags: 1,
        child: ball,
      );
    } else {
      return DragTarget<bool>(
        onAcceptWithDetails: (DragTargetDetails<bool> data) {
          callback(position);
        },
        builder: (BuildContext context, List<bool?> accepted, List<dynamic> rejected) {
          return dashedBall;
        },
      );
    }
  }
}

class DragAndDropApp extends StatefulWidget {
  const DragAndDropApp({super.key});

  @override
  DragAndDropAppState createState() => DragAndDropAppState();
}

class DragAndDropAppState extends State<DragAndDropApp> {
  int position = 1;

  void moveBall(int newPosition) {
    setState(() {
      position = newPosition;
    });
  }

  @override
  Widget build(BuildContext context) {
    return Scaffold(
      appBar: AppBar(title: const Text('Drag and Drop Flutter Demo')),
      body: Column(
        children: <Widget>[
          Expanded(
            child: Row(
              mainAxisAlignment: MainAxisAlignment.spaceAround,
              children: <Widget>[
                ExampleDragSource(color: Colors.yellow.shade300, child: const Text('under')),
                ExampleDragSource(
                  color: Colors.green.shade300,
                  under: false,
                  heavy: true,
                  child: const Text('long-press above'),
                ),
                ExampleDragSource(
                  color: Colors.indigo.shade300,
                  under: false,
                  child: const Text('above'),
                ),
              ],
            ),
          ),
          const Expanded(
            child: Row(
              children: <Widget>[
                Expanded(child: ExampleDragTarget()),
                Expanded(child: ExampleDragTarget()),
                Expanded(child: ExampleDragTarget()),
                Expanded(child: ExampleDragTarget()),
              ],
            ),
          ),
          Expanded(
            child: Row(
              mainAxisAlignment: MainAxisAlignment.spaceAround,
              children: <Widget>[
                MovableBall(1, position, moveBall),
                MovableBall(2, position, moveBall),
                MovableBall(3, position, moveBall),
              ],
            ),
          ),
        ],
      ),
    );
  }
}

void main() {
  runApp(const MaterialApp(title: 'Drag and Drop Flutter Demo', home: DragAndDropApp()));
}
