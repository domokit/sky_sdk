// Copyright 2014 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

import 'package:flutter/material.dart';

/// Flutter code sample for [Curve2D].

void main() => runApp(const Curve2DExampleApp());

class Curve2DExampleApp extends StatelessWidget {
  const Curve2DExampleApp({super.key});

  @override
  Widget build(BuildContext context) {
    return const MaterialApp(home: Curve2DExample());
  }
}

// This is the path that the child will follow. It's a CatmullRomSpline so
// that the coordinates can be specified that it must pass through. If the
// tension is set to 1.0, it will linearly interpolate between those points,
// instead of interpolating smoothly.
final CatmullRomSpline path = CatmullRomSpline(
  const <Offset>[
    Offset(0.05, 0.75),
    Offset(0.18, 0.23),
    Offset(0.32, 0.04),
    Offset(0.73, 0.5),
    Offset(0.42, 0.74),
    Offset(0.73, 0.01),
    Offset(0.93, 0.93),
    Offset(0.05, 0.75),
  ],
  startHandle: const Offset(0.93, 0.93),
  endHandle: const Offset(0.18, 0.23),
);

class FollowCurve2D extends StatefulWidget {
  const FollowCurve2D({
    super.key,
    required this.path,
    this.curve = Curves.easeInOut,
    required this.child,
    this.duration = const Duration(seconds: 1),
  });

  final Curve2D path;
  final Curve curve;
  final Duration duration;
  final Widget child;

  @override
  State<FollowCurve2D> createState() => _FollowCurve2DState();
}

class _FollowCurve2DState extends State<FollowCurve2D> with TickerProviderStateMixin {
  // The animation controller for this animation.
  late AnimationController controller;
  // The animation that will be used to apply the widget's animation curve.
  late Animation<double> animation;

  @override
  void initState() {
    super.initState();
    controller = AnimationController(duration: widget.duration, vsync: this);
    animation = CurvedAnimation(parent: controller, curve: widget.curve);
    // Have the controller repeat indefinitely. If you want it to "bounce" back
    // and forth, set the reverse parameter to true.
    controller.repeat();
    controller.addListener(() => setState(() {}));
  }

  @override
  void dispose() {
    // Always have to dispose of animation controllers when done.
    controller.dispose();
    super.dispose();
  }

  @override
  Widget build(BuildContext context) {
    // Scale the path values to match the -1.0 to 1.0 domain of the Alignment widget.
    final Offset position = widget.path.transform(animation.value) * 2.0 - const Offset(1.0, 1.0);
    return Align(alignment: Alignment(position.dx, position.dy), child: widget.child);
  }
}

class Curve2DExample extends StatelessWidget {
  const Curve2DExample({super.key});

  @override
  Widget build(BuildContext context) {
    return Container(
      color: Colors.white,
      alignment: Alignment.center,
      child: FollowCurve2D(
        path: path,
        duration: const Duration(seconds: 3),
        child: CircleAvatar(
          backgroundColor: Colors.yellow,
          child: DefaultTextStyle(
            style: Theme.of(context).textTheme.titleLarge!,
            child: const Text('B'), // Buzz, buzz!
          ),
        ),
      ),
    );
  }
}
