// Copyright 2014 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

// This example shows how to perform a simple animation using the underlying
// render tree.

import 'dart:math' as math;

import 'package:flutter/animation.dart';
import 'package:flutter/rendering.dart';
import 'package:flutter/scheduler.dart';

import 'src/binding.dart';

class NonStopVSync implements TickerProvider {
  const NonStopVSync();
  @override
  Ticker createTicker(TickerCallback onTick) => Ticker(onTick);
}

void main() {
  // We first create a render object that represents a green box.
  final RenderBox green = RenderDecoratedBox(
    decoration: const BoxDecoration(color: Color(0xFF00FF00)),
  );
  // Second, we wrap that green box in a render object that forces the green box
  // to have a specific size.
  final RenderBox square = RenderConstrainedBox(
    additionalConstraints: const BoxConstraints.tightFor(width: 200.0, height: 200.0),
    child: green,
  );
  // Third, we wrap the sized green square in a render object that applies rotation
  // transform before painting its child. Each frame of the animation, we'll
  // update the transform of this render object to cause the green square to
  // spin.
  final RenderTransform spin = RenderTransform(
    transform: Matrix4.identity(),
    alignment: Alignment.center,
    child: square,
  );
  // Finally, we center the spinning green square...
  final RenderBox root = RenderPositionedBox(child: spin);
  // and attach it to the window.
  ViewRenderingFlutterBinding(root: root);

  // To make the square spin, we use an animation that repeats every 1800
  // milliseconds.
  final AnimationController animation = AnimationController(
    duration: const Duration(milliseconds: 1800),
    vsync: const NonStopVSync(),
  )..repeat();
  // The animation will produce a value between 0.0 and 1.0 each frame, but we
  // want to rotate the square using a value between 0.0 and math.pi. To change
  // the range of the animation, we use a Tween.
  final Tween<double> tween = Tween<double>(begin: 0.0, end: math.pi);
  // We add a listener to the animation, which will be called every time the
  // animation ticks.
  animation.addListener(() {
    // This code runs every tick of the animation and sets a new transform on
    // the "spin" render object by evaluating the tween on the current value
    // of the animation. Setting this value will mark a number of dirty bits
    // inside the render tree, which cause the render tree to repaint with the
    // new transform value this frame.
    spin.transform = Matrix4.rotationZ(tween.evaluate(animation));
  });
}
