// Copyright 2014 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

import 'package:flutter/widgets.dart';

class SpinningSquare extends StatefulWidget {
  const SpinningSquare({super.key});

  @override
  State<SpinningSquare> createState() => _SpinningSquareState();
}

class _SpinningSquareState extends State<SpinningSquare> with SingleTickerProviderStateMixin {
  // We use 3600 milliseconds instead of 1800 milliseconds because 0.0 -> 1.0
  // represents an entire turn of the square whereas in the other examples
  // we used 0.0 -> math.pi, which is only half a turn.
  late final AnimationController _animation = AnimationController(
    duration: const Duration(milliseconds: 3600),
    vsync: this,
  )..repeat();

  @override
  void dispose() {
    _animation.dispose();
    super.dispose();
  }

  @override
  Widget build(BuildContext context) {
    return RotationTransition(
      turns: _animation,
      child: Container(width: 200.0, height: 200.0, color: const Color(0xFF00FF00)),
    );
  }
}

void main() {
  runApp(const Center(child: SpinningSquare()));
}
