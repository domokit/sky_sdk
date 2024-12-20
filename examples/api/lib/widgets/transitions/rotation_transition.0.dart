// Copyright 2014 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

import 'package:flutter/material.dart';

/// Flutter code sample for [RotationTransition].

void main() => runApp(const RotationTransitionExampleApp());

class RotationTransitionExampleApp extends StatelessWidget {
  const RotationTransitionExampleApp({super.key});

  @override
  Widget build(BuildContext context) {
    return const MaterialApp(home: RotationTransitionExample());
  }
}

class RotationTransitionExample extends StatefulWidget {
  const RotationTransitionExample({super.key});

  @override
  State<RotationTransitionExample> createState() => _RotationTransitionExampleState();
}

/// [AnimationController]s can be created with `vsync: this` because of
/// [TickerProviderStateMixin].
class _RotationTransitionExampleState extends State<RotationTransitionExample>
    with TickerProviderStateMixin {
  late final AnimationController _controller = AnimationController(
    duration: const Duration(seconds: 2),
    vsync: this,
  )..repeat(reverse: true);
  late final Animation<double> _animation = CurvedAnimation(
    parent: _controller,
    curve: Curves.elasticOut,
  );

  @override
  void dispose() {
    _controller.dispose();
    super.dispose();
  }

  @override
  Widget build(BuildContext context) {
    return Scaffold(
      body: Center(
        child: RotationTransition(
          turns: _animation,
          child: const Padding(padding: EdgeInsets.all(8.0), child: FlutterLogo(size: 150.0)),
        ),
      ),
    );
  }
}
