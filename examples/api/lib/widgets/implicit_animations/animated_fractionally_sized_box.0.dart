// Copyright 2014 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

import 'package:flutter/material.dart';

/// Flutter code sample for [AnimatedFractionallySizedBox].

void main() => runApp(const AnimatedFractionallySizedBoxExampleApp());

class AnimatedFractionallySizedBoxExampleApp extends StatelessWidget {
  const AnimatedFractionallySizedBoxExampleApp({super.key});

  static const Duration duration = Duration(seconds: 1);
  static const Curve curve = Curves.fastOutSlowIn;

  @override
  Widget build(BuildContext context) {
    return MaterialApp(
      home: Scaffold(
        appBar: AppBar(title: const Text('AnimatedFractionallySizedBox Sample')),
        body: const AnimatedFractionallySizedBoxExample(duration: duration, curve: curve),
      ),
    );
  }
}

class AnimatedFractionallySizedBoxExample extends StatefulWidget {
  const AnimatedFractionallySizedBoxExample({
    required this.duration,
    required this.curve,
    super.key,
  });

  final Duration duration;

  final Curve curve;

  @override
  State<AnimatedFractionallySizedBoxExample> createState() =>
      _AnimatedFractionallySizedBoxExampleState();
}

class _AnimatedFractionallySizedBoxExampleState extends State<AnimatedFractionallySizedBoxExample> {
  bool selected = false;

  @override
  Widget build(BuildContext context) {
    return GestureDetector(
      onTap: () {
        setState(() {
          selected = !selected;
        });
      },
      child: Center(
        child: SizedBox(
          width: 200,
          height: 200,
          child: ColoredBox(
            color: Colors.red,
            child: AnimatedFractionallySizedBox(
              widthFactor: selected ? 0.25 : 0.75,
              heightFactor: selected ? 0.75 : 0.25,
              alignment: selected ? Alignment.topLeft : Alignment.bottomRight,
              duration: widget.duration,
              curve: widget.curve,
              child: const ColoredBox(color: Colors.blue, child: FlutterLogo(size: 75)),
            ),
          ),
        ),
      ),
    );
  }
}
