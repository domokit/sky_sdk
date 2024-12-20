// Copyright 2014 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

import 'package:flutter/material.dart';

/// Flutter code sample for [AnimatedPositioned].

void main() => runApp(const AnimatedPositionedExampleApp());

class AnimatedPositionedExampleApp extends StatelessWidget {
  const AnimatedPositionedExampleApp({super.key});

  static const Duration duration = Duration(seconds: 2);
  static const Curve curve = Curves.fastOutSlowIn;

  @override
  Widget build(BuildContext context) {
    return MaterialApp(
      home: Scaffold(
        appBar: AppBar(title: const Text('AnimatedPositioned Sample')),
        body: const Center(child: AnimatedPositionedExample(duration: duration, curve: curve)),
      ),
    );
  }
}

class AnimatedPositionedExample extends StatefulWidget {
  const AnimatedPositionedExample({required this.duration, required this.curve, super.key});

  final Duration duration;

  final Curve curve;

  @override
  State<AnimatedPositionedExample> createState() => _AnimatedPositionedExampleState();
}

class _AnimatedPositionedExampleState extends State<AnimatedPositionedExample> {
  bool selected = false;

  @override
  Widget build(BuildContext context) {
    return SizedBox(
      width: 200,
      height: 350,
      child: Stack(
        children: <Widget>[
          AnimatedPositioned(
            width: selected ? 200.0 : 50.0,
            height: selected ? 50.0 : 200.0,
            top: selected ? 50.0 : 150.0,
            duration: widget.duration,
            curve: widget.curve,
            child: GestureDetector(
              onTap: () {
                setState(() {
                  selected = !selected;
                });
              },
              child: const ColoredBox(color: Colors.blue, child: Center(child: Text('Tap me'))),
            ),
          ),
        ],
      ),
    );
  }
}
