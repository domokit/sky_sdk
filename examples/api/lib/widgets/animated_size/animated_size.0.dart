// Copyright 2014 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

import 'package:flutter/material.dart';

/// Flutter code sample for [AnimatedSize].

void main() => runApp(const AnimatedSizeExampleApp());

class AnimatedSizeExampleApp extends StatelessWidget {
  const AnimatedSizeExampleApp({super.key});

  static const Duration duration = Duration(seconds: 1);
  static const Curve curve = Curves.easeIn;

  @override
  Widget build(BuildContext context) {
    return MaterialApp(
      home: Scaffold(
        appBar: AppBar(title: const Text('AnimatedSize Sample')),
        body: const Center(child: AnimatedSizeExample(duration: duration, curve: curve)),
      ),
    );
  }
}

class AnimatedSizeExample extends StatefulWidget {
  const AnimatedSizeExample({required this.duration, required this.curve, super.key});

  final Duration duration;

  final Curve curve;

  @override
  State<AnimatedSizeExample> createState() => _AnimatedSizeExampleState();
}

class _AnimatedSizeExampleState extends State<AnimatedSizeExample> {
  bool _isSelected = false;

  @override
  Widget build(BuildContext context) {
    return GestureDetector(
      onTap: () {
        setState(() {
          _isSelected = !_isSelected;
        });
      },
      child: ColoredBox(
        color: Colors.amberAccent,
        child: AnimatedSize(
          duration: widget.duration,
          curve: widget.curve,
          child: SizedBox.square(
            dimension: _isSelected ? 250.0 : 100.0,
            child: const Center(child: FlutterLogo(size: 75.0)),
          ),
        ),
      ),
    );
  }
}
