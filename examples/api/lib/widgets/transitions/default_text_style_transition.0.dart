// Copyright 2014 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

import 'package:flutter/material.dart';

/// Flutter code sample for [DefaultTextStyleTransition].

void main() => runApp(const DefaultTextStyleTransitionExampleApp());

class DefaultTextStyleTransitionExampleApp extends StatelessWidget {
  const DefaultTextStyleTransitionExampleApp({super.key});

  @override
  Widget build(BuildContext context) {
    return const MaterialApp(home: DefaultTextStyleTransitionExample());
  }
}

class DefaultTextStyleTransitionExample extends StatefulWidget {
  const DefaultTextStyleTransitionExample({super.key});

  @override
  State<DefaultTextStyleTransitionExample> createState() =>
      _DefaultTextStyleTransitionExampleState();
}

/// [AnimationController]s can be created with `vsync: this` because of
/// [TickerProviderStateMixin].
class _DefaultTextStyleTransitionExampleState extends State<DefaultTextStyleTransitionExample>
    with TickerProviderStateMixin {
  late AnimationController _controller;
  late TextStyleTween _styleTween;
  late CurvedAnimation _curvedAnimation;

  @override
  void initState() {
    super.initState();
    _controller = AnimationController(duration: const Duration(seconds: 2), vsync: this)
      ..repeat(reverse: true);
    _styleTween = TextStyleTween(
      begin: const TextStyle(fontSize: 50, color: Colors.blue, fontWeight: FontWeight.w900),
      end: const TextStyle(fontSize: 50, color: Colors.red, fontWeight: FontWeight.w100),
    );
    _curvedAnimation = CurvedAnimation(parent: _controller, curve: Curves.elasticInOut);
  }

  @override
  void dispose() {
    _controller.dispose();
    super.dispose();
  }

  @override
  Widget build(BuildContext context) {
    return Center(
      child: DefaultTextStyleTransition(
        style: _styleTween.animate(_curvedAnimation),
        child: const Text('Flutter'),
      ),
    );
  }
}
