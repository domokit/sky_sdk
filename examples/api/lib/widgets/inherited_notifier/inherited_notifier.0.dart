// Copyright 2014 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

import 'dart:math' as math;

import 'package:flutter/material.dart';

/// Flutter code sample for [InheritedNotifier].

void main() => runApp(const InheritedNotifierExampleApp());

class InheritedNotifierExampleApp extends StatelessWidget {
  const InheritedNotifierExampleApp({super.key});

  @override
  Widget build(BuildContext context) {
    return const MaterialApp(home: InheritedNotifierExample());
  }
}

class SpinModel extends InheritedNotifier<AnimationController> {
  const SpinModel({super.key, super.notifier, required super.child});

  static double of(BuildContext context) {
    return context.dependOnInheritedWidgetOfExactType<SpinModel>()!.notifier!.value;
  }
}

class Spinner extends StatelessWidget {
  const Spinner({super.key});

  @override
  Widget build(BuildContext context) {
    return Transform.rotate(
      angle: SpinModel.of(context) * 2.0 * math.pi,
      child: Container(
        width: 100,
        height: 100,
        color: Colors.green,
        child: const Center(child: Text('Whee!')),
      ),
    );
  }
}

class InheritedNotifierExample extends StatefulWidget {
  const InheritedNotifierExample({super.key});

  @override
  State<InheritedNotifierExample> createState() => _InheritedNotifierExampleState();
}

/// [AnimationController]s can be created with `vsync: this` because of
/// [TickerProviderStateMixin].
class _InheritedNotifierExampleState extends State<InheritedNotifierExample>
    with TickerProviderStateMixin {
  late AnimationController _controller;

  @override
  void initState() {
    super.initState();
    _controller = AnimationController(duration: const Duration(seconds: 10), vsync: this)..repeat();
  }

  @override
  void dispose() {
    _controller.dispose();
    super.dispose();
  }

  @override
  Widget build(BuildContext context) {
    return SpinModel(
      notifier: _controller,
      child: const Row(
        mainAxisAlignment: MainAxisAlignment.spaceAround,
        children: <Widget>[Spinner(), Spinner(), Spinner()],
      ),
    );
  }
}
