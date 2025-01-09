// Copyright 2014 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

import 'package:flutter/material.dart';

/// Flutter code sample for [MediaQueryData.systemGestureInsets].

void main() => runApp(const SystemGestureInsetsExampleApp());

class SystemGestureInsetsExampleApp extends StatelessWidget {
  const SystemGestureInsetsExampleApp({super.key});

  @override
  Widget build(BuildContext context) {
    return const MaterialApp(home: SystemGestureInsetsExample());
  }
}

class SystemGestureInsetsExample extends StatefulWidget {
  const SystemGestureInsetsExample({super.key});

  @override
  State<SystemGestureInsetsExample> createState() => _SystemGestureInsetsExampleState();
}

class _SystemGestureInsetsExampleState extends State<SystemGestureInsetsExample> {
  double _currentValue = 0.2;

  @override
  Widget build(BuildContext context) {
    final EdgeInsets systemGestureInsets = MediaQuery.of(context).systemGestureInsets;
    return Scaffold(
      appBar: AppBar(title: const Text('Pad Slider to avoid systemGestureInsets')),
      body: Padding(
        padding: EdgeInsets.only(
          // only left and right padding are needed here
          left: systemGestureInsets.left,
          right: systemGestureInsets.right,
        ),
        child: Slider(
          value: _currentValue,
          onChanged: (double newValue) {
            setState(() {
              _currentValue = newValue;
            });
          },
        ),
      ),
    );
  }
}
