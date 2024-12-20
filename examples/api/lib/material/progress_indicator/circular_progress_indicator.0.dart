// Copyright 2014 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

import 'package:flutter/material.dart';

/// Flutter code sample for [CircularProgressIndicator].

void main() => runApp(const ProgressIndicatorExampleApp());

class ProgressIndicatorExampleApp extends StatelessWidget {
  const ProgressIndicatorExampleApp({super.key});

  @override
  Widget build(BuildContext context) {
    return const MaterialApp(home: ProgressIndicatorExample());
  }
}

class ProgressIndicatorExample extends StatefulWidget {
  const ProgressIndicatorExample({super.key});

  @override
  State<ProgressIndicatorExample> createState() => _ProgressIndicatorExampleState();
}

class _ProgressIndicatorExampleState extends State<ProgressIndicatorExample>
    with TickerProviderStateMixin {
  late AnimationController controller;
  bool year2023 = true;

  @override
  void initState() {
    super.initState();
    controller =
        AnimationController(vsync: this, duration: const Duration(seconds: 5))
          ..addListener(() {
            setState(() {});
          })
          ..repeat(reverse: true);
  }

  @override
  void dispose() {
    controller.dispose();
    super.dispose();
  }

  @override
  Widget build(BuildContext context) {
    return Scaffold(
      body: Center(
        child: Column(
          spacing: 16.0,
          mainAxisAlignment: MainAxisAlignment.center,
          children: <Widget>[
            const Text('Determinate CircularProgressIndicator'),
            Padding(
              padding: const EdgeInsets.symmetric(horizontal: 16),
              child: CircularProgressIndicator(year2023: year2023, value: controller.value),
            ),
            const Text('Indeterminate CircularProgressIndicator'),
            Padding(
              padding: const EdgeInsets.symmetric(horizontal: 16),
              child: CircularProgressIndicator(year2023: year2023),
            ),
            SwitchListTile(
              value: year2023,
              title:
                  year2023
                      ? const Text('Switch to latest M3 style')
                      : const Text('Switch to year2023 M3 style'),
              onChanged: (bool value) {
                setState(() {
                  year2023 = !year2023;
                });
              },
            ),
          ],
        ),
      ),
    );
  }
}
