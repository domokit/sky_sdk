// Copyright 2014 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

import 'package:flutter/material.dart';

/// Flutter code sample for [LinearProgressIndicator].

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
  bool determinate = false;

  @override
  void initState() {
    super.initState();
    controller =
        AnimationController(
            /// [AnimationController]s can be created with `vsync: this` because of
            /// [TickerProviderStateMixin].
            vsync: this,
            duration: const Duration(seconds: 2),
          )
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
      body: Padding(
        padding: const EdgeInsets.all(20.0),
        child: Column(
          spacing: 16.0,
          mainAxisAlignment: MainAxisAlignment.center,
          children: <Widget>[
            const Text('Linear progress indicator', style: TextStyle(fontSize: 20)),
            LinearProgressIndicator(
              value: determinate ? controller.value : null,
              semanticsLabel: 'Linear progress indicator',
            ),
            Row(
              children: <Widget>[
                Expanded(
                  child: Text(
                    '${determinate ? 'Determinate' : 'Indeterminate'} Mode',
                    style: Theme.of(context).textTheme.titleSmall,
                  ),
                ),
                Switch(
                  value: determinate,
                  onChanged: (bool value) {
                    setState(() {
                      determinate = value;
                      if (determinate) {
                        controller.stop();
                      } else {
                        controller
                          ..forward(from: controller.value)
                          ..repeat();
                      }
                    });
                  },
                ),
              ],
            ),
          ],
        ),
      ),
    );
  }
}
