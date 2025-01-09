// Copyright 2014 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

import 'package:flutter/material.dart';

/// Flutter code sample for [StepStyle].

void main() => runApp(const StepStyleExampleApp());

class StepStyleExampleApp extends StatelessWidget {
  const StepStyleExampleApp({super.key});

  @override
  Widget build(BuildContext context) {
    return MaterialApp(
      home: Scaffold(
        appBar: AppBar(title: const Text('Step Style Example')),
        body: const Center(child: StepStyleExample()),
      ),
    );
  }
}

class StepStyleExample extends StatefulWidget {
  const StepStyleExample({super.key});

  @override
  State<StepStyleExample> createState() => _StepStyleExampleState();
}

class _StepStyleExampleState extends State<StepStyleExample> {
  final StepStyle _stepStyle = StepStyle(
    connectorThickness: 10,
    color: Colors.white,
    connectorColor: Colors.red,
    indexStyle: const TextStyle(color: Colors.black, fontSize: 20),
    border: Border.all(width: 2),
  );

  @override
  Widget build(BuildContext context) {
    return Stepper(
      type: StepperType.horizontal,
      stepIconHeight: 48,
      stepIconWidth: 48,
      stepIconMargin: EdgeInsets.zero,
      steps: <Step>[
        Step(
          title: const SizedBox.shrink(),
          content: const SizedBox.shrink(),
          isActive: true,
          stepStyle: _stepStyle,
        ),
        Step(
          title: const SizedBox.shrink(),
          content: const SizedBox.shrink(),
          isActive: true,
          stepStyle: _stepStyle.copyWith(
            connectorColor: Colors.orange,
            gradient: const LinearGradient(colors: <Color>[Colors.white, Colors.black]),
          ),
        ),
        Step(
          title: const SizedBox.shrink(),
          content: const SizedBox.shrink(),
          isActive: true,
          stepStyle: _stepStyle.copyWith(connectorColor: Colors.blue),
        ),
        Step(
          title: const SizedBox.shrink(),
          content: const SizedBox.shrink(),
          isActive: true,
          stepStyle: _stepStyle.merge(
            StepStyle(
              color: Colors.white,
              indexStyle: const TextStyle(color: Colors.black, fontSize: 20),
              border: Border.all(width: 2),
            ),
          ),
        ),
      ],
    );
  }
}
