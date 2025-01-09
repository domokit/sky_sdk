// Copyright 2014 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

import 'package:flutter/material.dart';

/// Flutter code sample for [Stepper.controlsBuilder].

void main() => runApp(const ControlsBuilderExampleApp());

class ControlsBuilderExampleApp extends StatelessWidget {
  const ControlsBuilderExampleApp({super.key});

  @override
  Widget build(BuildContext context) {
    return MaterialApp(
      home: Scaffold(
        appBar: AppBar(title: const Text('Stepper Sample')),
        body: const ControlsBuilderExample(),
      ),
    );
  }
}

class ControlsBuilderExample extends StatelessWidget {
  const ControlsBuilderExample({super.key});

  @override
  Widget build(BuildContext context) {
    return Stepper(
      controlsBuilder: (BuildContext context, ControlsDetails details) {
        return Row(
          children: <Widget>[
            TextButton(onPressed: details.onStepContinue, child: const Text('NEXT')),
            TextButton(onPressed: details.onStepCancel, child: const Text('CANCEL')),
          ],
        );
      },
      steps: const <Step>[
        Step(title: Text('A'), content: SizedBox(width: 100.0, height: 100.0)),
        Step(title: Text('B'), content: SizedBox(width: 100.0, height: 100.0)),
      ],
    );
  }
}
