// Copyright 2014 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

import 'dart:async';

import 'package:flutter/material.dart';
import 'package:flutter_driver/driver_extension.dart';

import 'src/system_navigation.dart';
import 'src/test_step.dart';

void main() {
  enableFlutterDriverExtension();
  runApp(const TestApp());
}

class TestApp extends StatefulWidget {
  const TestApp({super.key});

  @override
  State<TestApp> createState() => _TestAppState();
}

class _TestAppState extends State<TestApp> {
  static final List<TestStep> steps = <TestStep>[() => systemNavigatorPop()];
  Future<TestStepResult>? _result;
  int _step = 0;

  void _executeNextStep() {
    setState(() {
      if (_step < steps.length) {
        _result = steps[_step++]();
      } else {
        _result = Future<TestStepResult>.value(TestStepResult.complete);
      }
    });
  }

  Widget _buildTestResultWidget(BuildContext context, AsyncSnapshot<TestStepResult> snapshot) {
    return TestStepResult.fromSnapshot(snapshot).asWidget(context);
  }

  @override
  Widget build(BuildContext context) {
    return MaterialApp(
      title: 'Platform Interaction Test',
      home: Scaffold(
        appBar: AppBar(title: const Text('Platform Interaction Test')),
        body: Padding(
          padding: const EdgeInsets.all(20.0),
          child: FutureBuilder<TestStepResult>(future: _result, builder: _buildTestResultWidget),
        ),
        floatingActionButton: FloatingActionButton(
          key: const ValueKey<String>('step'),
          onPressed: _executeNextStep,
          child: const Icon(Icons.navigate_next),
        ),
      ),
    );
  }
}
