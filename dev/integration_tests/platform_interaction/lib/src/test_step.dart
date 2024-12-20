// Copyright 2014 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

import 'dart:async';

import 'package:flutter/material.dart';

enum TestStatus { ok, pending, failed, complete }

typedef TestStep = Future<TestStepResult> Function();

const String nothing = '-';

class TestStepResult {
  const TestStepResult(this.name, this.description, this.status);

  factory TestStepResult.fromSnapshot(AsyncSnapshot<TestStepResult> snapshot) {
    return switch (snapshot.connectionState) {
      ConnectionState.none => const TestStepResult('Not started', nothing, TestStatus.ok),
      ConnectionState.waiting => const TestStepResult('Executing', nothing, TestStatus.pending),
      ConnectionState.done => snapshot.data ?? snapshot.error! as TestStepResult,
      ConnectionState.active => throw 'Unsupported state: ConnectionState.active',
    };
  }

  final String name;
  final String description;
  final TestStatus status;

  static const TextStyle normal = TextStyle(height: 1.0);
  static const TextStyle bold = TextStyle(fontWeight: FontWeight.bold, height: 1.0);
  static const TestStepResult complete = TestStepResult(
    'Test complete',
    nothing,
    TestStatus.complete,
  );

  Widget asWidget(BuildContext context) {
    return Column(
      crossAxisAlignment: CrossAxisAlignment.start,
      children: <Widget>[
        Text('Step: $name', style: bold),
        Text(description, style: normal),
        const Text(' ', style: normal),
        Text(
          status.toString().substring('TestStatus.'.length),
          key: ValueKey<String>(status == TestStatus.pending ? 'nostatus' : 'status'),
          style: bold,
        ),
      ],
    );
  }
}
