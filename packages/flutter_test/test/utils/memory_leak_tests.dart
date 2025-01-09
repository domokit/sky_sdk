// Copyright 2014 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

import 'package:flutter/cupertino.dart';
import 'package:flutter/material.dart';
import 'package:flutter_test/flutter_test.dart';
import 'package:leak_tracker_flutter_testing/leak_tracker_flutter_testing.dart';

/// Objects that should not be GCed during test run.
final List<InstrumentedDisposable> _retainer = <InstrumentedDisposable>[];

/// Test cases for memory leaks.
///
/// They are separate from test execution to allow
/// excluding them from test helpers.
final List<LeakTestCase> memoryLeakTests = <LeakTestCase>[
  LeakTestCase(
    name: 'no leaks',
    body: (PumpWidgetsCallback? pumpWidgets, RunAsyncCallback<dynamic>? runAsync) async {
      await pumpWidgets!(Container());
    },
  ),
  LeakTestCase(
    name: 'not disposed disposable',
    body: (PumpWidgetsCallback? pumpWidgets, RunAsyncCallback<dynamic>? runAsync) async {
      InstrumentedDisposable();
    },
    notDisposedTotal: 1,
  ),
  LeakTestCase(
    name: 'not GCed disposable',
    body: (PumpWidgetsCallback? pumpWidgets, RunAsyncCallback<dynamic>? runAsync) async {
      _retainer.add(InstrumentedDisposable()..dispose());
    },
    notGCedTotal: 1,
  ),
  LeakTestCase(
    name: 'leaking widget',
    body: (PumpWidgetsCallback? pumpWidgets, RunAsyncCallback<dynamic>? runAsync) async {
      StatelessLeakingWidget();
    },
    notDisposedTotal: 1,
    notGCedTotal: 1,
  ),
  LeakTestCase(
    name: 'dispose in tear down',
    body: (PumpWidgetsCallback? pumpWidgets, RunAsyncCallback<dynamic>? runAsync) async {
      final InstrumentedDisposable myClass = InstrumentedDisposable();
      addTearDown(myClass.dispose);
    },
  ),
  LeakTestCase(
    name: 'pumped leaking widget',
    body: (PumpWidgetsCallback? pumpWidgets, RunAsyncCallback<dynamic>? runAsync) async {
      await pumpWidgets!(StatelessLeakingWidget());
    },
    notDisposedTotal: 1,
    notGCedTotal: 1,
  ),
  LeakTestCase(
    name: 'leaking widget in runAsync',
    body: (PumpWidgetsCallback? pumpWidgets, RunAsyncCallback<dynamic>? runAsync) async {
      await runAsync!(() async {
        StatelessLeakingWidget();
      });
    },
    notDisposedTotal: 1,
    notGCedTotal: 1,
  ),
  LeakTestCase(
    name: 'pumped in runAsync',
    body: (PumpWidgetsCallback? pumpWidgets, RunAsyncCallback<dynamic>? runAsync) async {
      await runAsync!(() async {
        await pumpWidgets!(StatelessLeakingWidget());
      });
    },
    notDisposedTotal: 1,
    notGCedTotal: 1,
  ),
];

String memoryLeakTestsFilePath() {
  return RegExp(r'(\/[^\/]*.dart):').firstMatch(StackTrace.current.toString())!.group(1).toString();
}
