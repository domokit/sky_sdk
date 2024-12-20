// Copyright 2014 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

import 'dart:ui';

import 'package:flutter/scheduler.dart';
import 'package:flutter/widgets.dart';
import 'package:flutter_test/flutter_test.dart';
import 'package:leak_tracker_flutter_testing/leak_tracker_flutter_testing.dart';

void main() {
  late SchedulerBinding binding;

  setUpAll(() {
    WidgetsFlutterBinding.ensureInitialized();
    binding = SchedulerBinding.instance;
  });

  test('PerformanceModeHandler make one request', () async {
    final PerformanceModeRequestHandle? requestHandle = binding.requestPerformanceMode(
      DartPerformanceMode.latency,
    );
    expect(requestHandle, isNotNull);
    expect(binding.debugGetRequestedPerformanceMode(), equals(DartPerformanceMode.latency));
    requestHandle?.dispose();
    expect(binding.debugGetRequestedPerformanceMode(), isNull);
  });

  test('PerformanceModeHandler make conflicting requests', () async {
    final PerformanceModeRequestHandle? requestHandle1 = binding.requestPerformanceMode(
      DartPerformanceMode.latency,
    );
    expect(requestHandle1, isNotNull);

    final PerformanceModeRequestHandle? requestHandle2 = binding.requestPerformanceMode(
      DartPerformanceMode.throughput,
    );
    expect(requestHandle2, isNull);

    expect(binding.debugGetRequestedPerformanceMode(), equals(DartPerformanceMode.latency));

    requestHandle1?.dispose();
    expect(binding.debugGetRequestedPerformanceMode(), isNull);
  });

  test('PerformanceModeHandler revert only after last requestor disposed', () async {
    final PerformanceModeRequestHandle? requestHandle1 = binding.requestPerformanceMode(
      DartPerformanceMode.latency,
    );
    expect(requestHandle1, isNotNull);

    expect(binding.debugGetRequestedPerformanceMode(), equals(DartPerformanceMode.latency));

    final PerformanceModeRequestHandle? requestHandle2 = binding.requestPerformanceMode(
      DartPerformanceMode.latency,
    );
    expect(requestHandle2, isNotNull);

    expect(binding.debugGetRequestedPerformanceMode(), equals(DartPerformanceMode.latency));
    requestHandle1?.dispose();
    expect(binding.debugGetRequestedPerformanceMode(), equals(DartPerformanceMode.latency));
    requestHandle2?.dispose();
    expect(binding.debugGetRequestedPerformanceMode(), isNull);
  });

  test('PerformanceModeRequestHandle dispatches memory events', () async {
    await expectLater(
      await memoryEvents(
        () => binding.requestPerformanceMode(DartPerformanceMode.latency)!.dispose(),
        PerformanceModeRequestHandle,
      ),
      areCreateAndDispose,
    );
  });
}
