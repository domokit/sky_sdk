// Copyright 2014 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

import 'dart:convert';

import 'package:flutter/material.dart';
import 'package:flutter_test/flutter_test.dart';
import 'package:integration_test/common.dart';
import 'package:integration_test/integration_test.dart';
import 'package:vm_service/vm_service.dart' as vm;

vm.Timeline _kTimelines = vm.Timeline(
  traceEvents: <vm.TimelineEvent>[],
  timeOriginMicros: 100,
  timeExtentMicros: 200,
);

Future<void> main() async {
  Future<Map<String, dynamic>>? request;

  group('Test Integration binding', () {
    final IntegrationTestWidgetsFlutterBinding binding =
        IntegrationTestWidgetsFlutterBinding.ensureInitialized();

    FakeVM? fakeVM;

    setUp(() {
      request = binding.callback(<String, String>{'command': 'request_data'});
      fakeVM = FakeVM(timeline: _kTimelines);
    });

    testWidgets('Run Integration app', (WidgetTester tester) async {
      runApp(const MaterialApp(home: Text('Test')));
      expect(tester.binding, binding);
      binding.reportData = <String, dynamic>{'answer': 42};
      await tester.pump();
    });

    testWidgets('hitTesting works when using setSurfaceSize', (WidgetTester tester) async {
      int invocations = 0;
      await tester.pumpWidget(
        MaterialApp(
          home: Center(
            child: GestureDetector(
              onTap: () {
                invocations++;
              },
              child: const Text('Test'),
            ),
          ),
        ),
      );

      await tester.tap(find.byType(Text));
      await tester.pump();
      expect(invocations, 1);

      await tester.binding.setSurfaceSize(const Size(200, 300));
      await tester.pump();
      await tester.tap(find.byType(Text));
      await tester.pump();
      expect(invocations, 2);

      await tester.binding.setSurfaceSize(null);
      await tester.pump();
      await tester.tap(find.byType(Text));
      await tester.pump();
      expect(invocations, 3);
    });

    testWidgets('setSurfaceSize works', (WidgetTester tester) async {
      await tester.pumpWidget(const MaterialApp(home: Center(child: Text('Test'))));

      final Size viewCenter = tester.view.physicalSize / tester.view.devicePixelRatio / 2;
      final double viewCenterX = viewCenter.width;
      final double viewCenterY = viewCenter.height;

      Offset widgetCenter = tester.getRect(find.byType(Text)).center;
      expect(widgetCenter.dx, viewCenterX);
      expect(widgetCenter.dy, viewCenterY);

      await tester.binding.setSurfaceSize(const Size(200, 300));
      await tester.pump();
      widgetCenter = tester.getRect(find.byType(Text)).center;
      expect(widgetCenter.dx, 100);
      expect(widgetCenter.dy, 150);

      await tester.binding.setSurfaceSize(null);
      await tester.pump();
      widgetCenter = tester.getRect(find.byType(Text)).center;
      expect(widgetCenter.dx, viewCenterX);
      expect(widgetCenter.dy, viewCenterY);
    });

    testWidgets('Test traceAction', (WidgetTester tester) async {
      await binding.enableTimeline(vmService: fakeVM);
      await binding.traceAction(() async {});
      expect(binding.reportData, isNotNull);
      expect(binding.reportData!.containsKey('timeline'), true);
      expect(json.encode(binding.reportData!['timeline']), json.encode(_kTimelines));
    });

    group('defaultTestTimeout', () {
      final Timeout originalTimeout = binding.defaultTestTimeout;
      tearDown(() {
        binding.defaultTestTimeout = originalTimeout;
      });

      test('can be configured', () {
        const Timeout newTimeout = Timeout(Duration(seconds: 17));
        binding.defaultTestTimeout = newTimeout;
        expect(binding.defaultTestTimeout, newTimeout);
      });
    });

    // Regression test for https://github.com/flutter/flutter/issues/66006.
    testWidgets('root view reports correct dimensions', (WidgetTester tester) async {
      await tester.pumpWidget(const Placeholder());

      expect(tester.binding.renderView.paintBounds, const Rect.fromLTWH(0, 0, 2400, 1800));
    });

    testWidgets('integration test has no label', (WidgetTester tester) async {
      expect(binding.label, null);
    });
  });

  tearDownAll(() async {
    // This part is outside the group so that `request` has been completed as
    // part of the `tearDownAll` registered in the group during
    // `IntegrationTestWidgetsFlutterBinding` initialization.
    final Map<String, dynamic> response = (await request)!['response'] as Map<String, dynamic>;
    final String message = response['message'] as String;
    final Response result = Response.fromJson(message);
    assert(result.data!['answer'] == 42);
  });
}

class FakeVM extends Fake implements vm.VmService {
  FakeVM({required this.timeline});

  vm.Timeline timeline;

  @override
  Future<vm.Timeline> getVMTimeline({int? timeOriginMicros, int? timeExtentMicros}) async {
    return timeline;
  }

  int lastTimeStamp = 0;
  @override
  Future<vm.Timestamp> getVMTimelineMicros() async {
    lastTimeStamp += 100;
    return vm.Timestamp(timestamp: lastTimeStamp);
  }

  @override
  Future<vm.Success> setVMTimelineFlags(List<String> recordedStreams) async {
    return vm.Success();
  }

  @override
  Future<vm.Success> clearVMTimeline() async {
    return vm.Success();
  }
}
