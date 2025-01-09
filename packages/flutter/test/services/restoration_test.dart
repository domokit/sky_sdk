// Copyright 2014 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

import 'dart:async';

import 'package:flutter/scheduler.dart';
import 'package:flutter/services.dart';
import 'package:flutter/widgets.dart';
import 'package:flutter_test/flutter_test.dart';
import 'package:leak_tracker_flutter_testing/leak_tracker_flutter_testing.dart';

import 'restoration.dart';

void main() {
  testWidgets('$RestorationManager dispatches memory events', (WidgetTester tester) async {
    await expectLater(
      await memoryEvents(() => RestorationManager().dispose(), RestorationManager),
      areCreateAndDispose,
    );
  });

  group('RestorationManager', () {
    testWidgets('root bucket retrieval', (WidgetTester tester) async {
      final List<MethodCall> callsToEngine = <MethodCall>[];
      final Completer<Map<dynamic, dynamic>> result = Completer<Map<dynamic, dynamic>>();
      tester.binding.defaultBinaryMessenger.setMockMethodCallHandler(SystemChannels.restoration, (
        MethodCall call,
      ) {
        callsToEngine.add(call);
        return result.future;
      });

      final RestorationManager manager = RestorationManager();
      addTearDown(manager.dispose);
      final Future<RestorationBucket?> rootBucketFuture = manager.rootBucket;
      RestorationBucket? rootBucket;
      rootBucketFuture.then((RestorationBucket? bucket) {
        rootBucket = bucket;
      });
      expect(rootBucketFuture, isNotNull);
      expect(rootBucket, isNull);

      // Accessing rootBucket again gives same future.
      expect(manager.rootBucket, same(rootBucketFuture));

      // Engine has only been contacted once.
      expect(callsToEngine, hasLength(1));
      expect(callsToEngine.single.method, 'get');

      // Complete the engine request.
      result.complete(_createEncodedRestorationData1());
      await tester.pump();

      // Root bucket future completed.
      expect(rootBucket, isNotNull);

      // Root bucket contains the expected data.
      expect(rootBucket!.read<int>('value1'), 10);
      expect(rootBucket!.read<String>('value2'), 'Hello');
      final RestorationBucket child = rootBucket!.claimChild('child1', debugOwner: null);
      addTearDown(child.dispose);
      expect(child.read<int>('another value'), 22);

      // Accessing the root bucket again completes synchronously with same bucket.
      RestorationBucket? synchronousBucket;
      manager.rootBucket.then((RestorationBucket? bucket) {
        synchronousBucket = bucket;
      });
      expect(synchronousBucket, isNotNull);
      expect(synchronousBucket, same(rootBucket));
    });

    testWidgets('root bucket received from engine before retrieval', (WidgetTester tester) async {
      SystemChannels.restoration.setMethodCallHandler(null);
      final List<MethodCall> callsToEngine = <MethodCall>[];
      tester.binding.defaultBinaryMessenger.setMockMethodCallHandler(SystemChannels.restoration, (
        MethodCall call,
      ) async {
        callsToEngine.add(call);
        return null;
      });
      final RestorationManager manager = RestorationManager();
      addTearDown(manager.dispose);

      await _pushDataFromEngine(_createEncodedRestorationData1());

      RestorationBucket? rootBucket;
      manager.rootBucket.then((RestorationBucket? bucket) => rootBucket = bucket);
      // Root bucket is available synchronously.
      expect(rootBucket, isNotNull);
      // Engine was never asked.
      expect(callsToEngine, isEmpty);
    });

    testWidgets('root bucket received while engine retrieval is pending', (
      WidgetTester tester,
    ) async {
      SystemChannels.restoration.setMethodCallHandler(null);
      final List<MethodCall> callsToEngine = <MethodCall>[];
      final Completer<Map<dynamic, dynamic>> result = Completer<Map<dynamic, dynamic>>();
      tester.binding.defaultBinaryMessenger.setMockMethodCallHandler(SystemChannels.restoration, (
        MethodCall call,
      ) {
        callsToEngine.add(call);
        return result.future;
      });
      final RestorationManager manager = RestorationManager();
      addTearDown(manager.dispose);

      RestorationBucket? rootBucket;
      manager.rootBucket.then((RestorationBucket? bucket) => rootBucket = bucket);
      expect(rootBucket, isNull);
      expect(callsToEngine.single.method, 'get');

      await _pushDataFromEngine(_createEncodedRestorationData1());
      expect(rootBucket, isNotNull);
      expect(rootBucket!.read<int>('value1'), 10);

      result.complete(_createEncodedRestorationData2());
      await tester.pump();

      RestorationBucket? rootBucket2;
      manager.rootBucket.then((RestorationBucket? bucket) => rootBucket2 = bucket);
      expect(rootBucket2, isNotNull);
      expect(rootBucket2, same(rootBucket));
      expect(rootBucket2!.read<int>('value1'), 10);
      expect(rootBucket2!.contains('foo'), isFalse);
    });

    testWidgets('root bucket is properly replaced when new data is available', (
      WidgetTester tester,
    ) async {
      tester.binding.defaultBinaryMessenger.setMockMethodCallHandler(SystemChannels.restoration, (
        MethodCall call,
      ) async {
        return _createEncodedRestorationData1();
      });
      final RestorationManager manager = RestorationManager();
      addTearDown(manager.dispose);
      RestorationBucket? rootBucket;
      manager.rootBucket.then((RestorationBucket? bucket) {
        rootBucket = bucket;
      });
      await tester.pump();
      expect(rootBucket, isNotNull);
      expect(rootBucket!.read<int>('value1'), 10);
      final RestorationBucket child = rootBucket!.claimChild('child1', debugOwner: null);
      expect(child.read<int>('another value'), 22);

      bool rootReplaced = false;
      RestorationBucket? newRoot;
      manager.addListener(() {
        rootReplaced = true;
        manager.rootBucket.then((RestorationBucket? bucket) {
          newRoot = bucket;
        });
        // The new bucket is available synchronously.
        expect(newRoot, isNotNull);
      });

      // Send new Data.
      await _pushDataFromEngine(_createEncodedRestorationData2());

      expect(rootReplaced, isTrue);
      expect(newRoot, isNot(same(rootBucket)));

      child.dispose();

      expect(newRoot!.read<int>('foo'), 33);
      expect(newRoot!.read<int>('value1'), null);
      final RestorationBucket newChild = newRoot!.claimChild('childFoo', debugOwner: null);
      addTearDown(newChild.dispose);
      expect(newChild.read<String>('bar'), 'Hello');
    });

    testWidgets('returns null as root bucket when restoration is disabled', (
      WidgetTester tester,
    ) async {
      final List<MethodCall> callsToEngine = <MethodCall>[];
      final Completer<Map<dynamic, dynamic>> result = Completer<Map<dynamic, dynamic>>();
      tester.binding.defaultBinaryMessenger.setMockMethodCallHandler(SystemChannels.restoration, (
        MethodCall call,
      ) {
        callsToEngine.add(call);
        return result.future;
      });
      int listenerCount = 0;
      final RestorationManager manager =
          RestorationManager()..addListener(() {
            listenerCount++;
          });
      addTearDown(manager.dispose);
      RestorationBucket? rootBucket;
      bool rootBucketResolved = false;
      manager.rootBucket.then((RestorationBucket? bucket) {
        rootBucketResolved = true;
        rootBucket = bucket;
      });
      await tester.pump();
      expect(rootBucketResolved, isFalse);
      expect(listenerCount, 0);

      result.complete(_packageRestorationData(enabled: false));
      await tester.pump();
      expect(rootBucketResolved, isTrue);
      expect(rootBucket, isNull);

      // Switch to non-null.
      await _pushDataFromEngine(_createEncodedRestorationData1());
      expect(listenerCount, 1);
      manager.rootBucket.then((RestorationBucket? bucket) {
        rootBucket = bucket;
      });
      expect(rootBucket, isNotNull);

      // Switch to null again.
      await _pushDataFromEngine(_packageRestorationData(enabled: false));
      expect(listenerCount, 2);
      manager.rootBucket.then((RestorationBucket? bucket) {
        rootBucket = bucket;
      });
      expect(rootBucket, isNull);
    });

    testWidgets('flushData', (WidgetTester tester) async {
      final List<MethodCall> callsToEngine = <MethodCall>[];
      final Completer<Map<dynamic, dynamic>> result = Completer<Map<dynamic, dynamic>>();
      tester.binding.defaultBinaryMessenger.setMockMethodCallHandler(SystemChannels.restoration, (
        MethodCall call,
      ) {
        callsToEngine.add(call);
        return result.future;
      });

      final RestorationManager manager = RestorationManager();
      addTearDown(manager.dispose);
      final Future<RestorationBucket?> rootBucketFuture = manager.rootBucket;
      RestorationBucket? rootBucket;
      rootBucketFuture.then((RestorationBucket? bucket) {
        rootBucket = bucket;
      });
      result.complete(_createEncodedRestorationData1());
      await tester.pump();
      expect(rootBucket, isNotNull);
      callsToEngine.clear();

      // Schedule a frame.
      SchedulerBinding.instance.ensureVisualUpdate();
      rootBucket!.write('foo', 1);
      // flushData is no-op because frame is scheduled.
      manager.flushData();
      expect(callsToEngine, isEmpty);
      // Data is flushed at the end of the frame.
      await tester.pump();
      expect(callsToEngine, hasLength(1));
      callsToEngine.clear();

      // flushData without frame sends data directly.
      rootBucket!.write('foo', 2);
      manager.flushData();
      expect(callsToEngine, hasLength(1));
    });

    testWidgets('isReplacing', (WidgetTester tester) async {
      final Completer<Map<dynamic, dynamic>> result = Completer<Map<dynamic, dynamic>>();
      tester.binding.defaultBinaryMessenger.setMockMethodCallHandler(SystemChannels.restoration, (
        MethodCall call,
      ) {
        return result.future;
      });

      final TestRestorationManager manager = TestRestorationManager();
      addTearDown(manager.dispose);
      expect(manager.isReplacing, isFalse);

      RestorationBucket? rootBucket;
      manager.rootBucket.then((RestorationBucket? bucket) {
        rootBucket = bucket;
      });
      result.complete(_createEncodedRestorationData1());
      await tester.idle();
      expect(rootBucket, isNotNull);
      expect(rootBucket!.isReplacing, isFalse);
      expect(manager.isReplacing, isFalse);
      tester.binding.scheduleFrame();
      await tester.pump();
      expect(manager.isReplacing, isFalse);
      expect(rootBucket!.isReplacing, isFalse);

      manager.receiveDataFromEngine(enabled: true, data: null);
      RestorationBucket? rootBucket2;
      manager.rootBucket.then((RestorationBucket? bucket) {
        rootBucket2 = bucket;
      });
      expect(rootBucket2, isNotNull);
      expect(rootBucket2, isNot(same(rootBucket)));
      expect(manager.isReplacing, isTrue);
      expect(rootBucket2!.isReplacing, isTrue);
      await tester.idle();
      expect(manager.isReplacing, isTrue);
      expect(rootBucket2!.isReplacing, isTrue);
      tester.binding.scheduleFrame();
      await tester.pump();
      expect(manager.isReplacing, isFalse);
      expect(rootBucket2!.isReplacing, isFalse);

      manager.receiveDataFromEngine(enabled: false, data: null);
      RestorationBucket? rootBucket3;
      manager.rootBucket.then((RestorationBucket? bucket) {
        rootBucket3 = bucket;
      });
      expect(rootBucket3, isNull);
      expect(manager.isReplacing, isFalse);
      await tester.idle();
      expect(manager.isReplacing, isFalse);
      tester.binding.scheduleFrame();
      await tester.pump();
      expect(manager.isReplacing, isFalse);
    });
  });

  test('debugIsSerializableForRestoration', () {
    expect(debugIsSerializableForRestoration(Object()), isFalse);
    expect(debugIsSerializableForRestoration(Container()), isFalse);

    expect(debugIsSerializableForRestoration(null), isTrue);
    expect(debugIsSerializableForRestoration(147823), isTrue);
    expect(debugIsSerializableForRestoration(12.43), isTrue);
    expect(debugIsSerializableForRestoration('Hello World'), isTrue);
    expect(debugIsSerializableForRestoration(<int>[12, 13, 14]), isTrue);
    expect(debugIsSerializableForRestoration(<String, int>{'v1': 10, 'v2': 23}), isTrue);
    expect(
      debugIsSerializableForRestoration(<String, dynamic>{
        'hello': <int>[12, 12, 12],
        'world': <int, bool>{1: true, 2: false, 4: true},
      }),
      isTrue,
    );
  });
}

Future<void> _pushDataFromEngine(Map<dynamic, dynamic> data) async {
  await TestDefaultBinaryMessengerBinding.instance.defaultBinaryMessenger.handlePlatformMessage(
    'flutter/restoration',
    const StandardMethodCodec().encodeMethodCall(MethodCall('push', data)),
    (_) {},
  );
}

Map<dynamic, dynamic> _createEncodedRestorationData1() {
  final Map<String, dynamic> data = <String, dynamic>{
    valuesMapKey: <String, dynamic>{'value1': 10, 'value2': 'Hello'},
    childrenMapKey: <String, dynamic>{
      'child1': <String, dynamic>{
        valuesMapKey: <String, dynamic>{'another value': 22},
      },
    },
  };
  return _packageRestorationData(data: data);
}

Map<dynamic, dynamic> _createEncodedRestorationData2() {
  final Map<String, dynamic> data = <String, dynamic>{
    valuesMapKey: <String, dynamic>{'foo': 33},
    childrenMapKey: <String, dynamic>{
      'childFoo': <String, dynamic>{
        valuesMapKey: <String, dynamic>{'bar': 'Hello'},
      },
    },
  };
  return _packageRestorationData(data: data);
}

Map<dynamic, dynamic> _packageRestorationData({bool enabled = true, Map<dynamic, dynamic>? data}) {
  final ByteData? encoded = const StandardMessageCodec().encodeMessage(data);
  return <dynamic, dynamic>{
    'enabled': enabled,
    'data': encoded?.buffer.asUint8List(encoded.offsetInBytes, encoded.lengthInBytes),
  };
}

class TestRestorationManager extends RestorationManager {
  void receiveDataFromEngine({required bool enabled, required Uint8List? data}) {
    handleRestorationUpdateFromEngine(enabled: enabled, data: data);
  }
}
