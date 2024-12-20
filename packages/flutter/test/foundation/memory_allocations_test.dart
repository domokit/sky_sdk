// Copyright 2014 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

import 'dart:ui';

import 'package:flutter/foundation.dart';
import 'package:flutter_test/flutter_test.dart';
import 'package:leak_tracker_flutter_testing/leak_tracker_flutter_testing.dart';

class PrintOverrideTestBinding extends AutomatedTestWidgetsFlutterBinding {
  @override
  DebugPrintCallback get debugPrintOverride => _enablePrint ? debugPrint : _emptyPrint;

  static void _emptyPrint(String? message, {int? wrapWidth}) {}

  static bool _enablePrint = true;

  static void runWithDebugPrintDisabled(void Function() f) {
    try {
      _enablePrint = false;
      f();
    } finally {
      _enablePrint = true;
    }
  }
}

void main() {
  // LeakTesting is turned off because it adds subscriptions to
  // [FlutterMemoryAllocations], that may interfere with the tests.
  LeakTesting.settings = LeakTesting.settings.withIgnoredAll();
  final FlutterMemoryAllocations ma = FlutterMemoryAllocations.instance;

  PrintOverrideTestBinding();

  setUp(() {
    assert(!ma.hasListeners);
    _checkSdkHandlersNotSet();
  });

  test('addListener and removeListener add and remove listeners.', () {
    final ObjectEvent event = ObjectDisposed(object: 'object');
    ObjectEvent? receivedEvent;
    void listener(ObjectEvent event) => receivedEvent = event;
    expect(ma.hasListeners, isFalse);

    ma.addListener(listener);
    _checkSdkHandlersSet();
    ma.dispatchObjectEvent(event);
    expect(receivedEvent, equals(event));
    expect(ma.hasListeners, isTrue);
    receivedEvent = null;

    ma.removeListener(listener);
    ma.dispatchObjectEvent(event);
    expect(receivedEvent, isNull);
    expect(ma.hasListeners, isFalse);
    _checkSdkHandlersNotSet();
  });

  testWidgets('dispatchObjectEvent handles bad listeners', (WidgetTester tester) async {
    final ObjectEvent event = ObjectDisposed(object: 'object');
    final List<String> log = <String>[];
    void badListener1(ObjectEvent event) {
      log.add('badListener1');
      throw ArgumentError();
    }

    void listener1(ObjectEvent event) => log.add('listener1');
    void badListener2(ObjectEvent event) {
      log.add('badListener2');
      throw ArgumentError();
    }

    void listener2(ObjectEvent event) => log.add('listener2');

    ma.addListener(badListener1);
    _checkSdkHandlersSet();
    ma.addListener(listener1);
    ma.addListener(badListener2);
    ma.addListener(listener2);

    PrintOverrideTestBinding.runWithDebugPrintDisabled(() => ma.dispatchObjectEvent(event));
    expect(log, <String>['badListener1', 'listener1', 'badListener2', 'listener2']);
    expect(tester.takeException(), contains('Multiple exceptions (2)'));

    ma.removeListener(badListener1);
    _checkSdkHandlersSet();
    ma.removeListener(listener1);
    ma.removeListener(badListener2);
    ma.removeListener(listener2);
    _checkSdkHandlersNotSet();

    log.clear();
    expect(ma.hasListeners, isFalse);
    ma.dispatchObjectEvent(event);
    expect(log, <String>[]);
  });

  test('dispatchObjectEvent does not invoke concurrently added listeners', () {
    final ObjectEvent event = ObjectDisposed(object: 'object');
    final List<String> log = <String>[];

    void listener2(ObjectEvent event) => log.add('listener2');
    void listener1(ObjectEvent event) {
      log.add('listener1');
      ma.addListener(listener2);
    }

    ma.addListener(listener1);
    _checkSdkHandlersSet();

    ma.dispatchObjectEvent(event);
    expect(log, <String>['listener1']);
    log.clear();

    ma.dispatchObjectEvent(event);
    expect(log, <String>['listener1', 'listener2']);
    log.clear();

    ma.removeListener(listener1);
    ma.removeListener(listener2);
    _checkSdkHandlersNotSet();

    expect(ma.hasListeners, isFalse);
    ma.dispatchObjectEvent(event);
    expect(log, <String>[]);
  });

  test('dispatchObjectEvent does not invoke concurrently removed listeners', () {
    final ObjectEvent event = ObjectDisposed(object: 'object');
    final List<String> log = <String>[];

    void listener2(ObjectEvent event) => log.add('listener2');
    void listener1(ObjectEvent event) {
      log.add('listener1');
      ma.removeListener(listener2);
      expect(ma.hasListeners, isFalse);
    }

    ma.addListener(listener1);
    ma.addListener(listener2);

    ma.dispatchObjectEvent(event);
    expect(log, <String>['listener1']);
    log.clear();

    ma.removeListener(listener1);
    _checkSdkHandlersNotSet();

    expect(ma.hasListeners, isFalse);
  });

  test('last removeListener unsubscribes from Flutter SDK events', () {
    void listener1(ObjectEvent event) {}
    void listener2(ObjectEvent event) {}

    ma.addListener(listener1);
    _checkSdkHandlersSet();

    ma.addListener(listener2);
    _checkSdkHandlersSet();

    ma.removeListener(listener1);
    _checkSdkHandlersSet();

    ma.removeListener(listener2);
    _checkSdkHandlersNotSet();
  });

  test('kFlutterMemoryAllocationsEnabled is true in debug mode.', () {
    expect(kFlutterMemoryAllocationsEnabled, isTrue);
  });

  test('publishers in Flutter dispatch events in debug mode', () async {
    int eventCount = 0;
    void listener(ObjectEvent event) => eventCount++;
    ma.addListener(listener);

    final int expectedEventCount = await _activateFlutterObjectsAndReturnCountOfEvents();
    expect(eventCount, expectedEventCount);

    ma.removeListener(listener);
    _checkSdkHandlersNotSet();
    expect(ma.hasListeners, isFalse);
  });
}

void _checkSdkHandlersSet() {
  expect(Image.onCreate, isNotNull);
  expect(Picture.onCreate, isNotNull);
  expect(Image.onDispose, isNotNull);
  expect(Picture.onDispose, isNotNull);
}

void _checkSdkHandlersNotSet() {
  expect(Image.onCreate, isNull);
  expect(Picture.onCreate, isNull);
  expect(Image.onDispose, isNull);
  expect(Picture.onDispose, isNull);
}

/// Create and dispose Flutter objects to fire memory allocation events.
Future<int> _activateFlutterObjectsAndReturnCountOfEvents() async {
  int count = 0;

  final ValueNotifier<bool> valueNotifier = ValueNotifier<bool>(true);
  count++;
  final ChangeNotifier changeNotifier = ChangeNotifier()..addListener(() {});
  count++;
  final Picture picture = _createPicture();
  count++;

  valueNotifier.dispose();
  count++;
  changeNotifier.dispose();
  count++;
  picture.dispose();
  count++;

  final Image image = await _createImage();
  count++;
  count++;
  count++;
  image.dispose();
  count++;

  return count;
}

Future<Image> _createImage() async {
  final Picture picture = _createPicture();
  final Image result = await picture.toImage(10, 10);
  picture.dispose();
  return result;
}

Picture _createPicture() {
  final PictureRecorder recorder = PictureRecorder();
  final Canvas canvas = Canvas(recorder);
  const Rect rect = Rect.fromLTWH(0.0, 0.0, 100.0, 100.0);
  canvas.clipRect(rect);
  return recorder.endRecording();
}
