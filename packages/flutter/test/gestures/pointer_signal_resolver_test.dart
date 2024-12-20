// Copyright 2014 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

import 'package:flutter/gestures.dart';
import 'package:flutter_test/flutter_test.dart';

class TestPointerSignalListener {
  TestPointerSignalListener(this.event);

  final PointerSignalEvent event;
  bool callbackRan = false;

  void callback(PointerSignalEvent event) {
    expect(event, equals(this.event));
    expect(callbackRan, isFalse);
    callbackRan = true;
  }
}

class PointerSignalTester {
  final PointerSignalResolver resolver = PointerSignalResolver();
  PointerSignalEvent event = const PointerScrollEvent();

  TestPointerSignalListener addListener() {
    final TestPointerSignalListener listener = TestPointerSignalListener(event);
    resolver.register(event, listener.callback);
    return listener;
  }

  /// Simulates a new event dispatch cycle by resolving the current event and
  /// setting a new event to use for future calls.
  void resolve() {
    resolver.resolve(event);
    event = const PointerScrollEvent();
  }
}

void main() {
  test('Resolving with no entries should be a no-op', () {
    final PointerSignalTester tester = PointerSignalTester();
    tester.resolver.resolve(tester.event);
  });

  test('Resolving with no entries should notify engine of no-op', () {
    bool allowedPlatformDefault = false;
    final PointerSignalTester tester = PointerSignalTester();
    tester.event = PointerScrollEvent(
      onRespond: ({required bool allowPlatformDefault}) {
        allowedPlatformDefault = allowPlatformDefault;
      },
    );
    tester.resolver.resolve(tester.event);
    expect(
      allowedPlatformDefault,
      isTrue,
      reason: 'Should have called respond with allowPlatformDefault: true',
    );
  });

  test('First entry should always win', () {
    final PointerSignalTester tester = PointerSignalTester();
    final TestPointerSignalListener first = tester.addListener();
    final TestPointerSignalListener second = tester.addListener();
    tester.resolve();
    expect(first.callbackRan, isTrue);
    expect(second.callbackRan, isFalse);
  });

  test('Re-use after resolve should work', () {
    final PointerSignalTester tester = PointerSignalTester();
    final TestPointerSignalListener first = tester.addListener();
    final TestPointerSignalListener second = tester.addListener();
    tester.resolve();
    expect(first.callbackRan, isTrue);
    expect(second.callbackRan, isFalse);

    final TestPointerSignalListener newEventListener = tester.addListener();
    tester.resolve();
    expect(newEventListener.callbackRan, isTrue);
    // Nothing should have changed for the previous event's listeners.
    expect(first.callbackRan, isTrue);
    expect(second.callbackRan, isFalse);
  });

  test('works with transformed events', () {
    final PointerSignalResolver resolver = PointerSignalResolver();
    const PointerScrollEvent originalEvent = PointerScrollEvent();
    final PointerSignalEvent transformedEvent = originalEvent.transformed(
      Matrix4.translationValues(10.0, 20.0, 0.0),
    );
    final PointerSignalEvent anotherTransformedEvent = originalEvent.transformed(
      Matrix4.translationValues(30.0, 50.0, 0.0),
    );

    expect(originalEvent, isNot(same(transformedEvent)));
    expect(transformedEvent.original, same(originalEvent));

    expect(originalEvent, isNot(same(anotherTransformedEvent)));
    expect(anotherTransformedEvent.original, same(originalEvent));

    final List<PointerSignalEvent> events = <PointerSignalEvent>[];
    resolver.register(transformedEvent, (PointerSignalEvent event) {
      events.add(event);
    });

    // Registering a second transformed event should not throw an assertion.
    expect(() {
      resolver.register(anotherTransformedEvent, (PointerSignalEvent event) {
        // This shouldn't be called because only the first registered callback is
        // invoked.
        events.add(event);
      });
    }, returnsNormally);

    resolver.resolve(originalEvent);

    expect(events.single, same(transformedEvent));
  });
}
