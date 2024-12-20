// Copyright 2014 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

import 'package:flutter/foundation.dart';
import 'package:flutter/gestures.dart';
import 'package:flutter_test/flutter_test.dart';

void main() {
  test('Should route pointers', () {
    bool callbackRan = false;
    void callback(PointerEvent event) {
      callbackRan = true;
    }

    final TestPointer pointer2 = TestPointer(2);
    final TestPointer pointer3 = TestPointer(3);

    final PointerRouter router = PointerRouter();
    router.addRoute(3, callback);
    router.route(pointer2.down(Offset.zero));
    expect(callbackRan, isFalse);
    router.route(pointer3.down(Offset.zero));
    expect(callbackRan, isTrue);
    callbackRan = false;
    router.removeRoute(3, callback);
    router.route(pointer3.up());
    expect(callbackRan, isFalse);
  });

  test('Supports re-entrant cancellation', () {
    bool callbackRan = false;
    void callback(PointerEvent event) {
      callbackRan = true;
    }

    final PointerRouter router = PointerRouter();
    router.addRoute(2, (PointerEvent event) {
      router.removeRoute(2, callback);
    });
    router.addRoute(2, callback);
    final TestPointer pointer2 = TestPointer(2);
    router.route(pointer2.down(Offset.zero));
    expect(callbackRan, isFalse);
  });

  test('Supports global callbacks', () {
    bool secondCallbackRan = false;
    void secondCallback(PointerEvent event) {
      secondCallbackRan = true;
    }

    bool firstCallbackRan = false;
    final PointerRouter router = PointerRouter();
    router.addGlobalRoute((PointerEvent event) {
      firstCallbackRan = true;
      router.addGlobalRoute(secondCallback);
    });

    final TestPointer pointer2 = TestPointer(2);
    router.route(pointer2.down(Offset.zero));
    expect(firstCallbackRan, isTrue);
    expect(secondCallbackRan, isFalse);
  });

  test('Supports re-entrant global cancellation', () {
    bool callbackRan = false;
    void callback(PointerEvent event) {
      callbackRan = true;
    }

    final PointerRouter router = PointerRouter();
    router.addGlobalRoute((PointerEvent event) {
      router.removeGlobalRoute(callback);
    });
    router.addGlobalRoute(callback);
    final TestPointer pointer2 = TestPointer(2);
    router.route(pointer2.down(Offset.zero));
    expect(callbackRan, isFalse);
  });

  test('Per-pointer callbacks cannot re-entrantly add global routes', () {
    bool callbackRan = false;
    void callback(PointerEvent event) {
      callbackRan = true;
    }

    final PointerRouter router = PointerRouter();
    bool perPointerCallbackRan = false;
    router.addRoute(2, (PointerEvent event) {
      perPointerCallbackRan = true;
      router.addGlobalRoute(callback);
    });
    final TestPointer pointer2 = TestPointer(2);
    router.route(pointer2.down(Offset.zero));
    expect(perPointerCallbackRan, isTrue);
    expect(callbackRan, isFalse);
  });

  test('Per-pointer callbacks happen before global callbacks', () {
    final List<String> log = <String>[];
    final PointerRouter router = PointerRouter();
    router.addGlobalRoute((PointerEvent event) {
      log.add('global 1');
    });
    router.addRoute(2, (PointerEvent event) {
      log.add('per-pointer 1');
    });
    router.addGlobalRoute((PointerEvent event) {
      log.add('global 2');
    });
    router.addRoute(2, (PointerEvent event) {
      log.add('per-pointer 2');
    });
    final TestPointer pointer2 = TestPointer(2);
    router.route(pointer2.down(Offset.zero));
    expect(log, equals(<String>['per-pointer 1', 'per-pointer 2', 'global 1', 'global 2']));
  });

  test('Exceptions do not stop pointer routing', () {
    final List<String> log = <String>[];
    final PointerRouter router = PointerRouter();
    router.addRoute(2, (PointerEvent event) {
      log.add('per-pointer 1');
    });
    router.addRoute(2, (PointerEvent event) {
      log.add('per-pointer 2');
      throw 'Having a bad day!';
    });
    router.addRoute(2, (PointerEvent event) {
      log.add('per-pointer 3');
    });

    final FlutterExceptionHandler? previousErrorHandler = FlutterError.onError;
    FlutterError.onError = (FlutterErrorDetails details) {
      log.add('error report');
    };

    final TestPointer pointer2 = TestPointer(2);
    router.route(pointer2.down(Offset.zero));
    expect(
      log,
      equals(<String>['per-pointer 1', 'per-pointer 2', 'error report', 'per-pointer 3']),
    );

    FlutterError.onError = previousErrorHandler;
  });

  test('Exceptions include router, route & event', () {
    try {
      final PointerRouter router = PointerRouter();
      router.addRoute(2, (PointerEvent event) => throw 'Pointer exception');
    } catch (e) {
      expect(e, contains("router: Instance of 'PointerRouter'"));
      expect(e, contains('route: Closure: (PointerEvent) => Null'));
      expect(e, contains('event: PointerDownEvent#[a-zA-Z0-9]{5}(position: Offset(0.0, 0.0))'));
    }
  });

  test('Should transform events', () {
    final List<PointerEvent> events = <PointerEvent>[];
    final List<PointerEvent> globalEvents = <PointerEvent>[];
    final PointerRouter router = PointerRouter();
    final Matrix4 transform = (Matrix4.identity()..scale(1 / 2.0, 1 / 2.0, 1.0)).multiplied(
      Matrix4.translationValues(-10, -30, 0),
    );

    router.addRoute(1, (PointerEvent event) {
      events.add(event);
    }, transform);

    router.addGlobalRoute((PointerEvent event) {
      globalEvents.add(event);
    }, transform);

    final TestPointer pointer1 = TestPointer();
    const Offset firstPosition = Offset(16, 36);
    router.route(pointer1.down(firstPosition));

    expect(events.single.transform, transform);
    expect(events.single.position, firstPosition);
    expect(events.single.delta, Offset.zero);
    expect(events.single.localPosition, const Offset(3, 3));
    expect(events.single.localDelta, Offset.zero);

    expect(globalEvents.single.transform, transform);
    expect(globalEvents.single.position, firstPosition);
    expect(globalEvents.single.delta, Offset.zero);
    expect(globalEvents.single.localPosition, const Offset(3, 3));
    expect(globalEvents.single.localDelta, Offset.zero);

    events.clear();
    globalEvents.clear();

    const Offset newPosition = Offset(20, 40);
    router.route(pointer1.move(newPosition));

    expect(events.single.transform, transform);
    expect(events.single.position, newPosition);
    expect(events.single.delta, newPosition - firstPosition);
    expect(events.single.localPosition, const Offset(5, 5));
    expect(events.single.localDelta, const Offset(2, 2));

    expect(globalEvents.single.transform, transform);
    expect(globalEvents.single.position, newPosition);
    expect(globalEvents.single.delta, newPosition - firstPosition);
    expect(globalEvents.single.localPosition, const Offset(5, 5));
    expect(globalEvents.single.localDelta, const Offset(2, 2));
  });
}
