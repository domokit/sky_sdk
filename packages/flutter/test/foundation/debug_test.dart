// Copyright 2014 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

import 'package:flutter/foundation.dart';
import 'package:flutter_test/flutter_test.dart';

void main() {
  group('debugInstrumentAction', () {
    late DebugPrintCallback originalDebugPrintCallback;
    late StringBuffer printBuffer;

    setUp(() {
      debugInstrumentationEnabled = true;
      printBuffer = StringBuffer();
      originalDebugPrintCallback = debugPrint;
      debugPrint = (String? message, {int? wrapWidth}) {
        printBuffer.writeln(message);
      };
    });

    tearDown(() {
      debugInstrumentationEnabled = false;
      debugPrint = originalDebugPrintCallback;
    });

    test('works with non-failing actions', () async {
      final int result = await debugInstrumentAction<int>('no-op', () async {
        debugPrint('action()');
        return 1;
      });
      expect(result, 1);
      expect(
        printBuffer.toString(),
        matches(RegExp('^action\\(\\)\nAction "no-op" took .+\$', multiLine: true)),
      );
    });

    test('returns failing future if action throws', () async {
      await expectLater(
        () => debugInstrumentAction<void>('throws', () async {
          await Future<void>.delayed(Duration.zero);
          throw 'Error';
        }),
        throwsA('Error'),
      );
      expect(printBuffer.toString(), matches(r'^Action "throws" took .+'));
    });
  });

  group('Memory allocations', () {
    ObjectEvent? dispatchedEvent;
    final Object object = List<int>.filled(1, 0);

    void listener(ObjectEvent event) {
      expect(dispatchedEvent, null);
      dispatchedEvent = event;
    }

    setUp(() {
      dispatchedEvent = null;
      FlutterMemoryAllocations.instance.addListener(listener);
    });

    tearDown(() {
      FlutterMemoryAllocations.instance.removeListener(listener);
    });

    test('debugMaybeDispatchCreated', () async {
      debugMaybeDispatchCreated('library', 'class', object);

      if (kFlutterMemoryAllocationsEnabled) {
        final ObjectEvent? theEvent = dispatchedEvent;

        if (theEvent is! ObjectCreated) {
          fail('Expected ObjectCreated event');
        }

        expect(theEvent.object, object);
        expect(theEvent.library, 'package:flutter/library.dart');
        expect(theEvent.className, 'class');
      } else {
        expect(dispatchedEvent, isNull);
      }
    });

    test('debugMaybeDispatchDisposed', () async {
      debugMaybeDispatchDisposed(object);

      if (kFlutterMemoryAllocationsEnabled) {
        final ObjectEvent? theEvent = dispatchedEvent;

        if (theEvent is! ObjectDisposed) {
          fail('Expected ObjectDisposed event');
        }

        expect(theEvent.object, object);
      } else {
        expect(dispatchedEvent, isNull);
      }
    });
  });
}
