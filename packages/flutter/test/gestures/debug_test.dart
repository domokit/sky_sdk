// Copyright 2014 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

import 'package:flutter/foundation.dart';
import 'package:flutter/gestures.dart';
import 'package:flutter_test/flutter_test.dart';

void main() {
  testWidgets('debugPrintGestureArenaDiagnostics', (WidgetTester tester) async {
    PointerEvent event;
    debugPrintGestureArenaDiagnostics = true;
    final DebugPrintCallback oldCallback = debugPrint;
    final List<String> log = <String>[];
    debugPrint = (String? s, {int? wrapWidth}) {
      log.add(s ?? '');
    };

    final TapGestureRecognizer tap =
        TapGestureRecognizer()
          ..onTapDown = (TapDownDetails details) {}
          ..onTapUp = (TapUpDetails details) {}
          ..onTap = () {}
          ..onTapCancel = () {};
    expect(log, isEmpty);

    event = const PointerDownEvent(pointer: 1, position: Offset(10.0, 10.0));
    tap.addPointer(event as PointerDownEvent);
    expect(log, hasLength(2));
    expect(log[0], equalsIgnoringHashCodes('Gesture arena 1    ❙ ★ Opening new gesture arena.'));
    expect(
      log[1],
      equalsIgnoringHashCodes(
        'Gesture arena 1    ❙ Adding: TapGestureRecognizer#00000(state: ready, button: 1)',
      ),
    );
    log.clear();

    GestureBinding.instance.gestureArena.close(1);
    expect(log, hasLength(1));
    expect(log[0], equalsIgnoringHashCodes('Gesture arena 1    ❙ Closing with 1 member.'));
    log.clear();

    GestureBinding.instance.pointerRouter.route(event);
    expect(log, isEmpty);

    event = const PointerUpEvent(pointer: 1, position: Offset(12.0, 8.0));
    GestureBinding.instance.pointerRouter.route(event);
    expect(log, isEmpty);

    GestureBinding.instance.gestureArena.sweep(1);
    expect(log, hasLength(2));
    expect(log[0], equalsIgnoringHashCodes('Gesture arena 1    ❙ Sweeping with 1 member.'));
    expect(
      log[1],
      equalsIgnoringHashCodes(
        'Gesture arena 1    ❙ Winner: TapGestureRecognizer#00000(state: ready, finalPosition: Offset(12.0, 8.0), button: 1)',
      ),
    );
    log.clear();

    tap.dispose();
    expect(log, isEmpty);

    debugPrintGestureArenaDiagnostics = false;
    debugPrint = oldCallback;
  });

  testWidgets('debugPrintRecognizerCallbacksTrace', (WidgetTester tester) async {
    PointerEvent event;
    debugPrintRecognizerCallbacksTrace = true;
    final DebugPrintCallback oldCallback = debugPrint;
    final List<String> log = <String>[];
    debugPrint = (String? s, {int? wrapWidth}) {
      log.add(s ?? '');
    };

    final TapGestureRecognizer tap =
        TapGestureRecognizer()
          ..onTapDown = (TapDownDetails details) {}
          ..onTapUp = (TapUpDetails details) {}
          ..onTap = () {}
          ..onTapCancel = () {};
    expect(log, isEmpty);

    event = const PointerDownEvent(pointer: 1, position: Offset(10.0, 10.0));
    tap.addPointer(event as PointerDownEvent);
    expect(log, isEmpty);

    GestureBinding.instance.gestureArena.close(1);
    expect(log, isEmpty);

    GestureBinding.instance.pointerRouter.route(event);
    expect(log, isEmpty);

    event = const PointerUpEvent(pointer: 1, position: Offset(12.0, 8.0));
    GestureBinding.instance.pointerRouter.route(event);
    expect(log, isEmpty);

    GestureBinding.instance.gestureArena.sweep(1);
    expect(log, hasLength(3));
    expect(
      log[0],
      equalsIgnoringHashCodes(
        'TapGestureRecognizer#00000(state: ready, finalPosition: Offset(12.0, 8.0), button: 1) calling onTapDown callback.',
      ),
    );
    expect(
      log[1],
      equalsIgnoringHashCodes(
        'TapGestureRecognizer#00000(state: ready, won arena, finalPosition: Offset(12.0, 8.0), button: 1, sent tap down) calling onTapUp callback.',
      ),
    );
    expect(
      log[2],
      equalsIgnoringHashCodes(
        'TapGestureRecognizer#00000(state: ready, won arena, finalPosition: Offset(12.0, 8.0), button: 1, sent tap down) calling onTap callback.',
      ),
    );
    log.clear();

    tap.dispose();
    expect(log, isEmpty);

    debugPrintRecognizerCallbacksTrace = false;
    debugPrint = oldCallback;
  });

  testWidgets('debugPrintGestureArenaDiagnostics and debugPrintRecognizerCallbacksTrace', (
    WidgetTester tester,
  ) async {
    PointerEvent event;
    debugPrintGestureArenaDiagnostics = true;
    debugPrintRecognizerCallbacksTrace = true;
    final DebugPrintCallback oldCallback = debugPrint;
    final List<String> log = <String>[];
    debugPrint = (String? s, {int? wrapWidth}) {
      log.add(s ?? '');
    };

    final TapGestureRecognizer tap =
        TapGestureRecognizer()
          ..onTapDown = (TapDownDetails details) {}
          ..onTapUp = (TapUpDetails details) {}
          ..onTap = () {}
          ..onTapCancel = () {};
    expect(log, isEmpty);

    event = const PointerDownEvent(pointer: 1, position: Offset(10.0, 10.0));
    tap.addPointer(event as PointerDownEvent);
    expect(log, hasLength(2));
    expect(log[0], equalsIgnoringHashCodes('Gesture arena 1    ❙ ★ Opening new gesture arena.'));
    expect(
      log[1],
      equalsIgnoringHashCodes(
        'Gesture arena 1    ❙ Adding: TapGestureRecognizer#00000(state: ready, button: 1)',
      ),
    );
    log.clear();

    GestureBinding.instance.gestureArena.close(1);
    expect(log, hasLength(1));
    expect(log[0], equalsIgnoringHashCodes('Gesture arena 1    ❙ Closing with 1 member.'));
    log.clear();

    GestureBinding.instance.pointerRouter.route(event);
    expect(log, isEmpty);

    event = const PointerUpEvent(pointer: 1, position: Offset(12.0, 8.0));
    GestureBinding.instance.pointerRouter.route(event);
    expect(log, isEmpty);

    GestureBinding.instance.gestureArena.sweep(1);
    expect(log, hasLength(5));
    expect(log[0], equalsIgnoringHashCodes('Gesture arena 1    ❙ Sweeping with 1 member.'));
    expect(
      log[1],
      equalsIgnoringHashCodes(
        'Gesture arena 1    ❙ Winner: TapGestureRecognizer#00000(state: ready, finalPosition: Offset(12.0, 8.0), button: 1)',
      ),
    );
    expect(
      log[2],
      equalsIgnoringHashCodes(
        '                   ❙ TapGestureRecognizer#00000(state: ready, finalPosition: Offset(12.0, 8.0), button: 1) calling onTapDown callback.',
      ),
    );
    expect(
      log[3],
      equalsIgnoringHashCodes(
        '                   ❙ TapGestureRecognizer#00000(state: ready, won arena, finalPosition: Offset(12.0, 8.0), button: 1, sent tap down) calling onTapUp callback.',
      ),
    );
    expect(
      log[4],
      equalsIgnoringHashCodes(
        '                   ❙ TapGestureRecognizer#00000(state: ready, won arena, finalPosition: Offset(12.0, 8.0), button: 1, sent tap down) calling onTap callback.',
      ),
    );
    log.clear();

    tap.dispose();
    expect(log, isEmpty);

    debugPrintGestureArenaDiagnostics = false;
    debugPrintRecognizerCallbacksTrace = false;
    debugPrint = oldCallback;
  });

  test('TapGestureRecognizer _sentTapDown toString', () {
    final TapGestureRecognizer tap =
        TapGestureRecognizer()..onTap = () {}; // Add a callback so that event can be added
    expect(tap.toString(), equalsIgnoringHashCodes('TapGestureRecognizer#00000(state: ready)'));
    const PointerDownEvent event = PointerDownEvent(pointer: 1, position: Offset(10.0, 10.0));
    tap.addPointer(event);
    tap.didExceedDeadline();
    expect(
      tap.toString(),
      equalsIgnoringHashCodes(
        'TapGestureRecognizer#00000(state: possible, button: 1, sent tap down)',
      ),
    );
    GestureBinding.instance.gestureArena.close(1);
    tap.dispose();
  });
}
