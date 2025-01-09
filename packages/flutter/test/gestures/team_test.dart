// Copyright 2014 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

import 'package:flutter/gestures.dart';
import 'package:flutter_test/flutter_test.dart';

import 'gesture_tester.dart';

void main() {
  TestWidgetsFlutterBinding.ensureInitialized();

  testGesture('GestureArenaTeam rejection test', (GestureTester tester) {
    final GestureArenaTeam team = GestureArenaTeam();
    final HorizontalDragGestureRecognizer horizontalDrag =
        HorizontalDragGestureRecognizer()..team = team;
    final VerticalDragGestureRecognizer verticalDrag = VerticalDragGestureRecognizer()..team = team;
    final TapGestureRecognizer tap = TapGestureRecognizer();

    expect(horizontalDrag.team, equals(team));
    expect(verticalDrag.team, equals(team));
    expect(tap.team, isNull);

    final List<String> log = <String>[];

    horizontalDrag.onStart = (DragStartDetails details) {
      log.add('horizontal-drag-start');
    };
    verticalDrag.onStart = (DragStartDetails details) {
      log.add('vertical-drag-start');
    };
    tap.onTap = () {
      log.add('tap');
    };

    void test(Offset delta) {
      const Offset origin = Offset(10.0, 10.0);
      final TestPointer pointer = TestPointer(5);
      final PointerDownEvent down = pointer.down(origin);
      horizontalDrag.addPointer(down);
      verticalDrag.addPointer(down);
      tap.addPointer(down);
      expect(log, isEmpty);
      tester.closeArena(5);
      expect(log, isEmpty);
      tester.route(down);
      expect(log, isEmpty);
      tester.route(pointer.move(origin + delta));
      tester.route(pointer.up());
    }

    test(Offset.zero);
    expect(log, <String>['tap']);
    log.clear();

    test(const Offset(0.0, 30.0));
    expect(log, <String>['vertical-drag-start']);
    log.clear();

    horizontalDrag.dispose();
    verticalDrag.dispose();
    tap.dispose();
  });

  testGesture('GestureArenaTeam captain', (GestureTester tester) {
    final GestureArenaTeam team = GestureArenaTeam();
    final PassiveGestureRecognizer captain = PassiveGestureRecognizer()..team = team;
    final HorizontalDragGestureRecognizer horizontalDrag =
        HorizontalDragGestureRecognizer()..team = team;
    final VerticalDragGestureRecognizer verticalDrag = VerticalDragGestureRecognizer()..team = team;
    final TapGestureRecognizer tap = TapGestureRecognizer();

    team.captain = captain;

    final List<String> log = <String>[];

    captain.onGestureAccepted = () {
      log.add('captain accepted gesture');
    };
    horizontalDrag.onStart = (DragStartDetails details) {
      log.add('horizontal-drag-start');
    };
    verticalDrag.onStart = (DragStartDetails details) {
      log.add('vertical-drag-start');
    };
    tap.onTap = () {
      log.add('tap');
    };

    void test(Offset delta) {
      const Offset origin = Offset(10.0, 10.0);
      final TestPointer pointer = TestPointer(5);
      final PointerDownEvent down = pointer.down(origin);
      captain.addPointer(down);
      horizontalDrag.addPointer(down);
      verticalDrag.addPointer(down);
      tap.addPointer(down);
      expect(log, isEmpty);
      tester.closeArena(5);
      expect(log, isEmpty);
      tester.route(down);
      expect(log, isEmpty);
      tester.route(pointer.move(origin + delta));
      tester.route(pointer.up());
    }

    test(Offset.zero);
    expect(log, <String>['tap']);
    log.clear();

    test(const Offset(0.0, 30.0));
    expect(log, <String>['captain accepted gesture']);
    log.clear();

    horizontalDrag.dispose();
    verticalDrag.dispose();
    tap.dispose();
    captain.dispose();
  });
}

typedef GestureAcceptedCallback = void Function();

class PassiveGestureRecognizer extends OneSequenceGestureRecognizer {
  GestureAcceptedCallback? onGestureAccepted;

  @override
  void addPointer(PointerDownEvent event) {
    startTrackingPointer(event.pointer);
  }

  @override
  String get debugDescription => 'passive';

  @override
  void didStopTrackingLastPointer(int pointer) {
    resolve(GestureDisposition.rejected);
  }

  @override
  void handleEvent(PointerEvent event) {
    if (event is PointerUpEvent || event is PointerCancelEvent) {
      stopTrackingPointer(event.pointer);
    }
  }

  @override
  void acceptGesture(int pointer) {
    onGestureAccepted?.call();
  }

  @override
  void rejectGesture(int pointer) {}
}
