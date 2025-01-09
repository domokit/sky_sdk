// Copyright 2014 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

// TODO(gspencergoog): Remove this tag once this test's state leaks/test
// dependencies have been fixed.
// https://github.com/flutter/flutter/issues/85160
// Fails with "flutter test --test-randomize-ordering-seed=123"
@Tags(<String>['no-shuffle'])
library;

import 'package:flutter/gestures.dart';
import 'package:flutter/material.dart';
import 'package:flutter/rendering.dart';
import 'package:flutter/services.dart';
import 'package:flutter_test/flutter_test.dart';

import 'semantics_tester.dart';

void main() {
  testWidgets('Drag and drop - control test', (WidgetTester tester) async {
    final List<int> accepted = <int>[];
    final List<DragTargetDetails<int>> acceptedDetails = <DragTargetDetails<int>>[];
    int dragStartedCount = 0;
    int moveCount = 0;

    await tester.pumpWidget(
      MaterialApp(
        home: Column(
          children: <Widget>[
            Draggable<int>(
              data: 1,
              feedback: const Text('Dragging'),
              onDragStarted: () {
                ++dragStartedCount;
              },
              child: const Text('Source'),
            ),
            DragTarget<int>(
              builder: (BuildContext context, List<int?> data, List<dynamic> rejects) {
                return const SizedBox(height: 100.0, child: Text('Target'));
              },
              onMove: (_) => moveCount++,
              onAccept: accepted.add,
              onAcceptWithDetails: acceptedDetails.add,
            ),
          ],
        ),
      ),
    );

    expect(accepted, isEmpty);
    expect(acceptedDetails, isEmpty);
    expect(find.text('Source'), findsOneWidget);
    expect(find.text('Dragging'), findsNothing);
    expect(find.text('Target'), findsOneWidget);
    expect(dragStartedCount, 0);
    expect(moveCount, 0);

    final Offset firstLocation = tester.getCenter(find.text('Source'));
    final TestGesture gesture = await tester.startGesture(firstLocation, pointer: 7);
    await tester.pump();

    expect(accepted, isEmpty);
    expect(acceptedDetails, isEmpty);
    expect(find.text('Source'), findsOneWidget);
    expect(find.text('Dragging'), findsOneWidget);
    expect(find.text('Target'), findsOneWidget);
    expect(dragStartedCount, 1);
    expect(moveCount, 0);

    final Offset secondLocation = tester.getCenter(find.text('Target'));
    await gesture.moveTo(secondLocation);
    await tester.pump();

    expect(accepted, isEmpty);
    expect(acceptedDetails, isEmpty);
    expect(find.text('Source'), findsOneWidget);
    expect(find.text('Dragging'), findsOneWidget);
    expect(find.text('Target'), findsOneWidget);
    expect(dragStartedCount, 1);
    expect(moveCount, 1);

    await gesture.up();
    await tester.pump();

    expect(accepted, equals(<int>[1]));
    expect(acceptedDetails, hasLength(1));
    expect(acceptedDetails.first.offset, const Offset(256.0, 74.0));
    expect(find.text('Source'), findsOneWidget);
    expect(find.text('Dragging'), findsNothing);
    expect(find.text('Target'), findsOneWidget);
    expect(dragStartedCount, 1);
    expect(moveCount, 1);
  });

  // Regression test for https://github.com/flutter/flutter/issues/76825
  testWidgets('Drag and drop - onLeave callback fires correctly with generic parameter', (
    WidgetTester tester,
  ) async {
    final Map<String, int> leftBehind = <String, int>{'Target 1': 0, 'Target 2': 0};

    await tester.pumpWidget(
      MaterialApp(
        home: Column(
          children: <Widget>[
            const Draggable<int>(data: 1, feedback: Text('Dragging'), child: Text('Source')),
            DragTarget<int>(
              builder: (BuildContext context, List<int?> data, List<dynamic> rejects) {
                return const SizedBox(height: 100.0, child: Text('Target 1'));
              },
              onLeave: (int? data) {
                if (data != null) {
                  leftBehind['Target 1'] = leftBehind['Target 1']! + data;
                }
              },
            ),
            DragTarget<int>(
              builder: (BuildContext context, List<int?> data, List<dynamic> rejects) {
                return const SizedBox(height: 100.0, child: Text('Target 2'));
              },
              onLeave: (int? data) {
                if (data != null) {
                  leftBehind['Target 2'] = leftBehind['Target 2']! + data;
                }
              },
            ),
          ],
        ),
      ),
    );

    expect(leftBehind['Target 1'], equals(0));
    expect(leftBehind['Target 2'], equals(0));

    final Offset firstLocation = tester.getCenter(find.text('Source'));
    final TestGesture gesture = await tester.startGesture(firstLocation, pointer: 7);
    await tester.pump();

    expect(leftBehind['Target 1'], equals(0));
    expect(leftBehind['Target 2'], equals(0));

    final Offset secondLocation = tester.getCenter(find.text('Target 1'));
    await gesture.moveTo(secondLocation);
    await tester.pump();

    expect(leftBehind['Target 1'], equals(0));
    expect(leftBehind['Target 2'], equals(0));

    final Offset thirdLocation = tester.getCenter(find.text('Target 2'));
    await gesture.moveTo(thirdLocation);
    await tester.pump();

    expect(leftBehind['Target 1'], equals(1));
    expect(leftBehind['Target 2'], equals(0));

    await gesture.moveTo(secondLocation);
    await tester.pump();

    expect(leftBehind['Target 1'], equals(1));
    expect(leftBehind['Target 2'], equals(1));

    await gesture.up();
    await tester.pump();

    expect(leftBehind['Target 1'], equals(1));
    expect(leftBehind['Target 2'], equals(1));
  });

  testWidgets('Drag and drop - onLeave callback fires correctly', (WidgetTester tester) async {
    final Map<String, int> leftBehind = <String, int>{'Target 1': 0, 'Target 2': 0};

    await tester.pumpWidget(
      MaterialApp(
        home: Column(
          children: <Widget>[
            const Draggable<int>(data: 1, feedback: Text('Dragging'), child: Text('Source')),
            DragTarget<int>(
              builder: (BuildContext context, List<int?> data, List<dynamic> rejects) {
                return const SizedBox(height: 100.0, child: Text('Target 1'));
              },
              onLeave: (Object? data) {
                if (data is int) {
                  leftBehind['Target 1'] = leftBehind['Target 1']! + data;
                }
              },
            ),
            DragTarget<int>(
              builder: (BuildContext context, List<int?> data, List<dynamic> rejects) {
                return const SizedBox(height: 100.0, child: Text('Target 2'));
              },
              onLeave: (Object? data) {
                if (data is int) {
                  leftBehind['Target 2'] = leftBehind['Target 2']! + data;
                }
              },
            ),
          ],
        ),
      ),
    );

    expect(leftBehind['Target 1'], equals(0));
    expect(leftBehind['Target 2'], equals(0));

    final Offset firstLocation = tester.getCenter(find.text('Source'));
    final TestGesture gesture = await tester.startGesture(firstLocation, pointer: 7);
    await tester.pump();

    expect(leftBehind['Target 1'], equals(0));
    expect(leftBehind['Target 2'], equals(0));

    final Offset secondLocation = tester.getCenter(find.text('Target 1'));
    await gesture.moveTo(secondLocation);
    await tester.pump();

    expect(leftBehind['Target 1'], equals(0));
    expect(leftBehind['Target 2'], equals(0));

    final Offset thirdLocation = tester.getCenter(find.text('Target 2'));
    await gesture.moveTo(thirdLocation);
    await tester.pump();

    expect(leftBehind['Target 1'], equals(1));
    expect(leftBehind['Target 2'], equals(0));

    await gesture.moveTo(secondLocation);
    await tester.pump();

    expect(leftBehind['Target 1'], equals(1));
    expect(leftBehind['Target 2'], equals(1));

    await gesture.up();
    await tester.pump();

    expect(leftBehind['Target 1'], equals(1));
    expect(leftBehind['Target 2'], equals(1));
  });

  // Regression test for https://github.com/flutter/flutter/issues/76825
  testWidgets('Drag and drop - onMove callback fires correctly with generic parameter', (
    WidgetTester tester,
  ) async {
    final Map<String, int> targetMoveCount = <String, int>{'Target 1': 0, 'Target 2': 0};

    await tester.pumpWidget(
      MaterialApp(
        home: Column(
          children: <Widget>[
            const Draggable<int>(data: 1, feedback: Text('Dragging'), child: Text('Source')),
            DragTarget<int>(
              builder: (BuildContext context, List<int?> data, List<dynamic> rejects) {
                return const SizedBox(height: 100.0, child: Text('Target 1'));
              },
              onMove: (DragTargetDetails<int> details) {
                targetMoveCount['Target 1'] = targetMoveCount['Target 1']! + details.data;
              },
            ),
            DragTarget<int>(
              builder: (BuildContext context, List<int?> data, List<dynamic> rejects) {
                return const SizedBox(height: 100.0, child: Text('Target 2'));
              },
              onMove: (DragTargetDetails<int> details) {
                targetMoveCount['Target 2'] = targetMoveCount['Target 2']! + details.data;
              },
            ),
          ],
        ),
      ),
    );

    expect(targetMoveCount['Target 1'], equals(0));
    expect(targetMoveCount['Target 2'], equals(0));

    final Offset firstLocation = tester.getCenter(find.text('Source'));
    final TestGesture gesture = await tester.startGesture(firstLocation, pointer: 7);
    await tester.pump();

    expect(targetMoveCount['Target 1'], equals(0));
    expect(targetMoveCount['Target 2'], equals(0));

    final Offset secondLocation = tester.getCenter(find.text('Target 1'));
    await gesture.moveTo(secondLocation);
    await tester.pump();

    expect(targetMoveCount['Target 1'], equals(1));
    expect(targetMoveCount['Target 2'], equals(0));

    final Offset thirdLocation = tester.getCenter(find.text('Target 2'));
    await gesture.moveTo(thirdLocation);
    await tester.pump();

    expect(targetMoveCount['Target 1'], equals(1));
    expect(targetMoveCount['Target 2'], equals(1));

    await gesture.moveTo(secondLocation);
    await tester.pump();

    expect(targetMoveCount['Target 1'], equals(2));
    expect(targetMoveCount['Target 2'], equals(1));

    await gesture.up();
    await tester.pump();

    expect(targetMoveCount['Target 1'], equals(2));
    expect(targetMoveCount['Target 2'], equals(1));
  });

  testWidgets('Drag and drop - onMove callback fires correctly', (WidgetTester tester) async {
    final Map<String, int> targetMoveCount = <String, int>{'Target 1': 0, 'Target 2': 0};

    await tester.pumpWidget(
      MaterialApp(
        home: Column(
          children: <Widget>[
            const Draggable<int>(data: 1, feedback: Text('Dragging'), child: Text('Source')),
            DragTarget<int>(
              builder: (BuildContext context, List<int?> data, List<dynamic> rejects) {
                return const SizedBox(height: 100.0, child: Text('Target 1'));
              },
              onMove: (DragTargetDetails<dynamic> details) {
                if (details.data is int) {
                  targetMoveCount['Target 1'] =
                      targetMoveCount['Target 1']! + (details.data as int);
                }
              },
            ),
            DragTarget<int>(
              builder: (BuildContext context, List<int?> data, List<dynamic> rejects) {
                return const SizedBox(height: 100.0, child: Text('Target 2'));
              },
              onMove: (DragTargetDetails<dynamic> details) {
                if (details.data is int) {
                  targetMoveCount['Target 2'] =
                      targetMoveCount['Target 2']! + (details.data as int);
                }
              },
            ),
          ],
        ),
      ),
    );

    expect(targetMoveCount['Target 1'], equals(0));
    expect(targetMoveCount['Target 2'], equals(0));

    final Offset firstLocation = tester.getCenter(find.text('Source'));
    final TestGesture gesture = await tester.startGesture(firstLocation, pointer: 7);
    await tester.pump();

    expect(targetMoveCount['Target 1'], equals(0));
    expect(targetMoveCount['Target 2'], equals(0));

    final Offset secondLocation = tester.getCenter(find.text('Target 1'));
    await gesture.moveTo(secondLocation);
    await tester.pump();

    expect(targetMoveCount['Target 1'], equals(1));
    expect(targetMoveCount['Target 2'], equals(0));

    final Offset thirdLocation = tester.getCenter(find.text('Target 2'));
    await gesture.moveTo(thirdLocation);
    await tester.pump();

    expect(targetMoveCount['Target 1'], equals(1));
    expect(targetMoveCount['Target 2'], equals(1));

    await gesture.moveTo(secondLocation);
    await tester.pump();

    expect(targetMoveCount['Target 1'], equals(2));
    expect(targetMoveCount['Target 2'], equals(1));

    await gesture.up();
    await tester.pump();

    expect(targetMoveCount['Target 1'], equals(2));
    expect(targetMoveCount['Target 2'], equals(1));
  });

  testWidgets('Drag and drop - onMove is not called if moved with null data', (
    WidgetTester tester,
  ) async {
    bool onMoveCalled = false;

    await tester.pumpWidget(
      MaterialApp(
        home: Column(
          children: <Widget>[
            const Draggable<int>(feedback: Text('Dragging'), child: Text('Source')),
            DragTarget<int>(
              builder: (BuildContext context, List<int?> data, List<dynamic> rejects) {
                return const SizedBox(height: 100.0, child: Text('Target'));
              },
              onMove: (DragTargetDetails<dynamic> details) {
                onMoveCalled = true;
              },
            ),
          ],
        ),
      ),
    );

    expect(onMoveCalled, isFalse);

    final Offset firstLocation = tester.getCenter(find.text('Source'));
    final TestGesture gesture = await tester.startGesture(firstLocation, pointer: 7);
    await tester.pump();

    expect(onMoveCalled, isFalse);

    final Offset secondLocation = tester.getCenter(find.text('Target'));
    await gesture.moveTo(secondLocation);
    await tester.pump();

    expect(onMoveCalled, isFalse);
    await gesture.up();
    await tester.pump();

    expect(onMoveCalled, isFalse);
  });

  testWidgets('Drag and drop - dragging over button', (WidgetTester tester) async {
    final List<String> events = <String>[];
    Offset firstLocation, secondLocation;

    await tester.pumpWidget(
      MaterialApp(
        home: Column(
          children: <Widget>[
            const Draggable<int>(data: 1, feedback: Text('Dragging'), child: Text('Source')),
            Stack(
              children: <Widget>[
                GestureDetector(
                  behavior: HitTestBehavior.opaque,
                  onTap: () {
                    events.add('tap');
                  },
                  child: const Text('Button'),
                ),
                DragTarget<int>(
                  builder: (BuildContext context, List<int?> data, List<dynamic> rejects) {
                    return const IgnorePointer(child: Text('Target'));
                  },
                  onAccept: (int? data) {
                    events.add('drop');
                  },
                  onAcceptWithDetails: (DragTargetDetails<int> _) {
                    events.add('details');
                  },
                ),
              ],
            ),
          ],
        ),
      ),
    );

    expect(events, isEmpty);
    expect(find.text('Source'), findsOneWidget);
    expect(find.text('Dragging'), findsNothing);
    expect(find.text('Target'), findsOneWidget);
    expect(find.text('Button'), findsOneWidget);

    // taps (we check both to make sure the test is consistent)

    expect(events, isEmpty);
    await tester.tap(find.text('Button'));
    expect(events, equals(<String>['tap']));
    events.clear();

    expect(events, isEmpty);
    await tester.tap(find.text('Target'), warnIfMissed: false); // (inside IgnorePointer)
    expect(events, equals(<String>['tap']));
    events.clear();

    // drag and drop

    firstLocation = tester.getCenter(find.text('Source'));
    TestGesture gesture = await tester.startGesture(firstLocation, pointer: 7);
    await tester.pump();

    secondLocation = tester.getCenter(find.text('Target'));
    await gesture.moveTo(secondLocation);
    await tester.pump();

    expect(events, isEmpty);
    await gesture.up();
    await tester.pump();
    expect(events, equals(<String>['drop', 'details']));
    events.clear();

    // drag and tap and drop

    firstLocation = tester.getCenter(find.text('Source'));
    gesture = await tester.startGesture(firstLocation, pointer: 7);
    await tester.pump();

    secondLocation = tester.getCenter(find.text('Target'));
    await gesture.moveTo(secondLocation);
    await tester.pump();

    expect(events, isEmpty);
    await tester.tap(find.text('Button'));
    await tester.tap(find.text('Target'), warnIfMissed: false); // (inside IgnorePointer)
    await gesture.up();
    await tester.pump();
    expect(events, equals(<String>['tap', 'tap', 'drop', 'details']));
    events.clear();
  });

  testWidgets('Drag and drop - tapping button', (WidgetTester tester) async {
    final List<String> events = <String>[];
    Offset firstLocation, secondLocation;

    await tester.pumpWidget(
      MaterialApp(
        home: Column(
          children: <Widget>[
            Draggable<int>(
              data: 1,
              feedback: const Text('Dragging'),
              child: GestureDetector(
                behavior: HitTestBehavior.opaque,
                onTap: () {
                  events.add('tap');
                },
                child: const Text('Button'),
              ),
            ),
            DragTarget<int>(
              builder: (BuildContext context, List<int?> data, List<dynamic> rejects) {
                return const Text('Target');
              },
              onAccept: (int? data) {
                events.add('drop');
              },
              onAcceptWithDetails: (DragTargetDetails<int> _) {
                events.add('details');
              },
            ),
          ],
        ),
      ),
    );

    expect(events, isEmpty);
    expect(find.text('Button'), findsOneWidget);
    expect(find.text('Target'), findsOneWidget);

    expect(events, isEmpty);
    await tester.tap(find.text('Button'));
    expect(events, equals(<String>['tap']));
    events.clear();

    firstLocation = tester.getCenter(find.text('Button'));
    final TestGesture gesture = await tester.startGesture(firstLocation, pointer: 7);
    await tester.pump();

    secondLocation = tester.getCenter(find.text('Target'));
    await gesture.moveTo(secondLocation);
    await tester.pump();

    expect(events, isEmpty);
    await gesture.up();
    await tester.pump();
    expect(events, equals(<String>['drop', 'details']));
    events.clear();
  });

  testWidgets('Drag and drop - long press draggable, short press', (WidgetTester tester) async {
    final List<String> events = <String>[];
    Offset firstLocation, secondLocation;

    await tester.pumpWidget(
      MaterialApp(
        home: Column(
          children: <Widget>[
            const LongPressDraggable<int>(
              data: 1,
              feedback: Text('Dragging'),
              child: Text('Source'),
            ),
            DragTarget<int>(
              builder: (BuildContext context, List<int?> data, List<dynamic> rejects) {
                return const Text('Target');
              },
              onAccept: (int? data) {
                events.add('drop');
              },
              onAcceptWithDetails: (DragTargetDetails<int> _) {
                events.add('details');
              },
            ),
          ],
        ),
      ),
    );

    expect(events, isEmpty);
    expect(find.text('Source'), findsOneWidget);
    expect(find.text('Target'), findsOneWidget);

    expect(events, isEmpty);
    await tester.tap(find.text('Source'));
    expect(events, isEmpty);

    firstLocation = tester.getCenter(find.text('Source'));
    final TestGesture gesture = await tester.startGesture(firstLocation, pointer: 7);
    await tester.pump();

    secondLocation = tester.getCenter(find.text('Target'));
    await gesture.moveTo(secondLocation);
    await tester.pump();

    expect(events, isEmpty);
    await gesture.up();
    await tester.pump();
    expect(events, isEmpty);
  });

  testWidgets('Drag and drop - long press draggable, long press', (WidgetTester tester) async {
    final List<String> events = <String>[];
    Offset firstLocation, secondLocation;

    await tester.pumpWidget(
      MaterialApp(
        home: Column(
          children: <Widget>[
            const Draggable<int>(data: 1, feedback: Text('Dragging'), child: Text('Source')),
            DragTarget<int>(
              builder: (BuildContext context, List<int?> data, List<dynamic> rejects) {
                return const Text('Target');
              },
              onAccept: (int? data) {
                events.add('drop');
              },
              onAcceptWithDetails: (DragTargetDetails<int> _) {
                events.add('details');
              },
            ),
          ],
        ),
      ),
    );

    expect(events, isEmpty);
    expect(find.text('Source'), findsOneWidget);
    expect(find.text('Target'), findsOneWidget);

    expect(events, isEmpty);
    await tester.tap(find.text('Source'));
    expect(events, isEmpty);

    firstLocation = tester.getCenter(find.text('Source'));
    final TestGesture gesture = await tester.startGesture(firstLocation, pointer: 7);
    await tester.pump();

    await tester.pump(const Duration(seconds: 20));

    secondLocation = tester.getCenter(find.text('Target'));
    await gesture.moveTo(secondLocation);
    await tester.pump();

    expect(events, isEmpty);
    await gesture.up();
    await tester.pump();
    expect(events, equals(<String>['drop', 'details']));
  });

  testWidgets('Drag and drop - horizontal and vertical draggables in vertical block', (
    WidgetTester tester,
  ) async {
    final List<String> events = <String>[];
    Offset firstLocation, secondLocation, thirdLocation;

    await tester.pumpWidget(
      MaterialApp(
        home: ListView(
          dragStartBehavior: DragStartBehavior.down,
          children: <Widget>[
            DragTarget<int>(
              builder: (BuildContext context, List<int?> data, List<dynamic> rejects) {
                return const Text('Target');
              },
              onAccept: (int? data) {
                events.add('drop $data');
              },
              onAcceptWithDetails: (DragTargetDetails<int> _) {
                events.add('details');
              },
            ),
            Container(height: 400.0),
            const Draggable<int>(
              data: 1,
              feedback: Text('Dragging'),
              affinity: Axis.horizontal,
              child: Text('H'),
            ),
            const Draggable<int>(
              data: 2,
              feedback: Text('Dragging'),
              affinity: Axis.vertical,
              child: Text('V'),
            ),
            Container(height: 500.0),
            Container(height: 500.0),
            Container(height: 500.0),
            Container(height: 500.0),
          ],
        ),
      ),
    );

    expect(events, isEmpty);
    expect(find.text('Target'), findsOneWidget);
    expect(find.text('H'), findsOneWidget);
    expect(find.text('V'), findsOneWidget);

    // vertical draggable drags vertically
    expect(events, isEmpty);
    firstLocation = tester.getCenter(find.text('V'));
    secondLocation = tester.getCenter(find.text('Target'));
    TestGesture gesture = await tester.startGesture(firstLocation, pointer: 7);
    await tester.pump();
    await gesture.moveTo(secondLocation);
    await tester.pump();
    await gesture.up();
    await tester.pump();
    expect(events, equals(<String>['drop 2', 'details']));
    expect(tester.getCenter(find.text('Target')).dy, greaterThan(0.0));
    events.clear();

    // horizontal draggable drags horizontally
    expect(events, isEmpty);
    firstLocation = tester.getTopLeft(find.text('H'));
    secondLocation = tester.getTopRight(find.text('H'));
    thirdLocation = tester.getCenter(find.text('Target'));
    gesture = await tester.startGesture(firstLocation, pointer: 7);
    await tester.pump();
    await gesture.moveTo(secondLocation);
    await tester.pump();
    await gesture.moveTo(thirdLocation);
    await tester.pump();
    await gesture.up();
    await tester.pump();
    expect(events, equals(<String>['drop 1', 'details']));
    expect(tester.getCenter(find.text('Target')).dy, greaterThan(0.0));
    events.clear();

    // vertical draggable drags horizontally when there's no competition
    // from other gesture detectors
    expect(events, isEmpty);
    firstLocation = tester.getTopLeft(find.text('V'));
    secondLocation = tester.getTopRight(find.text('V'));
    thirdLocation = tester.getCenter(find.text('Target'));
    gesture = await tester.startGesture(firstLocation, pointer: 7);
    await tester.pump();
    await gesture.moveTo(secondLocation);
    await tester.pump();
    await gesture.moveTo(thirdLocation);
    await tester.pump();
    await gesture.up();
    await tester.pump();
    expect(events, equals(<String>['drop 2', 'details']));
    expect(tester.getCenter(find.text('Target')).dy, greaterThan(0.0));
    events.clear();

    // horizontal draggable doesn't drag vertically when there is competition
    // for vertical gestures
    expect(events, isEmpty);
    firstLocation = tester.getCenter(find.text('H'));
    secondLocation = tester.getCenter(find.text('Target'));
    gesture = await tester.startGesture(firstLocation, pointer: 7);
    await tester.pump();
    await gesture.moveTo(secondLocation);
    await tester.pump(); // scrolls off screen!
    await gesture.up();
    await tester.pump();
    expect(events, equals(<String>[]));
    expect(find.text('Target'), findsNothing);
    events.clear();
  });

  testWidgets('Drag and drop - horizontal and vertical draggables in horizontal block', (
    WidgetTester tester,
  ) async {
    final List<String> events = <String>[];
    Offset firstLocation, secondLocation, thirdLocation;

    await tester.pumpWidget(
      MaterialApp(
        home: ListView(
          dragStartBehavior: DragStartBehavior.down,
          scrollDirection: Axis.horizontal,
          children: <Widget>[
            DragTarget<int>(
              builder: (BuildContext context, List<int?> data, List<dynamic> rejects) {
                return const Text('Target');
              },
              onAccept: (int? data) {
                events.add('drop $data');
              },
              onAcceptWithDetails: (DragTargetDetails<int> _) {
                events.add('details');
              },
            ),
            Container(width: 400.0),
            const Draggable<int>(
              data: 1,
              feedback: Text('Dragging'),
              affinity: Axis.horizontal,
              child: Text('H'),
            ),
            const Draggable<int>(
              data: 2,
              feedback: Text('Dragging'),
              affinity: Axis.vertical,
              child: Text('V'),
            ),
            Container(width: 500.0),
            Container(width: 500.0),
            Container(width: 500.0),
            Container(width: 500.0),
          ],
        ),
      ),
    );

    expect(events, isEmpty);
    expect(find.text('Target'), findsOneWidget);
    expect(find.text('H'), findsOneWidget);
    expect(find.text('V'), findsOneWidget);

    // horizontal draggable drags horizontally
    expect(events, isEmpty);
    firstLocation = tester.getCenter(find.text('H'));
    secondLocation = tester.getCenter(find.text('Target'));
    TestGesture gesture = await tester.startGesture(firstLocation, pointer: 7);
    await tester.pump();
    await gesture.moveTo(secondLocation);
    await tester.pump();
    await gesture.up();
    await tester.pump();
    expect(events, equals(<String>['drop 1', 'details']));
    expect(tester.getCenter(find.text('Target')).dx, greaterThan(0.0));
    events.clear();

    // vertical draggable drags vertically
    expect(events, isEmpty);
    firstLocation = tester.getTopLeft(find.text('V'));
    secondLocation = tester.getBottomLeft(find.text('V'));
    thirdLocation = tester.getCenter(find.text('Target'));
    gesture = await tester.startGesture(firstLocation, pointer: 7);
    await tester.pump();
    await gesture.moveTo(secondLocation);
    await tester.pump();
    await gesture.moveTo(thirdLocation);
    await tester.pump();
    await gesture.up();
    await tester.pump();
    expect(events, equals(<String>['drop 2', 'details']));
    expect(tester.getCenter(find.text('Target')).dx, greaterThan(0.0));
    events.clear();

    // horizontal draggable drags vertically when there's no competition
    // from other gesture detectors
    expect(events, isEmpty);
    firstLocation = tester.getTopLeft(find.text('H'));
    secondLocation = tester.getBottomLeft(find.text('H'));
    thirdLocation = tester.getCenter(find.text('Target'));
    gesture = await tester.startGesture(firstLocation, pointer: 7);
    await tester.pump();
    await gesture.moveTo(secondLocation);
    await tester.pump();
    await gesture.moveTo(thirdLocation);
    await tester.pump();
    await gesture.up();
    await tester.pump();
    expect(events, equals(<String>['drop 1', 'details']));
    expect(tester.getCenter(find.text('Target')).dx, greaterThan(0.0));
    events.clear();

    // vertical draggable doesn't drag horizontally when there is competition
    // for horizontal gestures
    expect(events, isEmpty);
    firstLocation = tester.getCenter(find.text('V'));
    secondLocation = tester.getCenter(find.text('Target'));
    gesture = await tester.startGesture(firstLocation, pointer: 7);
    await tester.pump();
    await gesture.moveTo(secondLocation);
    await tester.pump(); // scrolls off screen!
    await gesture.up();
    await tester.pump();
    expect(events, equals(<String>[]));
    expect(find.text('Target'), findsNothing);
    events.clear();
  });

  group('Drag and drop - Draggables with a set axis only move along that axis', () {
    final List<String> events = <String>[];

    Widget build() {
      return MaterialApp(
        home: ListView(
          scrollDirection: Axis.horizontal,
          children: <Widget>[
            DragTarget<int>(
              builder: (BuildContext context, List<int?> data, List<dynamic> rejects) {
                return const Text('Target');
              },
              onAccept: (int? data) {
                events.add('drop $data');
              },
              onAcceptWithDetails: (DragTargetDetails<int> _) {
                events.add('details');
              },
            ),
            Container(width: 400.0),
            const Draggable<int>(
              data: 1,
              feedback: Text('H'),
              childWhenDragging: SizedBox(),
              axis: Axis.horizontal,
              child: Text('H'),
            ),
            const Draggable<int>(
              data: 2,
              feedback: Text('V'),
              childWhenDragging: SizedBox(),
              axis: Axis.vertical,
              child: Text('V'),
            ),
            const Draggable<int>(
              data: 3,
              feedback: Text('N'),
              childWhenDragging: SizedBox(),
              child: Text('N'),
            ),
            Container(width: 500.0),
            Container(width: 500.0),
            Container(width: 500.0),
            Container(width: 500.0),
          ],
        ),
      );
    }

    testWidgets('Null axis draggable moves along all axes', (WidgetTester tester) async {
      await tester.pumpWidget(build());
      final Offset firstLocation = tester.getTopLeft(find.text('N'));
      final Offset secondLocation = firstLocation + const Offset(300.0, 300.0);
      final Offset thirdLocation = firstLocation + const Offset(-300.0, -300.0);
      final TestGesture gesture = await tester.startGesture(firstLocation, pointer: 7);
      await tester.pump();
      await gesture.moveTo(secondLocation);
      await tester.pump();
      expect(tester.getTopLeft(find.text('N')), secondLocation);
      await gesture.moveTo(thirdLocation);
      await tester.pump();
      expect(tester.getTopLeft(find.text('N')), thirdLocation);
    });

    testWidgets('Horizontal axis draggable moves horizontally', (WidgetTester tester) async {
      await tester.pumpWidget(build());
      final Offset firstLocation = tester.getTopLeft(find.text('H'));
      final Offset secondLocation = firstLocation + const Offset(300.0, 0.0);
      final Offset thirdLocation = firstLocation + const Offset(-300.0, 0.0);
      final TestGesture gesture = await tester.startGesture(firstLocation, pointer: 7);
      await tester.pump();
      await gesture.moveTo(secondLocation);
      await tester.pump();
      expect(tester.getTopLeft(find.text('H')), secondLocation);
      await gesture.moveTo(thirdLocation);
      await tester.pump();
      expect(tester.getTopLeft(find.text('H')), thirdLocation);
    });

    testWidgets('Horizontal axis draggable does not move vertically', (WidgetTester tester) async {
      await tester.pumpWidget(build());
      final Offset firstLocation = tester.getTopLeft(find.text('H'));
      final Offset secondDragLocation = firstLocation + const Offset(300.0, 200.0);
      // The horizontal drag widget won't scroll vertically.
      final Offset secondWidgetLocation = firstLocation + const Offset(300.0, 0.0);
      final Offset thirdDragLocation = firstLocation + const Offset(-300.0, -200.0);
      final Offset thirdWidgetLocation = firstLocation + const Offset(-300.0, 0.0);
      final TestGesture gesture = await tester.startGesture(firstLocation, pointer: 7);
      await tester.pump();
      await gesture.moveTo(secondDragLocation);
      await tester.pump();
      expect(tester.getTopLeft(find.text('H')), secondWidgetLocation);
      await gesture.moveTo(thirdDragLocation);
      await tester.pump();
      expect(tester.getTopLeft(find.text('H')), thirdWidgetLocation);
    });

    testWidgets('Vertical axis draggable moves vertically', (WidgetTester tester) async {
      await tester.pumpWidget(build());
      final Offset firstLocation = tester.getTopLeft(find.text('V'));
      final Offset secondLocation = firstLocation + const Offset(0.0, 300.0);
      final Offset thirdLocation = firstLocation + const Offset(0.0, -300.0);
      final TestGesture gesture = await tester.startGesture(firstLocation, pointer: 7);
      await tester.pump();
      await gesture.moveTo(secondLocation);
      await tester.pump();
      expect(tester.getTopLeft(find.text('V')), secondLocation);
      await gesture.moveTo(thirdLocation);
      await tester.pump();
      expect(tester.getTopLeft(find.text('V')), thirdLocation);
    });

    testWidgets('Vertical axis draggable does not move horizontally', (WidgetTester tester) async {
      await tester.pumpWidget(build());
      final Offset firstLocation = tester.getTopLeft(find.text('V'));
      final Offset secondDragLocation = firstLocation + const Offset(200.0, 300.0);
      // The vertical drag widget won't scroll horizontally.
      final Offset secondWidgetLocation = firstLocation + const Offset(0.0, 300.0);
      final Offset thirdDragLocation = firstLocation + const Offset(-200.0, -300.0);
      final Offset thirdWidgetLocation = firstLocation + const Offset(0.0, -300.0);
      final TestGesture gesture = await tester.startGesture(firstLocation, pointer: 7);
      await tester.pump();
      await gesture.moveTo(secondDragLocation);
      await tester.pump();
      expect(tester.getTopLeft(find.text('V')), secondWidgetLocation);
      await gesture.moveTo(thirdDragLocation);
      await tester.pump();
      expect(tester.getTopLeft(find.text('V')), thirdWidgetLocation);
    });
  });

  group('Drag and drop - onDragUpdate called if draggable moves along a set axis', () {
    int updated = 0;
    Offset dragDelta = Offset.zero;

    setUp(() {
      updated = 0;
      dragDelta = Offset.zero;
    });

    Widget build() {
      return MaterialApp(
        home: Column(
          children: <Widget>[
            Draggable<int>(
              data: 1,
              feedback: const Text('Dragging'),
              onDragUpdate: (DragUpdateDetails details) {
                dragDelta += details.delta;
                updated++;
              },
              child: const Text('Source'),
            ),
            Draggable<int>(
              data: 2,
              feedback: const Text('Vertical Dragging'),
              onDragUpdate: (DragUpdateDetails details) {
                dragDelta += details.delta;
                updated++;
              },
              axis: Axis.vertical,
              child: const Text('Vertical Source'),
            ),
            Draggable<int>(
              data: 3,
              feedback: const Text('Horizontal Dragging'),
              onDragUpdate: (DragUpdateDetails details) {
                dragDelta += details.delta;
                updated++;
              },
              axis: Axis.horizontal,
              child: const Text('Horizontal Source'),
            ),
          ],
        ),
      );
    }

    testWidgets('Null axis onDragUpdate called only if draggable moves in any direction', (
      WidgetTester tester,
    ) async {
      await tester.pumpWidget(build());

      expect(updated, 0);
      expect(find.text('Source'), findsOneWidget);
      expect(find.text('Dragging'), findsNothing);

      final Offset firstLocation = tester.getCenter(find.text('Source'));
      final TestGesture gesture = await tester.startGesture(firstLocation, pointer: 7);
      await tester.pump();

      expect(updated, 0);
      expect(find.text('Source'), findsOneWidget);
      expect(find.text('Dragging'), findsOneWidget);

      await gesture.moveBy(const Offset(10, 10));
      await tester.pump();

      expect(updated, 1);

      await gesture.moveBy(Offset.zero);
      await tester.pump();

      expect(updated, 1);

      await gesture.up();
      await tester.pump();

      expect(updated, 1);
      expect(find.text('Source'), findsOneWidget);
      expect(find.text('Dragging'), findsNothing);
      expect(dragDelta.dx, 10);
      expect(dragDelta.dy, 10);
    });

    testWidgets('Vertical axis onDragUpdate only called if draggable moves vertical', (
      WidgetTester tester,
    ) async {
      await tester.pumpWidget(build());

      expect(updated, 0);
      expect(find.text('Vertical Source'), findsOneWidget);
      expect(find.text('Vertical Dragging'), findsNothing);

      final Offset firstLocation = tester.getCenter(find.text('Vertical Source'));
      final TestGesture gesture = await tester.startGesture(firstLocation, pointer: 7);
      await tester.pump();

      expect(updated, 0);
      expect(find.text('Vertical Source'), findsOneWidget);
      expect(find.text('Vertical Dragging'), findsOneWidget);

      await gesture.moveBy(const Offset(0, 10));
      await tester.pump();

      expect(updated, 1);

      await gesture.moveBy(const Offset(10, 0));
      await tester.pump();

      expect(updated, 1);

      await gesture.up();
      await tester.pump();

      expect(updated, 1);
      expect(find.text('Vertical Source'), findsOneWidget);
      expect(find.text('Vertical Dragging'), findsNothing);
      expect(dragDelta.dx, 0);
      expect(dragDelta.dy, 10);
    });

    testWidgets('Horizontal axis onDragUpdate only called if draggable moves horizontal', (
      WidgetTester tester,
    ) async {
      await tester.pumpWidget(build());

      expect(updated, 0);
      expect(find.text('Horizontal Source'), findsOneWidget);
      expect(find.text('Horizontal Dragging'), findsNothing);

      final Offset firstLocation = tester.getCenter(find.text('Horizontal Source'));
      final TestGesture gesture = await tester.startGesture(firstLocation, pointer: 7);
      await tester.pump();

      expect(updated, 0);
      expect(find.text('Horizontal Source'), findsOneWidget);
      expect(find.text('Horizontal Dragging'), findsOneWidget);

      await gesture.moveBy(const Offset(0, 10));
      await tester.pump();

      expect(updated, 0);

      await gesture.moveBy(const Offset(10, 0));
      await tester.pump();

      expect(updated, 1);

      await gesture.up();
      await tester.pump();

      expect(updated, 1);
      expect(find.text('Horizontal Source'), findsOneWidget);
      expect(find.text('Horizontal Dragging'), findsNothing);
      expect(dragDelta.dx, 10);
      expect(dragDelta.dy, 0);
    });
  });

  testWidgets('Drag and drop - onDraggableCanceled not called if dropped on accepting target', (
    WidgetTester tester,
  ) async {
    final List<int> accepted = <int>[];
    final List<DragTargetDetails<int>> acceptedDetails = <DragTargetDetails<int>>[];
    bool onDraggableCanceledCalled = false;

    await tester.pumpWidget(
      MaterialApp(
        home: Column(
          children: <Widget>[
            Draggable<int>(
              data: 1,
              feedback: const Text('Dragging'),
              onDraggableCanceled: (Velocity velocity, Offset offset) {
                onDraggableCanceledCalled = true;
              },
              child: const Text('Source'),
            ),
            DragTarget<int>(
              builder: (BuildContext context, List<int?> data, List<dynamic> rejects) {
                return const SizedBox(height: 100.0, child: Text('Target'));
              },
              onAccept: accepted.add,
              onAcceptWithDetails: acceptedDetails.add,
            ),
          ],
        ),
      ),
    );

    expect(accepted, isEmpty);
    expect(acceptedDetails, isEmpty);
    expect(find.text('Source'), findsOneWidget);
    expect(find.text('Dragging'), findsNothing);
    expect(find.text('Target'), findsOneWidget);
    expect(onDraggableCanceledCalled, isFalse);

    final Offset firstLocation = tester.getCenter(find.text('Source'));
    final TestGesture gesture = await tester.startGesture(firstLocation, pointer: 7);
    await tester.pump();

    expect(accepted, isEmpty);
    expect(acceptedDetails, isEmpty);
    expect(find.text('Source'), findsOneWidget);
    expect(find.text('Dragging'), findsOneWidget);
    expect(find.text('Target'), findsOneWidget);
    expect(onDraggableCanceledCalled, isFalse);

    final Offset secondLocation = tester.getCenter(find.text('Target'));
    await gesture.moveTo(secondLocation);
    await tester.pump();

    expect(accepted, isEmpty);
    expect(acceptedDetails, isEmpty);
    expect(find.text('Source'), findsOneWidget);
    expect(find.text('Dragging'), findsOneWidget);
    expect(find.text('Target'), findsOneWidget);
    expect(onDraggableCanceledCalled, isFalse);

    await gesture.up();
    await tester.pump();

    expect(accepted, equals(<int>[1]));
    expect(acceptedDetails, hasLength(1));
    expect(acceptedDetails.first.offset, const Offset(256.0, 74.0));
    expect(find.text('Source'), findsOneWidget);
    expect(find.text('Dragging'), findsNothing);
    expect(find.text('Target'), findsOneWidget);
    expect(onDraggableCanceledCalled, isFalse);
  });

  testWidgets('Drag and drop - onDraggableCanceled called if dropped on non-accepting target', (
    WidgetTester tester,
  ) async {
    final List<int> accepted = <int>[];
    final List<DragTargetDetails<int>> acceptedDetails = <DragTargetDetails<int>>[];
    bool onDraggableCanceledCalled = false;
    late Velocity onDraggableCanceledVelocity;
    late Offset onDraggableCanceledOffset;

    await tester.pumpWidget(
      MaterialApp(
        home: Column(
          children: <Widget>[
            Draggable<int>(
              data: 1,
              feedback: const Text('Dragging'),
              onDraggableCanceled: (Velocity velocity, Offset offset) {
                onDraggableCanceledCalled = true;
                onDraggableCanceledVelocity = velocity;
                onDraggableCanceledOffset = offset;
              },
              child: const Text('Source'),
            ),
            DragTarget<int>(
              builder: (BuildContext context, List<int?> data, List<dynamic> rejects) {
                return const SizedBox(height: 100.0, child: Text('Target'));
              },
              onWillAccept: (int? data) => false,
              onAccept: accepted.add,
              onAcceptWithDetails: acceptedDetails.add,
            ),
          ],
        ),
      ),
    );

    expect(accepted, isEmpty);
    expect(acceptedDetails, isEmpty);
    expect(find.text('Source'), findsOneWidget);
    expect(find.text('Dragging'), findsNothing);
    expect(find.text('Target'), findsOneWidget);
    expect(onDraggableCanceledCalled, isFalse);

    final Offset firstLocation = tester.getTopLeft(find.text('Source'));
    final TestGesture gesture = await tester.startGesture(firstLocation, pointer: 7);
    await tester.pump();

    expect(accepted, isEmpty);
    expect(acceptedDetails, isEmpty);
    expect(find.text('Source'), findsOneWidget);
    expect(find.text('Dragging'), findsOneWidget);
    expect(find.text('Target'), findsOneWidget);
    expect(onDraggableCanceledCalled, isFalse);

    final Offset secondLocation = tester.getCenter(find.text('Target'));
    await gesture.moveTo(secondLocation);
    await tester.pump();

    expect(accepted, isEmpty);
    expect(acceptedDetails, isEmpty);
    expect(find.text('Source'), findsOneWidget);
    expect(find.text('Dragging'), findsOneWidget);
    expect(find.text('Target'), findsOneWidget);
    expect(onDraggableCanceledCalled, isFalse);

    await gesture.up();
    await tester.pump();

    expect(accepted, isEmpty);
    expect(acceptedDetails, isEmpty);
    expect(find.text('Source'), findsOneWidget);
    expect(find.text('Dragging'), findsNothing);
    expect(find.text('Target'), findsOneWidget);
    expect(onDraggableCanceledCalled, isTrue);
    expect(onDraggableCanceledVelocity, equals(Velocity.zero));
    expect(onDraggableCanceledOffset, equals(Offset(secondLocation.dx, secondLocation.dy)));
  });

  testWidgets(
    'Drag and drop - onDraggableCanceled called if dropped on non-accepting target with details',
    (WidgetTester tester) async {
      final List<int> accepted = <int>[];
      final List<DragTargetDetails<int>> acceptedDetails = <DragTargetDetails<int>>[];
      bool onDraggableCanceledCalled = false;
      late Velocity onDraggableCanceledVelocity;
      late Offset onDraggableCanceledOffset;

      await tester.pumpWidget(
        MaterialApp(
          home: Column(
            children: <Widget>[
              Draggable<int>(
                data: 1,
                feedback: const Text('Dragging'),
                onDraggableCanceled: (Velocity velocity, Offset offset) {
                  onDraggableCanceledCalled = true;
                  onDraggableCanceledVelocity = velocity;
                  onDraggableCanceledOffset = offset;
                },
                child: const Text('Source'),
              ),
              DragTarget<int>(
                builder: (BuildContext context, List<int?> data, List<dynamic> rejects) {
                  return const SizedBox(height: 100.0, child: Text('Target'));
                },
                onWillAcceptWithDetails: (DragTargetDetails<int> details) => false,
                onAccept: accepted.add,
                onAcceptWithDetails: acceptedDetails.add,
              ),
            ],
          ),
        ),
      );

      expect(accepted, isEmpty);
      expect(acceptedDetails, isEmpty);
      expect(find.text('Source'), findsOneWidget);
      expect(find.text('Dragging'), findsNothing);
      expect(find.text('Target'), findsOneWidget);
      expect(onDraggableCanceledCalled, isFalse);

      final Offset firstLocation = tester.getTopLeft(find.text('Source'));
      final TestGesture gesture = await tester.startGesture(firstLocation, pointer: 7);
      await tester.pump();

      expect(accepted, isEmpty);
      expect(acceptedDetails, isEmpty);
      expect(find.text('Source'), findsOneWidget);
      expect(find.text('Dragging'), findsOneWidget);
      expect(find.text('Target'), findsOneWidget);
      expect(onDraggableCanceledCalled, isFalse);

      final Offset secondLocation = tester.getCenter(find.text('Target'));
      await gesture.moveTo(secondLocation);
      await tester.pump();

      expect(accepted, isEmpty);
      expect(acceptedDetails, isEmpty);
      expect(find.text('Source'), findsOneWidget);
      expect(find.text('Dragging'), findsOneWidget);
      expect(find.text('Target'), findsOneWidget);
      expect(onDraggableCanceledCalled, isFalse);

      await gesture.up();
      await tester.pump();

      expect(accepted, isEmpty);
      expect(acceptedDetails, isEmpty);
      expect(find.text('Source'), findsOneWidget);
      expect(find.text('Dragging'), findsNothing);
      expect(find.text('Target'), findsOneWidget);
      expect(onDraggableCanceledCalled, isTrue);
      expect(onDraggableCanceledVelocity, equals(Velocity.zero));
      expect(onDraggableCanceledOffset, equals(Offset(secondLocation.dx, secondLocation.dy)));
    },
  );

  testWidgets(
    'Drag and drop - onDraggableCanceled called if dropped on non-accepting target with correct velocity',
    (WidgetTester tester) async {
      final List<int> accepted = <int>[];
      final List<DragTargetDetails<int>> acceptedDetails = <DragTargetDetails<int>>[];
      bool onDraggableCanceledCalled = false;
      late Velocity onDraggableCanceledVelocity;
      late Offset onDraggableCanceledOffset;

      await tester.pumpWidget(
        MaterialApp(
          home: Column(
            children: <Widget>[
              Draggable<int>(
                data: 1,
                feedback: const Text('Source'),
                onDraggableCanceled: (Velocity velocity, Offset offset) {
                  onDraggableCanceledCalled = true;
                  onDraggableCanceledVelocity = velocity;
                  onDraggableCanceledOffset = offset;
                },
                child: const Text('Source'),
              ),
              DragTarget<int>(
                builder: (BuildContext context, List<int?> data, List<dynamic> rejects) {
                  return const SizedBox(height: 100.0, child: Text('Target'));
                },
                onWillAccept: (int? data) => false,
                onAccept: accepted.add,
                onAcceptWithDetails: acceptedDetails.add,
              ),
            ],
          ),
        ),
      );

      expect(accepted, isEmpty);
      expect(acceptedDetails, isEmpty);
      expect(find.text('Source'), findsOneWidget);
      expect(find.text('Dragging'), findsNothing);
      expect(find.text('Target'), findsOneWidget);
      expect(onDraggableCanceledCalled, isFalse);

      final Offset flingStart = tester.getTopLeft(find.text('Source'));
      await tester.flingFrom(flingStart, const Offset(0.0, 100.0), 1000.0);
      await tester.pump();

      expect(accepted, isEmpty);
      expect(acceptedDetails, isEmpty);
      expect(find.text('Source'), findsOneWidget);
      expect(find.text('Dragging'), findsNothing);
      expect(find.text('Target'), findsOneWidget);
      expect(onDraggableCanceledCalled, isTrue);
      expect(onDraggableCanceledVelocity.pixelsPerSecond.dx.abs(), lessThan(0.0000001));
      expect((onDraggableCanceledVelocity.pixelsPerSecond.dy - 1000.0).abs(), lessThan(0.0000001));
      expect(
        onDraggableCanceledOffset,
        equals(Offset(flingStart.dx, flingStart.dy) + const Offset(0.0, 100.0)),
      );
    },
  );

  testWidgets('Drag and drop - onDragEnd not called if dropped on non-accepting target', (
    WidgetTester tester,
  ) async {
    final List<int> accepted = <int>[];
    final List<DragTargetDetails<int>> acceptedDetails = <DragTargetDetails<int>>[];
    bool onDragEndCalled = false;
    late DraggableDetails onDragEndDraggableDetails;
    await tester.pumpWidget(
      MaterialApp(
        home: Column(
          children: <Widget>[
            Draggable<int>(
              data: 1,
              feedback: const Text('Dragging'),
              onDragEnd: (DraggableDetails details) {
                onDragEndCalled = true;
                onDragEndDraggableDetails = details;
              },
              child: const Text('Source'),
            ),
            DragTarget<int>(
              builder: (BuildContext context, List<int?> data, List<dynamic> rejects) {
                return const SizedBox(height: 100.0, child: Text('Target'));
              },
              onWillAccept: (int? data) => false,
              onAccept: accepted.add,
              onAcceptWithDetails: acceptedDetails.add,
            ),
          ],
        ),
      ),
    );

    expect(accepted, isEmpty);
    expect(acceptedDetails, isEmpty);
    expect(find.text('Source'), findsOneWidget);
    expect(find.text('Dragging'), findsNothing);
    expect(find.text('Target'), findsOneWidget);
    expect(onDragEndCalled, isFalse);

    final Offset firstLocation = tester.getTopLeft(find.text('Source'));
    final TestGesture gesture = await tester.startGesture(firstLocation, pointer: 7);
    await tester.pump();

    expect(accepted, isEmpty);
    expect(acceptedDetails, isEmpty);
    expect(find.text('Source'), findsOneWidget);
    expect(find.text('Dragging'), findsOneWidget);
    expect(find.text('Target'), findsOneWidget);
    expect(onDragEndCalled, isFalse);

    final Offset secondLocation = tester.getCenter(find.text('Target'));
    await gesture.moveTo(secondLocation);
    await tester.pump();

    expect(accepted, isEmpty);
    expect(acceptedDetails, isEmpty);
    expect(find.text('Source'), findsOneWidget);
    expect(find.text('Dragging'), findsOneWidget);
    expect(find.text('Target'), findsOneWidget);
    expect(onDragEndCalled, isFalse);

    await gesture.up();
    await tester.pump();

    expect(accepted, isEmpty);
    expect(acceptedDetails, isEmpty);
    expect(find.text('Source'), findsOneWidget);
    expect(find.text('Dragging'), findsNothing);
    expect(find.text('Target'), findsOneWidget);
    expect(onDragEndCalled, isTrue);
    expect(onDragEndDraggableDetails, isNotNull);
    expect(onDragEndDraggableDetails.wasAccepted, isFalse);
    expect(onDragEndDraggableDetails.velocity, equals(Velocity.zero));
    expect(
      onDragEndDraggableDetails.offset,
      equals(Offset(secondLocation.dx, secondLocation.dy - firstLocation.dy)),
    );
  });

  testWidgets(
    'Drag and drop - onDragEnd not called if dropped on non-accepting target with details',
    (WidgetTester tester) async {
      final List<int> accepted = <int>[];
      final List<DragTargetDetails<int>> acceptedDetails = <DragTargetDetails<int>>[];
      bool onDragEndCalled = false;
      late DraggableDetails onDragEndDraggableDetails;
      await tester.pumpWidget(
        MaterialApp(
          home: Column(
            children: <Widget>[
              Draggable<int>(
                data: 1,
                feedback: const Text('Dragging'),
                onDragEnd: (DraggableDetails details) {
                  onDragEndCalled = true;
                  onDragEndDraggableDetails = details;
                },
                child: const Text('Source'),
              ),
              DragTarget<int>(
                builder: (BuildContext context, List<int?> data, List<dynamic> rejects) {
                  return const SizedBox(height: 100.0, child: Text('Target'));
                },
                onWillAcceptWithDetails: (DragTargetDetails<int> data) => false,
                onAccept: accepted.add,
                onAcceptWithDetails: acceptedDetails.add,
              ),
            ],
          ),
        ),
      );

      expect(accepted, isEmpty);
      expect(acceptedDetails, isEmpty);
      expect(find.text('Source'), findsOneWidget);
      expect(find.text('Dragging'), findsNothing);
      expect(find.text('Target'), findsOneWidget);
      expect(onDragEndCalled, isFalse);

      final Offset firstLocation = tester.getTopLeft(find.text('Source'));
      final TestGesture gesture = await tester.startGesture(firstLocation, pointer: 7);
      await tester.pump();

      expect(accepted, isEmpty);
      expect(acceptedDetails, isEmpty);
      expect(find.text('Source'), findsOneWidget);
      expect(find.text('Dragging'), findsOneWidget);
      expect(find.text('Target'), findsOneWidget);
      expect(onDragEndCalled, isFalse);

      final Offset secondLocation = tester.getCenter(find.text('Target'));
      await gesture.moveTo(secondLocation);
      await tester.pump();

      expect(accepted, isEmpty);
      expect(acceptedDetails, isEmpty);
      expect(find.text('Source'), findsOneWidget);
      expect(find.text('Dragging'), findsOneWidget);
      expect(find.text('Target'), findsOneWidget);
      expect(onDragEndCalled, isFalse);

      await gesture.up();
      await tester.pump();

      expect(accepted, isEmpty);
      expect(acceptedDetails, isEmpty);
      expect(find.text('Source'), findsOneWidget);
      expect(find.text('Dragging'), findsNothing);
      expect(find.text('Target'), findsOneWidget);
      expect(onDragEndCalled, isTrue);
      expect(onDragEndDraggableDetails, isNotNull);
      expect(onDragEndDraggableDetails.wasAccepted, isFalse);
      expect(onDragEndDraggableDetails.velocity, equals(Velocity.zero));
      expect(
        onDragEndDraggableDetails.offset,
        equals(Offset(secondLocation.dx, secondLocation.dy - firstLocation.dy)),
      );
    },
  );

  testWidgets(
    'Drag and drop - DragTarget rebuilds with and without rejected data when a rejected draggable enters and leaves',
    (WidgetTester tester) async {
      await tester.pumpWidget(
        MaterialApp(
          home: Column(
            children: <Widget>[
              const Draggable<int>(data: 1, feedback: Text('Dragging'), child: Text('Source')),
              DragTarget<int>(
                builder: (BuildContext context, List<int?> data, List<dynamic> rejects) {
                  return SizedBox(
                    height: 100.0,
                    child: rejects.isNotEmpty ? const Text('Rejected') : const Text('Target'),
                  );
                },
                onWillAccept: (int? data) => false,
              ),
            ],
          ),
        ),
      );

      expect(find.text('Dragging'), findsNothing);
      expect(find.text('Target'), findsOneWidget);
      expect(find.text('Rejected'), findsNothing);

      final Offset firstLocation = tester.getTopLeft(find.text('Source'));
      final TestGesture gesture = await tester.startGesture(firstLocation, pointer: 7);
      await tester.pump();

      expect(find.text('Dragging'), findsOneWidget);
      expect(find.text('Target'), findsOneWidget);
      expect(find.text('Rejected'), findsNothing);

      final Offset secondLocation = tester.getCenter(find.text('Target'));
      await gesture.moveTo(secondLocation);
      await tester.pump();

      expect(find.text('Dragging'), findsOneWidget);
      expect(find.text('Target'), findsNothing);
      expect(find.text('Rejected'), findsOneWidget);

      await gesture.moveTo(firstLocation);
      await tester.pump();

      expect(find.text('Dragging'), findsOneWidget);
      expect(find.text('Target'), findsOneWidget);
      expect(find.text('Rejected'), findsNothing);
    },
  );

  testWidgets('Drag and drop - Can drag and drop over a non-accepting target multiple times', (
    WidgetTester tester,
  ) async {
    int numberOfTimesOnDraggableCanceledCalled = 0;
    await tester.pumpWidget(
      MaterialApp(
        home: Column(
          children: <Widget>[
            Draggable<int>(
              data: 1,
              feedback: const Text('Dragging'),
              onDraggableCanceled: (Velocity velocity, Offset offset) {
                numberOfTimesOnDraggableCanceledCalled++;
              },
              child: const Text('Source'),
            ),
            DragTarget<int>(
              builder: (BuildContext context, List<int?> data, List<dynamic> rejects) {
                return SizedBox(
                  height: 100.0,
                  child: rejects.isNotEmpty ? const Text('Rejected') : const Text('Target'),
                );
              },
              onWillAccept: (int? data) => false,
            ),
          ],
        ),
      ),
    );

    expect(find.text('Dragging'), findsNothing);
    expect(find.text('Target'), findsOneWidget);
    expect(find.text('Rejected'), findsNothing);

    final Offset firstLocation = tester.getTopLeft(find.text('Source'));
    final TestGesture gesture = await tester.startGesture(firstLocation, pointer: 7);
    await tester.pump();

    expect(find.text('Dragging'), findsOneWidget);
    expect(find.text('Target'), findsOneWidget);
    expect(find.text('Rejected'), findsNothing);

    final Offset secondLocation = tester.getCenter(find.text('Target'));
    await gesture.moveTo(secondLocation);
    await tester.pump();

    expect(find.text('Dragging'), findsOneWidget);
    expect(find.text('Target'), findsNothing);
    expect(find.text('Rejected'), findsOneWidget);

    await gesture.up();
    await tester.pump();

    expect(find.text('Dragging'), findsNothing);
    expect(find.text('Target'), findsOneWidget);
    expect(find.text('Rejected'), findsNothing);
    expect(numberOfTimesOnDraggableCanceledCalled, 1);

    // Drag and drop the Draggable onto the Target a second time.
    final TestGesture secondGesture = await tester.startGesture(firstLocation, pointer: 7);
    await tester.pump();

    expect(find.text('Dragging'), findsOneWidget);
    expect(find.text('Target'), findsOneWidget);
    expect(find.text('Rejected'), findsNothing);

    await secondGesture.moveTo(secondLocation);
    await tester.pump();

    expect(find.text('Dragging'), findsOneWidget);
    expect(find.text('Target'), findsNothing);
    expect(find.text('Rejected'), findsOneWidget);

    await secondGesture.up();
    await tester.pump();

    expect(numberOfTimesOnDraggableCanceledCalled, 2);
    expect(find.text('Dragging'), findsNothing);
    expect(find.text('Target'), findsOneWidget);
    expect(find.text('Rejected'), findsNothing);
  });

  testWidgets('Drag and drop - onDragCompleted not called if dropped on non-accepting target', (
    WidgetTester tester,
  ) async {
    final List<int> accepted = <int>[];
    final List<DragTargetDetails<int>> acceptedDetails = <DragTargetDetails<int>>[];
    bool onDragCompletedCalled = false;

    await tester.pumpWidget(
      MaterialApp(
        home: Column(
          children: <Widget>[
            Draggable<int>(
              data: 1,
              feedback: const Text('Dragging'),
              onDragCompleted: () {
                onDragCompletedCalled = true;
              },
              child: const Text('Source'),
            ),
            DragTarget<int>(
              builder: (BuildContext context, List<int?> data, List<dynamic> rejects) {
                return const SizedBox(height: 100.0, child: Text('Target'));
              },
              onWillAccept: (int? data) => false,
              onAccept: accepted.add,
              onAcceptWithDetails: acceptedDetails.add,
            ),
          ],
        ),
      ),
    );

    expect(accepted, isEmpty);
    expect(acceptedDetails, isEmpty);
    expect(find.text('Source'), findsOneWidget);
    expect(find.text('Dragging'), findsNothing);
    expect(find.text('Target'), findsOneWidget);
    expect(onDragCompletedCalled, isFalse);

    final Offset firstLocation = tester.getTopLeft(find.text('Source'));
    final TestGesture gesture = await tester.startGesture(firstLocation, pointer: 7);
    await tester.pump();

    expect(accepted, isEmpty);
    expect(acceptedDetails, isEmpty);
    expect(find.text('Source'), findsOneWidget);
    expect(find.text('Dragging'), findsOneWidget);
    expect(find.text('Target'), findsOneWidget);
    expect(onDragCompletedCalled, isFalse);

    final Offset secondLocation = tester.getCenter(find.text('Target'));
    await gesture.moveTo(secondLocation);
    await tester.pump();

    expect(accepted, isEmpty);
    expect(acceptedDetails, isEmpty);
    expect(find.text('Source'), findsOneWidget);
    expect(find.text('Dragging'), findsOneWidget);
    expect(find.text('Target'), findsOneWidget);
    expect(onDragCompletedCalled, isFalse);

    await gesture.up();
    await tester.pump();

    expect(accepted, isEmpty);
    expect(acceptedDetails, isEmpty);
    expect(find.text('Source'), findsOneWidget);
    expect(find.text('Dragging'), findsNothing);
    expect(find.text('Target'), findsOneWidget);
    expect(onDragCompletedCalled, isFalse);
  });

  testWidgets(
    'Drag and drop - onDragCompleted not called if dropped on non-accepting target with details',
    (WidgetTester tester) async {
      final List<int> accepted = <int>[];
      final List<DragTargetDetails<int>> acceptedDetails = <DragTargetDetails<int>>[];
      bool onDragCompletedCalled = false;

      await tester.pumpWidget(
        MaterialApp(
          home: Column(
            children: <Widget>[
              Draggable<int>(
                data: 1,
                feedback: const Text('Dragging'),
                onDragCompleted: () {
                  onDragCompletedCalled = true;
                },
                child: const Text('Source'),
              ),
              DragTarget<int>(
                builder: (BuildContext context, List<int?> data, List<dynamic> rejects) {
                  return const SizedBox(height: 100.0, child: Text('Target'));
                },
                onWillAcceptWithDetails: (DragTargetDetails<int> data) => false,
                onAccept: accepted.add,
                onAcceptWithDetails: acceptedDetails.add,
              ),
            ],
          ),
        ),
      );

      expect(accepted, isEmpty);
      expect(acceptedDetails, isEmpty);
      expect(find.text('Source'), findsOneWidget);
      expect(find.text('Dragging'), findsNothing);
      expect(find.text('Target'), findsOneWidget);
      expect(onDragCompletedCalled, isFalse);

      final Offset firstLocation = tester.getTopLeft(find.text('Source'));
      final TestGesture gesture = await tester.startGesture(firstLocation, pointer: 7);
      await tester.pump();

      expect(accepted, isEmpty);
      expect(acceptedDetails, isEmpty);
      expect(find.text('Source'), findsOneWidget);
      expect(find.text('Dragging'), findsOneWidget);
      expect(find.text('Target'), findsOneWidget);
      expect(onDragCompletedCalled, isFalse);

      final Offset secondLocation = tester.getCenter(find.text('Target'));
      await gesture.moveTo(secondLocation);
      await tester.pump();

      expect(accepted, isEmpty);
      expect(acceptedDetails, isEmpty);
      expect(find.text('Source'), findsOneWidget);
      expect(find.text('Dragging'), findsOneWidget);
      expect(find.text('Target'), findsOneWidget);
      expect(onDragCompletedCalled, isFalse);

      await gesture.up();
      await tester.pump();

      expect(accepted, isEmpty);
      expect(acceptedDetails, isEmpty);
      expect(find.text('Source'), findsOneWidget);
      expect(find.text('Dragging'), findsNothing);
      expect(find.text('Target'), findsOneWidget);
      expect(onDragCompletedCalled, isFalse);
    },
  );

  testWidgets('Drag and drop - onDragEnd called if dropped on accepting target', (
    WidgetTester tester,
  ) async {
    final List<int> accepted = <int>[];
    final List<DragTargetDetails<int>> acceptedDetails = <DragTargetDetails<int>>[];
    bool onDragEndCalled = false;
    late DraggableDetails onDragEndDraggableDetails;
    await tester.pumpWidget(
      MaterialApp(
        home: Column(
          children: <Widget>[
            Draggable<int>(
              data: 1,
              feedback: const Text('Dragging'),
              onDragEnd: (DraggableDetails details) {
                onDragEndCalled = true;
                onDragEndDraggableDetails = details;
              },
              child: const Text('Source'),
            ),
            DragTarget<int>(
              builder: (BuildContext context, List<int?> data, List<dynamic> rejects) {
                return const SizedBox(height: 100.0, child: Text('Target'));
              },
              onAccept: accepted.add,
              onAcceptWithDetails: acceptedDetails.add,
            ),
          ],
        ),
      ),
    );

    expect(accepted, isEmpty);
    expect(acceptedDetails, isEmpty);
    expect(find.text('Source'), findsOneWidget);
    expect(find.text('Dragging'), findsNothing);
    expect(find.text('Target'), findsOneWidget);
    expect(onDragEndCalled, isFalse);

    final Offset firstLocation = tester.getCenter(find.text('Source'));
    final TestGesture gesture = await tester.startGesture(firstLocation, pointer: 7);
    await tester.pump();

    expect(accepted, isEmpty);
    expect(acceptedDetails, isEmpty);
    expect(find.text('Source'), findsOneWidget);
    expect(find.text('Dragging'), findsOneWidget);
    expect(find.text('Target'), findsOneWidget);
    expect(onDragEndCalled, isFalse);

    final Offset secondLocation = tester.getCenter(find.text('Target'));
    await gesture.moveTo(secondLocation);
    await tester.pump();

    expect(accepted, isEmpty);
    expect(acceptedDetails, isEmpty);
    expect(find.text('Source'), findsOneWidget);
    expect(find.text('Dragging'), findsOneWidget);
    expect(find.text('Target'), findsOneWidget);
    expect(onDragEndCalled, isFalse);

    await gesture.up();
    await tester.pump();

    final Offset droppedLocation = tester.getTopLeft(find.text('Target'));
    final Offset expectedDropOffset = Offset(
      droppedLocation.dx,
      secondLocation.dy - firstLocation.dy,
    );

    expect(accepted, equals(<int>[1]));
    expect(acceptedDetails, hasLength(1));
    expect(acceptedDetails.first.offset, const Offset(256.0, 74.0));
    expect(find.text('Source'), findsOneWidget);
    expect(find.text('Dragging'), findsNothing);
    expect(find.text('Target'), findsOneWidget);
    expect(onDragEndCalled, isTrue);
    expect(onDragEndDraggableDetails, isNotNull);
    expect(onDragEndDraggableDetails.wasAccepted, isTrue);
    expect(onDragEndDraggableDetails.velocity, equals(Velocity.zero));
    expect(onDragEndDraggableDetails.offset, equals(expectedDropOffset));
  });

  testWidgets('DragTarget does not call onDragEnd when remove from the tree', (
    WidgetTester tester,
  ) async {
    final List<String> events = <String>[];
    Offset firstLocation, secondLocation;
    int timesOnDragEndCalled = 0;
    await tester.pumpWidget(
      MaterialApp(
        home: Column(
          children: <Widget>[
            Draggable<int>(
              data: 1,
              feedback: const Text('Dragging'),
              onDragEnd: (DraggableDetails details) {
                timesOnDragEndCalled++;
              },
              child: const Text('Source'),
            ),
            DragTarget<int>(
              builder: (BuildContext context, List<int?> data, List<dynamic> rejects) {
                return const Text('Target');
              },
              onAccept: (int? data) {
                events.add('drop');
              },
              onAcceptWithDetails: (DragTargetDetails<int> _) {
                events.add('details');
              },
            ),
          ],
        ),
      ),
    );

    expect(events, isEmpty);
    expect(find.text('Source'), findsOneWidget);
    expect(find.text('Target'), findsOneWidget);

    expect(events, isEmpty);
    await tester.tap(find.text('Source'));
    expect(events, isEmpty);

    firstLocation = tester.getCenter(find.text('Source'));
    final TestGesture gesture = await tester.startGesture(firstLocation, pointer: 7);
    await tester.pump();

    await tester.pump(const Duration(seconds: 20));

    secondLocation = tester.getCenter(find.text('Target'));
    await gesture.moveTo(secondLocation);
    await tester.pump();

    await tester.pumpWidget(
      const MaterialApp(
        home: Column(
          children: <Widget>[
            Draggable<int>(data: 1, feedback: Text('Dragging'), child: Text('Source')),
          ],
        ),
      ),
    );

    expect(events, isEmpty);
    expect(timesOnDragEndCalled, equals(1));
    await gesture.up();
    await tester.pump();
  });

  testWidgets('Drag and drop - onDragCompleted called if dropped on accepting target', (
    WidgetTester tester,
  ) async {
    final List<int> accepted = <int>[];
    final List<DragTargetDetails<int>> acceptedDetails = <DragTargetDetails<int>>[];
    bool onDragCompletedCalled = false;

    await tester.pumpWidget(
      MaterialApp(
        home: Column(
          children: <Widget>[
            Draggable<int>(
              data: 1,
              feedback: const Text('Dragging'),
              onDragCompleted: () {
                onDragCompletedCalled = true;
              },
              child: const Text('Source'),
            ),
            DragTarget<int>(
              builder: (BuildContext context, List<int?> data, List<dynamic> rejects) {
                return const SizedBox(height: 100.0, child: Text('Target'));
              },
              onAccept: accepted.add,
              onAcceptWithDetails: acceptedDetails.add,
            ),
          ],
        ),
      ),
    );

    expect(accepted, isEmpty);
    expect(acceptedDetails, isEmpty);
    expect(find.text('Source'), findsOneWidget);
    expect(find.text('Dragging'), findsNothing);
    expect(find.text('Target'), findsOneWidget);
    expect(onDragCompletedCalled, isFalse);

    final Offset firstLocation = tester.getCenter(find.text('Source'));
    final TestGesture gesture = await tester.startGesture(firstLocation, pointer: 7);
    await tester.pump();

    expect(accepted, isEmpty);
    expect(acceptedDetails, isEmpty);
    expect(find.text('Source'), findsOneWidget);
    expect(find.text('Dragging'), findsOneWidget);
    expect(find.text('Target'), findsOneWidget);
    expect(onDragCompletedCalled, isFalse);

    final Offset secondLocation = tester.getCenter(find.text('Target'));
    await gesture.moveTo(secondLocation);
    await tester.pump();

    expect(accepted, isEmpty);
    expect(acceptedDetails, isEmpty);
    expect(find.text('Source'), findsOneWidget);
    expect(find.text('Dragging'), findsOneWidget);
    expect(find.text('Target'), findsOneWidget);
    expect(onDragCompletedCalled, isFalse);

    await gesture.up();
    await tester.pump();

    expect(accepted, equals(<int>[1]));
    expect(acceptedDetails, hasLength(1));
    expect(acceptedDetails.first.offset, const Offset(256.0, 74.0));
    expect(find.text('Source'), findsOneWidget);
    expect(find.text('Dragging'), findsNothing);
    expect(find.text('Target'), findsOneWidget);
    expect(onDragCompletedCalled, isTrue);
  });

  testWidgets('Drag and drop - allow pass through of unaccepted data test', (
    WidgetTester tester,
  ) async {
    final List<int> acceptedInts = <int>[];
    final List<DragTargetDetails<int>> acceptedIntsDetails = <DragTargetDetails<int>>[];
    final List<double> acceptedDoubles = <double>[];
    final List<DragTargetDetails<double>> acceptedDoublesDetails = <DragTargetDetails<double>>[];

    await tester.pumpWidget(
      MaterialApp(
        home: Column(
          children: <Widget>[
            const Draggable<int>(data: 1, feedback: Text('IntDragging'), child: Text('IntSource')),
            const Draggable<double>(
              data: 1.0,
              feedback: Text('DoubleDragging'),
              child: Text('DoubleSource'),
            ),
            Stack(
              children: <Widget>[
                DragTarget<int>(
                  builder: (BuildContext context, List<int?> data, List<dynamic> rejects) {
                    return const IgnorePointer(
                      child: SizedBox(height: 100.0, child: Text('Target1')),
                    );
                  },
                  onAccept: acceptedInts.add,
                  onAcceptWithDetails: acceptedIntsDetails.add,
                ),
                DragTarget<double>(
                  builder: (BuildContext context, List<double?> data, List<dynamic> rejects) {
                    return const IgnorePointer(
                      child: SizedBox(height: 100.0, child: Text('Target2')),
                    );
                  },
                  onAccept: acceptedDoubles.add,
                  onAcceptWithDetails: acceptedDoublesDetails.add,
                ),
              ],
            ),
          ],
        ),
      ),
    );

    expect(acceptedInts, isEmpty);
    expect(acceptedIntsDetails, isEmpty);
    expect(acceptedDoubles, isEmpty);
    expect(acceptedDoublesDetails, isEmpty);
    expect(find.text('IntSource'), findsOneWidget);
    expect(find.text('IntDragging'), findsNothing);
    expect(find.text('DoubleSource'), findsOneWidget);
    expect(find.text('DoubleDragging'), findsNothing);
    expect(find.text('Target1'), findsOneWidget);
    expect(find.text('Target2'), findsOneWidget);

    final Offset intLocation = tester.getCenter(find.text('IntSource'));
    final Offset doubleLocation = tester.getCenter(find.text('DoubleSource'));
    final Offset targetLocation = tester.getCenter(find.text('Target1'));

    // Drag the double draggable.
    final TestGesture doubleGesture = await tester.startGesture(doubleLocation, pointer: 7);
    await tester.pump();

    expect(acceptedInts, isEmpty);
    expect(acceptedIntsDetails, isEmpty);
    expect(acceptedDoubles, isEmpty);
    expect(acceptedDoublesDetails, isEmpty);
    expect(find.text('IntDragging'), findsNothing);
    expect(find.text('DoubleDragging'), findsOneWidget);

    await doubleGesture.moveTo(targetLocation);
    await tester.pump();

    expect(acceptedInts, isEmpty);
    expect(acceptedIntsDetails, isEmpty);
    expect(acceptedDoubles, isEmpty);
    expect(acceptedDoublesDetails, isEmpty);
    expect(find.text('IntDragging'), findsNothing);
    expect(find.text('DoubleDragging'), findsOneWidget);

    await doubleGesture.up();
    await tester.pump();

    expect(acceptedInts, isEmpty);
    expect(acceptedIntsDetails, isEmpty);
    expect(acceptedDoubles, equals(<double>[1.0]));
    expect(acceptedDoublesDetails, hasLength(1));
    expect(acceptedDoublesDetails.first.offset, const Offset(112.0, 122.0));
    expect(find.text('IntDragging'), findsNothing);
    expect(find.text('DoubleDragging'), findsNothing);

    acceptedDoubles.clear();
    acceptedDoublesDetails.clear();

    // Drag the int draggable.
    final TestGesture intGesture = await tester.startGesture(intLocation, pointer: 7);
    await tester.pump();

    expect(acceptedInts, isEmpty);
    expect(acceptedIntsDetails, isEmpty);
    expect(acceptedDoubles, isEmpty);
    expect(acceptedDoublesDetails, isEmpty);
    expect(find.text('IntDragging'), findsOneWidget);
    expect(find.text('DoubleDragging'), findsNothing);

    await intGesture.moveTo(targetLocation);
    await tester.pump();

    expect(acceptedInts, isEmpty);
    expect(acceptedIntsDetails, isEmpty);
    expect(acceptedDoubles, isEmpty);
    expect(acceptedDoublesDetails, isEmpty);
    expect(find.text('IntDragging'), findsOneWidget);
    expect(find.text('DoubleDragging'), findsNothing);

    await intGesture.up();
    await tester.pump();

    expect(acceptedInts, equals(<int>[1]));
    expect(acceptedIntsDetails, hasLength(1));
    expect(acceptedIntsDetails.first.offset, const Offset(184.0, 122.0));
    expect(acceptedDoubles, isEmpty);
    expect(acceptedDoublesDetails, isEmpty);
    expect(find.text('IntDragging'), findsNothing);
    expect(find.text('DoubleDragging'), findsNothing);
  });

  testWidgets('Drag and drop - allow pass through of unaccepted data twice test', (
    WidgetTester tester,
  ) async {
    final List<DragTargetData> acceptedDragTargetDatas = <DragTargetData>[];
    final List<DragTargetDetails<DragTargetData>> acceptedDragTargetDataDetails =
        <DragTargetDetails<DragTargetData>>[];
    final List<ExtendedDragTargetData> acceptedExtendedDragTargetDatas = <ExtendedDragTargetData>[];
    final List<DragTargetDetails<ExtendedDragTargetData>> acceptedExtendedDragTargetDataDetails =
        <DragTargetDetails<ExtendedDragTargetData>>[];
    final DragTargetData dragTargetData = DragTargetData();
    await tester.pumpWidget(
      MaterialApp(
        home: Column(
          children: <Widget>[
            Draggable<DragTargetData>(
              data: dragTargetData,
              feedback: const Text('Dragging'),
              child: const Text('Source'),
            ),
            Stack(
              children: <Widget>[
                DragTarget<DragTargetData>(
                  builder: (
                    BuildContext context,
                    List<DragTargetData?> data,
                    List<dynamic> rejects,
                  ) {
                    return const IgnorePointer(
                      child: SizedBox(height: 100.0, child: Text('Target1')),
                    );
                  },
                  onAccept: acceptedDragTargetDatas.add,
                  onAcceptWithDetails: acceptedDragTargetDataDetails.add,
                ),
                DragTarget<ExtendedDragTargetData>(
                  builder: (
                    BuildContext context,
                    List<ExtendedDragTargetData?> data,
                    List<dynamic> rejects,
                  ) {
                    return const IgnorePointer(
                      child: SizedBox(height: 100.0, child: Text('Target2')),
                    );
                  },
                  onAccept: acceptedExtendedDragTargetDatas.add,
                  onAcceptWithDetails: acceptedExtendedDragTargetDataDetails.add,
                ),
              ],
            ),
          ],
        ),
      ),
    );

    final Offset dragTargetLocation = tester.getCenter(find.text('Source'));
    final Offset targetLocation = tester.getCenter(find.text('Target1'));

    for (int i = 0; i < 2; i += 1) {
      final TestGesture gesture = await tester.startGesture(dragTargetLocation);
      await tester.pump();
      await gesture.moveTo(targetLocation);
      await tester.pump();
      await gesture.up();
      await tester.pump();

      expect(acceptedDragTargetDatas, equals(<DragTargetData>[dragTargetData]));
      expect(acceptedDragTargetDataDetails, hasLength(1));
      expect(acceptedDragTargetDataDetails.first.offset, const Offset(256.0, 74.0));
      expect(acceptedExtendedDragTargetDatas, isEmpty);
      expect(acceptedExtendedDragTargetDataDetails, isEmpty);

      acceptedDragTargetDatas.clear();
      acceptedDragTargetDataDetails.clear();
      await tester.pump();
    }
  });

  testWidgets('Drag and drop - maxSimultaneousDrags', (WidgetTester tester) async {
    final List<int> accepted = <int>[];
    final List<DragTargetDetails<int>> acceptedDetails = <DragTargetDetails<int>>[];

    Widget build(int maxSimultaneousDrags) {
      return MaterialApp(
        home: Column(
          children: <Widget>[
            Draggable<int>(
              data: 1,
              maxSimultaneousDrags: maxSimultaneousDrags,
              feedback: const Text('Dragging'),
              child: const Text('Source'),
            ),
            DragTarget<int>(
              builder: (BuildContext context, List<int?> data, List<dynamic> rejects) {
                return const SizedBox(height: 100.0, child: Text('Target'));
              },
              onAccept: accepted.add,
              onAcceptWithDetails: acceptedDetails.add,
            ),
          ],
        ),
      );
    }

    await tester.pumpWidget(build(0));

    final Offset firstLocation = tester.getCenter(find.text('Source'));
    final Offset secondLocation = tester.getCenter(find.text('Target'));

    expect(accepted, isEmpty);
    expect(acceptedDetails, isEmpty);
    expect(find.text('Source'), findsOneWidget);
    expect(find.text('Dragging'), findsNothing);
    expect(find.text('Target'), findsOneWidget);

    final TestGesture gesture = await tester.startGesture(firstLocation, pointer: 7);
    await tester.pump();

    expect(accepted, isEmpty);
    expect(acceptedDetails, isEmpty);
    expect(find.text('Source'), findsOneWidget);
    expect(find.text('Dragging'), findsNothing);
    expect(find.text('Target'), findsOneWidget);

    await gesture.up();

    await tester.pumpWidget(build(2));

    expect(accepted, isEmpty);
    expect(acceptedDetails, isEmpty);
    expect(find.text('Source'), findsOneWidget);
    expect(find.text('Dragging'), findsNothing);
    expect(find.text('Target'), findsOneWidget);

    final TestGesture gesture1 = await tester.startGesture(firstLocation, pointer: 8);
    await tester.pump();

    expect(accepted, isEmpty);
    expect(acceptedDetails, isEmpty);
    expect(find.text('Source'), findsOneWidget);
    expect(find.text('Dragging'), findsOneWidget);
    expect(find.text('Target'), findsOneWidget);

    final TestGesture gesture2 = await tester.startGesture(firstLocation, pointer: 9);
    await tester.pump();

    expect(accepted, isEmpty);
    expect(acceptedDetails, isEmpty);
    expect(find.text('Source'), findsOneWidget);
    expect(find.text('Dragging'), findsNWidgets(2));
    expect(find.text('Target'), findsOneWidget);

    final TestGesture gesture3 = await tester.startGesture(firstLocation, pointer: 10);
    await tester.pump();

    expect(accepted, isEmpty);
    expect(acceptedDetails, isEmpty);
    expect(find.text('Source'), findsOneWidget);
    expect(find.text('Dragging'), findsNWidgets(2));
    expect(find.text('Target'), findsOneWidget);

    await gesture1.moveTo(secondLocation);
    await gesture2.moveTo(secondLocation);
    await gesture3.moveTo(secondLocation);
    await tester.pump();

    expect(accepted, isEmpty);
    expect(acceptedDetails, isEmpty);
    expect(find.text('Source'), findsOneWidget);
    expect(find.text('Dragging'), findsNWidgets(2));
    expect(find.text('Target'), findsOneWidget);

    await gesture1.up();
    await tester.pump();

    expect(accepted, equals(<int>[1]));
    expect(acceptedDetails, hasLength(1));
    expect(acceptedDetails.first.offset, const Offset(256.0, 74.0));
    expect(find.text('Source'), findsOneWidget);
    expect(find.text('Dragging'), findsOneWidget);
    expect(find.text('Target'), findsOneWidget);

    await gesture2.up();
    await tester.pump();

    expect(accepted, equals(<int>[1, 1]));
    expect(acceptedDetails, hasLength(2));
    expect(acceptedDetails[0].offset, const Offset(256.0, 74.0));
    expect(acceptedDetails[1].offset, const Offset(256.0, 74.0));
    expect(find.text('Source'), findsOneWidget);
    expect(find.text('Dragging'), findsNothing);
    expect(find.text('Target'), findsOneWidget);

    await gesture3.up();
    await tester.pump();

    expect(accepted, equals(<int>[1, 1]));
    expect(acceptedDetails, hasLength(2));
    expect(acceptedDetails[0].offset, const Offset(256.0, 74.0));
    expect(acceptedDetails[1].offset, const Offset(256.0, 74.0));
    expect(find.text('Source'), findsOneWidget);
    expect(find.text('Dragging'), findsNothing);
    expect(find.text('Target'), findsOneWidget);
  });

  testWidgets('Drag and drop - onAccept is not called if dropped with null data', (
    WidgetTester tester,
  ) async {
    bool onAcceptCalled = false;
    bool onAcceptWithDetailsCalled = false;

    await tester.pumpWidget(
      MaterialApp(
        home: Column(
          children: <Widget>[
            const Draggable<int>(feedback: Text('Dragging'), child: Text('Source')),
            DragTarget<int>(
              builder: (BuildContext context, List<int?> data, List<dynamic> rejects) {
                return const SizedBox(height: 100.0, child: Text('Target'));
              },
              onAccept: (int data) {
                onAcceptCalled = true;
              },
              onAcceptWithDetails: (DragTargetDetails<int> details) {
                onAcceptWithDetailsCalled = true;
              },
            ),
          ],
        ),
      ),
    );

    expect(onAcceptCalled, isFalse);
    expect(onAcceptWithDetailsCalled, isFalse);
    expect(find.text('Source'), findsOneWidget);
    expect(find.text('Dragging'), findsNothing);
    expect(find.text('Target'), findsOneWidget);

    final Offset firstLocation = tester.getCenter(find.text('Source'));
    final TestGesture gesture = await tester.startGesture(firstLocation, pointer: 7);
    await tester.pump();

    expect(onAcceptCalled, isFalse);
    expect(onAcceptWithDetailsCalled, isFalse);
    expect(find.text('Source'), findsOneWidget);
    expect(find.text('Dragging'), findsOneWidget);
    expect(find.text('Target'), findsOneWidget);

    final Offset secondLocation = tester.getCenter(find.text('Target'));
    await gesture.moveTo(secondLocation);
    await tester.pump();

    expect(onAcceptCalled, isFalse);
    expect(onAcceptWithDetailsCalled, isFalse);
    expect(find.text('Source'), findsOneWidget);
    expect(find.text('Dragging'), findsOneWidget);
    expect(find.text('Target'), findsOneWidget);

    await gesture.up();
    await tester.pump();

    expect(onAcceptCalled, isFalse, reason: 'onAccept should not be called when data is null');
    expect(
      onAcceptWithDetailsCalled,
      isFalse,
      reason: 'onAcceptWithDetails should not be called when data is null',
    );
    expect(find.text('Source'), findsOneWidget);
    expect(find.text('Dragging'), findsNothing);
    expect(find.text('Target'), findsOneWidget);
  });

  testWidgets('Draggable disposes recognizer', (WidgetTester tester) async {
    late final OverlayEntry entry;
    addTearDown(
      () =>
          entry
            ..remove()
            ..dispose(),
    );

    bool didTap = false;
    await tester.pumpWidget(
      Directionality(
        textDirection: TextDirection.ltr,
        child: Overlay(
          initialEntries: <OverlayEntry>[
            entry = OverlayEntry(
              builder:
                  (BuildContext context) => GestureDetector(
                    onTap: () {
                      didTap = true;
                    },
                    child: Draggable<Object>(
                      feedback: Container(
                        width: 100.0,
                        height: 100.0,
                        color: const Color(0xFFFF0000),
                      ),
                      child: Container(color: const Color(0xFFFFFF00)),
                    ),
                  ),
            ),
          ],
        ),
      ),
    );

    final TestGesture gesture = await tester.startGesture(const Offset(10.0, 10.0));
    expect(didTap, isFalse);

    // This tears down the draggable without terminating the gesture sequence,
    // which used to trigger asserts in the multi-drag gesture recognizer.
    await tester.pumpWidget(Container(key: UniqueKey()));
    expect(didTap, isFalse);

    // Finish gesture to release resources.
    await gesture.up();
    await tester.pumpAndSettle();
  });

  // Regression test for https://github.com/flutter/flutter/issues/6128.
  testWidgets('Draggable plays nice with onTap', (WidgetTester tester) async {
    late final OverlayEntry entry;
    addTearDown(
      () =>
          entry
            ..remove()
            ..dispose(),
    );

    await tester.pumpWidget(
      Directionality(
        textDirection: TextDirection.ltr,
        child: Overlay(
          initialEntries: <OverlayEntry>[
            entry = OverlayEntry(
              builder:
                  (BuildContext context) => GestureDetector(
                    onTap: () {
                      /* registers a tap recognizer */
                    },
                    child: Draggable<Object>(
                      feedback: Container(
                        width: 100.0,
                        height: 100.0,
                        color: const Color(0xFFFF0000),
                      ),
                      child: Container(color: const Color(0xFFFFFF00)),
                    ),
                  ),
            ),
          ],
        ),
      ),
    );

    final TestGesture firstGesture = await tester.startGesture(
      const Offset(10.0, 10.0),
      pointer: 24,
    );
    final TestGesture secondGesture = await tester.startGesture(
      const Offset(10.0, 20.0),
      pointer: 25,
    );

    await firstGesture.moveBy(const Offset(100.0, 0.0));
    await secondGesture.up();
    await firstGesture.up();
  });

  testWidgets('DragTarget does not set state when remove from the tree', (
    WidgetTester tester,
  ) async {
    final List<String> events = <String>[];
    Offset firstLocation, secondLocation;

    await tester.pumpWidget(
      MaterialApp(
        home: Column(
          children: <Widget>[
            const Draggable<int>(data: 1, feedback: Text('Dragging'), child: Text('Source')),
            DragTarget<int>(
              builder: (BuildContext context, List<int?> data, List<dynamic> rejects) {
                return const Text('Target');
              },
              onAccept: (int? data) {
                events.add('drop');
              },
              onAcceptWithDetails: (DragTargetDetails<int> _) {
                events.add('details');
              },
            ),
          ],
        ),
      ),
    );

    expect(events, isEmpty);
    expect(find.text('Source'), findsOneWidget);
    expect(find.text('Target'), findsOneWidget);

    expect(events, isEmpty);
    await tester.tap(find.text('Source'));
    expect(events, isEmpty);

    firstLocation = tester.getCenter(find.text('Source'));
    final TestGesture gesture = await tester.startGesture(firstLocation, pointer: 7);
    await tester.pump();

    await tester.pump(const Duration(seconds: 20));

    secondLocation = tester.getCenter(find.text('Target'));
    await gesture.moveTo(secondLocation);
    await tester.pump();

    await tester.pumpWidget(
      const MaterialApp(
        home: Column(
          children: <Widget>[
            Draggable<int>(data: 1, feedback: Text('Dragging'), child: Text('Source')),
          ],
        ),
      ),
    );

    expect(events, isEmpty);
    await gesture.up();
    await tester.pump();
  });

  testWidgets('Drag and drop - remove draggable', (WidgetTester tester) async {
    final List<int> accepted = <int>[];
    final List<DragTargetDetails<int>> acceptedDetails = <DragTargetDetails<int>>[];

    await tester.pumpWidget(
      MaterialApp(
        home: Column(
          children: <Widget>[
            const Draggable<int>(data: 1, feedback: Text('Dragging'), child: Text('Source')),
            DragTarget<int>(
              builder: (BuildContext context, List<int?> data, List<dynamic> rejects) {
                return const SizedBox(height: 100.0, child: Text('Target'));
              },
              onAccept: accepted.add,
              onAcceptWithDetails: acceptedDetails.add,
            ),
          ],
        ),
      ),
    );

    expect(accepted, isEmpty);
    expect(acceptedDetails, isEmpty);
    expect(find.text('Source'), findsOneWidget);
    expect(find.text('Dragging'), findsNothing);
    expect(find.text('Target'), findsOneWidget);

    final Offset firstLocation = tester.getCenter(find.text('Source'));
    final TestGesture gesture = await tester.startGesture(firstLocation, pointer: 7);
    await tester.pump();

    expect(accepted, isEmpty);
    expect(acceptedDetails, isEmpty);
    expect(find.text('Source'), findsOneWidget);
    expect(find.text('Dragging'), findsOneWidget);
    expect(find.text('Target'), findsOneWidget);

    await tester.pumpWidget(
      MaterialApp(
        home: Column(
          children: <Widget>[
            DragTarget<int>(
              builder: (BuildContext context, List<int?> data, List<dynamic> rejects) {
                return const SizedBox(height: 100.0, child: Text('Target'));
              },
              onAccept: accepted.add,
              onAcceptWithDetails: acceptedDetails.add,
            ),
          ],
        ),
      ),
    );

    expect(accepted, isEmpty);
    expect(acceptedDetails, isEmpty);
    expect(find.text('Source'), findsNothing);
    expect(find.text('Dragging'), findsOneWidget);
    expect(find.text('Target'), findsOneWidget);

    final Offset secondLocation = tester.getCenter(find.text('Target'));
    await gesture.moveTo(secondLocation);
    await tester.pump();

    expect(accepted, isEmpty);
    expect(acceptedDetails, isEmpty);
    expect(find.text('Source'), findsNothing);
    expect(find.text('Dragging'), findsOneWidget);
    expect(find.text('Target'), findsOneWidget);

    await gesture.up();
    await tester.pump();

    expect(accepted, equals(<int>[1]));
    expect(acceptedDetails, hasLength(1));
    expect(acceptedDetails.first.offset, const Offset(256.0, 26.0));
    expect(find.text('Source'), findsNothing);
    expect(find.text('Dragging'), findsNothing);
    expect(find.text('Target'), findsOneWidget);
  });

  testWidgets('Tap above long-press draggable works', (WidgetTester tester) async {
    final List<String> events = <String>[];

    await tester.pumpWidget(
      MaterialApp(
        home: Material(
          child: Center(
            child: GestureDetector(
              onTap: () {
                events.add('tap');
              },
              child: const LongPressDraggable<int>(feedback: Text('Feedback'), child: Text('X')),
            ),
          ),
        ),
      ),
    );

    expect(events, isEmpty);
    await tester.tap(find.text('X'));
    expect(events, equals(<String>['tap']));
  });

  testWidgets('long-press draggable calls onDragEnd called if dropped on accepting target', (
    WidgetTester tester,
  ) async {
    final List<int> accepted = <int>[];
    final List<DragTargetDetails<int>> acceptedDetails = <DragTargetDetails<int>>[];
    bool onDragEndCalled = false;
    late DraggableDetails onDragEndDraggableDetails;

    await tester.pumpWidget(
      MaterialApp(
        home: Column(
          children: <Widget>[
            LongPressDraggable<int>(
              data: 1,
              feedback: const Text('Dragging'),
              onDragEnd: (DraggableDetails details) {
                onDragEndCalled = true;
                onDragEndDraggableDetails = details;
              },
              child: const Text('Source'),
            ),
            DragTarget<int>(
              builder: (BuildContext context, List<int?> data, List<dynamic> rejects) {
                return const SizedBox(height: 100.0, child: Text('Target'));
              },
              onAccept: accepted.add,
              onAcceptWithDetails: acceptedDetails.add,
            ),
          ],
        ),
      ),
    );

    expect(accepted, isEmpty);
    expect(acceptedDetails, isEmpty);
    expect(find.text('Source'), findsOneWidget);
    expect(find.text('Dragging'), findsNothing);
    expect(find.text('Target'), findsOneWidget);
    expect(onDragEndCalled, isFalse);

    final Offset firstLocation = tester.getCenter(find.text('Source'));
    final TestGesture gesture = await tester.startGesture(firstLocation, pointer: 7);
    await tester.pump();

    expect(accepted, isEmpty);
    expect(acceptedDetails, isEmpty);
    expect(find.text('Source'), findsOneWidget);
    expect(find.text('Dragging'), findsNothing);
    expect(find.text('Target'), findsOneWidget);
    expect(onDragEndCalled, isFalse);

    await tester.pump(kLongPressTimeout);

    expect(accepted, isEmpty);
    expect(acceptedDetails, isEmpty);
    expect(find.text('Source'), findsOneWidget);
    expect(find.text('Dragging'), findsOneWidget);
    expect(find.text('Target'), findsOneWidget);
    expect(onDragEndCalled, isFalse);

    final Offset secondLocation = tester.getCenter(find.text('Target'));
    await gesture.moveTo(secondLocation);
    await tester.pump();

    expect(accepted, isEmpty);
    expect(acceptedDetails, isEmpty);
    expect(find.text('Source'), findsOneWidget);
    expect(find.text('Dragging'), findsOneWidget);
    expect(find.text('Target'), findsOneWidget);
    expect(onDragEndCalled, isFalse);

    await gesture.up();
    await tester.pump();

    final Offset droppedLocation = tester.getTopLeft(find.text('Target'));
    final Offset expectedDropOffset = Offset(
      droppedLocation.dx,
      secondLocation.dy - firstLocation.dy,
    );

    expect(accepted, equals(<int>[1]));
    expect(acceptedDetails, hasLength(1));
    expect(acceptedDetails.first.offset, expectedDropOffset);
    expect(find.text('Source'), findsOneWidget);
    expect(find.text('Dragging'), findsNothing);
    expect(find.text('Target'), findsOneWidget);
    expect(onDragEndCalled, isTrue);
    expect(onDragEndDraggableDetails, isNotNull);
    expect(onDragEndDraggableDetails.wasAccepted, isTrue);
    expect(onDragEndDraggableDetails.velocity, equals(Velocity.zero));
    expect(onDragEndDraggableDetails.offset, equals(expectedDropOffset));
  });

  testWidgets('long-press draggable calls onDragCompleted called if dropped on accepting target', (
    WidgetTester tester,
  ) async {
    final List<int> accepted = <int>[];
    final List<DragTargetDetails<int>> acceptedDetails = <DragTargetDetails<int>>[];
    bool onDragCompletedCalled = false;

    await tester.pumpWidget(
      MaterialApp(
        home: Column(
          children: <Widget>[
            LongPressDraggable<int>(
              data: 1,
              feedback: const Text('Dragging'),
              onDragCompleted: () {
                onDragCompletedCalled = true;
              },
              child: const Text('Source'),
            ),
            DragTarget<int>(
              builder: (BuildContext context, List<int?> data, List<dynamic> rejects) {
                return const SizedBox(height: 100.0, child: Text('Target'));
              },
              onAccept: accepted.add,
              onAcceptWithDetails: acceptedDetails.add,
            ),
          ],
        ),
      ),
    );

    expect(accepted, isEmpty);
    expect(acceptedDetails, isEmpty);
    expect(find.text('Source'), findsOneWidget);
    expect(find.text('Dragging'), findsNothing);
    expect(find.text('Target'), findsOneWidget);
    expect(onDragCompletedCalled, isFalse);

    final Offset firstLocation = tester.getCenter(find.text('Source'));
    final TestGesture gesture = await tester.startGesture(firstLocation, pointer: 7);
    await tester.pump();

    expect(accepted, isEmpty);
    expect(acceptedDetails, isEmpty);
    expect(find.text('Source'), findsOneWidget);
    expect(find.text('Dragging'), findsNothing);
    expect(find.text('Target'), findsOneWidget);
    expect(onDragCompletedCalled, isFalse);

    await tester.pump(kLongPressTimeout);

    expect(accepted, isEmpty);
    expect(acceptedDetails, isEmpty);
    expect(find.text('Source'), findsOneWidget);
    expect(find.text('Dragging'), findsOneWidget);
    expect(find.text('Target'), findsOneWidget);
    expect(onDragCompletedCalled, isFalse);

    final Offset secondLocation = tester.getCenter(find.text('Target'));
    await gesture.moveTo(secondLocation);
    await tester.pump();

    expect(accepted, isEmpty);
    expect(acceptedDetails, isEmpty);
    expect(find.text('Source'), findsOneWidget);
    expect(find.text('Dragging'), findsOneWidget);
    expect(find.text('Target'), findsOneWidget);
    expect(onDragCompletedCalled, isFalse);

    await gesture.up();
    await tester.pump();

    expect(accepted, equals(<int>[1]));
    expect(acceptedDetails.first.offset, const Offset(256.0, 74.0));
    expect(find.text('Source'), findsOneWidget);
    expect(find.text('Dragging'), findsNothing);
    expect(find.text('Target'), findsOneWidget);
    expect(onDragCompletedCalled, isTrue);
  });

  testWidgets('long-press draggable calls onDragStartedCalled after long press', (
    WidgetTester tester,
  ) async {
    bool onDragStartedCalled = false;

    await tester.pumpWidget(
      MaterialApp(
        home: LongPressDraggable<int>(
          data: 1,
          feedback: const Text('Dragging'),
          onDragStarted: () {
            onDragStartedCalled = true;
          },
          child: const Text('Source'),
        ),
      ),
    );

    expect(find.text('Source'), findsOneWidget);
    expect(find.text('Dragging'), findsNothing);
    expect(onDragStartedCalled, isFalse);

    final Offset firstLocation = tester.getCenter(find.text('Source'));
    final TestGesture gesture = await tester.startGesture(firstLocation, pointer: 7);
    await tester.pump();

    expect(find.text('Source'), findsOneWidget);
    expect(find.text('Dragging'), findsNothing);
    expect(onDragStartedCalled, isFalse);

    await tester.pump(kLongPressTimeout);

    expect(find.text('Source'), findsOneWidget);
    expect(find.text('Dragging'), findsOneWidget);
    expect(onDragStartedCalled, isTrue);

    // Finish gesture to release resources.
    await gesture.up();
    await tester.pumpAndSettle();
  });

  testWidgets('Custom long press delay for LongPressDraggable', (WidgetTester tester) async {
    bool onDragStartedCalled = false;
    await tester.pumpWidget(
      MaterialApp(
        home: LongPressDraggable<int>(
          data: 1,
          delay: const Duration(seconds: 2),
          feedback: const Text('Dragging'),
          onDragStarted: () {
            onDragStartedCalled = true;
          },
          child: const Text('Source'),
        ),
      ),
    );
    expect(find.text('Source'), findsOneWidget);
    expect(find.text('Dragging'), findsNothing);
    expect(onDragStartedCalled, isFalse);
    final Offset firstLocation = tester.getCenter(find.text('Source'));
    final TestGesture gesture = await tester.startGesture(firstLocation, pointer: 7);
    await tester.pump();
    expect(find.text('Source'), findsOneWidget);
    expect(find.text('Dragging'), findsNothing);
    expect(onDragStartedCalled, isFalse);
    // Halfway into the long press duration.
    await tester.pump(const Duration(seconds: 1));
    expect(find.text('Source'), findsOneWidget);
    expect(find.text('Dragging'), findsNothing);
    expect(onDragStartedCalled, isFalse);
    // Long press draggable should be showing.
    await tester.pump(const Duration(seconds: 1));
    expect(find.text('Source'), findsOneWidget);
    expect(find.text('Dragging'), findsOneWidget);
    expect(onDragStartedCalled, isTrue);

    // Finish gesture to release resources.
    await gesture.up();
    await tester.pumpAndSettle();
  });

  testWidgets('Default long press delay for LongPressDraggable', (WidgetTester tester) async {
    bool onDragStartedCalled = false;
    await tester.pumpWidget(
      MaterialApp(
        home: LongPressDraggable<int>(
          data: 1,
          feedback: const Text('Dragging'),
          onDragStarted: () {
            onDragStartedCalled = true;
          },
          child: const Text('Source'),
        ),
      ),
    );
    expect(find.text('Source'), findsOneWidget);
    expect(find.text('Dragging'), findsNothing);
    expect(onDragStartedCalled, isFalse);
    final Offset firstLocation = tester.getCenter(find.text('Source'));
    final TestGesture gesture = await tester.startGesture(firstLocation, pointer: 7);
    await tester.pump();
    expect(find.text('Source'), findsOneWidget);
    expect(find.text('Dragging'), findsNothing);
    expect(onDragStartedCalled, isFalse);
    // Halfway into the long press duration.
    await tester.pump(const Duration(milliseconds: 250));
    expect(find.text('Source'), findsOneWidget);
    expect(find.text('Dragging'), findsNothing);
    expect(onDragStartedCalled, isFalse);
    // Long press draggable should be showing.
    await tester.pump(const Duration(milliseconds: 250));
    expect(find.text('Source'), findsOneWidget);
    expect(find.text('Dragging'), findsOneWidget);
    expect(onDragStartedCalled, isTrue);

    // Finish gesture to release resources.
    await gesture.up();
    await tester.pumpAndSettle();
  });

  testWidgets('long-press draggable calls Haptic Feedback onStart', (WidgetTester tester) async {
    await _testLongPressDraggableHapticFeedback(
      tester: tester,
      hapticFeedbackOnStart: true,
      expectedHapticFeedbackCount: 1,
    );
  });

  testWidgets('long-press draggable can disable Haptic Feedback', (WidgetTester tester) async {
    await _testLongPressDraggableHapticFeedback(
      tester: tester,
      hapticFeedbackOnStart: false,
      expectedHapticFeedbackCount: 0,
    );
  });

  testWidgets('Drag feedback with child anchor positions correctly', (WidgetTester tester) async {
    await _testChildAnchorFeedbackPosition(tester: tester);
  });

  testWidgets('Drag feedback with child anchor within a non-global Overlay positions correctly', (
    WidgetTester tester,
  ) async {
    await _testChildAnchorFeedbackPosition(tester: tester, left: 100.0, top: 100.0);
  });

  testWidgets('Drag feedback is put on root overlay with [rootOverlay] flag', (
    WidgetTester tester,
  ) async {
    final GlobalKey<NavigatorState> rootNavigatorKey = GlobalKey<NavigatorState>();
    final GlobalKey<NavigatorState> childNavigatorKey = GlobalKey<NavigatorState>();
    // Create a [MaterialApp], with a nested [Navigator], which has the
    // [Draggable].
    await tester.pumpWidget(
      MaterialApp(
        navigatorKey: rootNavigatorKey,
        home: Column(
          children: <Widget>[
            SizedBox(
              height: 200.0,
              child: Navigator(
                key: childNavigatorKey,
                onGenerateRoute: (RouteSettings settings) {
                  if (settings.name == '/') {
                    return MaterialPageRoute<void>(
                      settings: settings,
                      builder:
                          (BuildContext context) => const Draggable<int>(
                            data: 1,
                            feedback: Text('Dragging'),
                            rootOverlay: true,
                            child: Text('Source'),
                          ),
                    );
                  }
                  throw UnsupportedError('Unsupported route: $settings');
                },
              ),
            ),
            DragTarget<int>(
              builder: (BuildContext context, List<int?> data, List<dynamic> rejects) {
                return const SizedBox(height: 300.0, child: Center(child: Text('Target 1')));
              },
            ),
          ],
        ),
      ),
    );

    final Offset firstLocation = tester.getCenter(find.text('Source'));
    final TestGesture gesture = await tester.startGesture(firstLocation, pointer: 7);
    await tester.pump();

    final Offset secondLocation = tester.getCenter(find.text('Target 1'));
    await gesture.moveTo(secondLocation);
    await tester.pump();

    // Expect that the feedback widget is a descendant of the root overlay,
    // but not a descendant of the child overlay.
    expect(
      find.descendant(of: find.byType(Overlay).first, matching: find.text('Dragging')),
      findsOneWidget,
    );
    expect(
      find.descendant(of: find.byType(Overlay).last, matching: find.text('Dragging')),
      findsNothing,
    );
  });

  testWidgets('Drag feedback is put on root overlay with [rootOverlay] flag', (
    WidgetTester tester,
  ) async {
    final GlobalKey<NavigatorState> rootNavigatorKey = GlobalKey<NavigatorState>();
    final GlobalKey<NavigatorState> childNavigatorKey = GlobalKey<NavigatorState>();
    // Create a [MaterialApp], with a nested [Navigator], which has the
    // [Draggable].
    await tester.pumpWidget(
      MaterialApp(
        navigatorKey: rootNavigatorKey,
        home: Column(
          children: <Widget>[
            SizedBox(
              height: 200.0,
              child: Navigator(
                key: childNavigatorKey,
                onGenerateRoute: (RouteSettings settings) {
                  if (settings.name == '/') {
                    return MaterialPageRoute<void>(
                      settings: settings,
                      builder:
                          (BuildContext context) => const LongPressDraggable<int>(
                            data: 1,
                            feedback: Text('Dragging'),
                            rootOverlay: true,
                            child: Text('Source'),
                          ),
                    );
                  }
                  throw UnsupportedError('Unsupported route: $settings');
                },
              ),
            ),
            DragTarget<int>(
              builder: (BuildContext context, List<int?> data, List<dynamic> rejects) {
                return const SizedBox(height: 300.0, child: Center(child: Text('Target 1')));
              },
            ),
          ],
        ),
      ),
    );

    final Offset firstLocation = tester.getCenter(find.text('Source'));
    final TestGesture gesture = await tester.startGesture(firstLocation, pointer: 7);
    await tester.pump(kLongPressTimeout);

    final Offset secondLocation = tester.getCenter(find.text('Target 1'));
    await gesture.moveTo(secondLocation);
    await tester.pump();

    // Expect that the feedback widget is a descendant of the root overlay,
    // but not a descendant of the child overlay.
    expect(
      find.descendant(of: find.byType(Overlay).first, matching: find.text('Dragging')),
      findsOneWidget,
    );
    expect(
      find.descendant(of: find.byType(Overlay).last, matching: find.text('Dragging')),
      findsNothing,
    );
  });

  testWidgets('configurable DragTarget hit test behavior', (WidgetTester tester) async {
    const HitTestBehavior hitTestBehavior = HitTestBehavior.opaque;

    await tester.pumpWidget(
      const MaterialApp(
        home: Column(
          children: <Widget>[
            LongPressDraggable<int>(
              hitTestBehavior: hitTestBehavior,
              feedback: SizedBox(),
              child: SizedBox(),
            ),
          ],
        ),
      ),
    );
    expect(
      tester
          .widget<Listener>(
            find.descendant(of: find.byType(Column), matching: find.byType(Listener)),
          )
          .behavior,
      hitTestBehavior,
    );
  });

  // Regression test for https://github.com/flutter/flutter/issues/72483
  testWidgets('Drag and drop - DragTarget<Object> can accept Draggable<int> data', (
    WidgetTester tester,
  ) async {
    final List<Object> accepted = <Object>[];
    await tester.pumpWidget(
      MaterialApp(
        home: Column(
          children: <Widget>[
            const Draggable<int>(data: 1, feedback: Text('Dragging'), child: Text('Source')),
            DragTarget<Object>(
              builder: (BuildContext context, List<Object?> data, List<dynamic> rejects) {
                return const SizedBox(height: 100.0, child: Text('Target'));
              },
              onAccept: accepted.add,
            ),
          ],
        ),
      ),
    );

    expect(accepted, isEmpty);

    final Offset firstLocation = tester.getCenter(find.text('Source'));
    final TestGesture gesture = await tester.startGesture(firstLocation, pointer: 7);
    await tester.pump();

    final Offset secondLocation = tester.getCenter(find.text('Target'));
    await gesture.moveTo(secondLocation);
    await tester.pump();

    await gesture.up();
    await tester.pump();

    expect(accepted, equals(<int>[1]));
  });

  testWidgets(
    'Drag and drop - DragTarget<int> can accept Draggable<Object> data when runtime type is int',
    (WidgetTester tester) async {
      final List<int> accepted = <int>[];
      await tester.pumpWidget(
        MaterialApp(
          home: Column(
            children: <Widget>[
              const Draggable<Object>(data: 1, feedback: Text('Dragging'), child: Text('Source')),
              DragTarget<int>(
                builder: (BuildContext context, List<int?> data, List<dynamic> rejects) {
                  return const SizedBox(height: 100.0, child: Text('Target'));
                },
                onAccept: accepted.add,
              ),
            ],
          ),
        ),
      );

      expect(accepted, isEmpty);

      final Offset firstLocation = tester.getCenter(find.text('Source'));
      final TestGesture gesture = await tester.startGesture(firstLocation, pointer: 7);
      await tester.pump();

      final Offset secondLocation = tester.getCenter(find.text('Target'));
      await gesture.moveTo(secondLocation);
      await tester.pump();

      await gesture.up();
      await tester.pump();

      expect(accepted, equals(<int>[1]));
    },
  );

  testWidgets(
    'Drag and drop - DragTarget<int> should not accept Draggable<Object> data when runtime type null',
    (WidgetTester tester) async {
      final List<int> accepted = <int>[];
      bool isReceiveNullDataForCheck = false;
      await tester.pumpWidget(
        MaterialApp(
          home: Column(
            children: <Widget>[
              const Draggable<Object>(feedback: Text('Dragging'), child: Text('Source')),
              DragTarget<int>(
                builder: (BuildContext context, List<int?> data, List<dynamic> rejects) {
                  return const SizedBox(height: 100.0, child: Text('Target'));
                },
                onAccept: accepted.add,
                onWillAccept: (int? data) {
                  if (data == null) {
                    isReceiveNullDataForCheck = true;
                  }
                  return data != null;
                },
              ),
            ],
          ),
        ),
      );

      expect(accepted, isEmpty);

      final Offset firstLocation = tester.getCenter(find.text('Source'));
      final TestGesture gesture = await tester.startGesture(firstLocation, pointer: 7);
      await tester.pump();

      final Offset secondLocation = tester.getCenter(find.text('Target'));
      await gesture.moveTo(secondLocation);
      await tester.pump();

      await gesture.up();
      await tester.pump();

      expect(accepted, isEmpty);
      expect(isReceiveNullDataForCheck, true);
    },
  );

  testWidgets('Drag and drop can contribute semantics', (WidgetTester tester) async {
    final SemanticsTester semantics = SemanticsTester(tester);
    await tester.pumpWidget(
      MaterialApp(
        home: ListView(
          scrollDirection: Axis.horizontal,
          addSemanticIndexes: false,
          children: <Widget>[
            DragTarget<int>(
              builder: (BuildContext context, List<int?> data, List<dynamic> rejects) {
                return const Text('Target');
              },
            ),
            Container(width: 400.0),
            const Draggable<int>(
              data: 1,
              feedback: Text('H'),
              childWhenDragging: SizedBox(),
              axis: Axis.horizontal,
              ignoringFeedbackSemantics: false,
              child: Text('H'),
            ),
            const Draggable<int>(
              data: 2,
              feedback: Text('V'),
              childWhenDragging: SizedBox(),
              axis: Axis.vertical,
              ignoringFeedbackSemantics: false,
              child: Text('V'),
            ),
            const Draggable<int>(
              data: 3,
              feedback: Text('N'),
              childWhenDragging: SizedBox(),
              child: Text('N'),
            ),
          ],
        ),
      ),
    );

    expect(
      semantics,
      hasSemantics(
        TestSemantics.root(
          children: <TestSemantics>[
            TestSemantics(
              id: 1,
              textDirection: TextDirection.ltr,
              children: <TestSemantics>[
                TestSemantics(
                  id: 2,
                  children: <TestSemantics>[
                    TestSemantics(
                      id: 3,
                      flags: <SemanticsFlag>[SemanticsFlag.scopesRoute],
                      children: <TestSemantics>[
                        TestSemantics(
                          id: 4,
                          children: <TestSemantics>[
                            TestSemantics(
                              id: 9,
                              flags: <SemanticsFlag>[SemanticsFlag.hasImplicitScrolling],
                              actions: <SemanticsAction>[
                                SemanticsAction.scrollLeft,
                                SemanticsAction.scrollToOffset,
                              ],
                              children: <TestSemantics>[
                                TestSemantics(
                                  id: 5,
                                  tags: <SemanticsTag>[
                                    const SemanticsTag('RenderViewport.twoPane'),
                                  ],
                                  label: 'Target',
                                  textDirection: TextDirection.ltr,
                                ),
                                TestSemantics(
                                  id: 6,
                                  tags: <SemanticsTag>[
                                    const SemanticsTag('RenderViewport.twoPane'),
                                  ],
                                  label: 'H',
                                  textDirection: TextDirection.ltr,
                                ),
                                TestSemantics(
                                  id: 7,
                                  tags: <SemanticsTag>[
                                    const SemanticsTag('RenderViewport.twoPane'),
                                  ],
                                  label: 'V',
                                  textDirection: TextDirection.ltr,
                                ),
                                TestSemantics(
                                  id: 8,
                                  tags: <SemanticsTag>[
                                    const SemanticsTag('RenderViewport.twoPane'),
                                  ],
                                  label: 'N',
                                  textDirection: TextDirection.ltr,
                                ),
                              ],
                            ),
                          ],
                        ),
                      ],
                    ),
                  ],
                ),
              ],
            ),
          ],
        ),
        ignoreTransform: true,
        ignoreRect: true,
      ),
    );

    final Offset firstLocation = tester.getTopLeft(find.text('N'));
    final Offset secondLocation = firstLocation + const Offset(300.0, 300.0);
    final TestGesture gesture = await tester.startGesture(firstLocation, pointer: 7);
    await tester.pump();
    await gesture.moveTo(secondLocation);
    await tester.pump();

    expect(
      semantics,
      hasSemantics(
        TestSemantics.root(
          children: <TestSemantics>[
            TestSemantics(
              id: 1,
              textDirection: TextDirection.ltr,
              children: <TestSemantics>[
                TestSemantics(
                  id: 2,
                  children: <TestSemantics>[
                    TestSemantics(
                      id: 3,
                      flags: <SemanticsFlag>[SemanticsFlag.scopesRoute],
                      children: <TestSemantics>[
                        TestSemantics(
                          id: 4,
                          children: <TestSemantics>[
                            TestSemantics(
                              id: 9,
                              actions: <SemanticsAction>[SemanticsAction.scrollToOffset],
                              flags: <SemanticsFlag>[SemanticsFlag.hasImplicitScrolling],
                              children: <TestSemantics>[
                                TestSemantics(
                                  id: 5,
                                  tags: <SemanticsTag>[
                                    const SemanticsTag('RenderViewport.twoPane'),
                                  ],
                                  label: 'Target',
                                  textDirection: TextDirection.ltr,
                                ),
                                TestSemantics(
                                  id: 6,
                                  tags: <SemanticsTag>[
                                    const SemanticsTag('RenderViewport.twoPane'),
                                  ],
                                  label: 'H',
                                  textDirection: TextDirection.ltr,
                                ),
                                TestSemantics(
                                  id: 7,
                                  tags: <SemanticsTag>[
                                    const SemanticsTag('RenderViewport.twoPane'),
                                  ],
                                  label: 'V',
                                  textDirection: TextDirection.ltr,
                                ),

                                /// N is moved offscreen.
                              ],
                            ),
                          ],
                        ),
                      ],
                    ),
                  ],
                ),
              ],
            ),
          ],
        ),
        ignoreTransform: true,
        ignoreRect: true,
      ),
    );
    semantics.dispose();
  });

  testWidgets('Drag and drop - when a dragAnchorStrategy is provided it gets called', (
    WidgetTester tester,
  ) async {
    bool dragAnchorStrategyCalled = false;

    await tester.pumpWidget(
      MaterialApp(
        home: Column(
          children: <Widget>[
            Draggable<int>(
              feedback: const Text('Feedback'),
              dragAnchorStrategy: (
                Draggable<Object> widget,
                BuildContext context,
                Offset position,
              ) {
                dragAnchorStrategyCalled = true;
                return Offset.zero;
              },
              child: const Text('Source'),
            ),
          ],
        ),
      ),
    );

    final Offset location = tester.getCenter(find.text('Source'));
    final TestGesture gesture = await tester.startGesture(location, pointer: 7);

    expect(dragAnchorStrategyCalled, true);

    // Finish gesture to release resources.
    await gesture.up();
    await tester.pumpAndSettle();
  });

  testWidgets('Drag and drop - feedback matches pointer in scaled MaterialApp', (
    WidgetTester tester,
  ) async {
    await tester.pumpWidget(
      Transform.scale(
        scale: 0.5,
        child: const MaterialApp(
          home: Scaffold(
            body: Draggable<int>(data: 42, feedback: Text('Feedback'), child: Text('Source')),
          ),
        ),
      ),
    );

    final Offset location = tester.getTopLeft(find.text('Source'));
    final TestGesture gesture = await tester.startGesture(location);
    final Offset secondLocation = location + const Offset(100, 100);
    await gesture.moveTo(secondLocation);
    await tester.pump();
    final Offset appTopLeft = tester.getTopLeft(find.byType(MaterialApp));
    expect(tester.getTopLeft(find.text('Source')), appTopLeft);
    expect(tester.getTopLeft(find.text('Feedback')), secondLocation);

    // Finish gesture to release resources.
    await gesture.up();
    await tester.pump();
  });

  testWidgets('Drag and drop - childDragAnchorStrategy works in scaled MaterialApp', (
    WidgetTester tester,
  ) async {
    final Key sourceKey = UniqueKey();
    final Key feedbackKey = UniqueKey();
    await tester.pumpWidget(
      Transform.scale(
        scale: 0.5,
        child: MaterialApp(
          home: Scaffold(
            body: Draggable<int>(
              data: 42,
              feedback: Text('Text', key: feedbackKey),
              child: Text('Text', key: sourceKey),
            ),
          ),
        ),
      ),
    );
    final Finder source = find.byKey(sourceKey);
    final Finder feedback = find.byKey(feedbackKey);

    final TestGesture gesture = await tester.startGesture(tester.getCenter(source));
    await tester.pump();
    expect(tester.getTopLeft(source), tester.getTopLeft(feedback));

    // Finish gesture to release resources.
    await gesture.up();
    await tester.pump();
  });

  testWidgets('Drag and drop - feedback matches pointer in rotated MaterialApp', (
    WidgetTester tester,
  ) async {
    await tester.pumpWidget(
      Transform.rotate(
        angle: 1, // ~57 degrees
        child: const MaterialApp(
          home: Scaffold(
            body: Draggable<int>(data: 42, feedback: Text('Feedback'), child: Text('Source')),
          ),
        ),
      ),
    );

    final Offset location = tester.getTopLeft(find.text('Source'));
    final TestGesture gesture = await tester.startGesture(location);
    final Offset secondLocation = location + const Offset(100, 100);
    await gesture.moveTo(secondLocation);
    await tester.pump();
    final Offset appTopLeft = tester.getTopLeft(find.byType(MaterialApp));
    expect(tester.getTopLeft(find.text('Source')), appTopLeft);
    final Offset feedbackTopLeft = tester.getTopLeft(find.text('Feedback'));

    // Different rotations can incur rounding errors, this makes it more robust
    expect(feedbackTopLeft.dx, moreOrLessEquals(secondLocation.dx));
    expect(feedbackTopLeft.dy, moreOrLessEquals(secondLocation.dy));

    // Finish gesture to release resources.
    await gesture.up();
    await tester.pump();
  });

  testWidgets('Drag and drop - unmounting overlay ends drag gracefully', (
    WidgetTester tester,
  ) async {
    final ValueNotifier<bool> mountedNotifier = ValueNotifier<bool>(true);
    addTearDown(mountedNotifier.dispose);

    await tester.pumpWidget(
      ValueListenableBuilder<bool>(
        valueListenable: mountedNotifier,
        builder:
            (_, bool value, __) =>
                value
                    ? const MaterialApp(
                      home: Scaffold(
                        body: Draggable<int>(
                          data: 42,
                          feedback: Text('Feedback'),
                          child: Text('Source'),
                        ),
                      ),
                    )
                    : Container(),
      ),
    );

    final Offset location = tester.getCenter(find.text('Source'));
    final TestGesture gesture = await tester.startGesture(location);
    final Offset secondLocation = location + const Offset(100, 100);
    await gesture.moveTo(secondLocation);
    await tester.pump();
    expect(find.text('Feedback'), findsOneWidget);

    // Unmount overlay
    mountedNotifier.value = false;
    await tester.pump();

    // This should not throw
    await gesture.moveTo(location);

    expect(find.byType(Container), findsOneWidget);
    expect(find.text('Feedback'), findsNothing);

    // Finish gesture to release resources.
    await gesture.up();
    await tester.pump();
  });

  testWidgets('configurable Draggable hit test behavior', (WidgetTester tester) async {
    const HitTestBehavior hitTestBehavior = HitTestBehavior.deferToChild;

    await tester.pumpWidget(
      const MaterialApp(
        home: Column(
          children: <Widget>[
            Draggable<int>(
              feedback: SizedBox(height: 50.0, child: Text('Draggable')),
              child: SizedBox(height: 50.0, child: Text('Target')),
            ),
          ],
        ),
      ),
    );

    expect(tester.widget<Listener>(find.byType(Listener).first).behavior, hitTestBehavior);
  });

  // Regression test for https://github.com/flutter/flutter/issues/92083
  testWidgets('feedback respect the MouseRegion cursor configure', (WidgetTester tester) async {
    await tester.pumpWidget(
      const MaterialApp(
        home: Column(
          children: <Widget>[
            Draggable<int>(
              ignoringFeedbackPointer: false,
              feedback: MouseRegion(
                cursor: SystemMouseCursors.grabbing,
                child: SizedBox(height: 50.0, child: Text('Draggable')),
              ),
              child: SizedBox(height: 50.0, child: Text('Target')),
            ),
          ],
        ),
      ),
    );

    final Offset location = tester.getCenter(find.text('Target'));
    final TestGesture gesture = await tester.createGesture(kind: PointerDeviceKind.mouse);
    await gesture.addPointer(location: location);

    await gesture.down(location);
    await tester.pump();

    expect(
      RendererBinding.instance.mouseTracker.debugDeviceActiveCursor(1),
      SystemMouseCursors.grabbing,
    );
    gesture.up();
  });

  testWidgets('configurable feedback ignore pointer behavior', (WidgetTester tester) async {
    bool onTap = false;
    await tester.pumpWidget(
      MaterialApp(
        home: Column(
          children: <Widget>[
            Draggable<int>(
              ignoringFeedbackPointer: false,
              feedback: GestureDetector(
                onTap: () => onTap = true,
                child: const SizedBox(height: 50.0, child: Text('Draggable')),
              ),
              child: const SizedBox(height: 50.0, child: Text('Target')),
            ),
          ],
        ),
      ),
    );

    final Offset location = tester.getCenter(find.text('Target'));
    final TestGesture gesture = await tester.startGesture(location, pointer: 7);
    final Offset secondLocation = location + const Offset(7.0, 7.0);
    await gesture.moveTo(secondLocation);
    await tester.pump();

    await tester.tap(find.text('Draggable'));
    expect(onTap, true);
  });

  testWidgets('configurable feedback ignore pointer behavior - LongPressDraggable', (
    WidgetTester tester,
  ) async {
    bool onTap = false;
    await tester.pumpWidget(
      MaterialApp(
        home: Column(
          children: <Widget>[
            LongPressDraggable<int>(
              ignoringFeedbackPointer: false,
              feedback: GestureDetector(
                onTap: () => onTap = true,
                child: const SizedBox(height: 50.0, child: Text('Draggable')),
              ),
              child: const SizedBox(height: 50.0, child: Text('Target')),
            ),
          ],
        ),
      ),
    );

    final Offset location = tester.getCenter(find.text('Target'));
    final TestGesture gesture = await tester.startGesture(location, pointer: 7);
    await tester.pump(kLongPressTimeout);

    final Offset secondLocation = location + const Offset(7.0, 7.0);
    await gesture.moveTo(secondLocation);
    await tester.pump();

    await tester.tap(find.text('Draggable'));
    expect(onTap, true);
  });

  testWidgets('configurable DragTarget hit test behavior', (WidgetTester tester) async {
    const HitTestBehavior hitTestBehavior = HitTestBehavior.deferToChild;

    await tester.pumpWidget(
      MaterialApp(
        home: Column(
          children: <Widget>[
            DragTarget<int>(
              hitTestBehavior: hitTestBehavior,
              builder: (BuildContext context, List<int?> data, List<dynamic> rejects) {
                return const SizedBox(height: 100.0, child: Text('Target'));
              },
            ),
          ],
        ),
      ),
    );

    expect(tester.widget<MetaData>(find.byType(MetaData)).behavior, hitTestBehavior);
  });

  testWidgets('LongPressDraggable.dragAnchorStrategy', (WidgetTester tester) async {
    const Widget widget1 = Placeholder(key: ValueKey<int>(1));
    const Widget widget2 = Placeholder(key: ValueKey<int>(2));
    Offset dummyStrategy(Draggable<Object> draggable, BuildContext context, Offset position) =>
        Offset.zero;
    expect(const LongPressDraggable<int>(feedback: widget2, child: widget1), isA<Draggable<int>>());
    expect(const LongPressDraggable<int>(feedback: widget2, child: widget1).child, widget1);
    expect(const LongPressDraggable<int>(feedback: widget2, child: widget1).feedback, widget2);
    expect(
      LongPressDraggable<int>(
        feedback: widget2,
        dragAnchorStrategy: dummyStrategy,
        child: widget1,
      ).dragAnchorStrategy,
      dummyStrategy,
    );
  });

  testWidgets('Test allowedButtonsFilter', (WidgetTester tester) async {
    Widget build(bool Function(int buttons)? allowedButtonsFilter) {
      return MaterialApp(
        home: Draggable<int>(
          key: UniqueKey(),
          allowedButtonsFilter: allowedButtonsFilter,
          feedback: const Text('Dragging'),
          child: const Text('Source'),
        ),
      );
    }

    await tester.pumpWidget(build(null));
    final Offset firstLocation = tester.getCenter(find.text('Source'));
    expect(find.text('Dragging'), findsNothing);
    final TestGesture gesture = await tester.startGesture(firstLocation, pointer: 7);
    await tester.pump();
    expect(find.text('Dragging'), findsOneWidget);
    await gesture.up();

    await tester.pumpWidget(build((int buttons) => buttons == kSecondaryButton));
    expect(find.text('Dragging'), findsNothing);
    final TestGesture gesture1 = await tester.startGesture(firstLocation, pointer: 8);
    await tester.pump();
    expect(find.text('Dragging'), findsNothing);
    await gesture1.up();

    await tester.pumpWidget(
      build((int buttons) => buttons & kTertiaryButton != 0 || buttons & kPrimaryButton != 0),
    );
    expect(find.text('Dragging'), findsNothing);
    final TestGesture gesture2 = await tester.startGesture(firstLocation, pointer: 8);
    await tester.pump();
    expect(find.text('Dragging'), findsOneWidget);
    await gesture2.up();

    await tester.pumpWidget(build((int buttons) => false));
    expect(find.text('Dragging'), findsNothing);
    final TestGesture gesture3 = await tester.startGesture(firstLocation, pointer: 8);
    await tester.pump();
    expect(find.text('Dragging'), findsNothing);
    await gesture3.up();
  });

  testWidgets('throws error when both onWillAccept and onWillAcceptWithDetails are provided', (
    WidgetTester tester,
  ) async {
    expect(
      () => DragTarget<int>(
        builder: (BuildContext context, List<int?> data, List<dynamic> rejects) {
          return const SizedBox(height: 100.0, child: Text('Target'));
        },
        onWillAccept: (int? data) => true,
        onWillAcceptWithDetails: (DragTargetDetails<int> details) => false,
      ),
      throwsAssertionError,
    );
  });
}

Future<void> _testLongPressDraggableHapticFeedback({
  required WidgetTester tester,
  required bool hapticFeedbackOnStart,
  required int expectedHapticFeedbackCount,
}) async {
  bool onDragStartedCalled = false;

  int hapticFeedbackCalls = 0;
  tester.binding.defaultBinaryMessenger.setMockMethodCallHandler(SystemChannels.platform, (
    MethodCall methodCall,
  ) async {
    if (methodCall.method == 'HapticFeedback.vibrate') {
      hapticFeedbackCalls++;
    }
    return null;
  });

  await tester.pumpWidget(
    MaterialApp(
      home: LongPressDraggable<int>(
        data: 1,
        feedback: const Text('Dragging'),
        hapticFeedbackOnStart: hapticFeedbackOnStart,
        onDragStarted: () {
          onDragStartedCalled = true;
        },
        child: const Text('Source'),
      ),
    ),
  );

  expect(find.text('Source'), findsOneWidget);
  expect(find.text('Dragging'), findsNothing);
  expect(onDragStartedCalled, isFalse);

  final Offset firstLocation = tester.getCenter(find.text('Source'));
  final TestGesture gesture = await tester.startGesture(firstLocation, pointer: 7);
  await tester.pump();

  expect(find.text('Source'), findsOneWidget);
  expect(find.text('Dragging'), findsNothing);
  expect(onDragStartedCalled, isFalse);

  await tester.pump(kLongPressTimeout);

  expect(find.text('Source'), findsOneWidget);
  expect(find.text('Dragging'), findsOneWidget);
  expect(onDragStartedCalled, isTrue);
  expect(hapticFeedbackCalls, expectedHapticFeedbackCount);

  // Finish gesture to release resources.
  await gesture.up();
  await tester.pumpAndSettle();
}

Future<void> _testChildAnchorFeedbackPosition({
  required WidgetTester tester,
  double top = 0.0,
  double left = 0.0,
}) async {
  final List<int> accepted = <int>[];
  final List<DragTargetDetails<int>> acceptedDetails = <DragTargetDetails<int>>[];
  int dragStartedCount = 0;

  await tester.pumpWidget(
    Stack(
      textDirection: TextDirection.ltr,
      children: <Widget>[
        Positioned(
          left: left,
          top: top,
          right: 0.0,
          bottom: 0.0,
          child: MaterialApp(
            home: Column(
              children: <Widget>[
                Draggable<int>(
                  data: 1,
                  feedback: const Text('Dragging'),
                  onDragStarted: () {
                    ++dragStartedCount;
                  },
                  child: const Text('Source'),
                ),
                DragTarget<int>(
                  builder: (BuildContext context, List<int?> data, List<dynamic> rejects) {
                    return const SizedBox(height: 100.0, child: Text('Target'));
                  },
                  onAccept: accepted.add,
                  onAcceptWithDetails: acceptedDetails.add,
                ),
              ],
            ),
          ),
        ),
      ],
    ),
  );

  expect(accepted, isEmpty);
  expect(acceptedDetails, isEmpty);
  expect(find.text('Source'), findsOneWidget);
  expect(find.text('Dragging'), findsNothing);
  expect(find.text('Target'), findsOneWidget);
  expect(dragStartedCount, 0);

  final Offset firstLocation = tester.getCenter(find.text('Source'));
  final TestGesture gesture = await tester.startGesture(firstLocation, pointer: 7);
  await tester.pump();

  expect(accepted, isEmpty);
  expect(acceptedDetails, isEmpty);
  expect(find.text('Source'), findsOneWidget);
  expect(find.text('Dragging'), findsOneWidget);
  expect(find.text('Target'), findsOneWidget);
  expect(dragStartedCount, 1);

  final Offset secondLocation = tester.getBottomRight(find.text('Target'));
  await gesture.moveTo(secondLocation);
  await tester.pump();

  expect(accepted, isEmpty);
  expect(acceptedDetails, isEmpty);
  expect(find.text('Source'), findsOneWidget);
  expect(find.text('Dragging'), findsOneWidget);
  expect(find.text('Target'), findsOneWidget);
  expect(dragStartedCount, 1);

  final Offset feedbackTopLeft = tester.getTopLeft(find.text('Dragging'));
  final Offset sourceTopLeft = tester.getTopLeft(find.text('Source'));
  final Offset dragOffset = secondLocation - firstLocation;
  expect(feedbackTopLeft, equals(sourceTopLeft + dragOffset));
}

class DragTargetData {}

class ExtendedDragTargetData extends DragTargetData {}
