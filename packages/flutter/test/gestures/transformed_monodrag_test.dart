// Copyright 2014 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

import 'dart:math' as math;

import 'package:flutter/gestures.dart';
import 'package:flutter/material.dart';
import 'package:flutter_test/flutter_test.dart';

void main() {
  group('Horizontal', () {
    testWidgets('gets local coordinates', (WidgetTester tester) async {
      int dragCancelCount = 0;
      final List<DragDownDetails> downDetails = <DragDownDetails>[];
      final List<DragEndDetails> endDetails = <DragEndDetails>[];
      final List<DragStartDetails> startDetails = <DragStartDetails>[];
      final List<DragUpdateDetails> updateDetails = <DragUpdateDetails>[];

      final Key redContainer = UniqueKey();
      await tester.pumpWidget(
        Center(
          child: GestureDetector(
            onHorizontalDragCancel: () {
              dragCancelCount++;
            },
            onHorizontalDragDown: (DragDownDetails details) {
              downDetails.add(details);
            },
            onHorizontalDragEnd: (DragEndDetails details) {
              endDetails.add(details);
            },
            onHorizontalDragStart: (DragStartDetails details) {
              startDetails.add(details);
            },
            onHorizontalDragUpdate: (DragUpdateDetails details) {
              updateDetails.add(details);
            },
            child: Container(key: redContainer, width: 100, height: 150, color: Colors.red),
          ),
        ),
      );

      await tester.drag(find.byKey(redContainer), const Offset(100, 0));
      expect(dragCancelCount, 0);
      expect(downDetails.single.localPosition, const Offset(50, 75));
      expect(downDetails.single.globalPosition, const Offset(400, 300));
      expect(endDetails, hasLength(1));
      expect(startDetails.single.localPosition, const Offset(50, 75));
      expect(startDetails.single.globalPosition, const Offset(400, 300));
      expect(updateDetails.last.localPosition, const Offset(50 + 100.0, 75));
      expect(updateDetails.last.globalPosition, const Offset(400 + 100.0, 300));
      expect(
        updateDetails.fold(
          Offset.zero,
          (Offset offset, DragUpdateDetails details) => offset + details.delta,
        ),
        const Offset(100, 0),
      );
      expect(
        updateDetails.fold(
          0.0,
          (double offset, DragUpdateDetails details) => offset + (details.primaryDelta ?? 0),
        ),
        100.0,
      );
    });

    testWidgets('kTouchSlop is evaluated in the global coordinate space when scaled up', (
      WidgetTester tester,
    ) async {
      int dragCancelCount = 0;
      final List<DragDownDetails> downDetails = <DragDownDetails>[];
      final List<DragEndDetails> endDetails = <DragEndDetails>[];
      final List<DragStartDetails> startDetails = <DragStartDetails>[];
      final List<DragUpdateDetails> updateDetails = <DragUpdateDetails>[];

      final Key redContainer = UniqueKey();
      await tester.pumpWidget(
        Center(
          child: Transform.scale(
            scale: 2.0,
            child: GestureDetector(
              onHorizontalDragCancel: () {
                dragCancelCount++;
              },
              onHorizontalDragDown: (DragDownDetails details) {
                downDetails.add(details);
              },
              onHorizontalDragEnd: (DragEndDetails details) {
                endDetails.add(details);
              },
              onHorizontalDragStart: (DragStartDetails details) {
                startDetails.add(details);
              },
              onHorizontalDragUpdate: (DragUpdateDetails details) {
                updateDetails.add(details);
              },
              onTap: () {
                // Competing gesture detector.
              },
              child: Container(key: redContainer, width: 100, height: 150, color: Colors.red),
            ),
          ),
        ),
      );

      // Move just above kTouchSlop should recognize drag.
      await tester.drag(find.byKey(redContainer), const Offset(kTouchSlop + 1, 0));

      expect(dragCancelCount, 0);
      expect(downDetails.single.localPosition, const Offset(50, 75));
      expect(downDetails.single.globalPosition, const Offset(400, 300));
      expect(endDetails, hasLength(1));
      expect(startDetails.single.localPosition, const Offset(50 + (kTouchSlop + 1) / 2, 75));
      expect(startDetails.single.globalPosition, const Offset(400 + (kTouchSlop + 1), 300));
      expect(updateDetails, isEmpty);

      dragCancelCount = 0;
      downDetails.clear();
      endDetails.clear();
      startDetails.clear();
      updateDetails.clear();

      // Move just below kTouchSlop does not recognize drag.
      await tester.drag(find.byKey(redContainer), const Offset(kTouchSlop - 1, 0));
      expect(dragCancelCount, 1);
      expect(downDetails.single.localPosition, const Offset(50, 75));
      expect(downDetails.single.globalPosition, const Offset(400, 300));
      expect(endDetails, isEmpty);
      expect(startDetails, isEmpty);
      expect(updateDetails, isEmpty);

      dragCancelCount = 0;
      downDetails.clear();
      endDetails.clear();
      startDetails.clear();
      updateDetails.clear();

      // Move in two separate movements
      final TestGesture gesture = await tester.startGesture(
        tester.getCenter(find.byKey(redContainer)),
      );
      await gesture.moveBy(const Offset(kTouchSlop + 1, 30));
      await gesture.moveBy(const Offset(100, 10));
      await gesture.up();

      expect(dragCancelCount, 0);
      expect(downDetails.single.localPosition, const Offset(50, 75));
      expect(downDetails.single.globalPosition, const Offset(400, 300));
      expect(endDetails, hasLength(1));
      expect(
        startDetails.single.localPosition,
        const Offset(50 + (kTouchSlop + 1) / 2, 75.0 + 30.0 / 2),
      );
      expect(startDetails.single.globalPosition, const Offset(400 + (kTouchSlop + 1), 300 + 30.0));
      expect(
        updateDetails.single.localPosition,
        startDetails.single.localPosition + const Offset(100.0 / 2, 10 / 2),
      );
      expect(
        updateDetails.single.globalPosition,
        startDetails.single.globalPosition + const Offset(100.0, 10.0),
      );
      expect(updateDetails.single.delta, const Offset(100.0 / 2, 0.0));
      expect(updateDetails.single.primaryDelta, 100.0 / 2);

      dragCancelCount = 0;
      downDetails.clear();
      endDetails.clear();
      startDetails.clear();
      updateDetails.clear();
    });

    testWidgets('kTouchSlop is evaluated in the global coordinate space when scaled down', (
      WidgetTester tester,
    ) async {
      int dragCancelCount = 0;
      final List<DragDownDetails> downDetails = <DragDownDetails>[];
      final List<DragEndDetails> endDetails = <DragEndDetails>[];
      final List<DragStartDetails> startDetails = <DragStartDetails>[];
      final List<DragUpdateDetails> updateDetails = <DragUpdateDetails>[];

      final Key redContainer = UniqueKey();
      await tester.pumpWidget(
        Center(
          child: Transform.scale(
            scale: 0.5,
            child: GestureDetector(
              onHorizontalDragCancel: () {
                dragCancelCount++;
              },
              onHorizontalDragDown: (DragDownDetails details) {
                downDetails.add(details);
              },
              onHorizontalDragEnd: (DragEndDetails details) {
                endDetails.add(details);
              },
              onHorizontalDragStart: (DragStartDetails details) {
                startDetails.add(details);
              },
              onHorizontalDragUpdate: (DragUpdateDetails details) {
                updateDetails.add(details);
              },
              onTap: () {
                // Competing gesture detector.
              },
              child: Container(key: redContainer, width: 100, height: 150, color: Colors.red),
            ),
          ),
        ),
      );

      // Move just above kTouchSlop should recognize drag.
      await tester.drag(find.byKey(redContainer), const Offset(kTouchSlop + 1, 0));

      expect(dragCancelCount, 0);
      expect(downDetails.single.localPosition, const Offset(50, 75));
      expect(downDetails.single.globalPosition, const Offset(400, 300));
      expect(endDetails, hasLength(1));
      expect(startDetails.single.localPosition, const Offset(50 + (kTouchSlop + 1) * 2, 75));
      expect(startDetails.single.globalPosition, const Offset(400 + (kTouchSlop + 1), 300));
      expect(updateDetails, isEmpty);

      dragCancelCount = 0;
      downDetails.clear();
      endDetails.clear();
      startDetails.clear();
      updateDetails.clear();

      // Move just below kTouchSlop does not recognize drag.
      await tester.drag(find.byKey(redContainer), const Offset(kTouchSlop - 1, 0));
      expect(dragCancelCount, 1);
      expect(downDetails.single.localPosition, const Offset(50, 75));
      expect(downDetails.single.globalPosition, const Offset(400, 300));
      expect(endDetails, isEmpty);
      expect(startDetails, isEmpty);
      expect(updateDetails, isEmpty);

      dragCancelCount = 0;
      downDetails.clear();
      endDetails.clear();
      startDetails.clear();
      updateDetails.clear();

      // Move in two separate movements
      final TestGesture gesture = await tester.startGesture(
        tester.getCenter(find.byKey(redContainer)),
      );
      await gesture.moveBy(const Offset(kTouchSlop + 1, 30));
      await gesture.moveBy(const Offset(100, 10));
      await gesture.up();

      expect(dragCancelCount, 0);
      expect(downDetails.single.localPosition, const Offset(50, 75));
      expect(downDetails.single.globalPosition, const Offset(400, 300));
      expect(endDetails, hasLength(1));
      expect(
        startDetails.single.localPosition,
        const Offset(50 + (kTouchSlop + 1) * 2, 75.0 + 30.0 * 2),
      );
      expect(startDetails.single.globalPosition, const Offset(400 + (kTouchSlop + 1), 300 + 30.0));
      expect(
        updateDetails.single.localPosition,
        startDetails.single.localPosition + const Offset(100.0 * 2, 10.0 * 2.0),
      );
      expect(
        updateDetails.single.globalPosition,
        startDetails.single.globalPosition + const Offset(100.0, 10.0),
      );
      expect(updateDetails.single.delta, const Offset(100.0 * 2.0, 0.0));
      expect(updateDetails.single.primaryDelta, 100.0 * 2);

      dragCancelCount = 0;
      downDetails.clear();
      endDetails.clear();
      startDetails.clear();
      updateDetails.clear();
    });

    testWidgets('kTouchSlop is evaluated in the global coordinate space when rotated 45 degrees', (
      WidgetTester tester,
    ) async {
      int dragCancelCount = 0;
      final List<DragDownDetails> downDetails = <DragDownDetails>[];
      final List<DragEndDetails> endDetails = <DragEndDetails>[];
      final List<DragStartDetails> startDetails = <DragStartDetails>[];
      final List<DragUpdateDetails> updateDetails = <DragUpdateDetails>[];

      final Key redContainer = UniqueKey();
      await tester.pumpWidget(
        Center(
          child: Transform.rotate(
            angle: math.pi / 4,
            child: GestureDetector(
              onHorizontalDragCancel: () {
                dragCancelCount++;
              },
              onHorizontalDragDown: (DragDownDetails details) {
                downDetails.add(details);
              },
              onHorizontalDragEnd: (DragEndDetails details) {
                endDetails.add(details);
              },
              onHorizontalDragStart: (DragStartDetails details) {
                startDetails.add(details);
              },
              onHorizontalDragUpdate: (DragUpdateDetails details) {
                updateDetails.add(details);
              },
              onTap: () {
                // Competing gesture detector.
              },
              child: Container(key: redContainer, width: 100, height: 150, color: Colors.red),
            ),
          ),
        ),
      );

      // Move just below kTouchSlop should not recognize drag.
      const Offset moveBy1 = Offset(kTouchSlop / 2, kTouchSlop / 2);
      expect(moveBy1.distance, lessThan(kTouchSlop));
      await tester.drag(find.byKey(redContainer), moveBy1);
      expect(dragCancelCount, 1);
      expect(
        downDetails.single.localPosition,
        within(distance: 0.0001, from: const Offset(50, 75)),
      );
      expect(
        downDetails.single.globalPosition,
        within(distance: 0.0001, from: const Offset(400, 300)),
      );
      expect(endDetails, isEmpty);
      expect(startDetails, isEmpty);
      expect(updateDetails, isEmpty);

      dragCancelCount = 0;
      downDetails.clear();
      endDetails.clear();
      startDetails.clear();
      updateDetails.clear();

      // Move above kTouchSlop recognizes drag.
      final TestGesture gesture = await tester.startGesture(
        tester.getCenter(find.byKey(redContainer)),
      );
      await gesture.moveBy(const Offset(kTouchSlop, kTouchSlop));
      await gesture.moveBy(const Offset(3, 4));
      await gesture.up();

      expect(dragCancelCount, 0);
      expect(
        downDetails.single.localPosition,
        within(distance: 0.0001, from: const Offset(50, 75)),
      );
      expect(
        downDetails.single.globalPosition,
        within(distance: 0.0001, from: const Offset(400, 300)),
      );
      expect(endDetails, hasLength(1));
      expect(startDetails, hasLength(1));
      expect(
        updateDetails.single.globalPosition,
        within(distance: 0.0001, from: const Offset(400 + kTouchSlop + 3, 300 + kTouchSlop + 4)),
      );
      expect(
        updateDetails.single.delta,
        within(distance: 0.1, from: const Offset(5, 0.0)),
      ); // sqrt(3^2 + 4^2)
      expect(
        updateDetails.single.primaryDelta,
        within<double>(distance: 0.1, from: 5.0),
      ); // sqrt(3^2 + 4^2)
    });
  });

  group('Vertical', () {
    testWidgets('gets local coordinates', (WidgetTester tester) async {
      int dragCancelCount = 0;
      final List<DragDownDetails> downDetails = <DragDownDetails>[];
      final List<DragEndDetails> endDetails = <DragEndDetails>[];
      final List<DragStartDetails> startDetails = <DragStartDetails>[];
      final List<DragUpdateDetails> updateDetails = <DragUpdateDetails>[];

      final Key redContainer = UniqueKey();
      await tester.pumpWidget(
        Center(
          child: GestureDetector(
            onVerticalDragCancel: () {
              dragCancelCount++;
            },
            onVerticalDragDown: (DragDownDetails details) {
              downDetails.add(details);
            },
            onVerticalDragEnd: (DragEndDetails details) {
              endDetails.add(details);
            },
            onVerticalDragStart: (DragStartDetails details) {
              startDetails.add(details);
            },
            onVerticalDragUpdate: (DragUpdateDetails details) {
              updateDetails.add(details);
            },
            child: Container(key: redContainer, width: 100, height: 150, color: Colors.red),
          ),
        ),
      );

      await tester.drag(find.byKey(redContainer), const Offset(0, 100));
      expect(dragCancelCount, 0);
      expect(downDetails.single.localPosition, const Offset(50, 75));
      expect(downDetails.single.globalPosition, const Offset(400, 300));
      expect(endDetails, hasLength(1));
      expect(startDetails.single.localPosition, const Offset(50, 75));
      expect(startDetails.single.globalPosition, const Offset(400, 300));
      expect(updateDetails.last.localPosition, const Offset(50, 75 + 100.0));
      expect(updateDetails.last.globalPosition, const Offset(400, 300 + 100.0));
      expect(
        updateDetails.fold(
          Offset.zero,
          (Offset offset, DragUpdateDetails details) => offset + details.delta,
        ),
        const Offset(0, 100),
      );
      expect(
        updateDetails.fold(
          0.0,
          (double offset, DragUpdateDetails details) => offset + (details.primaryDelta ?? 0),
        ),
        100.0,
      );
    });

    testWidgets('kTouchSlop is evaluated in the global coordinate space when scaled up', (
      WidgetTester tester,
    ) async {
      int dragCancelCount = 0;
      final List<DragDownDetails> downDetails = <DragDownDetails>[];
      final List<DragEndDetails> endDetails = <DragEndDetails>[];
      final List<DragStartDetails> startDetails = <DragStartDetails>[];
      final List<DragUpdateDetails> updateDetails = <DragUpdateDetails>[];

      final Key redContainer = UniqueKey();
      await tester.pumpWidget(
        Center(
          child: Transform.scale(
            scale: 2.0,
            child: GestureDetector(
              onVerticalDragCancel: () {
                dragCancelCount++;
              },
              onVerticalDragDown: (DragDownDetails details) {
                downDetails.add(details);
              },
              onVerticalDragEnd: (DragEndDetails details) {
                endDetails.add(details);
              },
              onVerticalDragStart: (DragStartDetails details) {
                startDetails.add(details);
              },
              onVerticalDragUpdate: (DragUpdateDetails details) {
                updateDetails.add(details);
              },
              onTap: () {
                // Competing gesture detector.
              },
              child: Container(key: redContainer, width: 100, height: 150, color: Colors.red),
            ),
          ),
        ),
      );

      // Move just above kTouchSlop should recognize drag.
      await tester.drag(find.byKey(redContainer), const Offset(0, kTouchSlop + 1));

      expect(dragCancelCount, 0);
      expect(downDetails.single.localPosition, const Offset(50, 75));
      expect(downDetails.single.globalPosition, const Offset(400, 300));
      expect(endDetails, hasLength(1));
      expect(startDetails.single.localPosition, const Offset(50, 75 + (kTouchSlop + 1) / 2));
      expect(startDetails.single.globalPosition, const Offset(400, 300 + (kTouchSlop + 1)));
      expect(updateDetails, isEmpty);

      dragCancelCount = 0;
      downDetails.clear();
      endDetails.clear();
      startDetails.clear();
      updateDetails.clear();

      // Move just below kTouchSlop does not recognize drag.
      await tester.drag(find.byKey(redContainer), const Offset(0, kTouchSlop - 1));
      expect(dragCancelCount, 1);
      expect(downDetails.single.localPosition, const Offset(50, 75));
      expect(downDetails.single.globalPosition, const Offset(400, 300));
      expect(endDetails, isEmpty);
      expect(startDetails, isEmpty);
      expect(updateDetails, isEmpty);

      dragCancelCount = 0;
      downDetails.clear();
      endDetails.clear();
      startDetails.clear();
      updateDetails.clear();

      // Move in two separate movements
      final TestGesture gesture = await tester.startGesture(
        tester.getCenter(find.byKey(redContainer)),
      );
      await gesture.moveBy(const Offset(30, kTouchSlop + 1));
      await gesture.moveBy(const Offset(10, 100));
      await gesture.up();

      expect(dragCancelCount, 0);
      expect(downDetails.single.localPosition, const Offset(50, 75));
      expect(downDetails.single.globalPosition, const Offset(400, 300));
      expect(endDetails, hasLength(1));
      expect(
        startDetails.single.localPosition,
        const Offset(50 + 30.0 / 2, 75.0 + (kTouchSlop + 1) / 2),
      );
      expect(startDetails.single.globalPosition, const Offset(400 + 30.0, 300 + (kTouchSlop + 1)));
      expect(
        updateDetails.single.localPosition,
        startDetails.single.localPosition + const Offset(10.0 / 2, 100.0 / 2),
      );
      expect(
        updateDetails.single.globalPosition,
        startDetails.single.globalPosition + const Offset(10.0, 100.0),
      );
      expect(updateDetails.single.delta, const Offset(0.0, 100.0 / 2));
      expect(updateDetails.single.primaryDelta, 100.0 / 2);

      dragCancelCount = 0;
      downDetails.clear();
      endDetails.clear();
      startDetails.clear();
      updateDetails.clear();
    });

    testWidgets('kTouchSlop is evaluated in the global coordinate space when scaled down', (
      WidgetTester tester,
    ) async {
      int dragCancelCount = 0;
      final List<DragDownDetails> downDetails = <DragDownDetails>[];
      final List<DragEndDetails> endDetails = <DragEndDetails>[];
      final List<DragStartDetails> startDetails = <DragStartDetails>[];
      final List<DragUpdateDetails> updateDetails = <DragUpdateDetails>[];

      final Key redContainer = UniqueKey();
      await tester.pumpWidget(
        Center(
          child: Transform.scale(
            scale: 0.5,
            child: GestureDetector(
              onVerticalDragCancel: () {
                dragCancelCount++;
              },
              onVerticalDragDown: (DragDownDetails details) {
                downDetails.add(details);
              },
              onVerticalDragEnd: (DragEndDetails details) {
                endDetails.add(details);
              },
              onVerticalDragStart: (DragStartDetails details) {
                startDetails.add(details);
              },
              onVerticalDragUpdate: (DragUpdateDetails details) {
                updateDetails.add(details);
              },
              onTap: () {
                // Competing gesture detector.
              },
              child: Container(key: redContainer, width: 100, height: 150, color: Colors.red),
            ),
          ),
        ),
      );

      // Move just above kTouchSlop should recognize drag.
      await tester.drag(find.byKey(redContainer), const Offset(0, kTouchSlop + 1));

      expect(dragCancelCount, 0);
      expect(downDetails.single.localPosition, const Offset(50, 75));
      expect(downDetails.single.globalPosition, const Offset(400, 300));
      expect(endDetails, hasLength(1));
      expect(startDetails.single.localPosition, const Offset(50, 75 + (kTouchSlop + 1) * 2));
      expect(startDetails.single.globalPosition, const Offset(400, 300 + (kTouchSlop + 1)));
      expect(updateDetails, isEmpty);

      dragCancelCount = 0;
      downDetails.clear();
      endDetails.clear();
      startDetails.clear();
      updateDetails.clear();

      // Move just below kTouchSlop does not recognize drag.
      await tester.drag(find.byKey(redContainer), const Offset(0, kTouchSlop - 1));
      expect(dragCancelCount, 1);
      expect(downDetails.single.localPosition, const Offset(50, 75));
      expect(downDetails.single.globalPosition, const Offset(400, 300));
      expect(endDetails, isEmpty);
      expect(startDetails, isEmpty);
      expect(updateDetails, isEmpty);

      dragCancelCount = 0;
      downDetails.clear();
      endDetails.clear();
      startDetails.clear();
      updateDetails.clear();

      // Move in two separate movements
      final TestGesture gesture = await tester.startGesture(
        tester.getCenter(find.byKey(redContainer)),
      );
      await gesture.moveBy(const Offset(30, kTouchSlop + 1));
      await gesture.moveBy(const Offset(10, 100));
      await gesture.up();

      expect(dragCancelCount, 0);
      expect(downDetails.single.localPosition, const Offset(50, 75));
      expect(downDetails.single.globalPosition, const Offset(400, 300));
      expect(endDetails, hasLength(1));
      expect(
        startDetails.single.localPosition,
        const Offset(50 + 30.0 * 2, 75.0 + (kTouchSlop + 1) * 2),
      );
      expect(startDetails.single.globalPosition, const Offset(400 + 30.0, 300 + (kTouchSlop + 1)));
      expect(
        updateDetails.single.localPosition,
        startDetails.single.localPosition + const Offset(10.0 * 2, 100.0 * 2.0),
      );
      expect(
        updateDetails.single.globalPosition,
        startDetails.single.globalPosition + const Offset(10.0, 100.0),
      );
      expect(updateDetails.single.delta, const Offset(0.0, 100.0 * 2.0));
      expect(updateDetails.single.primaryDelta, 100.0 * 2);

      dragCancelCount = 0;
      downDetails.clear();
      endDetails.clear();
      startDetails.clear();
      updateDetails.clear();
    });

    testWidgets('kTouchSlop is evaluated in the global coordinate space when rotated 45 degrees', (
      WidgetTester tester,
    ) async {
      int dragCancelCount = 0;
      final List<DragDownDetails> downDetails = <DragDownDetails>[];
      final List<DragEndDetails> endDetails = <DragEndDetails>[];
      final List<DragStartDetails> startDetails = <DragStartDetails>[];
      final List<DragUpdateDetails> updateDetails = <DragUpdateDetails>[];

      final Key redContainer = UniqueKey();
      await tester.pumpWidget(
        Center(
          child: Transform.rotate(
            angle: math.pi / 4,
            child: GestureDetector(
              onVerticalDragCancel: () {
                dragCancelCount++;
              },
              onVerticalDragDown: (DragDownDetails details) {
                downDetails.add(details);
              },
              onVerticalDragEnd: (DragEndDetails details) {
                endDetails.add(details);
              },
              onVerticalDragStart: (DragStartDetails details) {
                startDetails.add(details);
              },
              onVerticalDragUpdate: (DragUpdateDetails details) {
                updateDetails.add(details);
              },
              onTap: () {
                // Competing gesture detector.
              },
              child: Container(key: redContainer, width: 100, height: 150, color: Colors.red),
            ),
          ),
        ),
      );

      // Move just below kTouchSlop should not recognize drag.
      const Offset moveBy1 = Offset(kTouchSlop / 2, kTouchSlop / 2);
      expect(moveBy1.distance, lessThan(kTouchSlop));
      await tester.drag(find.byKey(redContainer), moveBy1);
      expect(dragCancelCount, 1);
      expect(
        downDetails.single.localPosition,
        within(distance: 0.0001, from: const Offset(50, 75)),
      );
      expect(
        downDetails.single.globalPosition,
        within(distance: 0.0001, from: const Offset(400, 300)),
      );
      expect(endDetails, isEmpty);
      expect(startDetails, isEmpty);
      expect(updateDetails, isEmpty);

      dragCancelCount = 0;
      downDetails.clear();
      endDetails.clear();
      startDetails.clear();
      updateDetails.clear();

      // Move above kTouchSlop recognizes drag.
      final TestGesture gesture = await tester.startGesture(
        tester.getCenter(find.byKey(redContainer)),
      );
      await gesture.moveBy(const Offset(kTouchSlop, kTouchSlop));
      await gesture.moveBy(const Offset(-4, 3));
      await gesture.up();

      expect(dragCancelCount, 0);
      expect(
        downDetails.single.localPosition,
        within(distance: 0.0001, from: const Offset(50, 75)),
      );
      expect(
        downDetails.single.globalPosition,
        within(distance: 0.0001, from: const Offset(400, 300)),
      );
      expect(endDetails, hasLength(1));
      expect(startDetails, hasLength(1));
      expect(
        updateDetails.single.globalPosition,
        within(distance: 0.0001, from: const Offset(400 + kTouchSlop - 4, 300 + kTouchSlop + 3)),
      );
      expect(
        updateDetails.single.delta,
        within(distance: 0.1, from: const Offset(0.0, 5.0)),
      ); // sqrt(3^2 + 4^2)
      expect(
        updateDetails.single.primaryDelta,
        within<double>(distance: 0.1, from: 5.0),
      ); // sqrt(3^2 + 4^2)
    });
  });
}
