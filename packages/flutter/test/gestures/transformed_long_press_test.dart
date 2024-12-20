// Copyright 2014 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

import 'package:flutter/gestures.dart';
import 'package:flutter/material.dart';
import 'package:flutter_test/flutter_test.dart';

void main() {
  testWidgets('gets local coordinates', (WidgetTester tester) async {
    int longPressCount = 0;
    int longPressUpCount = 0;
    final List<LongPressEndDetails> endDetails = <LongPressEndDetails>[];
    final List<LongPressMoveUpdateDetails> moveDetails = <LongPressMoveUpdateDetails>[];
    final List<LongPressStartDetails> startDetails = <LongPressStartDetails>[];

    final Key redContainer = UniqueKey();
    await tester.pumpWidget(
      Center(
        child: GestureDetector(
          onLongPress: () {
            longPressCount++;
          },
          onLongPressEnd: (LongPressEndDetails details) {
            endDetails.add(details);
          },
          onLongPressMoveUpdate: (LongPressMoveUpdateDetails details) {
            moveDetails.add(details);
          },
          onLongPressStart: (LongPressStartDetails details) {
            startDetails.add(details);
          },
          onLongPressUp: () {
            longPressUpCount++;
          },
          child: Container(key: redContainer, width: 100, height: 150, color: Colors.red),
        ),
      ),
    );

    await tester.longPressAt(tester.getCenter(find.byKey(redContainer)));
    expect(longPressCount, 1);
    expect(longPressUpCount, 1);
    expect(moveDetails, isEmpty);
    expect(startDetails.single.localPosition, const Offset(50, 75));
    expect(startDetails.single.globalPosition, const Offset(400, 300));
    expect(endDetails.single.localPosition, const Offset(50, 75));
    expect(endDetails.single.globalPosition, const Offset(400, 300));
  });

  testWidgets('scaled up', (WidgetTester tester) async {
    int longPressCount = 0;
    int longPressUpCount = 0;
    final List<LongPressEndDetails> endDetails = <LongPressEndDetails>[];
    final List<LongPressMoveUpdateDetails> moveDetails = <LongPressMoveUpdateDetails>[];
    final List<LongPressStartDetails> startDetails = <LongPressStartDetails>[];

    final Key redContainer = UniqueKey();
    await tester.pumpWidget(
      Center(
        child: Transform.scale(
          scale: 2.0,
          child: GestureDetector(
            onLongPress: () {
              longPressCount++;
            },
            onLongPressEnd: (LongPressEndDetails details) {
              endDetails.add(details);
            },
            onLongPressMoveUpdate: (LongPressMoveUpdateDetails details) {
              moveDetails.add(details);
            },
            onLongPressStart: (LongPressStartDetails details) {
              startDetails.add(details);
            },
            onLongPressUp: () {
              longPressUpCount++;
            },
            child: Container(key: redContainer, width: 100, height: 150, color: Colors.red),
          ),
        ),
      ),
    );

    TestGesture gesture = await tester.startGesture(tester.getCenter(find.byKey(redContainer)));
    await gesture.moveBy(const Offset(0, 10.0));
    await tester.pump(kLongPressTimeout);
    await gesture.up();

    expect(longPressCount, 1);
    expect(longPressUpCount, 1);
    expect(startDetails.single.localPosition, const Offset(50, 75));
    expect(startDetails.single.globalPosition, const Offset(400, 300));
    expect(endDetails.single.localPosition, const Offset(50, 75 + 10.0 / 2.0));
    expect(endDetails.single.globalPosition, const Offset(400, 300.0 + 10.0));
    expect(moveDetails, isEmpty); // moved before long press was detected.

    startDetails.clear();
    endDetails.clear();
    longPressCount = 0;
    longPressUpCount = 0;

    // Move after recognized.
    gesture = await tester.startGesture(tester.getCenter(find.byKey(redContainer)));
    await tester.pump(kLongPressTimeout);
    await gesture.moveBy(const Offset(0, 100));
    await gesture.up();

    expect(longPressCount, 1);
    expect(longPressUpCount, 1);
    expect(startDetails.single.localPosition, const Offset(50, 75));
    expect(startDetails.single.globalPosition, const Offset(400, 300));
    expect(endDetails.single.localPosition, const Offset(50, 75 + 100.0 / 2.0));
    expect(endDetails.single.globalPosition, const Offset(400, 300.0 + 100.0));
    expect(moveDetails.single.localPosition, const Offset(50, 75 + 100.0 / 2.0));
    expect(moveDetails.single.globalPosition, const Offset(400, 300.0 + 100.0));
    expect(moveDetails.single.offsetFromOrigin, const Offset(0, 100.0));
    expect(moveDetails.single.localOffsetFromOrigin, const Offset(0, 100.0 / 2.0));
  });

  testWidgets('scaled down', (WidgetTester tester) async {
    int longPressCount = 0;
    int longPressUpCount = 0;
    final List<LongPressEndDetails> endDetails = <LongPressEndDetails>[];
    final List<LongPressMoveUpdateDetails> moveDetails = <LongPressMoveUpdateDetails>[];
    final List<LongPressStartDetails> startDetails = <LongPressStartDetails>[];

    final Key redContainer = UniqueKey();
    await tester.pumpWidget(
      Center(
        child: Transform.scale(
          scale: 0.5,
          child: GestureDetector(
            onLongPress: () {
              longPressCount++;
            },
            onLongPressEnd: (LongPressEndDetails details) {
              endDetails.add(details);
            },
            onLongPressMoveUpdate: (LongPressMoveUpdateDetails details) {
              moveDetails.add(details);
            },
            onLongPressStart: (LongPressStartDetails details) {
              startDetails.add(details);
            },
            onLongPressUp: () {
              longPressUpCount++;
            },
            child: Container(key: redContainer, width: 100, height: 150, color: Colors.red),
          ),
        ),
      ),
    );

    TestGesture gesture = await tester.startGesture(tester.getCenter(find.byKey(redContainer)));
    await gesture.moveBy(const Offset(0, 10.0));
    await tester.pump(kLongPressTimeout);
    await gesture.up();

    expect(longPressCount, 1);
    expect(longPressUpCount, 1);
    expect(startDetails.single.localPosition, const Offset(50, 75));
    expect(startDetails.single.globalPosition, const Offset(400, 300));
    expect(endDetails.single.localPosition, const Offset(50, 75 + 10.0 * 2.0));
    expect(endDetails.single.globalPosition, const Offset(400, 300.0 + 10.0));
    expect(moveDetails, isEmpty); // moved before long press was detected.

    startDetails.clear();
    endDetails.clear();
    longPressCount = 0;
    longPressUpCount = 0;

    // Move after recognized.
    gesture = await tester.startGesture(tester.getCenter(find.byKey(redContainer)));
    await tester.pump(kLongPressTimeout);
    await gesture.moveBy(const Offset(0, 100));
    await gesture.up();

    expect(longPressCount, 1);
    expect(longPressUpCount, 1);
    expect(startDetails.single.localPosition, const Offset(50, 75));
    expect(startDetails.single.globalPosition, const Offset(400, 300));
    expect(endDetails.single.localPosition, const Offset(50, 75 + 100.0 * 2.0));
    expect(endDetails.single.globalPosition, const Offset(400, 300.0 + 100.0));
    expect(moveDetails.single.localPosition, const Offset(50, 75 + 100.0 * 2.0));
    expect(moveDetails.single.globalPosition, const Offset(400, 300.0 + 100.0));
    expect(moveDetails.single.offsetFromOrigin, const Offset(0, 100.0));
    expect(moveDetails.single.localOffsetFromOrigin, const Offset(0, 100.0 * 2.0));
  });
}
