// Copyright 2014 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

import 'package:flutter/gestures.dart';
import 'package:flutter/material.dart';
import 'package:flutter_test/flutter_test.dart';

void main() {
  testWidgets('kTouchSlop is evaluated in the global coordinate space when scaled up', (
    WidgetTester tester,
  ) async {
    int doubleTapCount = 0;

    final Key redContainer = UniqueKey();
    await tester.pumpWidget(
      Center(
        child: Transform.scale(
          scale: 2.0,
          child: GestureDetector(
            onDoubleTap: () {
              doubleTapCount++;
            },
            child: Container(key: redContainer, width: 100, height: 150, color: Colors.red),
          ),
        ),
      ),
    );

    // Move just below kTouchSlop should recognize tap.
    final Offset center = tester.getCenter(find.byKey(redContainer));
    TestGesture gesture = await tester.startGesture(center);
    await gesture.up();
    await tester.pump(kDoubleTapMinTime);
    gesture = await tester.startGesture(center + const Offset(kDoubleTapSlop - 1, 0));
    await gesture.up();

    expect(doubleTapCount, 1);

    doubleTapCount = 0;

    gesture = await tester.startGesture(center);
    await gesture.up();
    await tester.pump(kDoubleTapMinTime);
    gesture = await tester.startGesture(center + const Offset(kDoubleTapSlop + 1, 0));
    await gesture.up();

    expect(doubleTapCount, 0);
  });

  testWidgets('kTouchSlop is evaluated in the global coordinate space when scaled down', (
    WidgetTester tester,
  ) async {
    int doubleTapCount = 0;

    final Key redContainer = UniqueKey();
    await tester.pumpWidget(
      Center(
        child: Transform.scale(
          scale: 0.5,
          child: GestureDetector(
            onDoubleTap: () {
              doubleTapCount++;
            },
            child: Container(key: redContainer, width: 500, height: 500, color: Colors.red),
          ),
        ),
      ),
    );

    // Move just below kTouchSlop should recognize tap.
    final Offset center = tester.getCenter(find.byKey(redContainer));
    TestGesture gesture = await tester.startGesture(center);
    await gesture.up();
    await tester.pump(kDoubleTapMinTime);
    gesture = await tester.startGesture(center + const Offset(kDoubleTapSlop - 1, 0));
    await gesture.up();

    expect(doubleTapCount, 1);

    doubleTapCount = 0;

    gesture = await tester.startGesture(center);
    await gesture.up();
    await tester.pump(kDoubleTapMinTime);
    gesture = await tester.startGesture(center + const Offset(kDoubleTapSlop + 1, 0));
    await gesture.up();

    expect(doubleTapCount, 0);
  });
}
