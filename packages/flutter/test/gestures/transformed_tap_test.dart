// Copyright 2014 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

import 'package:flutter/gestures.dart';
import 'package:flutter/material.dart';
import 'package:flutter_test/flutter_test.dart';

void main() {
  testWidgets('gets local coordinates', (WidgetTester tester) async {
    int tapCount = 0;
    int tapCancelCount = 0;
    final List<TapDownDetails> downDetails = <TapDownDetails>[];
    final List<TapUpDetails> upDetails = <TapUpDetails>[];

    final Key redContainer = UniqueKey();
    await tester.pumpWidget(
      Center(
        child: GestureDetector(
          onTap: () {
            tapCount++;
          },
          onTapCancel: () {
            tapCancelCount++;
          },
          onTapDown: (TapDownDetails details) {
            downDetails.add(details);
          },
          onTapUp: (TapUpDetails details) {
            upDetails.add(details);
          },
          child: Container(key: redContainer, width: 100, height: 150, color: Colors.red),
        ),
      ),
    );

    await tester.tapAt(tester.getCenter(find.byKey(redContainer)));
    expect(tapCount, 1);
    expect(tapCancelCount, 0);
    expect(downDetails.single.localPosition, const Offset(50, 75));
    expect(downDetails.single.globalPosition, const Offset(400, 300));
    expect(upDetails.single.localPosition, const Offset(50, 75));
    expect(upDetails.single.globalPosition, const Offset(400, 300));
  });

  testWidgets('kTouchSlop is evaluated in the global coordinate space when scaled up', (
    WidgetTester tester,
  ) async {
    int tapCount = 0;
    int tapCancelCount = 0;
    final List<TapDownDetails> downDetails = <TapDownDetails>[];
    final List<TapUpDetails> upDetails = <TapUpDetails>[];

    final Key redContainer = UniqueKey();
    await tester.pumpWidget(
      Center(
        child: Transform.scale(
          scale: 2.0,
          child: GestureDetector(
            onTap: () {
              tapCount++;
            },
            onTapCancel: () {
              tapCancelCount++;
            },
            onTapDown: (TapDownDetails details) {
              downDetails.add(details);
            },
            onTapUp: (TapUpDetails details) {
              upDetails.add(details);
            },
            child: Container(key: redContainer, width: 100, height: 150, color: Colors.red),
          ),
        ),
      ),
    );

    // Move just below kTouchSlop should recognize tap.
    TestGesture gesture = await tester.startGesture(tester.getCenter(find.byKey(redContainer)));
    await gesture.moveBy(const Offset(0, kTouchSlop - 1));
    await gesture.up();

    expect(tapCount, 1);
    expect(tapCancelCount, 0);
    expect(downDetails.single.localPosition, const Offset(50, 75));
    expect(downDetails.single.globalPosition, const Offset(400, 300));
    expect(upDetails.single.localPosition, const Offset(50, 75 + (kTouchSlop - 1) / 2.0));
    expect(upDetails.single.globalPosition, const Offset(400, 300 + (kTouchSlop - 1)));

    downDetails.clear();
    upDetails.clear();
    tapCount = 0;
    tapCancelCount = 0;

    // Move more then kTouchSlop should cancel.
    gesture = await tester.startGesture(tester.getCenter(find.byKey(redContainer)));
    await gesture.moveBy(const Offset(0, kTouchSlop + 1));
    await gesture.up();
    expect(tapCount, 0);
    expect(tapCancelCount, 1);
    expect(downDetails.single.localPosition, const Offset(50, 75));
    expect(downDetails.single.globalPosition, const Offset(400, 300));
    expect(upDetails, isEmpty);
  });

  testWidgets('kTouchSlop is evaluated in the global coordinate space when scaled down', (
    WidgetTester tester,
  ) async {
    int tapCount = 0;
    int tapCancelCount = 0;
    final List<TapDownDetails> downDetails = <TapDownDetails>[];
    final List<TapUpDetails> upDetails = <TapUpDetails>[];

    final Key redContainer = UniqueKey();
    await tester.pumpWidget(
      Center(
        child: Transform.scale(
          scale: 0.5,
          child: GestureDetector(
            onTap: () {
              tapCount++;
            },
            onTapCancel: () {
              tapCancelCount++;
            },
            onTapDown: (TapDownDetails details) {
              downDetails.add(details);
            },
            onTapUp: (TapUpDetails details) {
              upDetails.add(details);
            },
            child: Container(key: redContainer, width: 100, height: 150, color: Colors.red),
          ),
        ),
      ),
    );

    // Move just below kTouchSlop should recognize tap.
    TestGesture gesture = await tester.startGesture(tester.getCenter(find.byKey(redContainer)));
    await gesture.moveBy(const Offset(0, kTouchSlop - 1));
    await gesture.up();

    expect(tapCount, 1);
    expect(tapCancelCount, 0);
    expect(downDetails.single.localPosition, const Offset(50, 75));
    expect(downDetails.single.globalPosition, const Offset(400, 300));
    expect(upDetails.single.localPosition, const Offset(50, 75 + (kTouchSlop - 1) * 2.0));
    expect(upDetails.single.globalPosition, const Offset(400, 300 + (kTouchSlop - 1)));

    downDetails.clear();
    upDetails.clear();
    tapCount = 0;
    tapCancelCount = 0;

    // Move more then kTouchSlop should cancel.
    gesture = await tester.startGesture(tester.getCenter(find.byKey(redContainer)));
    await gesture.moveBy(const Offset(0, kTouchSlop + 1));
    await gesture.up();
    expect(tapCount, 0);
    expect(tapCancelCount, 1);
    expect(downDetails.single.localPosition, const Offset(50, 75));
    expect(downDetails.single.globalPosition, const Offset(400, 300));
    expect(upDetails, isEmpty);
  });
}
