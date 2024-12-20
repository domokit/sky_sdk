// Copyright 2014 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

import 'dart:ui';

import 'package:flutter/material.dart';
import 'package:flutter_api_samples/widgets/implicit_animations/animated_positioned.0.dart'
    as example;
import 'package:flutter_test/flutter_test.dart';

void main() {
  testWidgets('AnimatedPositioned animates on tap', (WidgetTester tester) async {
    await tester.pumpWidget(const example.AnimatedPositionedExampleApp());

    final Finder positionedFinder = find.descendant(
      of: find.byType(AnimatedPositioned),
      matching: find.byType(Positioned),
    );

    const double beginWidth = 50.0;
    const double endWidth = 200.0;
    const double beginHeight = 200.0;
    const double endHeight = 50.0;
    const double beginTop = 150.0;
    const double endTop = 50.0;

    Positioned positioned = tester.widget(positionedFinder);
    expect(positioned.width, beginWidth);
    expect(positioned.height, beginHeight);
    expect(positioned.top, beginTop);

    // Tap on the 'Tap me' text to start the forward animation.
    await tester.tap(find.text('Tap me'));
    await tester.pump();

    positioned = tester.widget(positionedFinder);
    expect(positioned.width, beginWidth);
    expect(positioned.height, beginHeight);
    expect(positioned.top, beginTop);

    // Advance animation to the middle.
    await tester.pump(example.AnimatedPositionedExampleApp.duration ~/ 2);

    final double t = example.AnimatedPositionedExampleApp.curve.transform(0.5);

    positioned = tester.widget(positionedFinder);
    expect(positioned.width, lerpDouble(beginWidth, endWidth, t));
    expect(positioned.height, lerpDouble(beginHeight, endHeight, t));
    expect(positioned.top, lerpDouble(beginTop, endTop, t));

    // Advance animation to the end.
    await tester.pump(example.AnimatedPositionedExampleApp.duration ~/ 2);

    positioned = tester.widget(positionedFinder);
    expect(positioned.width, endWidth);
    expect(positioned.height, endHeight);
    expect(positioned.top, endTop);

    // Tap on the 'Tap me' text again to start the reverse animation.
    await tester.tap(find.text('Tap me'));
    await tester.pump();

    positioned = tester.widget(positionedFinder);
    expect(positioned.width, endWidth);
    expect(positioned.height, endHeight);
    expect(positioned.top, endTop);

    // Advance animation to the middle.
    await tester.pump(example.AnimatedPositionedExampleApp.duration ~/ 2);

    positioned = tester.widget(positionedFinder);
    expect(positioned.width, lerpDouble(endWidth, beginWidth, t));
    expect(positioned.height, lerpDouble(endHeight, beginHeight, t));
    expect(positioned.top, lerpDouble(endTop, beginTop, t));

    // Advance animation to the end.
    await tester.pump(example.AnimatedPositionedExampleApp.duration ~/ 2);

    positioned = tester.widget(positionedFinder);
    expect(positioned.width, beginWidth);
    expect(positioned.height, beginHeight);
    expect(positioned.top, beginTop);
  });
}
