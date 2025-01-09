// Copyright 2014 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

import 'dart:ui';

import 'package:flutter/material.dart';
import 'package:flutter_api_samples/widgets/implicit_animations/animated_fractionally_sized_box.0.dart'
    as example;
import 'package:flutter_test/flutter_test.dart';

void main() {
  testWidgets('AnimatedFractionallySizedBox animates on tap', (WidgetTester tester) async {
    await tester.pumpWidget(const example.AnimatedFractionallySizedBoxExampleApp());

    final Finder fractionallySizedBoxFinder = find.descendant(
      of: find.byType(AnimatedFractionallySizedBox),
      matching: find.byType(FractionallySizedBox),
    );

    const double beginWidthFactor = 0.75;
    const double endWidthFactor = 0.25;
    const double beginHeightFactor = 0.25;
    const double endHeightFactor = 0.75;
    const Alignment beginAlignment = Alignment.bottomRight;
    const Alignment endAlignment = Alignment.topLeft;

    FractionallySizedBox fractionallySizedBox = tester.widget(fractionallySizedBoxFinder);
    expect(fractionallySizedBox.widthFactor, beginWidthFactor);
    expect(fractionallySizedBox.heightFactor, beginHeightFactor);
    expect(fractionallySizedBox.alignment, beginAlignment);

    // Tap on the AnimatedFractionallySizedBoxExample to start the forward
    // animation.
    await tester.tap(find.byType(example.AnimatedFractionallySizedBoxExample));
    await tester.pump();

    fractionallySizedBox = tester.widget(fractionallySizedBoxFinder);
    expect(fractionallySizedBox.widthFactor, beginWidthFactor);
    expect(fractionallySizedBox.heightFactor, beginHeightFactor);
    expect(fractionallySizedBox.alignment, beginAlignment);

    // Advance animation to the middle.
    await tester.pump(example.AnimatedFractionallySizedBoxExampleApp.duration ~/ 2);

    final double t = example.AnimatedFractionallySizedBoxExampleApp.curve.transform(0.5);

    fractionallySizedBox = tester.widget(fractionallySizedBoxFinder);
    expect(fractionallySizedBox.widthFactor, lerpDouble(beginWidthFactor, endWidthFactor, t));
    expect(fractionallySizedBox.heightFactor, lerpDouble(beginHeightFactor, endHeightFactor, t));
    expect(fractionallySizedBox.alignment, Alignment.lerp(beginAlignment, endAlignment, t));

    // Advance animation to the end.
    await tester.pump(example.AnimatedFractionallySizedBoxExampleApp.duration ~/ 2);

    fractionallySizedBox = tester.widget(fractionallySizedBoxFinder);
    expect(fractionallySizedBox.widthFactor, endWidthFactor);
    expect(fractionallySizedBox.heightFactor, endHeightFactor);
    expect(fractionallySizedBox.alignment, endAlignment);

    // Tap on the AnimatedFractionallySizedBoxExample again to start the
    // reverse animation.
    await tester.tap(find.byType(example.AnimatedFractionallySizedBoxExample));
    await tester.pump();

    fractionallySizedBox = tester.widget(fractionallySizedBoxFinder);
    expect(fractionallySizedBox.widthFactor, endWidthFactor);
    expect(fractionallySizedBox.heightFactor, endHeightFactor);
    expect(fractionallySizedBox.alignment, endAlignment);

    // Advance animation to the middle.
    await tester.pump(example.AnimatedFractionallySizedBoxExampleApp.duration ~/ 2);

    fractionallySizedBox = tester.widget(fractionallySizedBoxFinder);
    expect(fractionallySizedBox.widthFactor, lerpDouble(endWidthFactor, beginWidthFactor, t));
    expect(fractionallySizedBox.heightFactor, lerpDouble(endHeightFactor, beginHeightFactor, t));
    expect(fractionallySizedBox.alignment, Alignment.lerp(endAlignment, beginAlignment, t));

    // Advance animation to the end.
    await tester.pump(example.AnimatedFractionallySizedBoxExampleApp.duration ~/ 2);

    fractionallySizedBox = tester.widget(fractionallySizedBoxFinder);
    expect(fractionallySizedBox.widthFactor, beginWidthFactor);
    expect(fractionallySizedBox.heightFactor, beginHeightFactor);
    expect(fractionallySizedBox.alignment, beginAlignment);
  });
}
