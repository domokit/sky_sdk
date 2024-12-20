// Copyright 2014 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

import 'package:flutter/material.dart';
import 'package:flutter_api_samples/widgets/implicit_animations/animated_align.0.dart' as example;
import 'package:flutter_test/flutter_test.dart';

void main() {
  testWidgets('AnimatedAlign animates on tap', (WidgetTester tester) async {
    await tester.pumpWidget(const example.AnimatedAlignExampleApp());

    final Finder alignFinder = find.descendant(
      of: find.byType(AnimatedAlign),
      matching: find.byType(Align),
    );

    const Alignment beginAlignment = Alignment.bottomLeft;
    const Alignment endAlignment = Alignment.topRight;

    Align align = tester.widget(alignFinder);
    expect(align.alignment, beginAlignment);

    // Tap on the AnimatedAlignExample to start the forward animation.
    await tester.tap(find.byType(example.AnimatedAlignExample));
    await tester.pump();

    align = tester.widget(alignFinder);
    expect(align.alignment, beginAlignment);

    // Advance animation to the middle.
    await tester.pump(example.AnimatedAlignExampleApp.duration ~/ 2);

    align = tester.widget(alignFinder);
    expect(
      align.alignment,
      Alignment.lerp(
        beginAlignment,
        endAlignment,
        example.AnimatedAlignExampleApp.curve.transform(0.5),
      ),
    );

    // Advance animation to the end.
    await tester.pump(example.AnimatedAlignExampleApp.duration ~/ 2);

    align = tester.widget(alignFinder);
    expect(align.alignment, endAlignment);

    // Tap on the AnimatedAlignExample again to start the reverse animation.
    await tester.tap(find.byType(example.AnimatedAlignExample));
    await tester.pump();

    align = tester.widget(alignFinder);
    expect(align.alignment, endAlignment);

    // Advance animation to the middle.
    await tester.pump(example.AnimatedAlignExampleApp.duration ~/ 2);

    align = tester.widget(alignFinder);
    expect(
      align.alignment,
      Alignment.lerp(
        endAlignment,
        beginAlignment,
        example.AnimatedAlignExampleApp.curve.transform(0.5),
      ),
    );

    // Advance animation to the end.
    await tester.pump(example.AnimatedAlignExampleApp.duration ~/ 2);

    align = tester.widget(alignFinder);
    expect(align.alignment, beginAlignment);
  });
}
