// Copyright 2014 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

import 'dart:ui';

import 'package:flutter/material.dart';
import 'package:flutter_api_samples/widgets/implicit_animations/sliver_animated_opacity.0.dart'
    as example;
import 'package:flutter_test/flutter_test.dart';

void main() {
  testWidgets('SilverAnimatedOpacity animates on FloatingActionButton tap', (
    WidgetTester tester,
  ) async {
    await tester.pumpWidget(const example.SliverAnimatedOpacityExampleApp());

    final Finder fadeTransitionFinder = find.descendant(
      of: find.byType(SliverAnimatedOpacity),
      matching: find.byType(SliverFadeTransition),
    );

    const double beginOpacity = 1.0;
    const double endOpacity = 0.0;

    SliverFadeTransition fadeTransition = tester.widget(fadeTransitionFinder);
    expect(fadeTransition.opacity.value, beginOpacity);

    // Tap on the FloatingActionButton to start the forward animation.
    await tester.tap(find.byType(FloatingActionButton));
    await tester.pump();

    fadeTransition = tester.widget(fadeTransitionFinder);
    expect(fadeTransition.opacity.value, beginOpacity);

    // Advance animation to the middle.
    await tester.pump(example.SliverAnimatedOpacityExampleApp.duration ~/ 2);

    fadeTransition = tester.widget(fadeTransitionFinder);
    expect(
      fadeTransition.opacity.value,
      lerpDouble(
        beginOpacity,
        endOpacity,
        example.SliverAnimatedOpacityExampleApp.curve.transform(0.5),
      ),
    );

    // Advance animation to the end.
    await tester.pump(example.SliverAnimatedOpacityExampleApp.duration ~/ 2);

    fadeTransition = tester.widget(fadeTransitionFinder);
    expect(fadeTransition.opacity.value, endOpacity);

    // Tap on the FloatingActionButton again to start the reverse animation.
    await tester.tap(find.byType(FloatingActionButton));
    await tester.pump();

    fadeTransition = tester.widget(fadeTransitionFinder);
    expect(fadeTransition.opacity.value, endOpacity);

    // Advance animation to the middle.
    await tester.pump(example.SliverAnimatedOpacityExampleApp.duration ~/ 2);

    fadeTransition = tester.widget(fadeTransitionFinder);
    expect(
      fadeTransition.opacity.value,
      lerpDouble(
        endOpacity,
        beginOpacity,
        example.SliverAnimatedOpacityExampleApp.curve.transform(0.5),
      ),
    );

    // Advance animation to the end.
    await tester.pump(example.SliverAnimatedOpacityExampleApp.duration ~/ 2);

    fadeTransition = tester.widget(fadeTransitionFinder);
    expect(fadeTransition.opacity.value, beginOpacity);
  });
}
