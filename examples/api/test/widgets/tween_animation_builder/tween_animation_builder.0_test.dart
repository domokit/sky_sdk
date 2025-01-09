// Copyright 2014 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

import 'dart:ui';

import 'package:flutter/material.dart';
import 'package:flutter_api_samples/widgets/tween_animation_builder/tween_animation_builder.0.dart'
    as example;
import 'package:flutter_test/flutter_test.dart';

void main() {
  testWidgets('Animates icon size on first build', (WidgetTester tester) async {
    await tester.pumpWidget(const example.TweenAnimationBuilderExampleApp());

    // The animation duration defined in the example app.
    const Duration animationDuration = Duration(seconds: 1);

    const double beginSize = 0.0;
    const double endSize = 24.0;

    final Finder iconButtonFinder = find.byType(IconButton);

    IconButton iconButton = tester.widget(iconButtonFinder);
    expect(iconButton.iconSize, equals(beginSize));

    // Advance animation to the middle.
    await tester.pump(animationDuration ~/ 2);

    iconButton = tester.widget(iconButtonFinder);
    expect(iconButton.iconSize, equals(lerpDouble(beginSize, endSize, 0.5)));

    // Advance animation to the end.
    await tester.pump(animationDuration ~/ 2);

    iconButton = tester.widget(iconButtonFinder);
    expect(iconButton.iconSize, equals(endSize));
  });

  testWidgets('Animates icon size on IconButton tap', (WidgetTester tester) async {
    await tester.pumpWidget(const example.TweenAnimationBuilderExampleApp());
    await tester.pumpAndSettle();

    // The animation duration defined in the example app.
    const Duration animationDuration = Duration(seconds: 1);

    const double beginSize = 24.0;
    const double endSize = 48.0;

    final Finder iconButtonFinder = find.byType(IconButton);

    IconButton iconButton = tester.widget(iconButtonFinder);
    expect(iconButton.iconSize, equals(beginSize));

    // Tap on the IconButton to start the forward animation.
    await tester.tap(iconButtonFinder);
    await tester.pump();

    iconButton = tester.widget(iconButtonFinder);
    expect(iconButton.iconSize, equals(beginSize));

    // Advance animation to the middle.
    await tester.pump(animationDuration ~/ 2);

    iconButton = tester.widget(iconButtonFinder);
    expect(iconButton.iconSize, equals(lerpDouble(beginSize, endSize, 0.5)));

    // Advance animation to the end.
    await tester.pump(animationDuration ~/ 2);

    iconButton = tester.widget(iconButtonFinder);
    expect(iconButton.iconSize, equals(endSize));

    // Tap on the IconButton to start the reverse animation.
    await tester.tap(iconButtonFinder);
    await tester.pump();

    iconButton = tester.widget(iconButtonFinder);
    expect(iconButton.iconSize, equals(endSize));

    // Advance animation to the middle.
    await tester.pump(animationDuration ~/ 2);

    iconButton = tester.widget(iconButtonFinder);
    expect(iconButton.iconSize, equals(lerpDouble(endSize, beginSize, 0.5)));

    // Advance animation to the end.
    await tester.pump(animationDuration ~/ 2);

    iconButton = tester.widget(iconButtonFinder);
    expect(iconButton.iconSize, equals(beginSize));
  });

  testWidgets('Animation target can be updated during the animation', (WidgetTester tester) async {
    await tester.pumpWidget(const example.TweenAnimationBuilderExampleApp());
    await tester.pumpAndSettle();

    // The animation duration defined in the example app.
    const Duration animationDuration = Duration(seconds: 1);

    const double beginSize = 24.0;
    const double endSize = 48.0;
    final double middleSize = lerpDouble(beginSize, endSize, 0.5)!;

    final Finder iconButtonFinder = find.byType(IconButton);

    IconButton iconButton = tester.widget(iconButtonFinder);
    expect(iconButton.iconSize, equals(beginSize));

    // Tap on the IconButton to start the forward animation.
    await tester.tap(iconButtonFinder);
    await tester.pump();

    iconButton = tester.widget(iconButtonFinder);
    expect(iconButton.iconSize, equals(beginSize));

    // Advance animation to the middle.
    await tester.pump(animationDuration ~/ 2);

    iconButton = tester.widget(iconButtonFinder);
    expect(iconButton.iconSize, equals(middleSize));

    // Tap on the IconButton to start the backward animation.
    await tester.tap(iconButtonFinder);
    await tester.pump();

    iconButton = tester.widget(iconButtonFinder);
    expect(iconButton.iconSize, equals(middleSize));

    // Advance animation to the middle.
    await tester.pump(animationDuration ~/ 2);

    iconButton = tester.widget(iconButtonFinder);
    expect(iconButton.iconSize, equals(lerpDouble(middleSize, beginSize, 0.5)));

    // Advance animation to the end.
    await tester.pump(animationDuration ~/ 2);

    iconButton = tester.widget(iconButtonFinder);
    expect(iconButton.iconSize, equals(beginSize));
  });
}
