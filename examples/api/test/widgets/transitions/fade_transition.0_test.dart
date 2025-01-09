// Copyright 2014 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

import 'dart:ui';

import 'package:flutter/material.dart';
import 'package:flutter_api_samples/widgets/transitions/fade_transition.0.dart' as example;
import 'package:flutter_test/flutter_test.dart';

void main() {
  testWidgets('Shows FlutterLogo inside a FadeTransition', (WidgetTester tester) async {
    await tester.pumpWidget(const example.FadeTransitionExampleApp());

    expect(
      find.ancestor(of: find.byType(FlutterLogo), matching: find.byType(FadeTransition)),
      findsOneWidget,
    );
  });

  testWidgets('FadeTransition animates', (WidgetTester tester) async {
    await tester.pumpWidget(const example.FadeTransitionExampleApp());

    final Finder fadeTransitionFinder = find.ancestor(
      of: find.byType(FlutterLogo),
      matching: find.byType(FadeTransition),
    );

    const double beginOpacity = 0.0;
    const double endOpacity = 1.0;

    FadeTransition fadeTransition = tester.widget(fadeTransitionFinder);
    expect(fadeTransition.opacity.value, equals(beginOpacity));

    // Advance animation to the middle.
    await tester.pump(example.FadeTransitionExampleApp.duration ~/ 2);

    final double t = example.FadeTransitionExampleApp.curve.transform(0.5);

    fadeTransition = tester.widget(fadeTransitionFinder);
    expect(fadeTransition.opacity.value, equals(lerpDouble(beginOpacity, endOpacity, t)));

    // Advance animation to the end.
    await tester.pump(example.FadeTransitionExampleApp.duration ~/ 2);

    fadeTransition = tester.widget(fadeTransitionFinder);
    expect(fadeTransition.opacity.value, equals(endOpacity));

    // Advance animation to the middle.
    await tester.pump(example.FadeTransitionExampleApp.duration ~/ 2);

    fadeTransition = tester.widget(fadeTransitionFinder);
    expect(fadeTransition.opacity.value, equals(lerpDouble(beginOpacity, endOpacity, t)));

    // Advance animation to the end.
    await tester.pump(example.FadeTransitionExampleApp.duration ~/ 2);

    fadeTransition = tester.widget(fadeTransitionFinder);
    expect(fadeTransition.opacity.value, equals(beginOpacity));
  });
}
