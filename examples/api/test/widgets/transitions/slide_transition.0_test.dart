// Copyright 2014 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

import 'package:flutter/material.dart';
import 'package:flutter_api_samples/widgets/transitions/slide_transition.0.dart' as example;
import 'package:flutter_test/flutter_test.dart';

void main() {
  testWidgets('Shows flutter logo in transition', (WidgetTester tester) async {
    await tester.pumpWidget(const example.SlideTransitionExampleApp());
    expect(find.text('SlideTransition Sample'), findsOneWidget);
    expect(find.byType(Center), findsOneWidget);
    expect(find.byType(FlutterLogo), findsOneWidget);
    expect(find.byType(Padding), findsAtLeast(1));
    expect(find.byType(SlideTransition), findsOneWidget);
  });

  testWidgets('Animates repeatedly every 2 seconds', (WidgetTester tester) async {
    await tester.pumpWidget(const example.SlideTransitionExampleApp());

    expect(tester.getCenter(find.byType(FlutterLogo)), tester.getCenter(find.byType(Center)));

    await tester.pump(const Duration(seconds: 2));
    await tester.pump();

    final double width = tester.getSize(find.byType(FlutterLogo)).width + 2 * 8.0;
    expect(
      tester.getCenter(find.byType(FlutterLogo)).dx,
      tester.getCenter(find.byType(Center)).dx + 1.5 * width,
    );

    await tester.pump(const Duration(seconds: 2));
    await tester.pump();

    expect(tester.getCenter(find.byType(FlutterLogo)), tester.getCenter(find.byType(Center)));
  });
}
