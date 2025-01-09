// Copyright 2014 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

import 'package:flutter/material.dart';
import 'package:flutter_api_samples/widgets/transitions/align_transition.0.dart' as example;
import 'package:flutter_test/flutter_test.dart';

void main() {
  testWidgets('Shows flutter logo in transition', (WidgetTester tester) async {
    await tester.pumpWidget(const example.AlignTransitionExampleApp());
    expect(find.byType(ColoredBox), findsOneWidget);
    expect(
      find.byWidgetPredicate(
        (Widget padding) => padding is Padding && padding.padding == const EdgeInsets.all(8.0),
      ),
      findsOneWidget,
    );
    expect(find.byType(FlutterLogo), findsOneWidget);
    expect(find.byType(AlignTransition), findsOneWidget);
  });

  testWidgets('Animates repeatedly every 2 seconds', (WidgetTester tester) async {
    await tester.pumpWidget(const example.AlignTransitionExampleApp());
    final Finder paddingFinder = find.byWidgetPredicate(
      (Widget padding) => padding is Padding && padding.padding == const EdgeInsets.all(8.0),
    );

    expect(tester.getBottomLeft(paddingFinder), tester.getBottomLeft(find.byType(AlignTransition)));

    await tester.pump(const Duration(seconds: 2));
    await tester.pump();

    expect(tester.getCenter(paddingFinder), tester.getCenter(find.byType(AlignTransition)));

    await tester.pump(const Duration(seconds: 2));
    await tester.pump();

    expect(tester.getBottomLeft(paddingFinder), tester.getBottomLeft(find.byType(AlignTransition)));
  });
}
