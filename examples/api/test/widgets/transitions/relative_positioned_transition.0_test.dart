// Copyright 2014 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

import 'package:flutter/material.dart';
import 'package:flutter_api_samples/widgets/transitions/relative_positioned_transition.0.dart'
    as example;
import 'package:flutter_test/flutter_test.dart';

void main() {
  testWidgets('Shows flutter logo in transition', (WidgetTester tester) async {
    await tester.pumpWidget(const example.RelativePositionedTransitionExampleApp());
    expect(find.byType(FlutterLogo), findsOneWidget);
    expect(find.byType(Padding), findsAtLeast(1));
    expect(find.byType(RelativePositionedTransition), findsOneWidget);
  });

  testWidgets('Animates repeatedly every 2 seconds', (WidgetTester tester) async {
    await tester.pumpWidget(const example.RelativePositionedTransitionExampleApp());

    expect(
      tester.getSize(find.byType(FlutterLogo)),
      const Size(200.0 - 2.0 * 8.0, 200.0 - 2.0 * 8.0),
    );
    expect(tester.getTopLeft(find.byType(FlutterLogo)), const Offset(8.0, 8.0));

    await tester.pump(const Duration(seconds: 2));
    await tester.pump();

    final Size canvasSize = tester.getSize(find.byType(LayoutBuilder));
    expect(
      tester.getSize(find.byType(FlutterLogo)),
      const Size(100.0 - 2.0 * 8.0, 100.0 - 2.0 * 8.0),
    );
    expect(
      tester.getBottomRight(find.byType(FlutterLogo)),
      Offset(canvasSize.width - 8.0, canvasSize.height - 8.0),
    );

    await tester.pump(const Duration(seconds: 2));
    await tester.pump();

    expect(
      tester.getSize(find.byType(FlutterLogo)),
      const Size(200.0 - 2.0 * 8.0, 200.0 - 2.0 * 8.0),
    );
    expect(tester.getTopLeft(find.byType(FlutterLogo)), const Offset(8.0, 8.0));

    await tester.pump(const Duration(seconds: 2));
    await tester.pump();

    expect(
      tester.getSize(find.byType(FlutterLogo)),
      const Size(100.0 - 2.0 * 8.0, 100.0 - 2.0 * 8.0),
    );
    expect(
      tester.getBottomRight(find.byType(FlutterLogo)),
      Offset(canvasSize.width - 8.0, canvasSize.height - 8.0),
    );
  });
}
