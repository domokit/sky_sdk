// Copyright 2014 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

import 'package:flutter/material.dart';
import 'package:flutter_api_samples/widgets/interactive_viewer/interactive_viewer.0.dart'
    as example;
import 'package:flutter_test/flutter_test.dart';

void main() {
  testWidgets('Has correct items on screen', (WidgetTester tester) async {
    await tester.pumpWidget(const example.InteractiveViewerExampleApp());

    expect(find.widgetWithText(AppBar, 'InteractiveViewer Sample'), findsOne);
    expect(find.byType(InteractiveViewer), findsOne);

    expect(
      tester.getRect(find.byType(Container)),
      rectMoreOrLessEquals(const Rect.fromLTRB(0.0, 56.0, 800.0, 600.0)),
    );

    await tester.drag(find.byType(Container), const Offset(20, 20));

    // Pans.
    const Offset panStart = Offset(400.0, 300.0);
    final Offset panEnd = panStart + const Offset(20.0, 20.0);
    final TestGesture gesture = await tester.startGesture(panStart);
    await tester.pump();
    await gesture.moveTo(panEnd);
    await tester.pump();
    await gesture.up();
    await tester.pumpAndSettle();

    expect(
      tester.getRect(find.byType(Container)),
      rectMoreOrLessEquals(const Rect.fromLTRB(20.0, 76.0, 820.0, 620.0)),
    );

    // Zooms.
    const Offset scaleStart1 = Offset(400.0, 300.0);
    final Offset scaleStart2 = scaleStart1 + const Offset(10.0, 0.0);
    final Offset scaleEnd1 = scaleStart1 - const Offset(10.0, 0.0);
    final Offset scaleEnd2 = scaleStart2 + const Offset(10.0, 0.0);
    final TestGesture gesture1 = await tester.createGesture();
    final TestGesture gesture2 = await tester.createGesture();
    await gesture1.down(scaleStart1);
    await gesture2.down(scaleStart2);
    await tester.pump();
    await gesture1.moveTo(scaleEnd1);
    await gesture2.moveTo(scaleEnd2);
    await tester.pump();
    await gesture1.up();
    await gesture2.up();
    await tester.pumpAndSettle();

    expect(
      tester.getRect(find.byType(Container)),
      rectMoreOrLessEquals(const Rect.fromLTRB(-203.0, -58.4, 1077.0, 812.0)),
    );
  });
}
