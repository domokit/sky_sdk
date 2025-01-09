// Copyright 2014 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

import 'package:flutter/material.dart';
import 'package:flutter_api_samples/widgets/interactive_viewer/interactive_viewer.constrained.0.dart'
    as example;
import 'package:flutter_test/flutter_test.dart';

void main() {
  testWidgets('The reset button resets the view with an animation', (WidgetTester tester) async {
    await tester.pumpWidget(const example.ConstrainedExampleApp());

    expect(find.widgetWithText(AppBar, 'Constrained Sample'), findsOne);
    expect(find.byType(InteractiveViewer), findsOne);
    for (int row = 0; row < 48; row += 1) {
      for (int column = 0; column < 6; column += 1) {
        expect(find.text('$row x $column'), findsOne);
      }
    }
    expect(
      tester.getRect(find.byType(Table)),
      rectMoreOrLessEquals(const Rect.fromLTRB(0.0, 56.0, 1200.0, 1304.0)),
    );

    // Pans along the x axis.
    const Offset panStart = Offset(400.0, 300.0);
    final Offset panXEnd = panStart - const Offset(0, 20);
    final TestGesture gesturePanX = await tester.startGesture(panStart);
    await tester.pump();
    await gesturePanX.moveTo(panXEnd);
    await tester.pump();
    await gesturePanX.up();
    await tester.pumpAndSettle();

    expect(
      tester.getRect(find.byType(Table)),
      rectMoreOrLessEquals(const Rect.fromLTRB(0.0, 36.0, 1200.0, 1284.0)),
    );

    // Pans along the Y axis.
    final Offset panYEnd = panStart - const Offset(20, 0);
    final TestGesture gesturePanY = await tester.startGesture(panStart);
    await tester.pump();
    await gesturePanY.moveTo(panYEnd);
    await tester.pump();
    await gesturePanY.up();
    await tester.pumpAndSettle();

    expect(
      tester.getRect(find.byType(Table)),
      rectMoreOrLessEquals(const Rect.fromLTRB(-20.0, 36.0, 1180.0, 1284.0)),
    );

    // Tries to zooms even if it is disabled.
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
      tester.getRect(find.byType(Table)),
      rectMoreOrLessEquals(const Rect.fromLTRB(-20.0, 36.0, 1180.0, 1284.0)),
    );
  });
}
