// Copyright 2014 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

import 'package:flutter/material.dart';
import 'package:flutter_api_samples/widgets/interactive_viewer/interactive_viewer.transformation_controller.0.dart'
    as example;
import 'package:flutter_test/flutter_test.dart';

void main() {
  testWidgets('The reset button resets the view with an animation', (WidgetTester tester) async {
    await tester.pumpWidget(const example.TransformationControllerExampleApp());

    expect(find.widgetWithText(AppBar, 'Controller demo'), findsOne);
    expect(find.byType(InteractiveViewer), findsOne);

    expect(
      tester.getRect(find.byType(Container).first),
      rectMoreOrLessEquals(const Rect.fromLTRB(0.0, 56.0, 800.0, 535.0)),
    );

    await tester.drag(find.byType(Container).first, const Offset(20, 20));

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
      tester.getRect(find.byType(Container).first),
      rectMoreOrLessEquals(const Rect.fromLTRB(40.0, 96.0, 840.0, 575.0)),
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
      tester.getRect(find.byType(Container).first),
      rectMoreOrLessEquals(const Rect.fromLTRB(45.0, 96.0, 845.0, 575.0)),
    );

    await tester.tap(find.widgetWithIcon(IconButton, Icons.replay));
    await tester.pump();
    await tester.pump(const Duration(milliseconds: 200));

    expect(
      tester.getRect(find.byType(Container).first),
      rectMoreOrLessEquals(const Rect.fromLTRB(22.5, 76.0, 822.5, 555.0)),
    );

    await tester.pump(const Duration(milliseconds: 200));

    expect(
      tester.getRect(find.byType(Container).first),
      rectMoreOrLessEquals(const Rect.fromLTRB(0.0, 56.0, 800.0, 535.0)),
    );
  });
}
