// Copyright 2014 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

import 'package:flutter/material.dart';
import 'package:flutter_api_samples/widgets/magnifier/magnifier.0.dart' as example;
import 'package:flutter_test/flutter_test.dart';

void main() {
  testWidgets('should update magnifier position on drag', (WidgetTester tester) async {
    await tester.pumpWidget(const example.MagnifierExampleApp());

    Matcher isPositionedAt(Offset at) {
      return isA<Positioned>().having(
        (Positioned positioned) => Offset(positioned.left!, positioned.top!),
        'magnifier position',
        at,
      );
    }

    // Make sure magnifier is present.
    final Finder positionedWidget = find.byType(Positioned);
    final Widget positionedWidgetInTree = tester.widget(positionedWidget);
    final Positioned oldConcretePositioned = positionedWidgetInTree as Positioned;
    expect(positionedWidget, findsOneWidget);

    // Confirm if magnifier is in the center of the FlutterLogo.
    final Offset centerOfPositioned = tester.getCenter(positionedWidget);
    final Offset centerOfFlutterLogo = tester.getCenter(find.byType(FlutterLogo));
    expect(centerOfPositioned, equals(centerOfFlutterLogo));

    // Drag the magnifier and confirm its new position is expected.
    const Offset dragDistance = Offset(10, 10);
    final Offset updatedPositioned = Offset(
      oldConcretePositioned.left ?? 0.0 + 10.0,
      oldConcretePositioned.top ?? 0.0 + 10.0,
    );
    await tester.dragFrom(centerOfPositioned, dragDistance);
    await tester.pump();
    expect(positionedWidgetInTree, isPositionedAt(updatedPositioned));
  });

  testWidgets('should match golden', (WidgetTester tester) async {
    await tester.pumpWidget(const example.MagnifierExampleApp());

    final Offset centerOfPositioned = tester.getCenter(find.byType(Positioned));
    const Offset dragDistance = Offset(10, 10);

    await tester.dragFrom(centerOfPositioned, dragDistance);
    await tester.pump();

    await expectLater(find.byType(RepaintBoundary).last, matchesGoldenFile('magnifier.0_test.png'));
  });
}
