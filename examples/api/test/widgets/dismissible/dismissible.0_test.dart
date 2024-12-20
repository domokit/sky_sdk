// Copyright 2014 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

import 'package:flutter/material.dart';
import 'package:flutter_api_samples/widgets/dismissible/dismissible.0.dart' as example;
import 'package:flutter_test/flutter_test.dart';

void main() {
  Future<void> dismissHorizontally({
    required WidgetTester tester,
    required Finder finder,
    required AxisDirection direction,
  }) async {
    final double width = (tester.renderObject(finder) as RenderBox).size.width;
    final double dx = width * 0.8;

    final Offset offset = switch (direction) {
      AxisDirection.left => Offset(-dx, 0.0),
      AxisDirection.right => Offset(dx, 0.0),
      _ => throw ArgumentError('$direction is not supported'),
    };

    await tester.drag(finder, offset);
  }

  testWidgets('ListTiles can be dismissed from right to left', (WidgetTester tester) async {
    await tester.pumpWidget(const example.DismissibleExampleApp());

    for (final int index in <int>[0, 33, 66, 99]) {
      final ValueKey<int> key = ValueKey<int>(index);

      await tester.scrollUntilVisible(find.byKey(key), 100);

      expect(find.byKey(key), findsOneWidget);

      await dismissHorizontally(
        tester: tester,
        finder: find.byKey(key),
        direction: AxisDirection.left,
      );

      await tester.pumpAndSettle();

      expect(find.byKey(key), findsNothing);
    }
  });

  testWidgets('ListTiles can be dismissed from left to right', (WidgetTester tester) async {
    await tester.pumpWidget(const example.DismissibleExampleApp());

    for (final int index in <int>[0, 33, 66, 99]) {
      final ValueKey<int> key = ValueKey<int>(index);

      await tester.scrollUntilVisible(find.byKey(key), 100);

      expect(find.byKey(key), findsOneWidget);

      await dismissHorizontally(
        tester: tester,
        finder: find.byKey(key),
        direction: AxisDirection.right,
      );

      await tester.pumpAndSettle();

      expect(find.byKey(key), findsNothing);
    }
  });
}
