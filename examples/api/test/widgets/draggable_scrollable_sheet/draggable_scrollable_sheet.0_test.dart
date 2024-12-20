// Copyright 2014 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

import 'package:flutter/material.dart';
import 'package:flutter_api_samples/widgets/draggable_scrollable_sheet/draggable_scrollable_sheet.0.dart'
    as example;
import 'package:flutter_test/flutter_test.dart';

void main() {
  testWidgets('Test DraggableScrollableSheet initial state', (WidgetTester tester) async {
    await tester.pumpWidget(const example.DraggableScrollableSheetExampleApp());

    final Finder sheetFinder = find.byType(DraggableScrollableSheet);

    // Verify that DraggableScrollableSheet is initially present
    expect(sheetFinder, findsOneWidget);

    // Verify that DraggableScrollableSheet is shown initially at 50% height
    final DraggableScrollableSheet draggableSheet = tester.widget(sheetFinder);
    expect(draggableSheet.initialChildSize, 0.5);
  });

  testWidgets(
    'Test DraggableScrollableSheet drag behavior on mobile platforms',
    (WidgetTester tester) async {
      await tester.pumpWidget(const example.DraggableScrollableSheetExampleApp());

      // Verify that ListView is visible
      final Finder listViewFinder = find.byType(ListView);
      expect(listViewFinder, findsOneWidget);

      // Get the initial size of the ListView
      final Size listViewInitialSize = tester.getSize(listViewFinder);

      // Drag the sheet from anywhere inside the sheet to change the sheet position
      await tester.drag(listViewFinder, const Offset(0.0, -100.0));
      await tester.pump();

      // Verify that the ListView is expanded
      final Size listViewCurrentSize = tester.getSize(listViewFinder);
      expect(listViewCurrentSize.height, greaterThan(listViewInitialSize.height));
    },
    variant: TargetPlatformVariant.mobile(),
  );

  testWidgets(
    'Test DraggableScrollableSheet drag behavior on desktop platforms',
    (WidgetTester tester) async {
      await tester.pumpWidget(const example.DraggableScrollableSheetExampleApp());

      // Verify that Grabber is visible
      final Finder grabberFinder = find.byType(example.Grabber);
      expect(grabberFinder, findsOneWidget);

      // Drag the Grabber to change the sheet position
      await tester.drag(grabberFinder, const Offset(0.0, -100.0));
      await tester.pump();

      // Verify that the DraggableScrollableSheet's initialChildSize is updated
      final DraggableScrollableSheet draggableSheet = tester.widget(
        find.byType(DraggableScrollableSheet),
      );
      expect(draggableSheet.initialChildSize, isNot(0.5));
    },
    variant: TargetPlatformVariant.desktop(),
  );
}
