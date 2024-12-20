// Copyright 2014 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

import 'package:flutter/material.dart';
import 'package:flutter_api_samples/material/bottom_sheet/show_modal_bottom_sheet.2.dart'
    as example;
import 'package:flutter_test/flutter_test.dart';

void main() {
  testWidgets('Modal bottom sheet animation can be customized using AnimationStyle', (
    WidgetTester tester,
  ) async {
    await tester.pumpWidget(const example.ModalBottomSheetApp());

    // Show the bottom sheet with default animation style.
    await tester.tap(find.widgetWithText(ElevatedButton, 'showModalBottomSheet'));
    await tester.pump();
    // Advance the animation by 1/2 of the default forward duration.
    await tester.pump(const Duration(milliseconds: 125));

    // The bottom sheet is partially visible.
    expect(tester.getTopLeft(find.byType(BottomSheet)).dy, closeTo(316.7, 0.1));

    // Advance the animation by 1/2 of the default forward duration.
    await tester.pump(const Duration(milliseconds: 125));

    // The bottom sheet is fully visible.
    expect(tester.getTopLeft(find.byType(BottomSheet)).dy, equals(262.5));

    // Dismiss the bottom sheet.
    await tester.tap(find.widgetWithText(ElevatedButton, 'Close'));
    await tester.pumpAndSettle();

    // Select custom animation style.
    await tester.tap(find.text('Custom'));
    await tester.pumpAndSettle();

    // Show the bottom sheet with custom animation style.
    await tester.tap(find.widgetWithText(ElevatedButton, 'showModalBottomSheet'));
    await tester.pump();
    // Advance the animation by 1/2 of the custom forward duration.
    await tester.pump(const Duration(milliseconds: 1500));

    // The bottom sheet is partially visible.
    expect(tester.getTopLeft(find.byType(BottomSheet)).dy, closeTo(316.7, 0.1));

    // Advance the animation by 1/2 of the custom forward duration.
    await tester.pump(const Duration(milliseconds: 1500));

    // The bottom sheet is fully visible.
    expect(tester.getTopLeft(find.byType(BottomSheet)).dy, equals(262.5));

    // Dismiss the bottom sheet.
    await tester.tap(find.widgetWithText(ElevatedButton, 'Close'));
    await tester.pumpAndSettle();

    // Select no animation style.
    await tester.tap(find.text('None'));
    await tester.pumpAndSettle();

    // Show the bottom sheet with no animation style.
    await tester.tap(find.widgetWithText(ElevatedButton, 'showModalBottomSheet'));
    await tester.pump();
    expect(tester.getTopLeft(find.byType(BottomSheet)).dy, equals(262.5));
  });
}
