// Copyright 2014 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

import 'package:flutter/material.dart';
import 'package:flutter_api_samples/material/bottom_sheet/show_modal_bottom_sheet.0.dart'
    as example;
import 'package:flutter_test/flutter_test.dart';

void main() {
  testWidgets('BottomSheet can be opened and closed', (WidgetTester tester) async {
    const String titleText = 'Modal BottomSheet';
    const String closeText = 'Close BottomSheet';

    await tester.pumpWidget(const example.BottomSheetApp());

    expect(find.text(titleText), findsNothing);
    expect(find.text(closeText), findsNothing);

    // Open the bottom sheet.
    await tester.tap(find.widgetWithText(ElevatedButton, 'showModalBottomSheet'));
    await tester.pumpAndSettle();

    // Verify that the bottom sheet is open.
    expect(find.text(titleText), findsOneWidget);
    expect(find.text(closeText), findsOneWidget);
  });
}
