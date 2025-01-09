// Copyright 2014 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

import 'package:flutter/material.dart';
import 'package:flutter_api_samples/material/scaffold/scaffold_state.show_bottom_sheet.0.dart'
    as example;
import 'package:flutter_test/flutter_test.dart';

void main() {
  testWidgets('The button should show a bottom sheet when pressed', (WidgetTester tester) async {
    await tester.pumpWidget(const example.ShowBottomSheetExampleApp());

    expect(find.widgetWithText(AppBar, 'ScaffoldState Sample'), findsOne);
    await tester.tap(find.widgetWithText(ElevatedButton, 'showBottomSheet'));
    await tester.pumpAndSettle();

    expect(find.widgetWithText(BottomSheet, 'BottomSheet'), findsOne);

    await tester.tap(find.widgetWithText(ElevatedButton, 'Close BottomSheet'));
    await tester.pumpAndSettle();

    expect(find.widgetWithText(BottomSheet, 'BottomSheet'), findsNothing);
  });
}
