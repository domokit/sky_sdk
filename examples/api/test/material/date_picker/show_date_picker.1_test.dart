// Copyright 2014 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

import 'package:flutter/material.dart';
import 'package:flutter_api_samples/material/date_picker/show_date_picker.1.dart' as example;
import 'package:flutter_test/flutter_test.dart';

void main() {
  testWidgets('Can show date picker', (WidgetTester tester) async {
    const String datePickerTitle = 'Select date';
    const String initialDate = 'Sun, Jul 25';

    await tester.pumpWidget(const example.DatePickerApp());

    // The date picker is not shown initially.
    expect(find.text(datePickerTitle), findsNothing);
    expect(find.text(initialDate), findsNothing);

    expect(find.text('No date selected'), findsOneWidget);
    expect(find.byType(OutlinedButton), findsOneWidget);

    // Tap the button to show the date picker.
    await tester.tap(find.byType(OutlinedButton));
    await tester.pumpAndSettle();

    // The initial date is shown.
    expect(find.text(datePickerTitle), findsOneWidget);
    expect(find.text(initialDate), findsOneWidget);

    // Tap another date to select it.
    await tester.tap(find.text('30'));
    await tester.pumpAndSettle();

    // The selected date is shown.
    expect(find.text(datePickerTitle), findsOneWidget);
    expect(find.text('Fri, Jul 30'), findsOneWidget);

    // Tap OK to confirm the selection and close the date picker.
    await tester.tap(find.text('OK'));
    await tester.pumpAndSettle();

    // The date picker is closed and the selected date is shown.
    expect(find.text(datePickerTitle), findsNothing);
    expect(find.text('30/7/2021'), findsOneWidget);
  });
}
