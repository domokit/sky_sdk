// Copyright 2014 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

import 'package:flutter/cupertino.dart';
import 'package:flutter_api_samples/cupertino/picker/cupertino_picker.0.dart' as example;
import 'package:flutter_test/flutter_test.dart';

const Offset _kRowOffset = Offset(0.0, -50.0);

void main() {
  testWidgets('Change selected fruit using CupertinoPicker', (WidgetTester tester) async {
    await tester.pumpWidget(const example.CupertinoPickerApp());

    // Open the Cupertino picker.
    await tester.tap(find.widgetWithText(CupertinoButton, 'Apple'));
    await tester.pumpAndSettle();

    // Test the initial item.
    CupertinoPicker picker = tester.widget<CupertinoPicker>(find.byType(CupertinoPicker));
    expect(picker.scrollController!.initialItem, 0);

    // Drag the wheel to change fruit selection.
    await tester.drag(
      find.text('Mango'),
      _kRowOffset,
      touchSlopY: 0,
      warnIfMissed: false,
    ); // see top of file
    await tester.pump();
    await tester.pump(const Duration(milliseconds: 500));

    // Close the Cupertino picker.
    await tester.tapAt(const Offset(1.0, 1.0));
    await tester.pumpAndSettle();

    expect(find.widgetWithText(CupertinoButton, 'Banana'), findsOneWidget);

    // Test if the initial item has updated.
    await tester.tap(find.widgetWithText(CupertinoButton, 'Banana'));
    await tester.pumpAndSettle();

    picker = tester.widget<CupertinoPicker>(find.byType(CupertinoPicker));
    expect(picker.scrollController!.initialItem, 2);
  });
}
