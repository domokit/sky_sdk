// Copyright 2014 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

import 'package:flutter_api_samples/material/dropdown/dropdown_button.0.dart' as example;
import 'package:flutter_test/flutter_test.dart';

void main() {
  testWidgets('Select an item from DropdownButton', (WidgetTester tester) async {
    await tester.pumpWidget(const example.DropdownButtonApp());

    expect(find.text('One'), findsOneWidget);

    await tester.tap(find.text('One'));
    await tester.pumpAndSettle();
    await tester.tap(find.text('Two').last);
    await tester.pumpAndSettle();
    expect(find.text('Two'), findsOneWidget);
  });
}
