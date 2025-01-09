// Copyright 2014 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

import 'package:flutter_api_samples/material/dropdown/dropdown_button.style.0.dart' as example;
import 'package:flutter_test/flutter_test.dart';

void main() {
  testWidgets('Select an item from DropdownButton', (WidgetTester tester) async {
    await tester.pumpWidget(const example.DropdownButtonApp());

    expect(find.text('One'), findsOneWidget);
    expect(find.text('One', skipOffstage: false), findsNWidgets(4));

    await tester.tap(find.text('One').first);
    await tester.pumpAndSettle();
    expect(find.text('Two'), findsOneWidget);
    await tester.tap(find.text('Two'));
    await tester.pumpAndSettle();
    expect(find.text('Two'), findsOneWidget);
    expect(find.text('Two', skipOffstage: false), findsNWidgets(4));
  });
}
