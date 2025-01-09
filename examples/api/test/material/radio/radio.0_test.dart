// Copyright 2014 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

import 'package:flutter/material.dart';
import 'package:flutter_api_samples/material/radio/radio.0.dart' as example;
import 'package:flutter_test/flutter_test.dart';

void main() {
  testWidgets('Radio Smoke Test', (WidgetTester tester) async {
    await tester.pumpWidget(const example.RadioExampleApp());

    expect(find.widgetWithText(AppBar, 'Radio Sample'), findsOneWidget);
    final Finder listTile1 = find.widgetWithText(ListTile, 'Lafayette');
    expect(listTile1, findsOneWidget);
    final Finder listTile2 = find.widgetWithText(ListTile, 'Thomas Jefferson');
    expect(listTile2, findsOneWidget);

    final Finder radioButton1 = find.byType(Radio<example.SingingCharacter>).first;
    final Finder radioButton2 = find.byType(Radio<example.SingingCharacter>).last;

    await tester.tap(radioButton1);
    await tester.pumpAndSettle();
    expect(
      tester.widget<Radio<example.SingingCharacter>>(radioButton1).groupValue,
      tester.widget<Radio<example.SingingCharacter>>(radioButton1).value,
    );
    expect(
      tester.widget<Radio<example.SingingCharacter>>(radioButton2).groupValue,
      isNot(tester.widget<Radio<example.SingingCharacter>>(radioButton2).value),
    );
    await tester.tap(radioButton2);
    await tester.pumpAndSettle();
    expect(
      tester.widget<Radio<example.SingingCharacter>>(radioButton1).groupValue,
      isNot(tester.widget<Radio<example.SingingCharacter>>(radioButton1).value),
    );
    expect(
      tester.widget<Radio<example.SingingCharacter>>(radioButton2).groupValue,
      tester.widget<Radio<example.SingingCharacter>>(radioButton2).value,
    );
  });
}
