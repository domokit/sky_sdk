// Copyright 2014 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

import 'package:flutter/cupertino.dart';
import 'package:flutter_api_samples/cupertino/radio/cupertino_radio.toggleable.0.dart' as example;
import 'package:flutter_test/flutter_test.dart';

void main() {
  testWidgets('Has 2 CupertinoRadio widgets that can be toggled off', (WidgetTester tester) async {
    await tester.pumpWidget(const example.CupertinoRadioApp());

    expect(find.byType(CupertinoRadio<example.SingingCharacter>), findsNWidgets(2));

    CupertinoRadio<example.SingingCharacter> radio = tester.widget(
      find.byType(CupertinoRadio<example.SingingCharacter>).first,
    );
    expect(radio.groupValue, example.SingingCharacter.mulligan);

    radio = tester.widget(find.byType(CupertinoRadio<example.SingingCharacter>).last);
    expect(radio.groupValue, example.SingingCharacter.mulligan);

    await tester.tap(find.byType(CupertinoRadio<example.SingingCharacter>).last);
    await tester.pumpAndSettle();

    radio = tester.widget(find.byType(CupertinoRadio<example.SingingCharacter>).last);
    expect(radio.groupValue, example.SingingCharacter.hamilton);

    radio = tester.widget(find.byType(CupertinoRadio<example.SingingCharacter>).first);
    expect(radio.groupValue, example.SingingCharacter.hamilton);

    await tester.tap(find.byType(CupertinoRadio<example.SingingCharacter>).last);
    await tester.pumpAndSettle();

    radio = tester.widget(find.byType(CupertinoRadio<example.SingingCharacter>).last);
    expect(radio.groupValue, null);
  });
}
