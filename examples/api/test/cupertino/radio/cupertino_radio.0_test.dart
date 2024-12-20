// Copyright 2014 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

import 'package:flutter/cupertino.dart';
import 'package:flutter_api_samples/cupertino/radio/cupertino_radio.0.dart' as example;
import 'package:flutter_test/flutter_test.dart';

void main() {
  testWidgets('Has 2 CupertinoRadio widgets', (WidgetTester tester) async {
    await tester.pumpWidget(const example.CupertinoRadioApp());

    expect(find.byType(CupertinoRadio<example.SingingCharacter>), findsNWidgets(2));

    CupertinoRadio<example.SingingCharacter> radio = tester.widget(
      find.byType(CupertinoRadio<example.SingingCharacter>).first,
    );
    expect(radio.groupValue, example.SingingCharacter.lafayette);

    radio = tester.widget(find.byType(CupertinoRadio<example.SingingCharacter>).last);
    expect(radio.groupValue, example.SingingCharacter.lafayette);

    await tester.tap(find.byType(CupertinoRadio<example.SingingCharacter>).last);
    await tester.pumpAndSettle();

    radio = tester.widget(find.byType(CupertinoRadio<example.SingingCharacter>).last);
    expect(radio.groupValue, example.SingingCharacter.jefferson);

    radio = tester.widget(find.byType(CupertinoRadio<example.SingingCharacter>).first);
    expect(radio.groupValue, example.SingingCharacter.jefferson);
  });
}
