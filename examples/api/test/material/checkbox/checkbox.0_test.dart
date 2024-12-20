// Copyright 2014 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

import 'package:flutter/material.dart';
import 'package:flutter_api_samples/material/checkbox/checkbox.0.dart' as example;
import 'package:flutter_test/flutter_test.dart';

void main() {
  testWidgets('Checkbox can be checked', (WidgetTester tester) async {
    await tester.pumpWidget(const example.CheckboxExampleApp());

    Checkbox checkbox = tester.widget(find.byType(Checkbox));
    expect(checkbox.value, isFalse);

    await tester.tap(find.byType(Checkbox));
    await tester.pump();

    checkbox = tester.widget(find.byType(Checkbox));
    expect(checkbox.value, isTrue);

    await tester.tap(find.byType(Checkbox));
    await tester.pump();

    checkbox = tester.widget(find.byType(Checkbox));
    expect(checkbox.value, isFalse);
  });

  testWidgets('Checkbox color can be changed', (WidgetTester tester) async {
    await tester.pumpWidget(const example.CheckboxExampleApp());
    final Checkbox checkbox = tester.widget(find.byType(Checkbox));

    expect(checkbox.checkColor, Colors.white);
    expect(checkbox.fillColor!.resolve(<MaterialState>{}), Colors.red);
    expect(checkbox.fillColor!.resolve(<MaterialState>{MaterialState.pressed}), Colors.blue);
    expect(checkbox.fillColor!.resolve(<MaterialState>{MaterialState.hovered}), Colors.blue);
    expect(checkbox.fillColor!.resolve(<MaterialState>{MaterialState.focused}), Colors.blue);
  });
}
