// Copyright 2014 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

import 'package:flutter/material.dart';
import 'package:flutter_api_samples/material/text_field/text_field.2.dart' as example;
import 'package:flutter_test/flutter_test.dart';

void main() {
  testWidgets('Creates styled text fields', (WidgetTester tester) async {
    await tester.pumpWidget(const example.TextFieldExamplesApp());

    expect(find.text('TextField Examples'), findsOneWidget);
    expect(find.byType(TextField), findsNWidgets(2));
    expect(find.byType(example.FilledTextFieldExample), findsOneWidget);
    expect(find.byType(example.OutlinedTextFieldExample), findsOneWidget);

    final TextField filled = tester.widget<TextField>(
      find.descendant(
        of: find.byType(example.FilledTextFieldExample),
        matching: find.byType(TextField),
      ),
    );
    expect(
      filled.decoration!.prefixIcon,
      isA<Icon>().having((Icon icon) => icon.icon, 'icon', Icons.search),
    );
    expect(
      filled.decoration!.suffixIcon,
      isA<Icon>().having((Icon icon) => icon.icon, 'icon', Icons.clear),
    );
    expect(filled.decoration!.labelText, 'Filled');
    expect(filled.decoration!.hintText, 'hint text');
    expect(filled.decoration!.helperText, 'supporting text');
    expect(filled.decoration!.filled, true);

    final TextField outlined = tester.widget<TextField>(
      find.descendant(
        of: find.byType(example.OutlinedTextFieldExample),
        matching: find.byType(TextField),
      ),
    );
    expect(
      outlined.decoration!.prefixIcon,
      isA<Icon>().having((Icon icon) => icon.icon, 'icon', Icons.search),
    );
    expect(
      outlined.decoration!.suffixIcon,
      isA<Icon>().having((Icon icon) => icon.icon, 'icon', Icons.clear),
    );
    expect(outlined.decoration!.labelText, 'Outlined');
    expect(outlined.decoration!.hintText, 'hint text');
    expect(outlined.decoration!.helperText, 'supporting text');
    expect(outlined.decoration!.border, isA<OutlineInputBorder>());
  });
}
