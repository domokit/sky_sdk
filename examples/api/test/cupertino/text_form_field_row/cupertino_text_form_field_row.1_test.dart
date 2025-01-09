// Copyright 2014 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

import 'package:flutter/cupertino.dart';
import 'package:flutter_api_samples/cupertino/text_form_field_row/cupertino_text_form_field_row.1.dart'
    as example;
import 'package:flutter_test/flutter_test.dart';

void main() {
  testWidgets('Can enter text in CupertinoTextFormFieldRow', (WidgetTester tester) async {
    await tester.pumpWidget(const example.FormSectionApp());

    expect(find.byType(CupertinoFormSection), findsOneWidget);
    expect(find.byType(CupertinoTextFormFieldRow), findsNWidgets(5));

    expect(find.widgetWithText(CupertinoTextFormFieldRow, 'abcd'), findsNothing);
    await tester.enterText(find.byType(CupertinoTextFormFieldRow).first, 'abcd');
    await tester.pump();
    expect(find.widgetWithText(CupertinoTextFormFieldRow, 'abcd'), findsOneWidget);
  });
}
