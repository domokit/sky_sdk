// Copyright 2014 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

import 'package:flutter/material.dart';
import 'package:flutter_api_samples/widgets/tap_region/text_field_tap_region.0.dart' as example;
import 'package:flutter_test/flutter_test.dart';

void main() {
  testWidgets('shows a text field with a zero count, and the spinner buttons', (
    WidgetTester tester,
  ) async {
    await tester.pumpWidget(const example.TapRegionApp());

    expect(find.byType(TextField), findsOneWidget);
    expect(getFieldValue(tester).text, equals('0'));
    expect(find.byIcon(Icons.add), findsOneWidget);
    expect(find.byIcon(Icons.remove), findsOneWidget);
  });

  testWidgets('tapping increment/decrement works', (WidgetTester tester) async {
    await tester.pumpWidget(const example.TapRegionApp());
    await tester.pump();

    expect(getFieldValue(tester).text, equals('0'));
    expect(getFieldValue(tester).selection, equals(const TextSelection.collapsed(offset: 1)));

    await tester.tap(find.byIcon(Icons.add));
    await tester.pumpAndSettle();

    expect(getFieldValue(tester).text, equals('1'));
    expect(
      getFieldValue(tester).selection,
      equals(const TextSelection(baseOffset: 0, extentOffset: 1)),
    );

    await tester.tap(find.byIcon(Icons.remove));
    await tester.pumpAndSettle();
    await tester.tap(find.byIcon(Icons.remove));
    await tester.pumpAndSettle();

    expect(getFieldValue(tester).text, equals('-1'));
    expect(
      getFieldValue(tester).selection,
      equals(const TextSelection(baseOffset: 0, extentOffset: 2)),
    );
  });

  testWidgets('entering text and then incrementing/decrementing works', (
    WidgetTester tester,
  ) async {
    await tester.pumpWidget(const example.TapRegionApp());
    await tester.pump();

    await tester.tap(find.byIcon(Icons.add));
    await tester.pumpAndSettle();

    expect(getFieldValue(tester).text, equals('1'));
    expect(
      getFieldValue(tester).selection,
      equals(const TextSelection(baseOffset: 0, extentOffset: 1)),
    );

    await tester.enterText(find.byType(TextField), '123');
    await tester.pumpAndSettle();
    expect(getFieldValue(tester).text, equals('123'));
    expect(getFieldValue(tester).selection, equals(const TextSelection.collapsed(offset: 3)));

    await tester.tap(find.byIcon(Icons.remove));
    await tester.pumpAndSettle();
    await tester.tap(find.byIcon(Icons.remove));
    await tester.pumpAndSettle();

    expect(getFieldValue(tester).text, equals('121'));
    expect(
      getFieldValue(tester).selection,
      equals(const TextSelection(baseOffset: 0, extentOffset: 3)),
    );
    final FocusNode textFieldFocusNode = Focus.of(
      tester.element(
        find.byWidgetPredicate((Widget widget) {
          return widget.runtimeType.toString() == '_Editable';
        }),
      ),
    );
    expect(textFieldFocusNode.hasPrimaryFocus, isTrue);
  });
}

TextEditingValue getFieldValue(WidgetTester tester) {
  return (tester.widget(find.byType(TextField)) as TextField).controller!.value;
}
