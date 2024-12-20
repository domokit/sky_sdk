// Copyright 2014 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

import 'package:flutter/cupertino.dart';
import 'package:flutter_api_samples/cupertino/search_field/cupertino_search_field.1.dart'
    as example;
import 'package:flutter_test/flutter_test.dart';

void main() {
  testWidgets('Value changed callback updates entered text', (WidgetTester tester) async {
    await tester.pumpWidget(const example.SearchTextFieldApp());

    expect(find.byType(CupertinoSearchTextField), findsOneWidget);

    await tester.enterText(find.byType(CupertinoSearchTextField), 'photos');
    await tester.pump();
    expect(find.text('The text has changed to: photos'), findsOneWidget);

    await tester.enterText(find.byType(CupertinoSearchTextField), 'photos from vacation');
    await tester.showKeyboard(find.byType(CupertinoTextField));
    await tester.testTextInput.receiveAction(TextInputAction.done);
    await tester.pump();
    expect(find.text('Submitted text: photos from vacation'), findsOneWidget);
  });
}
