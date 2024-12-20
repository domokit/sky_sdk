// Copyright 2014 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

import 'package:flutter/cupertino.dart';
import 'package:flutter_api_samples/cupertino/search_field/cupertino_search_field.0.dart'
    as example;
import 'package:flutter_test/flutter_test.dart';

void main() {
  testWidgets('CupertinoTextField has initial text', (WidgetTester tester) async {
    await tester.pumpWidget(const example.SearchTextFieldApp());

    expect(find.byType(CupertinoSearchTextField), findsOneWidget);
    expect(find.text('initial text'), findsOneWidget);

    await tester.tap(find.byIcon(CupertinoIcons.xmark_circle_fill));
    await tester.pump();
    expect(find.text('initial text'), findsNothing);

    await tester.enterText(find.byType(CupertinoSearchTextField), 'photos');
    await tester.pump();
    expect(find.text('photos'), findsOneWidget);
  });
}
