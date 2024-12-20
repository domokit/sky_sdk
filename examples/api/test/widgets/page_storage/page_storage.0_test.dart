// Copyright 2014 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

import 'package:flutter/material.dart';
import 'package:flutter_api_samples/widgets/page_storage/page_storage.0.dart' as example;
import 'package:flutter_test/flutter_test.dart';

void main() {
  testWidgets('Can choose to stay on page', (WidgetTester tester) async {
    await tester.pumpWidget(const example.PageStorageExampleApp());

    expect(find.widgetWithText(AppBar, 'Persistence Example'), findsOne);

    expect(find.text('0'), findsOne);

    await tester.scrollUntilVisible(find.text('10'), 100);

    expect(find.text('0'), findsNothing);

    await tester.tap(find.byIcon(Icons.settings));
    await tester.pumpAndSettle();

    expect(find.text('0'), findsOne);
    await tester.scrollUntilVisible(find.text('20'), 100);

    await tester.tap(find.byIcon(Icons.home));
    await tester.pumpAndSettle();

    expect(find.text('10'), findsOne);

    await tester.tap(find.byIcon(Icons.settings));
    await tester.pumpAndSettle();

    expect(find.text('20'), findsOne);
  });
}
