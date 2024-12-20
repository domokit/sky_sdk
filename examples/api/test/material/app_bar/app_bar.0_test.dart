// Copyright 2014 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

import 'package:flutter/material.dart';
import 'package:flutter_api_samples/material/app_bar/app_bar.0.dart' as example;
import 'package:flutter_test/flutter_test.dart';

void main() {
  testWidgets('Appbar updates on navigation', (WidgetTester tester) async {
    await tester.pumpWidget(const example.AppBarApp());

    expect(find.widgetWithText(AppBar, 'AppBar Demo'), findsOneWidget);
    expect(find.text('This is the home page'), findsOneWidget);

    await tester.tap(find.byIcon(Icons.navigate_next));
    await tester.pumpAndSettle();

    expect(find.widgetWithText(AppBar, 'Next page'), findsOneWidget);
    expect(find.text('This is the next page'), findsOneWidget);
  });
}
