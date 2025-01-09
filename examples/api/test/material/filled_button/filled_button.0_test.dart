// Copyright 2014 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

import 'package:flutter/material.dart';
import 'package:flutter_api_samples/material/filled_button/filled_button.0.dart' as example;
import 'package:flutter_test/flutter_test.dart';

void main() {
  testWidgets('FilledButton Smoke Test', (WidgetTester tester) async {
    await tester.pumpWidget(const example.FilledButtonApp());

    expect(find.widgetWithText(AppBar, 'FilledButton Sample'), findsOneWidget);
    final Finder disabledButton = find.widgetWithText(FilledButton, 'Disabled');
    expect(disabledButton, findsNWidgets(2));
    expect(tester.widget<FilledButton>(disabledButton.first).onPressed.runtimeType, Null);
    expect(tester.widget<FilledButton>(disabledButton.last).onPressed.runtimeType, Null);
    final Finder enabledButton = find.widgetWithText(FilledButton, 'Enabled');
    expect(enabledButton, findsNWidgets(2));
    expect(tester.widget<FilledButton>(enabledButton.first).onPressed.runtimeType, VoidCallback);
    expect(tester.widget<FilledButton>(enabledButton.last).onPressed.runtimeType, VoidCallback);
  });
}
