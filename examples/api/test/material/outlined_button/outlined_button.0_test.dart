// Copyright 2014 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

import 'package:flutter/material.dart';
import 'package:flutter_api_samples/material/outlined_button/outlined_button.0.dart' as example;
import 'package:flutter_test/flutter_test.dart';

void main() {
  testWidgets('OutlinedButton Smoke Test', (WidgetTester tester) async {
    await tester.pumpWidget(const example.OutlinedButtonExampleApp());

    expect(find.widgetWithText(AppBar, 'OutlinedButton Sample'), findsOneWidget);
    final Finder outlinedButton = find.widgetWithText(OutlinedButton, 'Click Me');
    expect(outlinedButton, findsOneWidget);
    final OutlinedButton outlinedButtonWidget = tester.widget<OutlinedButton>(outlinedButton);
    expect(outlinedButtonWidget.onPressed.runtimeType, VoidCallback);
  });
}
