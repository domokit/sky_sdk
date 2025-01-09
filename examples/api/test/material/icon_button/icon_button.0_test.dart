// Copyright 2014 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

import 'package:flutter/material.dart';
import 'package:flutter_api_samples/material/icon_button/icon_button.0.dart' as example;
import 'package:flutter_test/flutter_test.dart';

void main() {
  testWidgets('IconButton increments volume when tapped', (WidgetTester tester) async {
    await tester.pumpWidget(const example.IconButtonExampleApp());

    expect(find.byIcon(Icons.volume_up), findsOneWidget);
    expect(find.text('Volume : 0.0'), findsOneWidget);

    await tester.tap(find.byType(IconButton));
    await tester.pumpAndSettle();

    expect(find.text('Volume : 10.0'), findsOneWidget);
  });

  testWidgets('IconButton shows tooltip when long pressed', (WidgetTester tester) async {
    await tester.pumpWidget(const example.IconButtonExampleApp());

    expect(find.text('Increase volume by 10'), findsNothing);
    await tester.longPress(find.byType(IconButton));
    await tester.pumpAndSettle();

    expect(find.text('Increase volume by 10'), findsOneWidget);
  });
}
