// Copyright 2014 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

import 'package:flutter/material.dart';
import 'package:flutter_api_samples/material/refresh_indicator/refresh_indicator.0.dart' as example;
import 'package:flutter_test/flutter_test.dart';

void main() {
  testWidgets('Trigger RefreshIndicator - Pull from top', (WidgetTester tester) async {
    await tester.pumpWidget(const example.RefreshIndicatorExampleApp());

    await tester.fling(find.text('Item 1'), const Offset(0.0, 300.0), 1000.0);
    await tester.pump();
    await tester.pump(const Duration(seconds: 1));
    await tester.pump(const Duration(seconds: 1));
    expect(tester.getCenter(find.byType(RefreshProgressIndicator)).dy, lessThan(300.0));
    await tester.pumpAndSettle(); // Advance pending time
  });

  testWidgets('Trigger RefreshIndicator - Button', (WidgetTester tester) async {
    await tester.pumpWidget(const example.RefreshIndicatorExampleApp());

    await tester.tap(find.byType(FloatingActionButton));
    await tester.pump();
    await tester.pump(const Duration(seconds: 1));
    await tester.pump(const Duration(seconds: 1));
    expect(tester.getCenter(find.byType(RefreshProgressIndicator)).dy, lessThan(300.0));
    await tester.pumpAndSettle(); // Advance pending time
  });
}
