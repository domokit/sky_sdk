// Copyright 2014 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

import 'package:flutter/material.dart';
import 'package:flutter_api_samples/widgets/scroll_position/scroll_metrics_notification.0.dart'
    as example;
import 'package:flutter_test/flutter_test.dart';

void main() {
  testWidgets('A snackbar is shown when the scroll metrics change', (WidgetTester tester) async {
    await tester.pumpWidget(const example.ScrollMetricsDemo());

    expect(find.widgetWithText(AppBar, 'ScrollMetrics Demo'), findsOne);
    expect(find.byType(FlutterLogo), findsOne);

    await tester.tap(find.widgetWithIcon(FloatingActionButton, Icons.add));
    await tester.pumpAndSettle();

    expect(find.widgetWithText(SnackBar, 'Scroll metrics changed!'), findsOne);
  });
}
