// Copyright 2014 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

import 'package:flutter/material.dart';
import 'package:flutter_api_samples/material/refresh_indicator/refresh_indicator.2.dart' as example;
import 'package:flutter_test/flutter_test.dart';

void main() {
  testWidgets(
    'Pulling from scroll view triggers a refresh indicator which shows a CircularProgressIndicator',
    (WidgetTester tester) async {
      await tester.pumpWidget(const example.RefreshIndicatorExampleApp());

      // Pull the first item.
      await tester.fling(find.text('Pull down here').first, const Offset(0.0, 300.0), 1000.0);
      await tester.pump();
      await tester.pump(const Duration(seconds: 1));
      await tester.pump(const Duration(seconds: 1));

      expect(find.byType(RefreshProgressIndicator), findsNothing);
      expect(find.bySemanticsLabel('Circular progress indicator'), findsOneWidget);

      await tester.pumpAndSettle(); // Advance pending time.
    },
  );
}
