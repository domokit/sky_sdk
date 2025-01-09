// Copyright 2014 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

import 'package:flutter/material.dart';
import 'package:flutter_gallery/demo/material/buttons_demo.dart';
import 'package:flutter_test/flutter_test.dart';

void main() {
  testWidgets('Button locations are OK', (WidgetTester tester) async {
    // Regression test for https://github.com/flutter/flutter/pull/85351
    {
      await tester.pumpWidget(
        MaterialApp(theme: ThemeData(useMaterial3: false), home: const ButtonsDemo()),
      );
      expect(find.byType(ElevatedButton).evaluate().length, 2);
      final Offset topLeft1 = tester.getTopLeft(find.byType(ElevatedButton).first);
      final Offset topLeft2 = tester.getTopLeft(find.byType(ElevatedButton).last);
      expect(topLeft1.dx, 203);
      expect(topLeft2.dx, 453);
      expect(topLeft1.dy, topLeft2.dy);
    }

    {
      await tester.tap(find.text('TEXT'));
      await tester.pumpAndSettle();
      expect(find.byType(TextButton).evaluate().length, 2);
      final Offset topLeft1 = tester.getTopLeft(find.byType(TextButton).first);
      final Offset topLeft2 = tester.getTopLeft(find.byType(TextButton).last);
      expect(topLeft1.dx, 247);
      expect(topLeft2.dx, 425);
      expect(topLeft1.dy, topLeft2.dy);
    }

    {
      await tester.tap(find.text('OUTLINED'));
      await tester.pumpAndSettle();
      expect(find.byType(OutlinedButton).evaluate().length, 2);
      final Offset topLeft1 = tester.getTopLeft(find.byType(OutlinedButton).first);
      final Offset topLeft2 = tester.getTopLeft(find.byType(OutlinedButton).last);
      expect(topLeft1.dx, 203);
      expect(topLeft2.dx, 453);
      expect(topLeft1.dy, topLeft2.dy);
    }
  });
}
