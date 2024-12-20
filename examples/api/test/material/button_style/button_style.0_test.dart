// Copyright 2014 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

import 'package:flutter/material.dart';
import 'package:flutter_api_samples/material/button_style/button_style.0.dart' as example;
import 'package:flutter_test/flutter_test.dart';

void main() {
  testWidgets(
    'Shows ElevatedButtons, FilledButtons, OutlinedButtons and TextButtons in enabled and disabled states',
    (WidgetTester tester) async {
      await tester.pumpWidget(const example.ButtonApp());

      expect(
        find.byWidgetPredicate((Widget widget) {
          return widget is ElevatedButton && widget.onPressed == null;
        }),
        findsOne,
      );

      expect(
        find.byWidgetPredicate((Widget widget) {
          return widget is ElevatedButton && widget.onPressed != null;
        }),
        findsOne,
      );

      // One OutlinedButton with onPressed null.
      expect(
        find.byWidgetPredicate((Widget widget) {
          return widget is OutlinedButton && widget.onPressed == null;
        }),
        findsOne,
      );

      // One OutlinedButton with onPressed not null.
      expect(
        find.byWidgetPredicate((Widget widget) {
          return widget is OutlinedButton && widget.onPressed != null;
        }),
        findsOne,
      );

      expect(
        find.byWidgetPredicate((Widget widget) {
          return widget is TextButton && widget.onPressed == null;
        }),
        findsOne,
      );

      expect(
        find.byWidgetPredicate((Widget widget) {
          return widget is TextButton && widget.onPressed != null;
        }),
        findsOne,
      );

      expect(
        find.byWidgetPredicate((Widget widget) {
          return widget is FilledButton && widget.onPressed != null;
        }),
        findsNWidgets(2),
      );

      expect(
        find.byWidgetPredicate((Widget widget) {
          return widget is FilledButton && widget.onPressed == null;
        }),
        findsNWidgets(2),
      );
    },
  );
}
