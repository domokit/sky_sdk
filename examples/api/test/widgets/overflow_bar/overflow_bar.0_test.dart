// Copyright 2014 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

import 'package:flutter/material.dart';
import 'package:flutter_api_samples/widgets/overflow_bar/overflow_bar.0.dart' as example;
import 'package:flutter_test/flutter_test.dart';

void main() {
  testWidgets('OverflowBar displays buttons', (WidgetTester tester) async {
    await tester.pumpWidget(const example.OverflowBarExampleApp());

    // Creates a finder that matches widgets of the given
    // `widgetType`, ensuring that the given widgets exist
    // inside of an OverflowBar.
    Finder buttonsFinder(Type widgetType) {
      return find.descendant(of: find.byType(OverflowBar), matching: find.byType(widgetType));
    }

    expect(buttonsFinder(TextButton), findsNWidgets(2));
    expect(buttonsFinder(OutlinedButton), findsOne);
  });
}
