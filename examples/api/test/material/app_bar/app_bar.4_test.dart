// Copyright 2014 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

import 'package:flutter/material.dart';
import 'package:flutter_api_samples/material/app_bar/app_bar.4.dart' as example;
import 'package:flutter_test/flutter_test.dart';

void main() {
  testWidgets('AppBar uses custom shape', (WidgetTester tester) async {
    await tester.pumpWidget(const example.AppBarExampleApp());

    Material getMaterial() => tester.widget<Material>(
      find.descendant(of: find.byType(AppBar), matching: find.byType(Material)),
    );
    expect(getMaterial().shape, const example.CustomAppBarShape());
  });

  testWidgets('AppBar bottom contains TextField', (WidgetTester tester) async {
    await tester.pumpWidget(const example.AppBarExampleApp());

    final Finder textFieldFinder = find.descendant(
      of: find.byType(AppBar),
      matching: find.byType(TextField),
    );

    expect(textFieldFinder, findsOneWidget);
  });
}
