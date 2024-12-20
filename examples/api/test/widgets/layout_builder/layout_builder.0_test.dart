// Copyright 2014 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

import 'package:flutter/widgets.dart';
import 'package:flutter_api_samples/widgets/layout_builder/layout_builder.0.dart' as example;
import 'package:flutter_test/flutter_test.dart';

void main() {
  testWidgets('has two containers when wide', (WidgetTester tester) async {
    await tester.pumpWidget(const example.LayoutBuilderExampleApp());

    final Finder containerFinder = find.byType(Container);
    expect(containerFinder, findsNWidgets(2));
  });
  testWidgets('has one container when narrow', (WidgetTester tester) async {
    await tester.pumpWidget(
      ConstrainedBox(
        constraints: const BoxConstraints(maxWidth: 400),
        child: const example.LayoutBuilderExampleApp(),
      ),
    );

    final Finder containerFinder = find.byType(Container);
    expect(containerFinder, findsNWidgets(2));
  });
}
