// Copyright 2014 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

import 'package:flutter/material.dart';
import 'package:flutter_api_samples/painting/gradient/linear_gradient.0.dart' as example;
import 'package:flutter_test/flutter_test.dart';

void main() {
  testWidgets('finds a gradient', (WidgetTester tester) async {
    await tester.pumpWidget(const MaterialApp(home: example.MoodyGradient()));

    expect(find.byType(example.MoodyGradient), findsOneWidget);
  });

  testWidgets('gradient matches golden', (WidgetTester tester) async {
    await tester.pumpWidget(
      const MaterialApp(
        home: SizedBox(
          width: 800,
          height: 600,
          child: RepaintBoundary(child: example.MoodyGradient()),
        ),
      ),
    );
    await expectLater(
      find.byType(example.MoodyGradient),
      matchesGoldenFile('linear_gradient.0_test.png'),
    );
  });
}
