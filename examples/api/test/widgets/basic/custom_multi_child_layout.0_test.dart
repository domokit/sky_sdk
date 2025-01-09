// Copyright 2014 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

import 'package:flutter/material.dart';
import 'package:flutter_api_samples/widgets/basic/custom_multi_child_layout.0.dart' as example;
import 'package:flutter_test/flutter_test.dart';

void main() {
  testWidgets('has four containers', (WidgetTester tester) async {
    await tester.pumpWidget(const example.CustomMultiChildLayoutApp());
    final Finder containerFinder = find.byType(Container);
    expect(containerFinder, findsNWidgets(4));
  });

  testWidgets('containers are the same size', (WidgetTester tester) async {
    await tester.pumpWidget(
      const MaterialApp(home: Scaffold(body: example.CustomMultiChildLayoutExample())),
    );
    final Finder containerFinder = find.byType(Container);
    const Size expectedSize = Size(100, 100);
    for (int i = 0; i < 4; i += 1) {
      expect(tester.getSize(containerFinder.at(i)), equals(expectedSize));
    }
    expect(containerFinder, findsNWidgets(4));
  });

  testWidgets('containers are offset', (WidgetTester tester) async {
    await tester.pumpWidget(
      const MaterialApp(home: Scaffold(body: example.CustomMultiChildLayoutExample())),
    );
    final Finder containerFinder = find.byType(Container);
    Rect previousRect = tester.getRect(containerFinder.first);
    for (int i = 1; i < 4; i += 1) {
      expect(
        tester.getRect(containerFinder.at(i)),
        equals(previousRect.shift(const Offset(100, 70))),
        reason: 'Rect $i not correct size',
      );
      previousRect = tester.getRect(containerFinder.at(i));
    }
    expect(containerFinder, findsNWidgets(4));
  });
}
