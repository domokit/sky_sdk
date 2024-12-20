// Copyright 2014 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

import 'package:flutter/material.dart';
import 'package:flutter_api_samples/widgets/basic/expanded.1.dart' as example;
import 'package:flutter_test/flutter_test.dart';

void main() {
  testWidgets('Expanded widgets in a Row', (WidgetTester tester) async {
    const double rowWidth = 800.0;
    const double rowHeight = 100.0;
    const double containerOneWidth = (rowWidth - 50) * 2 / 3;
    const double containerTwoWidth = 50;
    const double containerThreeWidth = (rowWidth - 50) * 1 / 3;

    await tester.pumpWidget(const example.ExpandedApp());

    final Size row = tester.getSize(find.byType(Row));
    expect(row, const Size(rowWidth, rowHeight));

    // This container is wrapped in an Expanded widget, so it should take up
    // two thirds of the remaining space in the Row.
    final Size containerOne = tester.getSize(find.byType(Container).at(0));
    expect(containerOne, const Size(containerOneWidth, rowHeight));

    final Size containerTwo = tester.getSize(find.byType(Container).at(1));
    expect(containerTwo, const Size(containerTwoWidth, rowHeight));

    // This container is wrapped in an Expanded widget, so it should take up
    // one third of the remaining space in the Row.
    final Size containerThree = tester.getSize(find.byType(Container).at(2));
    expect(containerThree, const Size(containerThreeWidth, rowHeight));
  });
}
