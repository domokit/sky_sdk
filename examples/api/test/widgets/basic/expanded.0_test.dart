// Copyright 2014 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

import 'package:flutter/material.dart';
import 'package:flutter_api_samples/widgets/basic/expanded.0.dart' as example;
import 'package:flutter_test/flutter_test.dart';

void main() {
  testWidgets('Expanded widget in a Column', (WidgetTester tester) async {
    const double totalHeight = 600;
    const double appBarHeight = 56.0;
    const double columnWidth = 100.0;
    const double columnHeight = totalHeight - appBarHeight;
    const double containerOneHeight = 100;
    const double containerTwoHeight = columnHeight - 200;
    const double containerThreeHeight = 100;

    await tester.pumpWidget(const example.ExpandedApp());

    final Size column = tester.getSize(find.byType(Column));
    expect(column, const Size(columnWidth, columnHeight));

    final Size containerOne = tester.getSize(find.byType(Container).at(0));
    expect(containerOne, const Size(columnWidth, containerOneHeight));

    // This Container is wrapped in an Expanded widget, so it should take up
    // the remaining space in the Column.
    final Size containerTwo = tester.getSize(find.byType(Container).at(1));
    expect(containerTwo, const Size(columnWidth, containerTwoHeight));

    final Size containerThree = tester.getSize(find.byType(Container).at(2));
    expect(containerThree, const Size(columnWidth, containerThreeHeight));
  });
}
