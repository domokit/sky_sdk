// Copyright 2014 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

import 'package:flutter/material.dart';
import 'package:flutter_api_samples/rendering/sliver_grid/sliver_grid_delegate_with_fixed_cross_axis_count.0.dart'
    as example;
import 'package:flutter_test/flutter_test.dart';

void main() {
  testWidgets('Each tiles should have a width of 200.0 and a height of 100.0', (
    WidgetTester tester,
  ) async {
    await tester.pumpWidget(const example.SliverGridDelegateWithFixedCrossAxisCountExampleApp());

    for (int i = 0; i < 20; i++) {
      expect(find.text('$i'), findsOne);
      final Element element = tester.element(find.text('$i'));

      expect(element.size, const Size(200, 100));
    }
  });
}
