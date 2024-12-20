// Copyright 2014 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

import 'package:flutter/rendering.dart';
import 'package:flutter_api_samples/widgets/scroll_view/grid_view.0.dart';
import 'package:flutter_test/flutter_test.dart';

void main() {
  testWidgets('$CustomGridLayout', (WidgetTester tester) async {
    const CustomGridLayout layout = CustomGridLayout(
      crossAxisCount: 2,
      fullRowPeriod: 3,
      dimension: 100,
    );
    final List<double> scrollOffsets = List<double>.generate(
      10,
      (int i) => layout.computeMaxScrollOffset(i),
    );
    expect(scrollOffsets, <double>[
      0.0,
      0.0,
      100.0,
      100.0,
      200.0,
      300.0,
      300.0,
      400.0,
      400.0,
      500.0,
    ]);
    final List<int> minOffsets = List<int>.generate(
      10,
      (int i) => layout.getMinChildIndexForScrollOffset(i * 80.0),
    );
    expect(minOffsets, <int>[0, 0, 2, 4, 5, 7, 7, 9, 10, 12]);
    final List<int> maxOffsets = List<int>.generate(
      10,
      (int i) => layout.getMaxChildIndexForScrollOffset(i * 80.0),
    );
    expect(maxOffsets, <double>[1, 1, 3, 4, 6, 8, 8, 9, 11, 13]);
    final List<SliverGridGeometry> offsets = List<SliverGridGeometry>.generate(
      20,
      (int i) => layout.getGeometryForChildIndex(i),
    );
    offsets.reduce((SliverGridGeometry a, SliverGridGeometry b) {
      if (a.scrollOffset == b.scrollOffset) {
        expect(a.crossAxisOffset, lessThan(b.crossAxisOffset));
      } else {
        expect(a.scrollOffset, lessThan(b.scrollOffset));
      }
      return b;
    });
  });
}
