// Copyright 2014 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

import 'package:flutter/widgets.dart';
import 'package:flutter_test/flutter_test.dart';

void main() {
  testWidgets('relayout boundary change does not trigger relayout', (WidgetTester tester) async {
    final RenderLayoutCount renderLayoutCount = RenderLayoutCount();
    final Widget layoutCounter = Center(
      key: GlobalKey(),
      child: WidgetToRenderBoxAdapter(renderBox: renderLayoutCount),
    );

    await tester.pumpWidget(
      Center(
        child: SizedBox(
          width: 100,
          height: 100,
          child: Center(
            child: SizedBox(width: 100, height: 100, child: Center(child: layoutCounter)),
          ),
        ),
      ),
    );

    expect(renderLayoutCount.layoutCount, 1);

    await tester.pumpWidget(Center(child: SizedBox(width: 100, height: 100, child: layoutCounter)));

    expect(renderLayoutCount.layoutCount, 1);
  });
}

// This class is needed because LayoutBuilder's RenderObject does not always
// call the builder method in its PerformLayout method.
class RenderLayoutCount extends RenderBox {
  int layoutCount = 0;

  @override
  void performLayout() {
    layoutCount += 1;
    size = constraints.biggest;
  }
}
