// Copyright 2014 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

import 'package:flutter/material.dart';
import 'package:flutter_api_samples/widgets/basic/overflowbox.0.dart' as example;
import 'package:flutter_test/flutter_test.dart';

void main() {
  testWidgets('OverflowBox allows child widget to overflow parent container', (
    WidgetTester tester,
  ) async {
    const Size containerSize = Size(100, 100);
    const Size maxSize = Size(200, 200);

    await tester.pumpWidget(const example.OverflowBoxApp());

    // The parent container has fixed width and height of 100 pixels.
    expect(tester.getSize(find.byType(Container).first), containerSize);

    final OverflowBox overflowBox = tester.widget(find.byType(OverflowBox));
    // The OverflowBox imposes its own constraints of maxWidth and maxHeight of
    // 200 on its child which allows the child to overflow the parent container.
    expect(overflowBox.maxWidth, maxSize.width);
    expect(overflowBox.maxHeight, maxSize.height);

    // The child widget overflows the parent container.
    expect(tester.getSize(find.byType(FlutterLogo)), greaterThan(containerSize));
    expect(tester.getSize(find.byType(FlutterLogo)), maxSize);
  });
}
