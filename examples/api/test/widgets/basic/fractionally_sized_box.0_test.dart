// Copyright 2014 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

import 'package:flutter/material.dart';
import 'package:flutter_api_samples/widgets/basic/fractionally_sized_box.0.dart' as example;
import 'package:flutter_test/flutter_test.dart';

void main() {
  testWidgets('FractionallySizedBox sizes DecoratedBox', (WidgetTester tester) async {
    const double appBarHeight = 56.0;
    const double widthFactor = 0.5;
    const double heightFactor = 0.5;

    await tester.pumpWidget(const example.FractionallySizedBoxApp());

    final FractionallySizedBox fractionallySizedBox = tester.widget(
      find.byType(FractionallySizedBox),
    );
    expect(fractionallySizedBox.widthFactor, widthFactor);
    expect(fractionallySizedBox.heightFactor, heightFactor);

    final Size boxSize = tester.getSize(find.byType(DecoratedBox));
    expect(boxSize.width, 800 * widthFactor);
    expect(boxSize.height, (600 - appBarHeight) * heightFactor);
  });
}
