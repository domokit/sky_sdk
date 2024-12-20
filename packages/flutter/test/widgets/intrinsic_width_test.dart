// Copyright 2014 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

import 'package:flutter/widgets.dart';
import 'package:flutter_test/flutter_test.dart';

void main() {
  testWidgets('Intrinsic stepWidth, stepHeight', (WidgetTester tester) async {
    // Regression test for https://github.com/flutter/flutter/issues/25224
    Widget buildFrame(double? stepWidth, double? stepHeight) {
      return Center(
        child: IntrinsicWidth(
          stepWidth: stepWidth,
          stepHeight: stepHeight,
          child: const SizedBox(width: 100.0, height: 50.0),
        ),
      );
    }

    await tester.pumpWidget(buildFrame(null, null));
    expect(tester.getSize(find.byType(IntrinsicWidth)), const Size(100.0, 50.0));

    await tester.pumpWidget(buildFrame(0.0, 0.0));
    expect(tester.getSize(find.byType(IntrinsicWidth)), const Size(100.0, 50.0));

    expect(() {
      buildFrame(-1.0, 0.0);
    }, throwsAssertionError);
    expect(() {
      buildFrame(0.0, -1.0);
    }, throwsAssertionError);
  });
}
