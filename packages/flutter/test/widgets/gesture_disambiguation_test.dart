// Copyright 2014 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

import 'package:flutter/widgets.dart';
import 'package:flutter_test/flutter_test.dart';

void main() {
  testWidgets('onTap detection with canceled pointer and a drag listener', (
    WidgetTester tester,
  ) async {
    int detector1TapCount = 0;
    int detector2TapCount = 0;

    final Widget widget = GestureDetector(
      child: Column(
        children: <Widget>[
          GestureDetector(
            onTap: () {
              detector1TapCount += 1;
            },
            behavior: HitTestBehavior.opaque,
            child: const SizedBox(width: 200.0, height: 200.0),
          ),
          GestureDetector(
            onTap: () {
              detector2TapCount += 1;
            },
            behavior: HitTestBehavior.opaque,
            child: const SizedBox(width: 200.0, height: 200.0),
          ),
        ],
      ),
    );

    await tester.pumpWidget(widget);

    // The following pointer event sequence was causing the issue described
    // in https://github.com/flutter/flutter/issues/12470 by triggering 2 tap
    // events on the second detector.
    final TestGesture gesture1 = await tester.startGesture(const Offset(400.0, 10.0));
    final TestGesture gesture2 = await tester.startGesture(const Offset(400.0, 210.0));
    await gesture1.up();
    await gesture2.cancel();
    final TestGesture gesture3 = await tester.startGesture(const Offset(400.0, 250.0));
    await gesture3.up();

    expect(detector1TapCount, 1);
    expect(detector2TapCount, 1);
  });
}
