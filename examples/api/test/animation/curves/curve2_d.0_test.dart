// Copyright 2014 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

import 'package:flutter/material.dart';
import 'package:flutter_api_samples/animation/curves/curve2_d.0.dart' as example;
import 'package:flutter_test/flutter_test.dart';

void main() {
  testWidgets('The buzz widget should move around', (WidgetTester tester) async {
    await tester.pumpWidget(const example.Curve2DExampleApp());

    final Finder textFinder = find.widgetWithText(CircleAvatar, 'B');
    expect(tester.getCenter(textFinder), const Offset(58, 440));

    const List<Offset> expectedOffsets = <Offset>[
      Offset(43, 407),
      Offset(81, 272),
      Offset(185, 103),
      Offset(378, 92),
      Offset(463, 392),
      Offset(479, 124),
      Offset(745, 389),
      Offset(450, 555),
      Offset(111, 475),
      Offset(58, 440),
      Offset(43, 407),
    ];

    const int steps = 10;
    for (int i = 0; i <= steps; i++) {
      await tester.pump(const Duration(seconds: 3) * (1 / steps));
      final Offset center = tester.getCenter(textFinder);
      expect(center.dx, moreOrLessEquals(expectedOffsets[i].dx, epsilon: 1));
      expect(center.dy, moreOrLessEquals(expectedOffsets[i].dy, epsilon: 1));
    }
  });
}
