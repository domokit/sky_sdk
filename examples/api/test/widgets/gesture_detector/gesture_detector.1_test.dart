// Copyright 2014 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

import 'package:flutter/material.dart';
import 'package:flutter_api_samples/widgets/gesture_detector/gesture_detector.1.dart' as example;
import 'package:flutter_test/flutter_test.dart';

void main() {
  testWidgets('GestureDetector updates Container color on tap', (WidgetTester tester) async {
    await tester.pumpWidget(const example.GestureDetectorExampleApp());

    Container container = tester.widget(
      find.ancestor(of: find.byType(GestureDetector), matching: find.byType(Container)),
    );

    expect(container.color, Colors.white);

    await tester.tap(find.byType(GestureDetector));
    await tester.pump();

    container = tester.widget(
      find.ancestor(of: find.byType(GestureDetector), matching: find.byType(Container)),
    );

    expect(container.color, Colors.yellow);

    await tester.tap(find.byType(GestureDetector));
    await tester.pump();

    container = tester.widget(
      find.ancestor(of: find.byType(GestureDetector), matching: find.byType(Container)),
    );

    expect(container.color, Colors.white);
  });
}
