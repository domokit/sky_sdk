// Copyright 2014 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

import 'package:flutter/gestures.dart';
import 'package:flutter/material.dart';
import 'package:flutter_api_samples/widgets/gesture_detector/gesture_detector.3.dart' as example;
import 'package:flutter_test/flutter_test.dart';

void main() {
  testWidgets('The red box always moves inside the green box', (WidgetTester tester) async {
    await tester.pumpWidget(const example.DragBoundaryExampleApp());
    final Finder greenFinder = find.byType(Container).first;
    final Finder redFinder = find.byType(Container).last;
    final TestGesture drag = await tester.startGesture(tester.getCenter(redFinder));
    await tester.pump(kLongPressTimeout);
    await drag.moveBy(const Offset(1000, 1000));
    await tester.pumpAndSettle();
    expect(tester.getBottomRight(redFinder), tester.getBottomRight(greenFinder));
    await drag.moveBy(const Offset(-2000, -2000));
    await tester.pumpAndSettle();
    expect(tester.getTopLeft(redFinder), tester.getTopLeft(greenFinder));
    await drag.up();
    await tester.pumpAndSettle();
  });
}
