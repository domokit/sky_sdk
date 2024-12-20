// Copyright 2014 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

import 'package:flutter/material.dart';
import 'package:flutter_api_samples/widgets/media_query/media_query_data.system_gesture_insets.0.dart'
    as example;
import 'package:flutter_test/flutter_test.dart';

void main() {
  testWidgets('The slider should be padded with the system gesture insets', (
    WidgetTester tester,
  ) async {
    tester.view.systemGestureInsets = const FakeViewPadding(left: 60, right: 60);
    await tester.pumpWidget(const example.SystemGestureInsetsExampleApp());

    expect(find.widgetWithText(AppBar, 'Pad Slider to avoid systemGestureInsets'), findsOne);

    final Rect rect = tester.getRect(find.byType(Slider));

    expect(rect, rectMoreOrLessEquals(const Rect.fromLTRB(20.0, 56.0, 780.0, 600.0)));
  });
}
