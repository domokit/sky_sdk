// Copyright 2014 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

import 'package:flutter/material.dart';
import 'package:flutter_api_samples/material/app_bar/sliver_app_bar.1.dart' as example;
import 'package:flutter_test/flutter_test.dart';

const Offset _kOffset = Offset(0.0, -200.0);

void main() {
  testWidgets('SliverAppbar can be pinned', (WidgetTester tester) async {
    await tester.pumpWidget(const example.AppBarApp());

    expect(find.widgetWithText(SliverAppBar, 'SliverAppBar'), findsOneWidget);
    expect(tester.getBottomLeft(find.text('SliverAppBar')).dy, 144.0);

    await tester.drag(find.text('0'), _kOffset, touchSlopY: 0, warnIfMissed: false);
    await tester.pump();
    await tester.pump(const Duration(milliseconds: 500));
    expect(tester.getBottomLeft(find.text('SliverAppBar')).dy, 40.0);
  });
}
