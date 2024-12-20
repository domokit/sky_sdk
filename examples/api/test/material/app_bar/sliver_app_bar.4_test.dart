// Copyright 2014 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

import 'package:flutter/material.dart';
import 'package:flutter_api_samples/material/app_bar/sliver_app_bar.4.dart' as example;
import 'package:flutter_test/flutter_test.dart';

const Offset _kOffset = Offset(0.0, 200.0);

void main() {
  testWidgets('SliverAppbar can be stretched', (WidgetTester tester) async {
    await tester.pumpWidget(const example.StretchableSliverAppBar());

    final Finder switchFinder = find.byType(Switch);
    Switch materialSwitch = tester.widget<Switch>(switchFinder);
    expect(materialSwitch.value, true);

    expect(find.widgetWithText(SliverAppBar, 'SliverAppBar'), findsOneWidget);
    expect(tester.getBottomLeft(find.text('SliverAppBar')).dy, 184.0);

    await tester.drag(find.text('0'), _kOffset, touchSlopY: 0, warnIfMissed: false);
    await tester.pump();
    await tester.pump(const Duration(milliseconds: 500));
    expect(find.widgetWithText(SliverAppBar, 'SliverAppBar'), findsOneWidget);
    expect(tester.getBottomLeft(find.text('SliverAppBar')).dy, 187.63506380825314);

    await tester.tap(switchFinder);
    await tester.pumpAndSettle();
    materialSwitch = tester.widget<Switch>(switchFinder);
    expect(materialSwitch.value, false);

    await tester.drag(find.text('0'), _kOffset, touchSlopY: 0, warnIfMissed: false);
    await tester.pump();
    await tester.pump(const Duration(milliseconds: 500));

    expect(find.text('SliverAppBar'), findsOneWidget);
    expect(tester.getBottomLeft(find.text('SliverAppBar')).dy, 184.0);
  });
}
