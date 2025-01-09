// Copyright 2014 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

import 'package:flutter/material.dart';
import 'package:flutter_gallery/demo/material/drawer_demo.dart';
import 'package:flutter_test/flutter_test.dart';

void main() {
  testWidgets('Drawer header does not scroll', (WidgetTester tester) async {
    await tester.pumpWidget(
      MaterialApp(theme: ThemeData(platform: TargetPlatform.iOS), home: const DrawerDemo()),
    );

    await tester.tap(find.text('Tap here to open the drawer'));
    await tester.pump();
    await tester.pump(const Duration(milliseconds: 500));

    expect(tester.getTopLeft(find.byType(UserAccountsDrawerHeader)).dy, 0.0);
    final double initialTopItemSaneY = tester.getTopLeft(find.text('Drawer item A')).dy;
    expect(initialTopItemSaneY, greaterThan(0.0));

    await tester.drag(find.text('Drawer item B'), const Offset(0.0, 400.0));
    await tester.pump();

    expect(tester.getTopLeft(find.byType(UserAccountsDrawerHeader)).dy, 0.0);
    expect(tester.getTopLeft(find.text('Drawer item A')).dy, greaterThan(initialTopItemSaneY));
    expect(
      tester.getTopLeft(find.text('Drawer item A')).dy,
      lessThanOrEqualTo(initialTopItemSaneY + 400.0),
    );
  });
}
