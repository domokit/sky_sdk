// Copyright 2014 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

import 'package:flutter/cupertino.dart';
import 'package:flutter_api_samples/cupertino/nav_bar/cupertino_navigation_bar.0.dart' as example;
import 'package:flutter_test/flutter_test.dart';

void main() {
  testWidgets('CupertinoNavigationBar is semi transparent', (WidgetTester tester) async {
    await tester.pumpWidget(const example.NavBarApp());

    final Finder navBarFinder = find.byType(CupertinoNavigationBar);
    expect(navBarFinder, findsOneWidget);
    final CupertinoNavigationBar cupertinoNavigationBar = tester.widget<CupertinoNavigationBar>(
      navBarFinder,
    );
    expect(cupertinoNavigationBar.backgroundColor, CupertinoColors.systemGrey.withOpacity(0.5));
  });
}
