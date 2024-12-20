// Copyright 2014 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

import 'package:flutter/cupertino.dart';
import 'package:flutter_api_samples/cupertino/nav_bar/cupertino_navigation_bar.2.dart' as example;
import 'package:flutter_test/flutter_test.dart';

void main() {
  testWidgets('CupertinoNavigationBar is large', (WidgetTester tester) async {
    await tester.pumpWidget(const example.NavBarApp());

    final Finder navBarFinder = find.byType(CupertinoNavigationBar);
    expect(navBarFinder, findsOneWidget);
    expect(find.text('Large Sample'), findsOneWidget);
    expect(find.text('0'), findsOneWidget);
    expect(find.text('1'), findsNothing);
    await tester.tap(find.text('Increment'));
    await tester.pump();
    expect(find.text('0'), findsNothing);
    expect(find.text('1'), findsOneWidget);
  });
}
