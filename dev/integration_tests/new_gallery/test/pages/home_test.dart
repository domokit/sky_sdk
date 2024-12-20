// Copyright 2014 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

import 'package:flutter/material.dart';
import 'package:flutter_test/flutter_test.dart';
import 'package:gallery/main.dart';

void main() {
  testWidgets('Home page hides settings semantics when closed', (WidgetTester tester) async {
    await tester.pumpWidget(const GalleryApp());

    await tester.pump(const Duration(seconds: 1));

    expect(find.bySemanticsLabel('Settings'), findsOneWidget);
    expect(find.bySemanticsLabel('Close settings'), findsNothing);

    await tester.tap(find.bySemanticsLabel('Settings'));
    await tester.pump(const Duration(seconds: 1));

    // The test no longer finds Setting and Close settings since the semantics
    // are excluded when settings mode is activated.
    expect(find.bySemanticsLabel('Settings'), findsNothing);
    expect(find.bySemanticsLabel('Close settings'), findsOneWidget);
  });

  testWidgets('Home page list view is the primary list view', (WidgetTester tester) async {
    await tester.pumpWidget(const GalleryApp());
    await tester.pumpAndSettle();

    final ListView listview = tester.widget(find.byKey(const ValueKey<String>('HomeListView')));

    expect(listview.primary, true);
  });
}
