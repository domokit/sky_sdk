// Copyright 2014 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

import 'package:flutter/cupertino.dart';
import 'package:flutter_api_samples/cupertino/list_section/list_section_inset.0.dart' as example;
import 'package:flutter_test/flutter_test.dart';

void main() {
  testWidgets('Has exactly 1 CupertinoListSection inset grouped widget', (
    WidgetTester tester,
  ) async {
    await tester.pumpWidget(const example.CupertinoListSectionInsetApp());

    final Finder listSectionFinder = find.byType(CupertinoListSection);
    expect(listSectionFinder, findsOneWidget);

    final CupertinoListSection listSectionWidget = tester.widget<CupertinoListSection>(
      listSectionFinder,
    );
    expect(listSectionWidget.type, equals(CupertinoListSectionType.insetGrouped));
  });

  testWidgets('CupertinoListSection has 3 CupertinoListTile children', (WidgetTester tester) async {
    await tester.pumpWidget(const example.CupertinoListSectionInsetApp());

    expect(find.byType(CupertinoListTile), findsNWidgets(3));
  });
}
