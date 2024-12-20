// Copyright 2014 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

import 'package:a11y_assessments/use_cases/check_box_list_tile.dart';
import 'package:flutter_test/flutter_test.dart';

import 'test_utils.dart';

void main() {
  testWidgets('check box list tile use-case renders check boxes', (WidgetTester tester) async {
    await pumpsUseCase(tester, CheckBoxListTile());
    expect(find.text('a check box list title'), findsOneWidget);
    expect(find.text('a disabled check box list title'), findsOneWidget);
  });

  testWidgets('check box list has one h1 tag', (WidgetTester tester) async {
    await pumpsUseCase(tester, CheckBoxListTile());
    final Finder findHeadingLevelOnes = find.bySemanticsLabel(RegExp('CheckBoxListTile Demo'));
    await tester.pumpAndSettle();
    expect(findHeadingLevelOnes, findsOne);
  });
}
