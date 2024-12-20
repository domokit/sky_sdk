// Copyright 2014 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

import 'package:a11y_assessments/use_cases/radio_list_tile.dart';
import 'package:flutter_test/flutter_test.dart';

import 'test_utils.dart';

void main() {
  testWidgets('radio list tile use-case renders radio buttons', (WidgetTester tester) async {
    await pumpsUseCase(tester, RadioListTileUseCase());
    expect(find.text('Lafayette'), findsOneWidget);
    expect(find.text('Jefferson'), findsOneWidget);
  });

  testWidgets('radio list tile demo page has one h1 tag', (WidgetTester tester) async {
    await pumpsUseCase(tester, RadioListTileUseCase());
    final Finder findHeadingLevelOnes = find.bySemanticsLabel('RadioListTile Demo');
    await tester.pumpAndSettle();
    expect(findHeadingLevelOnes, findsOne);
  });
}
