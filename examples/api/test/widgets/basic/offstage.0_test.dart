// Copyright 2014 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

import 'package:flutter_api_samples/widgets/basic/offstage.0.dart' as example;
import 'package:flutter_test/flutter_test.dart';

void main() {
  testWidgets('Can off/on stage Flutter logo widget', (WidgetTester tester) async {
    await tester.pumpWidget(const example.OffstageApp());

    // The Flutter logo is off stage and not visible.
    expect(find.text('Flutter logo is offstage: true'), findsOneWidget);

    // Tap to get the Flutter logo size.
    await tester.tap(find.text('Get Flutter Logo size'));
    await tester.pumpAndSettle();

    expect(find.text('Flutter Logo size is Size(150.0, 150.0)'), findsOneWidget);

    // Tap to toggle the offstage value.
    await tester.tap(find.text('Toggle Offstage Value'));
    await tester.pumpAndSettle();

    // The Flutter logo is on stage and visible.
    expect(find.text('Flutter logo is offstage: false'), findsOneWidget);
  });
}
