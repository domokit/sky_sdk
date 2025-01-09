// Copyright 2014 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

import 'package:flutter/material.dart';
import 'package:flutter_api_samples/material/divider/vertical_divider.1.dart' as example;
import 'package:flutter_test/flutter_test.dart';

void main() {
  testWidgets('Vertical Divider', (WidgetTester tester) async {
    await tester.pumpWidget(const example.VerticalDividerExampleApp());

    expect(find.byType(VerticalDivider), findsOneWidget);

    // Divider is positioned vertically.
    Offset card = tester.getTopRight(find.byType(Card).first);
    expect(card.dx, tester.getTopLeft(find.byType(VerticalDivider)).dx);

    card = tester.getTopLeft(find.byType(Card).last);
    expect(card.dx, tester.getTopRight(find.byType(VerticalDivider)).dx);
  });
}
