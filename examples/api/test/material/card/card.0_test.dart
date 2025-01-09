// Copyright 2014 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

import 'package:flutter/material.dart';
import 'package:flutter_api_samples/material/card/card.0.dart' as example;
import 'package:flutter_test/flutter_test.dart';

void main() {
  testWidgets('Card Smoke Test', (WidgetTester tester) async {
    await tester.pumpWidget(const example.CardExampleApp());
    expect(find.byType(Card), findsOneWidget);
    expect(find.widgetWithIcon(Card, Icons.album), findsOneWidget);
    expect(find.widgetWithText(Card, 'The Enchanted Nightingale'), findsOneWidget);
    expect(
      find.widgetWithText(Card, 'Music by Julie Gable. Lyrics by Sidney Stein.'),
      findsOneWidget,
    );
    expect(find.widgetWithText(Card, 'BUY TICKETS'), findsOneWidget);
    expect(find.widgetWithText(Card, 'LISTEN'), findsOneWidget);
  });
}
