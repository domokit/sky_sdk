// Copyright 2014 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

import 'package:flutter/material.dart';
import 'package:flutter_api_samples/material/card/card.2.dart' as example;
import 'package:flutter_test/flutter_test.dart';

void main() {
  testWidgets('Card variants', (WidgetTester tester) async {
    await tester.pumpWidget(const example.CardExamplesApp());

    expect(find.byType(Card), findsNWidgets(3));

    expect(find.widgetWithText(Card, 'Elevated Card'), findsOneWidget);
    expect(find.widgetWithText(Card, 'Filled Card'), findsOneWidget);
    expect(find.widgetWithText(Card, 'Outlined Card'), findsOneWidget);

    Material getCardMaterial(WidgetTester tester, int cardIndex) {
      return tester.widget<Material>(
        find.descendant(of: find.byType(Card).at(cardIndex), matching: find.byType(Material)),
      );
    }

    final Material defaultCard = getCardMaterial(tester, 0);
    expect(defaultCard.clipBehavior, Clip.none);
    expect(defaultCard.elevation, 1.0);
    expect(
      defaultCard.shape,
      const RoundedRectangleBorder(borderRadius: BorderRadius.all(Radius.circular(12.0))),
    );
    expect(defaultCard.color, const Color(0xfff7f2fa));
    expect(defaultCard.shadowColor, const Color(0xff000000));
    expect(defaultCard.surfaceTintColor, Colors.transparent);

    final Material filledCard = getCardMaterial(tester, 1);
    expect(filledCard.clipBehavior, Clip.none);
    expect(filledCard.elevation, 0.0);
    expect(
      filledCard.shape,
      const RoundedRectangleBorder(borderRadius: BorderRadius.all(Radius.circular(12.0))),
    );
    expect(filledCard.color, const Color(0xffe6e0e9));
    expect(filledCard.shadowColor, const Color(0xff000000));
    expect(filledCard.surfaceTintColor, const Color(0x00000000));

    final Material outlinedCard = getCardMaterial(tester, 2);
    expect(outlinedCard.clipBehavior, Clip.none);
    expect(outlinedCard.elevation, 0.0);
    expect(
      outlinedCard.shape,
      const RoundedRectangleBorder(
        side: BorderSide(color: Color(0xffcac4d0)),
        borderRadius: BorderRadius.all(Radius.circular(12.0)),
      ),
    );
    expect(outlinedCard.color, const Color(0xfffef7ff));
    expect(outlinedCard.shadowColor, const Color(0xff000000));
    expect(outlinedCard.surfaceTintColor, Colors.transparent);
  });
}
