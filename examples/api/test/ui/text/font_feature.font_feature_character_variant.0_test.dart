// Copyright 2014 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

import 'package:flutter/material.dart';
import 'package:flutter_api_samples/ui/text/font_feature.font_feature_character_variant.0.dart'
    as example;
import 'package:flutter_test/flutter_test.dart';

void main() {
  testWidgets('shows font features', (WidgetTester tester) async {
    await tester.pumpWidget(const MaterialApp(home: example.ExampleWidget()));

    expect(find.byType(Text), findsOneWidget);
    expect(
      (tester.widget(find.byType(Text).first) as Text).style!.fontFamily,
      equals('Source Code Pro'),
    );
    expect(
      (tester.widget(find.byType(Text).first) as Text).style!.fontFeatures,
      equals(<FontFeature>[
        FontFeature.characterVariant(1),
        FontFeature.characterVariant(2),
        FontFeature.characterVariant(4),
      ]),
    );
  });
}
