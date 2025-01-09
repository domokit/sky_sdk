// Copyright 2014 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

import 'package:flutter/material.dart';
import 'package:flutter_api_samples/ui/text/font_feature.font_feature_historical_ligatures.0.dart'
    as example;
import 'package:flutter_test/flutter_test.dart';

void main() {
  testWidgets('shows font features', (WidgetTester tester) async {
    await tester.pumpWidget(const MaterialApp(home: example.ExampleWidget()));

    expect(find.byType(Text), findsOneWidget);
    expect(
      (tester.widget(find.byType(Text).first) as Text).style!.fontFamily,
      equals('Sorts Mill Goudy'),
    );
    expect(
      (tester.widget(find.byType(Text).first) as Text).style!.fontFeatures,
      equals(const <FontFeature>[FontFeature.historicalForms(), FontFeature.historicalLigatures()]),
    );
  });
}
