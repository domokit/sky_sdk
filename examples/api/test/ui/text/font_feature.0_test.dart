// Copyright 2014 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

import 'package:flutter/material.dart';
import 'package:flutter_api_samples/ui/text/font_feature.0.dart' as example;
import 'package:flutter_test/flutter_test.dart';

void main() {
  testWidgets('shows font features', (WidgetTester tester) async {
    await tester.pumpWidget(const MaterialApp(home: example.ExampleWidget()));

    expect(find.byType(Text), findsNWidgets(9));
    expect((tester.widget(find.byType(Text).at(0)) as Text).style!.fontSize, equals(18.0));
    expect((tester.widget(find.byType(Text).at(1)) as Text).style!.fontFamily, equals('Cardo'));
    expect(
      (tester.widget(find.byType(Text).at(3)) as Text).style!.fontFeatures,
      equals(const <FontFeature>[FontFeature.oldstyleFigures()]),
    );
    expect(
      (tester.widget(find.byType(Text).at(5)) as Text).style!.fontFeatures,
      equals(const <FontFeature>[FontFeature.alternativeFractions()]),
    );
    expect(
      (tester.widget(find.byType(Text).at(8)) as Text).style!.fontFeatures,
      equals(<FontFeature>[FontFeature.stylisticSet(1)]),
    );
  });
}
