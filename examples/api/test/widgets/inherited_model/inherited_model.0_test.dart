// Copyright 2014 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

import 'package:flutter/material.dart';
import 'package:flutter_api_samples/widgets/inherited_model/inherited_model.0.dart' as example;
import 'package:flutter_test/flutter_test.dart';

void main() {
  testWidgets('Rebuild widget using InheritedModel', (WidgetTester tester) async {
    await tester.pumpWidget(const example.InheritedModelApp());

    BoxDecoration? decoration =
        tester.widget<AnimatedContainer>(find.byType(AnimatedContainer).first).decoration
            as BoxDecoration?;
    expect(decoration!.color, Colors.blue);

    await tester.tap(find.text('Update background'));
    await tester.pumpAndSettle();
    decoration =
        tester.widget<AnimatedContainer>(find.byType(AnimatedContainer).first).decoration
            as BoxDecoration?;
    expect(decoration!.color, Colors.red);

    double? size = tester.widget<FlutterLogo>(find.byType(FlutterLogo)).size;
    expect(size, 100.0);
    await tester.tap(find.text('Resize Logo'));
    await tester.pumpAndSettle();
    size = tester.widget<FlutterLogo>(find.byType(FlutterLogo)).size;
    expect(size, 200.0);
  });
}
