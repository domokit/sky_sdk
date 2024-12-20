// Copyright 2014 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

import 'package:flutter/material.dart';
import 'package:flutter_api_samples/widgets/implicit_animations/animated_padding.0.dart' as example;
import 'package:flutter_test/flutter_test.dart';

void main() {
  testWidgets('AnimatedPadding animates on ElevatedButton tap', (WidgetTester tester) async {
    await tester.pumpWidget(const example.AnimatedPaddingExampleApp());

    Padding padding = tester.widget(
      find.descendant(of: find.byType(AnimatedPadding), matching: find.byType(Padding)),
    );
    expect(padding.padding, equals(EdgeInsets.zero));

    await tester.tap(find.byType(ElevatedButton));
    await tester.pump();

    padding = tester.widget(
      find.descendant(of: find.byType(AnimatedPadding), matching: find.byType(Padding)),
    );
    expect(padding.padding, equals(EdgeInsets.zero));

    // Advance animation to the end by the 2-second duration specified in
    // the example app.
    await tester.pump(const Duration(seconds: 2));

    padding = tester.widget(
      find.descendant(of: find.byType(AnimatedPadding), matching: find.byType(Padding)),
    );
    expect(padding.padding, equals(const EdgeInsets.all(100.0)));
  });
}
