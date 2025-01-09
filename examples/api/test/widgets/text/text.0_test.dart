// Copyright 2014 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

import 'package:flutter/material.dart';
import 'package:flutter_api_samples/widgets/text/text.0.dart' as example;
import 'package:flutter_test/flutter_test.dart';

void main() {
  testWidgets('WidgetsApp test', (WidgetTester tester) async {
    await tester.pumpWidget(const example.DefaultTextStyleApp());

    expect(find.text('Flutter'), findsOneWidget);

    final RichText text = tester.firstWidget(find.byType(RichText));
    expect(text.text.style!.fontSize, 24);
    expect(text.text.style!.fontWeight, FontWeight.bold);

    // Because this example uses Material 3 and a light brightness, the text color
    // should be the color scheme `onSurface` color.
    final Color textColor = ColorScheme.fromSeed(seedColor: Colors.purple).onSurface;
    expect(text.text.style!.color, textColor);
  });
}
