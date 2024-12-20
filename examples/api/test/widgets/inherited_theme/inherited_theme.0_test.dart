// Copyright 2014 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

import 'package:flutter/material.dart';
import 'package:flutter_api_samples/widgets/inherited_theme/inherited_theme.0.dart' as example;
import 'package:flutter_test/flutter_test.dart';

void main() {
  testWidgets('StreamBuilder listens to internal stream', (WidgetTester tester) async {
    await tester.pumpWidget(const example.InheritedThemeExampleApp());

    expect(find.byType(GestureDetector), findsOne);
    expect(find.text('Tap Here'), findsOne);

    final DefaultTextStyle bodyDefaultTextStyle = DefaultTextStyle.of(
      tester.element(find.text('Tap Here')),
    );
    expect(bodyDefaultTextStyle.style, const TextStyle(fontSize: 48, color: Colors.blue));

    await tester.tap(find.text('Tap Here'));
    await tester.pumpAndSettle();

    expect(find.text('Hello World'), findsOne);

    final DefaultTextStyle routeDefaultTextStyle = DefaultTextStyle.of(
      tester.element(find.text('Hello World')),
    );
    expect(routeDefaultTextStyle.style, const TextStyle(fontSize: 48, color: Colors.blue));
  });
}
