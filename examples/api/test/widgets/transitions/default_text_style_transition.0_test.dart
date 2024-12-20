// Copyright 2014 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

import 'package:flutter/material.dart';
import 'package:flutter_api_samples/widgets/transitions/default_text_style_transition.0.dart'
    as example;
import 'package:flutter_test/flutter_test.dart';

void main() {
  testWidgets('Transforms text style periodically', (WidgetTester tester) async {
    await tester.pumpWidget(const example.DefaultTextStyleTransitionExampleApp());
    expect(find.byType(Center), findsOneWidget);
    expect(find.byType(Text), findsOneWidget);
    expect(find.text('Flutter'), findsOneWidget);
    expect(find.descendant(of: find.byType(Center), matching: find.byType(Text)), findsOneWidget);
    expect(find.byType(DefaultTextStyleTransition), findsOneWidget);
    expect(
      tester.widget(find.byType(DefaultTextStyleTransition)),
      isA<DefaultTextStyleTransition>().having(
        (DefaultTextStyleTransition transition) => transition.style.value,
        'style',
        const TextStyle(fontSize: 50, color: Colors.blue, fontWeight: FontWeight.w900),
      ),
    );

    await tester.pump(const Duration(seconds: 2));
    await tester.pump();

    expect(
      tester.widget(find.byType(DefaultTextStyleTransition)),
      isA<DefaultTextStyleTransition>().having(
        (DefaultTextStyleTransition transition) => transition.style.value,
        'style',
        const TextStyle(fontSize: 50, color: Colors.red, fontWeight: FontWeight.w100),
      ),
    );

    await tester.pump(const Duration(seconds: 2));
    await tester.pump();

    expect(
      tester.widget(find.byType(DefaultTextStyleTransition)),
      isA<DefaultTextStyleTransition>().having(
        (DefaultTextStyleTransition transition) => transition.style.value,
        'style',
        const TextStyle(fontSize: 50, color: Colors.blue, fontWeight: FontWeight.w900),
      ),
    );
  });
}
