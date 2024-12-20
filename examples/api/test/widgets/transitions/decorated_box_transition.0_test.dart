// Copyright 2014 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

import 'package:flutter/material.dart';
import 'package:flutter_api_samples/widgets/transitions/decorated_box_transition.0.dart' as example;
import 'package:flutter_test/flutter_test.dart';

void main() {
  testWidgets('Shows container in 3 second loop', (WidgetTester tester) async {
    await tester.pumpWidget(const example.DecoratedBoxTransitionExampleApp());
    expect(find.byType(FlutterLogo), findsOneWidget);
    expect(find.byType(Center), findsOneWidget);
    expect(
      find.descendant(of: find.byType(Center), matching: find.byType(FlutterLogo)),
      findsOneWidget,
    );
    expect(
      find.ancestor(of: find.byType(FlutterLogo), matching: find.byType(Container)),
      findsAtLeast(1),
    );
    expect(find.byType(DecoratedBoxTransition), findsOneWidget);

    expect(
      tester.widget(find.byType(DecoratedBoxTransition)),
      isA<DecoratedBoxTransition>().having(
        (DecoratedBoxTransition transition) => transition.decoration.value,
        'decoration',
        BoxDecoration(
          color: const Color(0xFFFFFFFF),
          border: Border.all(style: BorderStyle.none),
          borderRadius: BorderRadius.circular(60.0),
          boxShadow: const <BoxShadow>[
            BoxShadow(
              color: Color(0x66666666),
              blurRadius: 10.0,
              spreadRadius: 3.0,
              offset: Offset(0, 6.0),
            ),
          ],
        ),
      ),
    );

    await tester.pump(const Duration(seconds: 3));
    await tester.pump();

    expect(
      tester.widget(find.byType(DecoratedBoxTransition)),
      isA<DecoratedBoxTransition>().having(
        (DecoratedBoxTransition transition) => transition.decoration.value,
        'decoration',
        BoxDecoration(
          color: const Color(0xFFFFFFFF),
          border: Border.all(style: BorderStyle.none),
          borderRadius: BorderRadius.zero,
          // No shadow.
        ),
      ),
    );
  });
}
