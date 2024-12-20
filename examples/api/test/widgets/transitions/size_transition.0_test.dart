// Copyright 2014 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

import 'package:flutter/material.dart';
import 'package:flutter_api_samples/widgets/transitions/size_transition.0.dart' as example;
import 'package:flutter_test/flutter_test.dart';

void main() {
  testWidgets('Shows flutter logo in transition', (WidgetTester tester) async {
    await tester.pumpWidget(const example.SizeTransitionExampleApp());
    expect(find.byType(FlutterLogo), findsOneWidget);
    expect(find.byType(Center), findsOneWidget);
    expect(
      find.descendant(of: find.byType(Center), matching: find.byType(FlutterLogo)),
      findsOneWidget,
    );
    expect(find.byType(SizeTransition), findsOneWidget);

    expect(
      tester.widget(find.byType(SizeTransition)),
      isA<SizeTransition>()
          .having((SizeTransition transition) => transition.axis, 'axis', Axis.horizontal)
          .having((SizeTransition transition) => transition.axisAlignment, 'axis alignment', -1)
          .having(
            (SizeTransition transition) => transition.sizeFactor,
            'factor',
            isA<CurvedAnimation>()
                .having(
                  (CurvedAnimation animation) => animation.curve,
                  'curve',
                  Curves.fastOutSlowIn,
                )
                .having(
                  (CurvedAnimation animation) => animation.parent,
                  'paren',
                  isA<AnimationController>().having(
                    (AnimationController controller) => controller.duration,
                    'duration',
                    const Duration(seconds: 3),
                  ),
                ),
          ),
    );
  });
}
