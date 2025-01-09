// Copyright 2014 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

import 'dart:math' as math;

import 'package:flutter/material.dart';
import 'package:flutter_api_samples/widgets/transitions/animated_widget.0.dart' as example;
import 'package:flutter_test/flutter_test.dart';

void main() {
  testWidgets('Rotates green container', (WidgetTester tester) async {
    await tester.pumpWidget(const example.AnimatedWidgetExampleApp());
    expect(find.byType(Container), findsOneWidget);
    expect(
      tester.widget(find.byType(Container)),
      isA<Container>().having((Container container) => container.color, 'color', Colors.green),
    );

    expect(
      find.byWidgetPredicate(
        (Widget widget) =>
            widget is Transform && widget.transform == Transform.rotate(angle: 0.0).transform,
      ),
      findsOneWidget,
    );

    await tester.pump(const Duration(seconds: 5));
    await tester.pump();

    expect(
      find.byWidgetPredicate(
        (Widget widget) =>
            widget is Transform && widget.transform == Transform.rotate(angle: math.pi).transform,
      ),
      findsOneWidget,
    );
  });
}
