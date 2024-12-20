// Copyright 2014 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

import 'package:flutter/material.dart';
import 'package:flutter_api_samples/material/stepper/step_style.0.dart' as example;
import 'package:flutter_test/flutter_test.dart';

void main() {
  testWidgets('StepStyle Smoke Test', (WidgetTester tester) async {
    await tester.pumpWidget(const example.StepStyleExampleApp());

    expect(find.widgetWithText(AppBar, 'Step Style Example'), findsOneWidget);

    final Stepper stepper = tester.widget<Stepper>(find.byType(Stepper));
    // Check that the stepper has the correct properties.
    expect(stepper.type, StepperType.horizontal);
    expect(stepper.stepIconHeight, 48);
    expect(stepper.stepIconWidth, 48);
    expect(stepper.stepIconMargin, EdgeInsets.zero);

    // Check that the first step has the correct properties.
    final Step firstStep = stepper.steps[0];
    expect(firstStep.title, isA<SizedBox>());
    expect(firstStep.content, isA<SizedBox>());
    expect(firstStep.isActive, true);
    expect(firstStep.stepStyle?.connectorThickness, 10);
    expect(firstStep.stepStyle?.color, Colors.white);
    expect(firstStep.stepStyle?.connectorColor, Colors.red);
    expect(firstStep.stepStyle?.indexStyle?.color, Colors.black);
    expect(firstStep.stepStyle?.indexStyle?.fontSize, 20);
    expect(firstStep.stepStyle?.border, Border.all(width: 2));

    // Check that the second step has the correct properties.
    final Step secondStep = stepper.steps[1];
    expect(secondStep.title, isA<SizedBox>());
    expect(secondStep.content, isA<SizedBox>());
    expect(secondStep.isActive, true);
    expect(secondStep.stepStyle?.connectorThickness, 10);
    expect(secondStep.stepStyle?.connectorColor, Colors.orange);
    expect(
      secondStep.stepStyle?.gradient,
      const LinearGradient(colors: <Color>[Colors.white, Colors.black]),
    );

    // Check that the third step has the correct properties.
    final Step thirdStep = stepper.steps[2];
    expect(thirdStep.title, isA<SizedBox>());
    expect(thirdStep.content, isA<SizedBox>());
    expect(thirdStep.isActive, true);
    expect(thirdStep.stepStyle?.connectorThickness, 10);
    expect(thirdStep.stepStyle?.color, Colors.white);
    expect(thirdStep.stepStyle?.connectorColor, Colors.blue);
    expect(thirdStep.stepStyle?.indexStyle?.color, Colors.black);
    expect(thirdStep.stepStyle?.indexStyle?.fontSize, 20);
    expect(thirdStep.stepStyle?.border, Border.all(width: 2));

    // Check that the fourth step has the correct properties.
    final Step fourthStep = stepper.steps[3];
    expect(fourthStep.title, isA<SizedBox>());
    expect(fourthStep.content, isA<SizedBox>());
    expect(fourthStep.isActive, true);
    expect(fourthStep.stepStyle?.color, Colors.white);
    expect(fourthStep.stepStyle?.indexStyle?.color, Colors.black);
    expect(fourthStep.stepStyle?.indexStyle?.fontSize, 20);
    expect(fourthStep.stepStyle?.border, Border.all(width: 2));
  });
}
