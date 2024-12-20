// Copyright 2014 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

import 'package:flutter/material.dart';
import 'package:flutter_api_samples/material/stepper/stepper.0.dart' as example;
import 'package:flutter_test/flutter_test.dart';

void main() {
  testWidgets('Stepper Smoke Test', (WidgetTester tester) async {
    await tester.pumpWidget(const example.StepperExampleApp());

    expect(find.widgetWithText(AppBar, 'Stepper Sample'), findsOneWidget);
    expect(find.text('Step 1 title').hitTestable(), findsOneWidget);
    expect(find.text('Step 2 title').hitTestable(), findsOneWidget);
    expect(find.text('Content for Step 1').hitTestable(), findsOneWidget);
    expect(find.text('Content for Step 2').hitTestable(), findsNothing);
    final Stepper stepper = tester.widget<Stepper>(find.byType(Stepper));

    // current: 0 & clicks cancel
    stepper.onStepCancel?.call();
    await tester.pumpAndSettle();
    expect(find.text('Content for Step 1').hitTestable(), findsOneWidget);
    expect(find.text('Content for Step 2').hitTestable(), findsNothing);

    // current: 0 & clicks 0th step
    stepper.onStepTapped?.call(0);
    await tester.pumpAndSettle();
    expect(find.text('Content for Step 1').hitTestable(), findsOneWidget);
    expect(find.text('Content for Step 2').hitTestable(), findsNothing);

    // current: 0 & clicks continue
    stepper.onStepContinue?.call();
    await tester.pumpAndSettle();
    expect(find.text('Content for Step 1').hitTestable(), findsNothing);
    expect(find.text('Content for Step 2').hitTestable(), findsOneWidget);

    // current: 1 & clicks 1st step
    stepper.onStepTapped?.call(1);
    await tester.pumpAndSettle();
    expect(find.text('Content for Step 1').hitTestable(), findsNothing);
    expect(find.text('Content for Step 2').hitTestable(), findsOneWidget);

    // current: 1 & clicks continue
    stepper.onStepContinue?.call();
    await tester.pumpAndSettle();
    expect(find.text('Content for Step 1').hitTestable(), findsNothing);
    expect(find.text('Content for Step 2').hitTestable(), findsOneWidget);

    // current: 1 & clicks cancel
    stepper.onStepCancel?.call();
    await tester.pumpAndSettle();
    expect(find.text('Content for Step 1').hitTestable(), findsOneWidget);
    expect(find.text('Content for Step 2').hitTestable(), findsNothing);

    // current: 0 & clicks 1st step
    stepper.onStepTapped?.call(1);
    await tester.pumpAndSettle();
    expect(find.text('Content for Step 1').hitTestable(), findsNothing);
    expect(find.text('Content for Step 2').hitTestable(), findsOneWidget);

    // current: 1 & clicks 0th step
    stepper.onStepTapped?.call(0);
    await tester.pumpAndSettle();
    expect(find.text('Content for Step 1').hitTestable(), findsOneWidget);
    expect(find.text('Content for Step 2').hitTestable(), findsNothing);
  });
}
