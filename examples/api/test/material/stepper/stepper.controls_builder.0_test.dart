// Copyright 2014 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

import 'package:flutter/material.dart';
import 'package:flutter_api_samples/material/stepper/stepper.controls_builder.0.dart' as example;
import 'package:flutter_test/flutter_test.dart';

void main() {
  testWidgets('Stepper control builder can be overridden to display custom buttons', (
    WidgetTester tester,
  ) async {
    await tester.pumpWidget(const example.ControlsBuilderExampleApp());

    expect(find.widgetWithText(AppBar, 'Stepper Sample'), findsOne);
    expect(find.text('A').hitTestable(), findsOne);
    expect(find.text('B').hitTestable(), findsOne);
    expect(find.widgetWithText(TextButton, 'NEXT').hitTestable(), findsOne);
    expect(find.widgetWithText(TextButton, 'CANCEL').hitTestable(), findsOne);
  });
}
