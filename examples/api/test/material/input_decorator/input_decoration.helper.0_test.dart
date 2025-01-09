// Copyright 2014 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

import 'package:flutter/material.dart';
import 'package:flutter_api_samples/material/input_decorator/input_decoration.helper.0.dart'
    as example;
import 'package:flutter_test/flutter_test.dart';

void main() {
  testWidgets('Shows multi element InputDecorator help decoration', (WidgetTester tester) async {
    await tester.pumpWidget(const example.HelperExampleApp());

    expect(find.byType(TextField), findsOneWidget);
    expect(find.text('Helper Text '), findsOneWidget);
    expect(find.byIcon(Icons.help_outline), findsOneWidget);
  });
}
