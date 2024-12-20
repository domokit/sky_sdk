// Copyright 2014 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

import 'package:flutter/material.dart';
import 'package:flutter_api_samples/material/input_decorator/input_decoration.label.0.dart'
    as example;
import 'package:flutter_test/flutter_test.dart';

void main() {
  testWidgets('Decorates TextField in sample app with label', (WidgetTester tester) async {
    await tester.pumpWidget(const example.LabelExampleApp());
    expect(find.text('InputDecoration.label Sample'), findsOneWidget);

    expect(find.byType(TextField), findsOneWidget);
    expect(find.text('Username'), findsOneWidget);
    expect(find.text('*'), findsOneWidget);
  });
}
