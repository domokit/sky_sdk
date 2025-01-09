// Copyright 2014 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

import 'package:flutter/material.dart';
import 'package:flutter_api_samples/material/input_decorator/input_decoration.label_style_error.0.dart'
    as example;
import 'package:flutter_test/flutter_test.dart';

void main() {
  testWidgets('InputDecorator label uses error color', (WidgetTester tester) async {
    await tester.pumpWidget(const example.LabelStyleErrorExampleApp());
    final Theme theme = tester.firstWidget(find.byType(Theme));

    final AnimatedDefaultTextStyle label = tester.firstWidget(
      find.ancestor(of: find.text('Name'), matching: find.byType(AnimatedDefaultTextStyle)),
    );
    expect(label.style.color, theme.data.colorScheme.error);
  });
}
