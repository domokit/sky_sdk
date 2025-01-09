// Copyright 2014 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

import 'package:flutter/material.dart';
import 'package:flutter_api_samples/painting/borders/border_side.stroke_align.0.dart' as example;
import 'package:flutter_test/flutter_test.dart';

void main() {
  testWidgets('Finds the expected BorderedBox', (WidgetTester tester) async {
    await tester.pumpWidget(const MaterialApp(home: example.StrokeAlignExample()));

    expect(find.byType(example.StrokeAlignExample), findsOneWidget);
    expect(find.byType(example.BorderedBox), findsNWidgets(10));
  });
}
