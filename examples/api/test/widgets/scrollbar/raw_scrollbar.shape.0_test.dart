// Copyright 2014 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

import 'package:flutter/material.dart';
import 'package:flutter_api_samples/widgets/scrollbar/raw_scrollbar.shape.0.dart' as example;
import 'package:flutter_test/flutter_test.dart';

void main() {
  testWidgets('The thumb shape is a stadium border', (WidgetTester tester) async {
    await tester.pumpWidget(const example.ShapeExampleApp());

    expect(find.byType(RawScrollbar), findsOne);
    expect(find.byType(ListView), findsOne);

    expect(find.text('0'), findsOne);
    expect(find.text('1'), findsOne);
    expect(find.text('4'), findsOne);

    await tester.pumpAndSettle();

    expect(
      find.byType(RawScrollbar),
      paints
        ..path(color: Colors.blue)
        ..rrect(
          rrect: RRect.fromLTRBR(786.5, 1.5, 798.5, 178.5, const Radius.circular(6)),
          color: Colors.brown,
        ),
    );
  });
}
