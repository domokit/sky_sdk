// Copyright 2014 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

import 'package:flutter/material.dart';
import 'package:flutter_api_samples/widgets/sliver/decorated_sliver.1.dart' as example;
import 'package:flutter_test/flutter_test.dart';

void main() {
  testWidgets('CustomScrollView clipBehavior is Clip.none when is Clipped is false', (
    WidgetTester tester,
  ) async {
    await tester.pumpWidget(const MaterialApp(home: example.DecoratedSliverClipExample()));

    final CustomScrollView customScrollView = tester.widget(find.byType(CustomScrollView));

    expect(customScrollView.clipBehavior, equals(Clip.none));
  });

  testWidgets('Verify the DecoratedSliver has shadow property in decoration', (
    WidgetTester tester,
  ) async {
    await tester.pumpWidget(
      const MaterialApp(home: example.ResizableCustomScrollView(isClipped: false)),
    );

    final DecoratedSliver decoratedSliver = tester.widget(find.byType(DecoratedSliver));
    final ShapeDecoration shapeDecoration = decoratedSliver.decoration as ShapeDecoration;

    expect(shapeDecoration.shadows, isNotEmpty);
  });

  testWidgets('Verify Slider and Switch widgets', (WidgetTester tester) async {
    await tester.pumpWidget(const example.DecoratedSliverClipExampleApp());

    expect(find.byType(Slider), findsOneWidget);

    expect(find.byType(Switch), findsOneWidget);
  });
}
