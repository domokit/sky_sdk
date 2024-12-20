// Copyright 2014 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

import 'package:flutter/material.dart';
import 'package:flutter/rendering.dart';
import 'package:flutter_api_samples/widgets/transitions/sliver_fade_transition.0.dart' as example;
import 'package:flutter_test/flutter_test.dart';

void main() {
  testWidgets('Shows color list in transition', (WidgetTester tester) async {
    await tester.pumpWidget(const example.SliverFadeTransitionExampleApp());
    expect(find.text('SliverFadeTransition Sample'), findsOneWidget);
    expect(find.byType(SliverFadeTransition), findsOneWidget);
    expect(find.byType(CustomScrollView), findsOneWidget);
    expect(
      find.byWidgetPredicate(
        (Widget widget) => widget is Container && widget.color == Colors.indigo[200],
      ),
      findsNWidgets(3),
    );
    expect(
      find.byWidgetPredicate(
        (Widget widget) => widget is Container && widget.color == Colors.orange[200],
      ),
      findsNWidgets(2),
    );
  });

  testWidgets('Animates repeatedly every second', (WidgetTester tester) async {
    await tester.pumpWidget(const example.SliverFadeTransitionExampleApp());

    expect(
      tester.renderObject(find.byType(SliverFadeTransition)),
      isA<RenderSliverAnimatedOpacity>().having(
        (RenderSliverAnimatedOpacity obj) => obj.opacity.value,
        'opacity',
        0.0,
      ),
    );

    await tester.pump(const Duration(seconds: 1));
    await tester.pump();

    expect(
      tester.renderObject(find.byType(SliverFadeTransition)),
      isA<RenderSliverAnimatedOpacity>().having(
        (RenderSliverAnimatedOpacity obj) => obj.opacity.value,
        'opacity',
        1.0,
      ),
    );
  });
}
