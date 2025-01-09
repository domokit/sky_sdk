// Copyright 2014 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

import 'package:flutter/material.dart';
import 'package:flutter/rendering.dart';
import 'package:flutter_api_samples/rendering/growth_direction/growth_direction.0.dart' as example;
import 'package:flutter_test/flutter_test.dart';

void main() {
  testWidgets('Example app has GrowthDirections represented', (WidgetTester tester) async {
    await tester.pumpWidget(const example.ExampleApp());

    final RenderViewport viewport = tester.renderObject(find.byType(Viewport));
    expect(find.text('AxisDirection.down'), findsNWidgets(2));
    expect(find.text('Axis.vertical'), findsNWidgets(2));
    expect(find.text('GrowthDirection.forward'), findsOneWidget);
    expect(find.text('GrowthDirection.reverse'), findsOneWidget);
    expect(find.byIcon(Icons.arrow_upward_rounded), findsNWidgets(2));
    expect(find.byIcon(Icons.arrow_downward_rounded), findsNWidgets(2));
    expect(viewport.axisDirection, AxisDirection.down);
    expect(viewport.anchor, 0.5);
    expect(viewport.center, isNotNull);

    await tester.tap(
      find.byWidgetPredicate((Widget widget) {
        return widget is Radio<AxisDirection> && widget.value == AxisDirection.up;
      }),
    );
    await tester.pumpAndSettle();

    expect(find.text('AxisDirection.up'), findsNWidgets(2));
    expect(find.text('Axis.vertical'), findsNWidgets(2));
    expect(find.text('GrowthDirection.forward'), findsOneWidget);
    expect(find.text('GrowthDirection.reverse'), findsOneWidget);
    expect(find.byIcon(Icons.arrow_upward_rounded), findsNWidgets(2));
    expect(find.byIcon(Icons.arrow_downward_rounded), findsNWidgets(2));
    expect(viewport.axisDirection, AxisDirection.up);
  });
}
