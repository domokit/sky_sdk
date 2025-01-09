// Copyright 2014 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

import 'package:flutter/gestures.dart';
import 'package:flutter/material.dart';
import 'package:flutter_api_samples/widgets/nested_scroll_view/nested_scroll_view_state.0.dart'
    as example;
import 'package:flutter_test/flutter_test.dart';

void main() {
  testWidgets('Access the outer and inner controllers', (WidgetTester tester) async {
    await tester.pumpWidget(const example.NestedScrollViewStateExampleApp());

    expect(find.byType(NestedScrollView), findsOne);
    expect(find.widgetWithText(SliverAppBar, 'NestedScrollViewState Demo!'), findsOne);
    expect(find.byType(CustomScrollView), findsOne);

    final example.NestedScrollViewStateExample widget = tester
        .widget<example.NestedScrollViewStateExample>(
          find.byType(example.NestedScrollViewStateExample),
        );

    final ScrollController outerController = widget.outerController;
    final ScrollController innerController = widget.innerController;

    expect(outerController.offset, 0);
    expect(innerController.offset, 0);

    await tester.sendEventToBinding(const PointerScrollEvent(scrollDelta: Offset(0.0, 10.0)));
    await tester.pump();

    await tester.pumpAndSettle();

    expect(outerController.offset, 10);
    expect(innerController.offset, 0);
  });
}
