// Copyright 2014 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

import 'package:flutter/material.dart';
import 'package:flutter/rendering.dart';
import 'package:flutter_api_samples/widgets/sliver/sliver_main_axis_group.0.dart' as example;
import 'package:flutter_test/flutter_test.dart';

void main() {
  testWidgets('SliverMainAxisGroup example', (WidgetTester tester) async {
    await tester.pumpWidget(const example.SliverMainAxisGroupExampleApp());

    final RenderSliverMainAxisGroup renderSliverGroup = tester.renderObject(
      find.byType(SliverMainAxisGroup),
    );
    expect(renderSliverGroup, isNotNull);

    final RenderSliverPersistentHeader renderAppBar = tester
        .renderObject<RenderSliverPersistentHeader>(find.byType(SliverAppBar));
    final RenderSliverList renderSliverList = tester.renderObject<RenderSliverList>(
      find.byType(SliverList),
    );
    final RenderSliverToBoxAdapter renderSliverAdapter = tester
        .renderObject<RenderSliverToBoxAdapter>(
          find.descendant(
            of: find.byType(SliverMainAxisGroup),
            matching: find.byType(SliverToBoxAdapter, skipOffstage: false),
            skipOffstage: false,
          ),
        );

    // renderAppBar, renderSliverList, and renderSliverAdapter1 are part of the same sliver group.
    expect(renderAppBar.geometry!.scrollExtent, equals(70.0));
    expect(renderSliverList.geometry!.scrollExtent, equals(100.0 * 5));
    expect(renderSliverAdapter.geometry!.scrollExtent, equals(100.0));
    expect(renderSliverGroup.geometry!.scrollExtent, equals(70.0 + 100.0 * 5 + 100.0));
  });
}
