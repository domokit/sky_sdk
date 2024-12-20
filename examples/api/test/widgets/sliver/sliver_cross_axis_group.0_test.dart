// Copyright 2014 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

import 'package:flutter/material.dart';
import 'package:flutter/rendering.dart';
import 'package:flutter_api_samples/widgets/sliver/sliver_cross_axis_group.0.dart' as example;
import 'package:flutter_test/flutter_test.dart';

void main() {
  testWidgets('SliverCrossAxisGroup example', (WidgetTester tester) async {
    await tester.pumpWidget(const example.SliverCrossAxisGroupExampleApp());

    final RenderSliverCrossAxisGroup renderSliverGroup = tester.renderObject(
      find.byType(SliverCrossAxisGroup),
    );
    expect(renderSliverGroup, isNotNull);

    final double crossAxisExtent = renderSliverGroup.constraints.crossAxisExtent;

    final List<RenderSliverList> renderSliverLists =
        tester.renderObjectList<RenderSliverList>(find.byType(SliverList)).toList();
    final RenderSliverList firstList = renderSliverLists[0];
    final RenderSliverList secondList = renderSliverLists[1];
    final RenderSliverList thirdList = renderSliverLists[2];

    final double expectedFirstExtent = (crossAxisExtent - 200) / 3;
    const double expectedSecondExtent = 200;
    final double expectedThirdExtent = 2 * (crossAxisExtent - 200) / 3;
    expect(firstList.constraints.crossAxisExtent, equals(expectedFirstExtent));
    expect(secondList.constraints.crossAxisExtent, equals(expectedSecondExtent));
    expect(thirdList.constraints.crossAxisExtent, equals(expectedThirdExtent));

    // Also check that the paint offsets are correct.
    final RenderSliverConstrainedCrossAxis renderConstrained = tester
        .renderObject<RenderSliverConstrainedCrossAxis>(find.byType(SliverConstrainedCrossAxis));

    expect((firstList.parentData! as SliverPhysicalParentData).paintOffset.dx, equals(0));
    expect(
      (renderConstrained.parentData! as SliverPhysicalParentData).paintOffset.dx,
      equals(expectedFirstExtent),
    );
    expect(
      (thirdList.parentData! as SliverPhysicalParentData).paintOffset.dx,
      equals(expectedFirstExtent + expectedSecondExtent),
    );
  });
}
