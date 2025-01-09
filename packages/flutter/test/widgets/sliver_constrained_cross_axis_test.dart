// Copyright 2014 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

import 'package:flutter/material.dart';
import 'package:flutter/rendering.dart';
import 'package:flutter_test/flutter_test.dart';

const double VIEWPORT_HEIGHT = 500;
const double VIEWPORT_WIDTH = 300;

void main() {
  testWidgets('SliverConstrainedCrossAxis basic test', (WidgetTester tester) async {
    await tester.pumpWidget(_buildSliverConstrainedCrossAxis(maxExtent: 50));

    final RenderBox box = tester.renderObject(find.byType(Container));
    expect(box.size.height, 100);
    expect(box.size.width, 50);

    final RenderSliver sliver = tester.renderObject(find.byType(SliverToBoxAdapter));
    expect(sliver.geometry!.paintExtent, equals(100));
  });

  testWidgets('SliverConstrainedCrossAxis updates correctly', (WidgetTester tester) async {
    await tester.pumpWidget(_buildSliverConstrainedCrossAxis(maxExtent: 50));

    final RenderBox box1 = tester.renderObject(find.byType(Container));
    expect(box1.size.height, 100);
    expect(box1.size.width, 50);

    await tester.pumpWidget(_buildSliverConstrainedCrossAxis(maxExtent: 80));

    final RenderBox box2 = tester.renderObject(find.byType(Container));
    expect(box2.size.height, 100);
    expect(box2.size.width, 80);
  });

  testWidgets('SliverConstrainedCrossAxis uses parent extent if maxExtent is greater', (
    WidgetTester tester,
  ) async {
    await tester.pumpWidget(_buildSliverConstrainedCrossAxis(maxExtent: 400));

    final RenderBox box = tester.renderObject(find.byType(Container));
    expect(box.size.height, 100);
    expect(box.size.width, VIEWPORT_WIDTH);
  });

  testWidgets('SliverConstrainedCrossAxis constrains the height when direction is horizontal', (
    WidgetTester tester,
  ) async {
    await tester.pumpWidget(
      _buildSliverConstrainedCrossAxis(maxExtent: 50, scrollDirection: Axis.horizontal),
    );

    final RenderBox box = tester.renderObject(find.byType(Container));
    expect(box.size.height, 50);
  });

  testWidgets('SliverConstrainedCrossAxis sets its own flex to 0', (WidgetTester tester) async {
    await tester.pumpWidget(_buildSliverConstrainedCrossAxis(maxExtent: 50));

    final RenderSliver sliver = tester.renderObject(find.byType(SliverConstrainedCrossAxis));
    expect((sliver.parentData! as SliverPhysicalParentData).crossAxisFlex, equals(0));
  });
}

Widget _buildSliverConstrainedCrossAxis({
  required double maxExtent,
  Axis scrollDirection = Axis.vertical,
}) {
  return Directionality(
    textDirection: TextDirection.ltr,
    child: Center(
      child: SizedBox(
        width: VIEWPORT_WIDTH,
        height: VIEWPORT_HEIGHT,
        child: CustomScrollView(
          scrollDirection: scrollDirection,
          slivers: <Widget>[
            SliverConstrainedCrossAxis(
              maxExtent: maxExtent,
              sliver: SliverToBoxAdapter(
                child:
                    scrollDirection == Axis.vertical
                        ? Container(height: 100)
                        : Container(width: 100),
              ),
            ),
          ],
        ),
      ),
    ),
  );
}
