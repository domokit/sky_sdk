// Copyright 2014 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

import 'dart:math' as math;

import 'package:flutter/rendering.dart';
import 'package:flutter/widgets.dart';
import 'package:flutter_test/flutter_test.dart';

void verifyPaintPosition(GlobalKey key, Offset ideal) {
  final RenderObject target = key.currentContext!.findRenderObject()!;
  expect(target.parent, isA<RenderViewport>());
  final SliverPhysicalParentData parentData = target.parentData! as SliverPhysicalParentData;
  final Offset actual = parentData.paintOffset;
  expect(actual, ideal);
}

void main() {
  testWidgets('Sliver protocol', (WidgetTester tester) async {
    GlobalKey key1, key2, key3, key4, key5;
    await tester.pumpWidget(
      Directionality(
        textDirection: TextDirection.ltr,
        child: CustomScrollView(
          slivers: <Widget>[
            BigSliver(key: key1 = GlobalKey()),
            OverlappingSliver(key: key2 = GlobalKey()),
            OverlappingSliver(key: key3 = GlobalKey()),
            BigSliver(key: key4 = GlobalKey()),
            BigSliver(key: key5 = GlobalKey()),
          ],
        ),
      ),
    );
    final ScrollPosition position = tester.state<ScrollableState>(find.byType(Scrollable)).position;
    const double max =
        RenderBigSliver.height * 3.0 +
        (RenderOverlappingSliver.totalHeight) * 2.0 -
        600.0; // 600 is the height of the test viewport
    assert(max < 10000.0);
    expect(max, 1450.0);
    expect(position.pixels, 0.0);
    expect(position.minScrollExtent, 0.0);
    expect(position.maxScrollExtent, max);
    position.animateTo(10000.0, curve: Curves.linear, duration: const Duration(minutes: 1));
    await tester.pumpAndSettle(const Duration(milliseconds: 10));
    expect(position.pixels, max);
    expect(position.minScrollExtent, 0.0);
    expect(position.maxScrollExtent, max);
    verifyPaintPosition(key1, Offset.zero);
    verifyPaintPosition(key2, Offset.zero);
    verifyPaintPosition(key3, Offset.zero);
    verifyPaintPosition(key4, Offset.zero);
    verifyPaintPosition(key5, const Offset(0.0, 50.0));
  });
}

class RenderBigSliver extends RenderSliver {
  static const double height = 550.0;
  double get paintExtent =>
      (height - constraints.scrollOffset).clamp(0.0, constraints.remainingPaintExtent);

  @override
  void performLayout() {
    geometry = SliverGeometry(
      scrollExtent: height,
      paintExtent: paintExtent,
      maxPaintExtent: height,
    );
  }
}

class BigSliver extends LeafRenderObjectWidget {
  const BigSliver({super.key});
  @override
  RenderBigSliver createRenderObject(BuildContext context) {
    return RenderBigSliver();
  }
}

class RenderOverlappingSliver extends RenderSliver {
  static const double totalHeight = 200.0;
  static const double fixedHeight = 100.0;

  double get paintExtent {
    return math.min(
      math.max(fixedHeight, totalHeight - constraints.scrollOffset),
      constraints.remainingPaintExtent,
    );
  }

  double get layoutExtent {
    return (totalHeight - constraints.scrollOffset).clamp(0.0, constraints.remainingPaintExtent);
  }

  @override
  void performLayout() {
    geometry = SliverGeometry(
      scrollExtent: totalHeight,
      paintExtent: paintExtent,
      layoutExtent: layoutExtent,
      maxPaintExtent: totalHeight,
    );
  }
}

class OverlappingSliver extends LeafRenderObjectWidget {
  const OverlappingSliver({super.key});
  @override
  RenderOverlappingSliver createRenderObject(BuildContext context) {
    return RenderOverlappingSliver();
  }
}
