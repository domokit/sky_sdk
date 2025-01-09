// Copyright 2014 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

import 'package:flutter/rendering.dart';
import 'package:flutter_test/flutter_test.dart';

void main() {
  test('applyGrowthDirectionToAxisDirection produces expected AxisDirection', () {
    expect(AxisDirection.values.length, 4);
    for (final AxisDirection axisDirection in AxisDirection.values) {
      expect(
        applyGrowthDirectionToAxisDirection(axisDirection, GrowthDirection.forward),
        axisDirection,
      );
    }
    expect(
      applyGrowthDirectionToAxisDirection(AxisDirection.up, GrowthDirection.reverse),
      AxisDirection.down,
    );
    expect(
      applyGrowthDirectionToAxisDirection(AxisDirection.down, GrowthDirection.reverse),
      AxisDirection.up,
    );
    expect(
      applyGrowthDirectionToAxisDirection(AxisDirection.left, GrowthDirection.reverse),
      AxisDirection.right,
    );
    expect(
      applyGrowthDirectionToAxisDirection(AxisDirection.right, GrowthDirection.reverse),
      AxisDirection.left,
    );
  });

  test('SliverConstraints are the same when copied', () {
    const SliverConstraints original = SliverConstraints(
      axisDirection: AxisDirection.down,
      growthDirection: GrowthDirection.forward,
      userScrollDirection: ScrollDirection.idle,
      scrollOffset: 0.0,
      precedingScrollExtent: 0.0,
      overlap: 0.0,
      remainingPaintExtent: 0.0,
      crossAxisExtent: 0.0,
      crossAxisDirection: AxisDirection.right,
      viewportMainAxisExtent: 0.0,
      cacheOrigin: 0.0,
      remainingCacheExtent: 0.0,
    );
    final SliverConstraints copy = original.copyWith();
    expect(original, equals(copy));
    expect(original.hashCode, equals(copy.hashCode));
    expect(original.toString(), equals(copy.toString()));
    expect(original, hasOneLineDescription);
    expect(original.normalizedGrowthDirection, equals(GrowthDirection.forward));
  });

  test(
    'SliverConstraints normalizedGrowthDirection is inferred from AxisDirection and GrowthDirection',
    () {
      const SliverConstraints a = SliverConstraints(
        axisDirection: AxisDirection.down,
        growthDirection: GrowthDirection.forward,
        userScrollDirection: ScrollDirection.idle,
        scrollOffset: 0.0,
        precedingScrollExtent: 0.0,
        overlap: 0.0,
        remainingPaintExtent: 0.0,
        crossAxisExtent: 0.0,
        crossAxisDirection: AxisDirection.right,
        viewportMainAxisExtent: 0.0,
        cacheOrigin: 0.0,
        remainingCacheExtent: 0.0,
      );

      final SliverConstraints c = a.copyWith(
        axisDirection: AxisDirection.up,
        growthDirection: GrowthDirection.reverse,
        userScrollDirection: ScrollDirection.forward,
        scrollOffset: 10.0,
        overlap: 20.0,
        remainingPaintExtent: 30.0,
        crossAxisExtent: 40.0,
        viewportMainAxisExtent: 30.0,
      );
      const SliverConstraints d = SliverConstraints(
        axisDirection: AxisDirection.up,
        growthDirection: GrowthDirection.reverse,
        userScrollDirection: ScrollDirection.forward,
        scrollOffset: 10.0,
        precedingScrollExtent: 0.0,
        overlap: 20.0,
        remainingPaintExtent: 30.0,
        crossAxisExtent: 40.0,
        crossAxisDirection: AxisDirection.right,
        viewportMainAxisExtent: 30.0,
        cacheOrigin: 0.0,
        remainingCacheExtent: 0.0,
      );
      expect(c, equals(d));
      expect(c.normalizedGrowthDirection, equals(GrowthDirection.forward));
      expect(d.normalizedGrowthDirection, equals(GrowthDirection.forward));

      final SliverConstraints e = d.copyWith(axisDirection: AxisDirection.right);
      expect(e.normalizedGrowthDirection, equals(GrowthDirection.reverse));

      final SliverConstraints f = d.copyWith(axisDirection: AxisDirection.left);
      expect(f.normalizedGrowthDirection, equals(GrowthDirection.forward));

      final SliverConstraints g = d.copyWith(growthDirection: GrowthDirection.forward);
      expect(g.normalizedGrowthDirection, equals(GrowthDirection.reverse));
    },
  );

  test('SliverGeometry with no arguments is valid', () {
    expect(SliverGeometry.zero.debugAssertIsValid(), isTrue);
  });

  test('SliverGeometry throws error when layoutExtent exceeds paintExtent', () {
    expect(() {
      const SliverGeometry(layoutExtent: 10.0, paintExtent: 9.0).debugAssertIsValid();
    }, throwsFlutterError);
  });

  test('SliverGeometry throws error when maxPaintExtent is less than paintExtent', () {
    expect(() {
      const SliverGeometry(paintExtent: 9.0, maxPaintExtent: 8.0).debugAssertIsValid();
    }, throwsFlutterError);
  });
}
