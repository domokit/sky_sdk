// Copyright 2014 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

// This file is separate from viewport_test.dart because we can't use both
// testWidgets and rendering_tester in the same file - testWidgets will
// initialize a binding, which rendering_tester will attempt to re-initialize
// (or vice versa).

import 'package:flutter/rendering.dart';
import 'package:flutter_test/flutter_test.dart';

import 'rendering_tester.dart';

void main() {
  TestRenderingFlutterBinding.ensureInitialized();

  const double width = 800;
  const double height = 600;
  Rect rectExpandedOnAxis(double value) => Rect.fromLTRB(0.0, 0.0 - value, width, height + value);
  late List<RenderSliver> children;

  setUp(() {
    children = <RenderSliver>[
      RenderSliverToBoxAdapter(child: RenderSizedBox(const Size(800, 400))),
    ];
  });

  test('Cache extent - null, pixels', () async {
    final RenderViewport renderViewport = RenderViewport(
      crossAxisDirection: AxisDirection.left,
      offset: ViewportOffset.zero(),
      children: children,
    );
    layout(renderViewport, phase: EnginePhase.flushSemantics);
    expect(
      renderViewport.describeSemanticsClip(null),
      rectExpandedOnAxis(RenderAbstractViewport.defaultCacheExtent),
    );
  });

  test('Cache extent - 0, pixels', () async {
    final RenderViewport renderViewport = RenderViewport(
      crossAxisDirection: AxisDirection.left,
      offset: ViewportOffset.zero(),
      cacheExtent: 0.0,
      children: children,
    );
    layout(renderViewport, phase: EnginePhase.flushSemantics);
    expect(renderViewport.describeSemanticsClip(null), rectExpandedOnAxis(0.0));
  });

  test('Cache extent - 500, pixels', () async {
    final RenderViewport renderViewport = RenderViewport(
      crossAxisDirection: AxisDirection.left,
      offset: ViewportOffset.zero(),
      cacheExtent: 500.0,
      children: children,
    );
    layout(renderViewport, phase: EnginePhase.flushSemantics);
    expect(renderViewport.describeSemanticsClip(null), rectExpandedOnAxis(500.0));
  });

  test('Cache extent - nullx viewport', () async {
    await expectLater(
      () => RenderViewport(
        crossAxisDirection: AxisDirection.left,
        offset: ViewportOffset.zero(),
        cacheExtentStyle: CacheExtentStyle.viewport,
        children: children,
      ),
      throwsAssertionError,
    );
  });

  test('Cache extent - 0x viewport', () async {
    final RenderViewport renderViewport = RenderViewport(
      crossAxisDirection: AxisDirection.left,
      offset: ViewportOffset.zero(),
      cacheExtent: 0.0,
      cacheExtentStyle: CacheExtentStyle.viewport,
      children: children,
    );

    layout(renderViewport);
    expect(renderViewport.describeSemanticsClip(null), rectExpandedOnAxis(0));
  });

  test('Cache extent - .5x viewport', () async {
    final RenderViewport renderViewport = RenderViewport(
      crossAxisDirection: AxisDirection.left,
      offset: ViewportOffset.zero(),
      cacheExtent: .5,
      cacheExtentStyle: CacheExtentStyle.viewport,
      children: children,
    );

    layout(renderViewport);
    expect(renderViewport.describeSemanticsClip(null), rectExpandedOnAxis(height / 2));
  });

  test('Cache extent - 1x viewport', () async {
    final RenderViewport renderViewport = RenderViewport(
      crossAxisDirection: AxisDirection.left,
      offset: ViewportOffset.zero(),
      cacheExtent: 1.0,
      cacheExtentStyle: CacheExtentStyle.viewport,
      children: children,
    );

    layout(renderViewport);
    expect(renderViewport.describeSemanticsClip(null), rectExpandedOnAxis(height));
  });

  test('Cache extent - 2.5x viewport', () async {
    final RenderViewport renderViewport = RenderViewport(
      crossAxisDirection: AxisDirection.left,
      offset: ViewportOffset.zero(),
      cacheExtent: 2.5,
      cacheExtentStyle: CacheExtentStyle.viewport,
      children: children,
    );

    layout(renderViewport);
    expect(renderViewport.describeSemanticsClip(null), rectExpandedOnAxis(height * 2.5));
  });

  test(
    'RenderShrinkWrappingViewport describeApproximatePaintClip with infinite viewportMainAxisExtent returns finite rect',
    () {
      final RenderSliver child = CustomConstraintsRenderSliver(
        const SliverConstraints(
          axisDirection: AxisDirection.down,
          cacheOrigin: 0.0,
          crossAxisDirection: AxisDirection.left,
          crossAxisExtent: 400.0,
          growthDirection: GrowthDirection.forward,
          overlap: 1.0, // must not equal 0 for this test
          precedingScrollExtent: 0.0,
          remainingPaintExtent: double.infinity,
          remainingCacheExtent: 0.0,
          scrollOffset: 0.0,
          userScrollDirection: ScrollDirection.idle,
          viewportMainAxisExtent: double.infinity, // must == infinity
        ),
      );

      final RenderShrinkWrappingViewport viewport = RenderShrinkWrappingViewport(
        crossAxisDirection: AxisDirection.left,
        offset: ViewportOffset.zero(),
        children: <RenderSliver>[child],
      );

      layout(viewport);
      expect(
        viewport.describeApproximatePaintClip(child),
        const Rect.fromLTRB(0.0, 0.0, 800.0, 600.0),
      );
    },
  );
}

class CustomConstraintsRenderSliver extends RenderSliver {
  CustomConstraintsRenderSliver(this.constraints);

  @override
  SliverGeometry get geometry => SliverGeometry.zero;

  @override
  final SliverConstraints constraints;

  @override
  void performLayout() {}
}
