// Copyright 2014 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

import 'package:flutter/rendering.dart';
import 'package:flutter/widgets.dart';
import 'package:flutter_test/flutter_test.dart';

import 'rendering_tester.dart';

void main() {
  TestRenderingFlutterBinding.ensureInitialized();

  test('RenderSliverOpacity does not composite if it is transparent', () {
    final RenderSliverOpacity renderSliverOpacity = RenderSliverOpacity(
      opacity: 0.0,
      sliver: RenderSliverToBoxAdapter(
        child: RenderSizedBox(const Size(1.0, 1.0)), // size doesn't matter
      ),
    );

    final RenderViewport root = RenderViewport(
      crossAxisDirection: AxisDirection.right,
      offset: ViewportOffset.zero(),
      cacheExtent: 250.0,
      children: <RenderSliver>[renderSliverOpacity],
    );

    layout(root, phase: EnginePhase.composite);
    expect(renderSliverOpacity.needsCompositing, false);
  });

  test('RenderSliverOpacity does composite if it is opaque', () {
    final RenderSliverOpacity renderSliverOpacity = RenderSliverOpacity(
      sliver: RenderSliverToBoxAdapter(
        child: RenderSizedBox(const Size(1.0, 1.0)), // size doesn't matter
      ),
    );

    final RenderViewport root = RenderViewport(
      crossAxisDirection: AxisDirection.right,
      offset: ViewportOffset.zero(),
      cacheExtent: 250.0,
      children: <RenderSliver>[renderSliverOpacity],
    );

    layout(root, phase: EnginePhase.composite);
    expect(renderSliverOpacity.needsCompositing, true);
  });

  test('RenderSliverOpacity reuses its layer', () {
    final RenderSliverOpacity renderSliverOpacity = RenderSliverOpacity(
      opacity: 0.5,
      sliver: RenderSliverToBoxAdapter(
        child: RenderSizedBox(const Size(1.0, 1.0)), // size doesn't matter
      ),
    );

    final RenderViewport root = RenderViewport(
      crossAxisDirection: AxisDirection.right,
      offset: ViewportOffset.zero(),
      cacheExtent: 250.0,
      children: <RenderSliver>[renderSliverOpacity],
    );

    expect(renderSliverOpacity.debugLayer, null);
    layout(root, phase: EnginePhase.paint, constraints: BoxConstraints.tight(const Size(10, 10)));
    final ContainerLayer layer = renderSliverOpacity.debugLayer!;
    expect(layer, isNotNull);

    // Mark for repaint otherwise pumpFrame is a noop.
    renderSliverOpacity.markNeedsPaint();
    expect(renderSliverOpacity.debugNeedsPaint, true);
    pumpFrame(phase: EnginePhase.paint);
    expect(renderSliverOpacity.debugNeedsPaint, false);
    expect(renderSliverOpacity.debugLayer, same(layer));
  });

  test('RenderSliverAnimatedOpacity does not composite if it is transparent', () async {
    final Animation<double> opacityAnimation = AnimationController(vsync: FakeTickerProvider())
      ..value = 0.0;

    final RenderSliverAnimatedOpacity renderSliverAnimatedOpacity = RenderSliverAnimatedOpacity(
      opacity: opacityAnimation,
      sliver: RenderSliverToBoxAdapter(
        child: RenderSizedBox(const Size(1.0, 1.0)), // size doesn't matter
      ),
    );

    final RenderViewport root = RenderViewport(
      crossAxisDirection: AxisDirection.right,
      offset: ViewportOffset.zero(),
      cacheExtent: 250.0,
      children: <RenderSliver>[renderSliverAnimatedOpacity],
    );

    layout(root, phase: EnginePhase.composite);
    expect(renderSliverAnimatedOpacity.needsCompositing, false);
  });

  test('RenderSliverAnimatedOpacity does composite if it is partially opaque', () {
    final Animation<double> opacityAnimation = AnimationController(vsync: FakeTickerProvider())
      ..value = 0.5;

    final RenderSliverAnimatedOpacity renderSliverAnimatedOpacity = RenderSliverAnimatedOpacity(
      opacity: opacityAnimation,
      sliver: RenderSliverToBoxAdapter(
        child: RenderSizedBox(const Size(1.0, 1.0)), // size doesn't matter
      ),
    );

    final RenderViewport root = RenderViewport(
      crossAxisDirection: AxisDirection.right,
      offset: ViewportOffset.zero(),
      cacheExtent: 250.0,
      children: <RenderSliver>[renderSliverAnimatedOpacity],
    );

    layout(root, phase: EnginePhase.composite);
    expect(renderSliverAnimatedOpacity.needsCompositing, true);
  });

  test('RenderSliverAnimatedOpacity does composite if it is opaque', () {
    final Animation<double> opacityAnimation = AnimationController(vsync: FakeTickerProvider())
      ..value = 1.0;

    final RenderSliverAnimatedOpacity renderSliverAnimatedOpacity = RenderSliverAnimatedOpacity(
      opacity: opacityAnimation,
      sliver: RenderSliverToBoxAdapter(
        child: RenderSizedBox(const Size(1.0, 1.0)), // size doesn't matter
      ),
    );

    final RenderViewport root = RenderViewport(
      crossAxisDirection: AxisDirection.right,
      offset: ViewportOffset.zero(),
      cacheExtent: 250.0,
      children: <RenderSliver>[renderSliverAnimatedOpacity],
    );

    layout(root, phase: EnginePhase.composite);
    expect(renderSliverAnimatedOpacity.needsCompositing, true);
  });

  test('RenderSliverAnimatedOpacity reuses its layer', () {
    final Animation<double> opacityAnimation = AnimationController(vsync: FakeTickerProvider())
      ..value = 0.5; // must not be 0 or 1.0. Otherwise, it won't create a layer

    final RenderSliverAnimatedOpacity renderSliverAnimatedOpacity = RenderSliverAnimatedOpacity(
      opacity: opacityAnimation,
      sliver: RenderSliverToBoxAdapter(
        child: RenderSizedBox(const Size(1.0, 1.0)), // size doesn't matter
      ),
    );

    final RenderViewport root = RenderViewport(
      crossAxisDirection: AxisDirection.right,
      offset: ViewportOffset.zero(),
      cacheExtent: 250.0,
      children: <RenderSliver>[renderSliverAnimatedOpacity],
    );

    expect(renderSliverAnimatedOpacity.debugLayer, null);
    layout(root, phase: EnginePhase.paint, constraints: BoxConstraints.tight(const Size(10, 10)));
    final ContainerLayer layer = renderSliverAnimatedOpacity.debugLayer!;
    expect(layer, isNotNull);

    // Mark for repaint otherwise pumpFrame is a noop.
    renderSliverAnimatedOpacity.markNeedsPaint();
    expect(renderSliverAnimatedOpacity.debugNeedsPaint, true);
    pumpFrame(phase: EnginePhase.paint);
    expect(renderSliverAnimatedOpacity.debugNeedsPaint, false);
    expect(renderSliverAnimatedOpacity.debugLayer, same(layer));
  });
}
