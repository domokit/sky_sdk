// Copyright 2014 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

import 'package:flutter/gestures.dart';
import 'package:flutter/rendering.dart';
import 'package:flutter_test/flutter_test.dart';
import 'package:vector_math/vector_math_64.dart';

import 'rendering_tester.dart';

void main() {
  TestRenderingFlutterBinding.ensureInitialized();

  test('RenderViewport basic test - no children', () {
    final RenderViewport root = RenderViewport(
      crossAxisDirection: AxisDirection.right,
      offset: ViewportOffset.zero(),
    );
    expect(root, hasAGoodToStringDeep);
    expect(
      root.toStringDeep(minLevel: DiagnosticLevel.info),
      equalsIgnoringHashCodes(
        'RenderViewport#00000 NEEDS-LAYOUT NEEDS-PAINT DETACHED\n'
        '   needs compositing\n'
        '   parentData: MISSING\n'
        '   constraints: MISSING\n'
        '   size: MISSING\n'
        '   axisDirection: down\n'
        '   crossAxisDirection: right\n'
        '   offset: _FixedViewportOffset#00000(offset: 0.0)\n'
        '   anchor: 0.0\n',
      ),
    );
    layout(root);
    root.offset = ViewportOffset.fixed(900.0);
    expect(root, hasAGoodToStringDeep);
    expect(
      root.toStringDeep(minLevel: DiagnosticLevel.info),
      equalsIgnoringHashCodes(
        'RenderViewport#00000 NEEDS-LAYOUT NEEDS-PAINT\n'
        '   needs compositing\n'
        '   parentData: <none>\n'
        '   constraints: BoxConstraints(w=800.0, h=600.0)\n'
        '   size: Size(800.0, 600.0)\n'
        '   axisDirection: down\n'
        '   crossAxisDirection: right\n'
        '   offset: _FixedViewportOffset#00000(offset: 900.0)\n'
        '   anchor: 0.0\n',
      ),
    );

    pumpFrame();
  });

  test('RenderViewport basic test - down', () {
    RenderBox a, b, c, d, e;
    final RenderViewport root = RenderViewport(
      crossAxisDirection: AxisDirection.right,
      offset: ViewportOffset.zero(),
      children: <RenderSliver>[
        RenderSliverToBoxAdapter(child: a = RenderSizedBox(const Size(100.0, 400.0))),
        RenderSliverToBoxAdapter(child: b = RenderSizedBox(const Size(100.0, 400.0))),
        RenderSliverToBoxAdapter(child: c = RenderSizedBox(const Size(100.0, 400.0))),
        RenderSliverToBoxAdapter(child: d = RenderSizedBox(const Size(100.0, 400.0))),
        RenderSliverToBoxAdapter(child: e = RenderSizedBox(const Size(100.0, 400.0))),
      ],
    );
    expect(root, hasAGoodToStringDeep);
    layout(root);

    expect(root.size.width, equals(800.0));
    expect(root.size.height, equals(600.0));

    expect(root, hasAGoodToStringDeep);
    expect(
      root.toStringDeep(minLevel: DiagnosticLevel.info),
      equalsIgnoringHashCodes(
        'RenderViewport#00000 NEEDS-PAINT NEEDS-COMPOSITING-BITS-UPDATE\n'
        ' │ needs compositing\n'
        ' │ parentData: <none>\n'
        ' │ constraints: BoxConstraints(w=800.0, h=600.0)\n'
        ' │ size: Size(800.0, 600.0)\n'
        ' │ axisDirection: down\n'
        ' │ crossAxisDirection: right\n'
        ' │ offset: _FixedViewportOffset#00000(offset: 0.0)\n'
        ' │ anchor: 0.0\n'
        ' │\n'
        ' ├─center child: RenderSliverToBoxAdapter#00000 relayoutBoundary=up1 NEEDS-PAINT NEEDS-COMPOSITING-BITS-UPDATE\n'
        ' │ │ parentData: paintOffset=Offset(0.0, 0.0) (can use size)\n'
        ' │ │ constraints: SliverConstraints(AxisDirection.down,\n'
        ' │ │   GrowthDirection.forward, ScrollDirection.idle, scrollOffset:\n'
        ' │ │   0.0, precedingScrollExtent: 0.0, remainingPaintExtent: 600.0,\n'
        ' │ │   crossAxisExtent: 800.0, crossAxisDirection:\n'
        ' │ │   AxisDirection.right, viewportMainAxisExtent: 600.0,\n'
        ' │ │   remainingCacheExtent: 850.0, cacheOrigin: 0.0)\n'
        ' │ │ geometry: SliverGeometry(scrollExtent: 400.0, paintExtent: 400.0,\n'
        ' │ │   maxPaintExtent: 400.0, cacheExtent: 400.0)\n'
        ' │ │\n'
        ' │ └─child: RenderSizedBox#00000 NEEDS-PAINT\n'
        ' │     parentData: paintOffset=Offset(0.0, -0.0) (can use size)\n'
        ' │     constraints: BoxConstraints(w=800.0, 0.0<=h<=Infinity)\n'
        ' │     size: Size(800.0, 400.0)\n'
        ' │\n'
        ' ├─child 1: RenderSliverToBoxAdapter#00000 relayoutBoundary=up1 NEEDS-PAINT NEEDS-COMPOSITING-BITS-UPDATE\n'
        ' │ │ parentData: paintOffset=Offset(0.0, 400.0) (can use size)\n'
        ' │ │ constraints: SliverConstraints(AxisDirection.down,\n'
        ' │ │   GrowthDirection.forward, ScrollDirection.idle, scrollOffset:\n'
        ' │ │   0.0, precedingScrollExtent: 400.0, remainingPaintExtent: 200.0,\n'
        ' │ │   crossAxisExtent: 800.0, crossAxisDirection:\n'
        ' │ │   AxisDirection.right, viewportMainAxisExtent: 600.0,\n'
        ' │ │   remainingCacheExtent: 450.0, cacheOrigin: 0.0)\n'
        ' │ │ geometry: SliverGeometry(scrollExtent: 400.0, paintExtent: 200.0,\n'
        ' │ │   maxPaintExtent: 400.0, hasVisualOverflow: true, cacheExtent:\n'
        ' │ │   400.0)\n'
        ' │ │\n'
        ' │ └─child: RenderSizedBox#00000 NEEDS-PAINT\n'
        ' │     parentData: paintOffset=Offset(0.0, -0.0) (can use size)\n'
        ' │     constraints: BoxConstraints(w=800.0, 0.0<=h<=Infinity)\n'
        ' │     size: Size(800.0, 400.0)\n'
        ' │\n'
        ' ├─child 2: RenderSliverToBoxAdapter#00000 relayoutBoundary=up1 NEEDS-PAINT NEEDS-COMPOSITING-BITS-UPDATE\n'
        ' │ │ parentData: paintOffset=Offset(0.0, 800.0) (can use size)\n'
        ' │ │ constraints: SliverConstraints(AxisDirection.down,\n'
        ' │ │   GrowthDirection.forward, ScrollDirection.idle, scrollOffset:\n'
        ' │ │   0.0, precedingScrollExtent: 800.0, remainingPaintExtent: 0.0,\n'
        ' │ │   crossAxisExtent: 800.0, crossAxisDirection:\n'
        ' │ │   AxisDirection.right, viewportMainAxisExtent: 600.0,\n'
        ' │ │   remainingCacheExtent: 50.0, cacheOrigin: 0.0)\n'
        ' │ │ geometry: SliverGeometry(scrollExtent: 400.0, hidden,\n'
        ' │ │   maxPaintExtent: 400.0, hasVisualOverflow: true, cacheExtent:\n'
        ' │ │   50.0)\n'
        ' │ │\n'
        ' │ └─child: RenderSizedBox#00000 NEEDS-PAINT\n'
        ' │     parentData: paintOffset=Offset(0.0, -0.0) (can use size)\n'
        ' │     constraints: BoxConstraints(w=800.0, 0.0<=h<=Infinity)\n'
        ' │     size: Size(800.0, 400.0)\n'
        ' │\n'
        ' ├─child 3: RenderSliverToBoxAdapter#00000 relayoutBoundary=up1 NEEDS-PAINT NEEDS-COMPOSITING-BITS-UPDATE\n'
        ' │ │ parentData: paintOffset=Offset(0.0, 1200.0) (can use size)\n'
        ' │ │ constraints: SliverConstraints(AxisDirection.down,\n'
        ' │ │   GrowthDirection.forward, ScrollDirection.idle, scrollOffset:\n'
        ' │ │   0.0, precedingScrollExtent: 1200.0, remainingPaintExtent: 0.0,\n'
        ' │ │   crossAxisExtent: 800.0, crossAxisDirection:\n'
        ' │ │   AxisDirection.right, viewportMainAxisExtent: 600.0,\n'
        ' │ │   remainingCacheExtent: 0.0, cacheOrigin: 0.0)\n'
        ' │ │ geometry: SliverGeometry(scrollExtent: 400.0, hidden,\n'
        ' │ │   maxPaintExtent: 400.0, hasVisualOverflow: true)\n'
        ' │ │\n'
        ' │ └─child: RenderSizedBox#00000 NEEDS-PAINT\n'
        ' │     parentData: paintOffset=Offset(0.0, -0.0) (can use size)\n'
        ' │     constraints: BoxConstraints(w=800.0, 0.0<=h<=Infinity)\n'
        ' │     size: Size(800.0, 400.0)\n'
        ' │\n'
        ' └─child 4: RenderSliverToBoxAdapter#00000 relayoutBoundary=up1 NEEDS-PAINT NEEDS-COMPOSITING-BITS-UPDATE\n'
        '   │ parentData: paintOffset=Offset(0.0, 1600.0) (can use size)\n'
        '   │ constraints: SliverConstraints(AxisDirection.down,\n'
        '   │   GrowthDirection.forward, ScrollDirection.idle, scrollOffset:\n'
        '   │   0.0, precedingScrollExtent: 1600.0, remainingPaintExtent: 0.0,\n'
        '   │   crossAxisExtent: 800.0, crossAxisDirection:\n'
        '   │   AxisDirection.right, viewportMainAxisExtent: 600.0,\n'
        '   │   remainingCacheExtent: 0.0, cacheOrigin: 0.0)\n'
        '   │ geometry: SliverGeometry(scrollExtent: 400.0, hidden,\n'
        '   │   maxPaintExtent: 400.0, hasVisualOverflow: true)\n'
        '   │\n'
        '   └─child: RenderSizedBox#00000 NEEDS-PAINT\n'
        '       parentData: paintOffset=Offset(0.0, -0.0) (can use size)\n'
        '       constraints: BoxConstraints(w=800.0, 0.0<=h<=Infinity)\n'
        '       size: Size(800.0, 400.0)\n',
      ),
    );
    expect(a.localToGlobal(Offset.zero), Offset.zero);
    expect(b.localToGlobal(Offset.zero), const Offset(0.0, 400.0));
    expect(c.localToGlobal(Offset.zero), const Offset(0.0, 800.0));
    expect(d.localToGlobal(Offset.zero), const Offset(0.0, 1200.0));
    expect(e.localToGlobal(Offset.zero), const Offset(0.0, 1600.0));

    expect(a.localToGlobal(const Offset(800.0, 400.0)), const Offset(800.0, 400.0));
    expect(b.localToGlobal(const Offset(800.0, 400.0)), const Offset(800.0, 800.0));
    expect(c.localToGlobal(const Offset(800.0, 400.0)), const Offset(800.0, 1200.0));
    expect(d.localToGlobal(const Offset(800.0, 400.0)), const Offset(800.0, 1600.0));
    expect(e.localToGlobal(const Offset(800.0, 400.0)), const Offset(800.0, 2000.0));

    root.offset = ViewportOffset.fixed(200.0);
    pumpFrame();
    expect(a.localToGlobal(Offset.zero), const Offset(0.0, -200.0));
    expect(b.localToGlobal(Offset.zero), const Offset(0.0, 200.0));
    expect(c.localToGlobal(Offset.zero), const Offset(0.0, 600.0));
    expect(d.localToGlobal(Offset.zero), const Offset(0.0, 1000.0));
    expect(e.localToGlobal(Offset.zero), const Offset(0.0, 1400.0));

    root.offset = ViewportOffset.fixed(600.0);
    pumpFrame();
    expect(a.localToGlobal(Offset.zero), const Offset(0.0, -600.0));
    expect(b.localToGlobal(Offset.zero), const Offset(0.0, -200.0));
    expect(c.localToGlobal(Offset.zero), const Offset(0.0, 200.0));
    expect(d.localToGlobal(Offset.zero), const Offset(0.0, 600.0));
    expect(e.localToGlobal(Offset.zero), const Offset(0.0, 1000.0));

    root.offset = ViewportOffset.fixed(900.0);
    pumpFrame();
    expect(a.localToGlobal(Offset.zero), const Offset(0.0, -900.0));
    expect(b.localToGlobal(Offset.zero), const Offset(0.0, -500.0));
    expect(c.localToGlobal(Offset.zero), const Offset(0.0, -100.0));
    expect(d.localToGlobal(Offset.zero), const Offset(0.0, 300.0));
    expect(e.localToGlobal(Offset.zero), const Offset(0.0, 700.0));

    final BoxHitTestResult result = BoxHitTestResult();
    root.hitTest(result, position: const Offset(130.0, 150.0));
    expect(result.path.first.target, equals(c));
  });

  test('RenderViewport basic test - up', () {
    RenderBox a, b, c, d, e;
    final RenderViewport root = RenderViewport(
      axisDirection: AxisDirection.up,
      crossAxisDirection: AxisDirection.right,
      offset: ViewportOffset.zero(),
      children: <RenderSliver>[
        RenderSliverToBoxAdapter(child: a = RenderSizedBox(const Size(100.0, 400.0))),
        RenderSliverToBoxAdapter(child: b = RenderSizedBox(const Size(100.0, 400.0))),
        RenderSliverToBoxAdapter(child: c = RenderSizedBox(const Size(100.0, 400.0))),
        RenderSliverToBoxAdapter(child: d = RenderSizedBox(const Size(100.0, 400.0))),
        RenderSliverToBoxAdapter(child: e = RenderSizedBox(const Size(100.0, 400.0))),
      ],
    );
    layout(root);

    expect(root.size.width, equals(800.0));
    expect(root.size.height, equals(600.0));

    expect(a.localToGlobal(Offset.zero), const Offset(0.0, 200.0));
    expect(b.localToGlobal(Offset.zero), const Offset(0.0, -200.0));
    expect(c.localToGlobal(Offset.zero), const Offset(0.0, -600.0));
    expect(d.localToGlobal(Offset.zero), const Offset(0.0, -1000.0));
    expect(e.localToGlobal(Offset.zero), const Offset(0.0, -1400.0));

    root.offset = ViewportOffset.fixed(200.0);
    pumpFrame();
    expect(a.localToGlobal(Offset.zero), const Offset(0.0, 400.0));
    expect(b.localToGlobal(Offset.zero), Offset.zero);
    expect(c.localToGlobal(Offset.zero), const Offset(0.0, -400.0));
    expect(d.localToGlobal(Offset.zero), const Offset(0.0, -800.0));
    expect(e.localToGlobal(Offset.zero), const Offset(0.0, -1200.0));

    root.offset = ViewportOffset.fixed(600.0);
    pumpFrame();
    expect(a.localToGlobal(Offset.zero), const Offset(0.0, 800.0));
    expect(b.localToGlobal(Offset.zero), const Offset(0.0, 400.0));
    expect(c.localToGlobal(Offset.zero), Offset.zero);
    expect(d.localToGlobal(Offset.zero), const Offset(0.0, -400.0));
    expect(e.localToGlobal(Offset.zero), const Offset(0.0, -800.0));

    root.offset = ViewportOffset.fixed(900.0);
    pumpFrame();
    expect(a.localToGlobal(Offset.zero), const Offset(0.0, 1100.0));
    expect(b.localToGlobal(Offset.zero), const Offset(0.0, 700.0));
    expect(c.localToGlobal(Offset.zero), const Offset(0.0, 300.0));
    expect(d.localToGlobal(Offset.zero), const Offset(0.0, -100.0));
    expect(e.localToGlobal(Offset.zero), const Offset(0.0, -500.0));

    final BoxHitTestResult result = BoxHitTestResult();
    root.hitTest(result, position: const Offset(150.0, 350.0));
    expect(result.path.first.target, equals(c));
  });

  Offset getPaintOrigin(RenderObject render) {
    final Vector3 transformed3 = render
        .getTransformTo(null)
        .perspectiveTransform(Vector3(0.0, 0.0, 0.0));
    return Offset(transformed3.x, transformed3.y);
  }

  test('RenderViewport basic test - right', () {
    RenderBox a, b, c, d, e;
    final RenderViewport root = RenderViewport(
      axisDirection: AxisDirection.right,
      crossAxisDirection: AxisDirection.down,
      offset: ViewportOffset.zero(),
      children: <RenderSliver>[
        RenderSliverToBoxAdapter(child: a = RenderSizedBox(const Size(400.0, 100.0))),
        RenderSliverToBoxAdapter(child: b = RenderSizedBox(const Size(400.0, 100.0))),
        RenderSliverToBoxAdapter(child: c = RenderSizedBox(const Size(400.0, 100.0))),
        RenderSliverToBoxAdapter(child: d = RenderSizedBox(const Size(400.0, 100.0))),
        RenderSliverToBoxAdapter(child: e = RenderSizedBox(const Size(400.0, 100.0))),
      ],
    );
    layout(root);

    expect(root.size.width, equals(800.0));
    expect(root.size.height, equals(600.0));

    final RenderSliver sliverA = a.parent! as RenderSliver;
    final RenderSliver sliverB = b.parent! as RenderSliver;
    final RenderSliver sliverC = c.parent! as RenderSliver;
    final RenderSliver sliverD = d.parent! as RenderSliver;
    final RenderSliver sliverE = e.parent! as RenderSliver;

    expect(a.localToGlobal(Offset.zero), Offset.zero);
    expect(b.localToGlobal(Offset.zero), const Offset(400.0, 0.0));
    expect(c.localToGlobal(Offset.zero), const Offset(800.0, 0.0));
    expect(d.localToGlobal(Offset.zero), const Offset(1200.0, 0.0));
    expect(e.localToGlobal(Offset.zero), const Offset(1600.0, 0.0));

    expect(getPaintOrigin(sliverA), Offset.zero);
    expect(getPaintOrigin(sliverB), const Offset(400.0, 0.0));
    expect(getPaintOrigin(sliverC), const Offset(800.0, 0.0));
    expect(getPaintOrigin(sliverD), const Offset(1200.0, 0.0));
    expect(getPaintOrigin(sliverE), const Offset(1600.0, 0.0));

    root.offset = ViewportOffset.fixed(200.0);
    pumpFrame();
    expect(a.localToGlobal(Offset.zero), const Offset(-200.0, 0.0));
    expect(b.localToGlobal(Offset.zero), const Offset(200.0, 0.0));
    expect(c.localToGlobal(Offset.zero), const Offset(600.0, 0.0));
    expect(d.localToGlobal(Offset.zero), const Offset(1000.0, 0.0));
    expect(e.localToGlobal(Offset.zero), const Offset(1400.0, 0.0));

    expect(getPaintOrigin(sliverA), Offset.zero);
    expect(getPaintOrigin(sliverB), const Offset(200.0, 0.0));
    expect(getPaintOrigin(sliverC), const Offset(600.0, 0.0));
    expect(getPaintOrigin(sliverD), const Offset(1000.0, 0.0));
    expect(getPaintOrigin(sliverE), const Offset(1400.0, 0.0));

    root.offset = ViewportOffset.fixed(600.0);
    pumpFrame();
    expect(a.localToGlobal(Offset.zero), const Offset(-600.0, 0.0));
    expect(b.localToGlobal(Offset.zero), const Offset(-200.0, 0.0));
    expect(c.localToGlobal(Offset.zero), const Offset(200.0, 0.0));
    expect(d.localToGlobal(Offset.zero), const Offset(600.0, 0.0));
    expect(e.localToGlobal(Offset.zero), const Offset(1000.0, 0.0));

    expect(getPaintOrigin(sliverA), Offset.zero);
    expect(getPaintOrigin(sliverB), Offset.zero);
    expect(getPaintOrigin(sliverC), const Offset(200.0, 0.0));
    expect(getPaintOrigin(sliverD), const Offset(600.0, 0.0));
    expect(getPaintOrigin(sliverE), const Offset(1000.0, 0.0));

    root.offset = ViewportOffset.fixed(900.0);
    pumpFrame();
    expect(a.localToGlobal(Offset.zero), const Offset(-900.0, 0.0));
    expect(b.localToGlobal(Offset.zero), const Offset(-500.0, 0.0));
    expect(c.localToGlobal(Offset.zero), const Offset(-100.0, 0.0));
    expect(d.localToGlobal(Offset.zero), const Offset(300.0, 0.0));
    expect(e.localToGlobal(Offset.zero), const Offset(700.0, 0.0));

    expect(getPaintOrigin(sliverA), Offset.zero);
    expect(getPaintOrigin(sliverB), Offset.zero);
    expect(getPaintOrigin(sliverC), Offset.zero);
    expect(getPaintOrigin(sliverD), const Offset(300.0, 0.0));
    expect(getPaintOrigin(sliverE), const Offset(700.0, 0.0));

    final BoxHitTestResult result = BoxHitTestResult();
    root.hitTest(result, position: const Offset(150.0, 450.0));
    expect(result.path.first.target, equals(c));
  });

  test('RenderViewport basic test - left', () {
    RenderBox a, b, c, d, e;
    final RenderViewport root = RenderViewport(
      axisDirection: AxisDirection.left,
      crossAxisDirection: AxisDirection.down,
      offset: ViewportOffset.zero(),
      children: <RenderSliver>[
        RenderSliverToBoxAdapter(child: a = RenderSizedBox(const Size(400.0, 100.0))),
        RenderSliverToBoxAdapter(child: b = RenderSizedBox(const Size(400.0, 100.0))),
        RenderSliverToBoxAdapter(child: c = RenderSizedBox(const Size(400.0, 100.0))),
        RenderSliverToBoxAdapter(child: d = RenderSizedBox(const Size(400.0, 100.0))),
        RenderSliverToBoxAdapter(child: e = RenderSizedBox(const Size(400.0, 100.0))),
      ],
    );
    layout(root);

    expect(root.size.width, equals(800.0));
    expect(root.size.height, equals(600.0));

    expect(a.localToGlobal(Offset.zero), const Offset(400.0, 0.0));
    expect(b.localToGlobal(Offset.zero), Offset.zero);
    expect(c.localToGlobal(Offset.zero), const Offset(-400.0, 0.0));
    expect(d.localToGlobal(Offset.zero), const Offset(-800.0, 0.0));
    expect(e.localToGlobal(Offset.zero), const Offset(-1200.0, 0.0));

    root.offset = ViewportOffset.fixed(200.0);
    pumpFrame();
    expect(a.localToGlobal(Offset.zero), const Offset(600.0, 0.0));
    expect(b.localToGlobal(Offset.zero), const Offset(200.0, 0.0));
    expect(c.localToGlobal(Offset.zero), const Offset(-200.0, 0.0));
    expect(d.localToGlobal(Offset.zero), const Offset(-600.0, 0.0));
    expect(e.localToGlobal(Offset.zero), const Offset(-1000.0, 0.0));

    root.offset = ViewportOffset.fixed(600.0);
    pumpFrame();
    expect(a.localToGlobal(Offset.zero), const Offset(1000.0, 0.0));
    expect(b.localToGlobal(Offset.zero), const Offset(600.0, 0.0));
    expect(c.localToGlobal(Offset.zero), const Offset(200.0, 0.0));
    expect(d.localToGlobal(Offset.zero), const Offset(-200.0, 0.0));
    expect(e.localToGlobal(Offset.zero), const Offset(-600.0, 0.0));

    root.offset = ViewportOffset.fixed(900.0);
    pumpFrame();
    expect(a.localToGlobal(Offset.zero), const Offset(1300.0, 0.0));
    expect(b.localToGlobal(Offset.zero), const Offset(900.0, 0.0));
    expect(c.localToGlobal(Offset.zero), const Offset(500.0, 0.0));
    expect(d.localToGlobal(Offset.zero), const Offset(100.0, 0.0));
    expect(e.localToGlobal(Offset.zero), const Offset(-300.0, 0.0));

    final BoxHitTestResult result = BoxHitTestResult();
    root.hitTest(result, position: const Offset(550.0, 150.0));
    expect(result.path.first.target, equals(c));
  });

  // TODO(ianh): test anchor
  // TODO(ianh): test center
  // TODO(ianh): test semantics

  test('RenderShrinkWrappingViewport basic test - no children', () {
    final RenderShrinkWrappingViewport root = RenderShrinkWrappingViewport(
      crossAxisDirection: AxisDirection.right,
      offset: ViewportOffset.zero(),
    );
    expect(root, hasAGoodToStringDeep);
    layout(root);
    root.offset = ViewportOffset.fixed(900.0);
    pumpFrame();
  });

  test('RenderShrinkWrappingViewport basic test - down', () {
    RenderBox a, b, c, d, e;
    final RenderShrinkWrappingViewport root = RenderShrinkWrappingViewport(
      crossAxisDirection: AxisDirection.right,
      offset: ViewportOffset.zero(),
      children: <RenderSliver>[
        RenderSliverToBoxAdapter(child: a = RenderSizedBox(const Size(100.0, 400.0))),
        RenderSliverToBoxAdapter(child: b = RenderSizedBox(const Size(100.0, 400.0))),
        RenderSliverToBoxAdapter(child: c = RenderSizedBox(const Size(100.0, 400.0))),
        RenderSliverToBoxAdapter(child: d = RenderSizedBox(const Size(100.0, 400.0))),
        RenderSliverToBoxAdapter(child: e = RenderSizedBox(const Size(100.0, 400.0))),
      ],
    );
    layout(root);

    expect(root.size.width, equals(800.0));
    expect(root.size.height, equals(600.0));

    expect(a.localToGlobal(Offset.zero), Offset.zero);
    expect(b.localToGlobal(Offset.zero), const Offset(0.0, 400.0));
    expect(c.localToGlobal(Offset.zero), const Offset(0.0, 800.0));
    expect(d.localToGlobal(Offset.zero), const Offset(0.0, 1200.0));
    expect(e.localToGlobal(Offset.zero), const Offset(0.0, 1600.0));

    expect(a.localToGlobal(const Offset(800.0, 400.0)), const Offset(800.0, 400.0));
    expect(b.localToGlobal(const Offset(800.0, 400.0)), const Offset(800.0, 800.0));
    expect(c.localToGlobal(const Offset(800.0, 400.0)), const Offset(800.0, 1200.0));
    expect(d.localToGlobal(const Offset(800.0, 400.0)), const Offset(800.0, 1600.0));
    expect(e.localToGlobal(const Offset(800.0, 400.0)), const Offset(800.0, 2000.0));

    root.offset = ViewportOffset.fixed(200.0);
    pumpFrame();
    expect(a.localToGlobal(Offset.zero), const Offset(0.0, -200.0));
    expect(b.localToGlobal(Offset.zero), const Offset(0.0, 200.0));
    expect(c.localToGlobal(Offset.zero), const Offset(0.0, 600.0));
    expect(d.localToGlobal(Offset.zero), const Offset(0.0, 1000.0));
    expect(e.localToGlobal(Offset.zero), const Offset(0.0, 1400.0));

    root.offset = ViewportOffset.fixed(600.0);
    pumpFrame();
    expect(a.localToGlobal(Offset.zero), const Offset(0.0, -600.0));
    expect(b.localToGlobal(Offset.zero), const Offset(0.0, -200.0));
    expect(c.localToGlobal(Offset.zero), const Offset(0.0, 200.0));
    expect(d.localToGlobal(Offset.zero), const Offset(0.0, 600.0));
    expect(e.localToGlobal(Offset.zero), const Offset(0.0, 1000.0));

    root.offset = ViewportOffset.fixed(900.0);
    pumpFrame();
    expect(a.localToGlobal(Offset.zero), const Offset(0.0, -900.0));
    expect(b.localToGlobal(Offset.zero), const Offset(0.0, -500.0));
    expect(c.localToGlobal(Offset.zero), const Offset(0.0, -100.0));
    expect(d.localToGlobal(Offset.zero), const Offset(0.0, 300.0));
    expect(e.localToGlobal(Offset.zero), const Offset(0.0, 700.0));

    final BoxHitTestResult result = BoxHitTestResult();
    root.hitTest(result, position: const Offset(130.0, 150.0));
    expect(result.path.first.target, equals(c));
  });

  test('RenderShrinkWrappingViewport basic test - up', () {
    RenderBox a, b, c, d, e;
    final RenderShrinkWrappingViewport root = RenderShrinkWrappingViewport(
      axisDirection: AxisDirection.up,
      crossAxisDirection: AxisDirection.right,
      offset: ViewportOffset.zero(),
      children: <RenderSliver>[
        RenderSliverToBoxAdapter(child: a = RenderSizedBox(const Size(100.0, 400.0))),
        RenderSliverToBoxAdapter(child: b = RenderSizedBox(const Size(100.0, 400.0))),
        RenderSliverToBoxAdapter(child: c = RenderSizedBox(const Size(100.0, 400.0))),
        RenderSliverToBoxAdapter(child: d = RenderSizedBox(const Size(100.0, 400.0))),
        RenderSliverToBoxAdapter(child: e = RenderSizedBox(const Size(100.0, 400.0))),
      ],
    );
    layout(root);

    expect(root.size.width, equals(800.0));
    expect(root.size.height, equals(600.0));

    expect(a.localToGlobal(Offset.zero), const Offset(0.0, 200.0));
    expect(b.localToGlobal(Offset.zero), const Offset(0.0, -200.0));
    expect(c.localToGlobal(Offset.zero), const Offset(0.0, -600.0));
    expect(d.localToGlobal(Offset.zero), const Offset(0.0, -1000.0));
    expect(e.localToGlobal(Offset.zero), const Offset(0.0, -1400.0));

    root.offset = ViewportOffset.fixed(200.0);
    pumpFrame();
    expect(a.localToGlobal(Offset.zero), const Offset(0.0, 400.0));
    expect(b.localToGlobal(Offset.zero), Offset.zero);
    expect(c.localToGlobal(Offset.zero), const Offset(0.0, -400.0));
    expect(d.localToGlobal(Offset.zero), const Offset(0.0, -800.0));
    expect(e.localToGlobal(Offset.zero), const Offset(0.0, -1200.0));

    root.offset = ViewportOffset.fixed(600.0);
    pumpFrame();
    expect(a.localToGlobal(Offset.zero), const Offset(0.0, 800.0));
    expect(b.localToGlobal(Offset.zero), const Offset(0.0, 400.0));
    expect(c.localToGlobal(Offset.zero), Offset.zero);
    expect(d.localToGlobal(Offset.zero), const Offset(0.0, -400.0));
    expect(e.localToGlobal(Offset.zero), const Offset(0.0, -800.0));

    root.offset = ViewportOffset.fixed(900.0);
    pumpFrame();
    expect(a.localToGlobal(Offset.zero), const Offset(0.0, 1100.0));
    expect(b.localToGlobal(Offset.zero), const Offset(0.0, 700.0));
    expect(c.localToGlobal(Offset.zero), const Offset(0.0, 300.0));
    expect(d.localToGlobal(Offset.zero), const Offset(0.0, -100.0));
    expect(e.localToGlobal(Offset.zero), const Offset(0.0, -500.0));

    final BoxHitTestResult result = BoxHitTestResult();
    root.hitTest(result, position: const Offset(150.0, 350.0));
    expect(result.path.first.target, equals(c));
  });

  test('RenderShrinkWrappingViewport basic test - right', () {
    RenderBox a, b, c, d, e;
    final RenderShrinkWrappingViewport root = RenderShrinkWrappingViewport(
      axisDirection: AxisDirection.right,
      crossAxisDirection: AxisDirection.down,
      offset: ViewportOffset.zero(),
      children: <RenderSliver>[
        RenderSliverToBoxAdapter(child: a = RenderSizedBox(const Size(400.0, 100.0))),
        RenderSliverToBoxAdapter(child: b = RenderSizedBox(const Size(400.0, 100.0))),
        RenderSliverToBoxAdapter(child: c = RenderSizedBox(const Size(400.0, 100.0))),
        RenderSliverToBoxAdapter(child: d = RenderSizedBox(const Size(400.0, 100.0))),
        RenderSliverToBoxAdapter(child: e = RenderSizedBox(const Size(400.0, 100.0))),
      ],
    );
    layout(root);

    expect(root.size.width, equals(800.0));
    expect(root.size.height, equals(600.0));

    expect(a.localToGlobal(Offset.zero), Offset.zero);
    expect(b.localToGlobal(Offset.zero), const Offset(400.0, 0.0));
    expect(c.localToGlobal(Offset.zero), const Offset(800.0, 0.0));
    expect(d.localToGlobal(Offset.zero), const Offset(1200.0, 0.0));
    expect(e.localToGlobal(Offset.zero), const Offset(1600.0, 0.0));

    root.offset = ViewportOffset.fixed(200.0);
    pumpFrame();
    expect(a.localToGlobal(Offset.zero), const Offset(-200.0, 0.0));
    expect(b.localToGlobal(Offset.zero), const Offset(200.0, 0.0));
    expect(c.localToGlobal(Offset.zero), const Offset(600.0, 0.0));
    expect(d.localToGlobal(Offset.zero), const Offset(1000.0, 0.0));
    expect(e.localToGlobal(Offset.zero), const Offset(1400.0, 0.0));

    root.offset = ViewportOffset.fixed(600.0);
    pumpFrame();
    expect(a.localToGlobal(Offset.zero), const Offset(-600.0, 0.0));
    expect(b.localToGlobal(Offset.zero), const Offset(-200.0, 0.0));
    expect(c.localToGlobal(Offset.zero), const Offset(200.0, 0.0));
    expect(d.localToGlobal(Offset.zero), const Offset(600.0, 0.0));
    expect(e.localToGlobal(Offset.zero), const Offset(1000.0, 0.0));

    root.offset = ViewportOffset.fixed(900.0);
    pumpFrame();
    expect(a.localToGlobal(Offset.zero), const Offset(-900.0, 0.0));
    expect(b.localToGlobal(Offset.zero), const Offset(-500.0, 0.0));
    expect(c.localToGlobal(Offset.zero), const Offset(-100.0, 0.0));
    expect(d.localToGlobal(Offset.zero), const Offset(300.0, 0.0));
    expect(e.localToGlobal(Offset.zero), const Offset(700.0, 0.0));

    final BoxHitTestResult result = BoxHitTestResult();
    root.hitTest(result, position: const Offset(150.0, 450.0));
    expect(result.path.first.target, equals(c));
  });

  test('RenderShrinkWrappingViewport basic test - left', () {
    RenderBox a, b, c, d, e;
    final RenderShrinkWrappingViewport root = RenderShrinkWrappingViewport(
      axisDirection: AxisDirection.left,
      crossAxisDirection: AxisDirection.down,
      offset: ViewportOffset.zero(),
      children: <RenderSliver>[
        RenderSliverToBoxAdapter(child: a = RenderSizedBox(const Size(400.0, 100.0))),
        RenderSliverToBoxAdapter(child: b = RenderSizedBox(const Size(400.0, 100.0))),
        RenderSliverToBoxAdapter(child: c = RenderSizedBox(const Size(400.0, 100.0))),
        RenderSliverToBoxAdapter(child: d = RenderSizedBox(const Size(400.0, 100.0))),
        RenderSliverToBoxAdapter(child: e = RenderSizedBox(const Size(400.0, 100.0))),
      ],
    );
    layout(root);

    expect(root.size.width, equals(800.0));
    expect(root.size.height, equals(600.0));

    expect(a.localToGlobal(Offset.zero), const Offset(400.0, 0.0));
    expect(b.localToGlobal(Offset.zero), Offset.zero);
    expect(c.localToGlobal(Offset.zero), const Offset(-400.0, 0.0));
    expect(d.localToGlobal(Offset.zero), const Offset(-800.0, 0.0));
    expect(e.localToGlobal(Offset.zero), const Offset(-1200.0, 0.0));

    root.offset = ViewportOffset.fixed(200.0);
    pumpFrame();
    expect(a.localToGlobal(Offset.zero), const Offset(600.0, 0.0));
    expect(b.localToGlobal(Offset.zero), const Offset(200.0, 0.0));
    expect(c.localToGlobal(Offset.zero), const Offset(-200.0, 0.0));
    expect(d.localToGlobal(Offset.zero), const Offset(-600.0, 0.0));
    expect(e.localToGlobal(Offset.zero), const Offset(-1000.0, 0.0));

    root.offset = ViewportOffset.fixed(600.0);
    pumpFrame();
    expect(a.localToGlobal(Offset.zero), const Offset(1000.0, 0.0));
    expect(b.localToGlobal(Offset.zero), const Offset(600.0, 0.0));
    expect(c.localToGlobal(Offset.zero), const Offset(200.0, 0.0));
    expect(d.localToGlobal(Offset.zero), const Offset(-200.0, 0.0));
    expect(e.localToGlobal(Offset.zero), const Offset(-600.0, 0.0));

    root.offset = ViewportOffset.fixed(900.0);
    pumpFrame();
    expect(a.localToGlobal(Offset.zero), const Offset(1300.0, 0.0));
    expect(b.localToGlobal(Offset.zero), const Offset(900.0, 0.0));
    expect(c.localToGlobal(Offset.zero), const Offset(500.0, 0.0));
    expect(d.localToGlobal(Offset.zero), const Offset(100.0, 0.0));
    expect(e.localToGlobal(Offset.zero), const Offset(-300.0, 0.0));

    final BoxHitTestResult result = BoxHitTestResult();
    root.hitTest(result, position: const Offset(550.0, 150.0));
    expect(result.path.first.target, equals(c));
  });

  test('RenderShrinkWrappingViewport shrinkwrap test - 1 child', () {
    RenderBox child;
    final RenderBox root = RenderPositionedBox(
      child:
          child = RenderShrinkWrappingViewport(
            axisDirection: AxisDirection.left,
            crossAxisDirection: AxisDirection.down,
            offset: ViewportOffset.fixed(200.0),
            children: <RenderSliver>[
              RenderSliverToBoxAdapter(child: RenderSizedBox(const Size(400.0, 100.0))),
            ],
          ),
    );
    layout(root);

    expect(root.size.width, equals(800.0));
    expect(root.size.height, equals(600.0));
    expect(child.size.width, equals(400.0));
    expect(child.size.height, equals(600.0));
  });

  test('RenderShrinkWrappingViewport shrinkwrap test - 2 children', () {
    RenderBox child;
    final RenderBox root = RenderPositionedBox(
      child:
          child = RenderShrinkWrappingViewport(
            axisDirection: AxisDirection.right,
            crossAxisDirection: AxisDirection.down,
            offset: ViewportOffset.fixed(200.0),
            children: <RenderSliver>[
              RenderSliverToBoxAdapter(child: RenderSizedBox(const Size(300.0, 100.0))),
              RenderSliverToBoxAdapter(child: RenderSizedBox(const Size(150.0, 100.0))),
            ],
          ),
    );
    layout(root);

    expect(root.size.width, equals(800.0));
    expect(root.size.height, equals(600.0));
    expect(child.size.width, equals(450.0));
    expect(child.size.height, equals(600.0));
  });

  test('SliverGeometry toString', () {
    expect(
      SliverGeometry.zero.toString(),
      equals('SliverGeometry(scrollExtent: 0.0, hidden, maxPaintExtent: 0.0)'),
    );
    expect(
      const SliverGeometry(
        scrollExtent: 100.0,
        paintExtent: 50.0,
        layoutExtent: 20.0,
        visible: true,
      ).toString(),
      equals(
        'SliverGeometry(scrollExtent: 100.0, paintExtent: 50.0, layoutExtent: 20.0, maxPaintExtent: 0.0, cacheExtent: 20.0)',
      ),
    );
    expect(
      const SliverGeometry(scrollExtent: 100.0, layoutExtent: 20.0).toString(),
      equals(
        'SliverGeometry(scrollExtent: 100.0, hidden, layoutExtent: 20.0, maxPaintExtent: 0.0, cacheExtent: 20.0)',
      ),
    );
  });

  test('Sliver paintBounds and semanticBounds - vertical', () {
    const double height = 150.0;

    final RenderSliver sliver = RenderSliverToBoxAdapter(
      child: RenderSizedBox(const Size(400.0, height)),
    );
    final RenderViewport root = RenderViewport(
      crossAxisDirection: AxisDirection.right,
      offset: ViewportOffset.zero(),
      children: <RenderSliver>[sliver],
    );
    layout(root);

    final Rect expectedRect = Rect.fromLTWH(0.0, 0.0, root.size.width, height);

    expect(sliver.paintBounds, expectedRect);
    expect(sliver.semanticBounds, expectedRect);
  });

  test('Sliver paintBounds and semanticBounds - horizontal', () {
    const double width = 150.0;

    final RenderSliver sliver = RenderSliverToBoxAdapter(
      child: RenderSizedBox(const Size(width, 400.0)),
    );
    final RenderViewport root = RenderViewport(
      axisDirection: AxisDirection.right,
      crossAxisDirection: AxisDirection.down,
      offset: ViewportOffset.zero(),
      children: <RenderSliver>[sliver],
    );
    layout(root);

    final Rect expectedRect = Rect.fromLTWH(0.0, 0.0, width, root.size.height);

    expect(sliver.paintBounds, expectedRect);
    expect(sliver.semanticBounds, expectedRect);
  });

  test('precedingScrollExtent is 0.0 for first Sliver in list', () {
    const double viewportWidth = 800.0;
    final RenderSliver sliver = RenderSliverToBoxAdapter(
      child: RenderSizedBox(const Size(viewportWidth, 150.0)),
    );
    final RenderViewport root = RenderViewport(
      crossAxisDirection: AxisDirection.right,
      offset: ViewportOffset.zero(),
      children: <RenderSliver>[sliver],
    );
    layout(root);

    expect(sliver.constraints.precedingScrollExtent, 0.0);
  });

  test('precedingScrollExtent accumulates over multiple Slivers', () {
    const double viewportWidth = 800.0;
    final RenderSliver sliver1 = RenderSliverToBoxAdapter(
      child: RenderSizedBox(const Size(viewportWidth, 150.0)),
    );
    final RenderSliver sliver2 = RenderSliverToBoxAdapter(
      child: RenderSizedBox(const Size(viewportWidth, 150.0)),
    );
    final RenderSliver sliver3 = RenderSliverToBoxAdapter(
      child: RenderSizedBox(const Size(viewportWidth, 150.0)),
    );
    final RenderViewport root = RenderViewport(
      crossAxisDirection: AxisDirection.right,
      offset: ViewportOffset.zero(),
      children: <RenderSliver>[sliver1, sliver2, sliver3],
    );
    layout(root);

    // The 3rd Sliver comes after 300.0px of preceding scroll extent by first 2 Slivers.
    expect(sliver3.constraints.precedingScrollExtent, 300.0);
  });

  test('precedingScrollExtent is not impacted by scrollOffset', () {
    const double viewportWidth = 800.0;
    final RenderSliver sliver1 = RenderSliverToBoxAdapter(
      child: RenderSizedBox(const Size(viewportWidth, 150.0)),
    );
    final RenderSliver sliver2 = RenderSliverToBoxAdapter(
      child: RenderSizedBox(const Size(viewportWidth, 150.0)),
    );
    final RenderSliver sliver3 = RenderSliverToBoxAdapter(
      child: RenderSizedBox(const Size(viewportWidth, 150.0)),
    );
    final RenderViewport root = RenderViewport(
      crossAxisDirection: AxisDirection.right,
      offset: ViewportOffset.fixed(100.0),
      children: <RenderSliver>[sliver1, sliver2, sliver3],
    );
    layout(root);

    // The 3rd Sliver comes after 300.0px of preceding scroll extent by first 2 Slivers.
    // In this test a ViewportOffset is applied to simulate a scrollOffset. That
    // offset is not expected to impact the precedingScrollExtent.
    expect(sliver3.constraints.precedingScrollExtent, 300.0);
  });

  group('hit testing', () {
    test('SliverHitTestResult wrapping HitTestResult', () {
      final HitTestEntry entry1 = HitTestEntry(_DummyHitTestTarget());
      final HitTestEntry entry2 = HitTestEntry(_DummyHitTestTarget());
      final HitTestEntry entry3 = HitTestEntry(_DummyHitTestTarget());
      final Matrix4 transform = Matrix4.translationValues(40.0, 150.0, 0.0);

      final HitTestResult wrapped = MyHitTestResult()..publicPushTransform(transform);
      wrapped.add(entry1);
      expect(wrapped.path, equals(<HitTestEntry>[entry1]));
      expect(entry1.transform, transform);

      final SliverHitTestResult wrapping = SliverHitTestResult.wrap(wrapped);
      expect(wrapping.path, equals(<HitTestEntry>[entry1]));
      expect(wrapping.path, same(wrapped.path));

      wrapping.add(entry2);
      expect(wrapping.path, equals(<HitTestEntry>[entry1, entry2]));
      expect(wrapped.path, equals(<HitTestEntry>[entry1, entry2]));
      expect(entry2.transform, transform);

      wrapped.add(entry3);
      expect(wrapping.path, equals(<HitTestEntry>[entry1, entry2, entry3]));
      expect(wrapped.path, equals(<HitTestEntry>[entry1, entry2, entry3]));
      expect(entry3.transform, transform);
    });

    test('addWithAxisOffset', () {
      final SliverHitTestResult result = SliverHitTestResult();
      final List<double> mainAxisPositions = <double>[];
      final List<double> crossAxisPositions = <double>[];
      const Offset paintOffsetDummy = Offset.zero;

      bool isHit = result.addWithAxisOffset(
        paintOffset: paintOffsetDummy,
        mainAxisOffset: 0.0,
        crossAxisOffset: 0.0,
        mainAxisPosition: 0.0,
        crossAxisPosition: 0.0,
        hitTest: (
          SliverHitTestResult result, {
          required double mainAxisPosition,
          required double crossAxisPosition,
        }) {
          mainAxisPositions.add(mainAxisPosition);
          crossAxisPositions.add(crossAxisPosition);
          return true;
        },
      );
      expect(isHit, isTrue);
      expect(mainAxisPositions.single, 0.0);
      expect(crossAxisPositions.single, 0.0);
      mainAxisPositions.clear();
      crossAxisPositions.clear();

      isHit = result.addWithAxisOffset(
        paintOffset: paintOffsetDummy,
        mainAxisOffset: 5.0,
        crossAxisOffset: 6.0,
        mainAxisPosition: 10.0,
        crossAxisPosition: 20.0,
        hitTest: (
          SliverHitTestResult result, {
          required double mainAxisPosition,
          required double crossAxisPosition,
        }) {
          mainAxisPositions.add(mainAxisPosition);
          crossAxisPositions.add(crossAxisPosition);
          return false;
        },
      );
      expect(isHit, isFalse);
      expect(mainAxisPositions.single, 10.0 - 5.0);
      expect(crossAxisPositions.single, 20.0 - 6.0);
      mainAxisPositions.clear();
      crossAxisPositions.clear();

      isHit = result.addWithAxisOffset(
        paintOffset: paintOffsetDummy,
        mainAxisOffset: -5.0,
        crossAxisOffset: -6.0,
        mainAxisPosition: 10.0,
        crossAxisPosition: 20.0,
        hitTest: (
          SliverHitTestResult result, {
          required double mainAxisPosition,
          required double crossAxisPosition,
        }) {
          mainAxisPositions.add(mainAxisPosition);
          crossAxisPositions.add(crossAxisPosition);
          return false;
        },
      );
      expect(isHit, isFalse);
      expect(mainAxisPositions.single, 10.0 + 5.0);
      expect(crossAxisPositions.single, 20.0 + 6.0);
      mainAxisPositions.clear();
      crossAxisPositions.clear();
    });

    test('addWithAxisOffset with non zero paintOffset', () {
      final SliverHitTestResult result = SliverHitTestResult();
      late double recordedMainAxisPosition;
      late double recordedCrossAxisPosition;
      final HitTestEntry entry = HitTestEntry(_DummyHitTestTarget());
      const Offset paintOffset = Offset(7, 11);

      final bool isHit = result.addWithAxisOffset(
        paintOffset: paintOffset,
        mainAxisOffset: 5.0,
        crossAxisOffset: 6.0,
        mainAxisPosition: 10.0,
        crossAxisPosition: 20.0,
        hitTest: (
          SliverHitTestResult result, {
          required double mainAxisPosition,
          required double crossAxisPosition,
        }) {
          recordedMainAxisPosition = mainAxisPosition;
          recordedCrossAxisPosition = crossAxisPosition;
          result.add(entry);
          return true;
        },
      );
      expect(isHit, isTrue);
      expect(recordedMainAxisPosition, 10.0 - 5.0);
      expect(recordedCrossAxisPosition, 20.0 - 6.0);
      expect(entry.transform!..translate(paintOffset.dx, paintOffset.dy), Matrix4.identity());
    });
  });

  test('SliverConstraints check for NaN on all double properties', () {
    const SliverConstraints constraints = SliverConstraints(
      axisDirection: AxisDirection.down,
      cacheOrigin: double.nan,
      crossAxisDirection: AxisDirection.left,
      crossAxisExtent: double.nan,
      growthDirection: GrowthDirection.forward,
      overlap: double.nan,
      precedingScrollExtent: double.nan,
      remainingCacheExtent: double.nan,
      remainingPaintExtent: double.nan,
      scrollOffset: double.nan,
      userScrollDirection: ScrollDirection.idle,
      viewportMainAxisExtent: double.nan,
    );
    bool threw = false;
    try {
      constraints.debugAssertIsValid();
    } on FlutterError catch (error) {
      expect(
        error.message,
        'SliverConstraints is not valid:\n'
        '  The "scrollOffset" is NaN.\n'
        '  The "overlap" is NaN.\n'
        '  The "crossAxisExtent" is NaN.\n'
        '  The "scrollOffset" is NaN, expected greater than or equal to zero.\n'
        '  The "viewportMainAxisExtent" is NaN, expected greater than or equal to zero.\n'
        '  The "remainingPaintExtent" is NaN, expected greater than or equal to zero.\n'
        '  The "remainingCacheExtent" is NaN, expected greater than or equal to zero.\n'
        '  The "cacheOrigin" is NaN, expected less than or equal to zero.\n'
        '  The "precedingScrollExtent" is NaN, expected greater than or equal to zero.\n'
        '  The constraints are not normalized.\n'
        'The offending constraints were:\n'
        '  SliverConstraints(AxisDirection.down, GrowthDirection.forward, ScrollDirection.idle, scrollOffset: NaN, precedingScrollExtent: NaN, remainingPaintExtent: NaN, overlap: NaN, crossAxisExtent: NaN, crossAxisDirection: AxisDirection.left, viewportMainAxisExtent: NaN, remainingCacheExtent: NaN, cacheOrigin: NaN)',
      );
      threw = true;
    }
    expect(threw, true);
  });

  test('SliverConstraints check for sign on relevant double properties', () {
    const SliverConstraints constraints = SliverConstraints(
      axisDirection: AxisDirection.down,
      cacheOrigin: 1.0,
      crossAxisDirection: AxisDirection.left,
      crossAxisExtent: 0.0,
      growthDirection: GrowthDirection.forward,
      overlap: 0.0,
      precedingScrollExtent: -1.0,
      remainingCacheExtent: -1.0,
      remainingPaintExtent: -1.0,
      scrollOffset: -1.0,
      userScrollDirection: ScrollDirection.idle,
      viewportMainAxisExtent: 0.0,
    );
    bool threw = false;
    try {
      constraints.debugAssertIsValid();
    } on FlutterError catch (error) {
      expect(
        error.message,
        'SliverConstraints is not valid:\n'
        '  The "scrollOffset" is negative.\n'
        '  The "remainingPaintExtent" is negative.\n'
        '  The "remainingCacheExtent" is negative.\n'
        '  The "cacheOrigin" is positive.\n'
        '  The "precedingScrollExtent" is negative.\n'
        '  The constraints are not normalized.\n'
        'The offending constraints were:\n'
        '  SliverConstraints(AxisDirection.down, GrowthDirection.forward, ScrollDirection.idle, scrollOffset: -1.0, precedingScrollExtent: -1.0, remainingPaintExtent: -1.0, crossAxisExtent: 0.0, crossAxisDirection: AxisDirection.left, viewportMainAxisExtent: 0.0, remainingCacheExtent: -1.0, cacheOrigin: 1.0)',
      );
      threw = true;
    }
    expect(threw, true);
  });

  // Regression test for https://github.com/flutter/flutter/issues/150305
  test('SliverLayoutDimensions has correct hashCode', () {
    const SliverLayoutDimensions dimensions = SliverLayoutDimensions(
      scrollOffset: 1.0,
      precedingScrollExtent: 2.0,
      viewportMainAxisExtent: 3.0,
      crossAxisExtent: 4.0,
    );
    expect(
      dimensions.hashCode,
      Object.hash(
        dimensions.scrollOffset,
        dimensions.precedingScrollExtent,
        dimensions.viewportMainAxisExtent,
        dimensions.crossAxisExtent,
      ),
    );
  });
}

class _DummyHitTestTarget implements HitTestTarget {
  @override
  void handleEvent(PointerEvent event, HitTestEntry entry) {
    // Nothing to do.
  }
}

class MyHitTestResult extends HitTestResult {
  void publicPushTransform(Matrix4 transform) => pushTransform(transform);
}
