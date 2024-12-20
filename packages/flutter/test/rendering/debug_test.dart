// Copyright 2014 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

import 'package:flutter/foundation.dart';
import 'package:flutter/rendering.dart';
import 'package:flutter_test/flutter_test.dart';
import 'package:vector_math/vector_math_64.dart';

import 'rendering_tester.dart';

void main() {
  TestRenderingFlutterBinding.ensureInitialized();

  test('Describe transform control test', () {
    final Matrix4 identity = Matrix4.identity();
    final List<String> description = debugDescribeTransform(identity);
    expect(description, <String>[
      '[0] 1.0,0.0,0.0,0.0',
      '[1] 0.0,1.0,0.0,0.0',
      '[2] 0.0,0.0,1.0,0.0',
      '[3] 0.0,0.0,0.0,1.0',
    ]);
  });

  test('transform property test', () {
    final Matrix4 transform = Matrix4.diagonal3(Vector3.all(2.0));
    final TransformProperty simple = TransformProperty('transform', transform);
    expect(simple.name, equals('transform'));
    expect(simple.value, same(transform));
    expect(
      simple.toString(parentConfiguration: sparseTextConfiguration),
      equals(
        'transform:\n'
        '  [0] 2.0,0.0,0.0,0.0\n'
        '  [1] 0.0,2.0,0.0,0.0\n'
        '  [2] 0.0,0.0,2.0,0.0\n'
        '  [3] 0.0,0.0,0.0,1.0',
      ),
    );
    expect(
      simple.toString(parentConfiguration: singleLineTextConfiguration),
      equals('transform: [2.0,0.0,0.0,0.0; 0.0,2.0,0.0,0.0; 0.0,0.0,2.0,0.0; 0.0,0.0,0.0,1.0]'),
    );

    final TransformProperty nullProperty = TransformProperty('transform', null);
    expect(nullProperty.name, equals('transform'));
    expect(nullProperty.value, isNull);
    expect(nullProperty.toString(), equals('transform: null'));

    final TransformProperty hideNull = TransformProperty('transform', null, defaultValue: null);
    expect(hideNull.value, isNull);
    expect(hideNull.toString(), equals('transform: null'));
  });

  test('debugPaintPadding', () {
    expect((Canvas canvas) {
      debugPaintPadding(canvas, const Rect.fromLTRB(10.0, 10.0, 20.0, 20.0), null);
    }, paints..rect(color: const Color(0x90909090)));
    expect(
      (Canvas canvas) {
        debugPaintPadding(
          canvas,
          const Rect.fromLTRB(10.0, 10.0, 20.0, 20.0),
          const Rect.fromLTRB(11.0, 11.0, 19.0, 19.0),
        );
      },
      paints
        ..path(color: const Color(0x900090FF))
        ..path(color: const Color(0xFF0090FF)),
    );
    expect(
      (Canvas canvas) {
        debugPaintPadding(
          canvas,
          const Rect.fromLTRB(10.0, 10.0, 20.0, 20.0),
          const Rect.fromLTRB(15.0, 15.0, 15.0, 15.0),
        );
      },
      paints
        ..rect(rect: const Rect.fromLTRB(10.0, 10.0, 20.0, 20.0), color: const Color(0x90909090)),
    );
  });

  test('debugPaintPadding from render objects', () {
    debugPaintSizeEnabled = true;
    RenderSliver s;
    RenderBox b;
    final RenderViewport root = RenderViewport(
      crossAxisDirection: AxisDirection.right,
      offset: ViewportOffset.zero(),
      children: <RenderSliver>[
        s = RenderSliverPadding(
          padding: const EdgeInsets.all(10.0),
          child: RenderSliverToBoxAdapter(
            child: b = RenderPadding(padding: const EdgeInsets.all(10.0)),
          ),
        ),
      ],
    );
    layout(root);
    expect(
      b.debugPaint,
      paints
        ..rect(color: const Color(0xFF00FFFF))
        ..rect(color: const Color(0x90909090)),
    );
    expect(b.debugPaint, isNot(paints..path()));
    expect(
      s.debugPaint,
      paints
        ..circle(hasMaskFilter: true)
        ..line(hasMaskFilter: true)
        ..path(hasMaskFilter: true)
        ..path(hasMaskFilter: true)
        ..path(color: const Color(0x900090FF))
        ..path(color: const Color(0xFF0090FF)),
    );
    expect(s.debugPaint, isNot(paints..rect()));
    debugPaintSizeEnabled = false;
  });

  test('debugPaintPadding from render objects', () {
    debugPaintSizeEnabled = true;
    RenderSliver s;
    final RenderBox b = RenderPadding(
      padding: const EdgeInsets.all(10.0),
      child: RenderViewport(
        crossAxisDirection: AxisDirection.right,
        offset: ViewportOffset.zero(),
        children: <RenderSliver>[s = RenderSliverPadding(padding: const EdgeInsets.all(10.0))],
      ),
    );
    layout(b);
    expect(s.debugPaint, paints..rect(color: const Color(0x90909090)));
    expect(
      s.debugPaint,
      isNot(
        paints
          ..circle(hasMaskFilter: true)
          ..line(hasMaskFilter: true)
          ..path(hasMaskFilter: true)
          ..path(hasMaskFilter: true)
          ..path(color: const Color(0x900090FF))
          ..path(color: const Color(0xFF0090FF)),
      ),
    );
    expect(
      b.debugPaint,
      paints
        ..rect(color: const Color(0xFF00FFFF))
        ..path(color: const Color(0x900090FF))
        ..path(color: const Color(0xFF0090FF)),
    );
    expect(b.debugPaint, isNot(paints..rect(color: const Color(0x90909090))));
    debugPaintSizeEnabled = false;
  });

  test('debugPaintPadding from render objects with inverted direction vertical', () {
    debugPaintSizeEnabled = true;
    RenderSliver s;
    final RenderViewport root = RenderViewport(
      axisDirection: AxisDirection.up,
      crossAxisDirection: AxisDirection.right,
      offset: ViewportOffset.zero(),
      children: <RenderSliver>[
        s = RenderSliverPadding(
          padding: const EdgeInsets.all(10.0),
          child: RenderSliverToBoxAdapter(
            child: RenderPadding(padding: const EdgeInsets.all(10.0)),
          ),
        ),
      ],
    );
    layout(root);
    dynamic error;
    final PaintingContext context = PaintingContext(
      ContainerLayer(),
      const Rect.fromLTRB(0.0, 0.0, 800.0, 600.0),
    );
    try {
      s.debugPaint(context, const Offset(0.0, 500));
    } catch (e) {
      error = e;
    }
    expect(error, isNull);
    debugPaintSizeEnabled = false;
  });

  test('debugPaintPadding from render objects with inverted direction horizontal', () {
    debugPaintSizeEnabled = true;
    RenderSliver s;
    final RenderViewport root = RenderViewport(
      axisDirection: AxisDirection.left,
      crossAxisDirection: AxisDirection.down,
      offset: ViewportOffset.zero(),
      children: <RenderSliver>[
        s = RenderSliverPadding(
          padding: const EdgeInsets.all(10.0),
          child: RenderSliverToBoxAdapter(
            child: RenderPadding(padding: const EdgeInsets.all(10.0)),
          ),
        ),
      ],
    );
    layout(root);
    dynamic error;
    final PaintingContext context = PaintingContext(
      ContainerLayer(),
      const Rect.fromLTRB(0.0, 0.0, 800.0, 600.0),
    );
    try {
      s.debugPaint(context, const Offset(0.0, 500));
    } catch (e) {
      error = e;
    }
    expect(error, isNull);
    debugPaintSizeEnabled = false;
  });

  test('debugDisableOpacity keeps things in the right spot', () {
    debugDisableOpacityLayers = true;

    final RenderDecoratedBox blackBox = RenderDecoratedBox(
      decoration: const BoxDecoration(color: Color(0xff000000)),
      child: RenderConstrainedBox(
        additionalConstraints: BoxConstraints.tight(const Size.square(20.0)),
      ),
    );
    final RenderOpacity root = RenderOpacity(
      opacity: .5,
      child: RenderRepaintBoundary(child: blackBox),
    );
    layout(root, phase: EnginePhase.compositingBits);

    final OffsetLayer rootLayer = OffsetLayer();
    final PaintingContext context = PaintingContext(rootLayer, const Rect.fromLTWH(0, 0, 500, 500));
    context.paintChild(root, const Offset(40, 40));

    final OpacityLayer opacityLayer = rootLayer.firstChild! as OpacityLayer;
    expect(opacityLayer.offset, const Offset(40, 40));
    debugDisableOpacityLayers = false;
  });

  test('debugAssertAllRenderVarsUnset warns when debugProfileLayoutsEnabled set', () {
    debugProfileLayoutsEnabled = true;
    expect(() => debugAssertAllRenderVarsUnset('ERROR'), throwsFlutterError);
    debugProfileLayoutsEnabled = false;
  });

  test('debugAssertAllRenderVarsUnset warns when debugDisableClipLayers set', () {
    debugDisableClipLayers = true;
    expect(() => debugAssertAllRenderVarsUnset('ERROR'), throwsFlutterError);
    debugDisableClipLayers = false;
  });

  test('debugAssertAllRenderVarsUnset warns when debugDisablePhysicalShapeLayers set', () {
    debugDisablePhysicalShapeLayers = true;
    expect(() => debugAssertAllRenderVarsUnset('ERROR'), throwsFlutterError);
    debugDisablePhysicalShapeLayers = false;
  });

  test('debugAssertAllRenderVarsUnset warns when debugDisableOpacityLayers set', () {
    debugDisableOpacityLayers = true;
    expect(() => debugAssertAllRenderVarsUnset('ERROR'), throwsFlutterError);
    debugDisableOpacityLayers = false;
  });

  test('debugCheckHasBoundedAxis warns for vertical and horizontal axis', () {
    expect(
      () => debugCheckHasBoundedAxis(Axis.vertical, const BoxConstraints()),
      throwsFlutterError,
    );
    expect(
      () => debugCheckHasBoundedAxis(Axis.horizontal, const BoxConstraints()),
      throwsFlutterError,
    );
  });
}
