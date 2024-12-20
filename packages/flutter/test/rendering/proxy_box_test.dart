// Copyright 2014 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

import 'dart:ui' as ui show Gradient, Image, ImageFilter;

import 'package:flutter/foundation.dart';
import 'package:flutter/material.dart';
import 'package:flutter/rendering.dart';
import 'package:flutter_test/flutter_test.dart';

import 'rendering_tester.dart';

void main() {
  TestRenderingFlutterBinding.ensureInitialized();
  test('RenderFittedBox handles applying paint transform and hit-testing with empty size', () {
    final RenderFittedBox fittedBox = RenderFittedBox(
      child: RenderCustomPaint(painter: TestCallbackPainter(onPaint: () {})),
    );

    layout(fittedBox, phase: EnginePhase.flushSemantics);
    final Matrix4 transform = Matrix4.identity();
    fittedBox.applyPaintTransform(fittedBox.child!, transform);
    expect(transform, Matrix4.zero());

    final BoxHitTestResult hitTestResult = BoxHitTestResult();
    expect(fittedBox.hitTestChildren(hitTestResult, position: Offset.zero), isFalse);
  });

  test('RenderFittedBox does not paint with empty sizes', () {
    bool painted;
    RenderFittedBox makeFittedBox(Size size) {
      return RenderFittedBox(
        child: RenderCustomPaint(
          preferredSize: size,
          painter: TestCallbackPainter(
            onPaint: () {
              painted = true;
            },
          ),
        ),
      );
    }

    // The RenderFittedBox paints if both its size and its child's size are nonempty.
    painted = false;
    layout(makeFittedBox(const Size(1, 1)), phase: EnginePhase.paint);
    expect(painted, equals(true));

    // The RenderFittedBox should not paint if its child is empty-sized.
    painted = false;
    layout(makeFittedBox(Size.zero), phase: EnginePhase.paint);
    expect(painted, equals(false));

    // The RenderFittedBox should not paint if it is empty.
    painted = false;
    layout(
      makeFittedBox(const Size(1, 1)),
      constraints: BoxConstraints.tight(Size.zero),
      phase: EnginePhase.paint,
    );
    expect(painted, equals(false));
  });

  test('RenderPhysicalModel compositing', () {
    final RenderPhysicalModel root = RenderPhysicalModel(color: const Color(0xffff00ff));
    layout(root, phase: EnginePhase.composite);
    expect(root.needsCompositing, isFalse);

    // On Fuchsia, the system compositor is responsible for drawing shadows
    // for physical model layers with non-zero elevation.
    root.elevation = 1.0;
    pumpFrame(phase: EnginePhase.composite);
    expect(root.needsCompositing, isFalse);

    root.elevation = 0.0;
    pumpFrame(phase: EnginePhase.composite);
    expect(root.needsCompositing, isFalse);
  });

  test('RenderSemanticsGestureHandler adds/removes correct semantic actions', () {
    final RenderSemanticsGestureHandler renderObj = RenderSemanticsGestureHandler(
      onTap: () {},
      onHorizontalDragUpdate: (DragUpdateDetails details) {},
    );

    SemanticsConfiguration config = SemanticsConfiguration();
    renderObj.describeSemanticsConfiguration(config);
    expect(config.getActionHandler(SemanticsAction.tap), isNotNull);
    expect(config.getActionHandler(SemanticsAction.scrollLeft), isNotNull);
    expect(config.getActionHandler(SemanticsAction.scrollRight), isNotNull);

    config = SemanticsConfiguration();
    renderObj.validActions = <SemanticsAction>{SemanticsAction.tap, SemanticsAction.scrollLeft};

    renderObj.describeSemanticsConfiguration(config);
    expect(config.getActionHandler(SemanticsAction.tap), isNotNull);
    expect(config.getActionHandler(SemanticsAction.scrollLeft), isNotNull);
    expect(config.getActionHandler(SemanticsAction.scrollRight), isNull);
  });

  group('RenderPhysicalShape', () {
    test('shape change triggers repaint', () {
      for (final TargetPlatform platform in TargetPlatform.values) {
        debugDefaultTargetPlatformOverride = platform;

        final RenderPhysicalShape root = RenderPhysicalShape(
          color: const Color(0xffff00ff),
          clipper: const ShapeBorderClipper(shape: CircleBorder()),
        );
        layout(root, phase: EnginePhase.composite);
        expect(root.debugNeedsPaint, isFalse);

        // Same shape, no repaint.
        root.clipper = const ShapeBorderClipper(shape: CircleBorder());
        expect(root.debugNeedsPaint, isFalse);

        // Different shape triggers repaint.
        root.clipper = const ShapeBorderClipper(shape: StadiumBorder());
        expect(root.debugNeedsPaint, isTrue);
      }
      debugDefaultTargetPlatformOverride = null;
    });

    test('compositing', () {
      for (final TargetPlatform platform in TargetPlatform.values) {
        debugDefaultTargetPlatformOverride = platform;
        final RenderPhysicalShape root = RenderPhysicalShape(
          color: const Color(0xffff00ff),
          clipper: const ShapeBorderClipper(shape: CircleBorder()),
        );
        layout(root, phase: EnginePhase.composite);
        expect(root.needsCompositing, isFalse);

        // On non-Fuchsia platforms, we composite physical shape layers
        root.elevation = 1.0;
        pumpFrame(phase: EnginePhase.composite);
        expect(root.needsCompositing, isFalse);

        root.elevation = 0.0;
        pumpFrame(phase: EnginePhase.composite);
        expect(root.needsCompositing, isFalse);
      }
      debugDefaultTargetPlatformOverride = null;
    });
  });

  test('RenderRepaintBoundary can capture images of itself', () async {
    RenderRepaintBoundary boundary = RenderRepaintBoundary();
    layout(boundary, constraints: BoxConstraints.tight(const Size(100.0, 200.0)));
    pumpFrame(phase: EnginePhase.composite);
    ui.Image image = await boundary.toImage();
    expect(image.width, equals(100));
    expect(image.height, equals(200));

    // Now with pixel ratio set to something other than 1.0.
    boundary = RenderRepaintBoundary();
    layout(boundary, constraints: BoxConstraints.tight(const Size(100.0, 200.0)));
    pumpFrame(phase: EnginePhase.composite);
    image = await boundary.toImage(pixelRatio: 2.0);
    expect(image.width, equals(200));
    expect(image.height, equals(400));

    // Try building one with two child layers and make sure it renders them both.
    boundary = RenderRepaintBoundary();
    final RenderStack stack = RenderStack()..alignment = Alignment.topLeft;
    final RenderDecoratedBox blackBox = RenderDecoratedBox(
      decoration: const BoxDecoration(color: Color(0xff000000)),
      child: RenderConstrainedBox(
        additionalConstraints: BoxConstraints.tight(const Size.square(20.0)),
      ),
    );
    stack.add(
      RenderOpacity()
        ..opacity = 0.5
        ..child = blackBox,
    );
    final RenderDecoratedBox whiteBox = RenderDecoratedBox(
      decoration: const BoxDecoration(color: Color(0xffffffff)),
      child: RenderConstrainedBox(
        additionalConstraints: BoxConstraints.tight(const Size.square(10.0)),
      ),
    );
    final RenderPositionedBox positioned = RenderPositionedBox(
      widthFactor: 2.0,
      heightFactor: 2.0,
      alignment: Alignment.topRight,
      child: whiteBox,
    );
    stack.add(positioned);
    boundary.child = stack;
    layout(boundary, constraints: BoxConstraints.tight(const Size(20.0, 20.0)));
    pumpFrame(phase: EnginePhase.composite);
    image = await boundary.toImage();
    expect(image.width, equals(20));
    expect(image.height, equals(20));
    ByteData data = (await image.toByteData())!;

    int getPixel(int x, int y) => data.getUint32((x + y * image.width) * 4);

    expect(data.lengthInBytes, equals(20 * 20 * 4));
    expect(data.elementSizeInBytes, equals(1));
    expect(getPixel(0, 0), equals(0x00000080));
    expect(getPixel(image.width - 1, 0), equals(0xffffffff));

    final OffsetLayer layer = boundary.debugLayer! as OffsetLayer;

    image = await layer.toImage(Offset.zero & const Size(20.0, 20.0));
    expect(image.width, equals(20));
    expect(image.height, equals(20));
    data = (await image.toByteData())!;
    expect(getPixel(0, 0), equals(0x00000080));
    expect(getPixel(image.width - 1, 0), equals(0xffffffff));

    // non-zero offsets.
    image = await layer.toImage(const Offset(-10.0, -10.0) & const Size(30.0, 30.0));
    expect(image.width, equals(30));
    expect(image.height, equals(30));
    data = (await image.toByteData())!;
    expect(getPixel(0, 0), equals(0x00000000));
    expect(getPixel(10, 10), equals(0x00000080));
    expect(getPixel(image.width - 1, 0), equals(0x00000000));
    expect(getPixel(image.width - 1, 10), equals(0xffffffff));

    // offset combined with a custom pixel ratio.
    image = await layer.toImage(
      const Offset(-10.0, -10.0) & const Size(30.0, 30.0),
      pixelRatio: 2.0,
    );
    expect(image.width, equals(60));
    expect(image.height, equals(60));
    data = (await image.toByteData())!;
    expect(getPixel(0, 0), equals(0x00000000));
    expect(getPixel(20, 20), equals(0x00000080));
    expect(getPixel(image.width - 1, 0), equals(0x00000000));
    expect(getPixel(image.width - 1, 20), equals(0xffffffff));
  }, skip: isBrowser); // https://github.com/flutter/flutter/issues/49857

  test('RenderRepaintBoundary can capture images of itself synchronously', () async {
    RenderRepaintBoundary boundary = RenderRepaintBoundary();
    layout(boundary, constraints: BoxConstraints.tight(const Size(100.0, 200.0)));
    pumpFrame(phase: EnginePhase.composite);
    ui.Image image = boundary.toImageSync();
    expect(image.width, equals(100));
    expect(image.height, equals(200));

    // Now with pixel ratio set to something other than 1.0.
    boundary = RenderRepaintBoundary();
    layout(boundary, constraints: BoxConstraints.tight(const Size(100.0, 200.0)));
    pumpFrame(phase: EnginePhase.composite);
    image = boundary.toImageSync(pixelRatio: 2.0);
    expect(image.width, equals(200));
    expect(image.height, equals(400));

    // Try building one with two child layers and make sure it renders them both.
    boundary = RenderRepaintBoundary();
    final RenderStack stack = RenderStack()..alignment = Alignment.topLeft;
    final RenderDecoratedBox blackBox = RenderDecoratedBox(
      decoration: const BoxDecoration(color: Color(0xff000000)),
      child: RenderConstrainedBox(
        additionalConstraints: BoxConstraints.tight(const Size.square(20.0)),
      ),
    );
    stack.add(
      RenderOpacity()
        ..opacity = 0.5
        ..child = blackBox,
    );
    final RenderDecoratedBox whiteBox = RenderDecoratedBox(
      decoration: const BoxDecoration(color: Color(0xffffffff)),
      child: RenderConstrainedBox(
        additionalConstraints: BoxConstraints.tight(const Size.square(10.0)),
      ),
    );
    final RenderPositionedBox positioned = RenderPositionedBox(
      widthFactor: 2.0,
      heightFactor: 2.0,
      alignment: Alignment.topRight,
      child: whiteBox,
    );
    stack.add(positioned);
    boundary.child = stack;
    layout(boundary, constraints: BoxConstraints.tight(const Size(20.0, 20.0)));
    pumpFrame(phase: EnginePhase.composite);
    image = boundary.toImageSync();
    expect(image.width, equals(20));
    expect(image.height, equals(20));
    ByteData data = (await image.toByteData())!;

    int getPixel(int x, int y) => data.getUint32((x + y * image.width) * 4);

    expect(data.lengthInBytes, equals(20 * 20 * 4));
    expect(data.elementSizeInBytes, equals(1));
    expect(getPixel(0, 0), equals(0x00000080));
    expect(getPixel(image.width - 1, 0), equals(0xffffffff));

    final OffsetLayer layer = boundary.debugLayer! as OffsetLayer;

    image = layer.toImageSync(Offset.zero & const Size(20.0, 20.0));
    expect(image.width, equals(20));
    expect(image.height, equals(20));
    data = (await image.toByteData())!;
    expect(getPixel(0, 0), equals(0x00000080));
    expect(getPixel(image.width - 1, 0), equals(0xffffffff));

    // non-zero offsets.
    image = layer.toImageSync(const Offset(-10.0, -10.0) & const Size(30.0, 30.0));
    expect(image.width, equals(30));
    expect(image.height, equals(30));
    data = (await image.toByteData())!;
    expect(getPixel(0, 0), equals(0x00000000));
    expect(getPixel(10, 10), equals(0x00000080));
    expect(getPixel(image.width - 1, 0), equals(0x00000000));
    expect(getPixel(image.width - 1, 10), equals(0xffffffff));

    // offset combined with a custom pixel ratio.
    image = layer.toImageSync(const Offset(-10.0, -10.0) & const Size(30.0, 30.0), pixelRatio: 2.0);
    expect(image.width, equals(60));
    expect(image.height, equals(60));
    data = (await image.toByteData())!;
    expect(getPixel(0, 0), equals(0x00000000));
    expect(getPixel(20, 20), equals(0x00000080));
    expect(getPixel(image.width - 1, 0), equals(0x00000000));
    expect(getPixel(image.width - 1, 20), equals(0xffffffff));
  }, skip: isBrowser); // https://github.com/flutter/flutter/issues/49857

  test('RenderOpacity does not composite if it is transparent', () {
    final RenderOpacity renderOpacity = RenderOpacity(
      opacity: 0.0,
      child: RenderSizedBox(const Size(1.0, 1.0)), // size doesn't matter
    );

    layout(renderOpacity, phase: EnginePhase.composite);
    expect(renderOpacity.needsCompositing, false);
  });

  test('RenderOpacity does composite if it is opaque', () {
    final RenderOpacity renderOpacity = RenderOpacity(
      child: RenderSizedBox(const Size(1.0, 1.0)), // size doesn't matter
    );

    layout(renderOpacity, phase: EnginePhase.composite);
    expect(renderOpacity.needsCompositing, true);
  });

  test('RenderOpacity does composite if it is partially opaque', () {
    final RenderOpacity renderOpacity = RenderOpacity(
      opacity: 0.1,
      child: RenderSizedBox(const Size(1.0, 1.0)), // size doesn't matter
    );

    layout(renderOpacity, phase: EnginePhase.composite);
    expect(renderOpacity.needsCompositing, true);
  });

  test('RenderOpacity reuses its layer', () {
    _testLayerReuse<OpacityLayer>(
      RenderOpacity(
        opacity: 0.5, // must not be 0 or 1.0. Otherwise, it won't create a layer
        child: RenderRepaintBoundary(
          child: RenderSizedBox(const Size(1.0, 1.0)),
        ), // size doesn't matter
      ),
    );
  });

  test('RenderAnimatedOpacity does not composite if it is transparent', () async {
    final Animation<double> opacityAnimation = AnimationController(vsync: FakeTickerProvider())
      ..value = 0.0;

    final RenderAnimatedOpacity renderAnimatedOpacity = RenderAnimatedOpacity(
      opacity: opacityAnimation,
      child: RenderSizedBox(const Size(1.0, 1.0)), // size doesn't matter
    );

    layout(renderAnimatedOpacity, phase: EnginePhase.composite);
    expect(renderAnimatedOpacity.needsCompositing, false);
  });

  test('RenderAnimatedOpacity does composite if it is opaque', () {
    final Animation<double> opacityAnimation = AnimationController(vsync: FakeTickerProvider())
      ..value = 1.0;

    final RenderAnimatedOpacity renderAnimatedOpacity = RenderAnimatedOpacity(
      opacity: opacityAnimation,
      child: RenderSizedBox(const Size(1.0, 1.0)), // size doesn't matter
    );

    layout(renderAnimatedOpacity, phase: EnginePhase.composite);
    expect(renderAnimatedOpacity.needsCompositing, true);
  });

  test('RenderAnimatedOpacity does composite if it is partially opaque', () {
    final Animation<double> opacityAnimation = AnimationController(vsync: FakeTickerProvider())
      ..value = 0.5;

    final RenderAnimatedOpacity renderAnimatedOpacity = RenderAnimatedOpacity(
      opacity: opacityAnimation,
      child: RenderSizedBox(const Size(1.0, 1.0)), // size doesn't matter
    );

    layout(renderAnimatedOpacity, phase: EnginePhase.composite);
    expect(renderAnimatedOpacity.needsCompositing, true);
  });

  test('RenderAnimatedOpacity reuses its layer', () {
    final Animation<double> opacityAnimation = AnimationController(vsync: FakeTickerProvider())
      ..value = 0.5; // must not be 0 or 1.0. Otherwise, it won't create a layer

    _testLayerReuse<OpacityLayer>(
      RenderAnimatedOpacity(
        opacity: opacityAnimation,
        child: RenderSizedBox(const Size(1.0, 1.0)), // size doesn't matter
      ),
    );
  });

  test('RenderShaderMask reuses its layer', () {
    _testLayerReuse<ShaderMaskLayer>(
      RenderShaderMask(
        shaderCallback: (Rect rect) {
          return ui.Gradient.radial(rect.center, rect.shortestSide / 2.0, const <Color>[
            Color.fromRGBO(0, 0, 0, 1.0),
            Color.fromRGBO(255, 255, 255, 1.0),
          ]);
        },
        child: RenderSizedBox(const Size(1.0, 1.0)), // size doesn't matter
      ),
    );
  });

  test('RenderBackdropFilter reuses its layer', () {
    _testLayerReuse<BackdropFilterLayer>(
      RenderBackdropFilter(
        filter: ui.ImageFilter.blur(),
        child: RenderSizedBox(const Size(1.0, 1.0)), // size doesn't matter
      ),
    );
  });

  test('RenderClipRect reuses its layer', () {
    _testLayerReuse<ClipRectLayer>(
      RenderClipRect(
        clipper: _TestRectClipper(),
        child: RenderRepaintBoundary(
          child: RenderSizedBox(const Size(1.0, 1.0)),
        ), // size doesn't matter
      ),
    );
  });

  test('RenderClipRRect reuses its layer', () {
    _testLayerReuse<ClipRRectLayer>(
      RenderClipRRect(
        clipper: _TestRRectClipper(),
        child: RenderRepaintBoundary(
          child: RenderSizedBox(const Size(1.0, 1.0)),
        ), // size doesn't matter
      ),
    );
  });

  test('RenderClipOval reuses its layer', () {
    _testLayerReuse<ClipPathLayer>(
      RenderClipOval(
        clipper: _TestRectClipper(),
        child: RenderRepaintBoundary(
          child: RenderSizedBox(const Size(1.0, 1.0)),
        ), // size doesn't matter
      ),
    );
  });

  test('RenderClipPath reuses its layer', () {
    _testLayerReuse<ClipPathLayer>(
      RenderClipPath(
        clipper: _TestPathClipper(),
        child: RenderRepaintBoundary(
          child: RenderSizedBox(const Size(1.0, 1.0)),
        ), // size doesn't matter
      ),
    );
  });

  test('RenderPhysicalModel reuses its layer', () {
    _testLayerReuse<ClipRRectLayer>(
      RenderPhysicalModel(
        clipBehavior: Clip.hardEdge,
        color: const Color.fromRGBO(0, 0, 0, 1.0),
        child: RenderRepaintBoundary(
          child: RenderSizedBox(const Size(1.0, 1.0)),
        ), // size doesn't matter
      ),
    );
  });

  test('RenderPhysicalShape reuses its layer', () {
    _testLayerReuse<ClipPathLayer>(
      RenderPhysicalShape(
        clipper: _TestPathClipper(),
        clipBehavior: Clip.hardEdge,
        color: const Color.fromRGBO(0, 0, 0, 1.0),
        child: RenderRepaintBoundary(
          child: RenderSizedBox(const Size(1.0, 1.0)),
        ), // size doesn't matter
      ),
    );
  });

  test('RenderTransform reuses its layer', () {
    _testLayerReuse<TransformLayer>(
      RenderTransform(
        // Use a 3D transform to force compositing.
        transform: Matrix4.rotationX(0.1),
        child: RenderRepaintBoundary(
          child: RenderSizedBox(const Size(1.0, 1.0)),
        ), // size doesn't matter
      ),
    );
  });

  void testFittedBoxWithClipRectLayer() {
    _testLayerReuse<ClipRectLayer>(
      RenderFittedBox(
        fit: BoxFit.cover,
        clipBehavior: Clip.hardEdge,
        // Inject opacity under the clip to force compositing.
        child: RenderRepaintBoundary(
          child: RenderSizedBox(const Size(100.0, 200.0)),
        ), // size doesn't matter
      ),
    );
  }

  void testFittedBoxWithTransformLayer() {
    _testLayerReuse<TransformLayer>(
      RenderFittedBox(
        fit: BoxFit.fill,
        // Inject opacity under the clip to force compositing.
        child: RenderRepaintBoundary(
          child: RenderSizedBox(const Size(1, 1)),
        ), // size doesn't matter
      ),
    );
  }

  test('RenderFittedBox reuses ClipRectLayer', () {
    testFittedBoxWithClipRectLayer();
  });

  test('RenderFittedBox reuses TransformLayer', () {
    testFittedBoxWithTransformLayer();
  });

  test('RenderFittedBox switches between ClipRectLayer and TransformLayer, and reuses them', () {
    testFittedBoxWithClipRectLayer();

    // clip -> transform
    testFittedBoxWithTransformLayer();
    // transform -> clip
    testFittedBoxWithClipRectLayer();
  });

  test('RenderFittedBox respects clipBehavior', () {
    const BoxConstraints viewport = BoxConstraints(maxHeight: 100.0, maxWidth: 100.0);
    for (final Clip? clip in <Clip?>[null, ...Clip.values]) {
      final TestClipPaintingContext context = TestClipPaintingContext();
      final RenderFittedBox box;
      switch (clip) {
        case Clip.none:
        case Clip.hardEdge:
        case Clip.antiAlias:
        case Clip.antiAliasWithSaveLayer:
          box = RenderFittedBox(child: box200x200, fit: BoxFit.none, clipBehavior: clip!);
        case null:
          box = RenderFittedBox(child: box200x200, fit: BoxFit.none);
      }
      layout(
        box,
        constraints: viewport,
        phase: EnginePhase.composite,
        onErrors: expectNoFlutterErrors,
      );
      box.paint(context, Offset.zero);
      // By default, clipBehavior should be Clip.none
      expect(context.clipBehavior, equals(clip ?? Clip.none));
    }
  });

  test('RenderMouseRegion can change properties when detached', () {
    final RenderMouseRegion object = RenderMouseRegion();
    object
      ..opaque = false
      ..onEnter = (_) {}
      ..onExit = (_) {}
      ..onHover = (_) {};
    // Passes if no error is thrown
  });

  test('RenderFractionalTranslation updates its semantics after its translation value is set', () {
    final _TestSemanticsUpdateRenderFractionalTranslation box =
        _TestSemanticsUpdateRenderFractionalTranslation(translation: const Offset(0.5, 0.5));
    layout(box, constraints: BoxConstraints.tight(const Size(200.0, 200.0)));
    expect(box.markNeedsSemanticsUpdateCallCount, 1);
    box.translation = const Offset(0.4, 0.4);
    expect(box.markNeedsSemanticsUpdateCallCount, 2);
    box.translation = const Offset(0.3, 0.3);
    expect(box.markNeedsSemanticsUpdateCallCount, 3);
  });

  test('RenderFollowerLayer hit test without a leader layer and the showWhenUnlinked is true', () {
    final RenderFollowerLayer follower = RenderFollowerLayer(
      link: LayerLink(),
      child: RenderSizedBox(const Size(1.0, 1.0)),
    );
    layout(follower, constraints: BoxConstraints.tight(const Size(200.0, 200.0)));
    final BoxHitTestResult hitTestResult = BoxHitTestResult();
    expect(follower.hitTest(hitTestResult, position: Offset.zero), isTrue);
  });

  test('RenderFollowerLayer hit test without a leader layer and the showWhenUnlinked is false', () {
    final RenderFollowerLayer follower = RenderFollowerLayer(
      link: LayerLink(),
      showWhenUnlinked: false,
      child: RenderSizedBox(const Size(1.0, 1.0)),
    );
    layout(follower, constraints: BoxConstraints.tight(const Size(200.0, 200.0)));
    final BoxHitTestResult hitTestResult = BoxHitTestResult();
    expect(follower.hitTest(hitTestResult, position: Offset.zero), isFalse);
  });

  test('RenderFollowerLayer hit test with a leader layer and the showWhenUnlinked is true', () {
    // Creates a layer link with a leader.
    final LayerLink link = LayerLink();
    final LeaderLayer leader = LeaderLayer(link: link);
    leader.attach(Object());

    final RenderFollowerLayer follower = RenderFollowerLayer(
      link: link,
      child: RenderSizedBox(const Size(1.0, 1.0)),
    );
    layout(follower, constraints: BoxConstraints.tight(const Size(200.0, 200.0)));
    final BoxHitTestResult hitTestResult = BoxHitTestResult();
    expect(follower.hitTest(hitTestResult, position: Offset.zero), isTrue);
  });

  test('RenderFollowerLayer hit test with a leader layer and the showWhenUnlinked is false', () {
    // Creates a layer link with a leader.
    final LayerLink link = LayerLink();
    final LeaderLayer leader = LeaderLayer(link: link);
    leader.attach(Object());

    final RenderFollowerLayer follower = RenderFollowerLayer(
      link: link,
      showWhenUnlinked: false,
      child: RenderSizedBox(const Size(1.0, 1.0)),
    );
    layout(follower, constraints: BoxConstraints.tight(const Size(200.0, 200.0)));
    final BoxHitTestResult hitTestResult = BoxHitTestResult();
    // The follower is still hit testable because there is a leader layer.
    expect(follower.hitTest(hitTestResult, position: Offset.zero), isTrue);
  });

  test('RenderObject can become a repaint boundary', () {
    final ConditionalRepaintBoundary childBox = ConditionalRepaintBoundary();
    final ConditionalRepaintBoundary renderBox = ConditionalRepaintBoundary(child: childBox);

    layout(renderBox, phase: EnginePhase.composite);

    expect(childBox.paintCount, 1);
    expect(renderBox.paintCount, 1);

    renderBox.isRepaintBoundary = true;
    renderBox.markNeedsCompositingBitsUpdate();
    renderBox.markNeedsCompositedLayerUpdate();

    pumpFrame(phase: EnginePhase.composite);

    // The first time the render object becomes a repaint boundary
    // we must repaint from the parent to allow the layer to be
    // created.
    expect(childBox.paintCount, 2);
    expect(renderBox.paintCount, 2);
    expect(renderBox.debugLayer, isA<OffsetLayer>());

    renderBox.markNeedsCompositedLayerUpdate();
    expect(renderBox.debugNeedsPaint, false);
    expect(renderBox.debugNeedsCompositedLayerUpdate, true);

    pumpFrame(phase: EnginePhase.composite);

    // The second time the layer exists and we can skip paint.
    expect(childBox.paintCount, 2);
    expect(renderBox.paintCount, 2);
    expect(renderBox.debugLayer, isA<OffsetLayer>());

    renderBox.isRepaintBoundary = false;
    renderBox.markNeedsCompositingBitsUpdate();

    pumpFrame(phase: EnginePhase.composite);

    // Once it stops being a repaint boundary we must repaint to
    // remove the layer. its required that the render object
    // perform this action in paint.
    expect(childBox.paintCount, 3);
    expect(renderBox.paintCount, 3);
    expect(renderBox.debugLayer, null);

    // When the render object is not a repaint boundary, calling
    // markNeedsLayerPropertyUpdate is the same as calling
    // markNeedsPaint.

    renderBox.markNeedsCompositedLayerUpdate();
    expect(renderBox.debugNeedsPaint, true);
    expect(renderBox.debugNeedsCompositedLayerUpdate, true);
  });

  test(
    'RenderObject with repaint boundary asserts when a composited layer is replaced during layer property update',
    () {
      final ConditionalRepaintBoundary childBox = ConditionalRepaintBoundary(
        isRepaintBoundary: true,
      );
      final ConditionalRepaintBoundary renderBox = ConditionalRepaintBoundary(child: childBox);

      // Ignore old layer.
      childBox.offsetLayerFactory = (OffsetLayer? oldLayer) {
        return TestOffsetLayerA();
      };

      layout(renderBox, phase: EnginePhase.composite);

      expect(childBox.paintCount, 1);
      expect(renderBox.paintCount, 1);

      renderBox.markNeedsCompositedLayerUpdate();

      pumpFrame(phase: EnginePhase.composite, onErrors: expectAssertionError);
    },
    skip: kIsWeb, // https://github.com/flutter/flutter/issues/102086
  );

  test(
    'RenderObject with repaint boundary asserts when a composited layer is replaced during painting',
    () {
      final ConditionalRepaintBoundary childBox = ConditionalRepaintBoundary(
        isRepaintBoundary: true,
      );
      final ConditionalRepaintBoundary renderBox = ConditionalRepaintBoundary(child: childBox);

      // Ignore old layer.
      childBox.offsetLayerFactory = (OffsetLayer? oldLayer) {
        return TestOffsetLayerA();
      };

      layout(renderBox, phase: EnginePhase.composite);

      expect(childBox.paintCount, 1);
      expect(renderBox.paintCount, 1);
      renderBox.markNeedsPaint();

      pumpFrame(phase: EnginePhase.composite, onErrors: expectAssertionError);
    },
    skip: kIsWeb, // https://github.com/flutter/flutter/issues/102086
  );

  test(
    'RenderObject with repaint boundary asserts when a composited layer tries to update its own offset',
    () {
      final ConditionalRepaintBoundary childBox = ConditionalRepaintBoundary(
        isRepaintBoundary: true,
      );
      final ConditionalRepaintBoundary renderBox = ConditionalRepaintBoundary(child: childBox);

      // Ignore old layer.
      childBox.offsetLayerFactory = (OffsetLayer? oldLayer) {
        return (oldLayer ?? TestOffsetLayerA())..offset = const Offset(2133, 4422);
      };

      layout(renderBox, phase: EnginePhase.composite);

      expect(childBox.paintCount, 1);
      expect(renderBox.paintCount, 1);
      renderBox.markNeedsPaint();

      pumpFrame(phase: EnginePhase.composite, onErrors: expectAssertionError);
    },
    skip: kIsWeb, // https://github.com/flutter/flutter/issues/102086
  );

  test(
    'RenderObject markNeedsPaint while repaint boundary, and then updated to no longer be a repaint boundary with '
    'calling markNeedsCompositingBitsUpdate 1',
    () {
      final ConditionalRepaintBoundary childBox = ConditionalRepaintBoundary(
        isRepaintBoundary: true,
      );
      final ConditionalRepaintBoundary renderBox = ConditionalRepaintBoundary(child: childBox);
      // Ignore old layer.
      childBox.offsetLayerFactory = (OffsetLayer? oldLayer) {
        return oldLayer ?? TestOffsetLayerA();
      };

      layout(renderBox, phase: EnginePhase.composite);

      expect(childBox.paintCount, 1);
      expect(renderBox.paintCount, 1);

      childBox.markNeedsPaint();
      childBox.isRepaintBoundary = false;
      childBox.markNeedsCompositingBitsUpdate();

      expect(() => pumpFrame(phase: EnginePhase.composite), returnsNormally);
    },
  );

  test(
    'RenderObject markNeedsPaint while repaint boundary, and then updated to no longer be a repaint boundary with '
    'calling markNeedsCompositingBitsUpdate 2',
    () {
      final ConditionalRepaintBoundary childBox = ConditionalRepaintBoundary(
        isRepaintBoundary: true,
      );
      final ConditionalRepaintBoundary renderBox = ConditionalRepaintBoundary(child: childBox);
      // Ignore old layer.
      childBox.offsetLayerFactory = (OffsetLayer? oldLayer) {
        return oldLayer ?? TestOffsetLayerA();
      };

      layout(renderBox, phase: EnginePhase.composite);

      expect(childBox.paintCount, 1);
      expect(renderBox.paintCount, 1);

      childBox.isRepaintBoundary = false;
      childBox.markNeedsCompositingBitsUpdate();
      childBox.markNeedsPaint();

      expect(() => pumpFrame(phase: EnginePhase.composite), returnsNormally);
    },
  );

  test(
    'RenderObject markNeedsPaint while repaint boundary, and then updated to no longer be a repaint boundary with '
    'calling markNeedsCompositingBitsUpdate 3',
    () {
      final ConditionalRepaintBoundary childBox = ConditionalRepaintBoundary(
        isRepaintBoundary: true,
      );
      final ConditionalRepaintBoundary renderBox = ConditionalRepaintBoundary(child: childBox);
      // Ignore old layer.
      childBox.offsetLayerFactory = (OffsetLayer? oldLayer) {
        return oldLayer ?? TestOffsetLayerA();
      };

      layout(renderBox, phase: EnginePhase.composite);

      expect(childBox.paintCount, 1);
      expect(renderBox.paintCount, 1);

      childBox.isRepaintBoundary = false;
      childBox.markNeedsCompositedLayerUpdate();
      childBox.markNeedsCompositingBitsUpdate();

      expect(() => pumpFrame(phase: EnginePhase.composite), returnsNormally);
    },
  );

  test('Offstage implements paintsChild correctly', () {
    final RenderConstrainedBox box = RenderConstrainedBox(
      additionalConstraints: const BoxConstraints.tightFor(width: 20),
    );
    final RenderConstrainedBox parent = RenderConstrainedBox(
      additionalConstraints: const BoxConstraints.tightFor(width: 20),
    );
    final RenderOffstage offstage = RenderOffstage(offstage: false, child: box);
    parent.child = offstage;

    expect(offstage.paintsChild(box), true);

    offstage.offstage = true;

    expect(offstage.paintsChild(box), false);
  });

  test('Opacity implements paintsChild correctly', () {
    final RenderBox box = RenderConstrainedBox(
      additionalConstraints: const BoxConstraints.tightFor(width: 20),
    );
    final RenderOpacity opacity = RenderOpacity(child: box);

    expect(opacity.paintsChild(box), true);

    opacity.opacity = 0;

    expect(opacity.paintsChild(box), false);
  });

  test('AnimatedOpacity sets paint matrix to zero when alpha == 0', () {
    final RenderBox box = RenderConstrainedBox(
      additionalConstraints: const BoxConstraints.tightFor(width: 20),
    );
    final AnimationController opacityAnimation = AnimationController(
      value: 1,
      vsync: FakeTickerProvider(),
    );
    final RenderAnimatedOpacity opacity = RenderAnimatedOpacity(
      opacity: opacityAnimation,
      child: box,
    );

    // Make it listen to the animation.
    opacity.attach(PipelineOwner());

    expect(opacity.paintsChild(box), true);

    opacityAnimation.value = 0;

    expect(opacity.paintsChild(box), false);
  });

  test('AnimatedOpacity sets paint matrix to zero when alpha == 0 (sliver)', () {
    final RenderSliver sliver = RenderSliverToBoxAdapter(
      child: RenderConstrainedBox(additionalConstraints: const BoxConstraints.tightFor(width: 20)),
    );
    final AnimationController opacityAnimation = AnimationController(
      value: 1,
      vsync: FakeTickerProvider(),
    );
    final RenderSliverAnimatedOpacity opacity = RenderSliverAnimatedOpacity(
      opacity: opacityAnimation,
      sliver: sliver,
    );

    // Make it listen to the animation.
    opacity.attach(PipelineOwner());

    expect(opacity.paintsChild(sliver), true);

    opacityAnimation.value = 0;

    expect(opacity.paintsChild(sliver), false);
  });

  test('RenderCustomClip extenders respect clipBehavior when asked to describeApproximateClip', () {
    final RenderBox child = RenderConstrainedBox(
      additionalConstraints: const BoxConstraints.tightFor(width: 200, height: 200),
    );
    final RenderClipRect renderClipRect = RenderClipRect(clipBehavior: Clip.none, child: child);
    layout(renderClipRect);
    expect(renderClipRect.describeApproximatePaintClip(child), null);
    renderClipRect.clipBehavior = Clip.hardEdge;
    expect(renderClipRect.describeApproximatePaintClip(child), Offset.zero & renderClipRect.size);
    renderClipRect.clipBehavior = Clip.antiAlias;
    expect(renderClipRect.describeApproximatePaintClip(child), Offset.zero & renderClipRect.size);
    renderClipRect.clipBehavior = Clip.antiAliasWithSaveLayer;
    expect(renderClipRect.describeApproximatePaintClip(child), Offset.zero & renderClipRect.size);
  });

  // Simulate painting a RenderBox as if 'debugPaintSizeEnabled == true'
  DebugPaintCallback debugPaint(RenderBox renderBox) {
    layout(renderBox);
    pumpFrame(phase: EnginePhase.compositingBits);
    return (PaintingContext context, Offset offset) {
      renderBox.paint(context, offset);
      renderBox.debugPaintSize(context, offset);
    };
  }

  test(
    'RenderClipPath.debugPaintSize draws a path and a debug text when clipBehavior is not Clip.none',
    () {
      DebugPaintCallback debugPaintClipRect(Clip clip) {
        final RenderBox child = RenderConstrainedBox(
          additionalConstraints: const BoxConstraints.tightFor(width: 200, height: 200),
        );
        final RenderClipPath renderClipPath = RenderClipPath(clipBehavior: clip, child: child);
        return debugPaint(renderClipPath);
      }

      // RenderClipPath.debugPaintSize draws when clipBehavior is not Clip.none
      expect(debugPaintClipRect(Clip.hardEdge), paintsExactlyCountTimes(#drawPath, 1));
      expect(debugPaintClipRect(Clip.hardEdge), paintsExactlyCountTimes(#drawParagraph, 1));

      // RenderClipPath.debugPaintSize does not draw when clipBehavior is Clip.none
      // Regression test for https://github.com/flutter/flutter/issues/105969
      expect(debugPaintClipRect(Clip.none), paintsExactlyCountTimes(#drawPath, 0));
      expect(debugPaintClipRect(Clip.none), paintsExactlyCountTimes(#drawParagraph, 0));
    },
  );

  test(
    'RenderClipRect.debugPaintSize draws a rect and a debug text when clipBehavior is not Clip.none',
    () {
      DebugPaintCallback debugPaintClipRect(Clip clip) {
        final RenderBox child = RenderConstrainedBox(
          additionalConstraints: const BoxConstraints.tightFor(width: 200, height: 200),
        );
        final RenderClipRect renderClipRect = RenderClipRect(clipBehavior: clip, child: child);
        return debugPaint(renderClipRect);
      }

      // RenderClipRect.debugPaintSize draws when clipBehavior is not Clip.none
      expect(debugPaintClipRect(Clip.hardEdge), paintsExactlyCountTimes(#drawRect, 1));
      expect(debugPaintClipRect(Clip.hardEdge), paintsExactlyCountTimes(#drawParagraph, 1));

      // RenderClipRect.debugPaintSize does not draw when clipBehavior is Clip.none
      expect(debugPaintClipRect(Clip.none), paintsExactlyCountTimes(#drawRect, 0));
      expect(debugPaintClipRect(Clip.none), paintsExactlyCountTimes(#drawParagraph, 0));
    },
  );

  test(
    'RenderClipRRect.debugPaintSize draws a rounded rect and a debug text when clipBehavior is not Clip.none',
    () {
      DebugPaintCallback debugPaintClipRRect(Clip clip) {
        final RenderBox child = RenderConstrainedBox(
          additionalConstraints: const BoxConstraints.tightFor(width: 200, height: 200),
        );
        final RenderClipRRect renderClipRRect = RenderClipRRect(clipBehavior: clip, child: child);
        return debugPaint(renderClipRRect);
      }

      // RenderClipRRect.debugPaintSize draws when clipBehavior is not Clip.none
      expect(debugPaintClipRRect(Clip.hardEdge), paintsExactlyCountTimes(#drawRRect, 1));
      expect(debugPaintClipRRect(Clip.hardEdge), paintsExactlyCountTimes(#drawParagraph, 1));

      // RenderClipRRect.debugPaintSize does not draw when clipBehavior is Clip.none
      expect(debugPaintClipRRect(Clip.none), paintsExactlyCountTimes(#drawRRect, 0));
      expect(debugPaintClipRRect(Clip.none), paintsExactlyCountTimes(#drawParagraph, 0));
    },
  );

  test(
    'RenderClipOval.debugPaintSize draws a path and a debug text when clipBehavior is not Clip.none',
    () {
      DebugPaintCallback debugPaintClipOval(Clip clip) {
        final RenderBox child = RenderConstrainedBox(
          additionalConstraints: const BoxConstraints.tightFor(width: 200, height: 200),
        );
        final RenderClipOval renderClipOval = RenderClipOval(clipBehavior: clip, child: child);
        return debugPaint(renderClipOval);
      }

      // RenderClipOval.debugPaintSize draws when clipBehavior is not Clip.none
      expect(debugPaintClipOval(Clip.hardEdge), paintsExactlyCountTimes(#drawPath, 1));
      expect(debugPaintClipOval(Clip.hardEdge), paintsExactlyCountTimes(#drawParagraph, 1));

      // RenderClipOval.debugPaintSize does not draw when clipBehavior is Clip.none
      expect(debugPaintClipOval(Clip.none), paintsExactlyCountTimes(#drawPath, 0));
      expect(debugPaintClipOval(Clip.none), paintsExactlyCountTimes(#drawParagraph, 0));
    },
  );

  test('RenderProxyBox behavior can be mixed in along with another base class', () {
    final RenderFancyProxyBox fancyProxyBox = RenderFancyProxyBox(fancy: 6);
    // Box has behavior from its base class:
    expect(fancyProxyBox.fancyMethod(), 36);
    // Box has behavior from RenderProxyBox:
    expect(
      // ignore: invalid_use_of_protected_member
      fancyProxyBox.computeDryLayout(const BoxConstraints(minHeight: 8)),
      const Size(0, 8),
    );
  });

  test('computeDryLayout constraints are covariant', () {
    final RenderBoxWithTestConstraints box = RenderBoxWithTestConstraints();
    const TestConstraints constraints = TestConstraints(testValue: 6);
    expect(box.computeDryLayout(constraints), const Size.square(6));
  });
}

class _TestRectClipper extends CustomClipper<Rect> {
  @override
  Rect getClip(Size size) {
    return Rect.zero;
  }

  @override
  Rect getApproximateClipRect(Size size) => getClip(size);

  @override
  bool shouldReclip(_TestRectClipper oldClipper) => true;
}

class _TestRRectClipper extends CustomClipper<RRect> {
  @override
  RRect getClip(Size size) {
    return RRect.zero;
  }

  @override
  Rect getApproximateClipRect(Size size) => getClip(size).outerRect;

  @override
  bool shouldReclip(_TestRRectClipper oldClipper) => true;
}

// Forces two frames and checks that:
// - a layer is created on the first frame
// - the layer is reused on the second frame
void _testLayerReuse<L extends Layer>(RenderBox renderObject) {
  expect(L, isNot(Layer));
  expect(renderObject.debugLayer, null);
  layout(
    renderObject,
    phase: EnginePhase.paint,
    constraints: BoxConstraints.tight(const Size(10, 10)),
  );
  final Layer? layer = renderObject.debugLayer;
  expect(layer, isA<L>());
  expect(layer, isNotNull);

  // Mark for repaint otherwise pumpFrame is a noop.
  renderObject.markNeedsPaint();
  expect(renderObject.debugNeedsPaint, true);
  pumpFrame(phase: EnginePhase.paint);
  expect(renderObject.debugNeedsPaint, false);
  expect(renderObject.debugLayer, same(layer));
}

class _TestPathClipper extends CustomClipper<Path> {
  @override
  Path getClip(Size size) {
    return Path()..addRect(const Rect.fromLTWH(50.0, 50.0, 100.0, 100.0));
  }

  @override
  bool shouldReclip(_TestPathClipper oldClipper) => false;
}

class _TestSemanticsUpdateRenderFractionalTranslation extends RenderFractionalTranslation {
  _TestSemanticsUpdateRenderFractionalTranslation({required super.translation});

  int markNeedsSemanticsUpdateCallCount = 0;

  @override
  void markNeedsSemanticsUpdate() {
    markNeedsSemanticsUpdateCallCount++;
    super.markNeedsSemanticsUpdate();
  }
}

class ConditionalRepaintBoundary extends RenderProxyBox {
  ConditionalRepaintBoundary({this.isRepaintBoundary = false, RenderBox? child}) : super(child);

  @override
  bool isRepaintBoundary = false;

  OffsetLayer Function(OffsetLayer?)? offsetLayerFactory;

  int paintCount = 0;

  @override
  OffsetLayer updateCompositedLayer({required covariant OffsetLayer? oldLayer}) {
    return offsetLayerFactory?.call(oldLayer) ?? super.updateCompositedLayer(oldLayer: oldLayer);
  }

  @override
  void paint(PaintingContext context, Offset offset) {
    paintCount += 1;
    super.paint(context, offset);
  }
}

class TestOffsetLayerA extends OffsetLayer {}

class RenderFancyBox extends RenderBox {
  RenderFancyBox({required this.fancy}) : super();

  late int fancy;

  int fancyMethod() {
    return fancy * fancy;
  }
}

class RenderFancyProxyBox extends RenderFancyBox
    with RenderObjectWithChildMixin<RenderBox>, RenderProxyBoxMixin<RenderBox> {
  RenderFancyProxyBox({required super.fancy});
}

void expectAssertionError() {
  final FlutterErrorDetails errorDetails =
      TestRenderingFlutterBinding.instance.takeFlutterErrorDetails()!;
  final bool asserted = errorDetails.toString().contains('Failed assertion');
  if (!asserted) {
    FlutterError.reportError(errorDetails);
  }
}

typedef DebugPaintCallback = void Function(PaintingContext context, Offset offset);

class TestConstraints extends BoxConstraints {
  const TestConstraints({double extent = 100, required this.testValue})
    : super(maxWidth: extent, maxHeight: extent);

  final double testValue;
}

class RenderBoxWithTestConstraints extends RenderProxyBox {
  @override
  Size computeDryLayout(TestConstraints constraints) {
    return constraints.constrain(Size.square(constraints.testValue));
  }
}
