// Copyright 2014 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

import 'dart:ui' as ui;

import 'package:flutter/foundation.dart';
import 'package:flutter/material.dart';
import 'package:flutter/rendering.dart';
import 'package:flutter_test/flutter_test.dart';
import 'package:leak_tracker_flutter_testing/leak_tracker_flutter_testing.dart';

import 'rendering_tester.dart';

void main() {
  TestRenderingFlutterBinding.ensureInitialized();

  test('PipelineOwner dispatches memory events', () async {
    await expectLater(
      await memoryEvents(() => PipelineOwner().dispose(), PipelineOwner),
      areCreateAndDispose,
    );
  });

  test('ensure frame is scheduled for markNeedsSemanticsUpdate', () {
    // Initialize all bindings because owner.flushSemantics() requires a window
    final TestRenderObject renderObject = TestRenderObject();
    int onNeedVisualUpdateCallCount = 0;
    final PipelineOwner owner = PipelineOwner(
      onNeedVisualUpdate: () {
        onNeedVisualUpdateCallCount += 1;
      },
      onSemanticsUpdate: (ui.SemanticsUpdate update) {},
    );
    owner.ensureSemantics();
    renderObject.attach(owner);
    renderObject.layout(
      const BoxConstraints.tightForFinite(),
    ); // semantics are only calculated if layout information is up to date.
    owner.flushSemantics();

    expect(onNeedVisualUpdateCallCount, 1);
    renderObject.markNeedsSemanticsUpdate();
    expect(onNeedVisualUpdateCallCount, 2);
  });

  test('onSemanticsUpdate is called during flushSemantics.', () {
    int onSemanticsUpdateCallCount = 0;
    final PipelineOwner owner = PipelineOwner(
      onSemanticsUpdate: (ui.SemanticsUpdate update) {
        onSemanticsUpdateCallCount += 1;
      },
    );
    owner.ensureSemantics();

    expect(onSemanticsUpdateCallCount, 0);

    final TestRenderObject renderObject = TestRenderObject();
    renderObject.attach(owner);
    renderObject.layout(const BoxConstraints.tightForFinite());
    owner.flushSemantics();

    expect(onSemanticsUpdateCallCount, 1);
  });

  test('Enabling semantics without configuring onSemanticsUpdate is invalid.', () {
    final PipelineOwner pipelineOwner = PipelineOwner();
    expect(() => pipelineOwner.ensureSemantics(), throwsAssertionError);
  });

  test('onSemanticsUpdate during sendSemanticsUpdate.', () {
    int onSemanticsUpdateCallCount = 0;
    final SemanticsOwner owner = SemanticsOwner(
      onSemanticsUpdate: (ui.SemanticsUpdate update) {
        onSemanticsUpdateCallCount += 1;
      },
    );

    final SemanticsNode node = SemanticsNode.root(owner: owner);
    node.rect = Rect.largest;

    expect(onSemanticsUpdateCallCount, 0);

    owner.sendSemanticsUpdate();

    expect(onSemanticsUpdateCallCount, 1);
  });

  test('detached RenderObject does not do semantics', () {
    final TestRenderObject renderObject = TestRenderObject();
    expect(renderObject.attached, isFalse);
    expect(renderObject.describeSemanticsConfigurationCallCount, 0);

    renderObject.markNeedsSemanticsUpdate();
    expect(renderObject.describeSemanticsConfigurationCallCount, 0);
  });

  test('ensure errors processing render objects are well formatted', () {
    late FlutterErrorDetails errorDetails;
    final FlutterExceptionHandler? oldHandler = FlutterError.onError;
    FlutterError.onError = (FlutterErrorDetails details) {
      errorDetails = details;
    };
    final PipelineOwner owner = PipelineOwner();
    final TestThrowingRenderObject renderObject = TestThrowingRenderObject();
    try {
      renderObject.attach(owner);
      renderObject.layout(const BoxConstraints());
    } finally {
      FlutterError.onError = oldHandler;
    }

    expect(errorDetails, isNotNull);
    expect(errorDetails.stack, isNotNull);
    // Check the ErrorDetails without the stack trace
    final List<String> lines = errorDetails.toString().split('\n');
    // The lines in the middle of the error message contain the stack trace
    // which will change depending on where the test is run.
    expect(lines.length, greaterThan(8));
    expect(
      lines.take(4).join('\n'),
      equalsIgnoringHashCodes(
        '══╡ EXCEPTION CAUGHT BY RENDERING LIBRARY ╞══════════════════════\n'
        'The following assertion was thrown during performLayout():\n'
        'TestThrowingRenderObject does not support performLayout.\n',
      ),
    );

    expect(
      lines.getRange(lines.length - 8, lines.length).join('\n'),
      equalsIgnoringHashCodes(
        '\n'
        'The following RenderObject was being processed when the exception was fired:\n'
        '  TestThrowingRenderObject#00000 NEEDS-PAINT:\n'
        '  parentData: MISSING\n'
        '  constraints: BoxConstraints(unconstrained)\n'
        'This RenderObject has no descendants.\n'
        '═════════════════════════════════════════════════════════════════\n',
      ),
    );
  });

  test('ContainerParentDataMixin requires nulled out pointers to siblings before detach', () {
    expect(() => TestParentData().detach(), isNot(throwsAssertionError));

    final TestParentData data1 =
        TestParentData()
          ..nextSibling = RenderOpacity()
          ..previousSibling = RenderOpacity();
    expect(() => data1.detach(), throwsAssertionError);

    final TestParentData data2 = TestParentData()..previousSibling = RenderOpacity();
    expect(() => data2.detach(), throwsAssertionError);

    final TestParentData data3 = TestParentData()..nextSibling = RenderOpacity();
    expect(() => data3.detach(), throwsAssertionError);
  });

  test('RenderObject.getTransformTo asserts if target not in the same render tree', () {
    final PipelineOwner owner = PipelineOwner();
    final TestRenderObject renderObject1 = TestRenderObject();
    renderObject1.attach(owner);
    final TestRenderObject renderObject2 = TestRenderObject();
    renderObject2.attach(owner);
    expect(() => renderObject1.getTransformTo(renderObject2), throwsAssertionError);
  });

  test('RenderObject.getTransformTo works for siblings and descendants', () {
    final PipelineOwner owner = PipelineOwner();
    final TestRenderObject renderObject1 = TestRenderObject()..attach(owner);
    final TestRenderObject renderObject11 = TestRenderObject();
    final TestRenderObject renderObject12 = TestRenderObject();

    renderObject1
      ..add(renderObject11)
      ..add(renderObject12);
    expect(renderObject11.getTransformTo(renderObject12), equals(Matrix4.identity()));
    expect(renderObject1.getTransformTo(renderObject11), equals(Matrix4.identity()));
    expect(renderObject1.getTransformTo(renderObject12), equals(Matrix4.identity()));
    expect(renderObject11.getTransformTo(renderObject1), equals(Matrix4.identity()));
    expect(renderObject12.getTransformTo(renderObject1), equals(Matrix4.identity()));

    expect(renderObject1.getTransformTo(renderObject1), equals(Matrix4.identity()));
    expect(renderObject11.getTransformTo(renderObject11), equals(Matrix4.identity()));
    expect(renderObject12.getTransformTo(renderObject12), equals(Matrix4.identity()));
  });

  test('RenderObject.getTransformTo gets the correct paint transform', () {
    final PipelineOwner owner = PipelineOwner();
    final TestRenderObject renderObject0 = TestRenderObject()..attach(owner);
    final TestRenderObject renderObject1 = TestRenderObject();
    final TestRenderObject renderObject2 = TestRenderObject();
    renderObject0
      ..add(renderObject1)
      ..add(renderObject2)
      ..paintTransform = Matrix4.diagonal3Values(9, 4, 1);

    final TestRenderObject renderObject11 = TestRenderObject();
    final TestRenderObject renderObject21 = TestRenderObject();
    renderObject1
      ..add(renderObject11)
      ..paintTransform = Matrix4.translationValues(8, 16, 32);
    renderObject2
      ..add(renderObject21)
      ..paintTransform = Matrix4.translationValues(32, 64, 128);

    expect(
      renderObject11.getTransformTo(renderObject21),
      equals(Matrix4.translationValues((8 - 32) * 9, (16 - 64) * 4, 32 - 128)),
    );
    // Turn one of the paint transforms into a singular matrix and getTransformTo
    // should return Matrix4.zero().
    renderObject0.paintTransform = Matrix4(
      1,
      1,
      1,
      1,
      2,
      2,
      2,
      2,
      3,
      3,
      3,
      3,
      4,
      4,
      4,
      4,
    ); // Not a full rank matrix, so it has to be singular.
    expect(renderObject11.getTransformTo(renderObject21), equals(Matrix4.zero()));
  });

  test('PaintingContext.pushClipRect reuses the layer', () {
    _testPaintingContextLayerReuse<ClipRectLayer>((
      PaintingContextCallback painter,
      PaintingContext context,
      Offset offset,
      Layer? oldLayer,
    ) {
      return context.pushClipRect(
        true,
        offset,
        Rect.zero,
        painter,
        oldLayer: oldLayer as ClipRectLayer?,
      );
    });
  });

  test('PaintingContext.pushClipRRect reuses the layer', () {
    _testPaintingContextLayerReuse<ClipRRectLayer>((
      PaintingContextCallback painter,
      PaintingContext context,
      Offset offset,
      Layer? oldLayer,
    ) {
      return context.pushClipRRect(
        true,
        offset,
        Rect.zero,
        RRect.fromRectAndRadius(Rect.zero, const Radius.circular(1.0)),
        painter,
        oldLayer: oldLayer as ClipRRectLayer?,
      );
    });
  });

  test('PaintingContext.pushClipPath reuses the layer', () {
    _testPaintingContextLayerReuse<ClipPathLayer>((
      PaintingContextCallback painter,
      PaintingContext context,
      Offset offset,
      Layer? oldLayer,
    ) {
      return context.pushClipPath(
        true,
        offset,
        Rect.zero,
        Path(),
        painter,
        oldLayer: oldLayer as ClipPathLayer?,
      );
    });
  });

  test('PaintingContext.pushColorFilter reuses the layer', () {
    _testPaintingContextLayerReuse<ColorFilterLayer>((
      PaintingContextCallback painter,
      PaintingContext context,
      Offset offset,
      Layer? oldLayer,
    ) {
      return context.pushColorFilter(
        offset,
        const ColorFilter.mode(Color.fromRGBO(0, 0, 0, 1.0), BlendMode.clear),
        painter,
        oldLayer: oldLayer as ColorFilterLayer?,
      );
    });
  });

  test('PaintingContext.pushTransform reuses the layer', () {
    _testPaintingContextLayerReuse<TransformLayer>((
      PaintingContextCallback painter,
      PaintingContext context,
      Offset offset,
      Layer? oldLayer,
    ) {
      return context.pushTransform(
        true,
        offset,
        Matrix4.identity(),
        painter,
        oldLayer: oldLayer as TransformLayer?,
      );
    });
  });

  test('PaintingContext.pushOpacity reuses the layer', () {
    _testPaintingContextLayerReuse<OpacityLayer>((
      PaintingContextCallback painter,
      PaintingContext context,
      Offset offset,
      Layer? oldLayer,
    ) {
      return context.pushOpacity(offset, 100, painter, oldLayer: oldLayer as OpacityLayer?);
    });
  });

  test('RenderObject.dispose sets debugDisposed to true', () {
    final TestRenderObject renderObject = TestRenderObject();
    expect(renderObject.debugDisposed, false);
    renderObject.dispose();
    expect(renderObject.debugDisposed, true);
    expect(renderObject.toStringShort(), contains('DISPOSED'));
  });

  test('Leader layer can switch to a different render object within one frame', () {
    List<FlutterErrorDetails?>? caughtErrors;
    TestRenderingFlutterBinding.instance.onErrors = () {
      caughtErrors = TestRenderingFlutterBinding.instance.takeAllFlutterErrorDetails().toList();
    };

    final LayerLink layerLink = LayerLink();
    // renderObject1 paints the leader layer first.
    final LeaderLayerRenderObject renderObject1 = LeaderLayerRenderObject();
    renderObject1.layerLink = layerLink;
    renderObject1.attach(TestRenderingFlutterBinding.instance.pipelineOwner);
    final OffsetLayer rootLayer1 = OffsetLayer();
    rootLayer1.attach(renderObject1);
    renderObject1.scheduleInitialPaint(rootLayer1);
    renderObject1.layout(const BoxConstraints.tightForFinite());

    final LeaderLayerRenderObject renderObject2 = LeaderLayerRenderObject();
    final OffsetLayer rootLayer2 = OffsetLayer();
    rootLayer2.attach(renderObject2);
    renderObject2.attach(TestRenderingFlutterBinding.instance.pipelineOwner);
    renderObject2.scheduleInitialPaint(rootLayer2);
    renderObject2.layout(const BoxConstraints.tightForFinite());
    TestRenderingFlutterBinding.instance.pumpCompleteFrame();

    // Swap the layer link to renderObject2 in the same frame
    renderObject1.layerLink = null;
    renderObject1.markNeedsPaint();
    renderObject2.layerLink = layerLink;
    renderObject2.markNeedsPaint();
    TestRenderingFlutterBinding.instance.pumpCompleteFrame();

    // Swap the layer link to renderObject1 in the same frame
    renderObject1.layerLink = layerLink;
    renderObject1.markNeedsPaint();
    renderObject2.layerLink = null;
    renderObject2.markNeedsPaint();
    TestRenderingFlutterBinding.instance.pumpCompleteFrame();

    TestRenderingFlutterBinding.instance.onErrors = null;
    expect(caughtErrors, isNull);
  });

  test('Leader layer append to two render objects does crash', () {
    List<FlutterErrorDetails?>? caughtErrors;
    TestRenderingFlutterBinding.instance.onErrors = () {
      caughtErrors = TestRenderingFlutterBinding.instance.takeAllFlutterErrorDetails().toList();
    };
    final LayerLink layerLink = LayerLink();
    // renderObject1 paints the leader layer first.
    final LeaderLayerRenderObject renderObject1 = LeaderLayerRenderObject();
    renderObject1.layerLink = layerLink;
    renderObject1.attach(TestRenderingFlutterBinding.instance.pipelineOwner);
    final OffsetLayer rootLayer1 = OffsetLayer();
    rootLayer1.attach(renderObject1);
    renderObject1.scheduleInitialPaint(rootLayer1);
    renderObject1.layout(const BoxConstraints.tightForFinite());

    final LeaderLayerRenderObject renderObject2 = LeaderLayerRenderObject();
    renderObject2.layerLink = layerLink;
    final OffsetLayer rootLayer2 = OffsetLayer();
    rootLayer2.attach(renderObject2);
    renderObject2.attach(TestRenderingFlutterBinding.instance.pipelineOwner);
    renderObject2.scheduleInitialPaint(rootLayer2);
    renderObject2.layout(const BoxConstraints.tightForFinite());
    TestRenderingFlutterBinding.instance.pumpCompleteFrame();

    TestRenderingFlutterBinding.instance.onErrors = null;
    expect(caughtErrors!.isNotEmpty, isTrue);
  });

  test('RenderObject.dispose null the layer on repaint boundaries', () {
    final TestRenderObject renderObject = TestRenderObject(allowPaintBounds: true);
    // Force a layer to get set.
    renderObject.isRepaintBoundary = true;
    PaintingContext.repaintCompositedChild(renderObject, debugAlsoPaintedParent: true);
    expect(renderObject.debugLayer, isA<OffsetLayer>());

    // Dispose with repaint boundary still being true.
    renderObject.dispose();
    expect(renderObject.debugLayer, null);
  });

  test('RenderObject.dispose nulls the layer on non-repaint boundaries', () {
    final TestRenderObject renderObject = TestRenderObject(allowPaintBounds: true);
    // Force a layer to get set.
    renderObject.isRepaintBoundary = true;
    PaintingContext.repaintCompositedChild(renderObject, debugAlsoPaintedParent: true);

    // Dispose with repaint boundary being false.
    renderObject.isRepaintBoundary = false;
    renderObject.dispose();
    expect(renderObject.debugLayer, null);
  });

  test('Add composition callback works', () {
    final ContainerLayer root = ContainerLayer();
    final PaintingContext context = PaintingContext(root, Rect.zero);
    bool calledBack = false;
    final TestObservingRenderObject object = TestObservingRenderObject((Layer layer) {
      expect(layer, root);
      calledBack = true;
    });

    expect(calledBack, false);
    object.paint(context, Offset.zero);
    expect(calledBack, false);

    root.buildScene(ui.SceneBuilder()).dispose();
    expect(calledBack, true);
  });

  test('Change isRepaintBoundary after both markNeedsCompositedLayerUpdate and markNeedsPaint', () {
    List<FlutterErrorDetails?>? caughtErrors;
    TestRenderingFlutterBinding.instance.onErrors = () {
      caughtErrors = TestRenderingFlutterBinding.instance.takeAllFlutterErrorDetails().toList();
    };
    final TestRenderObject object = TestRenderObject(allowPaintBounds: true);
    object.isRepaintBoundary = true;
    object.attach(TestRenderingFlutterBinding.instance.pipelineOwner);
    object.layout(const BoxConstraints.tightForFinite());
    PaintingContext.repaintCompositedChild(object, debugAlsoPaintedParent: true);

    object.markNeedsCompositedLayerUpdate();
    object.markNeedsPaint();
    object.isRepaintBoundary = false;
    object.markNeedsCompositingBitsUpdate();
    TestRenderingFlutterBinding.instance.pumpCompleteFrame();
    expect(caughtErrors, isNull);
  });

  test('ContainerParentDataMixin asserts parentData type', () {
    final TestRenderObject renderObject = TestRenderObjectWithoutSetupParentData();
    final TestRenderObject child = TestRenderObject();
    expect(
      () => renderObject.add(child),
      throwsA(
        isA<AssertionError>().having(
          (AssertionError error) => error.toString(),
          'description',
          contains(
            'A child of TestRenderObjectWithoutSetupParentData has parentData of type ParentData, '
            'which does not conform to TestRenderObjectParentData. Class using ContainerRenderObjectMixin '
            'should override setupParentData() to set parentData to type TestRenderObjectParentData.',
          ),
        ),
      ),
    );
  });
}

class TestObservingRenderObject extends RenderBox {
  TestObservingRenderObject(this.callback);

  final CompositionCallback callback;

  @override
  bool get sizedByParent => true;

  @override
  void paint(PaintingContext context, Offset offset) {
    context.addCompositionCallback(callback);
  }
}

// Tests the create-update cycle by pumping two frames. The first frame has no
// prior layer and forces the painting context to create a new one. The second
// frame reuses the layer painted on the first frame.
void _testPaintingContextLayerReuse<L extends Layer>(_LayerTestPaintCallback painter) {
  final _TestCustomLayerBox box = _TestCustomLayerBox(painter);
  layout(box, phase: EnginePhase.paint);

  // Force a repaint. Otherwise, pumpFrame is a noop.
  box.markNeedsPaint();
  pumpFrame(phase: EnginePhase.paint);
  expect(box.paintedLayers, hasLength(2));
  expect(box.paintedLayers[0], isA<L>());
  expect(box.paintedLayers[0], same(box.paintedLayers[1]));
}

typedef _LayerTestPaintCallback =
    Layer? Function(
      PaintingContextCallback painter,
      PaintingContext context,
      Offset offset,
      Layer? oldLayer,
    );

class _TestCustomLayerBox extends RenderBox {
  _TestCustomLayerBox(this.painter);

  final _LayerTestPaintCallback painter;
  final List<Layer> paintedLayers = <Layer>[];

  @override
  bool get isRepaintBoundary => false;

  @override
  void performLayout() {
    size = constraints.smallest;
  }

  @override
  void paint(PaintingContext context, Offset offset) {
    final Layer paintedLayer = painter(super.paint, context, offset, layer)!;
    paintedLayers.add(paintedLayer);
    layer = paintedLayer as ContainerLayer;
  }
}

class TestParentData extends ParentData with ContainerParentDataMixin<RenderBox> {}

class TestRenderObjectParentData extends ParentData
    with ContainerParentDataMixin<TestRenderObject> {}

class TestRenderObject extends RenderObject
    with ContainerRenderObjectMixin<TestRenderObject, TestRenderObjectParentData> {
  TestRenderObject({this.allowPaintBounds = false});

  final bool allowPaintBounds;

  @override
  bool isRepaintBoundary = false;

  @override
  void debugAssertDoesMeetConstraints() {}

  @override
  Rect get paintBounds {
    assert(allowPaintBounds); // For some tests, this should not get called.
    return Rect.zero;
  }

  Matrix4 paintTransform = Matrix4.identity();
  @override
  void applyPaintTransform(covariant RenderObject child, Matrix4 transform) {
    super.applyPaintTransform(child, transform);
    transform.multiply(paintTransform);
  }

  @override
  void setupParentData(RenderObject child) {
    if (child.parentData is! TestRenderObjectParentData) {
      child.parentData = TestRenderObjectParentData();
    }
  }

  @override
  void performLayout() {}

  @override
  void performResize() {}

  @override
  Rect get semanticBounds => const Rect.fromLTWH(0.0, 0.0, 10.0, 20.0);

  int describeSemanticsConfigurationCallCount = 0;

  @override
  void describeSemanticsConfiguration(SemanticsConfiguration config) {
    super.describeSemanticsConfiguration(config);
    config.isSemanticBoundary = true;
    describeSemanticsConfigurationCallCount++;
  }
}

class TestRenderObjectWithoutSetupParentData extends TestRenderObject {
  @override
  void setupParentData(RenderObject child) {
    // Use a mismatched parent data type.
    if (child.parentData is! ParentData) {
      child.parentData = ParentData();
    }
  }
}

class LeaderLayerRenderObject extends RenderObject {
  LeaderLayerRenderObject();

  LayerLink? layerLink;

  @override
  bool isRepaintBoundary = true;

  @override
  void debugAssertDoesMeetConstraints() {}

  @override
  Rect get paintBounds {
    return Rect.zero;
  }

  @override
  void paint(PaintingContext context, Offset offset) {
    if (layerLink != null) {
      context.pushLayer(LeaderLayer(link: layerLink!), super.paint, offset);
    }
  }

  @override
  void performLayout() {}

  @override
  void performResize() {}

  @override
  Rect get semanticBounds => const Rect.fromLTWH(0.0, 0.0, 10.0, 20.0);
}

class TestThrowingRenderObject extends RenderObject {
  @override
  void performLayout() {
    throw FlutterError('TestThrowingRenderObject does not support performLayout.');
  }

  @override
  void debugAssertDoesMeetConstraints() {}

  @override
  Rect get paintBounds {
    assert(false); // The test shouldn't call this.
    return Rect.zero;
  }

  @override
  void performResize() {}

  @override
  Rect get semanticBounds {
    assert(false); // The test shouldn't call this.
    return Rect.zero;
  }
}
