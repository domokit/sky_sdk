// Copyright 2014 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

import 'package:flutter/foundation.dart';
import 'package:flutter/rendering.dart';
import 'package:flutter_test/flutter_test.dart';

import 'rendering_tester.dart';

void main() {
  MyTestRenderingFlutterBinding.ensureInitialized();

  tearDown(() {
    final List<PipelineOwner> children = <PipelineOwner>[];
    RendererBinding.instance.rootPipelineOwner.visitChildren((PipelineOwner child) {
      children.add(child);
    });
    children.forEach(RendererBinding.instance.rootPipelineOwner.dropChild);
  });

  test(
    "BindingPipelineManifold notifies binding if render object managed by binding's PipelineOwner tree needs visual update",
    () {
      final PipelineOwner child = PipelineOwner();
      RendererBinding.instance.rootPipelineOwner.adoptChild(child);

      final RenderObject renderObject = TestRenderObject();
      child.rootNode = renderObject;
      renderObject.scheduleInitialLayout();
      RendererBinding.instance.rootPipelineOwner.flushLayout();

      MyTestRenderingFlutterBinding.instance.ensureVisualUpdateCount = 0;
      renderObject.markNeedsLayout();
      expect(MyTestRenderingFlutterBinding.instance.ensureVisualUpdateCount, 1);
    },
  );

  test('Turning global semantics on/off creates semantics owners in PipelineOwner tree', () {
    final PipelineOwner child = PipelineOwner(onSemanticsUpdate: (_) {});
    RendererBinding.instance.rootPipelineOwner.adoptChild(child);

    expect(child.semanticsOwner, isNull);
    expect(RendererBinding.instance.rootPipelineOwner.semanticsOwner, isNull);

    final SemanticsHandle handle = SemanticsBinding.instance.ensureSemantics();

    expect(child.semanticsOwner, isNotNull);
    expect(RendererBinding.instance.rootPipelineOwner.semanticsOwner, isNotNull);

    handle.dispose();

    expect(child.semanticsOwner, isNull);
    expect(RendererBinding.instance.rootPipelineOwner.semanticsOwner, isNull);
  });
}

class MyTestRenderingFlutterBinding extends TestRenderingFlutterBinding {
  static MyTestRenderingFlutterBinding get instance => BindingBase.checkInstance(_instance);
  static MyTestRenderingFlutterBinding? _instance;

  static MyTestRenderingFlutterBinding ensureInitialized() {
    return _instance ?? MyTestRenderingFlutterBinding();
  }

  @override
  void initInstances() {
    super.initInstances();
    _instance = this;
  }

  int ensureVisualUpdateCount = 0;

  @override
  void ensureVisualUpdate() {
    super.ensureVisualUpdate();
    ensureVisualUpdateCount++;
  }
}

class TestRenderObject extends RenderObject {
  @override
  void debugAssertDoesMeetConstraints() {}

  @override
  Rect get paintBounds => Rect.zero;

  @override
  void performLayout() {}

  @override
  void performResize() {}

  @override
  Rect get semanticBounds => Rect.zero;
}
