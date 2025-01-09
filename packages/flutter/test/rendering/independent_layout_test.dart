// Copyright 2014 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

import 'package:flutter/rendering.dart';
import 'package:flutter_test/flutter_test.dart';

import 'rendering_tester.dart';

class TestLayout {
  TestLayout() {
    // incoming constraints are tight 800x600
    root = RenderPositionedBox(
      child: RenderConstrainedBox(
        additionalConstraints: const BoxConstraints.tightFor(width: 800.0),
        child: RenderCustomPaint(
          painter: TestCallbackPainter(
            onPaint: () {
              painted = true;
            },
          ),
          child:
              child = RenderConstrainedBox(
                additionalConstraints: const BoxConstraints.tightFor(height: 10.0, width: 10.0),
              ),
        ),
      ),
    );
  }
  late RenderBox root;
  late RenderBox child;
  bool painted = false;
}

void main() {
  TestRenderingFlutterBinding.ensureInitialized();

  final ViewConfiguration testConfiguration = ViewConfiguration(
    logicalConstraints: BoxConstraints.tight(const Size(800.0, 600.0)),
  );

  test('onscreen layout does not affect offscreen', () {
    final TestLayout onscreen = TestLayout();
    final TestLayout offscreen = TestLayout();
    expect(onscreen.child.hasSize, isFalse);
    expect(onscreen.painted, isFalse);
    expect(offscreen.child.hasSize, isFalse);
    expect(offscreen.painted, isFalse);
    // Attach the offscreen to a custom render view and owner
    final RenderView renderView = RenderView(
      configuration: testConfiguration,
      view: RendererBinding.instance.platformDispatcher.views.single,
    );
    final PipelineOwner pipelineOwner = PipelineOwner();
    renderView.attach(pipelineOwner);
    renderView.child = offscreen.root;
    renderView.prepareInitialFrame();
    pipelineOwner.requestVisualUpdate();
    // Lay out the onscreen in the default binding
    layout(onscreen.root, phase: EnginePhase.paint);
    expect(onscreen.child.hasSize, isTrue);
    expect(onscreen.painted, isTrue);
    expect(onscreen.child.size, equals(const Size(800.0, 10.0)));
    // Make sure the offscreen didn't get laid out
    expect(offscreen.child.hasSize, isFalse);
    expect(offscreen.painted, isFalse);
    // Now lay out the offscreen
    pipelineOwner.flushLayout();
    expect(offscreen.child.hasSize, isTrue);
    expect(offscreen.painted, isFalse);
    pipelineOwner.flushCompositingBits();
    pipelineOwner.flushPaint();
    expect(offscreen.painted, isTrue);
  });

  test('offscreen layout does not affect onscreen', () {
    final TestLayout onscreen = TestLayout();
    final TestLayout offscreen = TestLayout();
    expect(onscreen.child.hasSize, isFalse);
    expect(onscreen.painted, isFalse);
    expect(offscreen.child.hasSize, isFalse);
    expect(offscreen.painted, isFalse);
    // Attach the offscreen to a custom render view and owner
    final RenderView renderView = RenderView(
      configuration: testConfiguration,
      view: RendererBinding.instance.platformDispatcher.views.single,
    );
    final PipelineOwner pipelineOwner = PipelineOwner();
    renderView.attach(pipelineOwner);
    renderView.child = offscreen.root;
    renderView.prepareInitialFrame();
    pipelineOwner.requestVisualUpdate();
    // Lay out the offscreen
    pipelineOwner.flushLayout();
    expect(offscreen.child.hasSize, isTrue);
    expect(offscreen.painted, isFalse);
    pipelineOwner.flushCompositingBits();
    pipelineOwner.flushPaint();
    expect(offscreen.painted, isTrue);
    // Make sure the onscreen didn't get laid out
    expect(onscreen.child.hasSize, isFalse);
    expect(onscreen.painted, isFalse);
    // Now lay out the onscreen in the default binding
    layout(onscreen.root, phase: EnginePhase.paint);
    expect(onscreen.child.hasSize, isTrue);
    expect(onscreen.painted, isTrue);
    expect(onscreen.child.size, equals(const Size(800.0, 10.0)));
  });
}
