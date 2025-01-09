// Copyright 2014 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

import 'dart:ui';

import 'package:flutter/rendering.dart';
import 'package:flutter_test/flutter_test.dart';

void main() {
  final RendererBinding binding = RenderingFlutterBinding.ensureInitialized();

  test('Adding/removing renderviews updates renderViews getter', () {
    final FlutterView flutterView = FakeFlutterView();
    final RenderView view = RenderView(view: flutterView);

    expect(binding.renderViews, isEmpty);
    binding.addRenderView(view);
    expect(binding.renderViews, contains(view));
    expect(view.configuration.devicePixelRatio, flutterView.devicePixelRatio);
    expect(
      view.configuration.logicalConstraints,
      BoxConstraints.tight(flutterView.physicalSize) / flutterView.devicePixelRatio,
    );

    binding.removeRenderView(view);
    expect(binding.renderViews, isEmpty);
  });

  test('illegal add/remove renderviews', () {
    final FlutterView flutterView = FakeFlutterView();
    final RenderView view1 = RenderView(view: flutterView);
    final RenderView view2 = RenderView(view: flutterView);
    final RenderView view3 = RenderView(view: FakeFlutterView(viewId: 200));

    expect(binding.renderViews, isEmpty);
    binding.addRenderView(view1);
    expect(binding.renderViews, contains(view1));

    expect(() => binding.addRenderView(view1), throwsAssertionError);
    expect(() => binding.addRenderView(view2), throwsAssertionError);
    expect(() => binding.removeRenderView(view2), throwsAssertionError);
    expect(() => binding.removeRenderView(view3), throwsAssertionError);

    expect(binding.renderViews, contains(view1));
    binding.removeRenderView(view1);
    expect(binding.renderViews, isEmpty);
    expect(() => binding.removeRenderView(view1), throwsAssertionError);
    expect(() => binding.removeRenderView(view2), throwsAssertionError);
  });

  test('changing metrics updates configuration', () {
    final FakeFlutterView flutterView = FakeFlutterView();
    final RenderView view = RenderView(view: flutterView);
    binding.addRenderView(view);
    expect(view.configuration.devicePixelRatio, 2.5);
    expect(view.configuration.logicalConstraints.isTight, isTrue);
    expect(view.configuration.logicalConstraints.minWidth, 160.0);
    expect(view.configuration.logicalConstraints.minHeight, 240.0);

    flutterView.devicePixelRatio = 3.0;
    flutterView.physicalSize = const Size(300, 300);
    binding.handleMetricsChanged();
    expect(view.configuration.devicePixelRatio, 3.0);
    expect(view.configuration.logicalConstraints.isTight, isTrue);
    expect(view.configuration.logicalConstraints.minWidth, 100.0);
    expect(view.configuration.logicalConstraints.minHeight, 100.0);

    binding.removeRenderView(view);
  });

  test('semantics actions are performed on the right view', () {
    final FakeFlutterView flutterView1 = FakeFlutterView(viewId: 1);
    final FakeFlutterView flutterView2 = FakeFlutterView(viewId: 2);
    final RenderView renderView1 = RenderView(view: flutterView1);
    final RenderView renderView2 = RenderView(view: flutterView2);
    final PipelineOwnerSpy owner1 = PipelineOwnerSpy()..rootNode = renderView1;
    final PipelineOwnerSpy owner2 = PipelineOwnerSpy()..rootNode = renderView2;

    binding.addRenderView(renderView1);
    binding.addRenderView(renderView2);

    binding.performSemanticsAction(
      const SemanticsActionEvent(type: SemanticsAction.copy, viewId: 1, nodeId: 11),
    );
    expect(owner1.semanticsOwner.performedActions.single, (11, SemanticsAction.copy, null));
    expect(owner2.semanticsOwner.performedActions, isEmpty);
    owner1.semanticsOwner.performedActions.clear();

    binding.performSemanticsAction(
      const SemanticsActionEvent(type: SemanticsAction.tap, viewId: 2, nodeId: 22),
    );
    expect(owner1.semanticsOwner.performedActions, isEmpty);
    expect(owner2.semanticsOwner.performedActions.single, (22, SemanticsAction.tap, null));
    owner2.semanticsOwner.performedActions.clear();

    binding.performSemanticsAction(
      const SemanticsActionEvent(type: SemanticsAction.tap, viewId: 3, nodeId: 22),
    );
    expect(owner1.semanticsOwner.performedActions, isEmpty);
    expect(owner2.semanticsOwner.performedActions, isEmpty);

    binding.removeRenderView(renderView1);
    binding.removeRenderView(renderView2);
  });

  test('all registered renderviews are asked to composite frame', () {
    final FakeFlutterView flutterView1 = FakeFlutterView(viewId: 1);
    final FakeFlutterView flutterView2 = FakeFlutterView(viewId: 2);
    final RenderView renderView1 = RenderView(view: flutterView1);
    final RenderView renderView2 = RenderView(view: flutterView2);
    final PipelineOwner owner1 = PipelineOwner()..rootNode = renderView1;
    final PipelineOwner owner2 = PipelineOwner()..rootNode = renderView2;
    binding.rootPipelineOwner.adoptChild(owner1);
    binding.rootPipelineOwner.adoptChild(owner2);
    binding.addRenderView(renderView1);
    binding.addRenderView(renderView2);
    renderView1.prepareInitialFrame();
    renderView2.prepareInitialFrame();

    expect(flutterView1.renderedScenes, isEmpty);
    expect(flutterView2.renderedScenes, isEmpty);

    binding.handleBeginFrame(Duration.zero);
    binding.handleDrawFrame();

    expect(flutterView1.renderedScenes, hasLength(1));
    expect(flutterView2.renderedScenes, hasLength(1));

    binding.removeRenderView(renderView1);

    binding.handleBeginFrame(Duration.zero);
    binding.handleDrawFrame();

    expect(flutterView1.renderedScenes, hasLength(1));
    expect(flutterView2.renderedScenes, hasLength(2));

    binding.removeRenderView(renderView2);

    binding.handleBeginFrame(Duration.zero);
    binding.handleDrawFrame();

    expect(flutterView1.renderedScenes, hasLength(1));
    expect(flutterView2.renderedScenes, hasLength(2));
  });

  test('hit-testing reaches the right view', () {
    final FakeFlutterView flutterView1 = FakeFlutterView(viewId: 1);
    final FakeFlutterView flutterView2 = FakeFlutterView(viewId: 2);
    final RenderView renderView1 = RenderView(view: flutterView1);
    final RenderView renderView2 = RenderView(view: flutterView2);
    binding.addRenderView(renderView1);
    binding.addRenderView(renderView2);

    HitTestResult result = HitTestResult();
    binding.hitTestInView(result, Offset.zero, 1);
    expect(result.path, hasLength(2));
    expect(result.path.first.target, renderView1);
    expect(result.path.last.target, binding);

    result = HitTestResult();
    binding.hitTestInView(result, Offset.zero, 2);
    expect(result.path, hasLength(2));
    expect(result.path.first.target, renderView2);
    expect(result.path.last.target, binding);

    result = HitTestResult();
    binding.hitTestInView(result, Offset.zero, 3);
    expect(result.path.single.target, binding);

    binding.removeRenderView(renderView1);
    binding.removeRenderView(renderView2);
  });
}

class FakeFlutterView extends Fake implements FlutterView {
  FakeFlutterView({
    this.viewId = 100,
    this.devicePixelRatio = 2.5,
    this.physicalSize = const Size(400, 600),
    this.padding = FakeViewPadding.zero,
  });

  @override
  final int viewId;
  @override
  double devicePixelRatio;
  @override
  Size physicalSize;
  @override
  ViewConstraints get physicalConstraints => ViewConstraints.tight(physicalSize);
  @override
  ViewPadding padding;

  List<Scene> renderedScenes = <Scene>[];

  @override
  void render(Scene scene, {Size? size}) {
    renderedScenes.add(scene);
  }
}

class PipelineOwnerSpy extends PipelineOwner {
  @override
  final SemanticsOwnerSpy semanticsOwner = SemanticsOwnerSpy();
}

class SemanticsOwnerSpy extends Fake implements SemanticsOwner {
  final List<(int, SemanticsAction, Object?)> performedActions =
      <(int, SemanticsAction, Object?)>[];

  @override
  void performAction(int id, SemanticsAction action, [Object? args]) {
    performedActions.add((id, action, args));
  }
}
