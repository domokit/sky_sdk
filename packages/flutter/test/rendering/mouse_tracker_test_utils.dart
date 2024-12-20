// Copyright 2014 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

import 'dart:ui' as ui;

import 'package:flutter/foundation.dart';
import 'package:flutter/gestures.dart';
import 'package:flutter/rendering.dart';
import 'package:flutter/scheduler.dart';
import 'package:flutter/services.dart';
import 'package:flutter_test/flutter_test.dart' show TestDefaultBinaryMessengerBinding;

class _TestHitTester extends RenderBox {
  _TestHitTester(this.hitTestOverride);

  final BoxHitTest hitTestOverride;

  @override
  bool hitTest(BoxHitTestResult result, {required ui.Offset position}) {
    return hitTestOverride(result, position);
  }
}

// A binding used to test MouseTracker, allowing the test to override hit test
// searching.
class TestMouseTrackerFlutterBinding extends BindingBase
    with
        SchedulerBinding,
        ServicesBinding,
        GestureBinding,
        SemanticsBinding,
        RendererBinding,
        TestDefaultBinaryMessengerBinding {
  @override
  void initInstances() {
    super.initInstances();
    postFrameCallbacks = <void Function(Duration)>[];
  }

  late final RenderView _renderView = RenderView(view: platformDispatcher.implicitView!);

  late final PipelineOwner _pipelineOwner = PipelineOwner(
    onSemanticsUpdate: (ui.SemanticsUpdate _) {
      assert(false);
    },
  );

  void setHitTest(BoxHitTest hitTest) {
    if (_pipelineOwner.rootNode == null) {
      _pipelineOwner.rootNode = _renderView;
      rootPipelineOwner.adoptChild(_pipelineOwner);
      addRenderView(_renderView);
    }
    _renderView.child = _TestHitTester(hitTest);
  }

  SchedulerPhase? _overridePhase;
  @override
  SchedulerPhase get schedulerPhase => _overridePhase ?? super.schedulerPhase;

  // Manually schedule a post-frame check.
  //
  // In real apps this is done by the renderer binding, but in tests we have to
  // bypass the phase assertion of [MouseTracker.schedulePostFrameCheck].
  void scheduleMouseTrackerPostFrameCheck() {
    final SchedulerPhase? lastPhase = _overridePhase;
    _overridePhase = SchedulerPhase.persistentCallbacks;
    addPostFrameCallback((_) {
      mouseTracker.updateAllDevices();
    });
    _overridePhase = lastPhase;
  }

  List<void Function(Duration)> postFrameCallbacks = <void Function(Duration)>[];

  // Proxy post-frame callbacks.
  @override
  void addPostFrameCallback(void Function(Duration) callback, {String debugLabel = 'callback'}) {
    postFrameCallbacks.add(callback);
  }

  void flushPostFrameCallbacks(Duration duration) {
    for (final void Function(Duration) callback in postFrameCallbacks) {
      callback(duration);
    }
    postFrameCallbacks.clear();
  }
}

// An object that mocks the behavior of a render object with [MouseTrackerAnnotation].
class TestAnnotationTarget with Diagnosticable implements MouseTrackerAnnotation, HitTestTarget {
  const TestAnnotationTarget({
    this.onEnter,
    this.onHover,
    this.onExit,
    this.cursor = MouseCursor.defer,
    this.validForMouseTracker = true,
  });

  @override
  final PointerEnterEventListener? onEnter;

  final PointerHoverEventListener? onHover;

  @override
  final PointerExitEventListener? onExit;

  @override
  final MouseCursor cursor;

  @override
  final bool validForMouseTracker;

  @override
  void handleEvent(PointerEvent event, HitTestEntry entry) {
    if (event is PointerHoverEvent) {
      onHover?.call(event);
    }
  }
}

// A hit test entry that can be assigned with a [TestAnnotationTarget] and an
// optional transform matrix.
class TestAnnotationEntry extends HitTestEntry<TestAnnotationTarget> {
  TestAnnotationEntry(super.target, [Matrix4? transform])
    : transform = transform ?? Matrix4.identity();

  @override
  final Matrix4 transform;
}
