// Copyright 2014 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

import 'dart:async';
import 'dart:ui' show SemanticsUpdate;

import 'package:flutter/foundation.dart';
import 'package:flutter/gestures.dart';
import 'package:flutter/rendering.dart';
import 'package:flutter/scheduler.dart';
import 'package:flutter/services.dart';
import 'package:flutter_test/flutter_test.dart'
    show EnginePhase, TestDefaultBinaryMessengerBinding, fail;

export 'package:flutter/foundation.dart' show FlutterError, FlutterErrorDetails;
export 'package:flutter_test/flutter_test.dart' show EnginePhase, TestDefaultBinaryMessengerBinding;

class TestRenderingFlutterBinding extends BindingBase
    with
        SchedulerBinding,
        ServicesBinding,
        GestureBinding,
        PaintingBinding,
        SemanticsBinding,
        RendererBinding,
        TestDefaultBinaryMessengerBinding {
  /// Creates a binding for testing rendering library functionality.
  ///
  /// If [onErrors] is not null, it is called if [FlutterError] caught any errors
  /// while drawing the frame. If [onErrors] is null and [FlutterError] caught at least
  /// one error, this function fails the test. A test may override [onErrors] and
  /// inspect errors using [takeFlutterErrorDetails].
  ///
  /// Errors caught between frames will cause the test to fail unless
  /// [FlutterError.onError] has been overridden.
  TestRenderingFlutterBinding({this.onErrors}) {
    FlutterError.onError = (FlutterErrorDetails details) {
      FlutterError.dumpErrorToConsole(details);
      Zone.current.parent!.handleUncaughtError(details.exception, details.stack!);
    };
  }

  /// The current [TestRenderingFlutterBinding], if one has been created.
  ///
  /// Provides access to the features exposed by this binding. The binding must
  /// be initialized before using this getter; this is typically done by calling
  /// [TestRenderingFlutterBinding.ensureInitialized].
  static TestRenderingFlutterBinding get instance => BindingBase.checkInstance(_instance);
  static TestRenderingFlutterBinding? _instance;

  @override
  void initInstances() {
    super.initInstances();
    _instance = this;
    // TODO(goderbauer): Create (fake) window if embedder doesn't provide an implicit view.
    assert(platformDispatcher.implicitView != null);
    _renderView = initRenderView(platformDispatcher.implicitView!);
  }

  @override
  RenderView get renderView => _renderView;
  late RenderView _renderView;

  @override
  PipelineOwner get pipelineOwner => rootPipelineOwner;

  /// Creates a [RenderView] object to be the root of the
  /// [RenderObject] rendering tree, and initializes it so that it
  /// will be rendered when the next frame is requested.
  ///
  /// Called automatically when the binding is created.
  RenderView initRenderView(FlutterView view) {
    final RenderView renderView = RenderView(view: view);
    rootPipelineOwner.rootNode = renderView;
    addRenderView(renderView);
    renderView.prepareInitialFrame();
    return renderView;
  }

  @override
  PipelineOwner createRootPipelineOwner() {
    return PipelineOwner(
      onSemanticsOwnerCreated: () {
        renderView.scheduleInitialSemantics();
      },
      onSemanticsUpdate: (SemanticsUpdate update) {
        renderView.updateSemantics(update);
      },
      onSemanticsOwnerDisposed: () {
        renderView.clearSemantics();
      },
    );
  }

  /// Creates and initializes the binding. This function is
  /// idempotent; calling it a second time will just return the
  /// previously-created instance.
  static TestRenderingFlutterBinding ensureInitialized({VoidCallback? onErrors}) {
    return _instance ?? TestRenderingFlutterBinding(onErrors: onErrors);
  }

  final List<FlutterErrorDetails> _errors = <FlutterErrorDetails>[];

  /// A function called after drawing a frame if [FlutterError] caught any errors.
  ///
  /// This function is expected to inspect these errors and decide whether they
  /// are expected or not. Use [takeFlutterErrorDetails] to take one error at a
  /// time, or [takeAllFlutterErrorDetails] to iterate over all errors.
  VoidCallback? onErrors;

  /// Returns the error least recently caught by [FlutterError] and removes it
  /// from the list of captured errors.
  ///
  /// Returns null if no errors were captures, or if the list was exhausted by
  /// calling this method repeatedly.
  FlutterErrorDetails? takeFlutterErrorDetails() {
    if (_errors.isEmpty) {
      return null;
    }
    return _errors.removeAt(0);
  }

  /// Returns all error details caught by [FlutterError] from least recently caught to
  /// most recently caught, and removes them from the list of captured errors.
  ///
  /// The returned iterable takes errors lazily. If, for example, you iterate over 2
  /// errors, but there are 5 errors total, this binding will still fail the test.
  /// Tests are expected to take and inspect all errors.
  Iterable<FlutterErrorDetails> takeAllFlutterErrorDetails() sync* {
    // sync* and yield are used for lazy evaluation. Otherwise, the list would be
    // drained eagerly and allow a test pass with unexpected errors.
    while (_errors.isNotEmpty) {
      yield _errors.removeAt(0);
    }
  }

  /// Returns all exceptions caught by [FlutterError] from least recently caught to
  /// most recently caught, and removes them from the list of captured errors.
  ///
  /// The returned iterable takes errors lazily. If, for example, you iterate over 2
  /// errors, but there are 5 errors total, this binding will still fail the test.
  /// Tests are expected to take and inspect all errors.
  Iterable<dynamic> takeAllFlutterExceptions() sync* {
    // sync* and yield are used for lazy evaluation. Otherwise, the list would be
    // drained eagerly and allow a test pass with unexpected errors.
    while (_errors.isNotEmpty) {
      yield _errors.removeAt(0).exception;
    }
  }

  EnginePhase phase = EnginePhase.composite;

  /// Pumps a frame and runs its entire life cycle.
  ///
  /// This method runs all of the [SchedulerPhase]s in a frame, this is useful
  /// to test [SchedulerPhase.postFrameCallbacks].
  void pumpCompleteFrame() {
    final FlutterExceptionHandler? oldErrorHandler = FlutterError.onError;
    FlutterError.onError = _errors.add;
    try {
      TestRenderingFlutterBinding.instance.handleBeginFrame(null);
      TestRenderingFlutterBinding.instance.handleDrawFrame();
    } finally {
      FlutterError.onError = oldErrorHandler;
      if (_errors.isNotEmpty) {
        if (onErrors != null) {
          onErrors!();
          if (_errors.isNotEmpty) {
            _errors.forEach(FlutterError.dumpErrorToConsole);
            fail(
              'There are more errors than the test inspected using TestRenderingFlutterBinding.takeFlutterErrorDetails.',
            );
          }
        } else {
          _errors.forEach(FlutterError.dumpErrorToConsole);
          fail('Caught error while rendering frame. See preceding logs for details.');
        }
      }
    }
  }

  @override
  void drawFrame() {
    assert(
      phase != EnginePhase.build,
      'rendering_tester does not support testing the build phase; use flutter_test instead',
    );
    final FlutterExceptionHandler? oldErrorHandler = FlutterError.onError;
    FlutterError.onError = _errors.add;
    try {
      rootPipelineOwner.flushLayout();
      if (phase == EnginePhase.layout) {
        return;
      }
      rootPipelineOwner.flushCompositingBits();
      if (phase == EnginePhase.compositingBits) {
        return;
      }
      rootPipelineOwner.flushPaint();
      if (phase == EnginePhase.paint) {
        return;
      }
      for (final RenderView renderView in renderViews) {
        renderView.compositeFrame();
      }
      if (phase == EnginePhase.composite) {
        return;
      }
      rootPipelineOwner.flushSemantics();
      if (phase == EnginePhase.flushSemantics) {
        return;
      }
      assert(phase == EnginePhase.flushSemantics || phase == EnginePhase.sendSemanticsUpdate);
    } finally {
      FlutterError.onError = oldErrorHandler;
      if (_errors.isNotEmpty) {
        if (onErrors != null) {
          onErrors!();
          if (_errors.isNotEmpty) {
            _errors.forEach(FlutterError.dumpErrorToConsole);
            fail(
              'There are more errors than the test inspected using TestRenderingFlutterBinding.takeFlutterErrorDetails.',
            );
          }
        } else {
          _errors.forEach(FlutterError.dumpErrorToConsole);
          fail('Caught error while rendering frame. See preceding logs for details.');
        }
      }
    }
  }
}

/// Place the box in the render tree, at the given size and with the given
/// alignment on the screen.
///
/// If you've updated `box` and want to lay it out again, use [pumpFrame].
///
/// Once a particular [RenderBox] has been passed to [layout], it cannot easily
/// be put in a different place in the tree or passed to [layout] again, because
/// [layout] places the given object into another [RenderBox] which you would
/// need to unparent it from (but that box isn't itself made available).
///
/// The EnginePhase must not be [EnginePhase.build], since the rendering layer
/// has no build phase.
///
/// If `onErrors` is not null, it is set as
/// [TestRenderingFlutterBinding.onErrors].
void layout(
  RenderBox box, { // If you want to just repump the last box, call pumpFrame().
  BoxConstraints? constraints,
  Alignment alignment = Alignment.center,
  EnginePhase phase = EnginePhase.layout,
  VoidCallback? onErrors,
}) {
  assert(box.parent == null); // We stick the box in another, so you can't reuse it easily, sorry.

  TestRenderingFlutterBinding.instance.renderView.child = null;
  if (constraints != null) {
    box = RenderPositionedBox(
      alignment: alignment,
      child: RenderConstrainedBox(additionalConstraints: constraints, child: box),
    );
  }
  TestRenderingFlutterBinding.instance.renderView.child = box;

  pumpFrame(phase: phase, onErrors: onErrors);
}

/// Pumps a single frame.
///
/// If `onErrors` is not null, it is set as
/// [TestRenderingFlutterBinding.onErrors].
void pumpFrame({EnginePhase phase = EnginePhase.layout, VoidCallback? onErrors}) {
  assert(TestRenderingFlutterBinding.instance.renderView.child != null); // call layout() first!

  if (onErrors != null) {
    TestRenderingFlutterBinding.instance.onErrors = onErrors;
  }

  TestRenderingFlutterBinding.instance.phase = phase;
  TestRenderingFlutterBinding.instance.drawFrame();
}

class TestCallbackPainter extends CustomPainter {
  const TestCallbackPainter({required this.onPaint});

  final VoidCallback onPaint;

  @override
  void paint(Canvas canvas, Size size) {
    onPaint();
  }

  @override
  bool shouldRepaint(TestCallbackPainter oldPainter) => true;
}

class RenderSizedBox extends RenderBox {
  RenderSizedBox(this._size);

  final Size _size;

  @override
  double computeMinIntrinsicWidth(double height) {
    return _size.width;
  }

  @override
  double computeMaxIntrinsicWidth(double height) {
    return _size.width;
  }

  @override
  double computeMinIntrinsicHeight(double width) {
    return _size.height;
  }

  @override
  double computeMaxIntrinsicHeight(double width) {
    return _size.height;
  }

  @override
  bool get sizedByParent => true;

  @override
  void performResize() {
    size = constraints.constrain(_size);
  }

  @override
  void performLayout() {}

  @override
  bool hitTestSelf(Offset position) => true;
}

class FakeTickerProvider implements TickerProvider {
  @override
  Ticker createTicker(TickerCallback onTick, [bool disableAnimations = false]) {
    return FakeTicker();
  }
}

class FakeTicker implements Ticker {
  @override
  bool muted = false;

  @override
  void absorbTicker(Ticker originalTicker) {}

  @override
  String? get debugLabel => null;

  @override
  bool get isActive => throw UnimplementedError();

  @override
  bool get isTicking => throw UnimplementedError();

  @override
  bool get scheduled => throw UnimplementedError();

  @override
  bool get shouldScheduleTick => throw UnimplementedError();

  @override
  void dispose() {}

  @override
  void scheduleTick({bool rescheduling = false}) {}

  @override
  TickerFuture start() {
    throw UnimplementedError();
  }

  @override
  void stop({bool canceled = false}) {}

  @override
  void unscheduleTick() {}

  @override
  String toString({bool debugIncludeStack = false}) => super.toString();

  @override
  DiagnosticsNode describeForError(String name) {
    return DiagnosticsProperty<Ticker>(name, this, style: DiagnosticsTreeStyle.errorProperty);
  }
}

class TestClipPaintingContext extends PaintingContext {
  TestClipPaintingContext() : this._(ContainerLayer());

  TestClipPaintingContext._(this._containerLayer) : super(_containerLayer, Rect.zero);

  final ContainerLayer _containerLayer;

  @override
  ClipRectLayer? pushClipRect(
    bool needsCompositing,
    Offset offset,
    Rect clipRect,
    PaintingContextCallback painter, {
    Clip clipBehavior = Clip.hardEdge,
    ClipRectLayer? oldLayer,
  }) {
    this.clipBehavior = clipBehavior;
    return null;
  }

  Clip clipBehavior = Clip.none;

  @mustCallSuper
  void dispose() {
    _containerLayer.dispose();
  }
}

class TestPushLayerPaintingContext extends PaintingContext {
  TestPushLayerPaintingContext() : super(ContainerLayer(), Rect.zero);

  final List<ContainerLayer> pushedLayers = <ContainerLayer>[];

  @override
  void pushLayer(
    ContainerLayer childLayer,
    PaintingContextCallback painter,
    Offset offset, {
    Rect? childPaintBounds,
  }) {
    pushedLayers.add(childLayer);
    super.pushLayer(childLayer, painter, offset, childPaintBounds: childPaintBounds);
  }
}

// Absorbs errors that don't have "overflowed" in their error details.
void absorbOverflowedErrors() {
  final Iterable<FlutterErrorDetails> errorDetails =
      TestRenderingFlutterBinding.instance.takeAllFlutterErrorDetails();
  final Iterable<FlutterErrorDetails> filtered = errorDetails.where((FlutterErrorDetails details) {
    return !details.toString().contains('overflowed');
  });
  if (filtered.isNotEmpty) {
    filtered.forEach(FlutterError.reportError);
  }
}

// Reports any FlutterErrors.
void expectNoFlutterErrors() {
  final Iterable<FlutterErrorDetails> errorDetails =
      TestRenderingFlutterBinding.instance.takeAllFlutterErrorDetails();
  errorDetails.forEach(FlutterError.reportError);
}

RenderConstrainedBox get box200x200 => RenderConstrainedBox(
  additionalConstraints: const BoxConstraints.tightFor(height: 200.0, width: 200.0),
);
