// Copyright 2013 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

import 'dart:async';
import 'dart:js_interop';

import 'package:meta/meta.dart';
import 'package:ui/ui.dart' as ui;

import 'dom.dart';
import 'frame_timing_recorder.dart';
import 'platform_dispatcher.dart';

/// Provides frame scheduling functionality and frame lifecycle information to
/// all of the web engine.
///
/// If new frame-related functionality needs to be added to the web engine,
/// prefer to add it here instead of implementing it ad hoc.
class FrameService {
  /// The singleton instance of the [FrameService] used to schedule frames.
  ///
  /// This may be overridden in tests, for example, to pump fake frames, using
  /// [debugOverrideFrameService].
  static FrameService get instance => _instance ??= FrameService();
  static FrameService? _instance;

  /// Overrides the value returned by [instance].
  ///
  /// If [mock] is null, resets the value of [instance] back to real
  /// implementation.
  ///
  /// This is intended for tests only.
  @visibleForTesting
  static void debugOverrideFrameService(FrameService? mock) {
    _instance = mock;
  }

  /// A monotonically increasing frame number being rendered.
  ///
  /// This is intended for tests only.
  int get debugFrameNumber => _debugFrameNumber;
  int _debugFrameNumber = 0;

  /// Resets [debugFrameNumber] back to zero.
  ///
  /// This is intended for tests only.
  @visibleForTesting
  void debugResetFrameNumber() {
    _debugFrameNumber = 0;
  }

  /// Whether a frame has already been scheduled.
  ///
  /// If this value is currently true, then calling [scheduleFrame] has no effect.
  bool get isFrameScheduled => _isFrameScheduled;
  bool _isFrameScheduled = false;

  /// Whether the engine and framework are in the middle of rendering a frame.
  ///
  /// Some DOM events can be triggered synchronously with DOM mutations, such as
  /// the DOM "focus" event. Handlers of such events may wish to be aware of the
  /// fact that the engine is actively rendering a frame. This is especially
  /// true for DOM event handlers that send notifications to the framework. It
  /// goes against the framework's design to receive events that lead to widget
  /// state changes invalidating the current frame. That must be done in the
  /// next frame.
  ///
  /// DOM event handlers whose notifications to the framework result in state
  /// changes may want to delay their notifications, e.g. by scheduling them in
  /// a timer.
  bool get isRenderingFrame => _isRenderingFrame;
  bool _isRenderingFrame = false;

  /// If not null, called immediately and synchronously after rendering a frame.
  ///
  /// At the time this callback is called, the framework completed responding to
  /// `onBeginFrame` and `onDrawFrame`, and [isRenderingFrame] is set to false.
  ///
  /// Any microtasks scheduled while rendering the frame execute after this
  /// callback.
  ui.VoidCallback? onFinishedRenderingFrame;

  void scheduleFrame() {
    // A frame is already scheduled. Do nothing.
    if (_isFrameScheduled) {
      return;
    }

    _isFrameScheduled = true;

    domWindow.requestAnimationFrame((JSNumber highResTime) {
      // Reset immediately for two reasons:
      //
      // * While drawing a frame the framework may attempt to schedule a new
      //   frame, e.g. when there's a continuous animation.
      // * If this value is stuck in `true` state, there will be no way to
      //   schedule new frames and the app will freeze. It is therefore the
      //   safest to reset this value before running any significant amount of
      //   functionality that may throw exceptions, or produce wasm traps.
      _isFrameScheduled = false;

      try {
        _isRenderingFrame = true;
        _debugFrameNumber += 1;
        _renderFrame(highResTime.toDartDouble);
      } finally {
        _isRenderingFrame = false;
        onFinishedRenderingFrame?.call();
      }
    });
  }

  /// The framework has special handling for the warm-up frame. It uses timers,
  /// ensures that there's no regular frame scheduling happening before or
  /// between timers. So this logic here trusts the the framework fulfills its
  /// promises. For example, there's no check if _isFrameScheduled is already
  /// true. The assumption that no prior frames were scheduled.
  void scheduleWarmUpFrame({
    required ui.VoidCallback beginFrame,
    required ui.VoidCallback drawFrame,
  }) {
    // A note from dkwingsmt:
    //
    // We use timers here to ensure that microtasks flush in between.
    //
    // TODO(dkwingsmt): This logic was moved from the framework and is different
    // from how Web renders a regular frame, which doesn't flush microtasks
    // between the callbacks at all (see `initializeEngineServices`). We might
    // want to change this. See the to-do in `initializeEngineServices` and
    // https://github.com/flutter/engine/pull/50570#discussion_r1496671676

    Timer.run(() {
      _isRenderingFrame = true;
      _debugFrameNumber += 1;
      // TODO(yjbanov): it's funky that if beginFrame crashes, the drawFrame
      //                fires anyway. We should clean this up, or better explain
      //                what the expectations are for various situations. The
      //                "we did this before so let's continue doing it" excuse
      //                only works so far (referring to the discussion linked
      //                above).
      try {
        beginFrame();
      } finally {
        _isRenderingFrame = false;
      }
    });

    Timer.run(() {
      _isRenderingFrame = true;
      try {
        drawFrame();
      } finally {
        _isRenderingFrame = false;
        onFinishedRenderingFrame?.call();
      }
    });
  }

  void _renderFrame(double highResTime) {
    FrameTimingRecorder.recordCurrentFrameVsync();

    // In Flutter terminology "building a frame" consists of "beginning
    // frame" and "drawing frame".
    //
    // We do not call `recordBuildFinish` from here because
    // part of the rasterization process, particularly in the HTML
    // renderer, takes place in the `SceneBuilder.build()`.
    FrameTimingRecorder.recordCurrentFrameBuildStart();

    // We have to convert high-resolution time to `int` so we can construct
    // a `Duration` out of it. However, high-res time is supplied in
    // milliseconds as a double value, with sub-millisecond information
    // hidden in the fraction. So we first multiply it by 1000 to uncover
    // microsecond precision, and only then convert to `int`.
    final int highResTimeMicroseconds = (1000 * highResTime).toInt();

    if (EnginePlatformDispatcher.instance.onBeginFrame != null) {
      EnginePlatformDispatcher.instance.invokeOnBeginFrame(
        Duration(microseconds: highResTimeMicroseconds),
      );
    }

    if (EnginePlatformDispatcher.instance.onDrawFrame != null) {
      // On mobile Flutter flushes microtasks between onBeginFrame and
      // onDrawFrame. The web doesn't because there's no way to hook into the
      // event loop, which is controlled by the browser (mobile Flutter hooks
      // into the event loop using C++ code behind-the-scenes). This hasn't
      // been an issue yet. However, if in the future someone can find a way
      // to implement it exactly like mobile does, that would be great.
      //
      // (Also see the to-do in
      //                `EnginePlatformDispatcher.scheduleWarmUpFrame`).
      EnginePlatformDispatcher.instance.invokeOnDrawFrame();
    }
  }
}
