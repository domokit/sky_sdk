// Copyright 2014 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

/// @docImport 'package:fake_async/fake_async.dart';
/// @docImport 'package:flutter/rendering.dart';
/// @docImport 'package:flutter/widgets.dart';
///
/// @docImport 'recognizer.dart';
library;

import 'dart:async';
import 'dart:collection';
import 'dart:ui' as ui show PointerDataPacket;

import 'package:flutter/foundation.dart';
import 'package:flutter/scheduler.dart';

import 'arena.dart';
import 'converter.dart';
import 'debug.dart';
import 'events.dart';
import 'hit_test.dart';
import 'pointer_router.dart';
import 'pointer_signal_resolver.dart';
import 'resampler.dart';

export 'dart:ui' show Offset;

export 'package:flutter/foundation.dart' show DiagnosticsNode, InformationCollector;

export 'arena.dart' show GestureArenaManager;
export 'events.dart' show PointerEvent;
export 'hit_test.dart' show HitTestEntry, HitTestResult, HitTestTarget;
export 'pointer_router.dart' show PointerRouter;
export 'pointer_signal_resolver.dart' show PointerSignalResolver;

typedef _HandleSampleTimeChangedCallback = void Function();

/// Class that implements clock used for sampling.
class SamplingClock {
  /// Returns current time.
  DateTime now() => DateTime.now();

  /// Returns a new stopwatch that uses the current time as reported by `this`.
  ///
  /// See also:
  ///
  ///   * [GestureBinding.debugSamplingClock], which is used in tests and
  ///     debug builds to observe [FakeAsync].
  Stopwatch stopwatch() => Stopwatch(); // flutter_ignore: stopwatch (see analyze.dart)
  // Ignore context: This is replaced by debugSampling clock in the test binding.
}

// Class that handles resampling of touch events for multiple pointer
// devices.
//
// The `samplingInterval` is used to determine the approximate next
// time for resampling.
// SchedulerBinding's `currentSystemFrameTimeStamp` is used to determine
// sample time.
class _Resampler {
  _Resampler(this._handlePointerEvent, this._handleSampleTimeChanged, this._samplingInterval);

  // Resamplers used to filter incoming pointer events.
  final Map<int, PointerEventResampler> _resamplers = <int, PointerEventResampler>{};

  // Flag to track if a frame callback has been scheduled.
  bool _frameCallbackScheduled = false;

  // Last frame time for resampling.
  Duration _frameTime = Duration.zero;

  // Time since `_frameTime` was updated.
  Stopwatch _frameTimeAge = Stopwatch(); // flutter_ignore: stopwatch (see analyze.dart)
  // Ignore context: This is tested safely outside of FakeAsync.

  // Last sample time and time stamp of last event.
  //
  // Only used for debugPrint of resampling margin.
  Duration _lastSampleTime = Duration.zero;
  Duration _lastEventTime = Duration.zero;

  // Callback used to handle pointer events.
  final HandleEventCallback _handlePointerEvent;

  // Callback used to handle sample time changes.
  final _HandleSampleTimeChangedCallback _handleSampleTimeChanged;

  // Interval used for sampling.
  final Duration _samplingInterval;

  // Timer used to schedule resampling.
  Timer? _timer;

  // Add `event` for resampling or dispatch it directly if
  // not a touch event.
  void addOrDispatch(PointerEvent event) {
    // Add touch event to resampler or dispatch pointer event directly.
    if (event.kind == PointerDeviceKind.touch) {
      // Save last event time for debugPrint of resampling margin.
      _lastEventTime = event.timeStamp;

      final PointerEventResampler resampler = _resamplers.putIfAbsent(
        event.device,
        () => PointerEventResampler(),
      );
      resampler.addEvent(event);
    } else {
      _handlePointerEvent(event);
    }
  }

  // Sample and dispatch events.
  //
  // The `samplingOffset` is relative to the current frame time, which
  // can be in the past when we're not actively resampling.
  //
  // The `samplingClock` is the clock used to determine frame time age.
  void sample(Duration samplingOffset, SamplingClock clock) {
    final SchedulerBinding scheduler = SchedulerBinding.instance;

    // Initialize `_frameTime` if needed. This will be used for periodic
    // sampling when frame callbacks are not received.
    if (_frameTime == Duration.zero) {
      _frameTime = Duration(milliseconds: clock.now().millisecondsSinceEpoch);
      _frameTimeAge = clock.stopwatch()..start();
    }

    // Schedule periodic resampling if `_timer` is not already active.
    if (_timer?.isActive != true) {
      _timer = Timer.periodic(_samplingInterval, (_) => _onSampleTimeChanged());
    }

    // Calculate the effective frame time by taking the number
    // of sampling intervals since last time `_frameTime` was
    // updated into account. This allows us to advance sample
    // time without having to receive frame callbacks.
    final int samplingIntervalUs = _samplingInterval.inMicroseconds;
    final int elapsedIntervals = _frameTimeAge.elapsedMicroseconds ~/ samplingIntervalUs;
    final int elapsedUs = elapsedIntervals * samplingIntervalUs;
    final Duration frameTime = _frameTime + Duration(microseconds: elapsedUs);

    // Determine sample time by adding the offset to the current
    // frame time. This is expected to be in the past and not
    // result in any dispatched events unless we're actively
    // resampling events.
    final Duration sampleTime = frameTime + samplingOffset;

    // Determine next sample time by adding the sampling interval
    // to the current sample time.
    final Duration nextSampleTime = sampleTime + _samplingInterval;

    // Iterate over active resamplers and sample pointer events for
    // current sample time.
    for (final PointerEventResampler resampler in _resamplers.values) {
      resampler.sample(sampleTime, nextSampleTime, _handlePointerEvent);
    }

    // Remove inactive resamplers.
    _resamplers.removeWhere((int key, PointerEventResampler resampler) {
      return !resampler.hasPendingEvents && !resampler.isDown;
    });

    // Save last sample time for debugPrint of resampling margin.
    _lastSampleTime = sampleTime;

    // Early out if another call to `sample` isn't needed.
    if (_resamplers.isEmpty) {
      _timer!.cancel();
      return;
    }

    // Schedule a frame callback if another call to `sample` is needed.
    if (!_frameCallbackScheduled) {
      _frameCallbackScheduled = true;
      // Add a post frame callback as this avoids producing unnecessary
      // frames but ensures that sampling phase is adjusted to frame
      // time when frames are produced.
      scheduler.addPostFrameCallback((_) {
        _frameCallbackScheduled = false;
        // We use `currentSystemFrameTimeStamp` here as it's critical that
        // sample time is in the same clock as the event time stamps, and
        // never adjusted or scaled like `currentFrameTimeStamp`.
        _frameTime = scheduler.currentSystemFrameTimeStamp;
        _frameTimeAge.reset();
        // Reset timer to match phase of latest frame callback.
        _timer?.cancel();
        _timer = Timer.periodic(_samplingInterval, (_) => _onSampleTimeChanged());
        // Trigger an immediate sample time change.
        _onSampleTimeChanged();
      }, debugLabel: 'Resampler.startTimer');
    }
  }

  // Stop all resampling and dispatched any queued events.
  void stop() {
    for (final PointerEventResampler resampler in _resamplers.values) {
      resampler.stop(_handlePointerEvent);
    }
    _resamplers.clear();
    _frameTime = Duration.zero;
    _timer?.cancel();
  }

  void _onSampleTimeChanged() {
    assert(() {
      if (debugPrintResamplingMargin) {
        final Duration resamplingMargin = _lastEventTime - _lastSampleTime;
        debugPrint('$resamplingMargin');
      }
      return true;
    }());
    _handleSampleTimeChanged();
  }
}

// The default sampling offset.
//
// Sampling offset is relative to presentation time. If we produce frames
// 16.667 ms before presentation and input rate is ~60hz, worst case latency
// is 33.334 ms. This however assumes zero latency from the input driver.
// 4.666 ms margin is added for this.
const Duration _defaultSamplingOffset = Duration(milliseconds: -38);

// The sampling interval.
//
// Sampling interval is used to determine the approximate time for subsequent
// sampling. This is used to sample events when frame callbacks are not
// being received and decide if early processing of up and removed events
// is appropriate. 16667 us for 60hz sampling interval.
const Duration _samplingInterval = Duration(microseconds: 16667);

/// A binding for the gesture subsystem.
///
/// ## Lifecycle of pointer events and the gesture arena
///
/// ### [PointerDownEvent]
///
/// When a [PointerDownEvent] is received by the [GestureBinding] (from
/// [dart:ui.PlatformDispatcher.onPointerDataPacket], as interpreted by the
/// [PointerEventConverter]), a [hitTest] is performed to determine which
/// [HitTestTarget] nodes are affected. (Other bindings are expected to
/// implement [hitTest] to defer to [HitTestable] objects. For example, the
/// rendering layer defers to the [RenderView] and the rest of the render object
/// hierarchy.)
///
/// The affected nodes then are given the event to handle ([dispatchEvent] calls
/// [HitTestTarget.handleEvent] for each affected node). If any have relevant
/// [GestureRecognizer]s, they provide the event to them using
/// [GestureRecognizer.addPointer]. This typically causes the recognizer to
/// register with the [PointerRouter] to receive notifications regarding the
/// pointer in question.
///
/// Once the hit test and dispatching logic is complete, the event is then
/// passed to the aforementioned [PointerRouter], which passes it to any objects
/// that have registered interest in that event.
///
/// Finally, the [gestureArena] is closed for the given pointer
/// ([GestureArenaManager.close]), which begins the process of selecting a
/// gesture to win that pointer.
///
/// ### Other events
///
/// A pointer that is [PointerEvent.down] may send further events, such as
/// [PointerMoveEvent], [PointerUpEvent], or [PointerCancelEvent]. These are
/// sent to the same [HitTestTarget] nodes as were found when the
/// [PointerDownEvent] was received (even if they have since been disposed; it is
/// the responsibility of those objects to be aware of that possibility).
///
/// Then, the events are routed to any still-registered entrants in the
/// [PointerRouter]'s table for that pointer.
///
/// When a [PointerUpEvent] is received, the [GestureArenaManager.sweep] method
/// is invoked to force the gesture arena logic to terminate if necessary.
mixin GestureBinding on BindingBase implements HitTestable, HitTestDispatcher, HitTestTarget {
  @override
  void initInstances() {
    super.initInstances();
    _instance = this;
    platformDispatcher.onPointerDataPacket = _handlePointerDataPacket;
  }

  /// The singleton instance of this object.
  ///
  /// Provides access to the features exposed by this mixin. The binding must
  /// be initialized before using this getter; this is typically done by calling
  /// [runApp] or [WidgetsFlutterBinding.ensureInitialized].
  static GestureBinding get instance => BindingBase.checkInstance(_instance);
  static GestureBinding? _instance;

  @override
  void unlocked() {
    super.unlocked();
    _flushPointerEventQueue();
  }

  final Queue<PointerEvent> _pendingPointerEvents = Queue<PointerEvent>();

  void _handlePointerDataPacket(ui.PointerDataPacket packet) {
    // We convert pointer data to logical pixels so that e.g. the touch slop can be
    // defined in a device-independent manner.
    try {
      _pendingPointerEvents.addAll(
        PointerEventConverter.expand(packet.data, _devicePixelRatioForView),
      );
      if (!locked) {
        _flushPointerEventQueue();
      }
    } catch (error, stack) {
      FlutterError.reportError(
        FlutterErrorDetails(
          exception: error,
          stack: stack,
          library: 'gestures library',
          context: ErrorDescription('while handling a pointer data packet'),
        ),
      );
    }
  }

  double? _devicePixelRatioForView(int viewId) {
    return platformDispatcher.view(id: viewId)?.devicePixelRatio;
  }

  /// Dispatch a [PointerCancelEvent] for the given pointer soon.
  ///
  /// The pointer event will be dispatched before the next pointer event and
  /// before the end of the microtask but not within this function call.
  void cancelPointer(int pointer) {
    if (_pendingPointerEvents.isEmpty && !locked) {
      scheduleMicrotask(_flushPointerEventQueue);
    }
    _pendingPointerEvents.addFirst(PointerCancelEvent(pointer: pointer));
  }

  void _flushPointerEventQueue() {
    assert(!locked);

    while (_pendingPointerEvents.isNotEmpty) {
      handlePointerEvent(_pendingPointerEvents.removeFirst());
    }
  }

  /// A router that routes all pointer events received from the engine.
  final PointerRouter pointerRouter = PointerRouter();

  /// The gesture arenas used for disambiguating the meaning of sequences of
  /// pointer events.
  final GestureArenaManager gestureArena = GestureArenaManager();

  /// The resolver used for determining which widget handles a
  /// [PointerSignalEvent].
  final PointerSignalResolver pointerSignalResolver = PointerSignalResolver();

  /// State for all pointers which are currently down.
  ///
  /// This map caches the hit test result done when the pointer goes down
  /// ([PointerDownEvent] and [PointerPanZoomStartEvent]). This hit test result
  /// will be used throughout the entire pointer interaction; that is, the
  /// pointer is seen as pointing to the same place even if it has moved away
  /// until pointer goes up ([PointerUpEvent] and [PointerPanZoomEndEvent]).
  /// This matches the expected gesture interaction with a button, and allows
  /// devices that don't support hovering to perform as few hit tests as
  /// possible.
  ///
  /// On the other hand, hovering requires hit testing on almost every frame.
  /// This is handled in [RendererBinding] and [MouseTracker], and will ignore
  /// the results cached here.
  final Map<int, HitTestResult> _hitTests = <int, HitTestResult>{};

  /// Dispatch an event to the targets found by a hit test on its position.
  ///
  /// This method sends the given event to [dispatchEvent] based on event types:
  ///
  ///  * [PointerDownEvent]s and [PointerSignalEvent]s are dispatched to the
  ///    result of a new [hitTest].
  ///  * [PointerUpEvent]s and [PointerMoveEvent]s are dispatched to the result of hit test of the
  ///    preceding [PointerDownEvent]s.
  ///  * [PointerHoverEvent]s, [PointerAddedEvent]s, and [PointerRemovedEvent]s
  ///    are dispatched without a hit test result.
  void handlePointerEvent(PointerEvent event) {
    assert(!locked);

    if (resamplingEnabled) {
      _resampler.addOrDispatch(event);
      _resampler.sample(samplingOffset, samplingClock);
      return;
    }

    // Stop resampler if resampling is not enabled. This is a no-op if
    // resampling was never enabled.
    _resampler.stop();
    _handlePointerEventImmediately(event);
  }

  void _handlePointerEventImmediately(PointerEvent event) {
    HitTestResult? hitTestResult;
    if (event is PointerDownEvent ||
        event is PointerSignalEvent ||
        event is PointerHoverEvent ||
        event is PointerPanZoomStartEvent) {
      assert(
        !_hitTests.containsKey(event.pointer),
        'Pointer of ${event.toString(minLevel: DiagnosticLevel.debug)} unexpectedly has a HitTestResult associated with it.',
      );
      hitTestResult = HitTestResult();
      hitTestInView(hitTestResult, event.position, event.viewId);
      if (event is PointerDownEvent || event is PointerPanZoomStartEvent) {
        _hitTests[event.pointer] = hitTestResult;
      }
      assert(() {
        if (debugPrintHitTestResults) {
          debugPrint('${event.toString(minLevel: DiagnosticLevel.debug)}: $hitTestResult');
        }
        return true;
      }());
    } else if (event is PointerUpEvent ||
        event is PointerCancelEvent ||
        event is PointerPanZoomEndEvent) {
      hitTestResult = _hitTests.remove(event.pointer);
    } else if (event.down || event is PointerPanZoomUpdateEvent) {
      // Because events that occur with the pointer down (like
      // [PointerMoveEvent]s) should be dispatched to the same place that their
      // initial PointerDownEvent was, we want to re-use the path we found when
      // the pointer went down, rather than do hit detection each time we get
      // such an event.
      hitTestResult = _hitTests[event.pointer];
    }
    assert(() {
      if (debugPrintMouseHoverEvents && event is PointerHoverEvent) {
        debugPrint('$event');
      }
      return true;
    }());
    if (hitTestResult != null || event is PointerAddedEvent || event is PointerRemovedEvent) {
      dispatchEvent(event, hitTestResult);
    }
  }

  /// Determine which [HitTestTarget] objects are located at a given position in
  /// the specified view.
  @override // from HitTestable
  void hitTestInView(HitTestResult result, Offset position, int viewId) {
    result.add(HitTestEntry(this));
  }

  @override // from HitTestable
  @Deprecated(
    'Use hitTestInView and specify the view to hit test. '
    'This feature was deprecated after v3.11.0-20.0.pre.',
  )
  void hitTest(HitTestResult result, Offset position) {
    hitTestInView(result, position, platformDispatcher.implicitView!.viewId);
  }

  /// Dispatch an event to [pointerRouter] and the path of a hit test result.
  ///
  /// The `event` is routed to [pointerRouter]. If the `hitTestResult` is not
  /// null, the event is also sent to every [HitTestTarget] in the entries of the
  /// given [HitTestResult]. Any exceptions from the handlers are caught.
  ///
  /// The `hitTestResult` argument may only be null for [PointerAddedEvent]s or
  /// [PointerRemovedEvent]s.
  @override // from HitTestDispatcher
  @pragma('vm:notify-debugger-on-exception')
  void dispatchEvent(PointerEvent event, HitTestResult? hitTestResult) {
    assert(!locked);
    // No hit test information implies that this is a [PointerAddedEvent] or
    // [PointerRemovedEvent]. These events are specially routed here; other
    // events will be routed through the `handleEvent` below.
    if (hitTestResult == null) {
      assert(event is PointerAddedEvent || event is PointerRemovedEvent);
      try {
        pointerRouter.route(event);
      } catch (exception, stack) {
        FlutterError.reportError(
          FlutterErrorDetailsForPointerEventDispatcher(
            exception: exception,
            stack: stack,
            library: 'gesture library',
            context: ErrorDescription('while dispatching a non-hit-tested pointer event'),
            event: event,
            informationCollector:
                () => <DiagnosticsNode>[
                  DiagnosticsProperty<PointerEvent>(
                    'Event',
                    event,
                    style: DiagnosticsTreeStyle.errorProperty,
                  ),
                ],
          ),
        );
      }
      return;
    }
    for (final HitTestEntry entry in hitTestResult.path) {
      try {
        entry.target.handleEvent(event.transformed(entry.transform), entry);
      } catch (exception, stack) {
        FlutterError.reportError(
          FlutterErrorDetailsForPointerEventDispatcher(
            exception: exception,
            stack: stack,
            library: 'gesture library',
            context: ErrorDescription('while dispatching a pointer event'),
            event: event,
            hitTestEntry: entry,
            informationCollector:
                () => <DiagnosticsNode>[
                  DiagnosticsProperty<PointerEvent>(
                    'Event',
                    event,
                    style: DiagnosticsTreeStyle.errorProperty,
                  ),
                  DiagnosticsProperty<HitTestTarget>(
                    'Target',
                    entry.target,
                    style: DiagnosticsTreeStyle.errorProperty,
                  ),
                ],
          ),
        );
      }
    }
  }

  @override // from HitTestTarget
  void handleEvent(PointerEvent event, HitTestEntry entry) {
    pointerRouter.route(event);
    if (event is PointerDownEvent || event is PointerPanZoomStartEvent) {
      gestureArena.close(event.pointer);
    } else if (event is PointerUpEvent || event is PointerPanZoomEndEvent) {
      gestureArena.sweep(event.pointer);
    } else if (event is PointerSignalEvent) {
      pointerSignalResolver.resolve(event);
    }
  }

  /// Reset states of [GestureBinding].
  ///
  /// This clears the hit test records.
  ///
  /// This is typically called between tests.
  @protected
  void resetGestureBinding() {
    _hitTests.clear();
  }

  void _handleSampleTimeChanged() {
    if (!locked) {
      if (resamplingEnabled) {
        _resampler.sample(samplingOffset, samplingClock);
      } else {
        _resampler.stop();
      }
    }
  }

  /// Overrides the sampling clock for debugging and testing.
  ///
  /// This value is ignored in non-debug builds.
  @protected
  SamplingClock? get debugSamplingClock => null;

  /// Provides access to the current [DateTime] and `StopWatch` objects for
  /// sampling.
  ///
  /// Overridden by [debugSamplingClock] for debug builds and testing. Using
  /// this object under test will maintain synchronization with [FakeAsync].
  SamplingClock get samplingClock {
    SamplingClock value = SamplingClock();
    assert(() {
      final SamplingClock? debugValue = debugSamplingClock;
      if (debugValue != null) {
        value = debugValue;
      }
      return true;
    }());
    return value;
  }

  // Resampler used to filter incoming pointer events when resampling
  // is enabled.
  late final _Resampler _resampler = _Resampler(
    _handlePointerEventImmediately,
    _handleSampleTimeChanged,
    _samplingInterval,
  );

  /// Enable pointer event resampling for touch devices by setting
  /// this to true.
  ///
  /// Resampling results in smoother touch event processing at the
  /// cost of some added latency. Devices with low frequency sensors
  /// or when the frequency is not a multiple of the display frequency
  /// (e.g., 120Hz input and 90Hz display) benefit from this.
  ///
  /// This is typically set during application initialization but
  /// can be adjusted dynamically in case the application only
  /// wants resampling for some period of time.
  bool resamplingEnabled = false;

  /// Offset relative to current frame time that should be used for
  /// resampling. The [samplingOffset] is expected to be negative.
  /// Non-negative [samplingOffset] is allowed but will effectively
  /// disable resampling.
  Duration samplingOffset = _defaultSamplingOffset;
}

/// Variant of [FlutterErrorDetails] with extra fields for the gesture
/// library's binding's pointer event dispatcher ([GestureBinding.dispatchEvent]).
class FlutterErrorDetailsForPointerEventDispatcher extends FlutterErrorDetails {
  /// Creates a [FlutterErrorDetailsForPointerEventDispatcher] object with the given
  /// arguments setting the object's properties.
  ///
  /// The gesture library calls this constructor when catching an exception
  /// that will subsequently be reported using [FlutterError.onError].
  const FlutterErrorDetailsForPointerEventDispatcher({
    required super.exception,
    super.stack,
    super.library,
    super.context,
    this.event,
    this.hitTestEntry,
    super.informationCollector,
    super.silent,
  });

  /// The pointer event that was being routed when the exception was raised.
  final PointerEvent? event;

  /// The hit test result entry for the object whose handleEvent method threw
  /// the exception. May be null if no hit test entry is associated with the
  /// event (e.g. [PointerHoverEvent]s, [PointerAddedEvent]s, and
  /// [PointerRemovedEvent]s).
  ///
  /// The target object itself is given by the [HitTestEntry.target] property of
  /// the hitTestEntry object.
  final HitTestEntry? hitTestEntry;
}
