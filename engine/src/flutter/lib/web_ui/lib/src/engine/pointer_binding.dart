// Copyright 2013 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

import 'dart:async';
import 'dart:js_interop';
import 'dart:math' as math;

import 'package:meta/meta.dart';
import 'package:ui/src/engine/keyboard_binding.dart';
import 'package:ui/ui.dart' as ui;
import 'package:ui/ui_web/src/ui_web.dart' as ui_web;

import '../engine.dart' show registerHotRestartListener;
import 'browser_detection.dart' show isIosSafari;
import 'dom.dart';
import 'platform_dispatcher.dart';
import 'pointer_binding/event_position_helper.dart';
import 'pointer_converter.dart';
import 'safe_browser_api.dart';
import 'semantics.dart';
import 'window.dart';

/// Set this flag to true to log all the browser events.
const bool _debugLogPointerEvents = false;

/// Set this to true to log all the events sent to the Flutter framework.
const bool _debugLogFlutterEvents = false;

/// The signature of a callback that handles pointer events.
typedef _PointerDataCallback = void Function(DomEvent event, List<ui.PointerData>);

// The mask for the bitfield of event buttons. Buttons not contained in this
// mask are cut off.
//
// In Flutter we used `kMaxUnsignedSMI`, but since that value is not available
// here, we use an already very large number (30 bits).
const int _kButtonsMask = 0x3FFFFFFF;

// Assumes the device supports at most one mouse, one touch screen, and one
// trackpad, therefore these pointer events are assigned fixed device IDs.
const int _mouseDeviceId = -1;
const int _trackpadDeviceId = -2;
// For now only one stylus is supported.
//
// Device may support multiple styluses, but `PointerEvent` does not
// distinguish between them with unique identifiers. Additionally, repeated
// touches from the same stylus will be assigned different `pointerId`s each
// time. Since it's really hard to handle, support for multiple styluses is
// left for when demanded.
const int _stylusDeviceId = -4;

const int _kPrimaryMouseButton = 0x1;
const int _kSecondaryMouseButton = 0x2;
const int _kMiddleMouseButton = 0x4;

int _nthButton(int n) => 0x1 << n;

/// Convert the `button` property of PointerEvent to a bit mask of its `buttons`
/// property.
///
/// The `button` property is a integer describing the button changed in an event,
/// which is sequentially 0 for LMB, 1 for MMB, 2 for RMB, 3 for backward and
/// 4 for forward, etc.
///
/// The `buttons` property is a bitfield describing the buttons pressed after an
/// event, which is 0x1 for LMB, 0x4 for MMB, 0x2 for RMB, 0x8 for backward
/// and 0x10 for forward, etc.
@visibleForTesting
int convertButtonToButtons(int button) {
  assert(button >= 0, 'Unexpected negative button $button.');
  switch (button) {
    case 0:
      return _kPrimaryMouseButton;
    case 1:
      return _kMiddleMouseButton;
    case 2:
      return _kSecondaryMouseButton;
    default:
      return _nthButton(button);
  }
}

/// Wrapping the Safari iOS workaround that adds a dummy event listener
/// More info about the issue and workaround: https://github.com/flutter/flutter/issues/70858
class SafariPointerEventWorkaround {
  SafariPointerEventWorkaround._();

  DomEventListener? _listener;

  void workAroundMissingPointerEvents() {
    // We only need to attach the listener once.
    if (_listener == null) {
      _listener = createDomEventListener((_) {});
      domDocument.addEventListener('touchstart', _listener);
    }
  }

  void dispose() {
    if (_listener != null) {
      domDocument.removeEventListener('touchstart', _listener);
      _listener = null;
    }
  }
}

class PointerBinding {
  PointerBinding(
    this.view, {
    PointerSupportDetector detector = const PointerSupportDetector(),
    SafariPointerEventWorkaround? safariWorkaround,
  }) : _pointerDataConverter = PointerDataConverter(),
       _detector = detector {
    if (isIosSafari) {
      _safariWorkaround = safariWorkaround ?? _defaultSafariWorkaround;
      _safariWorkaround!.workAroundMissingPointerEvents();
    }
    _adapter = _createAdapter();
    assert(() {
      registerHotRestartListener(dispose);
      return true;
    }());
  }

  static final SafariPointerEventWorkaround _defaultSafariWorkaround =
      SafariPointerEventWorkaround._();
  static final ClickDebouncer clickDebouncer = ClickDebouncer();

  /// Resets global pointer state that's not tied to any single [PointerBinding]
  /// instance.
  @visibleForTesting
  static void debugResetGlobalState() {
    clickDebouncer.reset();
    PointerDataConverter.globalPointerState.reset();
  }

  SafariPointerEventWorkaround? _safariWorkaround;

  /// Performs necessary clean up for PointerBinding including removing event listeners
  /// and clearing the existing pointer state
  void dispose() {
    _adapter.dispose();
    _safariWorkaround?.dispose();
  }

  final EngineFlutterView view;
  DomElement get rootElement => view.dom.rootElement;

  final PointerSupportDetector _detector;
  final PointerDataConverter _pointerDataConverter;
  KeyboardConverter? _keyboardConverter = KeyboardBinding.instance?.converter;
  late _BaseAdapter _adapter;

  @visibleForTesting
  void debugOverrideKeyboardConverter(KeyboardConverter? keyboardConverter) {
    _keyboardConverter = keyboardConverter;
  }

  _BaseAdapter _createAdapter() {
    if (_detector.hasPointerEvents) {
      return _PointerAdapter(this);
    }
    throw UnsupportedError(
      'This browser does not support pointer events which '
      'are necessary to handle interactions with Flutter Web apps.',
    );
  }
}

@visibleForTesting
typedef QueuedEvent = ({DomEvent event, Duration timeStamp, List<ui.PointerData> data});

@visibleForTesting
typedef DebounceState = ({DomElement target, Timer timer, List<QueuedEvent> queue});

/// Disambiguates taps and clicks that are produced both by the framework from
/// `pointerdown`/`pointerup` events and those detected as DOM "click" events by
/// the browser.
///
/// The implementation is waiting for a `pointerdown`, and as soon as it sees
/// one stops forwarding pointer events to the framework, and instead queues
/// them in a list. The queuing process stops as soon as one of the following
/// two conditions happens first:
///
/// * 200ms passes after the `pointerdown` event. Most clicks, even slow ones,
///   are typically done by then. Importantly, screen readers simulate clicks
///   much faster than 200ms. So if the timer expires, it is likely the user is
///   not interested in producing a click, so the debouncing process stops and
///   all queued events are forwarded to the framework. If, for example, a
///   tappable node is inside a scrollable viewport, the events can be
///   intrepreted by the framework to initiate scrolling.
/// * A `click` event arrives. If the event queue has not been flushed to the
///   framework, the event is forwarded to the framework as a
///   `SemanticsAction.tap`, and all the pointer events are dropped. If, by the
///   time the click event arrives, the queue was flushed (but no more than 50ms
///   ago), then the click event is dropped instead under the assumption that
///   the flushed pointer events are interpreted by the framework as the desired
///   gesture.
///
/// This mechanism is in place to deal with https://github.com/flutter/flutter/issues/130162.
class ClickDebouncer {
  ClickDebouncer() {
    assert(() {
      registerHotRestartListener(reset);
      return true;
    }());
  }

  DebounceState? _state;

  @visibleForTesting
  DebounceState? get debugState => _state;

  // The timestamp of the last "pointerup" DOM event that was sent to the
  // framework.
  //
  // Not to be confused with the time when it was flushed. The two may be far
  // apart because the flushing can happen after a delay due to timer, or events
  // that happen after the said "pointerup".
  Duration? _lastSentPointerUpTimeStamp;

  /// Returns true if the debouncer has a non-empty queue of pointer events that
  /// were withheld from the framework.
  ///
  /// This value is normally false, and it flips to true when the first
  /// pointerdown is observed that lands on a tappable semantics node, denoted
  /// by the presence of the `flt-tappable` attribute.
  bool get isDebouncing => _state != null;

  /// Processes a pointer event.
  ///
  /// If semantics are off, simply forwards the event to the framework.
  ///
  /// If currently debouncing events (see [isDebouncing]), adds the event to
  /// the debounce queue, unless the target of the event is different from the
  /// target that initiated the debouncing process, in which case stops
  /// debouncing and flushes pointer events to the framework.
  ///
  /// If the event is a `pointerdown` and the target is `flt-tappable`, begins
  /// debouncing events.
  ///
  /// In all other situations forwards the event to the framework.
  void onPointerData(DomEvent event, List<ui.PointerData> data) {
    if (!EnginePlatformDispatcher.instance.semanticsEnabled) {
      _sendToFramework(event, data);
      return;
    }

    if (isDebouncing) {
      _debounce(event, data);
    } else if (event.type == 'pointerdown') {
      _startDebouncing(event, data);
    } else {
      if (event.type == 'pointerup') {
        // Record the last pointerup event even if not debouncing. This is
        // because the sequence of pointerdown-pointerup could indicate a
        // long-press, and the debounce timer is not long enough to capture it.
        // If a "click" is observed after a long-press it should be
        // discarded.
        _lastSentPointerUpTimeStamp = _BaseAdapter._eventTimeStampToDuration(event.timeStamp!);
      }
      _sendToFramework(event, data);
    }
  }

  /// Notifies the debouncer of the browser-detected "click" DOM event.
  ///
  /// Forwards the event to the framework, unless it is deduplicated because
  /// the corresponding pointer down/up events were recently flushed to the
  /// framework already.
  void onClick(DomEvent click, int viewId, int semanticsNodeId, bool isListening) {
    assert(click.type == 'click');

    if (!isDebouncing) {
      // There's no pending queue of pointer events that are being debounced. It
      // is a standalone click event. Unless pointer down/up were flushed
      // recently and if the node is currently listening to event, forward to
      // the framework.
      if (isListening && _shouldSendClickEventToFramework(click)) {
        _sendSemanticsTapToFramework(click, viewId, semanticsNodeId);
      }
      return;
    }

    if (isListening) {
      // There's a pending queue of pointer events. Prefer sending the tap action
      // instead of pointer events, because the pointer events may not land on the
      // combined semantic node and miss the click/tap.
      final DebounceState state = _state!;
      _state = null;
      state.timer.cancel();
      _sendSemanticsTapToFramework(click, viewId, semanticsNodeId);
    } else {
      // The semantic node is not listening to taps. Flush the pointer events
      // for the framework to figure out what to do with them. It's possible
      // the framework is interested in gestures other than taps.
      _flush();
    }
  }

  void _sendSemanticsTapToFramework(DomEvent click, int viewId, int semanticsNodeId) {
    // Tappable nodes can be nested inside other tappable nodes. If a click
    // lands on an inner element and is allowed to propagate, it will also
    // land on the ancestor tappable, leading to both the descendant and the
    // ancestor sending SemanticsAction.tap to the framework, creating a double
    // tap/click, which is wrong. More details:
    //
    // https://github.com/flutter/flutter/issues/134842
    click.stopPropagation();

    EnginePlatformDispatcher.instance.invokeOnSemanticsAction(
      viewId,
      semanticsNodeId,
      ui.SemanticsAction.tap,
      null,
    );
    reset();
  }

  void _startDebouncing(DomEvent event, List<ui.PointerData> data) {
    assert(_state == null, 'Cannot start debouncing. Already debouncing.');
    assert(event.type == 'pointerdown', 'Click debouncing must begin with a pointerdown');

    final DomEventTarget? target = event.target;
    if (target is DomElement && target.hasAttribute('flt-tappable')) {
      _state = (
        target: target,
        // The 200ms duration was chosen empirically by testing tapping, mouse
        // clicking, trackpad tapping and clicking, as well as the following
        // screen readers: TalkBack on Android, VoiceOver on macOS, Narrator/
        // NVDA/JAWS on Windows. 200ms seemed to hit the sweet spot by
        // satisfying the following:
        //   * It was short enough that delaying the `pointerdown` still allowed
        //     drag gestures to begin reasonably soon (e.g. scrolling).
        //   * It was long enough to register taps and clicks.
        //   * It was successful at detecting taps generated by all tested
        //     screen readers.
        timer: Timer(const Duration(milliseconds: 200), _onTimerExpired),
        queue: <QueuedEvent>[
          (
            event: event,
            timeStamp: _BaseAdapter._eventTimeStampToDuration(event.timeStamp!),
            data: data,
          ),
        ],
      );
    } else {
      // The event landed on an non-tappable target. Assume this won't lead to
      // double clicks and forward the event to the framework.
      _sendToFramework(event, data);
    }
  }

  void _debounce(DomEvent event, List<ui.PointerData> data) {
    assert(
      _state != null,
      'Cannot debounce event. Debouncing state not established by _startDebouncing.',
    );

    final DebounceState state = _state!;
    state.queue.add((
      event: event,
      timeStamp: _BaseAdapter._eventTimeStampToDuration(event.timeStamp!),
      data: data,
    ));

    // It's only interesting to debounce clicks when both `pointerdown` and
    // `pointerup` land on the same element.
    if (event.type == 'pointerup') {
      final bool targetChanged = event.target != state.target;
      if (targetChanged) {
        _flush();
      }
    }
  }

  void _onTimerExpired() {
    if (!isDebouncing) {
      return;
    }
    _flush();
  }

  // If the click event happens soon after the last `pointerup` event that was
  // already flushed to the framework, the click event is dropped to avoid
  // double click.
  bool _shouldSendClickEventToFramework(DomEvent click) {
    final Duration? lastSentPointerUpTimeStamp = _lastSentPointerUpTimeStamp;

    if (lastSentPointerUpTimeStamp == null) {
      // We haven't seen a pointerup. It's standalone click event. Let it through.
      return true;
    }

    final Duration clickTimeStamp = _BaseAdapter._eventTimeStampToDuration(click.timeStamp!);
    final Duration delta = clickTimeStamp - lastSentPointerUpTimeStamp;
    return delta >= const Duration(milliseconds: 50);
  }

  void _flush() {
    assert(_state != null);

    final DebounceState state = _state!;
    state.timer.cancel();

    final List<ui.PointerData> aggregateData = <ui.PointerData>[];
    for (final QueuedEvent queuedEvent in state.queue) {
      if (queuedEvent.event.type == 'pointerup') {
        _lastSentPointerUpTimeStamp = queuedEvent.timeStamp;
      }
      aggregateData.addAll(queuedEvent.data);
    }

    _sendToFramework(null, aggregateData);
    _state = null;
  }

  void _sendToFramework(DomEvent? event, List<ui.PointerData> data) {
    final ui.PointerDataPacket packet = ui.PointerDataPacket(data: data.toList());
    if (_debugLogFlutterEvents) {
      for (final ui.PointerData datum in data) {
        print('fw:${datum.change}    ${datum.physicalX},${datum.physicalY}');
      }
    }
    EnginePlatformDispatcher.instance.invokeOnPointerDataPacket(packet);
  }

  /// Cancels any pending debounce process and forgets anything that happened so
  /// far.
  ///
  /// This object can be used as if it was just initialized.
  void reset() {
    _state?.timer.cancel();
    _state = null;
    _lastSentPointerUpTimeStamp = null;
  }
}

class PointerSupportDetector {
  const PointerSupportDetector();

  bool get hasPointerEvents => hasJsProperty(domWindow, 'PointerEvent');

  @override
  String toString() => 'pointers:$hasPointerEvents';
}

/// Encapsulates a DomEvent registration so it can be easily unregistered later.
@visibleForTesting
class Listener {
  Listener._({required this.event, required this.target, required this.handler});

  /// Registers a listener for the given `event` on a `target`.
  ///
  /// If `passive` is null uses the default behavior determined by the event
  /// type. If `passive` is true, marks the handler as non-blocking for the
  /// built-in browser behavior. This means the browser will not wait for the
  /// handler to finish execution before performing the default action
  /// associated with this event. If `passive` is false, the browser will wait
  /// for the handler to finish execution before performing the respective
  /// action.
  factory Listener.register({
    required String event,
    required DomEventTarget target,
    required DartDomEventListener handler,
    bool? passive,
  }) {
    final DomEventListener jsHandler = createDomEventListener(handler);

    if (passive == null) {
      target.addEventListener(event, jsHandler);
    } else {
      final Map<String, Object> eventOptions = <String, Object>{'passive': passive};
      target.addEventListenerWithOptions(event, jsHandler, eventOptions);
    }

    final Listener listener = Listener._(event: event, target: target, handler: jsHandler);

    return listener;
  }

  final String event;
  final DomEventTarget target;
  final DomEventListener handler;

  void unregister() {
    target.removeEventListener(event, handler);
  }
}

/// Common functionality that's shared among adapters.
abstract class _BaseAdapter {
  _BaseAdapter(this._owner) {
    setup();
  }

  final PointerBinding _owner;

  EngineFlutterView get _view => _owner.view;
  _PointerDataCallback get _callback => PointerBinding.clickDebouncer.onPointerData;
  PointerDataConverter get _pointerDataConverter => _owner._pointerDataConverter;
  KeyboardConverter? get _keyboardConverter => _owner._keyboardConverter;

  final List<Listener> _listeners = <Listener>[];
  DomWheelEvent? _lastWheelEvent;
  bool _lastWheelEventWasTrackpad = false;
  bool _lastWheelEventAllowedDefault = false;

  DomElement get _viewTarget => _view.dom.rootElement;
  DomEventTarget get _globalTarget => _view.embeddingStrategy.globalEventTarget;

  /// Each subclass is expected to override this method to attach its own event
  /// listeners and convert events into pointer events.
  void setup();

  /// Cleans up all event listeners attached by this adapter.
  void dispose() {
    for (final Listener listener in _listeners) {
      listener.unregister();
    }
    _listeners.clear();
  }

  /// Adds a listener for the given [eventName] to [target].
  ///
  /// Generally speaking, down and leave events should use [_rootElement]
  /// as the [target], while move and up events should use [domWindow]
  /// instead, because the browser doesn't fire the latter two for DOM elements
  /// when the pointer is outside the window.
  void addEventListener(DomEventTarget target, String eventName, DartDomEventListener handler) {
    JSVoid loggedHandler(DomEvent event) {
      if (_debugLogPointerEvents) {
        if (domInstanceOfString(event, 'PointerEvent')) {
          final DomPointerEvent pointerEvent = event as DomPointerEvent;
          final ui.Offset offset = computeEventOffsetToTarget(event, _view);
          print(
            '${pointerEvent.type}    '
            '${offset.dx.toStringAsFixed(1)},'
            '${offset.dy.toStringAsFixed(1)}',
          );
        } else {
          print(event.type);
        }
      }
      // Report the event to semantics. This information is used to debounce
      // browser gestures. Semantics tells us whether it is safe to forward
      // the event to the framework.
      if (EngineSemantics.instance.receiveGlobalEvent(event)) {
        handler(event);
      }
    }

    _listeners.add(Listener.register(event: eventName, target: target, handler: loggedHandler));
  }

  /// Converts a floating number timestamp (in milliseconds) to a [Duration] by
  /// splitting it into two integer components: milliseconds + microseconds.
  static Duration _eventTimeStampToDuration(num milliseconds) {
    final int ms = milliseconds.toInt();
    final int micro = ((milliseconds - ms) * Duration.microsecondsPerMillisecond).toInt();
    return Duration(milliseconds: ms, microseconds: micro);
  }
}

mixin _WheelEventListenerMixin on _BaseAdapter {
  static double? _defaultScrollLineHeight;

  bool _isAcceleratedMouseWheelDelta(num delta, num? wheelDelta) {
    // On macOS, scrolling using a mouse wheel by default uses an acceleration
    // curve, so delta values ramp up and are not at fixed multiples of 120.
    // But in this case, the wheelDelta properties of the event still keep
    // their original values.
    // For all events without this acceleration curve applied, the wheelDelta
    // values are by convention three times greater than the delta values and with
    // the opposite sign.
    if (wheelDelta == null) {
      return false;
    }
    // Account for observed issues with integer truncation by allowing +-1px error.
    return (wheelDelta - (-3 * delta)).abs() > 1;
  }

  bool _isTrackpadEvent(DomWheelEvent event) {
    // This function relies on deprecated and non-standard implementation
    // details. Useful reference material can be found below.
    //
    // https://source.chromium.org/chromium/chromium/src/+/main:ui/events/event.cc
    // https://source.chromium.org/chromium/chromium/src/+/main:ui/events/cocoa/events_mac.mm
    // https://github.com/WebKit/WebKit/blob/main/Source/WebCore/platform/mac/PlatformEventFactoryMac.mm
    // https://searchfox.org/mozilla-central/source/dom/events/WheelEvent.h
    // https://learn.microsoft.com/en-us/windows/win32/inputdev/wm-mousewheel
    if (ui_web.browser.browserEngine == ui_web.BrowserEngine.firefox) {
      // Firefox has restricted the wheelDelta properties, they do not provide
      // enough information to accurately disambiguate trackpad events from mouse
      // wheel events.
      return false;
    }
    if (_isAcceleratedMouseWheelDelta(event.deltaX, event.wheelDeltaX) ||
        _isAcceleratedMouseWheelDelta(event.deltaY, event.wheelDeltaY)) {
      return false;
    }
    if (((event.deltaX % 120 == 0) && (event.deltaY % 120 == 0)) ||
        (((event.wheelDeltaX ?? 1) % 120 == 0) && ((event.wheelDeltaY ?? 1) % 120) == 0)) {
      // While not in any formal web standard, `blink` and `webkit` browsers use
      // a delta of 120 to represent one mouse wheel turn. If both dimensions of
      // the delta are divisible by 120, this event is probably from a mouse.
      // Checking if wheelDeltaX and wheelDeltaY are both divisible by 120
      // catches any macOS accelerated mouse wheel deltas which by random chance
      // are not caught by _isAcceleratedMouseWheelDelta.
      final num deltaXChange = (event.deltaX - (_lastWheelEvent?.deltaX ?? 0)).abs();
      final num deltaYChange = (event.deltaY - (_lastWheelEvent?.deltaY ?? 0)).abs();
      if ((_lastWheelEvent == null) ||
          (deltaXChange == 0 && deltaYChange == 0) ||
          !(deltaXChange < 20 && deltaYChange < 20)) {
        // A trackpad event might by chance have a delta of exactly 120, so
        // make sure this event does not have a similar delta to the previous
        // one before calling it a mouse event.
        if (event.timeStamp != null && _lastWheelEvent?.timeStamp != null) {
          // If the event has a large delta to the previous event, check if
          // it was preceded within 50 milliseconds by a trackpad event. This
          // handles unlucky 120-delta trackpad events during rapid movement.
          final num diffMs = event.timeStamp! - _lastWheelEvent!.timeStamp!;
          if (diffMs < 50 && _lastWheelEventWasTrackpad) {
            return true;
          }
        }
        return false;
      }
    }
    return true;
  }

  List<ui.PointerData> _convertWheelEventToPointerData(DomWheelEvent event) {
    const int domDeltaPixel = 0x00;
    const int domDeltaLine = 0x01;
    const int domDeltaPage = 0x02;

    ui.PointerDeviceKind kind = ui.PointerDeviceKind.mouse;
    int deviceId = _mouseDeviceId;
    if (_isTrackpadEvent(event)) {
      kind = ui.PointerDeviceKind.trackpad;
      deviceId = _trackpadDeviceId;
    }

    // Flutter only supports pixel scroll delta. Convert deltaMode values
    // to pixels.
    double deltaX = event.deltaX;
    double deltaY = event.deltaY;
    switch (event.deltaMode.toInt()) {
      case domDeltaLine:
        _defaultScrollLineHeight ??= _computeDefaultScrollLineHeight();
        deltaX *= _defaultScrollLineHeight!;
        deltaY *= _defaultScrollLineHeight!;
      case domDeltaPage:
        deltaX *= _view.physicalSize.width;
        deltaY *= _view.physicalSize.height;
      case domDeltaPixel:
        if (ui_web.browser.operatingSystem == ui_web.OperatingSystem.macOs) {
          // Safari and Firefox seem to report delta in logical pixels while
          // Chrome uses physical pixels.
          deltaX *= _view.devicePixelRatio;
          deltaY *= _view.devicePixelRatio;
        }
      default:
        break;
    }

    final List<ui.PointerData> data = <ui.PointerData>[];
    final ui.Offset offset = computeEventOffsetToTarget(event, _view);
    bool ignoreCtrlKey = false;
    if (ui_web.browser.operatingSystem == ui_web.OperatingSystem.macOs) {
      ignoreCtrlKey =
          (_keyboardConverter?.keyIsPressed(kPhysicalControlLeft) ?? false) ||
          (_keyboardConverter?.keyIsPressed(kPhysicalControlRight) ?? false);
    }
    if (event.ctrlKey && !ignoreCtrlKey) {
      _pointerDataConverter.convert(
        data,
        viewId: _view.viewId,
        change: ui.PointerChange.hover,
        timeStamp: _BaseAdapter._eventTimeStampToDuration(event.timeStamp!),
        kind: kind,
        signalKind: ui.PointerSignalKind.scale,
        device: deviceId,
        physicalX: offset.dx * _view.devicePixelRatio,
        physicalY: offset.dy * _view.devicePixelRatio,
        buttons: event.buttons!.toInt(),
        pressure: 1.0,
        pressureMax: 1.0,
        scale: math.exp(-deltaY / 200),
      );
    } else {
      _pointerDataConverter.convert(
        data,
        viewId: _view.viewId,
        change: ui.PointerChange.hover,
        timeStamp: _BaseAdapter._eventTimeStampToDuration(event.timeStamp!),
        kind: kind,
        signalKind: ui.PointerSignalKind.scroll,
        device: deviceId,
        physicalX: offset.dx * _view.devicePixelRatio,
        physicalY: offset.dy * _view.devicePixelRatio,
        buttons: event.buttons!.toInt(),
        pressure: 1.0,
        pressureMax: 1.0,
        scrollDeltaX: deltaX,
        scrollDeltaY: deltaY,
        onRespond: ({bool allowPlatformDefault = false}) {
          // Once `allowPlatformDefault` is `true`, never go back to `false`!
          _lastWheelEventAllowedDefault |= allowPlatformDefault;
        },
      );
    }
    _lastWheelEvent = event;
    _lastWheelEventWasTrackpad = kind == ui.PointerDeviceKind.trackpad;
    return data;
  }

  void _addWheelEventListener(DartDomEventListener handler) {
    _listeners.add(
      Listener.register(event: 'wheel', target: _viewTarget, handler: handler, passive: false),
    );
  }

  void _handleWheelEvent(DomEvent event) {
    assert(domInstanceOfString(event, 'WheelEvent'));
    if (_debugLogPointerEvents) {
      print(event.type);
    }
    _lastWheelEventAllowedDefault = false;
    // [ui.PointerData] can set the `_lastWheelEventAllowedDefault` variable
    // to true, when the framework says so. See the implementation of `respond`
    // when creating the PointerData object above.
    _callback(event, _convertWheelEventToPointerData(event as DomWheelEvent));
    // This works because the `_callback` is handled synchronously in the
    // framework, so it's able to modify `_lastWheelEventAllowedDefault`.
    if (!_lastWheelEventAllowedDefault) {
      event.preventDefault();
    }
  }

  /// For browsers that report delta line instead of pixels such as FireFox
  /// compute line height using the default font size.
  ///
  /// Use Firefox to test this code path.
  double _computeDefaultScrollLineHeight() {
    const double kFallbackFontHeight = 16.0;
    final DomHTMLDivElement probe = createDomHTMLDivElement();
    probe.style
      ..fontSize = 'initial'
      ..display = 'none';
    domDocument.body!.append(probe);
    String fontSize = domWindow.getComputedStyle(probe).fontSize;
    double? res;
    if (fontSize.contains('px')) {
      fontSize = fontSize.replaceAll('px', '');
      res = double.tryParse(fontSize);
    }
    probe.remove();
    return res == null ? kFallbackFontHeight : res / 4.0;
  }
}

@immutable
class _SanitizedDetails {
  const _SanitizedDetails({required this.buttons, required this.change});

  final ui.PointerChange change;
  final int buttons;

  @override
  String toString() => '$runtimeType(change: $change, buttons: $buttons)';
}

class _ButtonSanitizer {
  int _pressedButtons = 0;

  /// Transform [DomPointerEvent.buttons] to Flutter's PointerEvent buttons.
  int _htmlButtonsToFlutterButtons(int buttons) {
    // Flutter's button definition conveniently matches that of JavaScript
    // from primary button (0x1) to forward button (0x10), which allows us to
    // avoid transforming it bit by bit.
    return buttons & _kButtonsMask;
  }

  /// Given [DomPointerEvent.button] and [DomPointerEvent.buttons], tries to
  /// infer the correct value for Flutter buttons.
  int _inferDownFlutterButtons(int button, int buttons) {
    if (buttons == 0 && button > -1) {
      // In some cases, the browser sends `buttons:0` in a down event. In such
      // case, we try to infer the value from `button`.
      buttons = convertButtonToButtons(button);
    }
    return _htmlButtonsToFlutterButtons(buttons);
  }

  _SanitizedDetails sanitizeDownEvent({required int button, required int buttons}) {
    // If the pointer is already down, we just send a move event with the new
    // `buttons` value.
    if (_pressedButtons != 0) {
      return sanitizeMoveEvent(buttons: buttons);
    }

    _pressedButtons = _inferDownFlutterButtons(button, buttons);

    return _SanitizedDetails(change: ui.PointerChange.down, buttons: _pressedButtons);
  }

  _SanitizedDetails sanitizeMoveEvent({required int buttons}) {
    final int newPressedButtons = _htmlButtonsToFlutterButtons(buttons);
    // This could happen when the user clicks RMB then moves the mouse quickly.
    // The brower sends a move event with `buttons:2` even though there's no
    // buttons down yet.
    if (_pressedButtons == 0 && newPressedButtons != 0) {
      return _SanitizedDetails(change: ui.PointerChange.hover, buttons: _pressedButtons);
    }

    _pressedButtons = newPressedButtons;

    return _SanitizedDetails(
      change: _pressedButtons == 0 ? ui.PointerChange.hover : ui.PointerChange.move,
      buttons: _pressedButtons,
    );
  }

  _SanitizedDetails? sanitizeMissingRightClickUp({required int buttons}) {
    final int newPressedButtons = _htmlButtonsToFlutterButtons(buttons);
    // This could happen when RMB is clicked and released but no pointerup
    // event was received because context menu was shown.
    if (_pressedButtons != 0 && newPressedButtons == 0) {
      _pressedButtons = 0;
      return _SanitizedDetails(change: ui.PointerChange.up, buttons: _pressedButtons);
    }
    return null;
  }

  _SanitizedDetails? sanitizeLeaveEvent({required int buttons}) {
    final int newPressedButtons = _htmlButtonsToFlutterButtons(buttons);

    // The move event already handles the case where the pointer is currently
    // down, in which case handling the leave event as well is superfluous.
    if (newPressedButtons == 0) {
      _pressedButtons = 0;

      return _SanitizedDetails(change: ui.PointerChange.hover, buttons: _pressedButtons);
    }

    return null;
  }

  _SanitizedDetails? sanitizeUpEvent({required int? buttons}) {
    // The pointer could have been released by a `pointerout` event, in which
    // case `pointerup` should have no effect.
    if (_pressedButtons == 0) {
      return null;
    }

    _pressedButtons = _htmlButtonsToFlutterButtons(buttons ?? 0);

    if (_pressedButtons == 0) {
      // All buttons have been released.
      return _SanitizedDetails(change: ui.PointerChange.up, buttons: _pressedButtons);
    } else {
      // There are still some unreleased buttons, we shouldn't send an up event
      // yet. Instead we send a move event to update the position of the pointer.
      return _SanitizedDetails(change: ui.PointerChange.move, buttons: _pressedButtons);
    }
  }

  _SanitizedDetails sanitizeCancelEvent() {
    _pressedButtons = 0;
    return _SanitizedDetails(change: ui.PointerChange.cancel, buttons: _pressedButtons);
  }
}

typedef _PointerEventListener = dynamic Function(DomPointerEvent event);

/// Adapter class to be used with browsers that support native pointer events.
///
/// For the difference between MouseEvent and PointerEvent, see _MouseAdapter.
class _PointerAdapter extends _BaseAdapter with _WheelEventListenerMixin {
  _PointerAdapter(super.owner);

  final Map<int, _ButtonSanitizer> _sanitizers = <int, _ButtonSanitizer>{};

  @visibleForTesting
  Iterable<int> debugTrackedDevices() => _sanitizers.keys;

  _ButtonSanitizer _ensureSanitizer(int device) {
    return _sanitizers.putIfAbsent(device, () => _ButtonSanitizer());
  }

  _ButtonSanitizer _getSanitizer(int device) {
    assert(_sanitizers[device] != null);
    return _sanitizers[device]!;
  }

  bool _hasSanitizer(int device) {
    return _sanitizers.containsKey(device);
  }

  void _removePointerIfUnhoverable(DomPointerEvent event) {
    if (event.pointerType == 'touch') {
      _sanitizers.remove(event.pointerId);
    }
  }

  void _addPointerEventListener(
    DomEventTarget target,
    String eventName,
    _PointerEventListener handler, {
    bool checkModifiers = true,
  }) {
    addEventListener(target, eventName, (DomEvent event) {
      final DomPointerEvent pointerEvent = event as DomPointerEvent;
      if (checkModifiers) {
        _checkModifiersState(event);
      }
      handler(pointerEvent);
    });
  }

  void _checkModifiersState(DomPointerEvent event) {
    _keyboardConverter?.synthesizeModifiersIfNeeded(
      event.getModifierState('Alt'),
      event.getModifierState('Control'),
      event.getModifierState('Meta'),
      event.getModifierState('Shift'),
      event.timeStamp!,
    );
  }

  @override
  void setup() {
    _addPointerEventListener(_viewTarget, 'pointerdown', (DomPointerEvent event) {
      final int device = _getPointerId(event);
      final List<ui.PointerData> pointerData = <ui.PointerData>[];
      final _ButtonSanitizer sanitizer = _ensureSanitizer(device);
      final _SanitizedDetails? up = sanitizer.sanitizeMissingRightClickUp(
        buttons: event.buttons!.toInt(),
      );
      if (up != null) {
        _convertEventsToPointerData(data: pointerData, event: event, details: up);
      }
      final _SanitizedDetails down = sanitizer.sanitizeDownEvent(
        button: event.button.toInt(),
        buttons: event.buttons!.toInt(),
      );
      _convertEventsToPointerData(data: pointerData, event: event, details: down);
      _callback(event, pointerData);

      if (event.target == _viewTarget) {
        // Ensure smooth focus transitions between text fields within the Flutter view.
        // Without preventing the default and this delay, the engine may not have fully
        // rendered the next input element, leading to the focus incorrectly returning to
        // the main Flutter view instead.
        // A zero-length timer is sufficient in all tested browsers to achieve this.
        event.preventDefault();
        Timer(Duration.zero, () {
          EnginePlatformDispatcher.instance.requestViewFocusChange(
            viewId: _view.viewId,
            state: ui.ViewFocusState.focused,
            direction: ui.ViewFocusDirection.undefined,
          );
        });
      }
    });

    // Move event listeners should be added to `_globalTarget` instead of
    // `_viewTarget`. This is because `_viewTarget` (the root) captures pointers
    // by default, meaning a pointer that starts within `_viewTarget` continues
    // sending move events to its listener even when dragged outside.
    //
    // In contrast, `_globalTarget` (a regular <div>) stops sending move events
    // when the pointer moves outside its bounds and resumes them only when the
    // pointer re-enters.
    //
    // For demonstration, see this fiddle: https://jsfiddle.net/ditman/7towxaqp
    //
    // TODO(dkwingsmt): Investigate whether we can configure the behavior for
    // `_viewTarget`. https://github.com/flutter/flutter/issues/157968
    _addPointerEventListener(_globalTarget, 'pointermove', (DomPointerEvent moveEvent) {
      final int device = _getPointerId(moveEvent);
      final _ButtonSanitizer sanitizer = _ensureSanitizer(device);
      final List<ui.PointerData> pointerData = <ui.PointerData>[];
      final List<DomPointerEvent> expandedEvents = _expandEvents(moveEvent);
      for (final DomPointerEvent event in expandedEvents) {
        final _SanitizedDetails? up = sanitizer.sanitizeMissingRightClickUp(
          buttons: event.buttons!.toInt(),
        );
        if (up != null) {
          _convertEventsToPointerData(
            data: pointerData,
            event: event,
            details: up,
            pointerId: device,
            eventTarget: moveEvent.target,
          );
        }
        final _SanitizedDetails move = sanitizer.sanitizeMoveEvent(buttons: event.buttons!.toInt());
        _convertEventsToPointerData(
          data: pointerData,
          event: event,
          details: move,
          pointerId: device,
          eventTarget: moveEvent.target,
        );
      }
      _callback(moveEvent, pointerData);
    });

    _addPointerEventListener(_viewTarget, 'pointerleave', (DomPointerEvent event) {
      final int device = _getPointerId(event);
      final _ButtonSanitizer sanitizer = _ensureSanitizer(device);
      final List<ui.PointerData> pointerData = <ui.PointerData>[];
      final _SanitizedDetails? details = sanitizer.sanitizeLeaveEvent(
        buttons: event.buttons!.toInt(),
      );
      if (details != null) {
        _convertEventsToPointerData(data: pointerData, event: event, details: details);
        _callback(event, pointerData);
      }
    }, checkModifiers: false);

    // TODO(dit): This must happen in the flutterViewElement, https://github.com/flutter/flutter/issues/116561
    _addPointerEventListener(_globalTarget, 'pointerup', (DomPointerEvent event) {
      final int device = _getPointerId(event);
      if (_hasSanitizer(device)) {
        final List<ui.PointerData> pointerData = <ui.PointerData>[];
        final _SanitizedDetails? details = _getSanitizer(
          device,
        ).sanitizeUpEvent(buttons: event.buttons?.toInt());
        _removePointerIfUnhoverable(event);
        if (details != null) {
          _convertEventsToPointerData(data: pointerData, event: event, details: details);
          _callback(event, pointerData);
        }
      }
    });

    // TODO(dit): Synthesize a "cancel" event when 'pointerup' happens outside of the flutterViewElement, https://github.com/flutter/flutter/issues/116561

    // A browser fires cancel event if it concludes the pointer will no longer
    // be able to generate events (example: device is deactivated)
    _addPointerEventListener(_viewTarget, 'pointercancel', (DomPointerEvent event) {
      final int device = _getPointerId(event);
      if (_hasSanitizer(device)) {
        final List<ui.PointerData> pointerData = <ui.PointerData>[];
        final _SanitizedDetails details = _getSanitizer(device).sanitizeCancelEvent();
        _removePointerIfUnhoverable(event);
        _convertEventsToPointerData(data: pointerData, event: event, details: details);
        _callback(event, pointerData);
      }
    }, checkModifiers: false);

    _addWheelEventListener((DomEvent event) {
      _handleWheelEvent(event);
    });
  }

  // For each event that is de-coalesced from `event` and described in
  // `details`, convert it to pointer data and store in `data`.
  void _convertEventsToPointerData({
    required List<ui.PointerData> data,
    required DomPointerEvent event,
    required _SanitizedDetails details,
    // `pointerId` and `eventTarget` are optional but useful when it's not
    // desired to get those values from the event object. For example, when the
    // event is a coalesced event.
    int? pointerId,
    DomEventTarget? eventTarget,
  }) {
    final ui.PointerDeviceKind kind = _pointerTypeToDeviceKind(event.pointerType!);
    final double tilt = _computeHighestTilt(event);
    final Duration timeStamp = _BaseAdapter._eventTimeStampToDuration(event.timeStamp!);
    final num? pressure = event.pressure;
    final ui.Offset offset = computeEventOffsetToTarget(event, _view, eventTarget: eventTarget);
    _pointerDataConverter.convert(
      data,
      viewId: _view.viewId,
      change: details.change,
      timeStamp: timeStamp,
      kind: kind,
      signalKind: ui.PointerSignalKind.none,
      device: pointerId ?? _getPointerId(event),
      physicalX: offset.dx * _view.devicePixelRatio,
      physicalY: offset.dy * _view.devicePixelRatio,
      buttons: details.buttons,
      pressure: pressure == null ? 0.0 : pressure.toDouble(),
      pressureMax: 1.0,
      tilt: tilt,
    );
  }

  List<DomPointerEvent> _expandEvents(DomPointerEvent event) {
    // For browsers that don't support `getCoalescedEvents`, we fallback to
    // using the original event.
    if (hasJsProperty(event, 'getCoalescedEvents')) {
      final List<DomPointerEvent> coalescedEvents =
          event.getCoalescedEvents().cast<DomPointerEvent>();
      // Some events don't perform coalescing, so they return an empty list. In
      // that case, we also fallback to using the original event.
      if (coalescedEvents.isNotEmpty) {
        return coalescedEvents;
      }
    }
    // Important: coalesced events lack the `eventTarget` property (because they're
    // being handled in a deferred way).
    //
    // See the "Note" here: https://developer.mozilla.org/en-US/docs/Web/API/Event/currentTarget
    return <DomPointerEvent>[event];
  }

  ui.PointerDeviceKind _pointerTypeToDeviceKind(String pointerType) {
    switch (pointerType) {
      case 'mouse':
        return ui.PointerDeviceKind.mouse;
      case 'pen':
        return ui.PointerDeviceKind.stylus;
      case 'touch':
        return ui.PointerDeviceKind.touch;
      default:
        return ui.PointerDeviceKind.unknown;
    }
  }

  int _getPointerId(DomPointerEvent event) {
    // All mouse pointer events are given `_mouseDeviceId`, including wheel
    // events, because wheel events might come before any other PointerEvents,
    // and wheel PointerEvents don't contain pointerIds.
    return switch (_pointerTypeToDeviceKind(event.pointerType!)) {
      ui.PointerDeviceKind.mouse => _mouseDeviceId,

      ui.PointerDeviceKind.stylus || ui.PointerDeviceKind.invertedStylus => _stylusDeviceId,

      // Trackpad processing doesn't call this function.
      ui.PointerDeviceKind.trackpad => throw Exception('Unreachable'),

      ui.PointerDeviceKind.touch || ui.PointerDeviceKind.unknown => event.pointerId!.toInt(),
    };
  }

  /// Tilt angle is -90 to + 90. Take maximum deflection and convert to radians.
  double _computeHighestTilt(DomPointerEvent e) =>
      (e.tiltX!.abs() > e.tiltY!.abs() ? e.tiltX : e.tiltY)! / 180.0 * math.pi;
}
