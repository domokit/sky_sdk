// Copyright 2014 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

/// @docImport 'binding.dart';
library;

import 'dart:collection' show LinkedHashMap;
import 'dart:ui';

import 'package:flutter/foundation.dart';
import 'package:flutter/gestures.dart';
import 'package:flutter/services.dart';

import 'object.dart';

export 'package:flutter/services.dart' show MouseCursor, SystemMouseCursors;

/// Signature for hit testing at the given offset for the specified view.
///
/// It is used by the [MouseTracker] to fetch annotations for the mouse
/// position.
typedef MouseTrackerHitTest = HitTestResult Function(Offset offset, int viewId);

// Various states of a connected mouse device used by [MouseTracker].
class _MouseState {
  _MouseState({required PointerEvent initialEvent}) : _latestEvent = initialEvent;

  // The list of annotations that contains this device.
  //
  // It uses [LinkedHashMap] to keep the insertion order.
  LinkedHashMap<MouseTrackerAnnotation, Matrix4> get annotations => _annotations;
  LinkedHashMap<MouseTrackerAnnotation, Matrix4> _annotations =
      LinkedHashMap<MouseTrackerAnnotation, Matrix4>();

  LinkedHashMap<MouseTrackerAnnotation, Matrix4> replaceAnnotations(
    LinkedHashMap<MouseTrackerAnnotation, Matrix4> value,
  ) {
    final LinkedHashMap<MouseTrackerAnnotation, Matrix4> previous = _annotations;
    _annotations = value;
    return previous;
  }

  // The most recently processed mouse event observed from this device.
  PointerEvent get latestEvent => _latestEvent;
  PointerEvent _latestEvent;

  PointerEvent replaceLatestEvent(PointerEvent value) {
    assert(value.device == _latestEvent.device);
    final PointerEvent previous = _latestEvent;
    _latestEvent = value;
    return previous;
  }

  int get device => latestEvent.device;

  @override
  String toString() {
    final String describeLatestEvent = 'latestEvent: ${describeIdentity(latestEvent)}';
    final String describeAnnotations = 'annotations: [list of ${annotations.length}]';
    return '${describeIdentity(this)}($describeLatestEvent, $describeAnnotations)';
  }
}

// The information in `MouseTracker._handleDeviceUpdate` to provide the details
// of an update of a mouse device.
//
// This class contains the information needed to handle the update that might
// change the state of a mouse device, or the [MouseTrackerAnnotation]s that
// the mouse device is hovering.
@immutable
class _MouseTrackerUpdateDetails with Diagnosticable {
  /// When device update is triggered by a new frame.
  ///
  /// All parameters are required.
  const _MouseTrackerUpdateDetails.byNewFrame({
    required this.lastAnnotations,
    required this.nextAnnotations,
    required PointerEvent this.previousEvent,
  }) : triggeringEvent = null;

  /// When device update is triggered by a pointer event.
  ///
  /// The [lastAnnotations], [nextAnnotations], and [triggeringEvent] are
  /// required.
  const _MouseTrackerUpdateDetails.byPointerEvent({
    required this.lastAnnotations,
    required this.nextAnnotations,
    this.previousEvent,
    required PointerEvent this.triggeringEvent,
  });

  /// The annotations that the device is hovering before the update.
  ///
  /// It is never null.
  final LinkedHashMap<MouseTrackerAnnotation, Matrix4> lastAnnotations;

  /// The annotations that the device is hovering after the update.
  ///
  /// It is never null.
  final LinkedHashMap<MouseTrackerAnnotation, Matrix4> nextAnnotations;

  /// The last event that the device observed before the update.
  ///
  /// If the update is triggered by a frame, the [previousEvent] is never null,
  /// since the pointer must have been added before.
  ///
  /// If the update is triggered by a pointer event, the [previousEvent] is not
  /// null except for cases where the event is the first event observed by the
  /// pointer (which is not necessarily a [PointerAddedEvent]).
  final PointerEvent? previousEvent;

  /// The event that triggered this update.
  ///
  /// It is non-null if and only if the update is triggered by a pointer event.
  final PointerEvent? triggeringEvent;

  /// The pointing device of this update.
  int get device {
    final int result = (previousEvent ?? triggeringEvent)!.device;
    return result;
  }

  /// The last event that the device observed after the update.
  ///
  /// The [latestEvent] is never null.
  PointerEvent get latestEvent {
    final PointerEvent result = triggeringEvent ?? previousEvent!;
    return result;
  }

  @override
  void debugFillProperties(DiagnosticPropertiesBuilder properties) {
    super.debugFillProperties(properties);
    properties.add(IntProperty('device', device));
    properties.add(DiagnosticsProperty<PointerEvent>('previousEvent', previousEvent));
    properties.add(DiagnosticsProperty<PointerEvent>('triggeringEvent', triggeringEvent));
    properties.add(
      DiagnosticsProperty<Map<MouseTrackerAnnotation, Matrix4>>('lastAnnotations', lastAnnotations),
    );
    properties.add(
      DiagnosticsProperty<Map<MouseTrackerAnnotation, Matrix4>>('nextAnnotations', nextAnnotations),
    );
  }
}

/// Tracks the relationship between mouse devices and annotations, and
/// triggers mouse events and cursor changes accordingly.
///
/// The [MouseTracker] tracks the relationship between mouse devices and
/// [MouseTrackerAnnotation], notified by [updateWithEvent] and
/// [updateAllDevices]. At every update, [MouseTracker] triggers the following
/// changes if applicable:
///
///  * Dispatches mouse-related pointer events (pointer enter, hover, and exit).
///  * Changes mouse cursors.
///  * Notifies when [mouseIsConnected] changes.
///
/// This class is a [ChangeNotifier] that notifies its listeners if the value of
/// [mouseIsConnected] changes.
///
/// An instance of [MouseTracker] is owned by the global singleton
/// [RendererBinding].
class MouseTracker extends ChangeNotifier {
  /// Create a mouse tracker.
  ///
  /// The `hitTestInView` is used to find the render objects on a given
  /// position in the specific view. It is typically provided by the
  /// [RendererBinding].
  MouseTracker(MouseTrackerHitTest hitTestInView) : _hitTestInView = hitTestInView;

  final MouseTrackerHitTest _hitTestInView;

  final MouseCursorManager _mouseCursorMixin = MouseCursorManager(SystemMouseCursors.basic);

  // Tracks the state of connected mouse devices.
  //
  // It is the source of truth for the list of connected mouse devices, and
  // consists of two parts:
  //
  //  * The mouse devices that are connected.
  //  * In which annotations each device is contained.
  final Map<int, _MouseState> _mouseStates = <int, _MouseState>{};

  // Used to wrap any procedure that might change `mouseIsConnected`.
  //
  // This method records `mouseIsConnected`, runs `task`, and calls
  // [notifyListeners] at the end if the `mouseIsConnected` has changed.
  void _monitorMouseConnection(VoidCallback task) {
    final bool mouseWasConnected = mouseIsConnected;
    task();
    if (mouseWasConnected != mouseIsConnected) {
      notifyListeners();
    }
  }

  bool _debugDuringDeviceUpdate = false;
  // Used to wrap any procedure that might call `_handleDeviceUpdate`.
  //
  // In debug mode, this method uses `_debugDuringDeviceUpdate` to prevent
  // `_deviceUpdatePhase` being recursively called.
  void _deviceUpdatePhase(VoidCallback task) {
    assert(!_debugDuringDeviceUpdate);
    assert(() {
      _debugDuringDeviceUpdate = true;
      return true;
    }());
    task();
    assert(() {
      _debugDuringDeviceUpdate = false;
      return true;
    }());
  }

  // Whether an observed event might update a device.
  static bool _shouldMarkStateDirty(_MouseState? state, PointerEvent event) {
    if (state == null) {
      return true;
    }
    final PointerEvent lastEvent = state.latestEvent;
    assert(event.device == lastEvent.device);
    // An Added can only follow a Removed, and a Removed can only be followed
    // by an Added.
    assert((event is PointerAddedEvent) == (lastEvent is PointerRemovedEvent));

    // Ignore events that are unrelated to mouse tracking.
    if (event is PointerSignalEvent) {
      return false;
    }
    return lastEvent is PointerAddedEvent ||
        event is PointerRemovedEvent ||
        lastEvent.position != event.position;
  }

  LinkedHashMap<MouseTrackerAnnotation, Matrix4> _hitTestInViewResultToAnnotations(
    HitTestResult result,
  ) {
    final LinkedHashMap<MouseTrackerAnnotation, Matrix4> annotations =
        LinkedHashMap<MouseTrackerAnnotation, Matrix4>();
    for (final HitTestEntry entry in result.path) {
      final Object target = entry.target;
      if (target is MouseTrackerAnnotation) {
        annotations[target] = entry.transform!;
      }
    }
    return annotations;
  }

  // Find the annotations that is hovered by the device of the `state`, and
  // their respective global transform matrices.
  //
  // If the device is not connected or not a mouse, an empty map is returned
  // without calling `hitTest`.
  LinkedHashMap<MouseTrackerAnnotation, Matrix4> _findAnnotations(_MouseState state) {
    final Offset globalPosition = state.latestEvent.position;
    final int device = state.device;
    final int viewId = state.latestEvent.viewId;
    if (!_mouseStates.containsKey(device)) {
      return LinkedHashMap<MouseTrackerAnnotation, Matrix4>();
    }

    return _hitTestInViewResultToAnnotations(_hitTestInView(globalPosition, viewId));
  }

  // A callback that is called on the update of a device.
  //
  // An event (not necessarily a pointer event) that might change the
  // relationship between mouse devices and [MouseTrackerAnnotation]s is called
  // a _device update_. This method should be called at each such update.
  //
  // The update can be caused by two kinds of triggers:
  //
  //  * Triggered by the addition, movement, or removal of a pointer. Such calls
  //    occur during the handler of the event, indicated by
  //    `details.triggeringEvent` being non-null.
  //  * Triggered by the appearance, movement, or disappearance of an annotation.
  //    Such calls occur after each new frame, during the post-frame callbacks,
  //    indicated by `details.triggeringEvent` being null.
  //
  // Calls of this method must be wrapped in `_deviceUpdatePhase`.
  void _handleDeviceUpdate(_MouseTrackerUpdateDetails details) {
    assert(_debugDuringDeviceUpdate);
    _handleDeviceUpdateMouseEvents(details);
    _mouseCursorMixin.handleDeviceCursorUpdate(
      details.device,
      details.triggeringEvent,
      details.nextAnnotations.keys.map((MouseTrackerAnnotation annotation) => annotation.cursor),
    );
  }

  /// Whether or not at least one mouse is connected and has produced events.
  bool get mouseIsConnected => _mouseStates.isNotEmpty;

  /// Perform a device update for one device according to the given new event.
  ///
  /// The [updateWithEvent] is typically called by [RendererBinding] during the
  /// handler of a pointer event. All pointer events should call this method,
  /// and let [MouseTracker] filter which to react to.
  ///
  /// The `hitTestResult` serves as an optional optimization, and is the hit
  /// test result already performed by [RendererBinding] for other gestures. It
  /// can be null, but when it's not null, it should be identical to the result
  /// from directly calling `hitTestInView` given in the constructor (which
  /// means that it should not use the cached result for [PointerMoveEvent]).
  ///
  /// The [updateWithEvent] is one of the two ways of updating mouse
  /// states, the other one being [updateAllDevices].
  void updateWithEvent(PointerEvent event, HitTestResult? hitTestResult) {
    if (event.kind != PointerDeviceKind.mouse && event.kind != PointerDeviceKind.stylus) {
      return;
    }
    if (event is PointerSignalEvent) {
      return;
    }
    final HitTestResult result = switch (event) {
      PointerRemovedEvent() => HitTestResult(),
      _ => hitTestResult ?? _hitTestInView(event.position, event.viewId),
    };
    final int device = event.device;
    final _MouseState? existingState = _mouseStates[device];
    if (!_shouldMarkStateDirty(existingState, event)) {
      return;
    }

    _monitorMouseConnection(() {
      _deviceUpdatePhase(() {
        // Update mouseState to the latest devices that have not been removed,
        // so that [mouseIsConnected], which is decided by `_mouseStates`, is
        // correct during the callbacks.
        if (existingState == null) {
          if (event is PointerRemovedEvent) {
            return;
          }
          _mouseStates[device] = _MouseState(initialEvent: event);
        } else {
          assert(event is! PointerAddedEvent);
          if (event is PointerRemovedEvent) {
            _mouseStates.remove(event.device);
          }
        }
        final _MouseState targetState = _mouseStates[device] ?? existingState!;

        final PointerEvent lastEvent = targetState.replaceLatestEvent(event);
        final LinkedHashMap<MouseTrackerAnnotation, Matrix4> nextAnnotations =
            event is PointerRemovedEvent
                ? LinkedHashMap<MouseTrackerAnnotation, Matrix4>()
                : _hitTestInViewResultToAnnotations(result);
        final LinkedHashMap<MouseTrackerAnnotation, Matrix4> lastAnnotations = targetState
            .replaceAnnotations(nextAnnotations);

        _handleDeviceUpdate(
          _MouseTrackerUpdateDetails.byPointerEvent(
            lastAnnotations: lastAnnotations,
            nextAnnotations: nextAnnotations,
            previousEvent: lastEvent,
            triggeringEvent: event,
          ),
        );
      });
    });
  }

  /// Perform a device update for all detected devices.
  ///
  /// The [updateAllDevices] is typically called during the post frame phase,
  /// indicating a frame has passed and all objects have potentially moved. For
  /// each connected device, the [updateAllDevices] will make a hit test on the
  /// device's last seen position, and check if necessary changes need to be
  /// made.
  ///
  /// The [updateAllDevices] is one of the two ways of updating mouse
  /// states, the other one being [updateWithEvent].
  void updateAllDevices() {
    _deviceUpdatePhase(() {
      for (final _MouseState dirtyState in _mouseStates.values) {
        final PointerEvent lastEvent = dirtyState.latestEvent;
        final LinkedHashMap<MouseTrackerAnnotation, Matrix4> nextAnnotations = _findAnnotations(
          dirtyState,
        );
        final LinkedHashMap<MouseTrackerAnnotation, Matrix4> lastAnnotations = dirtyState
            .replaceAnnotations(nextAnnotations);

        _handleDeviceUpdate(
          _MouseTrackerUpdateDetails.byNewFrame(
            lastAnnotations: lastAnnotations,
            nextAnnotations: nextAnnotations,
            previousEvent: lastEvent,
          ),
        );
      }
    });
  }

  /// Returns the active mouse cursor for a device.
  ///
  /// The return value is the last [MouseCursor] activated onto this device, even
  /// if the activation failed.
  ///
  /// This function is only active when asserts are enabled. In release builds,
  /// it always returns null.
  @visibleForTesting
  MouseCursor? debugDeviceActiveCursor(int device) {
    return _mouseCursorMixin.debugDeviceActiveCursor(device);
  }

  // Handles device update and dispatches mouse event callbacks.
  static void _handleDeviceUpdateMouseEvents(_MouseTrackerUpdateDetails details) {
    final PointerEvent latestEvent = details.latestEvent;

    final LinkedHashMap<MouseTrackerAnnotation, Matrix4> lastAnnotations = details.lastAnnotations;
    final LinkedHashMap<MouseTrackerAnnotation, Matrix4> nextAnnotations = details.nextAnnotations;

    // Order is important for mouse event callbacks. The
    // `_hitTestInViewResultToAnnotations` returns annotations in the visual order
    // from front to back, called the "hit-test order". The algorithm here is
    // explained in https://github.com/flutter/flutter/issues/41420

    // Send exit events to annotations that are in last but not in next, in
    // hit-test order.
    final PointerExitEvent baseExitEvent = PointerExitEvent.fromMouseEvent(latestEvent);
    lastAnnotations.forEach((MouseTrackerAnnotation annotation, Matrix4 transform) {
      if (annotation.validForMouseTracker && !nextAnnotations.containsKey(annotation)) {
        annotation.onExit?.call(baseExitEvent.transformed(lastAnnotations[annotation]));
      }
    });

    // Send enter events to annotations that are not in last but in next, in
    // reverse hit-test order.
    final List<MouseTrackerAnnotation> enteringAnnotations =
        nextAnnotations.keys
            .where((MouseTrackerAnnotation annotation) => !lastAnnotations.containsKey(annotation))
            .toList();
    final PointerEnterEvent baseEnterEvent = PointerEnterEvent.fromMouseEvent(latestEvent);
    for (final MouseTrackerAnnotation annotation in enteringAnnotations.reversed) {
      if (annotation.validForMouseTracker) {
        annotation.onEnter?.call(baseEnterEvent.transformed(nextAnnotations[annotation]));
      }
    }
  }
}
