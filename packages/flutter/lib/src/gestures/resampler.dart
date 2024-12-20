// Copyright 2014 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

import 'dart:collection';

import 'events.dart';

export 'events.dart' show PointerEvent;

/// A callback used by [PointerEventResampler.sample] and
/// [PointerEventResampler.stop] to process a resampled `event`.
typedef HandleEventCallback = void Function(PointerEvent event);

/// Class for pointer event resampling.
///
/// An instance of this class can be used to resample one sequence
/// of pointer events. Multiple instances are expected to be used for
/// multi-touch support. The sampling frequency and the sampling
/// offset is determined by the caller.
///
/// This can be used to get smooth touch event processing at the cost
/// of adding some latency. Devices with low frequency sensors or when
/// the frequency is not a multiple of the display frequency
/// (e.g., 120Hz input and 90Hz display) benefit from this.
///
/// The following pointer event types are supported:
/// [PointerAddedEvent], [PointerHoverEvent], [PointerDownEvent],
/// [PointerMoveEvent], [PointerCancelEvent], [PointerUpEvent],
/// [PointerRemovedEvent].
///
/// Resampling is currently limited to event position and delta. All
/// pointer event types except [PointerAddedEvent] will be resampled.
/// [PointerHoverEvent] and [PointerMoveEvent] will only be generated
/// when the position has changed.
class PointerEventResampler {
  // Events queued for processing.
  final Queue<PointerEvent> _queuedEvents = Queue<PointerEvent>();

  // Pointer state required for resampling.
  PointerEvent? _last;
  PointerEvent? _next;
  Offset _position = Offset.zero;
  bool _isTracked = false;
  bool _isDown = false;
  int _pointerIdentifier = 0;
  int _hasButtons = 0;

  PointerEvent _toHoverEvent(
    PointerEvent event,
    Offset position,
    Offset delta,
    Duration timeStamp,
    int buttons,
  ) {
    return PointerHoverEvent(
      viewId: event.viewId,
      timeStamp: timeStamp,
      kind: event.kind,
      device: event.device,
      position: position,
      delta: delta,
      buttons: event.buttons,
      obscured: event.obscured,
      pressureMin: event.pressureMin,
      pressureMax: event.pressureMax,
      distance: event.distance,
      distanceMax: event.distanceMax,
      size: event.size,
      radiusMajor: event.radiusMajor,
      radiusMinor: event.radiusMinor,
      radiusMin: event.radiusMin,
      radiusMax: event.radiusMax,
      orientation: event.orientation,
      tilt: event.tilt,
      synthesized: event.synthesized,
      embedderId: event.embedderId,
    );
  }

  PointerEvent _toMoveEvent(
    PointerEvent event,
    Offset position,
    Offset delta,
    int pointerIdentifier,
    Duration timeStamp,
    int buttons,
  ) {
    return PointerMoveEvent(
      viewId: event.viewId,
      timeStamp: timeStamp,
      pointer: pointerIdentifier,
      kind: event.kind,
      device: event.device,
      position: position,
      delta: delta,
      buttons: buttons,
      obscured: event.obscured,
      pressure: event.pressure,
      pressureMin: event.pressureMin,
      pressureMax: event.pressureMax,
      distanceMax: event.distanceMax,
      size: event.size,
      radiusMajor: event.radiusMajor,
      radiusMinor: event.radiusMinor,
      radiusMin: event.radiusMin,
      radiusMax: event.radiusMax,
      orientation: event.orientation,
      tilt: event.tilt,
      platformData: event.platformData,
      synthesized: event.synthesized,
      embedderId: event.embedderId,
    );
  }

  PointerEvent _toMoveOrHoverEvent(
    PointerEvent event,
    Offset position,
    Offset delta,
    int pointerIdentifier,
    Duration timeStamp,
    bool isDown,
    int buttons,
  ) {
    return isDown
        ? _toMoveEvent(event, position, delta, pointerIdentifier, timeStamp, buttons)
        : _toHoverEvent(event, position, delta, timeStamp, buttons);
  }

  Offset _positionAt(Duration sampleTime) {
    // Use `next` position by default.
    double x = _next?.position.dx ?? 0.0;
    double y = _next?.position.dy ?? 0.0;

    final Duration nextTimeStamp = _next?.timeStamp ?? Duration.zero;
    final Duration lastTimeStamp = _last?.timeStamp ?? Duration.zero;

    // Resample if `next` time stamp is past `sampleTime`.
    if (nextTimeStamp > sampleTime && nextTimeStamp > lastTimeStamp) {
      final double interval = (nextTimeStamp - lastTimeStamp).inMicroseconds.toDouble();
      final double scalar = (sampleTime - lastTimeStamp).inMicroseconds.toDouble() / interval;
      final double lastX = _last?.position.dx ?? 0.0;
      final double lastY = _last?.position.dy ?? 0.0;
      x = lastX + (x - lastX) * scalar;
      y = lastY + (y - lastY) * scalar;
    }

    return Offset(x, y);
  }

  void _processPointerEvents(Duration sampleTime) {
    final Iterator<PointerEvent> it = _queuedEvents.iterator;
    while (it.moveNext()) {
      final PointerEvent event = it.current;

      // Update both `last` and `next` pointer event if time stamp is older
      // or equal to `sampleTime`.
      if (event.timeStamp <= sampleTime || _last == null) {
        _last = event;
        _next = event;
        continue;
      }

      // Update only `next` pointer event if time stamp is more recent than
      // `sampleTime` and next event is not already more recent.
      final Duration nextTimeStamp = _next?.timeStamp ?? Duration.zero;
      if (nextTimeStamp < sampleTime) {
        _next = event;
        break;
      }
    }
  }

  void _dequeueAndSampleNonHoverOrMovePointerEventsUntil(
    Duration sampleTime,
    Duration nextSampleTime,
    HandleEventCallback callback,
  ) {
    Duration endTime = sampleTime;
    // Scan queued events to determine end time.
    final Iterator<PointerEvent> it = _queuedEvents.iterator;
    while (it.moveNext()) {
      final PointerEvent event = it.current;

      // Potentially stop dispatching events if more recent than `sampleTime`.
      if (event.timeStamp > sampleTime) {
        // Definitely stop if more recent than `nextSampleTime`.
        if (event.timeStamp >= nextSampleTime) {
          break;
        }

        // Update `endTime` to allow early processing of up and removed
        // events as this improves resampling of these events, which is
        // important for fling animations.
        if (event is PointerUpEvent || event is PointerRemovedEvent) {
          endTime = event.timeStamp;
          continue;
        }

        // Stop if event is not move or hover.
        if (event is! PointerMoveEvent && event is! PointerHoverEvent) {
          break;
        }
      }
    }

    while (_queuedEvents.isNotEmpty) {
      final PointerEvent event = _queuedEvents.first;

      // Stop dispatching events if more recent than `endTime`.
      if (event.timeStamp > endTime) {
        break;
      }

      final bool wasTracked = _isTracked;
      final bool wasDown = _isDown;
      final int hadButtons = _hasButtons;

      // Update pointer state.
      _isTracked = event is! PointerRemovedEvent;
      _isDown = event.down;
      _hasButtons = event.buttons;

      // Position at `sampleTime`.
      final Offset position = _positionAt(sampleTime);

      // Initialize position if we are starting to track this pointer.
      if (_isTracked && !wasTracked) {
        _position = position;
      }

      // Current pointer identifier.
      final int pointerIdentifier = event.pointer;

      // Initialize pointer identifier for `move` events.
      // Identifier is expected to be the same while `down`.
      assert(!wasDown || _pointerIdentifier == pointerIdentifier);
      _pointerIdentifier = pointerIdentifier;

      // Skip `move` and `hover` events as they are automatically
      // generated when the position has changed.
      if (event is! PointerMoveEvent && event is! PointerHoverEvent) {
        // Add synthetics `move` or `hover` event if position has changed.
        //
        // Devices without `hover` events are expected to always have
        // `add` and `down` events with the same position and this logic will
        // therefore never produce `hover` events.
        if (position != _position) {
          final Offset delta = position - _position;
          callback(
            _toMoveOrHoverEvent(
              event,
              position,
              delta,
              _pointerIdentifier,
              sampleTime,
              wasDown,
              hadButtons,
            ),
          );
          _position = position;
        }
        callback(
          event.copyWith(
            position: position,
            delta: Offset.zero,
            pointer: pointerIdentifier,
            timeStamp: sampleTime,
          ),
        );
      }

      _queuedEvents.removeFirst();
    }
  }

  void _samplePointerPosition(Duration sampleTime, HandleEventCallback callback) {
    // Position at `sampleTime`.
    final Offset position = _positionAt(sampleTime);

    // Add `move` or `hover` events if position has changed.
    final PointerEvent? next = _next;
    if (position != _position && next != null) {
      final Offset delta = position - _position;
      callback(
        _toMoveOrHoverEvent(
          next,
          position,
          delta,
          _pointerIdentifier,
          sampleTime,
          _isDown,
          _hasButtons,
        ),
      );
      _position = position;
    }
  }

  /// Enqueue pointer `event` for resampling.
  void addEvent(PointerEvent event) {
    _queuedEvents.add(event);
  }

  /// Dispatch resampled pointer events for the specified `sampleTime`
  /// by calling [callback].
  ///
  /// This may dispatch multiple events if position is not the only
  /// state that has changed since last sample.
  ///
  /// Calling [callback] must not add or sample events.
  ///
  /// Positive value for `nextSampleTime` allow early processing of
  /// up and removed events. This improves resampling of these events,
  /// which is important for fling animations.
  void sample(Duration sampleTime, Duration nextSampleTime, HandleEventCallback callback) {
    _processPointerEvents(sampleTime);

    // Dequeue and sample pointer events until `sampleTime`.
    _dequeueAndSampleNonHoverOrMovePointerEventsUntil(sampleTime, nextSampleTime, callback);

    // Dispatch resampled pointer location event if tracked.
    if (_isTracked) {
      _samplePointerPosition(sampleTime, callback);
    }
  }

  /// Stop resampling.
  ///
  /// This will dispatch pending events by calling [callback] and reset
  /// internal state.
  void stop(HandleEventCallback callback) {
    while (_queuedEvents.isNotEmpty) {
      callback(_queuedEvents.removeFirst());
    }
    _pointerIdentifier = 0;
    _isDown = false;
    _isTracked = false;
    _position = Offset.zero;
    _next = null;
    _last = null;
  }

  /// Returns `true` if a call to [sample] can dispatch more events.
  bool get hasPendingEvents => _queuedEvents.isNotEmpty;

  /// Returns `true` if pointer is currently tracked.
  bool get isTracked => _isTracked;

  /// Returns `true` if pointer is currently down.
  bool get isDown => _isDown;
}
