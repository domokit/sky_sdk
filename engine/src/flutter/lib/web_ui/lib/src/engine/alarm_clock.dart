// Copyright 2013 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

import 'dart:async';

import 'package:ui/ui.dart' as ui;

/// A function that returns current system time.
typedef TimestampFunction = DateTime Function();

/// Notifies the [callback] at the given [datetime].
///
/// Allows changing [datetime] in either direction before the alarm goes off.
///
/// The implementation uses [Timer]s and therefore does not guarantee that it
/// will go off precisely at the specified time. For more details see:
///
/// https://developer.mozilla.org/en-US/docs/Web/API/WindowOrWorkerGlobalScope/setTimeout#Notes
class AlarmClock {
  /// Initializes Alarmclock with a closure that gets called with a timestamp.
  AlarmClock(TimestampFunction timestampFunction) : _timestampFunction = timestampFunction;

  /// The function used to get current time.
  final TimestampFunction _timestampFunction;

  /// The underlying timer used to schedule the callback.
  Timer? _timer;

  /// Current target time the [callback] is scheduled for.
  DateTime? _datetime;

  /// The callback called when the alarm goes off.
  ///
  /// If this is null, the alarm goes off without calling the callback. Set the
  /// callback to null if the callback is a closure holding onto expensive
  /// resources.
  ui.VoidCallback? callback;

  /// The time when the alarm clock will go off.
  ///
  /// If the time is in the past or is `null` the alarm clock will not go off.
  ///
  /// If the value is updated before an already scheduled timer goes off, the
  /// previous time will not call the [callback]. Think of the updating this
  /// value as "changing your mind" about when you want the next timer to fire.
  DateTime? get datetime => _datetime;
  set datetime(DateTime? value) {
    if (value == _datetime) {
      return;
    }

    if (value == null) {
      _cancelTimer();
      _datetime = null;
      return;
    }

    final DateTime now = _timestampFunction();

    // We use the "not before" logic instead of "is after" because zero-duration
    // timers are valid.
    final bool isInTheFuture = !value.isBefore(now);

    if (!isInTheFuture) {
      _cancelTimer();
      _datetime = value;
      return;
    }

    // At this point we have a non-null value that's in the future, and it is
    // different from the current _datetime. We need to decide whether we need
    // to create a new timer, or keep the existing one.
    if (_timer == null) {
      // We didn't have an existing timer, so create a new one.
      _timer = Timer(value.difference(now), _timerDidFire);
    } else {
      assert(_datetime != null, 'We can only have a timer if there is a non-null datetime');
      if (_datetime!.isAfter(value)) {
        // This is the case when the value moves the target time to an earlier
        // point. Because there is no way to reconfigure an existing timer, we
        // must cancel the old timer and schedule a new one.
        _cancelTimer();
        _timer = Timer(value.difference(now), _timerDidFire);
      }
      // We don't need to do anything in the "else" branch. If the new value
      // is in the future relative to the current datetime, the _timerDidFire
      // will reschedule.
    }

    _datetime = value;
  }

  void _cancelTimer() {
    _timer?.cancel();
    _timer = null;
  }

  void _timerDidFire() {
    assert(_datetime != null, 'If _datetime is null, the timer would have been cancelled');
    final DateTime now = _timestampFunction();
    // We use the "not before" logic instead of "is after" because we may have
    // zero difference between now and _datetime.
    if (!now.isBefore(_datetime!)) {
      _timer = null;
      callback?.call();
    } else {
      // The timer fired before the target date. We need to reschedule.
      _timer = Timer(_datetime!.difference(now), _timerDidFire);
    }
  }
}
