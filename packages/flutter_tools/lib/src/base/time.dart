// Copyright 2014 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

/// A class for making time based operations testable.
class SystemClock {
  /// A const constructor to allow subclasses to be const.
  const SystemClock();

  /// Create a clock with a fixed current time.
  const factory SystemClock.fixed(DateTime time) = _FixedTimeClock;

  /// Retrieve the current time.
  DateTime now() => DateTime.now();

  /// Compute the time a given duration ago.
  DateTime ago(Duration duration) {
    return now().subtract(duration);
  }
}

class _FixedTimeClock extends SystemClock {
  const _FixedTimeClock(this._fixedTime);

  final DateTime _fixedTime;

  @override
  DateTime now() => _fixedTime;
}

/// Format time as 'yyyy-MM-dd HH:mm:ss Z' where Z is the difference between the
/// timezone of t and UTC formatted according to RFC 822.
String formatDateTime(DateTime t) {
  final String sign = t.timeZoneOffset.isNegative ? '-' : '+';
  final Duration tzOffset = t.timeZoneOffset.abs();
  final int hoursOffset = tzOffset.inHours;
  final int minutesOffset = tzOffset.inMinutes - (Duration.minutesPerHour * hoursOffset);
  assert(hoursOffset < 24);
  assert(minutesOffset < 60);

  String twoDigits(int n) => (n >= 10) ? '$n' : '0$n';
  return '$t $sign${twoDigits(hoursOffset)}${twoDigits(minutesOffset)}';
}
