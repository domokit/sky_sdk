// Copyright 2013 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

import 'package:quiver/testing/async.dart';
import 'package:quiver/time.dart';
import 'package:test/bootstrap/browser.dart';
import 'package:test/test.dart';

import 'package:ui/src/engine/alarm_clock.dart';

void main() {
  internalBootstrapBrowserTest(() => testMain);
}

void testMain() {
  group(AlarmClock, () {
    _alarmClockTests();
  });
}

void _alarmClockTests() {
  int callCount = 0;

  void testCallback() {
    callCount += 1;
  }

  setUp(() {
    callCount = 0;
  });

  testAsync('AlarmClock calls the callback in the future', (FakeAsync fakeAsync) {
    final Clock clock = fakeAsync.getClock(DateTime(2019, 1, 24));
    final AlarmClock alarm = AlarmClock(clock.now);
    alarm.callback = testCallback;

    // There should be no timers scheduled until we set a non-null datetime.
    expect(fakeAsync.nonPeriodicTimerCount, 0);

    alarm.datetime = clock.fromNow(minutes: 1);

    // There should be exactly 1 timer scheduled.
    expect(fakeAsync.nonPeriodicTimerCount, 1);

    // No time has passed; the callback should not be called.
    expect(callCount, 0);

    // Not enough time has passed; the callback should not be called.
    fakeAsync.elapse(const Duration(seconds: 30));
    expect(callCount, 0);

    // Exactly 1 minute has passed; fire the callback.
    fakeAsync.elapse(const Duration(seconds: 30));
    expect(callCount, 1);

    // Timers should be cleaned up.
    expect(fakeAsync.nonPeriodicTimerCount, 0);

    // Rescheduling.
    alarm.datetime = clock.fromNow(minutes: 1);
    expect(fakeAsync.nonPeriodicTimerCount, 1);
    fakeAsync.elapse(const Duration(minutes: 1));
    expect(fakeAsync.nonPeriodicTimerCount, 0);
    expect(callCount, 2);
  });

  testAsync('AlarmClock does nothing when new datetime is the same', (FakeAsync fakeAsync) {
    final Clock clock = fakeAsync.getClock(DateTime(2019, 1, 24));
    final AlarmClock alarm = AlarmClock(clock.now);
    alarm.callback = testCallback;

    alarm.datetime = clock.fromNow(minutes: 1);
    expect(fakeAsync.nonPeriodicTimerCount, 1);
    expect(callCount, 0);

    alarm.datetime = alarm.datetime!.add(Duration.zero);
    expect(fakeAsync.nonPeriodicTimerCount, 1);
    expect(callCount, 0);

    fakeAsync.elapse(const Duration(seconds: 30));

    alarm.datetime = alarm.datetime!.add(Duration.zero);
    expect(fakeAsync.nonPeriodicTimerCount, 1);
    expect(callCount, 0);

    fakeAsync.elapse(const Duration(seconds: 30));
    expect(fakeAsync.nonPeriodicTimerCount, 0);
    expect(callCount, 1);
  });

  testAsync('AlarmClock does not call the callback in the past', (FakeAsync fakeAsync) {
    final Clock clock = fakeAsync.getClock(DateTime(2019, 1, 24));
    final AlarmClock alarm = AlarmClock(clock.now);
    alarm.callback = testCallback;
    alarm.datetime = clock.ago(minutes: 1);

    // No timers scheduled for past dates.
    expect(fakeAsync.nonPeriodicTimerCount, 0);
    expect(callCount, 0);
  });

  testAsync('AlarmClock reschedules to a future time', (FakeAsync fakeAsync) {
    final Clock clock = fakeAsync.getClock(DateTime(2019, 1, 24));
    final AlarmClock alarm = AlarmClock(clock.now);
    alarm.callback = testCallback;

    alarm.datetime = clock.fromNow(minutes: 1);
    expect(fakeAsync.nonPeriodicTimerCount, 1);

    expect(callCount, 0);
    fakeAsync.elapse(const Duration(seconds: 30));
    expect(callCount, 0);

    // Reschedule.
    alarm.datetime = alarm.datetime!.add(const Duration(minutes: 1));

    fakeAsync.elapse(const Duration(minutes: 1));

    // Still no calls because we rescheduled.
    expect(callCount, 0);

    fakeAsync.elapse(const Duration(seconds: 30));
    expect(callCount, 1);
    expect(fakeAsync.nonPeriodicTimerCount, 0);
  });

  testAsync('AlarmClock reschedules to an earlier time', (FakeAsync fakeAsync) {
    final Clock clock = fakeAsync.getClock(DateTime(2019, 1, 24));
    final AlarmClock alarm = AlarmClock(clock.now);
    alarm.callback = testCallback;

    alarm.datetime = clock.fromNow(minutes: 1);
    expect(fakeAsync.nonPeriodicTimerCount, 1);

    expect(callCount, 0);
    fakeAsync.elapse(const Duration(seconds: 30));
    expect(callCount, 0);

    // Reschedule to an earlier time that's still in the future.
    alarm.datetime = alarm.datetime!.subtract(const Duration(seconds: 15));

    fakeAsync.elapse(const Duration(seconds: 45));
    expect(callCount, 1);
    expect(fakeAsync.nonPeriodicTimerCount, 0);
  });

  testAsync('AlarmClock cancels the timer when datetime is null', (FakeAsync fakeAsync) {
    final Clock clock = fakeAsync.getClock(DateTime(2019, 1, 24));
    final AlarmClock alarm = AlarmClock(clock.now);
    alarm.callback = testCallback;

    alarm.datetime = clock.fromNow(minutes: 1);
    expect(fakeAsync.nonPeriodicTimerCount, 1);

    // Cancel.
    alarm.datetime = null;
    expect(fakeAsync.nonPeriodicTimerCount, 0);
    expect(callCount, 0);

    // Make sure nothing fires even if we wait long enough.
    fakeAsync.elapse(const Duration(minutes: 2));
    expect(callCount, 0);
    expect(fakeAsync.nonPeriodicTimerCount, 0);
  });

  testAsync('AlarmClock cancels the timer when datetime is in the past', (FakeAsync fakeAsync) {
    final Clock clock = fakeAsync.getClock(DateTime(2019, 1, 24));
    final AlarmClock alarm = AlarmClock(clock.now);
    alarm.callback = testCallback;

    alarm.datetime = clock.fromNow(minutes: 1);
    expect(fakeAsync.nonPeriodicTimerCount, 1);
    fakeAsync.elapse(const Duration(seconds: 30));
    expect(callCount, 0);
    expect(fakeAsync.nonPeriodicTimerCount, 1);

    // Cancel.
    alarm.datetime = clock.ago(seconds: 1);
    expect(fakeAsync.nonPeriodicTimerCount, 0);
    expect(callCount, 0);

    // Make sure nothing fires even if we wait long enough.
    fakeAsync.elapse(const Duration(minutes: 2));
    expect(callCount, 0);
    expect(fakeAsync.nonPeriodicTimerCount, 0);
  });
}

typedef FakeAsyncTest = void Function(FakeAsync);

void testAsync(String description, FakeAsyncTest fn) {
  test(description, () {
    FakeAsync().run(fn);
  });
}
