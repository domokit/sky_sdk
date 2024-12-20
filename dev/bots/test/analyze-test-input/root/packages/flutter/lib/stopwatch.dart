// Copyright 2014 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

import '../../foo/stopwatch_external_lib.dart' as externallib;

typedef ExternalStopwatchConstructor = externallib.MyStopwatch Function();

class StopwatchAtHome extends Stopwatch {
  StopwatchAtHome();
  StopwatchAtHome.create() : this();

  Stopwatch get stopwatch => this;
}

void testNoStopwatches(Stopwatch stopwatch) {
  // OK for now, but we probably want to catch public APIs that take a Stopwatch?
  stopwatch.runtimeType;
  // Bad: introducing Stopwatch from dart:core.
  final Stopwatch localVariable = Stopwatch();
  // Bad: introducing Stopwatch from dart:core.
  Stopwatch().runtimeType;

  (localVariable..runtimeType) // OK: not directly introducing Stopwatch.
      .runtimeType;

  // Bad: introducing a Stopwatch subclass.
  StopwatchAtHome().runtimeType;

  // OK: not directly introducing Stopwatch.
  Stopwatch anotherStopwatch = stopwatch;
  // Bad: introducing a Stopwatch constructor.
  StopwatchAtHome Function() constructor = StopwatchAtHome.new;
  assert(() {
    anotherStopwatch = constructor()..runtimeType;
    // Bad: introducing a Stopwatch constructor.
    constructor = StopwatchAtHome.create;
    anotherStopwatch = constructor()..runtimeType;
    return true;
  }());
  anotherStopwatch.runtimeType;

  // Bad: introducing an external Stopwatch constructor.
  externallib.MyStopwatch.create();
  ExternalStopwatchConstructor? externalConstructor;

  assert(() {
    // Bad: introducing an external Stopwatch constructor.
    externalConstructor = externallib.MyStopwatch.new;
    return true;
  }());
  externalConstructor?.call();

  // Bad: introducing an external Stopwatch.
  externallib.stopwatch.runtimeType;
  // Bad: calling an external function that returns a Stopwatch.
  externallib.createMyStopwatch().runtimeType;
  // Bad: calling an external function that returns a Stopwatch.
  externallib.createStopwatch().runtimeType;
  // Bad: introducing the tear-off form of an external function that returns a Stopwatch.
  externalConstructor = externallib.createMyStopwatch;

  // OK: existing instance.
  constructor.call().stopwatch;
}

void testStopwatchIgnore(Stopwatch stopwatch) {
  Stopwatch().runtimeType; // flutter_ignore: stopwatch (see analyze.dart)
  Stopwatch().runtimeType; // flutter_ignore: some_other_ignores, stopwatch (see analyze.dart)
}
