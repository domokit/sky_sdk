// Copyright 2014 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

// External Library that creates Stopwatches. This file will not be analyzed but
// its symbols will be imported by tests.

class MyStopwatch implements Stopwatch {
  MyStopwatch();
  MyStopwatch.create() : this();

  @override
  Duration get elapsed => throw UnimplementedError();

  @override
  int get elapsedMicroseconds => throw UnimplementedError();

  @override
  int get elapsedMilliseconds => throw UnimplementedError();

  @override
  int get elapsedTicks => throw UnimplementedError();

  @override
  int get frequency => throw UnimplementedError();

  @override
  bool get isRunning => throw UnimplementedError();

  @override
  void reset() {}

  @override
  void start() {}

  @override
  void stop() {}
}

final MyStopwatch stopwatch = MyStopwatch.create();

MyStopwatch createMyStopwatch() => MyStopwatch();
Stopwatch createStopwatch() => Stopwatch();
