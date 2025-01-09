// Copyright 2014 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

import 'dart:ui' as ui;

import 'package:flutter/foundation.dart';
import 'package:flutter/rendering.dart';
import 'package:flutter_test/flutter_test.dart';
import 'package:leak_tracker_flutter_testing/leak_tracker_flutter_testing.dart';

void main() {
  // LeakTesting is turned off because it adds subscriptions to
  // [FlutterMemoryAllocations], that may interfere with the tests.
  LeakTesting.settings = LeakTesting.settings.withIgnoredAll();

  final FlutterMemoryAllocations ma = FlutterMemoryAllocations.instance;

  setUp(() {
    assert(!ma.hasListeners);
  });

  test('Publishers dispatch events in debug mode', () async {
    int eventCount = 0;
    void listener(ObjectEvent event) => eventCount++;
    ma.addListener(listener);

    final int expectedEventCount = await _activateFlutterObjectsAndReturnCountOfEvents();
    expect(eventCount, expectedEventCount);

    ma.removeListener(listener);
    expect(ma.hasListeners, isFalse);
  });
}

class _TestRenderObject extends RenderObject {
  @override
  void debugAssertDoesMeetConstraints() {}

  @override
  Rect get paintBounds => throw UnimplementedError();

  @override
  void performLayout() {}

  @override
  void performResize() {}

  @override
  Rect get semanticBounds => throw UnimplementedError();
}

class _TestLayer extends Layer {
  @override
  void addToScene(ui.SceneBuilder builder) {}
}

/// Create and dispose Flutter objects to fire memory allocation events.
Future<int> _activateFlutterObjectsAndReturnCountOfEvents() async {
  int count = 0;

  final RenderObject renderObject = _TestRenderObject();
  count++;
  final Layer layer = _TestLayer();
  count++;

  renderObject.dispose();
  count++;
  layer.dispose();
  count++;

  return count;
}
