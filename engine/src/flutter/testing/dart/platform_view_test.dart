// Copyright 2013 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

import 'dart:ui';

import 'package:test/test.dart';

void main() {
  test('PlatformView layers do not emit errors from tester', () async {
    final SceneBuilder builder = SceneBuilder();
    builder.addPlatformView(1);
    PlatformDispatcher.instance.onBeginFrame = (Duration duration) {
      final Scene scene = builder.build();
      PlatformDispatcher.instance.implicitView!.render(scene);
      scene.dispose();
    };
    PlatformDispatcher.instance.scheduleFrame();
    // Test harness asserts that this does not emit an error from the shell logs.
  });
}
