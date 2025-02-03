// Copyright 2013 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

import 'dart:ui';

import 'scenario.dart';

/// Fills the screen with a solid blue color.
class SolidBlueScenario extends Scenario {
  /// Creates the SolidBlue scenario.
  SolidBlueScenario(super.view);

  @override
  void onBeginFrame(Duration duration) {
    final SceneBuilder builder = SceneBuilder();
    final PictureRecorder recorder = PictureRecorder();
    final Canvas canvas = Canvas(recorder);

    canvas.drawPaint(Paint()..color = const Color(0xFF0000FF));
    final Picture picture = recorder.endRecording();

    builder.addPicture(Offset.zero, picture, willChangeHint: true);
    final Scene scene = builder.build();
    view.render(scene);
    scene.dispose();
  }
}
