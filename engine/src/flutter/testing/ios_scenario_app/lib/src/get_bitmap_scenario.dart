// Copyright 2013 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

import 'dart:ui';

import 'scenario.dart';

/// A scenario with red on top and blue on the bottom.
class GetBitmapScenario extends Scenario {
  /// Creates the GetBitmap scenario.
  GetBitmapScenario(super.view);

  @override
  void onBeginFrame(Duration duration) {
    final PictureRecorder recorder = PictureRecorder();
    final Canvas canvas = Canvas(recorder);
    canvas.drawRect(
      Rect.fromLTWH(0, 0, view.physicalSize.width, 300),
      Paint()..color = const Color(0xFFFF0000),
    );
    canvas.drawRect(
      Rect.fromLTWH(0, view.physicalSize.height - 300, view.physicalSize.width, 300),
      Paint()..color = const Color(0xFF0000FF),
    );
    final Picture picture = recorder.endRecording();
    final SceneBuilder builder = SceneBuilder();
    builder.addPicture(Offset.zero, picture);
    final Scene scene = builder.build();
    view.render(scene);
    picture.dispose();
    scene.dispose();
  }
}
