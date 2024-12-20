// Copyright 2013 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

import 'dart:ui';

import 'scenario.dart';

/// Shows a text that is shown in app extension.
class DarwinAppExtensionScenario extends Scenario {
  /// Creates the DarwinAppExtensionScenario scenario.
  DarwinAppExtensionScenario(super.view);

  // Semi-arbitrary.
  final double _screenWidth = 700;

  @override
  void onBeginFrame(Duration duration) {
    final SceneBuilder builder = SceneBuilder();
    final PictureRecorder recorder = PictureRecorder();
    final Canvas canvas = Canvas(recorder);

    final ParagraphBuilder paragraphBuilder =
        ParagraphBuilder(ParagraphStyle())
          ..pushStyle(TextStyle(fontSize: 80))
          ..addText('flutter Scenarios app extension.')
          ..pop();
    final Paragraph paragraph = paragraphBuilder.build();

    paragraph.layout(ParagraphConstraints(width: _screenWidth));

    canvas.drawParagraph(paragraph, const Offset(50, 80));
    final Picture picture = recorder.endRecording();

    builder.addPicture(Offset.zero, picture, willChangeHint: true);
    final Scene scene = builder.build();
    view.render(scene);
    scene.dispose();
  }
}
