// Copyright 2013 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

import 'dart:ui';

import 'channel_util.dart';
import 'scenario.dart';

/// Tries to draw darwin system font: CupertinoSystemDisplay, CupertinoSystemText
class DarwinSystemFont extends Scenario {
  /// Creates the DarwinSystemFont scenario.
  DarwinSystemFont(super.view);

  // Semi-arbitrary.
  final double _screenWidth = 700;

  @override
  void onBeginFrame(Duration duration) {
    final SceneBuilder builder = SceneBuilder();
    final PictureRecorder recorder = PictureRecorder();
    final Canvas canvas = Canvas(recorder);

    final ParagraphBuilder paragraphBuilderDisplay =
        ParagraphBuilder(ParagraphStyle(fontFamily: 'CupertinoSystemDisplay'))
          ..pushStyle(TextStyle(fontSize: 50))
          ..addText('Cupertino System Display\n')
          ..pop();
    final ParagraphBuilder paragraphBuilderText =
        ParagraphBuilder(ParagraphStyle(fontFamily: 'CupertinoSystemText'))
          ..pushStyle(TextStyle(fontSize: 50))
          ..addText('Cupertino System Text\n')
          ..pop();

    final Paragraph paragraphPro = paragraphBuilderDisplay.build();
    paragraphPro.layout(ParagraphConstraints(width: _screenWidth));
    canvas.drawParagraph(paragraphPro, const Offset(50, 80));

    final Paragraph paragraphText = paragraphBuilderText.build();
    paragraphText.layout(ParagraphConstraints(width: _screenWidth));
    canvas.drawParagraph(paragraphText, const Offset(50, 200));

    final Picture picture = recorder.endRecording();

    builder.addPicture(Offset.zero, picture, willChangeHint: true);
    final Scene scene = builder.build();
    view.render(scene);
    scene.dispose();

    sendJsonMessage(
      dispatcher: view.platformDispatcher,
      channel: 'display_data',
      json: <String, dynamic>{'data': 'ready'},
    );
  }
}
