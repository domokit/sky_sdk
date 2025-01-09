// Copyright 2013 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

import 'package:test/bootstrap/browser.dart';
import 'package:ui/src/engine.dart';
import 'package:ui/ui.dart';

import '../../common/test_initialization.dart';
import 'helper.dart';
import 'text_goldens.dart';

void main() {
  internalBootstrapBrowserTest(() => testMain);
}

Future<void> testMain() async {
  final EngineGoldenTester goldenTester = await EngineGoldenTester.initialize(
    viewportSize: const Size(600, 600),
  );

  setUpUnitTests(
    withImplicitView: true,
    emulateTesterEnvironment: false,
    setUpTestViewDimensions: false,
  );

  testEachCanvas('draws paragraphs with placeholders', (EngineCanvas canvas) {
    const Rect screenRect = Rect.fromLTWH(0, 0, 600, 600);
    final RecordingCanvas recordingCanvas = RecordingCanvas(screenRect);

    Offset offset = Offset.zero;
    for (final PlaceholderAlignment placeholderAlignment in PlaceholderAlignment.values) {
      _paintTextWithPlaceholder(
        recordingCanvas,
        offset,
        before: 'Lorem ipsum',
        after: 'dolor sit amet, consectetur.',
        placeholderAlignment: placeholderAlignment,
      );
      offset = offset.translate(0.0, 80.0);
    }
    recordingCanvas.endRecording();
    recordingCanvas.apply(canvas, screenRect);
    return goldenTester.diffCanvasScreenshot(canvas, 'text_with_placeholders');
  });

  testEachCanvas('text alignment and placeholders', (EngineCanvas canvas) {
    const Rect screenRect = Rect.fromLTWH(0, 0, 600, 600);
    final RecordingCanvas recordingCanvas = RecordingCanvas(screenRect);

    Offset offset = Offset.zero;
    _paintTextWithPlaceholder(
      recordingCanvas,
      offset,
      before: 'Lorem',
      after: 'ipsum.',
      textAlignment: TextAlign.start,
    );
    offset = offset.translate(0.0, 80.0);
    _paintTextWithPlaceholder(
      recordingCanvas,
      offset,
      before: 'Lorem',
      after: 'ipsum.',
      textAlignment: TextAlign.center,
    );
    offset = offset.translate(0.0, 80.0);
    _paintTextWithPlaceholder(
      recordingCanvas,
      offset,
      before: 'Lorem',
      after: 'ipsum.',
      textAlignment: TextAlign.end,
    );
    recordingCanvas.endRecording();
    recordingCanvas.apply(canvas, screenRect);
    return goldenTester.diffCanvasScreenshot(canvas, 'text_align_with_placeholders');
  });
}

const Size placeholderSize = Size(80.0, 50.0);

void _paintTextWithPlaceholder(
  RecordingCanvas canvas,
  Offset offset, {
  required String before,
  required String after,
  PlaceholderAlignment placeholderAlignment = PlaceholderAlignment.baseline,
  TextAlign textAlignment = TextAlign.left,
}) {
  // First let's draw the paragraph.
  final Paragraph paragraph = _createParagraphWithPlaceholder(
    before,
    after,
    placeholderAlignment,
    textAlignment,
  );
  canvas.drawParagraph(paragraph, offset);

  // Then fill the placeholders.
  final TextBox placeholderBox = paragraph.getBoxesForPlaceholders().single;
  canvas.drawRect(placeholderBox.toRect().shift(offset), SurfacePaint()..color = red);
}

Paragraph _createParagraphWithPlaceholder(
  String before,
  String after,
  PlaceholderAlignment placeholderAlignment,
  TextAlign textAlignment,
) {
  final ParagraphBuilder builder = ParagraphBuilder(ParagraphStyle(textAlign: textAlignment));
  builder.pushStyle(TextStyle(color: black, fontFamily: 'Roboto', fontSize: 14));
  builder.addText(before);
  builder.addPlaceholder(
    placeholderSize.width,
    placeholderSize.height,
    placeholderAlignment,
    baselineOffset: 40.0,
    baseline: TextBaseline.alphabetic,
  );
  builder.pushStyle(TextStyle(color: blue, fontFamily: 'Roboto', fontSize: 14));
  builder.addText(after);
  return builder.build()..layout(const ParagraphConstraints(width: 200.0));
}
