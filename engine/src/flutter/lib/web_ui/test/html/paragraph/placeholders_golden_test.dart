// Copyright 2013 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

import 'dart:async';

import 'package:test/bootstrap/browser.dart';
import 'package:test/test.dart';
import 'package:ui/src/engine.dart';
import 'package:ui/ui.dart' hide window;

import '../../common/test_initialization.dart';
import 'helper.dart';

const Rect bounds = Rect.fromLTWH(0, 0, 800, 600);

void main() {
  internalBootstrapBrowserTest(() => testMain);
}

Future<void> testMain() async {
  setUpUnitTests(
    withImplicitView: true,
    emulateTesterEnvironment: false,
    setUpTestViewDimensions: false,
  );

  test('draws paragraphs with placeholders', () {
    final BitmapCanvas canvas = BitmapCanvas(bounds, RenderStrategy());

    Offset offset = Offset.zero;
    for (final PlaceholderAlignment placeholderAlignment in PlaceholderAlignment.values) {
      final CanvasParagraph paragraph = rich(
        EngineParagraphStyle(fontFamily: 'Roboto', fontSize: 14.0),
        (CanvasParagraphBuilder builder) {
          builder.pushStyle(TextStyle(color: black));
          builder.addText('Lorem ipsum');
          builder.addPlaceholder(
            80.0,
            50.0,
            placeholderAlignment,
            baselineOffset: 40.0,
            baseline: TextBaseline.alphabetic,
          );
          builder.pushStyle(TextStyle(color: blue));
          builder.addText('dolor sit amet, consecteur.');
        },
      )..layout(constrain(200.0));

      // Draw the paragraph.
      canvas.drawParagraph(paragraph, offset);

      // Then fill the placeholders.
      fillPlaceholder(canvas, offset, paragraph);

      offset = offset.translate(0.0, paragraph.height + 30.0);
    }

    return takeScreenshot(canvas, bounds, 'canvas_paragraph_placeholders');
  });

  test('draws paragraphs with placeholders and text align', () {
    final BitmapCanvas canvas = BitmapCanvas(bounds, RenderStrategy());

    const List<TextAlign> aligns = <TextAlign>[TextAlign.left, TextAlign.center, TextAlign.right];

    Offset offset = Offset.zero;
    for (final TextAlign align in aligns) {
      final CanvasParagraph paragraph = rich(
        EngineParagraphStyle(fontFamily: 'Roboto', fontSize: 14.0, textAlign: align),
        (CanvasParagraphBuilder builder) {
          builder.pushStyle(TextStyle(color: black));
          builder.addText('Lorem');
          builder.addPlaceholder(80.0, 50.0, PlaceholderAlignment.bottom);
          builder.pushStyle(TextStyle(color: blue));
          builder.addText('ipsum.');
        },
      )..layout(constrain(200.0));

      // Draw the paragraph.
      canvas.drawParagraph(paragraph, offset);

      // Then fill the placeholders.
      fillPlaceholder(canvas, offset, paragraph);

      offset = offset.translate(0.0, paragraph.height + 30.0);
    }

    return takeScreenshot(canvas, bounds, 'canvas_paragraph_placeholders_align');
  });

  test('draws paragraphs with placeholders and text align in DOM mode', () {
    final DomCanvas canvas = DomCanvas(domDocument.createElement('flt-picture'));

    const List<TextAlign> aligns = <TextAlign>[TextAlign.left, TextAlign.center, TextAlign.right];

    Offset offset = Offset.zero;
    for (final TextAlign align in aligns) {
      final CanvasParagraph paragraph = rich(
        EngineParagraphStyle(fontFamily: 'Roboto', fontSize: 14.0, textAlign: align),
        (CanvasParagraphBuilder builder) {
          builder.pushStyle(TextStyle(color: black));
          builder.addText('Lorem');
          builder.addPlaceholder(80.0, 50.0, PlaceholderAlignment.bottom);
          builder.pushStyle(TextStyle(color: blue));
          builder.addText('ipsum.');
        },
      )..layout(constrain(200.0));

      // Draw the paragraph.
      canvas.drawParagraph(paragraph, offset);

      // Then fill the placeholders.
      fillPlaceholder(canvas, offset, paragraph);

      offset = offset.translate(0.0, paragraph.height + 30.0);
    }

    return takeScreenshot(canvas, bounds, 'canvas_paragraph_placeholders_align_dom');
  });

  test('draws paragraphs starting or ending with a placeholder', () {
    const Rect bounds = Rect.fromLTWH(0, 0, 420, 300);
    final BitmapCanvas canvas = BitmapCanvas(bounds, RenderStrategy());

    Offset offset = const Offset(10, 10);

    // First paragraph with a placeholder at the beginning.
    final CanvasParagraph paragraph1 = rich(
      EngineParagraphStyle(fontFamily: 'Roboto', fontSize: 24.0, textAlign: TextAlign.center),
      (CanvasParagraphBuilder builder) {
        builder.addPlaceholder(
          80.0,
          50.0,
          PlaceholderAlignment.baseline,
          baseline: TextBaseline.alphabetic,
        );
        builder.pushStyle(TextStyle(color: black));
        builder.addText(' Lorem ipsum.');
      },
    )..layout(constrain(400.0));

    // Draw the paragraph.
    canvas.drawParagraph(paragraph1, offset);
    fillPlaceholder(canvas, offset, paragraph1);
    surroundParagraph(canvas, offset, paragraph1);

    offset = offset.translate(0.0, paragraph1.height + 30.0);

    // Second paragraph with a placeholder at the end.
    final CanvasParagraph paragraph2 = rich(
      EngineParagraphStyle(fontFamily: 'Roboto', fontSize: 24.0, textAlign: TextAlign.center),
      (CanvasParagraphBuilder builder) {
        builder.pushStyle(TextStyle(color: black));
        builder.addText('Lorem ipsum ');
        builder.addPlaceholder(
          80.0,
          50.0,
          PlaceholderAlignment.baseline,
          baseline: TextBaseline.alphabetic,
        );
      },
    )..layout(constrain(400.0));

    // Draw the paragraph.
    canvas.drawParagraph(paragraph2, offset);
    fillPlaceholder(canvas, offset, paragraph2);
    surroundParagraph(canvas, offset, paragraph2);

    offset = offset.translate(0.0, paragraph2.height + 30.0);

    // Third paragraph with a placeholder alone in the second line.
    final CanvasParagraph paragraph3 = rich(
      EngineParagraphStyle(fontFamily: 'Roboto', fontSize: 24.0, textAlign: TextAlign.center),
      (CanvasParagraphBuilder builder) {
        builder.pushStyle(TextStyle(color: black));
        builder.addText('Lorem ipsum ');
        builder.addPlaceholder(
          80.0,
          50.0,
          PlaceholderAlignment.baseline,
          baseline: TextBaseline.alphabetic,
        );
      },
    )..layout(constrain(200.0));

    // Draw the paragraph.
    canvas.drawParagraph(paragraph3, offset);
    fillPlaceholder(canvas, offset, paragraph3);
    surroundParagraph(canvas, offset, paragraph3);

    return takeScreenshot(canvas, bounds, 'canvas_paragraph_placeholders_start_and_end');
  });
}

void surroundParagraph(EngineCanvas canvas, Offset offset, CanvasParagraph paragraph) {
  final Rect rect = offset & Size(paragraph.width, paragraph.height);
  final SurfacePaint paint =
      SurfacePaint()
        ..color = blue
        ..style = PaintingStyle.stroke;
  canvas.drawRect(rect, paint.paintData);
}
