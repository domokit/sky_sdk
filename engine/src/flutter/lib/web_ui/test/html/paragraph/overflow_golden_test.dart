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

void main() {
  internalBootstrapBrowserTest(() => testMain);
}

Future<void> testMain() async {
  setUpUnitTests(
    withImplicitView: true,
    emulateTesterEnvironment: false,
    setUpTestViewDimensions: false,
  );

  void testEllipsis(EngineCanvas canvas) {
    Offset offset = Offset.zero;
    CanvasParagraph paragraph;

    const double fontSize = 22.0;
    const double width = 126.0;
    const double padding = 20.0;
    final SurfacePaintData borderPaint =
        SurfacePaintData()
          ..color = black.value
          ..style = PaintingStyle.stroke;

    paragraph = rich(
      EngineParagraphStyle(fontFamily: 'Roboto', fontSize: fontSize, ellipsis: '...'),
      (CanvasParagraphBuilder builder) {
        builder.pushStyle(EngineTextStyle.only(color: blue));
        builder.addText('Lorem ');
        builder.pushStyle(EngineTextStyle.only(color: green));
        builder.addText('ipsum');
      },
    )..layout(constrain(width));
    canvas.drawParagraph(paragraph, offset);
    canvas.drawRect(Rect.fromLTWH(offset.dx, offset.dy, width, paragraph.height), borderPaint);
    offset = offset.translate(0, paragraph.height + padding);

    paragraph = rich(
      EngineParagraphStyle(fontFamily: 'Roboto', fontSize: fontSize, ellipsis: '...'),
      (CanvasParagraphBuilder builder) {
        builder.pushStyle(EngineTextStyle.only(color: blue));
        builder.addText('Lorem\n');
        builder.pushStyle(EngineTextStyle.only(color: green));
        builder.addText('ipsum ');
        builder.pushStyle(EngineTextStyle.only(color: red));
        builder.addText('dolor sit');
      },
    )..layout(constrain(width));
    canvas.drawParagraph(paragraph, offset);
    canvas.drawRect(Rect.fromLTWH(offset.dx, offset.dy, width, paragraph.height), borderPaint);
    offset = offset.translate(0, paragraph.height + padding);

    paragraph = rich(
      EngineParagraphStyle(fontFamily: 'Roboto', fontSize: fontSize, ellipsis: '...'),
      (CanvasParagraphBuilder builder) {
        builder.pushStyle(EngineTextStyle.only(color: blue));
        builder.addText('Lorem\n');
        builder.pushStyle(EngineTextStyle.only(color: green));
        builder.addText('ipsum ');
        builder.pushStyle(EngineTextStyle.only(color: red));
        builder.addText('d');
        builder.pushStyle(EngineTextStyle.only(color: black));
        builder.addText('o');
        builder.pushStyle(EngineTextStyle.only(color: blue));
        builder.addText('l');
        builder.pushStyle(EngineTextStyle.only(color: green));
        builder.addText('o');
        builder.pushStyle(EngineTextStyle.only(color: red));
        builder.addText('r');
        builder.pushStyle(EngineTextStyle.only(color: black));
        builder.addText(' ');
        builder.pushStyle(EngineTextStyle.only(color: blue));
        builder.addText('s');
        builder.pushStyle(EngineTextStyle.only(color: green));
        builder.addText('i');
        builder.pushStyle(EngineTextStyle.only(color: red));
        builder.addText('t');
      },
    )..layout(constrain(width));
    canvas.drawParagraph(paragraph, offset);
    canvas.drawRect(Rect.fromLTWH(offset.dx, offset.dy, width, paragraph.height), borderPaint);
    offset = offset.translate(0, paragraph.height + padding);

    paragraph = rich(
      EngineParagraphStyle(fontFamily: 'Roboto', fontSize: fontSize, maxLines: 2, ellipsis: '...'),
      (CanvasParagraphBuilder builder) {
        builder.pushStyle(EngineTextStyle.only(color: blue));
        builder.addText('Lorem');
        builder.pushStyle(EngineTextStyle.only(color: green));
        builder.addText('ipsu');
        builder.pushStyle(EngineTextStyle.only(color: red));
        builder.addText('mdolor');
        builder.pushStyle(EngineTextStyle.only(color: black));
        builder.addText('sit');
        builder.pushStyle(EngineTextStyle.only(color: blue));
        builder.addText('amet');
        builder.pushStyle(EngineTextStyle.only(color: blue));
        builder.addText('consectetur');
      },
    )..layout(constrain(width));
    canvas.drawParagraph(paragraph, offset);
    canvas.drawRect(Rect.fromLTWH(offset.dx, offset.dy, width, paragraph.height), borderPaint);
    offset = offset.translate(0, paragraph.height + padding);
  }

  test('ellipsis', () {
    const Rect bounds = Rect.fromLTWH(0, 0, 300, 300);
    final EngineCanvas canvas = BitmapCanvas(bounds, RenderStrategy());
    testEllipsis(canvas);
    return takeScreenshot(canvas, bounds, 'canvas_paragraph_ellipsis');
  });

  test('ellipsis (dom)', () {
    const Rect bounds = Rect.fromLTWH(0, 0, 300, 300);
    final EngineCanvas canvas = DomCanvas(domDocument.createElement('flt-picture'));
    testEllipsis(canvas);
    return takeScreenshot(canvas, bounds, 'canvas_paragraph_ellipsis_dom');
  });
}
