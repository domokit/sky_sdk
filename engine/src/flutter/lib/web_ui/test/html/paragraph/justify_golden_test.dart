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

  void testJustifyWithMultipleSpans(EngineCanvas canvas) {
    void build(CanvasParagraphBuilder builder) {
      builder.pushStyle(EngineTextStyle.only(color: black));
      builder.addText('Lorem ipsum dolor sit ');
      builder.pushStyle(EngineTextStyle.only(color: blue));
      builder.addText('amet, consectetur ');
      builder.pushStyle(EngineTextStyle.only(color: green));
      builder.addText('adipiscing elit, sed do eiusmod tempor incididunt ut ');
      builder.pushStyle(EngineTextStyle.only(color: red));
      builder.addText('labore et dolore magna aliqua. Ut enim ad minim veniam, ');
      builder.pushStyle(EngineTextStyle.only(color: lightPurple));
      builder.addText('quis nostrud exercitation ullamco ');
      builder.pushStyle(EngineTextStyle.only(color: blue));
      builder.addText('laboris nisi ut aliquip ex ea commodo consequat.');
    }

    final CanvasParagraph paragraph = rich(
      EngineParagraphStyle(fontFamily: 'Roboto', fontSize: 20.0, textAlign: TextAlign.justify),
      build,
    );
    paragraph.layout(constrain(250.0));
    canvas.drawParagraph(paragraph, Offset.zero);
  }

  test('TextAlign.justify with multiple spans', () {
    const Rect bounds = Rect.fromLTWH(0, 0, 400, 400);
    final BitmapCanvas canvas = BitmapCanvas(bounds, RenderStrategy());
    testJustifyWithMultipleSpans(canvas);
    return takeScreenshot(canvas, bounds, 'canvas_paragraph_justify');
  });

  test('TextAlign.justify with multiple spans (DOM)', () {
    const Rect bounds = Rect.fromLTWH(0, 0, 400, 400);
    final DomCanvas canvas = DomCanvas(domDocument.createElement('flt-picture'));
    testJustifyWithMultipleSpans(canvas);
    return takeScreenshot(canvas, bounds, 'canvas_paragraph_justify_dom');
  });

  void testJustifyWithEmptyLine(EngineCanvas canvas) {
    void build(CanvasParagraphBuilder builder) {
      builder.pushStyle(bg(yellow));
      builder.pushStyle(EngineTextStyle.only(color: black));
      builder.addText('Loremipsumdolorsit');
      builder.pushStyle(EngineTextStyle.only(color: blue));
      builder.addText('amet,                consectetur\n\n');
      builder.pushStyle(EngineTextStyle.only(color: lightPurple));
      builder.addText('adipiscing elit, sed do eiusmod tempor incididunt ut ');
      builder.pushStyle(EngineTextStyle.only(color: red));
      builder.addText('labore et dolore magna aliqua.');
    }

    final CanvasParagraph paragraph = rich(
      EngineParagraphStyle(fontFamily: 'Roboto', fontSize: 20.0, textAlign: TextAlign.justify),
      build,
    );
    paragraph.layout(constrain(250.0));
    canvas.drawParagraph(paragraph, Offset.zero);
  }

  test('TextAlign.justify with single space and empty line', () {
    const Rect bounds = Rect.fromLTWH(0, 0, 400, 400);
    final BitmapCanvas canvas = BitmapCanvas(bounds, RenderStrategy());
    testJustifyWithEmptyLine(canvas);
    return takeScreenshot(canvas, bounds, 'canvas_paragraph_justify_empty_line');
  });

  test('TextAlign.justify with single space and empty line (DOM)', () {
    const Rect bounds = Rect.fromLTWH(0, 0, 400, 400);
    final DomCanvas canvas = DomCanvas(domDocument.createElement('flt-picture'));
    testJustifyWithEmptyLine(canvas);
    return takeScreenshot(canvas, bounds, 'canvas_paragraph_justify_empty_line_dom');
  });

  void testJustifyWithEllipsis(EngineCanvas canvas) {
    final EngineParagraphStyle paragraphStyle = EngineParagraphStyle(
      fontFamily: 'Roboto',
      fontSize: 20.0,
      textAlign: TextAlign.justify,
      maxLines: 4,
      ellipsis: '...',
    );
    final CanvasParagraph paragraph = rich(paragraphStyle, (CanvasParagraphBuilder builder) {
      builder.pushStyle(EngineTextStyle.only(color: black));
      builder.addText('Lorem ipsum dolor sit ');
      builder.pushStyle(EngineTextStyle.only(color: blue));
      builder.addText('amet, consectetur ');
      builder.pushStyle(EngineTextStyle.only(color: green));
      builder.addText('adipiscing elit, sed do eiusmod tempor incididunt ut ');
      builder.pushStyle(EngineTextStyle.only(color: red));
      builder.addText('labore et dolore magna aliqua. Ut enim ad minim veniam, ');
      builder.pushStyle(EngineTextStyle.only(color: lightPurple));
      builder.addText('quis nostrud exercitation ullamco ');
      builder.pushStyle(EngineTextStyle.only(color: blue));
      builder.addText('laboris nisi ut aliquip ex ea commodo consequat.');
    });
    paragraph.layout(constrain(250));
    canvas.drawParagraph(paragraph, Offset.zero);
  }

  test('TextAlign.justify with ellipsis', () {
    const Rect bounds = Rect.fromLTWH(0, 0, 400, 300);
    final BitmapCanvas canvas = BitmapCanvas(bounds, RenderStrategy());
    testJustifyWithEllipsis(canvas);
    return takeScreenshot(canvas, bounds, 'canvas_paragraph_justify_ellipsis');
  });

  test('TextAlign.justify with ellipsis (DOM)', () {
    const Rect bounds = Rect.fromLTWH(0, 0, 400, 300);
    final DomCanvas canvas = DomCanvas(domDocument.createElement('flt-picture'));
    testJustifyWithEllipsis(canvas);
    return takeScreenshot(canvas, bounds, 'canvas_paragraph_justify_ellipsis_dom');
  });

  void testJustifyWithBackground(EngineCanvas canvas) {
    final EngineParagraphStyle paragraphStyle = EngineParagraphStyle(
      fontFamily: 'Roboto',
      fontSize: 20.0,
      textAlign: TextAlign.justify,
    );
    final CanvasParagraph paragraph = rich(paragraphStyle, (CanvasParagraphBuilder builder) {
      builder.pushStyle(EngineTextStyle.only(color: black));
      builder.pushStyle(bg(blue));
      builder.addText('Lorem ipsum dolor sit ');
      builder.pushStyle(bg(black));
      builder.pushStyle(EngineTextStyle.only(color: white));
      builder.addText('amet, consectetur ');
      builder.pop();
      builder.pushStyle(bg(green));
      builder.addText('adipiscing elit, sed do eiusmod tempor incididunt ut ');
      builder.pushStyle(bg(yellow));
      builder.addText('labore et dolore magna aliqua. Ut enim ad minim veniam, ');
      builder.pushStyle(bg(red));
      builder.addText('quis nostrud exercitation ullamco ');
      builder.pushStyle(bg(green));
      builder.addText('laboris nisi ut aliquip ex ea commodo consequat.');
    });
    paragraph.layout(constrain(250));
    canvas.drawParagraph(paragraph, Offset.zero);
  }

  test('TextAlign.justify with background', () {
    const Rect bounds = Rect.fromLTWH(0, 0, 400, 400);
    final BitmapCanvas canvas = BitmapCanvas(bounds, RenderStrategy());
    testJustifyWithBackground(canvas);
    return takeScreenshot(canvas, bounds, 'canvas_paragraph_justify_background');
  });

  test('TextAlign.justify with background (DOM)', () {
    const Rect bounds = Rect.fromLTWH(0, 0, 400, 400);
    final DomCanvas canvas = DomCanvas(domDocument.createElement('flt-picture'));
    testJustifyWithBackground(canvas);
    return takeScreenshot(canvas, bounds, 'canvas_paragraph_justify_background_dom');
  });

  void testJustifyWithPlaceholder(EngineCanvas canvas) {
    void build(CanvasParagraphBuilder builder) {
      builder.pushStyle(EngineTextStyle.only(color: black));
      builder.addText('Lorem ipsum dolor sit ');
      builder.pushStyle(EngineTextStyle.only(color: blue));
      builder.addText('amet, consectetur ');
      builder.pushStyle(EngineTextStyle.only(color: green));
      builder.addText('adipiscing elit, sed do ');
      builder.addPlaceholder(40, 40, PlaceholderAlignment.bottom);
      builder.addText(' eiusmod tempor incididunt ut ');
      builder.pushStyle(EngineTextStyle.only(color: red));
      builder.addText('labore et dolore magna aliqua.');
    }

    final CanvasParagraph paragraph = rich(
      EngineParagraphStyle(fontFamily: 'Roboto', fontSize: 20.0, textAlign: TextAlign.justify),
      build,
    );
    paragraph.layout(constrain(250.0));
    canvas.drawParagraph(paragraph, Offset.zero);
    fillPlaceholder(canvas, Offset.zero, paragraph);
  }

  test('TextAlign.justify with placeholder', () {
    const Rect bounds = Rect.fromLTWH(0, 0, 400, 400);
    final BitmapCanvas canvas = BitmapCanvas(bounds, RenderStrategy());
    testJustifyWithPlaceholder(canvas);
    return takeScreenshot(canvas, bounds, 'canvas_paragraph_justify_placeholder');
  });

  test('TextAlign.justify with placeholder (DOM)', () {
    const Rect bounds = Rect.fromLTWH(0, 0, 400, 400);
    final DomCanvas canvas = DomCanvas(domDocument.createElement('flt-picture'));
    testJustifyWithPlaceholder(canvas);
    return takeScreenshot(canvas, bounds, 'canvas_paragraph_justify_placeholder_dom');
  });

  void testJustifyWithSelection(EngineCanvas canvas) {
    void build(CanvasParagraphBuilder builder) {
      builder.pushStyle(EngineTextStyle.only(color: black));
      builder.addText('Lorem ipsum dolor sit ');
      builder.pushStyle(EngineTextStyle.only(color: blue));
      builder.addText('amet, consectetur '); // 40
      builder.pushStyle(EngineTextStyle.only(color: green));
      builder.addText('adipiscing elit, sed do eiusmod tempor incididunt ut ');
      builder.pushStyle(EngineTextStyle.only(color: red));
      builder.addText('labore et dolore magna aliqua.');
    }

    final CanvasParagraph paragraph = rich(
      EngineParagraphStyle(fontFamily: 'Roboto', fontSize: 20.0, textAlign: TextAlign.justify),
      build,
    );
    paragraph.layout(constrain(250.0));

    // Draw selection for "em ipsum d".
    fillBoxes(canvas, Offset.zero, paragraph.getBoxesForRange(3, 13), lightBlue);
    // Draw selection for " ut labore et dolore mag".
    fillBoxes(canvas, Offset.zero, paragraph.getBoxesForRange(89, 113), lightPurple);

    canvas.drawParagraph(paragraph, Offset.zero);
  }

  test('TextAlign.justify with selection', () {
    const Rect bounds = Rect.fromLTWH(0, 0, 400, 400);
    final BitmapCanvas canvas = BitmapCanvas(bounds, RenderStrategy());
    testJustifyWithSelection(canvas);
    return takeScreenshot(canvas, bounds, 'canvas_paragraph_justify_selection');
  });

  test('TextAlign.justify with selection (DOM)', () {
    const Rect bounds = Rect.fromLTWH(0, 0, 400, 400);
    final DomCanvas canvas = DomCanvas(domDocument.createElement('flt-picture'));
    testJustifyWithSelection(canvas);
    return takeScreenshot(canvas, bounds, 'canvas_paragraph_justify_selection_dom');
  });

  void testJustifyRtl(EngineCanvas canvas) {
    const String rtlWord = 'مرحبا';
    void build(CanvasParagraphBuilder builder) {
      builder.pushStyle(EngineTextStyle.only(color: black));
      builder.addText('Lorem $rtlWord dolor sit ');
      builder.pushStyle(EngineTextStyle.only(color: blue));
      builder.addText('amet, consectetur '); // 40
      builder.pushStyle(EngineTextStyle.only(color: green));
      builder.addText('adipiscing elit, sed do eiusmod $rtlWord incididunt ut ');
      builder.pushStyle(EngineTextStyle.only(color: red));
      builder.addText('labore et dolore magna aliqua.');
    }

    final CanvasParagraph paragraph = rich(
      EngineParagraphStyle(fontFamily: 'Roboto', fontSize: 20.0, textAlign: TextAlign.justify),
      build,
    );
    paragraph.layout(constrain(250.0));

    // Draw selection for "em $rtlWord d".
    fillBoxes(canvas, Offset.zero, paragraph.getBoxesForRange(3, 13), lightBlue);
    // Draw selection for " ut labore et dolore mag".
    fillBoxes(canvas, Offset.zero, paragraph.getBoxesForRange(89, 113), lightPurple);

    canvas.drawParagraph(paragraph, Offset.zero);
  }

  test('TextAlign.justify rtl', () {
    const Rect bounds = Rect.fromLTWH(0, 0, 400, 400);
    final BitmapCanvas canvas = BitmapCanvas(bounds, RenderStrategy());
    testJustifyRtl(canvas);
    return takeScreenshot(canvas, bounds, 'canvas_paragraph_justify_rtl');
  });

  test('TextAlign.justify rtl (DOM)', () {
    const Rect bounds = Rect.fromLTWH(0, 0, 400, 400);
    final DomCanvas canvas = DomCanvas(domDocument.createElement('flt-picture'));
    testJustifyRtl(canvas);
    return takeScreenshot(canvas, bounds, 'canvas_paragraph_justify_rtl_dom');
  });
}

EngineTextStyle bg(Color color) {
  return EngineTextStyle.only(background: Paint()..color = color);
}
