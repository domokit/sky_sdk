// Copyright 2013 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

import 'package:test/bootstrap/browser.dart';
import 'package:test/test.dart';
import 'package:ui/src/engine.dart';
import 'package:ui/ui.dart' hide window;

import '../../common/test_initialization.dart';
import 'helper.dart';

const String _rtlWord1 = 'واحد';
const String _rtlWord2 = 'اثنان';

void main() {
  internalBootstrapBrowserTest(() => testMain);
}

Future<void> testMain() async {
  setUpUnitTests(
    withImplicitView: true,
    emulateTesterEnvironment: false,
    setUpTestViewDimensions: false,
  );

  void paintBasicBidiStartingWithLtr(
    EngineCanvas canvas,
    Rect bounds,
    double y,
    TextDirection textDirection,
    TextAlign textAlign,
  ) {
    // The text starts with a left-to-right word.
    const String text = 'One 12 $_rtlWord1 $_rtlWord2 34 two 56';

    final EngineParagraphStyle paragraphStyle = EngineParagraphStyle(
      fontFamily: 'Roboto',
      fontSize: 20.0,
      textDirection: textDirection,
      textAlign: textAlign,
    );
    final CanvasParagraph paragraph = plain(
      paragraphStyle,
      text,
      textStyle: EngineTextStyle.only(color: blue),
    );
    final double maxWidth = bounds.width - 10;
    paragraph.layout(constrain(maxWidth));
    canvas.drawParagraph(paragraph, Offset(bounds.left + 5, bounds.top + y + 5));
  }

  test('basic bidi starting with ltr', () {
    const Rect bounds = Rect.fromLTWH(0, 0, 340, 600);
    final BitmapCanvas canvas = BitmapCanvas(bounds, RenderStrategy());

    const double height = 40;

    // Border for ltr paragraphs.
    final Rect ltrBox = const Rect.fromLTWH(0, 0, 320, 5 * height).inflate(5).translate(10, 10);
    canvas.drawRect(
      ltrBox,
      SurfacePaintData()
        ..color = black.value
        ..style = PaintingStyle.stroke,
    );
    // LTR with different text align values:
    paintBasicBidiStartingWithLtr(canvas, ltrBox, 0 * height, TextDirection.ltr, TextAlign.left);
    paintBasicBidiStartingWithLtr(canvas, ltrBox, 1 * height, TextDirection.ltr, TextAlign.right);
    paintBasicBidiStartingWithLtr(canvas, ltrBox, 2 * height, TextDirection.ltr, TextAlign.center);
    paintBasicBidiStartingWithLtr(canvas, ltrBox, 3 * height, TextDirection.ltr, TextAlign.start);
    paintBasicBidiStartingWithLtr(canvas, ltrBox, 4 * height, TextDirection.ltr, TextAlign.end);

    // Border for rtl paragraphs.
    final Rect rtlBox = ltrBox.translate(0, ltrBox.height + 10);
    canvas.drawRect(
      rtlBox,
      SurfacePaintData()
        ..color = black.value
        ..style = PaintingStyle.stroke,
    );
    // RTL with different text align values:
    paintBasicBidiStartingWithLtr(canvas, rtlBox, 0 * height, TextDirection.rtl, TextAlign.left);
    paintBasicBidiStartingWithLtr(canvas, rtlBox, 1 * height, TextDirection.rtl, TextAlign.right);
    paintBasicBidiStartingWithLtr(canvas, rtlBox, 2 * height, TextDirection.rtl, TextAlign.center);
    paintBasicBidiStartingWithLtr(canvas, rtlBox, 3 * height, TextDirection.rtl, TextAlign.start);
    paintBasicBidiStartingWithLtr(canvas, rtlBox, 4 * height, TextDirection.rtl, TextAlign.end);

    return takeScreenshot(canvas, bounds, 'canvas_paragraph_bidi_start_ltr');
  });

  test('basic bidi starting with ltr (DOM)', () {
    const Rect bounds = Rect.fromLTWH(0, 0, 340, 600);
    final DomCanvas canvas = DomCanvas(domDocument.createElement('flt-picture'));

    const double height = 40;

    // Border for ltr paragraphs.
    final Rect ltrBox = const Rect.fromLTWH(0, 0, 320, 5 * height).inflate(5).translate(10, 10);
    canvas.drawRect(
      ltrBox,
      SurfacePaintData()
        ..color = black.value
        ..style = PaintingStyle.stroke,
    );
    // LTR with different text align values:
    paintBasicBidiStartingWithLtr(canvas, ltrBox, 0 * height, TextDirection.ltr, TextAlign.left);
    paintBasicBidiStartingWithLtr(canvas, ltrBox, 1 * height, TextDirection.ltr, TextAlign.right);
    paintBasicBidiStartingWithLtr(canvas, ltrBox, 2 * height, TextDirection.ltr, TextAlign.center);
    paintBasicBidiStartingWithLtr(canvas, ltrBox, 3 * height, TextDirection.ltr, TextAlign.start);
    paintBasicBidiStartingWithLtr(canvas, ltrBox, 4 * height, TextDirection.ltr, TextAlign.end);

    // Border for rtl paragraphs.
    final Rect rtlBox = ltrBox.translate(0, ltrBox.height + 10);
    canvas.drawRect(
      rtlBox,
      SurfacePaintData()
        ..color = black.value
        ..style = PaintingStyle.stroke,
    );
    // RTL with different text align values:
    paintBasicBidiStartingWithLtr(canvas, rtlBox, 0 * height, TextDirection.rtl, TextAlign.left);
    paintBasicBidiStartingWithLtr(canvas, rtlBox, 1 * height, TextDirection.rtl, TextAlign.right);
    paintBasicBidiStartingWithLtr(canvas, rtlBox, 2 * height, TextDirection.rtl, TextAlign.center);
    paintBasicBidiStartingWithLtr(canvas, rtlBox, 3 * height, TextDirection.rtl, TextAlign.start);
    paintBasicBidiStartingWithLtr(canvas, rtlBox, 4 * height, TextDirection.rtl, TextAlign.end);

    return takeScreenshot(canvas, bounds, 'canvas_paragraph_bidi_start_ltr_dom');
  });

  void paintBasicBidiStartingWithRtl(
    EngineCanvas canvas,
    Rect bounds,
    double y,
    TextDirection textDirection,
    TextAlign textAlign,
  ) {
    // The text starts with a right-to-left word.
    const String text = '$_rtlWord1 12 one 34 $_rtlWord2 56 two';

    final EngineParagraphStyle paragraphStyle = EngineParagraphStyle(
      fontFamily: 'Roboto',
      fontSize: 20.0,
      textDirection: textDirection,
      textAlign: textAlign,
    );
    final CanvasParagraph paragraph = plain(
      paragraphStyle,
      text,
      textStyle: EngineTextStyle.only(color: blue),
    );
    final double maxWidth = bounds.width - 10;
    paragraph.layout(constrain(maxWidth));
    canvas.drawParagraph(paragraph, Offset(bounds.left + 5, bounds.top + y + 5));
  }

  test('basic bidi starting with rtl', () {
    const Rect bounds = Rect.fromLTWH(0, 0, 340, 600);
    final BitmapCanvas canvas = BitmapCanvas(bounds, RenderStrategy());

    const double height = 40;

    // Border for ltr paragraphs.
    final Rect ltrBox = const Rect.fromLTWH(0, 0, 320, 5 * height).inflate(5).translate(10, 10);
    canvas.drawRect(
      ltrBox,
      SurfacePaintData()
        ..color = black.value
        ..style = PaintingStyle.stroke,
    );
    // LTR with different text align values:
    paintBasicBidiStartingWithRtl(canvas, ltrBox, 0 * height, TextDirection.ltr, TextAlign.left);
    paintBasicBidiStartingWithRtl(canvas, ltrBox, 1 * height, TextDirection.ltr, TextAlign.right);
    paintBasicBidiStartingWithRtl(canvas, ltrBox, 2 * height, TextDirection.ltr, TextAlign.center);
    paintBasicBidiStartingWithRtl(canvas, ltrBox, 3 * height, TextDirection.ltr, TextAlign.start);
    paintBasicBidiStartingWithRtl(canvas, ltrBox, 4 * height, TextDirection.ltr, TextAlign.end);

    // Border for rtl paragraphs.
    final Rect rtlBox = ltrBox.translate(0, ltrBox.height + 10);
    canvas.drawRect(
      rtlBox,
      SurfacePaintData()
        ..color = black.value
        ..style = PaintingStyle.stroke,
    );
    // RTL with different text align values:
    paintBasicBidiStartingWithRtl(canvas, rtlBox, 0 * height, TextDirection.rtl, TextAlign.left);
    paintBasicBidiStartingWithRtl(canvas, rtlBox, 1 * height, TextDirection.rtl, TextAlign.right);
    paintBasicBidiStartingWithRtl(canvas, rtlBox, 2 * height, TextDirection.rtl, TextAlign.center);
    paintBasicBidiStartingWithRtl(canvas, rtlBox, 3 * height, TextDirection.rtl, TextAlign.start);
    paintBasicBidiStartingWithRtl(canvas, rtlBox, 4 * height, TextDirection.rtl, TextAlign.end);

    return takeScreenshot(canvas, bounds, 'canvas_paragraph_bidi_start_rtl');
  });

  test('basic bidi starting with rtl (DOM)', () {
    const Rect bounds = Rect.fromLTWH(0, 0, 340, 600);
    final DomCanvas canvas = DomCanvas(domDocument.createElement('flt-picture'));

    const double height = 40;

    // Border for ltr paragraphs.
    final Rect ltrBox = const Rect.fromLTWH(0, 0, 320, 5 * height).inflate(5).translate(10, 10);
    canvas.drawRect(
      ltrBox,
      SurfacePaintData()
        ..color = black.value
        ..style = PaintingStyle.stroke,
    );
    // LTR with different text align values:
    paintBasicBidiStartingWithRtl(canvas, ltrBox, 0 * height, TextDirection.ltr, TextAlign.left);
    paintBasicBidiStartingWithRtl(canvas, ltrBox, 1 * height, TextDirection.ltr, TextAlign.right);
    paintBasicBidiStartingWithRtl(canvas, ltrBox, 2 * height, TextDirection.ltr, TextAlign.center);
    paintBasicBidiStartingWithRtl(canvas, ltrBox, 3 * height, TextDirection.ltr, TextAlign.start);
    paintBasicBidiStartingWithRtl(canvas, ltrBox, 4 * height, TextDirection.ltr, TextAlign.end);

    // Border for rtl paragraphs.
    final Rect rtlBox = ltrBox.translate(0, ltrBox.height + 10);
    canvas.drawRect(
      rtlBox,
      SurfacePaintData()
        ..color = black.value
        ..style = PaintingStyle.stroke,
    );
    // RTL with different text align values:
    paintBasicBidiStartingWithRtl(canvas, rtlBox, 0 * height, TextDirection.rtl, TextAlign.left);
    paintBasicBidiStartingWithRtl(canvas, rtlBox, 1 * height, TextDirection.rtl, TextAlign.right);
    paintBasicBidiStartingWithRtl(canvas, rtlBox, 2 * height, TextDirection.rtl, TextAlign.center);
    paintBasicBidiStartingWithRtl(canvas, rtlBox, 3 * height, TextDirection.rtl, TextAlign.start);
    paintBasicBidiStartingWithRtl(canvas, rtlBox, 4 * height, TextDirection.rtl, TextAlign.end);

    return takeScreenshot(canvas, bounds, 'canvas_paragraph_bidi_start_rtl_dom');
  });

  void paintMultilineBidi(
    EngineCanvas canvas,
    Rect bounds,
    double y,
    TextDirection textDirection,
    TextAlign textAlign,
  ) {
    // '''
    // Lorem 12 $_rtlWord1
    // $_rtlWord2 34 ipsum
    // dolor 56
    // '''
    const String text = 'Lorem 12 $_rtlWord1 $_rtlWord2 34 ipsum dolor 56';

    final EngineParagraphStyle paragraphStyle = EngineParagraphStyle(
      fontFamily: 'Roboto',
      fontSize: 20.0,
      textDirection: textDirection,
      textAlign: textAlign,
    );
    final CanvasParagraph paragraph = plain(
      paragraphStyle,
      text,
      textStyle: EngineTextStyle.only(color: blue),
    );
    final double maxWidth = bounds.width - 10;
    paragraph.layout(constrain(maxWidth));
    canvas.drawParagraph(paragraph, Offset(bounds.left + 5, bounds.top + y + 5));
  }

  test('multiline bidi', () {
    const Rect bounds = Rect.fromLTWH(0, 0, 400, 500);
    final BitmapCanvas canvas = BitmapCanvas(bounds, RenderStrategy());

    const double height = 95;

    // Border for ltr paragraphs.
    final Rect ltrBox = const Rect.fromLTWH(0, 0, 150, 5 * height).inflate(5).translate(10, 10);
    canvas.drawRect(
      ltrBox,
      SurfacePaintData()
        ..color = black.value
        ..style = PaintingStyle.stroke,
    );
    // LTR with different text align values:
    paintMultilineBidi(canvas, ltrBox, 0 * height, TextDirection.ltr, TextAlign.left);
    paintMultilineBidi(canvas, ltrBox, 1 * height, TextDirection.ltr, TextAlign.right);
    paintMultilineBidi(canvas, ltrBox, 2 * height, TextDirection.ltr, TextAlign.center);
    paintMultilineBidi(canvas, ltrBox, 3 * height, TextDirection.ltr, TextAlign.start);
    paintMultilineBidi(canvas, ltrBox, 4 * height, TextDirection.ltr, TextAlign.end);

    // Border for rtl paragraphs.
    final Rect rtlBox = ltrBox.translate(ltrBox.width + 10, 0);
    canvas.drawRect(
      rtlBox,
      SurfacePaintData()
        ..color = black.value
        ..style = PaintingStyle.stroke,
    );
    // RTL with different text align values:
    paintMultilineBidi(canvas, rtlBox, 0 * height, TextDirection.rtl, TextAlign.left);
    paintMultilineBidi(canvas, rtlBox, 1 * height, TextDirection.rtl, TextAlign.right);
    paintMultilineBidi(canvas, rtlBox, 2 * height, TextDirection.rtl, TextAlign.center);
    paintMultilineBidi(canvas, rtlBox, 3 * height, TextDirection.rtl, TextAlign.start);
    paintMultilineBidi(canvas, rtlBox, 4 * height, TextDirection.rtl, TextAlign.end);

    return takeScreenshot(canvas, bounds, 'canvas_paragraph_bidi_multiline');
  });

  test('multiline bidi (DOM)', () {
    const Rect bounds = Rect.fromLTWH(0, 0, 400, 500);
    final DomCanvas canvas = DomCanvas(domDocument.createElement('flt-picture'));

    const double height = 95;

    final Rect ltrBox = const Rect.fromLTWH(0, 0, 150, 5 * height).inflate(5).translate(10, 10);
    canvas.drawRect(
      ltrBox,
      SurfacePaintData()
        ..color = black.value
        ..style = PaintingStyle.stroke,
    );
    // LTR with different text align values:
    paintMultilineBidi(canvas, ltrBox, 0 * height, TextDirection.ltr, TextAlign.left);
    paintMultilineBidi(canvas, ltrBox, 1 * height, TextDirection.ltr, TextAlign.right);
    paintMultilineBidi(canvas, ltrBox, 2 * height, TextDirection.ltr, TextAlign.center);
    paintMultilineBidi(canvas, ltrBox, 3 * height, TextDirection.ltr, TextAlign.start);
    paintMultilineBidi(canvas, ltrBox, 4 * height, TextDirection.ltr, TextAlign.end);

    // Border for rtl paragraphs.
    final Rect rtlBox = ltrBox.translate(ltrBox.width + 10, 0);
    canvas.drawRect(
      rtlBox,
      SurfacePaintData()
        ..color = black.value
        ..style = PaintingStyle.stroke,
    );
    // RTL with different text align values:
    paintMultilineBidi(canvas, rtlBox, 0 * height, TextDirection.rtl, TextAlign.left);
    paintMultilineBidi(canvas, rtlBox, 1 * height, TextDirection.rtl, TextAlign.right);
    paintMultilineBidi(canvas, rtlBox, 2 * height, TextDirection.rtl, TextAlign.center);
    paintMultilineBidi(canvas, rtlBox, 3 * height, TextDirection.rtl, TextAlign.start);
    paintMultilineBidi(canvas, rtlBox, 4 * height, TextDirection.rtl, TextAlign.end);

    return takeScreenshot(canvas, bounds, 'canvas_paragraph_bidi_multiline_dom');
  });

  void paintMultSpanBidi(
    EngineCanvas canvas,
    Rect bounds,
    double y,
    TextDirection textDirection,
    TextAlign textAlign,
  ) {
    final EngineParagraphStyle paragraphStyle = EngineParagraphStyle(
      fontFamily: 'Roboto',
      fontSize: 20.0,
      textDirection: textDirection,
      textAlign: textAlign,
    );
    // '''
    // Lorem 12 $_rtlWord1
    // $_rtlWord2 34 ipsum
    // dolor 56
    // '''
    final CanvasParagraph paragraph = rich(paragraphStyle, (CanvasParagraphBuilder builder) {
      builder.pushStyle(EngineTextStyle.only(color: blue));
      builder.addText('Lorem ');
      builder.pushStyle(EngineTextStyle.only(color: green));
      builder.addText('12 ');
      builder.pushStyle(EngineTextStyle.only(color: red));
      builder.addText('$_rtlWord1 ');
      builder.pushStyle(EngineTextStyle.only(color: black));
      builder.addText('$_rtlWord2 ');
      builder.pushStyle(EngineTextStyle.only(color: blue));
      builder.addText('34 ipsum ');
      builder.pushStyle(EngineTextStyle.only(color: green));
      builder.addText('dolor 56 ');
    });
    final double maxWidth = bounds.width - 10;
    paragraph.layout(constrain(maxWidth));
    canvas.drawParagraph(paragraph, Offset(bounds.left + 5, bounds.top + y + 5));
  }

  test('multi span bidi', () {
    const Rect bounds = Rect.fromLTWH(0, 0, 400, 900);
    final BitmapCanvas canvas = BitmapCanvas(bounds, RenderStrategy());

    const double height = 95;

    // Border for ltr paragraphs.
    final Rect ltrBox = const Rect.fromLTWH(0, 0, 150, 5 * height).inflate(5).translate(10, 10);
    canvas.drawRect(
      ltrBox,
      SurfacePaintData()
        ..color = black.value
        ..style = PaintingStyle.stroke,
    );
    // LTR with different text align values:
    paintMultSpanBidi(canvas, ltrBox, 0 * height, TextDirection.ltr, TextAlign.left);
    paintMultSpanBidi(canvas, ltrBox, 1 * height, TextDirection.ltr, TextAlign.right);
    paintMultSpanBidi(canvas, ltrBox, 2 * height, TextDirection.ltr, TextAlign.center);
    paintMultSpanBidi(canvas, ltrBox, 3 * height, TextDirection.ltr, TextAlign.start);
    paintMultSpanBidi(canvas, ltrBox, 4 * height, TextDirection.ltr, TextAlign.end);

    // Border for rtl paragraphs.
    final Rect rtlBox = ltrBox.translate(ltrBox.width + 10, 0);
    canvas.drawRect(
      rtlBox,
      SurfacePaintData()
        ..color = black.value
        ..style = PaintingStyle.stroke,
    );
    // RTL with different text align values:
    paintMultSpanBidi(canvas, rtlBox, 0 * height, TextDirection.rtl, TextAlign.left);
    paintMultSpanBidi(canvas, rtlBox, 1 * height, TextDirection.rtl, TextAlign.right);
    paintMultSpanBidi(canvas, rtlBox, 2 * height, TextDirection.rtl, TextAlign.center);
    paintMultSpanBidi(canvas, rtlBox, 3 * height, TextDirection.rtl, TextAlign.start);
    paintMultSpanBidi(canvas, rtlBox, 4 * height, TextDirection.rtl, TextAlign.end);

    return takeScreenshot(canvas, bounds, 'canvas_paragraph_bidi_multispan');
  });

  test('multi span bidi (DOM)', () {
    const Rect bounds = Rect.fromLTWH(0, 0, 400, 900);
    final DomCanvas canvas = DomCanvas(domDocument.createElement('flt-picture'));

    const double height = 95;

    // Border for ltr paragraphs.
    final Rect ltrBox = const Rect.fromLTWH(0, 0, 150, 5 * height).inflate(5).translate(10, 10);
    canvas.drawRect(
      ltrBox,
      SurfacePaintData()
        ..color = black.value
        ..style = PaintingStyle.stroke,
    );
    // LTR with different text align values:
    paintMultSpanBidi(canvas, ltrBox, 0 * height, TextDirection.ltr, TextAlign.left);
    paintMultSpanBidi(canvas, ltrBox, 1 * height, TextDirection.ltr, TextAlign.right);
    paintMultSpanBidi(canvas, ltrBox, 2 * height, TextDirection.ltr, TextAlign.center);
    paintMultSpanBidi(canvas, ltrBox, 3 * height, TextDirection.ltr, TextAlign.start);
    paintMultSpanBidi(canvas, ltrBox, 4 * height, TextDirection.ltr, TextAlign.end);

    // Border for rtl paragraphs.
    final Rect rtlBox = ltrBox.translate(ltrBox.width + 10, 0);
    canvas.drawRect(
      rtlBox,
      SurfacePaintData()
        ..color = black.value
        ..style = PaintingStyle.stroke,
    );
    // RTL with different text align values:
    paintMultSpanBidi(canvas, rtlBox, 0 * height, TextDirection.rtl, TextAlign.left);
    paintMultSpanBidi(canvas, rtlBox, 1 * height, TextDirection.rtl, TextAlign.right);
    paintMultSpanBidi(canvas, rtlBox, 2 * height, TextDirection.rtl, TextAlign.center);
    paintMultSpanBidi(canvas, rtlBox, 3 * height, TextDirection.rtl, TextAlign.start);
    paintMultSpanBidi(canvas, rtlBox, 4 * height, TextDirection.rtl, TextAlign.end);

    return takeScreenshot(canvas, bounds, 'canvas_paragraph_bidi_multispan_dom');
  });

  void paintBidiWithSelection(
    EngineCanvas canvas,
    Rect bounds,
    double y,
    TextDirection textDirection,
    TextAlign textAlign,
  ) {
    // '''
    // Lorem 12 $_rtlWord1
    // $_rtlWord2 34 ipsum
    // dolor 56
    // '''
    const String text = 'Lorem 12 $_rtlWord1 $_rtlWord2 34 ipsum dolor 56';

    final EngineParagraphStyle paragraphStyle = EngineParagraphStyle(
      fontFamily: 'Roboto',
      fontSize: 20.0,
      textDirection: textDirection,
      textAlign: textAlign,
    );
    final CanvasParagraph paragraph = plain(
      paragraphStyle,
      text,
      textStyle: EngineTextStyle.only(color: blue),
    );

    final double maxWidth = bounds.width - 10;
    paragraph.layout(constrain(maxWidth));

    final Offset offset = Offset(bounds.left + 5, bounds.top + y + 5);

    // Range for "em 12 " and the first character of `_rtlWord1`.
    fillBoxes(canvas, offset, paragraph.getBoxesForRange(3, 10), lightBlue);
    // Range for the second half of `_rtlWord1` and all of `_rtlWord2` and " 3".
    fillBoxes(canvas, offset, paragraph.getBoxesForRange(11, 21), lightPurple);
    // Range for "psum dolo".
    fillBoxes(canvas, offset, paragraph.getBoxesForRange(24, 33), green);

    canvas.drawParagraph(paragraph, offset);
  }

  test('bidi with selection', () {
    const Rect bounds = Rect.fromLTWH(0, 0, 400, 500);
    final BitmapCanvas canvas = BitmapCanvas(bounds, RenderStrategy());

    const double height = 95;

    // Border for ltr paragraphs.
    final Rect ltrBox = const Rect.fromLTWH(0, 0, 150, 5 * height).inflate(5).translate(10, 10);
    canvas.drawRect(
      ltrBox,
      SurfacePaintData()
        ..color = black.value
        ..style = PaintingStyle.stroke,
    );
    // LTR with different text align values:
    paintBidiWithSelection(canvas, ltrBox, 0 * height, TextDirection.ltr, TextAlign.left);
    paintBidiWithSelection(canvas, ltrBox, 1 * height, TextDirection.ltr, TextAlign.right);
    paintBidiWithSelection(canvas, ltrBox, 2 * height, TextDirection.ltr, TextAlign.center);
    paintBidiWithSelection(canvas, ltrBox, 3 * height, TextDirection.ltr, TextAlign.start);
    paintBidiWithSelection(canvas, ltrBox, 4 * height, TextDirection.ltr, TextAlign.end);

    // Border for rtl paragraphs.
    final Rect rtlBox = ltrBox.translate(ltrBox.width + 10, 0);
    canvas.drawRect(
      rtlBox,
      SurfacePaintData()
        ..color = black.value
        ..style = PaintingStyle.stroke,
    );
    // RTL with different text align values:
    paintBidiWithSelection(canvas, rtlBox, 0 * height, TextDirection.rtl, TextAlign.left);
    paintBidiWithSelection(canvas, rtlBox, 1 * height, TextDirection.rtl, TextAlign.right);
    paintBidiWithSelection(canvas, rtlBox, 2 * height, TextDirection.rtl, TextAlign.center);
    paintBidiWithSelection(canvas, rtlBox, 3 * height, TextDirection.rtl, TextAlign.start);
    paintBidiWithSelection(canvas, rtlBox, 4 * height, TextDirection.rtl, TextAlign.end);

    return takeScreenshot(canvas, bounds, 'canvas_paragraph_bidi_selection');
  });

  test('bidi with selection (DOM)', () {
    const Rect bounds = Rect.fromLTWH(0, 0, 400, 500);
    final DomCanvas canvas = DomCanvas(domDocument.createElement('flt-picture'));

    const double height = 95;

    // Border for ltr paragraphs.
    final Rect ltrBox = const Rect.fromLTWH(0, 0, 150, 5 * height).inflate(5).translate(10, 10);
    canvas.drawRect(
      ltrBox,
      SurfacePaintData()
        ..color = black.value
        ..style = PaintingStyle.stroke,
    );
    // LTR with different text align values:
    paintBidiWithSelection(canvas, ltrBox, 0 * height, TextDirection.ltr, TextAlign.left);
    paintBidiWithSelection(canvas, ltrBox, 1 * height, TextDirection.ltr, TextAlign.right);
    paintBidiWithSelection(canvas, ltrBox, 2 * height, TextDirection.ltr, TextAlign.center);
    paintBidiWithSelection(canvas, ltrBox, 3 * height, TextDirection.ltr, TextAlign.start);
    paintBidiWithSelection(canvas, ltrBox, 4 * height, TextDirection.ltr, TextAlign.end);

    // Border for rtl paragraphs.
    final Rect rtlBox = ltrBox.translate(ltrBox.width + 10, 0);
    canvas.drawRect(
      rtlBox,
      SurfacePaintData()
        ..color = black.value
        ..style = PaintingStyle.stroke,
    );
    // RTL with different text align values:
    paintBidiWithSelection(canvas, rtlBox, 0 * height, TextDirection.rtl, TextAlign.left);
    paintBidiWithSelection(canvas, rtlBox, 1 * height, TextDirection.rtl, TextAlign.right);
    paintBidiWithSelection(canvas, rtlBox, 2 * height, TextDirection.rtl, TextAlign.center);
    paintBidiWithSelection(canvas, rtlBox, 3 * height, TextDirection.rtl, TextAlign.start);
    paintBidiWithSelection(canvas, rtlBox, 4 * height, TextDirection.rtl, TextAlign.end);

    return takeScreenshot(canvas, bounds, 'canvas_paragraph_bidi_selection_dom');
  });
}
