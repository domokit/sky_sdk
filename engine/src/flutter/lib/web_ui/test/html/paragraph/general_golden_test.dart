// Copyright 2013 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

import 'dart:async';
import 'dart:math' as math;

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

  test('paints spans and lines correctly', () {
    final BitmapCanvas canvas = BitmapCanvas(bounds, RenderStrategy());

    Offset offset = Offset.zero;
    CanvasParagraph paragraph;

    // Single-line multi-span.
    paragraph = rich(EngineParagraphStyle(fontFamily: 'Roboto'), (CanvasParagraphBuilder builder) {
      builder.pushStyle(EngineTextStyle.only(color: blue));
      builder.addText('Lorem ');
      builder.pushStyle(EngineTextStyle.only(color: green, background: Paint()..color = red));
      builder.addText('ipsum ');
      builder.pop();
      builder.addText('.');
    })..layout(constrain(double.infinity));
    canvas.drawParagraph(paragraph, offset);
    offset = offset.translate(0, paragraph.height + 10);

    // Multi-line single-span.
    paragraph = rich(EngineParagraphStyle(fontFamily: 'Roboto'), (CanvasParagraphBuilder builder) {
      builder.addText('Lorem ipsum dolor sit');
    })..layout(constrain(90.0));
    canvas.drawParagraph(paragraph, offset);
    offset = offset.translate(0, paragraph.height + 10);

    // Multi-line multi-span.
    paragraph = rich(EngineParagraphStyle(fontFamily: 'Roboto'), (CanvasParagraphBuilder builder) {
      builder.pushStyle(EngineTextStyle.only(color: blue));
      builder.addText('Lorem ipsum ');
      builder.pushStyle(EngineTextStyle.only(background: Paint()..color = red));
      builder.pushStyle(EngineTextStyle.only(color: green));
      builder.addText('dolor ');
      builder.pop();
      builder.addText('sit');
    })..layout(constrain(90.0));
    canvas.drawParagraph(paragraph, offset);
    offset = offset.translate(0, paragraph.height + 10);

    return takeScreenshot(canvas, bounds, 'canvas_paragraph_general');
  });

  test('respects alignment', () {
    final BitmapCanvas canvas = BitmapCanvas(bounds, RenderStrategy());

    Offset offset = Offset.zero;
    CanvasParagraph paragraph;

    void build(CanvasParagraphBuilder builder) {
      builder.pushStyle(EngineTextStyle.only(color: black));
      builder.addText('Lorem ');
      builder.pushStyle(EngineTextStyle.only(color: blue));
      builder.addText('ipsum ');
      builder.pushStyle(EngineTextStyle.only(color: green));
      builder.addText('dolor ');
      builder.pushStyle(EngineTextStyle.only(color: red));
      builder.addText('sit');
    }

    paragraph = rich(EngineParagraphStyle(fontFamily: 'Roboto', textAlign: TextAlign.left), build)
      ..layout(constrain(100.0));
    canvas.drawParagraph(paragraph, offset);
    offset = offset.translate(0, paragraph.height + 10);

    paragraph = rich(EngineParagraphStyle(fontFamily: 'Roboto', textAlign: TextAlign.center), build)
      ..layout(constrain(100.0));
    canvas.drawParagraph(paragraph, offset);
    offset = offset.translate(0, paragraph.height + 10);

    paragraph = rich(EngineParagraphStyle(fontFamily: 'Roboto', textAlign: TextAlign.right), build)
      ..layout(constrain(100.0));
    canvas.drawParagraph(paragraph, offset);
    offset = offset.translate(0, paragraph.height + 10);

    return takeScreenshot(canvas, bounds, 'canvas_paragraph_align');
  });

  test('respects alignment in DOM mode', () {
    final DomCanvas canvas = DomCanvas(domDocument.createElement('flt-picture'));

    Offset offset = Offset.zero;
    CanvasParagraph paragraph;

    void build(CanvasParagraphBuilder builder) {
      builder.pushStyle(EngineTextStyle.only(color: black));
      builder.addText('Lorem ');
      builder.pushStyle(EngineTextStyle.only(color: blue));
      builder.addText('ipsum ');
      builder.pushStyle(EngineTextStyle.only(color: green));
      builder.addText('dolor ');
      builder.pushStyle(EngineTextStyle.only(color: red));
      builder.addText('sit');
    }

    paragraph = rich(EngineParagraphStyle(fontFamily: 'Roboto', textAlign: TextAlign.left), build)
      ..layout(constrain(100.0));
    canvas.drawParagraph(paragraph, offset);
    offset = offset.translate(0, paragraph.height + 10);

    paragraph = rich(EngineParagraphStyle(fontFamily: 'Roboto', textAlign: TextAlign.center), build)
      ..layout(constrain(100.0));
    canvas.drawParagraph(paragraph, offset);
    offset = offset.translate(0, paragraph.height + 10);

    paragraph = rich(EngineParagraphStyle(fontFamily: 'Roboto', textAlign: TextAlign.right), build)
      ..layout(constrain(100.0));
    canvas.drawParagraph(paragraph, offset);
    offset = offset.translate(0, paragraph.height + 10);

    return takeScreenshot(canvas, bounds, 'canvas_paragraph_align_dom');
  });

  void testAlignAndTransform(EngineCanvas canvas) {
    CanvasParagraph paragraph;

    void build(CanvasParagraphBuilder builder) {
      builder.pushStyle(EngineTextStyle.only(color: white));
      builder.addText('Lorem ');
      builder.pushStyle(EngineTextStyle.only(color: red));
      builder.addText('ipsum\n');
      builder.pushStyle(EngineTextStyle.only(color: yellow));
      builder.addText('dolor');
    }

    void drawParagraphAt(Offset offset, TextAlign align) {
      paragraph = rich(
        EngineParagraphStyle(fontFamily: 'Roboto', fontSize: 20.0, textAlign: align),
        build,
      )..layout(constrain(150.0));
      canvas.save();
      canvas.translate(offset.dx, offset.dy);
      canvas.rotate(math.pi / 4);
      final Rect rect = Rect.fromLTRB(0.0, 0.0, 150.0, paragraph.height);
      canvas.drawRect(rect, SurfacePaintData()..color = black.value);
      canvas.drawParagraph(paragraph, Offset.zero);
      canvas.restore();
    }

    drawParagraphAt(const Offset(50.0, 0.0), TextAlign.left);
    drawParagraphAt(const Offset(150.0, 0.0), TextAlign.center);
    drawParagraphAt(const Offset(250.0, 0.0), TextAlign.right);
  }

  test('alignment and transform', () {
    final BitmapCanvas canvas = BitmapCanvas(bounds, RenderStrategy());
    testAlignAndTransform(canvas);
    return takeScreenshot(canvas, bounds, 'canvas_paragraph_align_transform');
  });

  test('alignment and transform (DOM)', () {
    final DomCanvas canvas = DomCanvas(domDocument.createElement('flt-picture'));
    testAlignAndTransform(canvas);
    return takeScreenshot(canvas, bounds, 'canvas_paragraph_align_transform_dom');
  });

  void testGiantParagraphStyles(EngineCanvas canvas) {
    final CanvasParagraph paragraph = rich(
      EngineParagraphStyle(fontFamily: 'Roboto', fontSize: 80.0),
      (CanvasParagraphBuilder builder) {
        builder.pushStyle(EngineTextStyle.only(color: yellow, fontSize: 24.0));
        builder.addText('Lorem ');
        builder.pushStyle(EngineTextStyle.only(color: red, fontSize: 32.0));
        builder.addText('ipsum');
      },
    )..layout(constrain(double.infinity));
    final Rect rect = Rect.fromLTRB(0.0, 0.0, paragraph.maxIntrinsicWidth, paragraph.height);
    canvas.drawRect(rect, SurfacePaintData()..color = black.value);
    canvas.drawParagraph(paragraph, Offset.zero);
  }

  test('giant paragraph style', () {
    const Rect bounds = Rect.fromLTWH(0, 0, 300, 200);
    final BitmapCanvas canvas = BitmapCanvas(bounds, RenderStrategy());
    testGiantParagraphStyles(canvas);
    return takeScreenshot(canvas, bounds, 'canvas_paragraph_giant_paragraph_style');
  });

  test('giant paragraph style (DOM)', () {
    const Rect bounds = Rect.fromLTWH(0, 0, 300, 200);
    final DomCanvas canvas = DomCanvas(domDocument.createElement('flt-picture'));
    testGiantParagraphStyles(canvas);
    return takeScreenshot(canvas, bounds, 'canvas_paragraph_giant_paragraph_style_dom');
  });

  test('giant font size on the body tag (DOM)', () async {
    const Rect bounds = Rect.fromLTWH(0, 0, 600, 200);

    // Store the old font size value on the body, and set a gaint font size.
    final String oldBodyFontSize = domDocument.body!.style.fontSize;
    domDocument.body!.style.fontSize = '100px';

    final DomCanvas canvas = DomCanvas(domDocument.createElement('flt-picture'));
    Offset offset = const Offset(10.0, 10.0);

    final CanvasParagraph paragraph = rich(EngineParagraphStyle(fontFamily: 'Roboto'), (
      CanvasParagraphBuilder builder,
    ) {
      builder.pushStyle(EngineTextStyle.only(color: yellow, fontSize: 24.0));
      builder.addText('Lorem ');
      builder.pushStyle(EngineTextStyle.only(color: red, fontSize: 48.0));
      builder.addText('ipsum');
    })..layout(constrain(double.infinity));
    final Rect rect = Rect.fromLTWH(
      offset.dx,
      offset.dy,
      paragraph.maxIntrinsicWidth,
      paragraph.height,
    );
    canvas.drawRect(rect, SurfacePaintData()..color = black.value);
    canvas.drawParagraph(paragraph, offset);
    offset = offset.translate(paragraph.maxIntrinsicWidth, 0.0);

    // Add some extra padding between the two paragraphs.
    offset = offset.translate(20.0, 0.0);

    // Use the same height as the previous paragraph so that the 2 paragraphs
    // look nice in the screenshot.
    final double placeholderHeight = paragraph.height;
    final double placeholderWidth = paragraph.height * 2;

    final CanvasParagraph paragraph2 = rich(EngineParagraphStyle(), (
      CanvasParagraphBuilder builder,
    ) {
      builder.addPlaceholder(
        placeholderWidth,
        placeholderHeight,
        PlaceholderAlignment.baseline,
        baseline: TextBaseline.alphabetic,
      );
    })..layout(constrain(double.infinity));
    final Rect rect2 = Rect.fromLTWH(
      offset.dx,
      offset.dy,
      paragraph2.maxIntrinsicWidth,
      paragraph2.height,
    );
    canvas.drawRect(rect2, SurfacePaintData()..color = black.value);
    canvas.drawParagraph(paragraph2, offset);
    // Draw a rect in the placeholder.
    // Leave some padding around the placeholder to make the black paragraph
    // background visible.
    const double padding = 5;
    final TextBox placeholderBox = paragraph2.getBoxesForPlaceholders().single;
    canvas.drawRect(
      placeholderBox.toRect().shift(offset).deflate(padding),
      SurfacePaintData()..color = red.value,
    );

    await takeScreenshot(canvas, bounds, 'canvas_paragraph_giant_body_font_size_dom');

    // Restore the old font size value.
    domDocument.body!.style.fontSize = oldBodyFontSize;
  });

  test('paints spans with varying heights/baselines', () {
    final BitmapCanvas canvas = BitmapCanvas(bounds, RenderStrategy());

    final CanvasParagraph paragraph = rich(EngineParagraphStyle(fontFamily: 'Roboto'), (
      CanvasParagraphBuilder builder,
    ) {
      builder.pushStyle(EngineTextStyle.only(fontSize: 20.0));
      builder.addText('Lorem ');
      builder.pushStyle(EngineTextStyle.only(fontSize: 40.0, background: Paint()..color = green));
      builder.addText('ipsum ');
      builder.pushStyle(
        EngineTextStyle.only(fontSize: 10.0, color: white, background: Paint()..color = black),
      );
      builder.addText('dolor ');
      builder.pushStyle(EngineTextStyle.only(fontSize: 30.0));
      builder.addText('sit ');
      builder.pop();
      builder.pop();
      builder.pushStyle(EngineTextStyle.only(fontSize: 20.0, background: Paint()..color = blue));
      builder.addText('amet');
    })..layout(constrain(220.0));
    canvas.drawParagraph(paragraph, Offset.zero);

    return takeScreenshot(canvas, bounds, 'canvas_paragraph_varying_heights');
  });

  test('respects letter-spacing', () {
    final BitmapCanvas canvas = BitmapCanvas(bounds, RenderStrategy());

    final CanvasParagraph paragraph = rich(EngineParagraphStyle(fontFamily: 'Roboto'), (
      CanvasParagraphBuilder builder,
    ) {
      builder.pushStyle(EngineTextStyle.only(color: blue));
      builder.addText('Lorem ');
      builder.pushStyle(EngineTextStyle.only(color: green, letterSpacing: 1));
      builder.addText('Lorem ');
      builder.pushStyle(EngineTextStyle.only(color: red, letterSpacing: 3));
      builder.addText('Lorem');
    })..layout(constrain(double.infinity));
    canvas.drawParagraph(paragraph, Offset.zero);

    return takeScreenshot(canvas, bounds, 'canvas_paragraph_letter_spacing');
  });

  test('letter-spacing Thai', () {
    final BitmapCanvas canvas = BitmapCanvas(bounds, RenderStrategy());

    const String yes = '\u0e43\u0e0a\u0e48';
    const String no = '\u0e44\u0e21\u0e48';

    final CanvasParagraph paragraph = rich(
      EngineParagraphStyle(fontFamily: 'Roboto', fontSize: 36),
      (CanvasParagraphBuilder builder) {
        builder.pushStyle(EngineTextStyle.only(color: blue));
        builder.addText('$yes $no ');
        builder.pushStyle(EngineTextStyle.only(color: green, letterSpacing: 1));
        builder.addText('$yes $no ');
        builder.pushStyle(EngineTextStyle.only(color: red, letterSpacing: 3));
        builder.addText('$yes $no');
      },
    )..layout(constrain(double.infinity));
    canvas.drawParagraph(paragraph, const Offset(20, 20));

    return takeScreenshot(canvas, bounds, 'canvas_paragraph_letter_spacing_thai');
  });

  test('draws text decorations', () {
    final BitmapCanvas canvas = BitmapCanvas(bounds, RenderStrategy());
    final List<TextDecorationStyle> decorationStyles = <TextDecorationStyle>[
      TextDecorationStyle.solid,
      TextDecorationStyle.double,
      TextDecorationStyle.dotted,
      TextDecorationStyle.dashed,
      TextDecorationStyle.wavy,
    ];

    final CanvasParagraph paragraph = rich(EngineParagraphStyle(fontFamily: 'Roboto'), (
      CanvasParagraphBuilder builder,
    ) {
      for (final TextDecorationStyle decorationStyle in decorationStyles) {
        builder.pushStyle(
          EngineTextStyle.only(
            color: const Color.fromRGBO(50, 50, 255, 1.0),
            decoration: TextDecoration.underline,
            decorationStyle: decorationStyle,
            decorationColor: red,
            fontFamily: 'Roboto',
            fontSize: 30,
          ),
        );
        builder.addText('Hello World');
        builder.pop();
        builder.addText(' ');
      }
    })..layout(constrain(double.infinity));

    canvas.drawParagraph(paragraph, Offset.zero);
    return takeScreenshot(canvas, bounds, 'canvas_paragraph_decoration');
  });

  void testFontFeatures(EngineCanvas canvas) {
    const String text = 'Bb Difficult ';
    const FontFeature enableSmallCaps = FontFeature('smcp');
    const FontFeature disableSmallCaps = FontFeature('smcp', 0);

    const String numeric = '123.4560';
    const FontFeature enableOnum = FontFeature('onum');

    const FontFeature disableLigatures = FontFeature('liga', 0);

    final CanvasParagraph paragraph = rich(EngineParagraphStyle(fontFamily: 'Roboto'), (
      CanvasParagraphBuilder builder,
    ) {
      // Small Caps
      builder.pushStyle(EngineTextStyle.only(height: 1.5, color: black, fontSize: 32.0));
      builder.pushStyle(
        EngineTextStyle.only(color: blue, fontFeatures: <FontFeature>[enableSmallCaps]),
      );
      builder.addText(text);
      // Make sure disabling a font feature also works.
      builder.pushStyle(
        EngineTextStyle.only(color: black, fontFeatures: <FontFeature>[disableSmallCaps]),
      );
      builder.addText(' (smcp)\n');
      builder.pop(); // disableSmallCaps
      builder.pop(); // enableSmallCaps

      // No ligatures
      builder.pushStyle(
        EngineTextStyle.only(color: blue, fontFeatures: <FontFeature>[disableLigatures]),
      );
      builder.addText(text);
      builder.pop(); // disableLigatures
      builder.addText(' (no liga)\n');

      // No font features
      builder.pushStyle(EngineTextStyle.only(color: blue));
      builder.addText(text);
      builder.pop(); // color: blue
      builder.addText(' (none)\n');

      // Onum
      builder.pushStyle(EngineTextStyle.only(color: blue, fontFeatures: <FontFeature>[enableOnum]));
      builder.addText(numeric);
      builder.pop(); // enableOnum
      builder.addText(' (onum)\n');

      // No font features
      builder.pushStyle(EngineTextStyle.only(color: blue));
      builder.addText(numeric);
      builder.pop(); // color: blue
      builder.addText(' (none)\n\n');

      // Multiple font features
      builder.addText('Combined (smcp, onum):\n');
      builder.pushStyle(
        EngineTextStyle.only(color: blue, fontFeatures: <FontFeature>[enableSmallCaps, enableOnum]),
      );
      builder.addText('$text  $numeric');
      builder.pop(); // enableSmallCaps, enableOnum
    })..layout(constrain(double.infinity));
    canvas.drawParagraph(paragraph, Offset.zero);
  }

  test('font features', () {
    const Rect bounds = Rect.fromLTWH(0, 0, 600, 500);
    final BitmapCanvas canvas = BitmapCanvas(bounds, RenderStrategy());
    testFontFeatures(canvas);
    return takeScreenshot(canvas, bounds, 'canvas_paragraph_font_features');
  });

  test('font features (DOM)', () {
    const Rect bounds = Rect.fromLTWH(0, 0, 600, 500);
    final DomCanvas canvas = DomCanvas(domDocument.createElement('flt-picture'));
    testFontFeatures(canvas);
    return takeScreenshot(canvas, bounds, 'canvas_paragraph_font_features_dom');
  });

  void testFontVariations(EngineCanvas canvas) {
    const String text = 'ABCDE 12345\n';
    FontVariation weight(double w) => FontVariation('wght', w);

    final CanvasParagraph paragraph = rich(EngineParagraphStyle(fontFamily: 'RobotoVariable'), (
      CanvasParagraphBuilder builder,
    ) {
      builder.pushStyle(EngineTextStyle.only(fontSize: 48.0));
      builder.addText(text);
      builder.pushStyle(
        EngineTextStyle.only(fontSize: 48.0, fontVariations: <FontVariation>[weight(900)]),
      );
      builder.addText(text);
      builder.pushStyle(
        EngineTextStyle.only(fontSize: 48.0, fontVariations: <FontVariation>[weight(200)]),
      );
      builder.addText(text);
      builder.pop();
      builder.pop();
      builder.pop();
    })..layout(constrain(double.infinity));
    canvas.drawParagraph(paragraph, Offset.zero);
  }

  test('font variations', () {
    const Rect bounds = Rect.fromLTWH(0, 0, 600, 500);
    final BitmapCanvas canvas = BitmapCanvas(bounds, RenderStrategy());
    testFontVariations(canvas);
    return takeScreenshot(canvas, bounds, 'canvas_paragraph_font_variations');
  });

  test('font variations (DOM)', () {
    const Rect bounds = Rect.fromLTWH(0, 0, 600, 500);
    final DomCanvas canvas = DomCanvas(domDocument.createElement('flt-picture'));
    testFontVariations(canvas);
    return takeScreenshot(canvas, bounds, 'canvas_paragraph_font_variations_dom');
  });

  void testBackgroundStyle(EngineCanvas canvas) {
    final CanvasParagraph paragraph = rich(
      EngineParagraphStyle(fontFamily: 'Roboto', fontSize: 40.0),
      (CanvasParagraphBuilder builder) {
        builder.pushStyle(EngineTextStyle.only(color: black));
        builder.pushStyle(EngineTextStyle.only(background: Paint()..color = blue));
        builder.addText('Lor');
        builder.pushStyle(EngineTextStyle.only(background: Paint()..color = black, color: white));
        builder.addText('em ');
        builder.pop();
        builder.pushStyle(EngineTextStyle.only(background: Paint()..color = green));
        builder.addText('ipsu');
        builder.pushStyle(EngineTextStyle.only(background: Paint()..color = yellow));
        builder.addText('m\ndo');
        builder.pushStyle(EngineTextStyle.only(background: Paint()..color = red));
        builder.addText('lor sit');
      },
    );
    paragraph.layout(constrain(double.infinity));
    canvas.drawParagraph(paragraph, Offset.zero);
  }

  test('background style', () {
    const Rect bounds = Rect.fromLTWH(0, 0, 300, 200);
    final BitmapCanvas canvas = BitmapCanvas(bounds, RenderStrategy());
    testBackgroundStyle(canvas);
    return takeScreenshot(canvas, bounds, 'canvas_paragraph_background_style');
  });

  test('background style (DOM)', () {
    const Rect bounds = Rect.fromLTWH(0, 0, 300, 200);
    final DomCanvas canvas = DomCanvas(domDocument.createElement('flt-picture'));
    testBackgroundStyle(canvas);
    return takeScreenshot(canvas, bounds, 'canvas_paragraph_background_style_dom');
  });

  void testForegroundStyle(EngineCanvas canvas) {
    final CanvasParagraph paragraph = rich(
      EngineParagraphStyle(fontFamily: 'Roboto', fontSize: 40.0),
      (CanvasParagraphBuilder builder) {
        builder.pushStyle(EngineTextStyle.only(color: blue));
        builder.addText('Lorem');
        builder.pop();
        builder.pushStyle(
          EngineTextStyle.only(
            foreground:
                Paint()
                  ..color = red
                  ..style = PaintingStyle.stroke,
          ),
        );
        builder.addText('ipsum\n');
        builder.pop();
        builder.pushStyle(
          EngineTextStyle.only(
            foreground:
                Paint()
                  ..color = blue
                  ..style = PaintingStyle.stroke
                  ..strokeWidth = 0.0,
          ),
        );
        builder.addText('dolor');
        builder.pop();
        builder.pushStyle(
          EngineTextStyle.only(
            foreground:
                Paint()
                  ..color = green
                  ..style = PaintingStyle.stroke
                  ..strokeWidth = 2.0,
          ),
        );
        builder.addText('sit\n');
        builder.pop();
        builder.pushStyle(
          EngineTextStyle.only(
            foreground:
                Paint()
                  ..color = yellow
                  ..style = PaintingStyle.stroke
                  ..strokeWidth = 4.0,
          ),
        );
        builder.addText('amet');
      },
    );
    paragraph.layout(constrain(double.infinity));
    canvas.drawParagraph(paragraph, Offset.zero);
  }

  test('foreground style', () {
    const Rect bounds = Rect.fromLTWH(0, 0, 300, 200);
    final BitmapCanvas canvas = BitmapCanvas(bounds, RenderStrategy());
    testForegroundStyle(canvas);
    return takeScreenshot(canvas, bounds, 'canvas_paragraph_foreground_style');
  });

  test('foreground style (DOM)', () {
    const Rect bounds = Rect.fromLTWH(0, 0, 300, 200);
    final DomCanvas canvas = DomCanvas(domDocument.createElement('flt-picture'));
    testForegroundStyle(canvas);
    return takeScreenshot(canvas, bounds, 'canvas_paragraph_foreground_style_dom');
  });

  test('paragraph bounds hug the text inside the paragraph', () async {
    const Rect bounds = Rect.fromLTWH(0, 0, 150, 100);

    final CanvasParagraphBuilder builder = CanvasParagraphBuilder(
      EngineParagraphStyle(fontFamily: 'Ahem', fontSize: 20, textAlign: TextAlign.center),
    );

    // Expected layout with center-alignment is something like this:
    //
    // _________________
    // |       A       |
    // |    |AAAAA|    |
    // |    | AAA |    |
    // |----|-----|----|
    // |    |<--->|    |
    // |      100      |
    // |               |
    // |<------------->|
    //        110
    //
    // The width of the paragraph is bigger than the actual content because the
    // longest line "AAAAA" is 100px, which is smaller than 110px specified in
    // the constraint. After the layout and centering the paint bounds would
    // "hug" the text inside the paragraph more tightly than the box allocated
    // for the paragraph.
    builder.addText('A AAAAA AAA');

    final CanvasParagraph paragraph = builder.build();
    paragraph.layout(const ParagraphConstraints(width: 110));
    final BitmapCanvas canvas = BitmapCanvas(bounds, RenderStrategy());
    canvas.translate(20, 20);
    canvas.drawParagraph(paragraph, Offset.zero);
    canvas.drawRect(
      Rect.fromLTRB(0, 0, paragraph.width, paragraph.height),
      SurfacePaintData()
        ..style = PaintingStyle.stroke
        ..strokeWidth = 1,
    );

    canvas.drawRect(
      paragraph.paintBounds,
      SurfacePaintData()
        ..color = 0xFF00FF00
        ..style = PaintingStyle.stroke
        ..strokeWidth = 1,
    );

    await takeScreenshot(canvas, bounds, 'canvas_paragraph_bounds');
  });
}
