// Copyright 2013 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

import 'package:test/bootstrap/browser.dart';
import 'package:test/test.dart';
import 'package:ui/src/engine.dart';
import 'package:ui/ui.dart' hide TextStyle;

import '../../common/test_initialization.dart';
import '../screenshot.dart';
import '../testimage.dart';

void main() {
  internalBootstrapBrowserTest(() => testMain);
}

SurfacePaint makePaint() => Paint() as SurfacePaint;

Future<void> testMain() async {
  setUpUnitTests(withImplicitView: true, setUpTestViewDimensions: false);

  const Color red = Color(0xFFFF0000);
  const Color green = Color(0xFF00FF00);
  const Color blue = Color(0xFF2196F3);
  const Color white = Color(0xFFFFFFFF);
  const Color grey = Color(0xFF808080);
  const Color black = Color(0xFF000000);

  final List<List<BlendMode>> modes = <List<BlendMode>>[
    <BlendMode>[
      BlendMode.clear,
      BlendMode.src,
      BlendMode.dst,
      BlendMode.srcOver,
      BlendMode.dstOver,
      BlendMode.srcIn,
      BlendMode.dstIn,
      BlendMode.srcOut,
    ],
    <BlendMode>[
      BlendMode.dstOut,
      BlendMode.srcATop,
      BlendMode.dstATop,
      BlendMode.xor,
      BlendMode.plus,
      BlendMode.modulate,
      BlendMode.screen,
      BlendMode.overlay,
    ],
    <BlendMode>[
      BlendMode.darken,
      BlendMode.lighten,
      BlendMode.colorDodge,
      BlendMode.hardLight,
      BlendMode.softLight,
      BlendMode.difference,
      BlendMode.exclusion,
      BlendMode.multiply,
    ],
    <BlendMode>[BlendMode.hue, BlendMode.saturation, BlendMode.color, BlendMode.luminosity],
  ];

  for (int blendGroup = 0; blendGroup < 4; ++blendGroup) {
    test('Draw image with Group$blendGroup blend modes', () async {
      final RecordingCanvas rc = RecordingCanvas(const Rect.fromLTRB(0, 0, 400, 400));
      rc.save();
      final List<BlendMode> blendModes = modes[blendGroup];
      for (int row = 0; row < blendModes.length; row++) {
        // draw white background for first 4, black for next 4 blends.
        final double top = row * 50.0;
        rc.drawRect(Rect.fromLTWH(0, top, 200, 50), makePaint()..color = white);
        rc.drawRect(Rect.fromLTWH(200, top, 200, 50), makePaint()..color = grey);
        final BlendMode blendMode = blendModes[row];
        rc.drawImage(
          createFlutterLogoTestImage(),
          Offset(0, top),
          makePaint()..colorFilter = EngineColorFilter.mode(red, blendMode),
        );
        rc.drawImage(
          createFlutterLogoTestImage(),
          Offset(50, top),
          makePaint()..colorFilter = EngineColorFilter.mode(green, blendMode),
        );
        rc.drawImage(
          createFlutterLogoTestImage(),
          Offset(100, top),
          makePaint()..colorFilter = EngineColorFilter.mode(blue, blendMode),
        );
        rc.drawImage(
          createFlutterLogoTestImage(),
          Offset(150, top),
          makePaint()..colorFilter = EngineColorFilter.mode(black, blendMode),
        );
        rc.drawImage(
          createFlutterLogoTestImage(),
          Offset(200, top),
          makePaint()..colorFilter = EngineColorFilter.mode(red, blendMode),
        );
        rc.drawImage(
          createFlutterLogoTestImage(),
          Offset(250, top),
          makePaint()..colorFilter = EngineColorFilter.mode(green, blendMode),
        );
        rc.drawImage(
          createFlutterLogoTestImage(),
          Offset(300, top),
          makePaint()..colorFilter = EngineColorFilter.mode(blue, blendMode),
        );
        rc.drawImage(
          createFlutterLogoTestImage(),
          Offset(350, top),
          makePaint()..colorFilter = EngineColorFilter.mode(black, blendMode),
        );
      }
      rc.restore();
      await canvasScreenshot(rc, 'canvas_image_blend_group$blendGroup');
    }, skip: isSafari);
  }

  // Regression test for https://github.com/flutter/flutter/issues/56971
  test('Draws image and paragraph at same vertical position', () async {
    final RecordingCanvas rc = RecordingCanvas(const Rect.fromLTRB(0, 0, 400, 400));
    rc.save();
    rc.drawRect(const Rect.fromLTWH(0, 50, 200, 50), makePaint()..color = white);
    rc.drawImage(
      createFlutterLogoTestImage(),
      const Offset(0, 50),
      makePaint()..colorFilter = const EngineColorFilter.mode(red, BlendMode.srcIn),
    );

    final Paragraph paragraph = createTestParagraph();
    const double textLeft = 80.0;
    const double textTop = 50.0;
    const double widthConstraint = 300.0;
    paragraph.layout(const ParagraphConstraints(width: widthConstraint));
    rc.drawParagraph(paragraph, const Offset(textLeft, textTop));

    rc.restore();
    await canvasScreenshot(rc, 'canvas_image_blend_and_text');
  });

  test('Does not re-use styles with same image src', () async {
    final RecordingCanvas rc = RecordingCanvas(const Rect.fromLTRB(0, 0, 400, 400));
    final HtmlImage flutterImage = createFlutterLogoTestImage();
    rc.save();
    rc.drawRect(const Rect.fromLTWH(0, 50, 200, 50), makePaint()..color = white);
    rc.drawImage(
      flutterImage,
      const Offset(0, 50),
      makePaint()..colorFilter = const EngineColorFilter.mode(red, BlendMode.modulate),
    );

    // Expect that the colorFilter is only applied to the first image, since the
    // colorFilter is applied to a clone of the flutterImage and not the original
    rc.drawImage(flutterImage, const Offset(0, 100), makePaint());

    rc.restore();
    await canvasScreenshot(rc, 'canvas_image_same_src');
  });
}

Paragraph createTestParagraph() {
  final ParagraphBuilder builder = ParagraphBuilder(
    ParagraphStyle(
      fontFamily: 'Ahem',
      fontStyle: FontStyle.normal,
      fontWeight: FontWeight.normal,
      fontSize: 14.0,
    ),
  );
  builder.addText('FOO');
  return builder.build();
}
