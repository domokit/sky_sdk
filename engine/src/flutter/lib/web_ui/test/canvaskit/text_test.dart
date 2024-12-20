// Copyright 2013 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

import 'package:test/bootstrap/browser.dart';
import 'package:test/test.dart';
import 'package:ui/src/engine.dart';
import 'package:ui/ui.dart' as ui;
import 'package:ui/ui_web/src/ui_web.dart' as ui_web;

import 'common.dart';

void main() {
  internalBootstrapBrowserTest(() => testMain);
}

void testMain() {
  group('CanvasKit text', () {
    setUpCanvasKitTest();

    test("doesn't crash when using shadows", () {
      final ui.TextStyle textStyleWithShadows = ui.TextStyle(
        fontSize: 16,
        shadows: <ui.Shadow>[
          const ui.Shadow(blurRadius: 3.0, offset: ui.Offset(3.0, 3.0)),
          const ui.Shadow(blurRadius: 3.0, offset: ui.Offset(-3.0, 3.0)),
          const ui.Shadow(blurRadius: 3.0, offset: ui.Offset(3.0, -3.0)),
          const ui.Shadow(blurRadius: 3.0, offset: ui.Offset(-3.0, -3.0)),
        ],
        fontFamily: 'Roboto',
      );

      for (int i = 0; i < 10; i++) {
        final ui.ParagraphBuilder builder = ui.ParagraphBuilder(ui.ParagraphStyle(fontSize: 16));
        builder.pushStyle(textStyleWithShadows);
        builder.addText('test');
        final ui.Paragraph paragraph = builder.build();
        expect(paragraph, isNotNull);
      }
    });

    // Regression test for https://github.com/flutter/flutter/issues/78550
    test('getBoxesForRange works for LTR text in an RTL paragraph', () {
      // Create builder for an RTL paragraph.
      final ui.ParagraphBuilder builder = ui.ParagraphBuilder(
        ui.ParagraphStyle(fontSize: 16, textDirection: ui.TextDirection.rtl),
      );
      builder.addText('hello');
      final ui.Paragraph paragraph = builder.build();
      paragraph.layout(const ui.ParagraphConstraints(width: 100));
      expect(paragraph, isNotNull);
      final List<ui.TextBox> boxes = paragraph.getBoxesForRange(0, 1);
      expect(boxes, hasLength(1));
      // The direction for this span is LTR even though the paragraph is RTL
      // because the directionality of the 'h' is LTR.
      expect(boxes.single.direction, equals(ui.TextDirection.ltr));
    });

    test('Renders tab as space instead of tofu', () async {
      // CanvasKit renders a tofu if the font does not have a glyph for a
      // character. However, Flutter opts-in to a CanvasKit feature to render
      // tabs as a single space.
      // See: https://github.com/flutter/flutter/issues/79153
      Future<ui.Image> drawText(String text) {
        const ui.Rect bounds = ui.Rect.fromLTRB(0, 0, 100, 100);
        final CkPictureRecorder recorder = CkPictureRecorder();
        final CkCanvas canvas = recorder.beginRecording(bounds);
        final CkParagraph paragraph = makeSimpleText(text);

        canvas.drawParagraph(paragraph, ui.Offset.zero);
        final ui.Picture picture = recorder.endRecording();
        return picture.toImage(100, 100);
      }

      // The backspace character, \b, does not have a corresponding glyph and
      // is rendered as a tofu.
      final ui.Image tabImage = await drawText('>\t<');
      final ui.Image spaceImage = await drawText('> <');
      final ui.Image tofuImage = await drawText('>\b<');

      expect(await matchImage(tabImage, spaceImage), isTrue);
      expect(await matchImage(tabImage, tofuImage), isFalse);
    });

    group('test fonts in flutterTester environment', () {
      final bool resetValue = ui_web.debugEmulateFlutterTesterEnvironment;
      ui_web.debugEmulateFlutterTesterEnvironment = true;
      tearDownAll(() {
        ui_web.debugEmulateFlutterTesterEnvironment = resetValue;
      });
      const List<String> testFonts = <String>['FlutterTest', 'Ahem'];

      test('The default test font is used when a non-test fontFamily is specified', () {
        final String defaultTestFontFamily = testFonts.first;

        expect(
          CkTextStyle(fontFamily: 'BogusFontFamily').effectiveFontFamily,
          defaultTestFontFamily,
        );
        expect(
          CkParagraphStyle(fontFamily: 'BogusFontFamily').getTextStyle().effectiveFontFamily,
          defaultTestFontFamily,
        );
        expect(
          CkStrutStyle(fontFamily: 'BogusFontFamily'),
          CkStrutStyle(fontFamily: defaultTestFontFamily),
        );
      });

      test('The default test font is used when fontFamily is unspecified', () {
        final String defaultTestFontFamily = testFonts.first;

        expect(CkTextStyle().effectiveFontFamily, defaultTestFontFamily);
        expect(CkParagraphStyle().getTextStyle().effectiveFontFamily, defaultTestFontFamily);
        expect(CkStrutStyle(), CkStrutStyle(fontFamily: defaultTestFontFamily));
      });

      test('Can specify test fontFamily to use', () {
        for (final String testFont in testFonts) {
          expect(CkTextStyle(fontFamily: testFont).effectiveFontFamily, testFont);
          expect(
            CkParagraphStyle(fontFamily: testFont).getTextStyle().effectiveFontFamily,
            testFont,
          );
        }
      });
    });
    // TODO(hterkelsen): https://github.com/flutter/flutter/issues/71520
  }, skip: isSafari || isFirefox);
}
