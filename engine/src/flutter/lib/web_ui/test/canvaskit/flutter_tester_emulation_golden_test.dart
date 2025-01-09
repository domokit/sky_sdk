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

const ui.Rect kDefaultRegion = ui.Rect.fromLTRB(0, 0, 500, 250);

void testMain() {
  ui_web.debugEmulateFlutterTesterEnvironment = true;

  group('flutter_tester emulation', () {
    setUpCanvasKitTest(withImplicitView: true);

    test('defaults to FlutterTest font family', () async {
      final CkPictureRecorder recorder = CkPictureRecorder();
      final CkCanvas canvas = recorder.beginRecording(kDefaultRegion);
      canvas.translate(10, 10);

      void drawTextWithOutline(
        String text, {
        String? paragraphFontFamily,
        String? textFontFamily,
        List<String>? textFontFallbacks,
        String? strutStyleFontFamily,
        List<String>? strutStyleFontFallbacks,
        double? strutStyleFontSize,
      }) {
        final CkStrutStyle? strutStyle;

        if (strutStyleFontFamily != null ||
            strutStyleFontFallbacks != null ||
            strutStyleFontSize != null) {
          strutStyle = CkStrutStyle(
            fontFamily: strutStyleFontFamily,
            fontFamilyFallback: strutStyleFontFallbacks,
            fontSize: strutStyleFontSize,
          );
        } else {
          strutStyle = null;
        }

        final CkParagraphBuilder builder = CkParagraphBuilder(
          CkParagraphStyle(fontFamily: paragraphFontFamily, strutStyle: strutStyle),
        );

        final bool needsTextStyle = textFontFamily != null || textFontFallbacks != null;

        if (needsTextStyle) {
          builder.pushStyle(
            CkTextStyle(fontFamily: textFontFamily, fontFamilyFallback: textFontFallbacks),
          );
        }

        builder.addText(text);

        if (needsTextStyle) {
          builder.pop();
        }

        final CkParagraph paragraph = builder.build();
        paragraph.layout(const ui.ParagraphConstraints(width: 10000));
        canvas.drawParagraph(paragraph, ui.Offset.zero);
        canvas.drawRect(
          ui.Rect.fromLTWH(-4, -4, paragraph.maxIntrinsicWidth + 8, paragraph.height + 8),
          CkPaint()
            ..style = ui.PaintingStyle.stroke
            ..strokeWidth = 1,
        );
        canvas.translate(0, paragraph.height + 16);
      }

      drawTextWithOutline('default');
      drawTextWithOutline('roboto paragraph', paragraphFontFamily: 'Roboto');
      drawTextWithOutline('roboto text', textFontFamily: 'Roboto');
      drawTextWithOutline('roboto text fallback', textFontFallbacks: <String>['Roboto']);
      drawTextWithOutline(
        'roboto strut style',
        strutStyleFontFamily: 'Roboto',
        strutStyleFontSize: 40,
      );
      drawTextWithOutline(
        'roboto strut style fallback',
        strutStyleFontFallbacks: <String>['Roboto'],
        strutStyleFontSize: 40,
      );

      await matchPictureGolden(
        'canvaskit_defaults_to_ahem.png',
        recorder.endRecording(),
        region: kDefaultRegion,
      );
    });
    // TODO(yjbanov): https://github.com/flutter/flutter/issues/71520
  }, skip: isSafari || isFirefox);
}
