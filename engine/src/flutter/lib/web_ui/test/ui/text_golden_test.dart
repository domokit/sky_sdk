// Copyright 2013 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

import 'dart:math' as math;

import 'package:test/bootstrap/browser.dart';
import 'package:test/test.dart';
import 'package:ui/src/engine.dart';
import 'package:ui/ui.dart' as ui;
import 'package:web_engine_tester/golden_tester.dart';

import '../common/test_initialization.dart';
import 'utils.dart';

void main() {
  internalBootstrapBrowserTest(() => testMain);
}

Future<void> testMain() async {
  setUpUnitTests(
    withImplicitView: true,
    emulateTesterEnvironment: false,
    setUpTestViewDimensions: false,
  );

  test('text styles - default', () async {
    await testTextStyle('default');
  });

  test('text styles - center aligned', () async {
    await testTextStyle('center aligned', paragraphTextAlign: ui.TextAlign.center);
  });

  test('text styles - right aligned', () async {
    await testTextStyle('right aligned', paragraphTextAlign: ui.TextAlign.right);
  });

  test('text styles - rtl', () async {
    await testTextStyle('rtl', paragraphTextDirection: ui.TextDirection.rtl);
  });

  test('text styles - multiline', () async {
    await testTextStyle('multiline', layoutWidth: 50);
  });

  test('text styles - max lines', () async {
    await testTextStyle('max lines', paragraphMaxLines: 1, layoutWidth: 50);
  });

  test('text styles - ellipsis', () async {
    await testTextStyle(
      'ellipsis',
      paragraphMaxLines: 1,
      paragraphEllipsis: '...',
      layoutWidth: 60,
    );
  });

  test('text styles - paragraph font family', () async {
    await testTextStyle('paragraph font family', paragraphFontFamily: 'Ahem');
  });

  test('text styles - paragraph font size', () async {
    await testTextStyle('paragraph font size', paragraphFontSize: 22);
  });

  test('text styles - paragraph height', () async {
    await testTextStyle('paragraph height', layoutWidth: 50, paragraphHeight: 1.5);
  });

  test('text styles - text style height overriding paragraph height', () async {
    await testTextStyle(
      'text style height and paragraph style height',
      layoutWidth: 50,
      paragraphHeight: 1.5,
      height: 2.0,
    );
  });

  test('text styles - paragraph text height behavior', () async {
    await testTextStyle(
      'paragraph text height behavior',
      layoutWidth: 50,
      paragraphHeight: 1.5,
      paragraphTextHeightBehavior: const ui.TextHeightBehavior(
        applyHeightToFirstAscent: false,
        applyHeightToLastDescent: false,
      ),
    );
  });

  test('text styles - paragraph weight', () async {
    await testTextStyle('paragraph weight', paragraphFontWeight: ui.FontWeight.w900);
  });

  test('text style - paragraph font style', () async {
    await testTextStyle('paragraph font style', paragraphFontStyle: ui.FontStyle.italic);
  });

  // TODO(yjbanov): locales specified in paragraph styles don't work:
  //                https://github.com/flutter/flutter/issues/74687
  // TODO(yjbanov): spaces are not rendered correctly:
  //                https://github.com/flutter/flutter/issues/74742
  test('text styles - paragraph locale zh_CN', () async {
    await testTextStyle(
      'paragraph locale zh_CN',
      outerText: '次 化 刃 直 入 令',
      innerText: '',
      paragraphLocale: const ui.Locale('zh', 'CN'),
    );
  });

  test('text styles - paragraph locale zh_TW', () async {
    await testTextStyle(
      'paragraph locale zh_TW',
      outerText: '次 化 刃 直 入 令',
      innerText: '',
      paragraphLocale: const ui.Locale('zh', 'TW'),
    );
  });

  test('text styles - paragraph locale ja', () async {
    await testTextStyle(
      'paragraph locale ja',
      outerText: '次 化 刃 直 入 令',
      innerText: '',
      paragraphLocale: const ui.Locale('ja'),
    );
  });

  test('text styles - paragraph locale ko', () async {
    await testTextStyle(
      'paragraph locale ko',
      outerText: '次 化 刃 直 入 令',
      innerText: '',
      paragraphLocale: const ui.Locale('ko'),
    );
  });

  test('text styles - color', () async {
    await testTextStyle('color', color: const ui.Color(0xFF009900));
  });

  test('text styles - decoration', () async {
    await testTextStyle('decoration', decoration: ui.TextDecoration.underline);
  });

  test('text styles - decoration style', () async {
    await testTextStyle(
      'decoration style',
      decoration: ui.TextDecoration.underline,
      decorationStyle: ui.TextDecorationStyle.dashed,
    );
  });

  test('text styles - decoration thickness', () async {
    await testTextStyle(
      'decoration thickness',
      decoration: ui.TextDecoration.underline,
      decorationThickness: 5.0,
    );
  });

  test('text styles - font weight', () async {
    await testTextStyle('font weight', fontWeight: ui.FontWeight.w900);
  });

  test('text styles - font style', () async {
    await testTextStyle('font style', fontStyle: ui.FontStyle.italic);
  });

  // TODO(yjbanov): not sure how to test this.
  test('text styles - baseline', () async {
    await testTextStyle('baseline', textBaseline: ui.TextBaseline.ideographic);
  });

  test('text styles - font family', () async {
    await testTextStyle('font family', fontFamily: 'Ahem');
  });

  test('text styles - non-existent font family', () async {
    await testTextStyle('non-existent font family', fontFamily: 'DoesNotExist');
  });

  test('text styles - family fallback', () async {
    await testTextStyle(
      'family fallback',
      fontFamily: 'DoesNotExist',
      fontFamilyFallback: <String>['Ahem'],
    );
  });

  test('text styles - font size', () async {
    await testTextStyle('font size', fontSize: 24);
  });

  // A regression test for the special case when CanvasKit would default to
  // a positive font size when Flutter specifies zero.
  //
  // See: https://github.com/flutter/flutter/issues/98248
  test('text styles - zero font size', () async {
    // This only sets the inner text style, but not the paragraph style, so
    // "Hello" should be visible, but "World!" should disappear.
    await testTextStyle('zero font size', fontSize: 0);

    // This sets the paragraph font size to zero, but the inner text gets
    // an explicit non-zero size that should override paragraph properties,
    // so this time "Hello" should disappear, but "World!" should still be
    // visible.
    await testTextStyle('zero paragraph font size', paragraphFontSize: 0, fontSize: 14);
  });

  test('text styles - letter spacing', () async {
    await testTextStyle('letter spacing', letterSpacing: 5);
  });

  test('text styles - word spacing', () async {
    await testTextStyle('word spacing', innerText: 'Beautiful World!', wordSpacing: 25);
  });

  test('text styles - height', () async {
    await testTextStyle('height', height: 2);
  });

  test('text styles - leading distribution', () async {
    await testTextStyle(
      'half leading',
      height: 20,
      fontSize: 10,
      leadingDistribution: ui.TextLeadingDistribution.even,
    );
    await testTextStyle(
      'half leading inherited from paragraph',
      height: 20,
      fontSize: 10,
      paragraphTextHeightBehavior: const ui.TextHeightBehavior(
        leadingDistribution: ui.TextLeadingDistribution.even,
      ),
    );
    await testTextStyle(
      'text style half leading overrides paragraph style half leading',
      height: 20,
      fontSize: 10,
      leadingDistribution: ui.TextLeadingDistribution.proportional,
      paragraphTextHeightBehavior: const ui.TextHeightBehavior(
        leadingDistribution: ui.TextLeadingDistribution.even,
      ),
    );
  });

  // TODO(yjbanov): locales specified in text styles don't work:
  //                https://github.com/flutter/flutter/issues/74687
  // TODO(yjbanov): spaces are not rendered correctly:
  //                https://github.com/flutter/flutter/issues/74742
  test('text styles - locale zh_CN', () async {
    await testTextStyle(
      'locale zh_CN',
      innerText: '次 化 刃 直 入 令',
      outerText: '',
      locale: const ui.Locale('zh', 'CN'),
    );
  });

  test('text styles - locale zh_TW', () async {
    await testTextStyle(
      'locale zh_TW',
      innerText: '次 化 刃 直 入 令',
      outerText: '',
      locale: const ui.Locale('zh', 'TW'),
    );
  });

  test('text styles - locale ja', () async {
    await testTextStyle(
      'locale ja',
      innerText: '次 化 刃 直 入 令',
      outerText: '',
      locale: const ui.Locale('ja'),
    );
  });

  test('text styles - locale ko', () async {
    await testTextStyle(
      'locale ko',
      innerText: '次 化 刃 直 入 令',
      outerText: '',
      locale: const ui.Locale('ko'),
    );
  });

  test('text styles - background', () async {
    await testTextStyle('background', background: ui.Paint()..color = const ui.Color(0xFF00FF00));
  });

  test('text styles - foreground', () async {
    await testTextStyle('foreground', foreground: ui.Paint()..color = const ui.Color(0xFF0000FF));
  });

  test('text styles - foreground and background', () async {
    await testTextStyle(
      'foreground and background',
      foreground: ui.Paint()..color = const ui.Color(0xFFFF5555),
      background: ui.Paint()..color = const ui.Color(0xFF007700),
    );
  });

  test('text styles - background and color', () async {
    await testTextStyle(
      'background and color',
      color: const ui.Color(0xFFFFFF00),
      background: ui.Paint()..color = const ui.Color(0xFF007700),
    );
  });

  test('text styles - shadows', () async {
    await testTextStyle(
      'shadows',
      shadows: <ui.Shadow>[
        const ui.Shadow(color: ui.Color(0xFF999900), offset: ui.Offset(10, 10), blurRadius: 5),
        const ui.Shadow(color: ui.Color(0xFF009999), offset: ui.Offset(-10, -10), blurRadius: 10),
      ],
    );
  });

  test('text styles - old style figures', () async {
    await testTextStyle(
      'old style figures',
      paragraphFontFamily: 'Roboto',
      paragraphFontSize: 24,
      outerText: '0 1 2 3 4 5 ',
      innerText: '0 1 2 3 4 5',
      fontFeatures: <ui.FontFeature>[const ui.FontFeature.oldstyleFigures()],
    );
  });

  test('text styles - stylistic set 1', () async {
    await testTextStyle(
      'stylistic set 1',
      paragraphFontFamily: 'Roboto',
      paragraphFontSize: 24,
      outerText: 'g',
      innerText: 'g',
      fontFeatures: <ui.FontFeature>[ui.FontFeature.stylisticSet(1)],
    );
  });

  test('text styles - stylistic set 2', () async {
    await testTextStyle(
      'stylistic set 2',
      paragraphFontFamily: 'Roboto',
      paragraphFontSize: 24,
      outerText: 'α',
      innerText: 'α',
      fontFeatures: <ui.FontFeature>[ui.FontFeature.stylisticSet(2)],
    );
  });

  test('text styles - override font family', () async {
    await testTextStyle('override font family', paragraphFontFamily: 'Ahem', fontFamily: 'Roboto');
  });

  test('text styles - override font size', () async {
    await testTextStyle('override font size', paragraphFontSize: 36, fontSize: 18);
  });

  test('text style - override font weight', () async {
    await testTextStyle(
      'override font weight',
      paragraphFontWeight: ui.FontWeight.w900,
      fontWeight: ui.FontWeight.normal,
    );
  });

  test('text style - override font style', () async {
    await testTextStyle(
      'override font style',
      paragraphFontStyle: ui.FontStyle.italic,
      fontStyle: ui.FontStyle.normal,
    );
  });

  test('text style - characters from multiple fallback fonts', () async {
    await testTextStyle(
      'multi-font characters',
      // This character is claimed by multiple fonts. This test makes sure
      // we can find a font supporting it.
      outerText: '欢',
      innerText: '',
    );
  });

  test('text style - symbols', () async {
    // One of the CJK fonts loaded in one of the tests above also contains
    // some of these symbols. To make sure the test produces predictable
    // results we reset the fallback data forcing the engine to reload
    // fallbacks, which for this test will only load Noto Symbols.
    await testTextStyle('symbols', outerText: '← ↑ → ↓ ', innerText: '');
  });

  test('strut style - override height', () async {
    await testTextStyle(
      'strut style',
      paragraphStrutStyle: ui.StrutStyle(forceStrutHeight: true, height: 2),
    );
  });

  test('sample Chinese text', () async {
    await testSampleText(
      'chinese',
      '也称乱数假文或者哑元文本， '
          '是印刷及排版领域所常用的虚拟文字。'
          '由于曾经一台匿名的打印机刻意打乱了'
          '一盒印刷字体从而造出一本字体样品书',
    );
  });

  test('sample Armenian text', () async {
    await testSampleText(
      'armenian',
      'տպագրության և տպագրական արդյունաբերության համար նախատեսված մոդելային տեքստ է',
    );
  });

  test('sample Albanian text', () async {
    await testSampleText(
      'albanian',
      'është një tekst shabllon i industrisë së printimit dhe shtypshkronjave Lorem Ipsum ka qenë teksti shabllon',
    );
  });

  test('sample Arabic text', () async {
    await testSampleText(
      'arabic',
      'هناك حقيقة مثبتة منذ زمن طويل وهي أن المحتوى المقروء لصفحة ما سيلهي',
      textDirection: ui.TextDirection.rtl,
    );
  });

  test('sample Bulgarian text', () async {
    await testSampleText(
      'bulgarian',
      'е елементарен примерен текст използван в печатарската и типографската индустрия',
    );
  });

  test('sample Catalan text', () async {
    await testSampleText(
      'catalan',
      'és un text de farciment usat per la indústria de la tipografia i la impremta',
    );
  });

  test('sample English text', () async {
    await testSampleText(
      'english',
      'Lorem Ipsum is simply dummy text of the printing and typesetting industry',
    );
  });

  test('sample Greek text', () async {
    await testSampleText(
      'greek',
      'είναι απλά ένα κείμενο χωρίς νόημα για τους επαγγελματίες της τυπογραφίας και στοιχειοθεσίας',
    );
  });

  test('sample Hebrew text', () async {
    await testSampleText(
      'hebrew',
      'זוהי עובדה מבוססת שדעתו של הקורא תהיה מוסחת על ידי טקטס קריא כאשר הוא יביט בפריסתו',
      textDirection: ui.TextDirection.rtl,
    );
  });

  test('sample Hindi text', () async {
    await testSampleText(
      'hindi',
      'छपाई और अक्षर योजन उद्योग का एक साधारण डमी पाठ है सन १५०० के बाद से अभी तक इस उद्योग का मानक डमी पाठ मन गया जब एक अज्ञात मुद्रक ने नमूना लेकर एक नमूना किताब बनाई',
    );
  });

  test('sample Thai text', () async {
    await testSampleText(
      'thai',
      'คือ เนื้อหาจำลองแบบเรียบๆ ที่ใช้กันในธุรกิจงานพิมพ์หรืองานเรียงพิมพ์ มันได้กลายมาเป็นเนื้อหาจำลองมาตรฐานของธุรกิจดังกล่าวมาตั้งแต่ศตวรรษที่',
    );
  });

  test('sample Georgian text', () async {
    await testSampleText(
      'georgian',
      'საბეჭდი და ტიპოგრაფიული ინდუსტრიის უშინაარსო ტექსტია. იგი სტანდარტად',
    );
  });

  test('sample Bengali text', () async {
    await testSampleText('bengali', 'ঈদের জামাত মসজিদে, মানতে হবে স্বাস্থ্যবিধি: ধর্ম মন্ত্রণালয়');
  });

  test('hindi svayan test', () async {
    await testSampleText('hindi_svayan', 'स्वयं');
  });

  // We've seen text break when we load many fonts simultaneously. This test
  // combines text in multiple languages into one long paragraph to make sure
  // we can handle it.
  test('sample multilingual text', () async {
    await testSampleText(
      'multilingual',
      '也称乱数假文或者哑元文本， 是印刷及排版领域所常用的虚拟文字。 '
          'տպագրության և տպագրական արդյունաբերության համար '
          'është një tekst shabllon i industrisë së printimit '
          ' زمن طويل وهي أن المحتوى المقروء لصفحة ما سيلهي '
          'е елементарен примерен текст използван в печатарската '
          'és un text de farciment usat per la indústria de la '
          'Lorem Ipsum is simply dummy text of the printing '
          'είναι απλά ένα κείμενο χωρίς νόημα για τους επαγγελματίες '
          ' זוהי עובדה מבוססת שדעתו של הקורא תהיה מוסחת על ידי טקטס קריא '
          'छपाई और अक्षर योजन उद्योग का एक साधारण डमी पाठ है सन '
          'คือ เนื้อหาจำลองแบบเรียบๆ ที่ใช้กันในธุรกิจงานพิมพ์หรืองานเรียงพิมพ์ '
          'საბეჭდი და ტიპოგრაფიული ინდუსტრიის უშინაარსო ტექსტია ',
    );
  });

  test('emoji text with skin tone', () async {
    await testSampleText('emoji_with_skin_tone', '👋🏿 👋🏾 👋🏽 👋🏼 👋🏻');
  }, timeout: const Timeout.factor(2));

  test('font variations are correctly rendered', () async {
    const double testWidth = 300;
    final ui.PictureRecorder recorder = ui.PictureRecorder();
    final ui.Canvas canvas = ui.Canvas(recorder);
    final ui.ParagraphBuilder builder = ui.ParagraphBuilder(
      ui.ParagraphStyle(fontSize: 40.0, textDirection: ui.TextDirection.ltr),
    );

    builder.pushStyle(ui.TextStyle(fontFamily: 'RobotoVariable'));
    builder.addText('Normal\n');
    builder.pop();

    ui.FontVariation weight(double w) => ui.FontVariation('wght', w);
    builder.pushStyle(
      ui.TextStyle(fontFamily: 'RobotoVariable', fontVariations: <ui.FontVariation>[weight(900)]),
    );
    builder.addText('Heavy\n');
    builder.pop();

    builder.pushStyle(
      ui.TextStyle(fontFamily: 'RobotoVariable', fontVariations: <ui.FontVariation>[weight(100)]),
    );
    builder.addText('Light\n');
    builder.pop();

    final ui.Paragraph paragraph = builder.build();
    paragraph.layout(const ui.ParagraphConstraints(width: testWidth - 20));
    canvas.drawParagraph(paragraph, const ui.Offset(10, 10));
    final ui.Picture picture = recorder.endRecording();
    await drawPictureUsingCurrentRenderer(picture);
    await matchGoldenFile(
      'ui_text_font_variation.png',
      region: ui.Rect.fromLTRB(0, 0, testWidth, paragraph.height + 20),
    );
  });

  test('text style - woff2 font', () async {
    await testTextStyle('emoji woff2', outerText: '🙂 🇺🇸 🙋‍♂️', innerText: '', fontSize: 24);
  });
}

/// A convenience function for testing paragraph and text styles.
///
/// Renders a paragraph with two pieces of text, [outerText] and [innerText].
/// [outerText] is added to the root of the paragraph where only paragraph
/// style applies. [innerText] is added under a text style with properties
/// set from the arguments to this method. Parameters with prefix "paragraph"
/// are applied to the paragraph style. Others are applied to the text style.
///
/// [name] is the name of the test used as the description on the golden as
/// well as in the golden file name. Avoid special characters. Spaces are OK;
/// they are replaced by "_" in the file name.
///
/// Use [layoutWidth] to customize the width of the paragraph constraints.
Future<void> testTextStyle(
  // Test properties
  String name, {
  double? layoutWidth,
  // Top-level text where only paragraph style applies
  String outerText = 'Hello ',
  // Second-level text where paragraph and text styles both apply.
  String innerText = 'World!',

  // ParagraphStyle properties
  ui.TextAlign? paragraphTextAlign,
  ui.TextDirection? paragraphTextDirection,
  int? paragraphMaxLines,
  String? paragraphFontFamily,
  double? paragraphFontSize,
  double? paragraphHeight,
  ui.TextHeightBehavior? paragraphTextHeightBehavior,
  ui.FontWeight? paragraphFontWeight,
  ui.FontStyle? paragraphFontStyle,
  ui.StrutStyle? paragraphStrutStyle,
  String? paragraphEllipsis,
  ui.Locale? paragraphLocale,

  // TextStyle properties
  ui.Color? color,
  ui.TextDecoration? decoration,
  ui.Color? decorationColor,
  ui.TextDecorationStyle? decorationStyle,
  double? decorationThickness,
  ui.FontWeight? fontWeight,
  ui.FontStyle? fontStyle,
  ui.TextBaseline? textBaseline,
  String? fontFamily,
  List<String>? fontFamilyFallback,
  double? fontSize,
  double? letterSpacing,
  double? wordSpacing,
  double? height,
  ui.TextLeadingDistribution? leadingDistribution,
  ui.Locale? locale,
  ui.Paint? background,
  ui.Paint? foreground,
  List<ui.Shadow>? shadows,
  List<ui.FontFeature>? fontFeatures,
}) async {
  late ui.Rect region;
  ui.Picture renderPicture() {
    const double testWidth = 512;
    final ui.PictureRecorder recorder = ui.PictureRecorder();
    final ui.Canvas canvas = ui.Canvas(recorder);
    canvas.translate(30, 10);
    final ui.ParagraphBuilder descriptionBuilder = ui.ParagraphBuilder(ui.ParagraphStyle());
    descriptionBuilder.addText(name);
    final ui.Paragraph descriptionParagraph = descriptionBuilder.build();
    descriptionParagraph.layout(const ui.ParagraphConstraints(width: testWidth / 2 - 70));
    const ui.Offset descriptionOffset = ui.Offset(testWidth / 2 + 30, 0);
    canvas.drawParagraph(descriptionParagraph, descriptionOffset);

    final ui.ParagraphBuilder pb = ui.ParagraphBuilder(
      ui.ParagraphStyle(
        textAlign: paragraphTextAlign,
        textDirection: paragraphTextDirection,
        maxLines: paragraphMaxLines,
        fontFamily: paragraphFontFamily,
        fontSize: paragraphFontSize,
        height: paragraphHeight,
        textHeightBehavior: paragraphTextHeightBehavior,
        fontWeight: paragraphFontWeight,
        fontStyle: paragraphFontStyle,
        strutStyle: paragraphStrutStyle,
        ellipsis: paragraphEllipsis,
        locale: paragraphLocale,
      ),
    );

    pb.addText(outerText);

    pb.pushStyle(
      ui.TextStyle(
        color: color,
        decoration: decoration,
        decorationColor: decorationColor,
        decorationStyle: decorationStyle,
        decorationThickness: decorationThickness,
        fontWeight: fontWeight,
        fontStyle: fontStyle,
        textBaseline: textBaseline,
        fontFamily: fontFamily,
        fontFamilyFallback: fontFamilyFallback,
        fontSize: fontSize,
        letterSpacing: letterSpacing,
        wordSpacing: wordSpacing,
        height: height,
        leadingDistribution: leadingDistribution,
        locale: locale,
        background: background,
        foreground: foreground,
        shadows: shadows,
        fontFeatures: fontFeatures,
      ),
    );
    pb.addText(innerText);
    pb.pop();
    final ui.Paragraph p = pb.build();
    p.layout(ui.ParagraphConstraints(width: layoutWidth ?? testWidth / 2));
    canvas.drawParagraph(p, ui.Offset.zero);

    canvas.drawPath(
      ui.Path()
        ..moveTo(-10, 0)
        ..lineTo(-20, 0)
        ..lineTo(-20, p.height)
        ..lineTo(-10, p.height),
      ui.Paint()
        ..style = ui.PaintingStyle.stroke
        ..strokeWidth = 1.0,
    );
    canvas.drawPath(
      ui.Path()
        ..moveTo(testWidth / 2 + 10, 0)
        ..lineTo(testWidth / 2 + 20, 0)
        ..lineTo(testWidth / 2 + 20, p.height)
        ..lineTo(testWidth / 2 + 10, p.height),
      ui.Paint()
        ..style = ui.PaintingStyle.stroke
        ..strokeWidth = 1.0,
    );
    const double padding = 20;
    region = ui.Rect.fromLTRB(
      0,
      0,
      testWidth,
      math.max(descriptionOffset.dy + descriptionParagraph.height + padding, p.height + padding),
    );
    return recorder.endRecording();
  }

  // Render once to trigger font downloads.
  renderPicture();
  await renderer.fontCollection.fontFallbackManager?.debugWhenIdle();
  final ui.Picture picture = renderPicture();
  await drawPictureUsingCurrentRenderer(picture);

  await matchGoldenFile('ui_text_styles_${name.replaceAll(' ', '_')}.png', region: region);
}

Future<void> testSampleText(
  String language,
  String text, {
  ui.TextDirection textDirection = ui.TextDirection.ltr,
}) async {
  const double testWidth = 300;
  double paragraphHeight = 0;
  ui.Picture renderPicture() {
    final ui.PictureRecorder recorder = ui.PictureRecorder();
    final ui.Canvas canvas = ui.Canvas(recorder);
    final ui.ParagraphBuilder paragraphBuilder = ui.ParagraphBuilder(
      ui.ParagraphStyle(textDirection: textDirection),
    );
    paragraphBuilder.addText(text);
    final ui.Paragraph paragraph = paragraphBuilder.build();
    paragraph.layout(const ui.ParagraphConstraints(width: testWidth - 20));
    canvas.drawParagraph(paragraph, const ui.Offset(10, 10));
    paragraphHeight = paragraph.height;
    return recorder.endRecording();
  }

  // Render once to trigger font downloads.
  renderPicture();
  await renderer.fontCollection.fontFallbackManager?.debugWhenIdle();
  final ui.Picture picture = renderPicture();
  await drawPictureUsingCurrentRenderer(picture);
  await matchGoldenFile(
    'ui_sample_text_$language.png',
    region: ui.Rect.fromLTRB(0, 0, testWidth, paragraphHeight + 20),
  );
}
