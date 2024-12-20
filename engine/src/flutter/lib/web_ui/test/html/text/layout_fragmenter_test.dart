// Copyright 2013 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

import 'package:test/bootstrap/browser.dart';
import 'package:test/test.dart';
import 'package:ui/src/engine.dart';
import 'package:ui/ui.dart';

import '../paragraph/helper.dart';

final EngineTextStyle defaultStyle = EngineTextStyle.only(
  fontFamily: StyleManager.defaultFontFamily,
  fontSize: StyleManager.defaultFontSize,
);
final EngineTextStyle style1 = defaultStyle.copyWith(fontSize: 20);
final EngineTextStyle style2 = defaultStyle.copyWith(color: blue);
final EngineTextStyle style3 = defaultStyle.copyWith(fontFamily: 'Roboto');

void main() {
  internalBootstrapBrowserTest(() => testMain);
}

Future<void> testMain() async {
  group('$LayoutFragmenter', () {
    test('empty paragraph', () {
      final CanvasParagraph paragraph1 = rich(
        EngineParagraphStyle(),
        (CanvasParagraphBuilder builder) {},
      );
      expect(split(paragraph1), <_Fragment>[
        _Fragment('', endOfText, null, ffPrevious, defaultStyle),
      ]);

      final CanvasParagraph paragraph2 = rich(EngineParagraphStyle(), (
        CanvasParagraphBuilder builder,
      ) {
        builder.addText('');
      });
      expect(split(paragraph2), <_Fragment>[
        _Fragment('', endOfText, null, ffPrevious, defaultStyle),
      ]);

      final CanvasParagraph paragraph3 = rich(EngineParagraphStyle(), (
        CanvasParagraphBuilder builder,
      ) {
        builder.pushStyle(style1);
        builder.addText('');
      });
      expect(split(paragraph3), <_Fragment>[_Fragment('', endOfText, null, ffPrevious, style1)]);
    });

    test('single span', () {
      final CanvasParagraph paragraph = plain(
        EngineParagraphStyle(),
        'Lorem 12 $rtlWord1   ipsum34',
      );
      expect(split(paragraph), <_Fragment>[
        _Fragment('Lorem', prohibited, ltr, ffLtr, defaultStyle),
        _Fragment(' ', opportunity, null, ffSandwich, defaultStyle, sp: 1),
        _Fragment('12', prohibited, ltr, ffPrevious, defaultStyle),
        _Fragment(' ', opportunity, null, ffSandwich, defaultStyle, sp: 1),
        _Fragment(rtlWord1, prohibited, rtl, ffRtl, defaultStyle),
        _Fragment('   ', opportunity, null, ffSandwich, defaultStyle, sp: 3),
        _Fragment('ipsum34', endOfText, ltr, ffLtr, defaultStyle),
      ]);
    });

    test('multi span', () {
      final CanvasParagraph paragraph = rich(EngineParagraphStyle(), (
        CanvasParagraphBuilder builder,
      ) {
        builder.pushStyle(style1);
        builder.addText('Lorem');
        builder.pop();
        builder.pushStyle(style2);
        builder.addText(' ipsum 12 ');
        builder.pop();
        builder.pushStyle(style3);
        builder.addText(' $rtlWord1 foo.');
        builder.pop();
      });

      expect(split(paragraph), <_Fragment>[
        _Fragment('Lorem', prohibited, ltr, ffLtr, style1),
        _Fragment(' ', opportunity, null, ffSandwich, style2, sp: 1),
        _Fragment('ipsum', prohibited, ltr, ffLtr, style2),
        _Fragment(' ', opportunity, null, ffSandwich, style2, sp: 1),
        _Fragment('12', prohibited, ltr, ffPrevious, style2),
        _Fragment(' ', prohibited, null, ffSandwich, style2, sp: 1),
        _Fragment(' ', opportunity, null, ffSandwich, style3, sp: 1),
        _Fragment(rtlWord1, prohibited, rtl, ffRtl, style3),
        _Fragment(' ', opportunity, null, ffSandwich, style3, sp: 1),
        _Fragment('foo', prohibited, ltr, ffLtr, style3),
        _Fragment('.', endOfText, null, ffSandwich, style3),
      ]);
    });

    test('new lines', () {
      final CanvasParagraph paragraph = rich(EngineParagraphStyle(), (
        CanvasParagraphBuilder builder,
      ) {
        builder.pushStyle(style1);
        builder.addText('Lor\nem \n');
        builder.pop();
        builder.pushStyle(style2);
        builder.addText(' \n  ipsum 12 ');
        builder.pop();
        builder.pushStyle(style3);
        builder.addText(' $rtlWord1 fo');
        builder.pop();
        builder.pushStyle(style1);
        builder.addText('o.');
        builder.pop();
      });

      expect(split(paragraph), <_Fragment>[
        _Fragment('Lor', prohibited, ltr, ffLtr, style1),
        _Fragment('\n', mandatory, null, ffSandwich, style1, nl: 1, sp: 1),
        _Fragment('em', prohibited, ltr, ffLtr, style1),
        _Fragment(' \n', mandatory, null, ffSandwich, style1, nl: 1, sp: 2),
        _Fragment(' \n', mandatory, null, ffSandwich, style2, nl: 1, sp: 2),
        _Fragment('  ', opportunity, null, ffSandwich, style2, sp: 2),
        _Fragment('ipsum', prohibited, ltr, ffLtr, style2),
        _Fragment(' ', opportunity, null, ffSandwich, style2, sp: 1),
        _Fragment('12', prohibited, ltr, ffPrevious, style2),
        _Fragment(' ', prohibited, null, ffSandwich, style2, sp: 1),
        _Fragment(' ', opportunity, null, ffSandwich, style3, sp: 1),
        _Fragment(rtlWord1, prohibited, rtl, ffRtl, style3),
        _Fragment(' ', opportunity, null, ffSandwich, style3, sp: 1),
        _Fragment('fo', prohibited, ltr, ffLtr, style3),
        _Fragment('o', prohibited, ltr, ffLtr, style1),
        _Fragment('.', endOfText, null, ffSandwich, style1),
      ]);
    });

    test('last line is empty', () {
      final CanvasParagraph paragraph = rich(EngineParagraphStyle(), (
        CanvasParagraphBuilder builder,
      ) {
        builder.pushStyle(style1);
        builder.addText('Lorem \n');
        builder.pop();
        builder.pushStyle(style2);
        builder.addText(' \n  ipsum \n');
        builder.pop();
      });

      expect(split(paragraph), <_Fragment>[
        _Fragment('Lorem', prohibited, ltr, ffLtr, style1),
        _Fragment(' \n', mandatory, null, ffSandwich, style1, nl: 1, sp: 2),
        _Fragment(' \n', mandatory, null, ffSandwich, style2, nl: 1, sp: 2),
        _Fragment('  ', opportunity, null, ffSandwich, style2, sp: 2),
        _Fragment('ipsum', prohibited, ltr, ffLtr, style2),
        _Fragment(' \n', mandatory, null, ffSandwich, style2, nl: 1, sp: 2),
        _Fragment('', endOfText, null, ffSandwich, style2),
      ]);
    });

    test('space-only spans', () {
      final CanvasParagraph paragraph = rich(EngineParagraphStyle(), (
        CanvasParagraphBuilder builder,
      ) {
        builder.addText('Lorem ');
        builder.pushStyle(style1);
        builder.addText('   ');
        builder.pop();
        builder.pushStyle(style2);
        builder.addText('  ');
        builder.pop();
        builder.addText('ipsum');
      });

      expect(split(paragraph), <_Fragment>[
        _Fragment('Lorem', prohibited, ltr, ffLtr, defaultStyle),
        _Fragment(' ', prohibited, null, ffSandwich, defaultStyle, sp: 1),
        _Fragment('   ', prohibited, null, ffSandwich, style1, sp: 3),
        _Fragment('  ', opportunity, null, ffSandwich, style2, sp: 2),
        _Fragment('ipsum', endOfText, ltr, ffLtr, defaultStyle),
      ]);
    });

    test('placeholders', () {
      final CanvasParagraph paragraph = rich(EngineParagraphStyle(), (
        CanvasParagraphBuilder builder,
      ) {
        builder.pushStyle(style1);
        builder.addPlaceholder(100, 100, PlaceholderAlignment.top);
        builder.addText('Lorem');
        builder.addPlaceholder(100, 100, PlaceholderAlignment.top);
        builder.addText('ipsum\n');
        builder.addPlaceholder(100, 100, PlaceholderAlignment.top);
        builder.pop();
        builder.pushStyle(style2);
        builder.addText('$rtlWord1 ');
        builder.addPlaceholder(100, 100, PlaceholderAlignment.top);
        builder.addText('\nsit');
        builder.pop();
        builder.addPlaceholder(100, 100, PlaceholderAlignment.top);
      });

      expect(split(paragraph), <_Fragment>[
        _Fragment(placeholderChar, opportunity, ltr, ffLtr, style1),
        _Fragment('Lorem', opportunity, ltr, ffLtr, style1),
        _Fragment(placeholderChar, opportunity, ltr, ffLtr, style1),
        _Fragment('ipsum', prohibited, ltr, ffLtr, style1),
        _Fragment('\n', mandatory, null, ffSandwich, style1, nl: 1, sp: 1),
        _Fragment(placeholderChar, opportunity, ltr, ffLtr, style1),
        _Fragment(rtlWord1, prohibited, rtl, ffRtl, style2),
        _Fragment(' ', opportunity, null, ffSandwich, style2, sp: 1),
        _Fragment(placeholderChar, prohibited, ltr, ffLtr, style2),
        _Fragment('\n', mandatory, null, ffSandwich, style2, nl: 1, sp: 1),
        _Fragment('sit', opportunity, ltr, ffLtr, style2),
        _Fragment(placeholderChar, endOfText, ltr, ffLtr, defaultStyle),
      ]);
    });
  });
}

/// Holds information about how a fragment.
class _Fragment {
  _Fragment(
    this.text,
    this.type,
    this.textDirection,
    this.fragmentFlow,
    this.style, {
    this.nl = 0,
    this.sp = 0,
  });

  factory _Fragment._fromLayoutFragment(String text, LayoutFragment layoutFragment) {
    return _Fragment(
      text.substring(layoutFragment.start, layoutFragment.end),
      layoutFragment.type,
      layoutFragment.textDirection,
      layoutFragment.fragmentFlow,
      layoutFragment.style,
      nl: layoutFragment.trailingNewlines,
      sp: layoutFragment.trailingSpaces,
    );
  }

  final String text;
  final LineBreakType type;
  final TextDirection? textDirection;
  final FragmentFlow fragmentFlow;
  final EngineTextStyle style;

  /// The number of trailing new line characters.
  final int nl;

  /// The number of trailing spaces.
  final int sp;

  @override
  int get hashCode => Object.hash(text, type, textDirection, fragmentFlow, style, nl, sp);

  @override
  bool operator ==(Object other) {
    return other is _Fragment &&
        other.text == text &&
        other.type == type &&
        other.textDirection == textDirection &&
        other.fragmentFlow == fragmentFlow &&
        other.style == style &&
        other.nl == nl &&
        other.sp == sp;
  }

  @override
  String toString() {
    return '"$text" ($type, $textDirection, $fragmentFlow, nl: $nl, sp: $sp)';
  }
}

List<_Fragment> split(CanvasParagraph paragraph) {
  return <_Fragment>[
    for (final LayoutFragment layoutFragment in computeLayoutFragments(paragraph))
      _Fragment._fromLayoutFragment(paragraph.plainText, layoutFragment),
  ];
}

List<LayoutFragment> computeLayoutFragments(CanvasParagraph paragraph) {
  return LayoutFragmenter(paragraph.plainText, paragraph.spans).fragment();
}

extension _EngineTextStyle on EngineTextStyle {
  EngineTextStyle copyWith({
    Color? color,
    TextDecoration? decoration,
    Color? decorationColor,
    TextDecorationStyle? decorationStyle,
    double? decorationThickness,
    FontWeight? fontWeight,
    FontStyle? fontStyle,
    TextBaseline? textBaseline,
    String? fontFamily,
    List<String>? fontFamilyFallback,
    double? fontSize,
    double? letterSpacing,
    double? wordSpacing,
    double? height,
    TextLeadingDistribution? leadingDistribution,
    Locale? locale,
    Paint? background,
    Paint? foreground,
    List<Shadow>? shadows,
    List<FontFeature>? fontFeatures,
    List<FontVariation>? fontVariations,
  }) {
    return EngineTextStyle(
      color: color ?? this.color,
      decoration: decoration ?? this.decoration,
      decorationColor: decorationColor ?? this.decorationColor,
      decorationStyle: decorationStyle ?? this.decorationStyle,
      decorationThickness: decorationThickness ?? this.decorationThickness,
      fontWeight: fontWeight ?? this.fontWeight,
      fontStyle: fontStyle ?? this.fontStyle,
      textBaseline: textBaseline ?? this.textBaseline,
      fontFamily: fontFamily ?? this.fontFamily,
      fontFamilyFallback: fontFamilyFallback ?? this.fontFamilyFallback,
      fontSize: fontSize ?? this.fontSize,
      letterSpacing: letterSpacing ?? this.letterSpacing,
      wordSpacing: wordSpacing ?? this.wordSpacing,
      height: height ?? this.height,
      leadingDistribution: leadingDistribution,
      locale: locale ?? this.locale,
      background: background ?? this.background,
      foreground: foreground ?? this.foreground,
      shadows: shadows ?? this.shadows,
      fontFeatures: fontFeatures ?? this.fontFeatures,
      fontVariations: fontVariations ?? this.fontVariations,
    );
  }
}
