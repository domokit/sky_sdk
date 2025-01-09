// Copyright 2013 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

import 'dart:convert';
import 'dart:ffi';
import 'dart:js_interop';

import 'package:ui/src/engine.dart';
import 'package:ui/src/engine/skwasm/skwasm_impl.dart';
import 'package:ui/ui.dart' as ui;
import 'package:ui/ui_web/src/ui_web.dart' as ui_web;

const int _kSoftLineBreak = 0;
const int _kHardLineBreak = 100;

final List<String> _testFonts = <String>['FlutterTest', 'Ahem'];
List<String> _computeEffectiveFontFamilies(List<String> fontFamilies) {
  if (!ui_web.debugEmulateFlutterTesterEnvironment) {
    return fontFamilies;
  }
  final Iterable<String> filteredFonts = fontFamilies.where(_testFonts.contains);
  return filteredFonts.isEmpty ? _testFonts : filteredFonts.toList();
}

class SkwasmLineMetrics extends SkwasmObjectWrapper<RawLineMetrics> implements ui.LineMetrics {
  factory SkwasmLineMetrics({
    required bool hardBreak,
    required double ascent,
    required double descent,
    required double unscaledAscent,
    required double height,
    required double width,
    required double left,
    required double baseline,
    required int lineNumber,
  }) => SkwasmLineMetrics._(
    lineMetricsCreate(
      hardBreak,
      ascent,
      descent,
      unscaledAscent,
      height,
      width,
      left,
      baseline,
      lineNumber,
    ),
  );

  SkwasmLineMetrics._(LineMetricsHandle handle) : super(handle, _registry);

  static final SkwasmFinalizationRegistry<RawLineMetrics> _registry =
      SkwasmFinalizationRegistry<RawLineMetrics>(lineMetricsDispose);

  @override
  bool get hardBreak => lineMetricsGetHardBreak(handle);

  @override
  double get ascent => lineMetricsGetAscent(handle);

  @override
  double get descent => lineMetricsGetDescent(handle);

  @override
  double get unscaledAscent => lineMetricsGetUnscaledAscent(handle);

  @override
  double get height => lineMetricsGetHeight(handle);

  @override
  double get width => lineMetricsGetWidth(handle);

  @override
  double get left => lineMetricsGetLeft(handle);

  @override
  double get baseline => lineMetricsGetBaseline(handle);

  @override
  int get lineNumber => lineMetricsGetLineNumber(handle);

  int get startIndex => lineMetricsGetStartIndex(handle);
  int get endIndex => lineMetricsGetEndIndex(handle);
}

class SkwasmParagraph extends SkwasmObjectWrapper<RawParagraph> implements ui.Paragraph {
  SkwasmParagraph(ParagraphHandle handle) : super(handle, _registry);

  static final SkwasmFinalizationRegistry<RawParagraph> _registry =
      SkwasmFinalizationRegistry<RawParagraph>(paragraphDispose);

  bool _hasCheckedForMissingCodePoints = false;

  @override
  double get width => paragraphGetWidth(handle);

  @override
  double get height => paragraphGetHeight(handle);

  @override
  double get longestLine => paragraphGetLongestLine(handle);

  @override
  double get minIntrinsicWidth => paragraphGetMinIntrinsicWidth(handle);

  @override
  double get maxIntrinsicWidth => paragraphGetMaxIntrinsicWidth(handle);

  @override
  double get alphabeticBaseline => paragraphGetAlphabeticBaseline(handle);

  @override
  double get ideographicBaseline => paragraphGetIdeographicBaseline(handle);

  @override
  bool get didExceedMaxLines => paragraphGetDidExceedMaxLines(handle);

  @override
  int get numberOfLines => paragraphGetLineCount(handle);

  @override
  int? getLineNumberAt(int codeUnitOffset) {
    final int lineNumber = paragraphGetLineNumberAt(handle, codeUnitOffset);
    return lineNumber >= 0 ? lineNumber : null;
  }

  @override
  void layout(ui.ParagraphConstraints constraints) {
    paragraphLayout(handle, constraints.width);
    if (!debugDisableFontFallbacks && !_hasCheckedForMissingCodePoints) {
      _hasCheckedForMissingCodePoints = true;
      final int missingCodePointCount = paragraphGetUnresolvedCodePoints(handle, nullptr, 0);
      if (missingCodePointCount > 0) {
        withStackScope((StackScope scope) {
          final Pointer<Uint32> codePointBuffer = scope.allocUint32Array(missingCodePointCount);
          final int returnedCodePointCount = paragraphGetUnresolvedCodePoints(
            handle,
            codePointBuffer,
            missingCodePointCount,
          );
          assert(missingCodePointCount == returnedCodePointCount);
          renderer.fontCollection.fontFallbackManager!.addMissingCodePoints(
            List<int>.generate(missingCodePointCount, (int index) => codePointBuffer[index]),
          );
        });
      }
    }
  }

  List<ui.TextBox> _convertTextBoxList(TextBoxListHandle listHandle) {
    final int length = textBoxListGetLength(listHandle);
    return withStackScope((StackScope scope) {
      final RawRect tempRect = scope.allocFloatArray(4);
      return List<ui.TextBox>.generate(length, (int index) {
        final int textDirectionIndex = textBoxListGetBoxAtIndex(listHandle, index, tempRect);
        return ui.TextBox.fromLTRBD(
          tempRect[0],
          tempRect[1],
          tempRect[2],
          tempRect[3],
          ui.TextDirection.values[textDirectionIndex],
        );
      });
    });
  }

  @override
  List<ui.TextBox> getBoxesForRange(
    int start,
    int end, {
    ui.BoxHeightStyle boxHeightStyle = ui.BoxHeightStyle.tight,
    ui.BoxWidthStyle boxWidthStyle = ui.BoxWidthStyle.tight,
  }) {
    final TextBoxListHandle listHandle = paragraphGetBoxesForRange(
      handle,
      start,
      end,
      boxHeightStyle.index,
      boxWidthStyle.index,
    );
    final List<ui.TextBox> boxes = _convertTextBoxList(listHandle);
    textBoxListDispose(listHandle);
    return boxes;
  }

  @override
  ui.TextPosition getPositionForOffset(ui.Offset offset) => withStackScope((StackScope scope) {
    final Pointer<Int32> outAffinity = scope.allocInt32Array(1);
    final int position = paragraphGetPositionForOffset(handle, offset.dx, offset.dy, outAffinity);
    return ui.TextPosition(offset: position, affinity: ui.TextAffinity.values[outAffinity[0]]);
  });

  @override
  ui.GlyphInfo? getGlyphInfoAt(int codeUnitOffset) {
    return withStackScope((StackScope scope) {
      final Pointer<Float> outRect = scope.allocFloatArray(4);
      final Pointer<Uint32> outRange = scope.allocUint32Array(2);
      final Pointer<Bool> outBooleanFlags = scope.allocBoolArray(1);
      return paragraphGetGlyphInfoAt(handle, codeUnitOffset, outRect, outRange, outBooleanFlags)
          ? ui.GlyphInfo(
            scope.convertRectFromNative(outRect),
            ui.TextRange(start: outRange[0], end: outRange[1]),
            outBooleanFlags[0] ? ui.TextDirection.ltr : ui.TextDirection.rtl,
          )
          : null;
    });
  }

  @override
  ui.GlyphInfo? getClosestGlyphInfoForOffset(ui.Offset offset) {
    return withStackScope((StackScope scope) {
      final Pointer<Float> outRect = scope.allocFloatArray(4);
      final Pointer<Uint32> outRange = scope.allocUint32Array(2);
      final Pointer<Bool> outBooleanFlags = scope.allocBoolArray(1);
      return paragraphGetClosestGlyphInfoAtCoordinate(
            handle,
            offset.dx,
            offset.dy,
            outRect,
            outRange,
            outBooleanFlags,
          )
          ? ui.GlyphInfo(
            scope.convertRectFromNative(outRect),
            ui.TextRange(start: outRange[0], end: outRange[1]),
            outBooleanFlags[0] ? ui.TextDirection.ltr : ui.TextDirection.rtl,
          )
          : null;
    });
  }

  @override
  ui.TextRange getWordBoundary(ui.TextPosition position) => withStackScope((StackScope scope) {
    final Pointer<Int32> outRange = scope.allocInt32Array(2);
    final int characterPosition = switch (position.affinity) {
      ui.TextAffinity.upstream => position.offset - 1,
      ui.TextAffinity.downstream => position.offset,
    };
    paragraphGetWordBoundary(handle, characterPosition, outRange);
    return ui.TextRange(start: outRange[0], end: outRange[1]);
  });

  @override
  ui.TextRange getLineBoundary(ui.TextPosition position) {
    final int offset = position.offset;
    for (final SkwasmLineMetrics metrics in computeLineMetrics()) {
      if (offset >= metrics.startIndex && offset <= metrics.endIndex) {
        return ui.TextRange(start: metrics.startIndex, end: metrics.endIndex);
      }
    }
    return ui.TextRange.empty;
  }

  @override
  List<ui.TextBox> getBoxesForPlaceholders() {
    final TextBoxListHandle listHandle = paragraphGetBoxesForPlaceholders(handle);
    final List<ui.TextBox> boxes = _convertTextBoxList(listHandle);
    textBoxListDispose(listHandle);
    return boxes;
  }

  @override
  List<SkwasmLineMetrics> computeLineMetrics() {
    final int lineCount = paragraphGetLineCount(handle);
    return List<SkwasmLineMetrics>.generate(
      lineCount,
      (int index) => SkwasmLineMetrics._(paragraphGetLineMetricsAtIndex(handle, index)),
    );
  }

  @override
  ui.LineMetrics? getLineMetricsAt(int index) {
    final LineMetricsHandle lineMetrics = paragraphGetLineMetricsAtIndex(handle, index);
    return lineMetrics != nullptr ? SkwasmLineMetrics._(lineMetrics) : null;
  }
}

void withScopedFontList(
  List<String> fontFamilies,
  void Function(Pointer<SkStringHandle>, int) callback,
) {
  withStackScope((StackScope scope) {
    final Pointer<SkStringHandle> familiesPtr =
        scope.allocPointerArray(fontFamilies.length).cast<SkStringHandle>();
    int nativeIndex = 0;
    for (int i = 0; i < fontFamilies.length; i++) {
      familiesPtr[nativeIndex] = skStringFromDartString(fontFamilies[i]);
      nativeIndex++;
    }
    callback(familiesPtr, fontFamilies.length);
    for (int i = 0; i < fontFamilies.length; i++) {
      skStringFree(familiesPtr[i]);
    }
  });
}

class SkwasmNativeTextStyle extends SkwasmObjectWrapper<RawTextStyle> {
  SkwasmNativeTextStyle(TextStyleHandle handle) : super(handle, _registry);

  factory SkwasmNativeTextStyle.defaultTextStyle() => SkwasmNativeTextStyle(textStyleCreate());

  static final SkwasmFinalizationRegistry<RawTextStyle> _registry =
      SkwasmFinalizationRegistry<RawTextStyle>(textStyleDispose);

  SkwasmNativeTextStyle copy() {
    return SkwasmNativeTextStyle(textStyleCopy(handle));
  }
}

class SkwasmTextStyle implements ui.TextStyle {
  SkwasmTextStyle({
    this.color,
    this.decoration,
    this.decorationColor,
    this.decorationStyle,
    this.decorationThickness,
    this.fontWeight,
    this.fontStyle,
    this.textBaseline,
    this.fontFamily,
    this.fontFamilyFallback,
    this.fontSize,
    this.letterSpacing,
    this.wordSpacing,
    this.height,
    this.leadingDistribution,
    this.locale,
    this.background,
    this.foreground,
    this.shadows,
    this.fontFeatures,
    this.fontVariations,
  }) : assert(
         color == null || foreground == null,
         'Cannot provide both a color and a foreground\n'
         'The color argument is just a shorthand for "foreground: Paint()..color = color".',
       );

  void applyToNative(SkwasmNativeTextStyle style) {
    final TextStyleHandle handle = style.handle;
    if (color != null) {
      textStyleSetColor(handle, color!.value);
    }
    if (decoration != null) {
      textStyleSetDecoration(handle, decoration!.maskValue);
    }
    if (decorationColor != null) {
      textStyleSetDecorationColor(handle, decorationColor!.value);
    }
    if (decorationStyle != null) {
      textStyleSetDecorationStyle(handle, decorationStyle!.index);
    }
    if (decorationThickness != null) {
      textStyleSetDecorationThickness(handle, decorationThickness!);
    }
    if (fontWeight != null || fontStyle != null) {
      textStyleSetFontStyle(
        handle,
        (fontWeight ?? ui.FontWeight.normal).value,
        (fontStyle ?? ui.FontStyle.normal).index,
      );
    }
    if (textBaseline != null) {
      textStyleSetTextBaseline(handle, textBaseline!.index);
    }

    final List<String> effectiveFontFamilies = _computeEffectiveFontFamilies(fontFamilies);
    if (effectiveFontFamilies.isNotEmpty) {
      withScopedFontList(
        effectiveFontFamilies,
        (Pointer<SkStringHandle> families, int count) =>
            textStyleAddFontFamilies(handle, families, count),
      );
    }

    if (fontSize != null) {
      textStyleSetFontSize(handle, fontSize!);
    }
    if (letterSpacing != null) {
      textStyleSetLetterSpacing(handle, letterSpacing!);
    }
    if (wordSpacing != null) {
      textStyleSetWordSpacing(handle, wordSpacing!);
    }
    if (height != null) {
      textStyleSetHeight(handle, height!);
    }
    if (leadingDistribution != null) {
      textStyleSetHalfLeading(handle, leadingDistribution == ui.TextLeadingDistribution.even);
    }
    if (locale != null) {
      final SkStringHandle localeHandle = skStringFromDartString(locale!.toLanguageTag());
      textStyleSetLocale(handle, localeHandle);
      skStringFree(localeHandle);
    }
    if (background != null) {
      final backgroundPaint = (background! as SkwasmPaint).toRawPaint();
      textStyleSetBackground(handle, backgroundPaint);
      paintDispose(backgroundPaint);
    }
    if (foreground != null) {
      final foregroundPaint = (foreground! as SkwasmPaint).toRawPaint();
      textStyleSetForeground(handle, foregroundPaint);
      paintDispose(foregroundPaint);
    }
    if (shadows != null) {
      for (final ui.Shadow shadow in shadows!) {
        textStyleAddShadow(
          handle,
          shadow.color.value,
          shadow.offset.dx,
          shadow.offset.dy,
          shadow.blurSigma,
        );
      }
    }
    if (fontFeatures != null) {
      for (final ui.FontFeature feature in fontFeatures!) {
        final SkStringHandle featureName = skStringFromDartString(feature.feature);
        textStyleAddFontFeature(handle, featureName, feature.value);
        skStringFree(featureName);
      }
    }

    if (fontVariations != null && fontVariations!.isNotEmpty) {
      final int variationCount = fontVariations!.length;
      withStackScope((StackScope scope) {
        final Pointer<Uint32> axisBuffer = scope.allocUint32Array(variationCount);
        final Pointer<Float> valueBuffer = scope.allocFloatArray(variationCount);
        for (int i = 0; i < variationCount; i++) {
          final ui.FontVariation variation = fontVariations![i];
          final String axis = variation.axis;
          assert(axis.length == 4); // 4 byte code
          final int axisNumber =
              axis.codeUnitAt(0) << 24 |
              axis.codeUnitAt(1) << 16 |
              axis.codeUnitAt(2) << 8 |
              axis.codeUnitAt(3);
          axisBuffer[i] = axisNumber;
          valueBuffer[i] = variation.value;
        }
        textStyleSetFontVariations(handle, axisBuffer, valueBuffer, variationCount);
      });
    }
  }

  List<String> get fontFamilies => <String>[
    if (fontFamily != null) fontFamily!,
    if (fontFamilyFallback != null) ...fontFamilyFallback!,
  ];

  final ui.Color? color;
  final ui.TextDecoration? decoration;
  final ui.Color? decorationColor;
  final ui.TextDecorationStyle? decorationStyle;
  final double? decorationThickness;
  final ui.FontWeight? fontWeight;
  final ui.FontStyle? fontStyle;
  final ui.TextBaseline? textBaseline;
  final String? fontFamily;
  final List<String>? fontFamilyFallback;
  final double? fontSize;
  final double? letterSpacing;
  final double? wordSpacing;
  final double? height;
  final ui.TextLeadingDistribution? leadingDistribution;
  final ui.Locale? locale;
  final ui.Paint? background;
  final ui.Paint? foreground;
  final List<ui.Shadow>? shadows;
  final List<ui.FontFeature>? fontFeatures;
  final List<ui.FontVariation>? fontVariations;

  @override
  bool operator ==(Object other) {
    if (identical(this, other)) {
      return true;
    }
    return other is SkwasmTextStyle &&
        other.color == color &&
        other.decoration == decoration &&
        other.decorationColor == decorationColor &&
        other.decorationStyle == decorationStyle &&
        other.fontWeight == fontWeight &&
        other.fontStyle == fontStyle &&
        other.textBaseline == textBaseline &&
        other.leadingDistribution == leadingDistribution &&
        other.fontFamily == fontFamily &&
        other.fontSize == fontSize &&
        other.letterSpacing == letterSpacing &&
        other.wordSpacing == wordSpacing &&
        other.height == height &&
        other.decorationThickness == decorationThickness &&
        other.locale == locale &&
        other.background == background &&
        other.foreground == foreground &&
        listEquals<ui.Shadow>(other.shadows, shadows) &&
        listEquals<String>(other.fontFamilyFallback, fontFamilyFallback) &&
        listEquals<ui.FontFeature>(other.fontFeatures, fontFeatures) &&
        listEquals<ui.FontVariation>(other.fontVariations, fontVariations);
  }

  @override
  int get hashCode {
    final List<ui.Shadow>? shadows = this.shadows;
    final List<ui.FontFeature>? fontFeatures = this.fontFeatures;
    final List<ui.FontVariation>? fontVariations = this.fontVariations;
    final List<String>? fontFamilyFallback = this.fontFamilyFallback;
    return Object.hash(
      color,
      decoration,
      decorationColor,
      decorationStyle,
      fontWeight,
      fontStyle,
      textBaseline,
      leadingDistribution,
      fontFamily,
      fontFamilyFallback == null ? null : Object.hashAll(fontFamilyFallback),
      fontSize,
      letterSpacing,
      wordSpacing,
      height,
      locale,
      background,
      foreground,
      shadows == null ? null : Object.hashAll(shadows),
      decorationThickness,
      // Object.hash goes up to 20 arguments, but we have 21
      Object.hash(
        fontFeatures == null ? null : Object.hashAll(fontFeatures),
        fontVariations == null ? null : Object.hashAll(fontVariations),
      ),
    );
  }

  @override
  String toString() {
    String result = super.toString();
    assert(() {
      final List<String>? fontFamilyFallback = this.fontFamilyFallback;
      final double? fontSize = this.fontSize;
      final double? height = this.height;
      result =
          'TextStyle('
          'color: ${color ?? "unspecified"}, '
          'decoration: ${decoration ?? "unspecified"}, '
          'decorationColor: ${decorationColor ?? "unspecified"}, '
          'decorationStyle: ${decorationStyle ?? "unspecified"}, '
          'decorationThickness: ${decorationThickness ?? "unspecified"}, '
          'fontWeight: ${fontWeight ?? "unspecified"}, '
          'fontStyle: ${fontStyle ?? "unspecified"}, '
          'textBaseline: ${textBaseline ?? "unspecified"}, '
          'fontFamily: ${fontFamily ?? "unspecified"}, '
          'fontFamilyFallback: ${fontFamilyFallback != null && fontFamilyFallback.isNotEmpty ? fontFamilyFallback : "unspecified"}, '
          'fontSize: ${fontSize != null ? fontSize.toStringAsFixed(1) : "unspecified"}, '
          'letterSpacing: ${letterSpacing != null ? "${letterSpacing}x" : "unspecified"}, '
          'wordSpacing: ${wordSpacing != null ? "${wordSpacing}x" : "unspecified"}, '
          'height: ${height != null ? "${height.toStringAsFixed(1)}x" : "unspecified"}, '
          'leadingDistribution: ${leadingDistribution ?? "unspecified"}, '
          'locale: ${locale ?? "unspecified"}, '
          'background: ${background ?? "unspecified"}, '
          'foreground: ${foreground ?? "unspecified"}, '
          'shadows: ${shadows ?? "unspecified"}, '
          'fontFeatures: ${fontFeatures ?? "unspecified"}, '
          'fontVariations: ${fontVariations ?? "unspecified"}'
          ')';
      return true;
    }());
    return result;
  }
}

final class SkwasmStrutStyle extends SkwasmObjectWrapper<RawStrutStyle> implements ui.StrutStyle {
  factory SkwasmStrutStyle({
    String? fontFamily,
    List<String>? fontFamilyFallback,
    double? fontSize,
    double? height,
    double? leading,
    ui.FontWeight? fontWeight,
    ui.FontStyle? fontStyle,
    bool? forceStrutHeight,
    ui.TextLeadingDistribution? leadingDistribution,
  }) {
    final StrutStyleHandle handle = strutStyleCreate();
    final List<String> effectiveFontFamilies = _computeEffectiveFontFamilies(<String>[
      if (fontFamily != null) fontFamily,
      if (fontFamilyFallback != null) ...fontFamilyFallback,
    ]);
    if (effectiveFontFamilies.isNotEmpty) {
      withScopedFontList(
        effectiveFontFamilies,
        (Pointer<SkStringHandle> families, int count) =>
            strutStyleSetFontFamilies(handle, families, count),
      );
    }
    if (fontSize != null) {
      strutStyleSetFontSize(handle, fontSize);
    }
    if (height != null && height != ui.kTextHeightNone) {
      strutStyleSetHeight(handle, height);
    }
    if (leadingDistribution != null) {
      strutStyleSetHalfLeading(handle, leadingDistribution == ui.TextLeadingDistribution.even);
    }
    if (leading != null) {
      strutStyleSetLeading(handle, leading);
    }
    if (fontWeight != null || fontStyle != null) {
      fontWeight ??= ui.FontWeight.normal;
      fontStyle ??= ui.FontStyle.normal;
      strutStyleSetFontStyle(handle, fontWeight.value, fontStyle.index);
    }
    if (forceStrutHeight != null) {
      strutStyleSetForceStrutHeight(handle, forceStrutHeight);
    }
    return SkwasmStrutStyle._(
      handle,
      fontFamily,
      fontFamilyFallback,
      fontSize,
      height,
      leadingDistribution,
      leading,
      fontWeight,
      fontStyle,
      forceStrutHeight,
    );
  }

  SkwasmStrutStyle._(
    StrutStyleHandle handle,
    this._fontFamily,
    this._fontFamilyFallback,
    this._fontSize,
    this._height,
    this._leadingDistribution,
    this._leading,
    this._fontWeight,
    this._fontStyle,
    this._forceStrutHeight,
  ) : super(handle, _registry);

  static final SkwasmFinalizationRegistry<RawStrutStyle> _registry =
      SkwasmFinalizationRegistry<RawStrutStyle>(strutStyleDispose);

  final String? _fontFamily;
  final List<String>? _fontFamilyFallback;
  final double? _fontSize;
  final double? _height;
  final double? _leading;
  final ui.FontWeight? _fontWeight;
  final ui.FontStyle? _fontStyle;
  final bool? _forceStrutHeight;
  final ui.TextLeadingDistribution? _leadingDistribution;

  @override
  bool operator ==(Object other) {
    return other is SkwasmStrutStyle &&
        other._fontFamily == _fontFamily &&
        other._fontSize == _fontSize &&
        other._height == _height &&
        other._leading == _leading &&
        other._leadingDistribution == _leadingDistribution &&
        other._fontWeight == _fontWeight &&
        other._fontStyle == _fontStyle &&
        other._forceStrutHeight == _forceStrutHeight &&
        listEquals<String>(other._fontFamilyFallback, _fontFamilyFallback);
  }

  @override
  int get hashCode {
    final List<String>? fontFamilyFallback = _fontFamilyFallback;
    return Object.hash(
      _fontFamily,
      fontFamilyFallback != null ? Object.hashAll(fontFamilyFallback) : null,
      _fontSize,
      _height,
      _leading,
      _leadingDistribution,
      _fontWeight,
      _fontStyle,
      _forceStrutHeight,
    );
  }

  @override
  String toString() {
    String result = super.toString();
    assert(() {
      final List<String>? fontFamilyFallback = _fontFamilyFallback;
      final double? fontSize = _fontSize;
      final double? height = _height;
      final double? leading = _leading;
      result =
          'StrutStyle('
          'fontFamily: ${_fontFamily ?? "unspecified"}, '
          'fontFamilyFallback: ${_fontFamilyFallback ?? "unspecified"}, '
          'fontFamilyFallback: ${fontFamilyFallback != null && fontFamilyFallback.isNotEmpty ? fontFamilyFallback : "unspecified"}, '
          'fontSize: ${fontSize != null ? fontSize.toStringAsFixed(1) : "unspecified"}, '
          'height: ${height != null ? "${height.toStringAsFixed(1)}x" : "unspecified"}, '
          'leading: ${leading != null ? "${leading.toStringAsFixed(1)}x" : "unspecified"}, '
          'fontWeight: ${_fontWeight ?? "unspecified"}, '
          'fontStyle: ${_fontStyle ?? "unspecified"}, '
          'forceStrutHeight: ${_forceStrutHeight ?? "unspecified"}, '
          'leadingDistribution: ${_leadingDistribution ?? "unspecified"}, '
          ')';
      return true;
    }());
    return result;
  }
}

class SkwasmParagraphStyle extends SkwasmObjectWrapper<RawParagraphStyle>
    implements ui.ParagraphStyle {
  factory SkwasmParagraphStyle({
    ui.TextAlign? textAlign,
    ui.TextDirection? textDirection,
    int? maxLines,
    String? fontFamily,
    double? fontSize,
    double? height,
    ui.TextHeightBehavior? textHeightBehavior,
    ui.FontWeight? fontWeight,
    ui.FontStyle? fontStyle,
    ui.StrutStyle? strutStyle,
    String? ellipsis,
    ui.Locale? locale,
  }) {
    final ParagraphStyleHandle handle = paragraphStyleCreate();
    if (textAlign != null) {
      paragraphStyleSetTextAlign(handle, textAlign.index);
    }
    if (textDirection != null) {
      paragraphStyleSetTextDirection(handle, textDirection.index);
    }
    if (maxLines != null) {
      paragraphStyleSetMaxLines(handle, maxLines);
    }
    if (height != null && height != ui.kTextHeightNone) {
      paragraphStyleSetHeight(handle, height);
    }
    if (textHeightBehavior != null) {
      paragraphStyleSetTextHeightBehavior(
        handle,
        textHeightBehavior.applyHeightToFirstAscent,
        textHeightBehavior.applyHeightToLastDescent,
      );
    }
    if (ellipsis != null) {
      final SkStringHandle ellipsisHandle = skStringFromDartString(ellipsis);
      paragraphStyleSetEllipsis(handle, ellipsisHandle);
      skStringFree(ellipsisHandle);
    }
    if (strutStyle != null) {
      strutStyle as SkwasmStrutStyle;
      paragraphStyleSetStrutStyle(handle, strutStyle.handle);
    }
    final SkwasmNativeTextStyle textStyle =
        (renderer.fontCollection as SkwasmFontCollection).defaultTextStyle.copy();
    final TextStyleHandle textStyleHandle = textStyle.handle;

    final List<String> effectiveFontFamilies = _computeEffectiveFontFamilies(<String>[
      if (fontFamily != null) fontFamily,
    ]);
    if (effectiveFontFamilies.isNotEmpty) {
      withScopedFontList(
        effectiveFontFamilies,
        (Pointer<SkStringHandle> families, int count) =>
            textStyleAddFontFamilies(textStyleHandle, families, count),
      );
    }
    if (fontSize != null) {
      textStyleSetFontSize(textStyleHandle, fontSize);
    }
    if (height != null) {
      textStyleSetHeight(textStyleHandle, height);
    }
    if (fontWeight != null || fontStyle != null) {
      fontWeight ??= ui.FontWeight.normal;
      fontStyle ??= ui.FontStyle.normal;
      textStyleSetFontStyle(textStyleHandle, fontWeight.value, fontStyle.index);
    }
    if (textHeightBehavior != null) {
      textStyleSetHalfLeading(
        textStyleHandle,
        textHeightBehavior.leadingDistribution == ui.TextLeadingDistribution.even,
      );
    }
    if (locale != null) {
      final SkStringHandle localeHandle = skStringFromDartString(locale.toLanguageTag());
      textStyleSetLocale(textStyleHandle, localeHandle);
      skStringFree(localeHandle);
    }
    paragraphStyleSetTextStyle(handle, textStyleHandle);
    paragraphStyleSetApplyRoundingHack(handle, false);

    return SkwasmParagraphStyle._(
      handle,
      textStyle,
      fontFamily,
      textAlign,
      textDirection,
      fontWeight,
      fontStyle,
      maxLines,
      fontFamily,
      fontSize,
      height,
      textHeightBehavior,
      strutStyle,
      ellipsis,
      locale,
    );
  }

  SkwasmParagraphStyle._(
    ParagraphStyleHandle handle,
    this.textStyle,
    this.defaultFontFamily,
    this._textAlign,
    this._textDirection,
    this._fontWeight,
    this._fontStyle,
    this._maxLines,
    this._fontFamily,
    this._fontSize,
    this._height,
    this._textHeightBehavior,
    this._strutStyle,
    this._ellipsis,
    this._locale,
  ) : super(handle, _registry);

  static final SkwasmFinalizationRegistry<RawParagraphStyle> _registry =
      SkwasmFinalizationRegistry<RawParagraphStyle>(paragraphStyleDispose);

  final SkwasmNativeTextStyle textStyle;
  final String? defaultFontFamily;

  final ui.TextAlign? _textAlign;
  final ui.TextDirection? _textDirection;
  final ui.FontWeight? _fontWeight;
  final ui.FontStyle? _fontStyle;
  final int? _maxLines;
  final String? _fontFamily;
  final double? _fontSize;
  final double? _height;
  final ui.TextHeightBehavior? _textHeightBehavior;
  final ui.StrutStyle? _strutStyle;
  final String? _ellipsis;
  final ui.Locale? _locale;

  @override
  bool operator ==(Object other) {
    if (identical(this, other)) {
      return true;
    }
    if (other.runtimeType != runtimeType) {
      return false;
    }
    return other is SkwasmParagraphStyle &&
        other._textAlign == _textAlign &&
        other._textDirection == _textDirection &&
        other._fontWeight == _fontWeight &&
        other._fontStyle == _fontStyle &&
        other._maxLines == _maxLines &&
        other._fontFamily == _fontFamily &&
        other._fontSize == _fontSize &&
        other._height == _height &&
        other._textHeightBehavior == _textHeightBehavior &&
        other._strutStyle == _strutStyle &&
        other._ellipsis == _ellipsis &&
        other._locale == _locale;
  }

  @override
  int get hashCode {
    return Object.hash(
      _textAlign,
      _textDirection,
      _fontWeight,
      _fontStyle,
      _maxLines,
      _fontFamily,
      _fontSize,
      _height,
      _textHeightBehavior,
      _strutStyle,
      _ellipsis,
      _locale,
    );
  }

  @override
  String toString() {
    String result = super.toString();
    assert(() {
      final double? fontSize = _fontSize;
      final double? height = _height;
      result =
          'ParagraphStyle('
          'textAlign: ${_textAlign ?? "unspecified"}, '
          'textDirection: ${_textDirection ?? "unspecified"}, '
          'fontWeight: ${_fontWeight ?? "unspecified"}, '
          'fontStyle: ${_fontStyle ?? "unspecified"}, '
          'maxLines: ${_maxLines ?? "unspecified"}, '
          'textHeightBehavior: ${_textHeightBehavior ?? "unspecified"}, '
          'fontFamily: ${_fontFamily ?? "unspecified"}, '
          'fontSize: ${fontSize != null ? fontSize.toStringAsFixed(1) : "unspecified"}, '
          'height: ${height != null ? "${height.toStringAsFixed(1)}x" : "unspecified"}, '
          'strutStyle: ${_strutStyle ?? "unspecified"}, '
          'ellipsis: ${_ellipsis != null ? '"$_ellipsis"' : "unspecified"}, '
          'locale: ${_locale ?? "unspecified"}'
          ')';
      return true;
    }());
    return result;
  }
}

class SkwasmParagraphBuilder extends SkwasmObjectWrapper<RawParagraphBuilder>
    implements ui.ParagraphBuilder {
  factory SkwasmParagraphBuilder(SkwasmParagraphStyle style, SkwasmFontCollection collection) =>
      SkwasmParagraphBuilder._(paragraphBuilderCreate(style.handle, collection.handle), style);

  SkwasmParagraphBuilder._(ParagraphBuilderHandle handle, this.style) : super(handle, _registry);

  static final SkwasmFinalizationRegistry<RawParagraphBuilder> _registry =
      SkwasmFinalizationRegistry<RawParagraphBuilder>(paragraphBuilderDispose);

  final SkwasmParagraphStyle style;
  final List<SkwasmNativeTextStyle> textStyleStack = <SkwasmNativeTextStyle>[];

  @override
  List<double> placeholderScales = <double>[];

  @override
  void addPlaceholder(
    double width,
    double height,
    ui.PlaceholderAlignment alignment, {
    double scale = 1.0,
    double? baselineOffset,
    ui.TextBaseline? baseline,
  }) {
    paragraphBuilderAddPlaceholder(
      handle,
      width * scale,
      height * scale,
      alignment.index,
      (baselineOffset ?? height) * scale,
      (baseline ?? ui.TextBaseline.alphabetic).index,
    );
    placeholderScales.add(scale);
  }

  @override
  void addText(String text) {
    final SkString16Handle stringHandle = skString16FromDartString(text);
    paragraphBuilderAddText(handle, stringHandle);
    skString16Free(stringHandle);
  }

  static final DomV8BreakIterator _v8BreakIterator = createV8BreakIterator();
  static final DomSegmenter _graphemeSegmenter = createIntlSegmenter(granularity: 'grapheme');
  static final DomSegmenter _wordSegmenter = createIntlSegmenter(granularity: 'word');
  static final DomTextDecoder _utf8Decoder = DomTextDecoder();

  void _addSegmenterData() => withStackScope((StackScope scope) {
    // Because some of the string processing is dealt with manually in dart,
    // and some is handled by the browser, we actually need both a dart string
    // and a JS string. Converting from linear memory to Dart strings or JS
    // strings is more efficient than directly converting to each other, so we
    // just create both up front here.
    final Pointer<Uint32> outSize = scope.allocUint32Array(1);
    final Pointer<Uint8> utf8Data = paragraphBuilderGetUtf8Text(handle, outSize);

    final String text;
    final JSString jsText;
    if (utf8Data == nullptr) {
      text = '';
      jsText = ''.toJS;
    } else {
      final List<int> codeUnitList = List<int>.generate(
        outSize.value,
        (int index) => utf8Data[index],
      );
      text = utf8.decode(codeUnitList);
      jsText = _utf8Decoder.decode(
        // In an ideal world we would just use a subview of wasm memory rather
        // than a slice, but the TextDecoder API doesn't work on shared buffer
        // sources yet.
        // See https://bugs.chromium.org/p/chromium/issues/detail?id=1012656
        createUint8ArrayFromBuffer(
          skwasmInstance.wasmMemory.buffer,
        ).slice(utf8Data.address.toJS, (utf8Data.address + outSize.value).toJS),
      );
    }

    _addGraphemeBreakData(text, jsText);
    _addWordBreakData(text, jsText);
    _addLineBreakData(text, jsText);
  });

  UnicodePositionBufferHandle _createBreakPositionBuffer(
    String text,
    JSString jsText,
    DomSegmenter segmenter,
  ) {
    final DomIteratorWrapper<DomSegment> iterator = segmenter.segmentRaw(jsText).iterator();
    final List<int> breaks = <int>[];
    while (iterator.moveNext()) {
      breaks.add(iterator.current.index);
    }
    breaks.add(text.length);

    final UnicodePositionBufferHandle positionBuffer = unicodePositionBufferCreate(breaks.length);
    final Pointer<Uint32> buffer = unicodePositionBufferGetDataPointer(positionBuffer);
    for (int i = 0; i < breaks.length; i++) {
      buffer[i] = breaks[i];
    }
    return positionBuffer;
  }

  void _addGraphemeBreakData(String text, JSString jsText) {
    final UnicodePositionBufferHandle positionBuffer = _createBreakPositionBuffer(
      text,
      jsText,
      _graphemeSegmenter,
    );
    paragraphBuilderSetGraphemeBreaksUtf16(handle, positionBuffer);
    unicodePositionBufferFree(positionBuffer);
  }

  void _addWordBreakData(String text, JSString jsText) {
    final UnicodePositionBufferHandle positionBuffer = _createBreakPositionBuffer(
      text,
      jsText,
      _wordSegmenter,
    );
    paragraphBuilderSetWordBreaksUtf16(handle, positionBuffer);
    unicodePositionBufferFree(positionBuffer);
  }

  void _addLineBreakData(String text, JSString jsText) {
    final List<LineBreakFragment> lineBreaks = breakLinesUsingV8BreakIterator(
      text,
      jsText,
      _v8BreakIterator,
    );
    final LineBreakBufferHandle lineBreakBuffer = lineBreakBufferCreate(lineBreaks.length + 1);
    final Pointer<LineBreak> lineBreakPointer = lineBreakBufferGetDataPointer(lineBreakBuffer);

    // First line break is always zero. The buffer is zero initialized, so we can just
    // skip the first one.
    for (int i = 0; i < lineBreaks.length; i++) {
      final LineBreakFragment fragment = lineBreaks[i];
      lineBreakPointer[i + 1].position = fragment.end;
      lineBreakPointer[i + 1].lineBreakType =
          fragment.type == LineBreakType.mandatory ? _kHardLineBreak : _kSoftLineBreak;
    }
    paragraphBuilderSetLineBreaksUtf16(handle, lineBreakBuffer);
    lineBreakBufferFree(lineBreakBuffer);
  }

  @override
  ui.Paragraph build() {
    _addSegmenterData();
    return SkwasmParagraph(paragraphBuilderBuild(handle));
  }

  @override
  int get placeholderCount => placeholderScales.length;

  @override
  void pop() {
    final SkwasmNativeTextStyle style = textStyleStack.removeLast();
    style.dispose();
    paragraphBuilderPop(handle);
  }

  @override
  void pushStyle(ui.TextStyle textStyle) {
    textStyle as SkwasmTextStyle;
    final SkwasmNativeTextStyle baseStyle =
        textStyleStack.isNotEmpty ? textStyleStack.last : style.textStyle;
    final SkwasmNativeTextStyle nativeStyle = baseStyle.copy();
    textStyle.applyToNative(nativeStyle);
    textStyleStack.add(nativeStyle);
    paragraphBuilderPushStyle(handle, nativeStyle.handle);
  }
}
