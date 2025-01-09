// Copyright 2013 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

import 'dart:typed_data';

import 'package:meta/meta.dart';
import 'package:ui/src/engine.dart';
import 'package:ui/ui.dart' as ui;
import 'package:ui/ui_web/src/ui_web.dart' as ui_web;

final bool _ckRequiresClientICU = canvasKit.ParagraphBuilder.RequiresClientICU();

final List<String> _testFonts = <String>['FlutterTest', 'Ahem'];
String? _computeEffectiveFontFamily(String? fontFamily) {
  return ui_web.debugEmulateFlutterTesterEnvironment && !_testFonts.contains(fontFamily)
      ? _testFonts.first
      : fontFamily;
}

@immutable
class CkParagraphStyle implements ui.ParagraphStyle {
  CkParagraphStyle({
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
  }) : skParagraphStyle = toSkParagraphStyle(
         textAlign,
         textDirection,
         maxLines,
         _computeEffectiveFontFamily(fontFamily),
         fontSize,
         height == ui.kTextHeightNone ? null : height,
         textHeightBehavior,
         fontWeight,
         fontStyle,
         strutStyle,
         ellipsis,
         locale,
       ),
       _textAlign = textAlign,
       _textDirection = textDirection,
       _fontWeight = fontWeight,
       _fontStyle = fontStyle,
       _maxLines = maxLines,
       _originalFontFamily = fontFamily,
       _effectiveFontFamily = _computeEffectiveFontFamily(fontFamily),
       _fontSize = fontSize,
       _height = height == ui.kTextHeightNone ? null : height,
       _textHeightBehavior = textHeightBehavior,
       _strutStyle = strutStyle,
       _ellipsis = ellipsis,
       _locale = locale;

  final SkParagraphStyle skParagraphStyle;

  final ui.TextAlign? _textAlign;
  final ui.TextDirection? _textDirection;
  final ui.FontWeight? _fontWeight;
  final ui.FontStyle? _fontStyle;
  final int? _maxLines;
  final String? _originalFontFamily;
  final String? _effectiveFontFamily;
  final double? _fontSize;
  final double? _height;
  final ui.TextHeightBehavior? _textHeightBehavior;
  final ui.StrutStyle? _strutStyle;
  final String? _ellipsis;
  final ui.Locale? _locale;

  static SkTextStyleProperties toSkTextStyleProperties(
    String? fontFamily,
    double? fontSize,
    double? height,
    ui.FontWeight? fontWeight,
    ui.FontStyle? fontStyle,
  ) {
    final SkTextStyleProperties skTextStyle = SkTextStyleProperties();
    if (fontWeight != null || fontStyle != null) {
      skTextStyle.fontStyle = toSkFontStyle(fontWeight, fontStyle);
    }

    if (fontSize != null) {
      skTextStyle.fontSize = fontSize;
    }

    if (height != null) {
      skTextStyle.heightMultiplier = height;
    }

    skTextStyle.fontFamilies = _computeCombinedFontFamilies(fontFamily);

    return skTextStyle;
  }

  static SkStrutStyleProperties toSkStrutStyleProperties(
    ui.StrutStyle value,
    ui.TextHeightBehavior? paragraphHeightBehavior,
  ) {
    final CkStrutStyle style = value as CkStrutStyle;
    final SkStrutStyleProperties skStrutStyle = SkStrutStyleProperties();
    skStrutStyle.fontFamilies = _computeCombinedFontFamilies(
      style._fontFamily,
      style._fontFamilyFallback,
    );

    if (style._fontSize != null) {
      skStrutStyle.fontSize = style._fontSize;
    }

    if (style._height != null) {
      skStrutStyle.heightMultiplier = style._height;
    }

    final ui.TextLeadingDistribution? effectiveLeadingDistribution =
        style._leadingDistribution ?? paragraphHeightBehavior?.leadingDistribution;
    switch (effectiveLeadingDistribution) {
      case null:
        break;
      case ui.TextLeadingDistribution.even:
        skStrutStyle.halfLeading = true;
      case ui.TextLeadingDistribution.proportional:
        skStrutStyle.halfLeading = false;
    }

    if (style._leading != null) {
      skStrutStyle.leading = style._leading;
    }

    if (style._fontWeight != null || style._fontStyle != null) {
      skStrutStyle.fontStyle = toSkFontStyle(style._fontWeight, style._fontStyle);
    }

    if (style._forceStrutHeight != null) {
      skStrutStyle.forceStrutHeight = style._forceStrutHeight;
    }

    skStrutStyle.strutEnabled = true;

    return skStrutStyle;
  }

  static SkParagraphStyle toSkParagraphStyle(
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
  ) {
    final SkParagraphStyleProperties properties = SkParagraphStyleProperties();

    if (textAlign != null) {
      properties.textAlign = toSkTextAlign(textAlign);
    }

    if (textDirection != null) {
      properties.textDirection = toSkTextDirection(textDirection);
    }

    if (maxLines != null) {
      properties.maxLines = maxLines;
    }

    if (height != null) {
      properties.heightMultiplier = height;
    }

    if (textHeightBehavior != null) {
      properties.textHeightBehavior = toSkTextHeightBehavior(textHeightBehavior);
    }

    if (ellipsis != null) {
      properties.ellipsis = ellipsis;
    }

    if (strutStyle != null) {
      properties.strutStyle = toSkStrutStyleProperties(strutStyle, textHeightBehavior);
    }

    properties.replaceTabCharacters = true;
    properties.textStyle = toSkTextStyleProperties(
      fontFamily,
      fontSize,
      height,
      fontWeight,
      fontStyle,
    );
    properties.applyRoundingHack = false;

    return canvasKit.ParagraphStyle(properties);
  }

  CkTextStyle getTextStyle() {
    return CkTextStyle._(
      originalFontFamily: _originalFontFamily,
      effectiveFontFamily: _effectiveFontFamily,
      fontSize: _fontSize,
      height: _height,
      leadingDistribution: _textHeightBehavior?.leadingDistribution,
      fontWeight: _fontWeight,
      fontStyle: _fontStyle,

      // Use defaults for everything else.
      color: null,
      decoration: null,
      decorationColor: null,
      decorationStyle: null,
      decorationThickness: null,
      textBaseline: null,
      originalFontFamilyFallback: null,
      effectiveFontFamilyFallback: null,
      letterSpacing: null,
      wordSpacing: null,
      locale: null,
      background: null,
      foreground: null,
      shadows: null,
      fontFeatures: null,
      fontVariations: null,
    );
  }

  @override
  bool operator ==(Object other) {
    if (identical(this, other)) {
      return true;
    }
    if (other.runtimeType != runtimeType) {
      return false;
    }
    return other is CkParagraphStyle &&
        other._textAlign == _textAlign &&
        other._textDirection == _textDirection &&
        other._fontWeight == _fontWeight &&
        other._fontStyle == _fontStyle &&
        other._maxLines == _maxLines &&
        other._originalFontFamily == _originalFontFamily &&
        // effectiveFontFamily is not compared as it's a computed value.
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
      _originalFontFamily,
      // effectiveFontFamily is not included as it's a computed value.
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
          'fontFamily: ${_originalFontFamily ?? "unspecified"}, '
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

@immutable
class CkTextStyle implements ui.TextStyle {
  factory CkTextStyle({
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
    CkPaint? background,
    CkPaint? foreground,
    List<ui.Shadow>? shadows,
    List<ui.FontFeature>? fontFeatures,
    List<ui.FontVariation>? fontVariations,
  }) {
    assert(
      color == null || foreground == null,
      'Cannot provide both a color and a foreground\n'
      'The color argument is just a shorthand for "foreground: Paint()..color = color".',
    );
    return CkTextStyle._(
      color: color,
      decoration: decoration,
      decorationColor: decorationColor,
      decorationStyle: decorationStyle,
      decorationThickness: decorationThickness,
      fontWeight: fontWeight,
      fontStyle: fontStyle,
      textBaseline: textBaseline,
      originalFontFamily: fontFamily,
      effectiveFontFamily: _computeEffectiveFontFamily(fontFamily),
      originalFontFamilyFallback: fontFamilyFallback,
      effectiveFontFamilyFallback:
          ui_web.debugEmulateFlutterTesterEnvironment ? null : fontFamilyFallback,
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
      fontVariations: fontVariations,
    );
  }

  CkTextStyle._({
    required this.color,
    required this.decoration,
    required this.decorationColor,
    required this.decorationStyle,
    required this.decorationThickness,
    required this.fontWeight,
    required this.fontStyle,
    required this.textBaseline,
    required this.originalFontFamily,
    required this.effectiveFontFamily,
    required this.originalFontFamilyFallback,
    required this.effectiveFontFamilyFallback,
    required this.fontSize,
    required this.letterSpacing,
    required this.wordSpacing,
    required this.height,
    required this.leadingDistribution,
    required this.locale,
    required this.background,
    required this.foreground,
    required this.shadows,
    required this.fontFeatures,
    required this.fontVariations,
  });

  final ui.Color? color;
  final ui.TextDecoration? decoration;
  final ui.Color? decorationColor;
  final ui.TextDecorationStyle? decorationStyle;
  final double? decorationThickness;
  final ui.FontWeight? fontWeight;
  final ui.FontStyle? fontStyle;
  final ui.TextBaseline? textBaseline;
  final String? originalFontFamily;
  final String? effectiveFontFamily;
  final List<String>? originalFontFamilyFallback;
  final List<String>? effectiveFontFamilyFallback;
  final double? fontSize;
  final double? letterSpacing;
  final double? wordSpacing;
  final double? height;
  final ui.TextLeadingDistribution? leadingDistribution;
  final ui.Locale? locale;
  final CkPaint? background;
  final CkPaint? foreground;
  final List<ui.Shadow>? shadows;
  final List<ui.FontFeature>? fontFeatures;
  final List<ui.FontVariation>? fontVariations;

  /// Merges this text style with [other] and returns the new text style.
  ///
  /// The values in this text style are used unless [other] specifically
  /// overrides it.
  CkTextStyle mergeWith(CkTextStyle other) {
    final double? textHeight = other.height == ui.kTextHeightNone ? null : (other.height ?? height);
    return CkTextStyle._(
      color: other.color ?? color,
      decoration: other.decoration ?? decoration,
      decorationColor: other.decorationColor ?? decorationColor,
      decorationStyle: other.decorationStyle ?? decorationStyle,
      decorationThickness: other.decorationThickness ?? decorationThickness,
      fontWeight: other.fontWeight ?? fontWeight,
      fontStyle: other.fontStyle ?? fontStyle,
      textBaseline: other.textBaseline ?? textBaseline,
      originalFontFamily: other.originalFontFamily ?? originalFontFamily,
      effectiveFontFamily: other.effectiveFontFamily ?? effectiveFontFamily,
      originalFontFamilyFallback: other.originalFontFamilyFallback ?? originalFontFamilyFallback,
      effectiveFontFamilyFallback: other.effectiveFontFamilyFallback ?? effectiveFontFamilyFallback,
      fontSize: other.fontSize ?? fontSize,
      letterSpacing: other.letterSpacing ?? letterSpacing,
      wordSpacing: other.wordSpacing ?? wordSpacing,
      height: textHeight,
      leadingDistribution: other.leadingDistribution ?? leadingDistribution,
      locale: other.locale ?? locale,
      background: other.background ?? background,
      foreground: other.foreground ?? foreground,
      shadows: other.shadows ?? shadows,
      fontFeatures: other.fontFeatures ?? fontFeatures,
      fontVariations: other.fontVariations ?? fontVariations,
    );
  }

  /// Lazy-initialized combination of font family and font family fallback sent to Skia.
  late final List<String> combinedFontFamilies = _computeCombinedFontFamilies(
    effectiveFontFamily,
    effectiveFontFamilyFallback,
  );

  /// Lazy-initialized Skia style used to pass the style to Skia.
  ///
  /// This is lazy because not every style ends up being passed to Skia, so the
  /// conversion would be wasteful.
  late final SkTextStyle skTextStyle = () {
    // Write field values to locals so null checks promote types to non-null.
    final ui.Color? color = this.color;
    final ui.TextDecoration? decoration = this.decoration;
    final ui.Color? decorationColor = this.decorationColor;
    final ui.TextDecorationStyle? decorationStyle = this.decorationStyle;
    final double? decorationThickness = this.decorationThickness;
    final ui.FontWeight? fontWeight = this.fontWeight;
    final ui.FontStyle? fontStyle = this.fontStyle;
    final ui.TextBaseline? textBaseline = this.textBaseline;
    final double? fontSize = this.fontSize;
    final double? letterSpacing = this.letterSpacing;
    final double? wordSpacing = this.wordSpacing;
    final double? height = this.height;
    final ui.Locale? locale = this.locale;
    final CkPaint? background = this.background;
    final CkPaint? foreground = this.foreground;
    final List<ui.Shadow>? shadows = this.shadows;
    final List<ui.FontFeature>? fontFeatures = this.fontFeatures;
    final List<ui.FontVariation>? fontVariations = this.fontVariations;

    final SkTextStyleProperties properties = SkTextStyleProperties();

    if (background != null) {
      properties.backgroundColor = makeFreshSkColor(background.color);
    }

    if (color != null) {
      properties.color = makeFreshSkColor(color);
    }

    if (decoration != null) {
      int decorationValue = canvasKit.NoDecoration.toInt();
      if (decoration.contains(ui.TextDecoration.underline)) {
        decorationValue |= canvasKit.UnderlineDecoration.toInt();
      }
      if (decoration.contains(ui.TextDecoration.overline)) {
        decorationValue |= canvasKit.OverlineDecoration.toInt();
      }
      if (decoration.contains(ui.TextDecoration.lineThrough)) {
        decorationValue |= canvasKit.LineThroughDecoration.toInt();
      }
      properties.decoration = decorationValue;
    }

    if (decorationThickness != null) {
      properties.decorationThickness = decorationThickness;
    }

    if (decorationColor != null) {
      properties.decorationColor = makeFreshSkColor(decorationColor);
    }

    if (decorationStyle != null) {
      properties.decorationStyle = toSkTextDecorationStyle(decorationStyle);
    }

    if (textBaseline != null) {
      properties.textBaseline = toSkTextBaseline(textBaseline);
    }

    if (fontSize != null) {
      properties.fontSize = fontSize;
    }

    if (letterSpacing != null) {
      properties.letterSpacing = letterSpacing;
    }

    if (wordSpacing != null) {
      properties.wordSpacing = wordSpacing;
    }

    if (height != null) {
      properties.heightMultiplier = height;
    }

    switch (leadingDistribution) {
      case null:
        break;
      case ui.TextLeadingDistribution.even:
        properties.halfLeading = true;
      case ui.TextLeadingDistribution.proportional:
        properties.halfLeading = false;
    }

    if (locale != null) {
      properties.locale = locale.toLanguageTag();
    }

    properties.fontFamilies = combinedFontFamilies;

    if (fontWeight != null || fontStyle != null) {
      properties.fontStyle = toSkFontStyle(fontWeight, fontStyle);
    }

    if (foreground != null) {
      properties.foregroundColor = makeFreshSkColor(foreground.color);
    }

    if (shadows != null) {
      final List<SkTextShadow> ckShadows = <SkTextShadow>[];
      for (final ui.Shadow shadow in shadows) {
        final SkTextShadow ckShadow = SkTextShadow();
        ckShadow.color = makeFreshSkColor(shadow.color);
        ckShadow.offset = toSkPoint(shadow.offset);
        ckShadow.blurRadius = shadow.blurRadius;
        ckShadows.add(ckShadow);
      }
      properties.shadows = ckShadows;
    }

    if (fontFeatures != null) {
      final List<SkFontFeature> skFontFeatures = <SkFontFeature>[];
      for (final ui.FontFeature fontFeature in fontFeatures) {
        final SkFontFeature skFontFeature = SkFontFeature();
        skFontFeature.name = fontFeature.feature;
        skFontFeature.value = fontFeature.value;
        skFontFeatures.add(skFontFeature);
      }
      properties.fontFeatures = skFontFeatures;
    }

    if (fontVariations != null) {
      final List<SkFontVariation> skFontVariations = <SkFontVariation>[];
      for (final ui.FontVariation fontVariation in fontVariations) {
        final SkFontVariation skFontVariation = SkFontVariation();
        skFontVariation.axis = fontVariation.axis;
        skFontVariation.value = fontVariation.value;
        skFontVariations.add(skFontVariation);
      }
      properties.fontVariations = skFontVariations;
    }

    return canvasKit.TextStyle(properties);
  }();

  @override
  bool operator ==(Object other) {
    if (identical(this, other)) {
      return true;
    }
    return other is CkTextStyle &&
        other.color == color &&
        other.decoration == decoration &&
        other.decorationColor == decorationColor &&
        other.decorationStyle == decorationStyle &&
        other.fontWeight == fontWeight &&
        other.fontStyle == fontStyle &&
        other.textBaseline == textBaseline &&
        other.leadingDistribution == leadingDistribution &&
        other.originalFontFamily == originalFontFamily &&
        other.fontSize == fontSize &&
        other.letterSpacing == letterSpacing &&
        other.wordSpacing == wordSpacing &&
        other.height == height &&
        other.decorationThickness == decorationThickness &&
        other.locale == locale &&
        other.background == background &&
        other.foreground == foreground &&
        listEquals<ui.Shadow>(other.shadows, shadows) &&
        listEquals<String>(other.originalFontFamilyFallback, originalFontFamilyFallback) &&
        listEquals<ui.FontFeature>(other.fontFeatures, fontFeatures) &&
        listEquals<ui.FontVariation>(other.fontVariations, fontVariations);
  }

  @override
  int get hashCode {
    final List<ui.Shadow>? shadows = this.shadows;
    final List<ui.FontFeature>? fontFeatures = this.fontFeatures;
    final List<ui.FontVariation>? fontVariations = this.fontVariations;
    final List<String>? fontFamilyFallback = originalFontFamilyFallback;
    return Object.hash(
      color,
      decoration,
      decorationColor,
      decorationStyle,
      fontWeight,
      fontStyle,
      textBaseline,
      leadingDistribution,
      originalFontFamily,
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
      final List<String>? fontFamilyFallback = originalFontFamilyFallback;
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
          'fontFamily: ${originalFontFamily ?? "unspecified"}, '
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

class CkStrutStyle implements ui.StrutStyle {
  CkStrutStyle({
    String? fontFamily,
    List<String>? fontFamilyFallback,
    double? fontSize,
    double? height,
    // TODO(mdebbar): implement leadingDistribution.
    ui.TextLeadingDistribution? leadingDistribution,
    double? leading,
    ui.FontWeight? fontWeight,
    ui.FontStyle? fontStyle,
    bool? forceStrutHeight,
  }) : _fontFamily = _computeEffectiveFontFamily(fontFamily),
       _fontFamilyFallback =
           ui_web.debugEmulateFlutterTesterEnvironment ? null : fontFamilyFallback,
       _fontSize = fontSize,
       _height = height == ui.kTextHeightNone ? null : height,
       _leadingDistribution = leadingDistribution,
       _leading = leading,
       _fontWeight = fontWeight,
       _fontStyle = fontStyle,
       _forceStrutHeight = forceStrutHeight;

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
    if (other.runtimeType != runtimeType) {
      return false;
    }
    return other is CkStrutStyle &&
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
}

SkFontStyle toSkFontStyle(ui.FontWeight? fontWeight, ui.FontStyle? fontStyle) {
  final SkFontStyle style = SkFontStyle();
  if (fontWeight != null) {
    style.weight = toSkFontWeight(fontWeight);
  }
  if (fontStyle != null) {
    style.slant = toSkFontSlant(fontStyle);
  }
  return style;
}

/// The CanvasKit implementation of [ui.Paragraph].
class CkParagraph implements ui.Paragraph {
  CkParagraph(SkParagraph skParagraph, this._paragraphStyle) {
    _ref = UniqueRef<SkParagraph>(this, skParagraph, 'Paragraph');
  }

  late final UniqueRef<SkParagraph> _ref;

  SkParagraph get skiaObject => _ref.nativeObject;

  /// The constraints from the last time we laid the paragraph out.
  ///
  /// This is used to resurrect the paragraph if the initial paragraph
  /// is deleted.
  double _lastLayoutConstraints = double.negativeInfinity;

  /// The paragraph style used to build this paragraph.
  ///
  /// This is used to resurrect the paragraph if the initial paragraph
  /// is deleted.
  final CkParagraphStyle _paragraphStyle;

  @override
  double get alphabeticBaseline => _alphabeticBaseline;
  double _alphabeticBaseline = 0;

  @override
  bool get didExceedMaxLines => _didExceedMaxLines;
  bool _didExceedMaxLines = false;

  @override
  double get height => _height;
  double _height = 0;

  @override
  double get ideographicBaseline => _ideographicBaseline;
  double _ideographicBaseline = 0;

  @override
  double get longestLine => _longestLine;
  double _longestLine = 0;

  @override
  double get maxIntrinsicWidth => _maxIntrinsicWidth;
  double _maxIntrinsicWidth = 0;

  @override
  double get minIntrinsicWidth => _minIntrinsicWidth;
  double _minIntrinsicWidth = 0;

  @override
  double get width => _width;
  double _width = 0;

  @override
  List<ui.TextBox> getBoxesForPlaceholders() => _boxesForPlaceholders;
  late List<ui.TextBox> _boxesForPlaceholders;

  @override
  List<ui.TextBox> getBoxesForRange(
    int start,
    int end, {
    ui.BoxHeightStyle boxHeightStyle = ui.BoxHeightStyle.tight,
    ui.BoxWidthStyle boxWidthStyle = ui.BoxWidthStyle.tight,
  }) {
    assert(!_disposed, 'Paragraph has been disposed.');
    if (start < 0 || end < 0) {
      return const <ui.TextBox>[];
    }

    final List<SkRectWithDirection> skRects = skiaObject.getRectsForRange(
      start.toDouble(),
      end.toDouble(),
      toSkRectHeightStyle(boxHeightStyle),
      toSkRectWidthStyle(boxWidthStyle),
    );

    return skRectsToTextBoxes(skRects);
  }

  List<ui.TextBox> skRectsToTextBoxes(List<SkRectWithDirection> skRects) {
    assert(!_disposed, 'Paragraph has been disposed.');
    final List<ui.TextBox> result = <ui.TextBox>[];

    for (int i = 0; i < skRects.length; i++) {
      final SkRectWithDirection skRect = skRects[i];
      final Float32List rect = skRect.rect;
      final int skTextDirection = skRect.dir.value.toInt();
      result.add(
        ui.TextBox.fromLTRBD(
          rect[0],
          rect[1],
          rect[2],
          rect[3],
          ui.TextDirection.values[skTextDirection],
        ),
      );
    }

    return result;
  }

  @override
  ui.TextPosition getPositionForOffset(ui.Offset offset) {
    assert(!_disposed, 'Paragraph has been disposed.');
    final SkTextPosition positionWithAffinity = skiaObject.getGlyphPositionAtCoordinate(
      offset.dx,
      offset.dy,
    );
    return fromPositionWithAffinity(positionWithAffinity);
  }

  @override
  ui.GlyphInfo? getClosestGlyphInfoForOffset(ui.Offset offset) {
    assert(!_disposed, 'Paragraph has been disposed.');
    return skiaObject.getClosestGlyphInfoAt(offset.dx, offset.dy);
  }

  @override
  ui.GlyphInfo? getGlyphInfoAt(int codeUnitOffset) {
    assert(!_disposed, 'Paragraph has been disposed.');
    return skiaObject.getGlyphInfoAt(codeUnitOffset.toDouble());
  }

  @override
  ui.TextRange getWordBoundary(ui.TextPosition position) {
    assert(!_disposed, 'Paragraph has been disposed.');
    final int characterPosition;
    switch (position.affinity) {
      case ui.TextAffinity.upstream:
        characterPosition = position.offset - 1;
      case ui.TextAffinity.downstream:
        characterPosition = position.offset;
    }
    final SkTextRange skRange = skiaObject.getWordBoundary(characterPosition.toDouble());
    return ui.TextRange(start: skRange.start.toInt(), end: skRange.end.toInt());
  }

  @override
  void layout(ui.ParagraphConstraints constraints) {
    assert(!_disposed, 'Paragraph has been disposed.');
    if (_lastLayoutConstraints == constraints.width) {
      return;
    }

    _lastLayoutConstraints = constraints.width;

    // TODO(het): CanvasKit throws an exception when laid out with
    // a font that wasn't registered.
    try {
      final SkParagraph paragraph = skiaObject;
      paragraph.layout(constraints.width);
      _alphabeticBaseline = paragraph.getAlphabeticBaseline();
      _didExceedMaxLines = paragraph.didExceedMaxLines();
      _height = paragraph.getHeight();
      _ideographicBaseline = paragraph.getIdeographicBaseline();
      _longestLine = paragraph.getLongestLine();
      _maxIntrinsicWidth = paragraph.getMaxIntrinsicWidth();
      _minIntrinsicWidth = paragraph.getMinIntrinsicWidth();
      _width = paragraph.getMaxWidth();
      _boxesForPlaceholders = skRectsToTextBoxes(paragraph.getRectsForPlaceholders());
    } catch (e) {
      printWarning(
        'CanvasKit threw an exception while laying '
        'out the paragraph. The font was "${_paragraphStyle._originalFontFamily}". '
        'Exception:\n$e',
      );
      rethrow;
    }
  }

  @override
  ui.TextRange getLineBoundary(ui.TextPosition position) {
    assert(!_disposed, 'Paragraph has been disposed.');
    final List<SkLineMetrics> metrics = skiaObject.getLineMetrics();
    final int offset = position.offset;
    for (final SkLineMetrics metric in metrics) {
      if (offset >= metric.startIndex && offset <= metric.endIndex) {
        return ui.TextRange(start: metric.startIndex.toInt(), end: metric.endIndex.toInt());
      }
    }
    return ui.TextRange.empty;
  }

  @override
  List<ui.LineMetrics> computeLineMetrics() {
    assert(!_disposed, 'Paragraph has been disposed.');
    final List<SkLineMetrics> skLineMetrics = skiaObject.getLineMetrics();
    final List<ui.LineMetrics> result = <ui.LineMetrics>[];
    for (final SkLineMetrics metric in skLineMetrics) {
      result.add(CkLineMetrics._(metric));
    }
    return result;
  }

  @override
  ui.LineMetrics? getLineMetricsAt(int lineNumber) {
    assert(!_disposed, 'Paragraph has been disposed.');
    final SkLineMetrics? metrics = skiaObject.getLineMetricsAt(lineNumber.toDouble());
    return metrics == null ? null : CkLineMetrics._(metrics);
  }

  @override
  int get numberOfLines {
    assert(!_disposed, 'Paragraph has been disposed.');
    return skiaObject.getNumberOfLines().toInt();
  }

  @override
  int? getLineNumberAt(int codeUnitOffset) {
    assert(!_disposed, 'Paragraph has been disposed.');
    final int lineNumber = skiaObject.getLineNumberAt(codeUnitOffset.toDouble()).toInt();
    return lineNumber >= 0 ? lineNumber : null;
  }

  bool _disposed = false;

  @override
  void dispose() {
    assert(!_disposed, 'Paragraph has been disposed.');
    _ref.dispose();
    _disposed = true;
  }

  @override
  bool get debugDisposed {
    bool? result;
    assert(() {
      result = _disposed;
      return true;
    }());

    if (result != null) {
      return result!;
    }

    throw StateError('Paragraph.debugDisposed is only available when asserts are enabled.');
  }
}

class CkLineMetrics implements ui.LineMetrics {
  CkLineMetrics._(this.skLineMetrics);

  final SkLineMetrics skLineMetrics;

  @override
  double get ascent => skLineMetrics.ascent;

  @override
  double get descent => skLineMetrics.descent;

  // TODO(hterkelsen): Implement this correctly once SkParagraph does.
  @override
  double get unscaledAscent => skLineMetrics.ascent;

  @override
  bool get hardBreak => skLineMetrics.isHardBreak;

  @override
  double get baseline => skLineMetrics.baseline;

  @override
  double get height => (skLineMetrics.ascent + skLineMetrics.descent).round().toDouble();

  @override
  double get left => skLineMetrics.left;

  @override
  double get width => skLineMetrics.width;

  @override
  int get lineNumber => skLineMetrics.lineNumber.toInt();
}

class CkParagraphBuilder implements ui.ParagraphBuilder {
  CkParagraphBuilder(ui.ParagraphStyle style)
    : _style = style as CkParagraphStyle,
      _placeholderCount = 0,
      _placeholderScales = <double>[],
      _styleStack = <CkTextStyle>[],
      _paragraphBuilder = canvasKit.ParagraphBuilder.MakeFromFontCollection(
        style.skParagraphStyle,
        CanvasKitRenderer.instance.fontCollection.skFontCollection,
      ) {
    _styleStack.add(_style.getTextStyle());
  }

  final SkParagraphBuilder _paragraphBuilder;
  final CkParagraphStyle _style;
  int _placeholderCount;
  final List<double> _placeholderScales;
  final List<CkTextStyle> _styleStack;

  @override
  void addPlaceholder(
    double width,
    double height,
    ui.PlaceholderAlignment alignment, {
    double scale = 1.0,
    double? baselineOffset,
    ui.TextBaseline? baseline,
  }) {
    // Require a baseline to be specified if using a baseline-based alignment.
    assert(
      !(alignment == ui.PlaceholderAlignment.aboveBaseline ||
              alignment == ui.PlaceholderAlignment.belowBaseline ||
              alignment == ui.PlaceholderAlignment.baseline) ||
          baseline != null,
    );

    _placeholderCount++;
    _placeholderScales.add(scale);
    final _CkParagraphPlaceholder placeholderStyle = toSkPlaceholderStyle(
      width * scale,
      height * scale,
      alignment,
      (baselineOffset ?? height) * scale,
      baseline ?? ui.TextBaseline.alphabetic,
    );
    _addPlaceholder(placeholderStyle);
  }

  void _addPlaceholder(_CkParagraphPlaceholder placeholderStyle) {
    _paragraphBuilder.addPlaceholder(
      placeholderStyle.width,
      placeholderStyle.height,
      placeholderStyle.alignment,
      placeholderStyle.baseline,
      placeholderStyle.offset,
    );
  }

  static _CkParagraphPlaceholder toSkPlaceholderStyle(
    double width,
    double height,
    ui.PlaceholderAlignment alignment,
    double baselineOffset,
    ui.TextBaseline baseline,
  ) {
    final _CkParagraphPlaceholder properties = _CkParagraphPlaceholder(
      width: width,
      height: height,
      alignment: toSkPlaceholderAlignment(alignment),
      offset: baselineOffset,
      baseline: toSkTextBaseline(baseline),
    );
    return properties;
  }

  @override
  void addText(String text) {
    final List<String> fontFamilies = <String>[];
    final CkTextStyle style = _peekStyle();
    if (style.effectiveFontFamily != null) {
      fontFamilies.add(style.effectiveFontFamily!);
    }
    if (style.effectiveFontFamilyFallback != null) {
      fontFamilies.addAll(style.effectiveFontFamilyFallback!);
    }
    renderer.fontCollection.fontFallbackManager!.ensureFontsSupportText(text, fontFamilies);
    _paragraphBuilder.addText(text);
  }

  @override
  CkParagraph build() {
    final SkParagraph builtParagraph = _buildSkParagraph();
    return CkParagraph(builtParagraph, _style);
  }

  /// Builds the CkParagraph with the builder and deletes the builder.
  SkParagraph _buildSkParagraph() {
    if (_ckRequiresClientICU) {
      injectClientICU(_paragraphBuilder);
    }
    final SkParagraph result = _paragraphBuilder.build();
    _paragraphBuilder.delete();
    return result;
  }

  @override
  int get placeholderCount => _placeholderCount;

  @override
  List<double> get placeholderScales => _placeholderScales;

  @override
  void pop() {
    if (_styleStack.length <= 1) {
      // The top-level text style is paragraph-level. We don't pop it off.
      assert(() {
        printWarning(
          'Cannot pop text style in ParagraphBuilder. '
          'Already popped all text styles from the style stack.',
        );
        return true;
      }());
      return;
    }
    _styleStack.removeLast();
    _paragraphBuilder.pop();
  }

  CkTextStyle _peekStyle() {
    assert(_styleStack.isNotEmpty);
    return _styleStack.last;
  }

  static SkPaint createForegroundPaint(CkTextStyle style) {
    final SkPaint foreground;
    if (style.foreground != null) {
      foreground = style.foreground!.toSkPaint();
    } else {
      foreground = SkPaint();
      foreground.setColorInt(style.color?.value ?? 0xFF000000);
    }
    return foreground;
  }

  static SkPaint createBackgroundPaint(CkTextStyle style) {
    final SkPaint background;
    if (style.background != null) {
      background = style.background!.toSkPaint();
    } else {
      background = SkPaint()..setColorInt(0x00000000);
    }
    return background;
  }

  @override
  void pushStyle(ui.TextStyle leafStyle) {
    leafStyle as CkTextStyle;

    final CkTextStyle baseStyle = _peekStyle();
    final CkTextStyle mergedStyle = baseStyle.mergeWith(leafStyle);
    _styleStack.add(mergedStyle);

    if (mergedStyle.foreground != null || mergedStyle.background != null) {
      final foreground = createForegroundPaint(mergedStyle);
      final background = createBackgroundPaint(mergedStyle);
      _paragraphBuilder.pushPaintStyle(mergedStyle.skTextStyle, foreground, background);
      foreground.delete();
      background.delete();
    } else {
      _paragraphBuilder.pushStyle(mergedStyle.skTextStyle);
    }
  }
}

class _CkParagraphPlaceholder {
  _CkParagraphPlaceholder({
    required this.width,
    required this.height,
    required this.alignment,
    required this.baseline,
    required this.offset,
  });

  final double width;
  final double height;
  final SkPlaceholderAlignment alignment;
  final SkTextBaseline baseline;
  final double offset;
}

List<String> _computeCombinedFontFamilies(String? fontFamily, [List<String>? fontFamilyFallback]) {
  final List<String> fontFamilies = <String>[];
  if (fontFamily != null) {
    fontFamilies.add(fontFamily);
  }
  if (fontFamilyFallback != null &&
      !fontFamilyFallback.every((String font) => fontFamily == font)) {
    fontFamilies.addAll(fontFamilyFallback);
  }
  fontFamilies.addAll(renderer.fontCollection.fontFallbackManager!.globalFontFallbacks);
  return fontFamilies;
}
