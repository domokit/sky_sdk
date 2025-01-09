// Copyright 2013 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

import 'package:ui/ui.dart' as ui;
import 'package:ui/ui_web/src/ui_web.dart' as ui_web;

import '../dom.dart';
import '../util.dart';
import '../view_embedder/style_manager.dart';
import 'measurement.dart';
import 'paragraph.dart';

String buildCssFontString({
  required ui.FontStyle? fontStyle,
  required ui.FontWeight? fontWeight,
  required double? fontSize,
  required String fontFamily,
}) {
  final String cssFontStyle = fontStyle?.toCssString() ?? StyleManager.defaultFontStyle;
  final String cssFontWeight = fontWeight?.toCssString() ?? StyleManager.defaultFontWeight;
  final int cssFontSize = (fontSize ?? StyleManager.defaultFontSize).floor();
  final String cssFontFamily = canonicalizeFontFamily(fontFamily)!;

  return '$cssFontStyle $cssFontWeight ${cssFontSize}px $cssFontFamily';
}

/// Contains all styles that have an effect on the height of text.
///
/// This is useful as a cache key for [TextHeightRuler].
class TextHeightStyle {
  TextHeightStyle({
    required this.fontFamily,
    required this.fontSize,
    required this.height,
    required this.fontFeatures,
    required this.fontVariations,
  });

  final String fontFamily;
  final double fontSize;
  final double? height;
  final List<ui.FontFeature>? fontFeatures;
  final List<ui.FontVariation>? fontVariations;

  @override
  bool operator ==(Object other) {
    if (identical(this, other)) {
      return true;
    }
    return other is TextHeightStyle && other.hashCode == hashCode;
  }

  @override
  late final int hashCode = Object.hash(
    fontFamily,
    fontSize,
    height,
    fontFeatures == null ? null : Object.hashAll(fontFeatures!),
    fontVariations == null ? null : Object.hashAll(fontVariations!),
  );
}

/// Provides text dimensions found on [_element]. The idea behind this class is
/// to allow the [ParagraphRuler] to mutate multiple dom elements and allow
/// consumers to lazily read the measurements.
///
/// The [ParagraphRuler] would have multiple instances of [TextDimensions] with
/// different backing elements for different types of measurements. When a
/// measurement is needed, the [ParagraphRuler] would mutate all the backing
/// elements at once. The consumer of the ruler can later read those
/// measurements.
///
/// The rationale behind this is to minimize browser reflows by batching dom
/// writes first, then performing all the reads.
class TextDimensions {
  TextDimensions(this._element);

  final DomElement _element;
  DomRect? _cachedBoundingClientRect;

  void _invalidateBoundsCache() {
    _cachedBoundingClientRect = null;
  }

  /// Sets text of contents to a single space character to measure empty text.
  void updateTextToSpace() {
    _invalidateBoundsCache();
    _element.text = ' ';
  }

  void applyHeightStyle(TextHeightStyle textHeightStyle) {
    final String fontFamily = textHeightStyle.fontFamily;
    final double fontSize = textHeightStyle.fontSize;
    final DomCSSStyleDeclaration style = _element.style;
    style
      ..fontSize = '${fontSize.floor()}px'
      ..fontFamily = canonicalizeFontFamily(fontFamily)!;

    final double? height = textHeightStyle.height;
    // Workaround the rounding introduced by https://github.com/flutter/flutter/issues/122066
    // in tests.
    final double? effectiveLineHeight = height ?? (fontFamily == 'FlutterTest' ? 1.0 : null);
    if (effectiveLineHeight != null) {
      style.lineHeight = effectiveLineHeight.toString();
    }
    _invalidateBoundsCache();
  }

  /// Appends element and probe to hostElement that is set up for a specific
  /// TextStyle.
  void appendToHost(DomHTMLElement hostElement) {
    hostElement.append(_element);
    _invalidateBoundsCache();
  }

  DomRect _readAndCacheMetrics() => _cachedBoundingClientRect ??= _element.getBoundingClientRect();

  /// The height of the paragraph being measured.
  double get height {
    double cachedHeight = _readAndCacheMetrics().height;
    if (ui_web.browser.browserEngine == ui_web.BrowserEngine.firefox &&
        // In the flutter tester environment, we use a predictable-size for font
        // measurement tests.
        !ui_web.debugEmulateFlutterTesterEnvironment) {
      // See subpixel rounding bug :
      // https://bugzilla.mozilla.org/show_bug.cgi?id=442139
      // This causes bottom of letters such as 'y' to be cutoff and
      // incorrect rendering of double underlines.
      cachedHeight += 1.0;
    }
    return cachedHeight;
  }
}

/// Performs height measurement for the given [textHeightStyle].
///
/// The two results of this ruler's measurement are:
///
/// 1. [alphabeticBaseline].
/// 2. [height].
class TextHeightRuler {
  TextHeightRuler(this.textHeightStyle, this.rulerHost);

  final TextHeightStyle textHeightStyle;
  final RulerHost rulerHost;

  // Elements used to measure the line-height metric.
  late final DomHTMLElement _probe = _createProbe();
  late final DomHTMLElement _host = _createHost();
  final TextDimensions _dimensions = TextDimensions(domDocument.createElement('flt-paragraph'));

  /// The alphabetic baseline for this ruler's [textHeightStyle].
  late final double alphabeticBaseline = _probe.getBoundingClientRect().bottom;

  /// The height for this ruler's [textHeightStyle].
  late final double height = _dimensions.height;

  /// Disposes of this ruler and detaches it from the DOM tree.
  void dispose() {
    _host.remove();
  }

  DomHTMLElement _createHost() {
    final DomHTMLDivElement host = createDomHTMLDivElement();
    host.style
      ..visibility = 'hidden'
      ..position = 'absolute'
      ..top = '0'
      ..left = '0'
      ..display = 'flex'
      ..flexDirection = 'row'
      ..alignItems = 'baseline'
      ..margin = '0'
      ..border = '0'
      ..padding = '0';

    assert(() {
      host.setAttribute('data-ruler', 'line-height');
      return true;
    }());

    _dimensions.applyHeightStyle(textHeightStyle);

    // Force single-line (even if wider than screen) and preserve whitespaces.
    _dimensions._element.style.whiteSpace = 'pre';

    // To measure line-height, all we need is a whitespace.
    _dimensions.updateTextToSpace();

    _dimensions.appendToHost(host);

    rulerHost.addElement(host);
    return host;
  }

  DomHTMLElement _createProbe() {
    final DomHTMLElement probe = createDomHTMLDivElement();
    _host.append(probe);
    return probe;
  }
}
