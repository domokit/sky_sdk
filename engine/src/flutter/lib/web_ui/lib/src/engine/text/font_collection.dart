// Copyright 2013 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

import 'dart:async';
import 'dart:typed_data';

import 'package:ui/src/engine.dart';
import 'package:ui/ui_web/src/ui_web.dart' as ui_web;

/// This class is responsible for registering and loading fonts.
///
/// Once an asset manager has been set in the framework, call
/// [downloadAssetFonts] with it to register fonts declared in the
/// font manifest. If test fonts are enabled, then call
/// [debugDownloadTestFonts] as well.
class HtmlFontCollection implements FlutterFontCollection {
  /// Reads the font manifest using the [ui_web.assetManager] and downloads all of the
  /// fonts declared within.
  @override
  Future<AssetFontsResult> loadAssetFonts(FontManifest manifest) async {
    final List<Future<(String, FontLoadError?)>> pendingFonts =
        <Future<(String, FontLoadError?)>>[];
    for (final FontFamily family in manifest.families) {
      for (final FontAsset fontAsset in family.fontAssets) {
        pendingFonts.add(() async {
          return (
            fontAsset.asset,
            await _loadFontAsset(family.name, fontAsset.asset, fontAsset.descriptors),
          );
        }());
      }
    }

    final List<String> loadedFonts = <String>[];
    final Map<String, FontLoadError> fontFailures = <String, FontLoadError>{};
    for (final (String asset, FontLoadError? error) in await Future.wait(pendingFonts)) {
      if (error == null) {
        loadedFonts.add(asset);
      } else {
        fontFailures[asset] = error;
      }
    }
    return AssetFontsResult(loadedFonts, fontFailures);
  }

  @override
  Future<bool> loadFontFromList(Uint8List list, {String? fontFamily}) async {
    if (fontFamily == null) {
      printWarning('Font family must be provided to HtmlFontCollection.');
      return false;
    }
    return _loadFontFaceBytes(fontFamily, list);
  }

  @override
  Null get fontFallbackManager => null;

  /// Unregister all fonts that have been registered.
  @override
  void clear() {
    domDocument.fonts!.clear();
  }

  // Regular expression to detect a string with no punctuations.
  // For example font family 'Ahem!' does not fall into this category
  // so the family name will be wrapped in quotes.
  static final RegExp notPunctuation = RegExp(r'[a-z0-9\s]+', caseSensitive: false);
  // Regular expression to detect tokens starting with a digit.
  // For example font family 'Goudy Bookletter 1911' falls into this
  // category.
  static final RegExp startWithDigit = RegExp(r'\b\d');

  /// Registers assets to Flutter Web Engine.
  ///
  /// Browsers and browsers versions differ significantly on how a valid font
  /// family name should be formatted. Notable issues are:
  ///
  /// Safari 12 and Firefox crash if you create a [DomFontFace] with a font
  /// family that is not correct CSS syntax. Font family names with invalid
  /// characters are accepted on these browsers, when wrapped it in
  /// quotes.
  ///
  /// Additionally, for Safari 12 to work [DomFontFace] name should be
  /// loaded correctly on the first try.
  ///
  /// A font in Chrome is not usable other than inside a '<p>' tag, if a
  /// [DomFontFace] is loaded wrapped with quotes. Unlike Safari 12 if a
  /// valid version of the font is also loaded afterwards it will show
  /// that font normally.
  ///
  /// In Safari 13 the [DomFontFace] should be loaded with unquoted family
  /// names.
  ///
  /// In order to avoid all these browser compatibility issues this method:
  /// * Detects the family names that might cause a conflict.
  /// * Loads it with the quotes.
  /// * Loads it again without the quotes.
  /// * For all the other family names [DomFontFace] is loaded only once.
  ///
  /// See also:
  ///
  /// * https://developer.mozilla.org/en-US/docs/Web/CSS/font-family#Valid_family_names
  /// * https://drafts.csswg.org/css-fonts-3/#font-family-prop
  Future<FontLoadError?> _loadFontAsset(
    String family,
    String asset,
    Map<String, String> descriptors,
  ) async {
    final List<DomFontFace> fontFaces = <DomFontFace>[];
    final List<FontLoadError> errors = <FontLoadError>[];
    try {
      if (startWithDigit.hasMatch(family) || notPunctuation.stringMatch(family) != family) {
        // Load a font family name with special characters once here wrapped in
        // quotes.
        fontFaces.add(await _loadFontFace("'$family'", asset, descriptors));
      }
    } on FontLoadError catch (error) {
      errors.add(error);
    }
    try {
      // Load all fonts, without quoted family names.
      fontFaces.add(await _loadFontFace(family, asset, descriptors));
    } on FontLoadError catch (error) {
      errors.add(error);
    }
    if (fontFaces.isEmpty) {
      // We failed to load either font face. Return the first error.
      return errors.first;
    }

    try {
      // Since we can't use tear-offs for interop members, this code is faster
      // and easier to read with a for loop instead of forEach.
      // ignore: prefer_foreach
      for (final DomFontFace font in fontFaces) {
        domDocument.fonts!.add(font);
      }
    } catch (e) {
      return FontInvalidDataError(asset);
    }
    return null;
  }

  Future<DomFontFace> _loadFontFace(
    String family,
    String asset,
    Map<String, String> descriptors,
  ) async {
    // try/catch because `new FontFace` can crash with an improper font family.
    try {
      final DomFontFace fontFace = createDomFontFace(
        family,
        'url(${ui_web.assetManager.getAssetUrl(asset)})',
        descriptors,
      );
      return await fontFace.load();
    } catch (e) {
      printWarning('Error while loading font family "$family":\n$e');
      throw FontDownloadError(asset, e);
    }
  }

  // Loads a font from bytes, surfacing errors through the future.
  Future<bool> _loadFontFaceBytes(String family, Uint8List list) async {
    // Since these fonts are loaded by user code, surface the error
    // through the returned future.
    try {
      final DomFontFace fontFace = createDomFontFace(family, list);
      if (fontFace.status == 'error') {
        // Font failed to load.
        return false;
      }
      domDocument.fonts!.add(fontFace);

      // There might be paragraph measurements for this new font before it is
      // loaded. They were measured using fallback font, so we should clear the
      // cache.
      Spanometer.clearRulersCache();
    } catch (exception) {
      // Failures here will throw an DomException. Return false.
      return false;
    }
    return true;
  }

  @override
  void debugResetFallbackFonts() {}
}
