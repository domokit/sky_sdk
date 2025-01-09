// Copyright 2013 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

import 'dart:async';
import 'dart:typed_data';

import 'package:ui/src/engine.dart';
import 'package:ui/ui_web/src/ui_web.dart' as ui_web;

// This URL was found by using the Google Fonts Developer API to find the URL
// for Roboto. The API warns that this URL is not stable. In order to update
// this, list out all of the fonts and find the URL for the regular
// Roboto font. The API reference is here:
// https://developers.google.com/fonts/docs/developer_api
String _robotoUrl =
    '${configuration.fontFallbackBaseUrl}roboto/v32/KFOmCnqEu92Fr1Me4GZLCzYlKw.woff2';

/// Manages the fonts used in the Skia-based backend.
class SkiaFontCollection implements FlutterFontCollection {
  final Set<String> _downloadedFontFamilies = <String>{};

  @override
  late FontFallbackManager fontFallbackManager = FontFallbackManager(SkiaFallbackRegistry(this));

  /// Fonts that started the download process, but are not yet registered.
  ///
  /// /// Once downloaded successfully, this map is cleared and the resulting
  /// [UnregisteredFont]s are added to [_registeredFonts].
  final List<UnregisteredFont> _unregisteredFonts = <UnregisteredFont>[];

  final List<RegisteredFont> _registeredFonts = <RegisteredFont>[];
  final List<RegisteredFont> registeredFallbackFonts = <RegisteredFont>[];

  /// Returns fonts that have been downloaded, registered, and parsed.
  ///
  /// This should only be used in tests.
  List<RegisteredFont>? get debugRegisteredFonts {
    List<RegisteredFont>? result;
    assert(() {
      result = _registeredFonts;
      return true;
    }());
    return result;
  }

  final Map<String, List<SkFont>> familyToFontMap = <String, List<SkFont>>{};

  void _registerWithFontProvider() {
    if (_fontProvider != null) {
      _fontProvider!.delete();
      _fontProvider = null;
      skFontCollection?.delete();
      skFontCollection = null;
    }
    _fontProvider = canvasKit.TypefaceFontProvider.Make();
    skFontCollection = canvasKit.FontCollection.Make();
    skFontCollection!.enableFontFallback();
    skFontCollection!.setDefaultFontManager(_fontProvider);
    familyToFontMap.clear();

    for (final RegisteredFont font in _registeredFonts) {
      _fontProvider!.registerFont(font.bytes, font.family);
      familyToFontMap.putIfAbsent(font.family, () => <SkFont>[]).add(SkFont(font.typeface));
    }

    for (final RegisteredFont font in registeredFallbackFonts) {
      _fontProvider!.registerFont(font.bytes, font.family);
      familyToFontMap.putIfAbsent(font.family, () => <SkFont>[]).add(SkFont(font.typeface));
    }
  }

  @override
  Future<bool> loadFontFromList(Uint8List list, {String? fontFamily}) async {
    if (fontFamily == null) {
      fontFamily = _readActualFamilyName(list);
      if (fontFamily == null) {
        printWarning('Failed to read font family name. Aborting font load.');
        return false;
      }
    }

    // Make sure CanvasKit is actually loaded
    await renderer.initialize();

    final SkTypeface? typeface = canvasKit.Typeface.MakeFreeTypeFaceFromData(list.buffer);
    if (typeface != null) {
      _registeredFonts.add(RegisteredFont(list, fontFamily, typeface));
      _registerWithFontProvider();
    } else {
      printWarning('Failed to parse font family "$fontFamily"');
      return false;
    }
    return true;
  }

  /// Loads fonts from `FontManifest.json`.
  @override
  Future<AssetFontsResult> loadAssetFonts(FontManifest manifest) async {
    final List<Future<FontDownloadResult>> pendingDownloads = <Future<FontDownloadResult>>[];
    bool loadedRoboto = false;
    for (final FontFamily family in manifest.families) {
      if (family.name == 'Roboto') {
        loadedRoboto = true;
      }
      for (final FontAsset fontAsset in family.fontAssets) {
        final String url = ui_web.assetManager.getAssetUrl(fontAsset.asset);
        pendingDownloads.add(_downloadFont(fontAsset.asset, url, family.name));
      }
    }

    /// We need a default fallback font for CanvasKit, in order to avoid
    /// crashing while laying out text with an unregistered font. We chose
    /// Roboto to match Android.
    if (!loadedRoboto) {
      // Download Roboto and add it to the font buffers.
      pendingDownloads.add(_downloadFont('Roboto', _robotoUrl, 'Roboto'));
    }

    final Map<String, FontLoadError> fontFailures = <String, FontLoadError>{};
    final List<(String, UnregisteredFont)> downloadedFonts = <(String, UnregisteredFont)>[];
    for (final FontDownloadResult result in await Future.wait(pendingDownloads)) {
      if (result.font != null) {
        downloadedFonts.add((result.assetName, result.font!));
      } else {
        fontFailures[result.assetName] = result.error!;
      }
    }

    // Make sure CanvasKit is actually loaded
    await renderer.initialize();

    final List<String> loadedFonts = <String>[];
    for (final (String assetName, UnregisteredFont unregisteredFont) in downloadedFonts) {
      final Uint8List bytes = unregisteredFont.bytes.asUint8List();
      final SkTypeface? typeface = canvasKit.Typeface.MakeFreeTypeFaceFromData(bytes.buffer);
      if (typeface != null) {
        loadedFonts.add(assetName);
        _registeredFonts.add(RegisteredFont(bytes, unregisteredFont.family, typeface));
      } else {
        printWarning('Failed to load font ${unregisteredFont.family} at ${unregisteredFont.url}');
        printWarning('Verify that ${unregisteredFont.url} contains a valid font.');
        fontFailures[assetName] = FontInvalidDataError(unregisteredFont.url);
      }
    }
    registerDownloadedFonts();
    return AssetFontsResult(loadedFonts, fontFailures);
  }

  void registerDownloadedFonts() {
    RegisteredFont? makeRegisterFont(ByteBuffer buffer, String url, String family) {
      final Uint8List bytes = buffer.asUint8List();
      final SkTypeface? typeface = canvasKit.Typeface.MakeFreeTypeFaceFromData(bytes.buffer);
      if (typeface != null) {
        return RegisteredFont(bytes, family, typeface);
      } else {
        printWarning('Failed to load font $family at $url');
        printWarning('Verify that $url contains a valid font.');
        return null;
      }
    }

    for (final UnregisteredFont unregisteredFont in _unregisteredFonts) {
      final RegisteredFont? registeredFont = makeRegisterFont(
        unregisteredFont.bytes,
        unregisteredFont.url,
        unregisteredFont.family,
      );
      if (registeredFont != null) {
        _registeredFonts.add(registeredFont);
      }
    }

    _unregisteredFonts.clear();
    _registerWithFontProvider();
  }

  Future<FontDownloadResult> _downloadFont(String assetName, String url, String fontFamily) async {
    final ByteBuffer fontData;

    // Try to get the font leniently. Do not crash the app when failing to
    // fetch the font in the spirit of "gradual degradation of functionality".
    try {
      final HttpFetchResponse response = await httpFetch(url);
      if (!response.hasPayload) {
        printWarning('Font family $fontFamily not found (404) at $url');
        return FontDownloadResult.fromError(assetName, FontNotFoundError(url));
      }

      fontData = await response.asByteBuffer();
    } catch (e) {
      printWarning('Failed to load font $fontFamily at $url');
      printWarning(e.toString());
      return FontDownloadResult.fromError(assetName, FontDownloadError(url, e));
    }
    _downloadedFontFamilies.add(fontFamily);
    return FontDownloadResult.fromFont(assetName, UnregisteredFont(fontData, url, fontFamily));
  }

  String? _readActualFamilyName(Uint8List bytes) {
    final SkFontMgr tmpFontMgr = canvasKit.FontMgr.FromData(<Uint8List>[bytes])!;
    final String? actualFamily = tmpFontMgr.getFamilyName(0);
    tmpFontMgr.delete();
    return actualFamily;
  }

  TypefaceFontProvider? _fontProvider;
  SkFontCollection? skFontCollection;

  @override
  void clear() {}

  @override
  void debugResetFallbackFonts() {
    fontFallbackManager = FontFallbackManager(SkiaFallbackRegistry(this));
    registeredFallbackFonts.clear();
  }
}

/// Represents a font that has been registered.
class RegisteredFont {
  RegisteredFont(this.bytes, this.family, this.typeface) {
    // This is a hack which causes Skia to cache the decoded font.
    final SkFont skFont = SkFont(typeface);
    skFont.getGlyphBounds(<int>[0], null, null);
  }

  /// The font family name for this font.
  final String family;

  /// The byte data for this font.
  final Uint8List bytes;

  /// The [SkTypeface] created from this font's [bytes].
  ///
  /// This is used to determine which code points are supported by this font.
  final SkTypeface typeface;
}

/// Represents a font that has been downloaded but not registered.
class UnregisteredFont {
  const UnregisteredFont(this.bytes, this.url, this.family);
  final ByteBuffer bytes;
  final String url;
  final String family;
}

class FontDownloadResult {
  FontDownloadResult.fromFont(this.assetName, UnregisteredFont this.font) : error = null;
  FontDownloadResult.fromError(this.assetName, FontLoadError this.error) : font = null;

  final String assetName;
  final UnregisteredFont? font;
  final FontLoadError? error;
}

class SkiaFallbackRegistry implements FallbackFontRegistry {
  SkiaFallbackRegistry(this.fontCollection);

  SkiaFontCollection fontCollection;

  @override
  List<int> getMissingCodePoints(List<int> codeUnits, List<String> fontFamilies) {
    final List<SkFont> fonts = <SkFont>[];
    for (final String font in fontFamilies) {
      final List<SkFont>? typefacesForFamily = fontCollection.familyToFontMap[font];
      if (typefacesForFamily != null) {
        fonts.addAll(typefacesForFamily);
      }
    }
    final List<bool> codePointsSupported = List<bool>.filled(codeUnits.length, false);
    final String testString = String.fromCharCodes(codeUnits);
    for (final SkFont font in fonts) {
      final Uint16List glyphs = font.getGlyphIDs(testString);
      assert(glyphs.length == codePointsSupported.length);
      for (int i = 0; i < glyphs.length; i++) {
        codePointsSupported[i] |= glyphs[i] != 0;
      }
    }

    final List<int> missingCodeUnits = <int>[];
    for (int i = 0; i < codePointsSupported.length; i++) {
      if (!codePointsSupported[i]) {
        missingCodeUnits.add(codeUnits[i]);
      }
    }
    return missingCodeUnits;
  }

  @override
  Future<void> loadFallbackFont(String familyName, String url) async {
    final ByteBuffer buffer = await httpFetchByteBuffer(url);
    final SkTypeface? typeface = canvasKit.Typeface.MakeFreeTypeFaceFromData(buffer);
    if (typeface == null) {
      printWarning('Failed to parse fallback font $familyName as a font.');
      return;
    }
    fontCollection.registeredFallbackFonts.add(
      RegisteredFont(buffer.asUint8List(), familyName, typeface),
    );
  }

  @override
  void updateFallbackFontFamilies(List<String> families) {
    fontCollection.registerDownloadedFonts();
  }
}
