// Copyright 2013 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

import 'dart:async';
import 'dart:convert';
import 'dart:js_interop';
import 'dart:typed_data';

import 'package:ui/src/engine.dart';
import 'package:ui/ui_web/src/ui_web.dart' as ui_web;

class FontAsset {
  FontAsset(this.asset, this.descriptors);

  final String asset;
  final Map<String, String> descriptors;
}

class FontFamily {
  FontFamily(this.name, this.fontAssets);

  final String name;
  final List<FontAsset> fontAssets;
}

class FontManifest {
  FontManifest(this.families);

  final List<FontFamily> families;
}

Future<FontManifest> fetchFontManifest(ui_web.AssetManager assetManager) async {
  final HttpFetchResponse response =
      await assetManager.loadAsset('FontManifest.json') as HttpFetchResponse;
  if (!response.hasPayload) {
    printWarning('Font manifest does not exist at `${response.url}` - ignoring.');
    return FontManifest(<FontFamily>[]);
  }

  final Converter<List<int>, Object?> decoder = const Utf8Decoder().fuse(const JsonDecoder());
  Object? fontManifestJson;
  final Sink<List<int>> inputSink = decoder.startChunkedConversion(
    ChunkedConversionSink<Object?>.withCallback((List<Object?> accumulated) {
      if (accumulated.length != 1) {
        throw AssertionError('There was a problem trying to load FontManifest.json');
      }
      fontManifestJson = accumulated.first;
    }),
  );
  await response.read((JSUint8Array chunk) => inputSink.add(chunk.toDart));
  inputSink.close();
  if (fontManifestJson == null) {
    throw AssertionError('There was a problem trying to load FontManifest.json');
  }
  final List<FontFamily> families =
      (fontManifestJson! as List<dynamic>).map((dynamic fontFamilyJson) {
        final Map<String, dynamic> fontFamily = fontFamilyJson as Map<String, dynamic>;
        final String familyName = fontFamily.readString('family');
        final List<dynamic> fontAssets = fontFamily.readList('fonts');
        return FontFamily(
          familyName,
          fontAssets.map((dynamic fontAssetJson) {
            String? asset;
            final Map<String, String> descriptors = <String, String>{};
            for (final MapEntry<String, dynamic> descriptor
                in (fontAssetJson as Map<String, dynamic>).entries) {
              if (descriptor.key == 'asset') {
                asset = descriptor.value as String;
              } else {
                // Sometimes these descriptors are strings, and sometimes numbers, so we stringify them here.
                descriptors[descriptor.key] = '${descriptor.value}';
              }
            }
            if (asset == null) {
              throw AssertionError("Invalid Font manifest, missing 'asset' key on font.");
            }
            return FontAsset(asset, descriptors);
          }).toList(),
        );
      }).toList();
  return FontManifest(families);
}

abstract class FontLoadError extends Error {
  FontLoadError(this.url);

  String url;
  String get message;
}

class FontNotFoundError extends FontLoadError {
  FontNotFoundError(super.url);

  @override
  String get message => 'Font asset not found at url $url.';
}

class FontDownloadError extends FontLoadError {
  FontDownloadError(super.url, this.error);

  dynamic error;

  @override
  String get message => 'Failed to download font asset at url $url with error: $error.';
}

class FontInvalidDataError extends FontLoadError {
  FontInvalidDataError(super.url);

  @override
  String get message => 'Invalid data for font asset at url $url.';
}

class AssetFontsResult {
  AssetFontsResult(this.loadedFonts, this.fontFailures);

  /// A list of asset keys for fonts that were successfully loaded.
  final List<String> loadedFonts;

  /// A map of the asset keys to failures for fonts that failed to load.
  final Map<String, FontLoadError> fontFailures;
}

abstract class FlutterFontCollection {
  /// Loads a font directly from font data.
  Future<bool> loadFontFromList(Uint8List list, {String? fontFamily});

  /// Completes when fonts from FontManifest.json have been loaded.
  Future<AssetFontsResult> loadAssetFonts(FontManifest manifest);

  // The font fallback manager for this font collection. HTML renderer doesn't
  // have a font fallback manager and just relies on the browser to fall back
  // properly.
  FontFallbackManager? get fontFallbackManager;

  // Reset the state of font fallbacks. Only to be used in testing.
  void debugResetFallbackFonts();

  // Unregisters all fonts.
  void clear();
}
