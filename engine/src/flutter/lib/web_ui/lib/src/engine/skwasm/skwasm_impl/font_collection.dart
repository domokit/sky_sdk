// Copyright 2013 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

import 'dart:async';
import 'dart:ffi';
import 'dart:js_interop';

import 'dart:typed_data';

import 'package:ui/src/engine.dart';
import 'package:ui/src/engine/skwasm/skwasm_impl.dart';
import 'package:ui/ui_web/src/ui_web.dart' as ui_web;

// This URL was found by using the Google Fonts Developer API to find the URL
// for Roboto. The API warns that this URL is not stable. In order to update
// this, list out all of the fonts and find the URL for the regular
// Roboto font. The API reference is here:
// https://developers.google.com/fonts/docs/developer_api
String _robotoUrl =
    '${configuration.fontFallbackBaseUrl}roboto/v32/KFOmCnqEu92Fr1Me4GZLCzYlKw.woff2';

class SkwasmTypeface extends SkwasmObjectWrapper<RawTypeface> {
  SkwasmTypeface(SkDataHandle data) : super(typefaceCreate(data), _registry);

  static final SkwasmFinalizationRegistry<RawTypeface> _registry =
      SkwasmFinalizationRegistry<RawTypeface>(typefaceDispose);
}

class SkwasmFontCollection implements FlutterFontCollection {
  SkwasmFontCollection() {
    setDefaultFontFamilies(<String>['Roboto']);
  }

  // Most of the time, when an object deals with native handles to skwasm objects,
  // we register it with a finalization registry so that it can clean up the handle
  // when the dart side of object gets GC'd. However, this object is basically a
  // singleton (the renderer creates one and just hangs onto it forever) so it's
  // not really worth it to do the finalization dance here.
  FontCollectionHandle handle = fontCollectionCreate();
  SkwasmNativeTextStyle defaultTextStyle = SkwasmNativeTextStyle.defaultTextStyle();
  final Map<String, List<SkwasmTypeface>> registeredTypefaces = <String, List<SkwasmTypeface>>{};

  void setDefaultFontFamilies(List<String> families) => withStackScope((StackScope scope) {
    final Pointer<SkStringHandle> familyPointers =
        scope.allocPointerArray(families.length).cast<SkStringHandle>();
    for (int i = 0; i < families.length; i++) {
      familyPointers[i] = skStringFromDartString(families[i]);
    }
    textStyleClearFontFamilies(defaultTextStyle.handle);
    textStyleAddFontFamilies(defaultTextStyle.handle, familyPointers, families.length);
    for (int i = 0; i < families.length; i++) {
      skStringFree(familyPointers[i]);
    }
  });

  @override
  late FontFallbackManager fontFallbackManager = FontFallbackManager(SkwasmFallbackRegistry(this));

  @override
  void clear() {
    fontCollectionDispose(handle);
    handle = fontCollectionCreate();
  }

  @override
  Future<AssetFontsResult> loadAssetFonts(FontManifest manifest) async {
    final List<Future<void>> fontFutures = <Future<void>>[];
    final Map<String, FontLoadError> fontFailures = <String, FontLoadError>{};

    /// We need a default fallback font for Skwasm, in order to avoid crashing
    /// while laying out text with an unregistered font. We chose Roboto to
    /// match Android.
    if (!manifest.families.any((FontFamily family) => family.name == 'Roboto')) {
      manifest.families.add(
        FontFamily('Roboto', <FontAsset>[FontAsset(_robotoUrl, <String, String>{})]),
      );
    }

    final List<String> loadedFonts = <String>[];
    for (final FontFamily family in manifest.families) {
      for (final FontAsset fontAsset in family.fontAssets) {
        loadedFonts.add(fontAsset.asset);
        fontFutures.add(() async {
          final FontLoadError? error = await _downloadFontAsset(fontAsset, family.name);
          if (error != null) {
            fontFailures[fontAsset.asset] = error;
          }
        }());
      }
    }

    await Future.wait(fontFutures);

    loadedFonts.removeWhere((String assetName) => fontFailures.containsKey(assetName));
    return AssetFontsResult(loadedFonts, fontFailures);
  }

  Future<FontLoadError?> _downloadFontAsset(FontAsset asset, String family) async {
    final HttpFetchResponse response;
    try {
      response = await ui_web.assetManager.loadAsset(asset.asset) as HttpFetchResponse;
    } catch (error) {
      return FontDownloadError(ui_web.assetManager.getAssetUrl(asset.asset), error);
    }
    if (!response.hasPayload) {
      return FontNotFoundError(ui_web.assetManager.getAssetUrl(asset.asset));
    }
    int length = 0;
    final List<JSUint8Array> chunks = <JSUint8Array>[];
    await response.read((JSUint8Array chunk) {
      length += chunk.length.toDartInt;
      chunks.add(chunk);
    });
    final SkDataHandle fontData = skDataCreate(length);
    int dataAddress = skDataGetPointer(fontData).cast<Int8>().address;
    final JSUint8Array wasmMemory = createUint8ArrayFromBuffer(skwasmInstance.wasmMemory.buffer);
    for (final JSUint8Array chunk in chunks) {
      wasmMemory.set(chunk, dataAddress.toJS);
      dataAddress += chunk.length.toDartInt;
    }
    final SkwasmTypeface typeface = SkwasmTypeface(fontData);
    skDataDispose(fontData);
    if (typeface.handle != nullptr) {
      final SkStringHandle familyNameHandle = skStringFromDartString(family);
      fontCollectionRegisterTypeface(handle, typeface.handle, familyNameHandle);
      registeredTypefaces.putIfAbsent(family, () => <SkwasmTypeface>[]).add(typeface);
      skStringFree(familyNameHandle);
      return null;
    } else {
      return FontInvalidDataError(ui_web.assetManager.getAssetUrl(asset.asset));
    }
  }

  Future<bool> loadFontFromUrl(String familyName, String url) async {
    final HttpFetchResponse response = await httpFetch(url);
    int length = 0;
    final List<JSUint8Array> chunks = <JSUint8Array>[];
    await response.read((JSUint8Array chunk) {
      length += chunk.length.toDartInt;
      chunks.add(chunk);
    });
    final SkDataHandle fontData = skDataCreate(length);
    int dataAddress = skDataGetPointer(fontData).cast<Int8>().address;
    final JSUint8Array wasmMemory = createUint8ArrayFromBuffer(skwasmInstance.wasmMemory.buffer);
    for (final JSUint8Array chunk in chunks) {
      wasmMemory.set(chunk, dataAddress.toJS);
      dataAddress += chunk.length.toDartInt;
    }

    final SkwasmTypeface typeface = SkwasmTypeface(fontData);
    skDataDispose(fontData);
    if (typeface.handle == nullptr) {
      return false;
    }
    final SkStringHandle familyNameHandle = skStringFromDartString(familyName);
    fontCollectionRegisterTypeface(handle, typeface.handle, familyNameHandle);
    registeredTypefaces.putIfAbsent(familyName, () => <SkwasmTypeface>[]).add(typeface);
    skStringFree(familyNameHandle);
    return true;
  }

  @override
  Future<bool> loadFontFromList(Uint8List list, {String? fontFamily}) async {
    final SkDataHandle dataHandle = skDataCreate(list.length);
    final Pointer<Int8> dataPointer = skDataGetPointer(dataHandle).cast<Int8>();
    for (int i = 0; i < list.length; i++) {
      dataPointer[i] = list[i];
    }
    final SkwasmTypeface typeface = SkwasmTypeface(dataHandle);
    skDataDispose(dataHandle);
    if (typeface.handle == nullptr) {
      return false;
    }

    if (fontFamily != null) {
      final SkStringHandle familyHandle = skStringFromDartString(fontFamily);
      fontCollectionRegisterTypeface(handle, typeface.handle, familyHandle);
      skStringFree(familyHandle);
    } else {
      fontCollectionRegisterTypeface(handle, typeface.handle, nullptr);
    }
    return true;
  }

  @override
  void debugResetFallbackFonts() {
    setDefaultFontFamilies(<String>['Roboto']);
    fontFallbackManager = FontFallbackManager(SkwasmFallbackRegistry(this));
    fontCollectionClearCaches(handle);
  }
}

class SkwasmFallbackRegistry implements FallbackFontRegistry {
  SkwasmFallbackRegistry(this.fontCollection);

  final SkwasmFontCollection fontCollection;

  @override
  List<int> getMissingCodePoints(List<int> codePoints, List<String> fontFamilies) =>
      withStackScope((StackScope scope) {
        final List<SkwasmTypeface> typefaces =
            fontFamilies
                .map((String family) => fontCollection.registeredTypefaces[family])
                .fold(
                  const Iterable<SkwasmTypeface>.empty(),
                  (Iterable<SkwasmTypeface> accumulated, List<SkwasmTypeface>? typefaces) =>
                      typefaces == null ? accumulated : accumulated.followedBy(typefaces),
                )
                .toList();
        final Pointer<TypefaceHandle> typefaceBuffer =
            scope.allocPointerArray(typefaces.length).cast<TypefaceHandle>();
        for (int i = 0; i < typefaces.length; i++) {
          typefaceBuffer[i] = typefaces[i].handle;
        }
        final Pointer<Int32> codePointBuffer = scope.allocInt32Array(codePoints.length);
        for (int i = 0; i < codePoints.length; i++) {
          codePointBuffer[i] = codePoints[i];
        }
        final int missingCodePointCount = typefacesFilterCoveredCodePoints(
          typefaceBuffer,
          typefaces.length,
          codePointBuffer,
          codePoints.length,
        );
        return List<int>.generate(missingCodePointCount, (int index) => codePointBuffer[index]);
      });

  @override
  Future<void> loadFallbackFont(String familyName, String url) =>
      fontCollection.loadFontFromUrl(familyName, url);

  @override
  void updateFallbackFontFamilies(List<String> families) =>
      fontCollection.setDefaultFontFamilies(families);
}
