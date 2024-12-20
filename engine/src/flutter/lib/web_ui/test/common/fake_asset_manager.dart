// Copyright 2013 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

import 'dart:convert';
import 'dart:typed_data';

import 'package:ui/src/engine.dart';
import 'package:ui/ui_web/src/ui_web.dart' as ui_web;

class FakeAssetManager implements ui_web.AssetManager {
  FakeAssetManager();

  @override
  String get assetsDir => 'assets';

  @override
  String getAssetUrl(String asset) => asset;

  @override
  Future<ByteData> load(String assetKey) async {
    final ByteData? assetData = await _currentScope?.getAssetData(assetKey);
    if (assetData == null) {
      throw HttpFetchNoPayloadError(assetKey, status: 404);
    }
    return assetData;
  }

  @override
  Future<HttpFetchResponse> loadAsset(String asset) async {
    final ByteData? assetData = await _currentScope?.getAssetData(asset);
    if (assetData != null) {
      return MockHttpFetchResponse(
        url: asset,
        status: 200,
        payload: MockHttpFetchPayload(byteBuffer: assetData.buffer),
      );
    } else {
      return MockHttpFetchResponse(url: asset, status: 404);
    }
  }

  FakeAssetScope pushAssetScope() {
    final FakeAssetScope scope = FakeAssetScope._(_currentScope);
    _currentScope = scope;
    return scope;
  }

  void popAssetScope(FakeAssetScope scope) {
    assert(_currentScope == scope);
    _currentScope = scope._parent;
  }

  FakeAssetScope? _currentScope;
}

class FakeAssetScope {
  FakeAssetScope._(this._parent);

  final FakeAssetScope? _parent;
  final Map<String, Future<ByteData> Function()> _assetFetcherMap =
      <String, Future<ByteData> Function()>{};

  void setAsset(String assetKey, ByteData assetData) {
    _assetFetcherMap[assetKey] = () async => assetData;
  }

  void setAssetPassthrough(String assetKey) {
    _assetFetcherMap[assetKey] = () async {
      return ByteData.view(await httpFetchByteBuffer(assetKey));
    };
  }

  Future<ByteData>? getAssetData(String assetKey) {
    final Future<ByteData> Function()? fetcher = _assetFetcherMap[assetKey];
    if (fetcher != null) {
      return fetcher();
    }
    if (_parent != null) {
      return _parent.getAssetData(assetKey);
    }
    return null;
  }
}

FakeAssetManager fakeAssetManager = FakeAssetManager();

ByteData stringAsUtf8Data(String string) {
  return ByteData.sublistView(utf8.encode(string));
}

const String ahemFontFamily = 'Ahem';
const String ahemFontUrl = '/assets/fonts/ahem.ttf';
const String robotoFontFamily = 'Roboto';
const String robotoTestFontUrl = '/assets/fonts/Roboto-Regular.ttf';
const String robotoVariableFontFamily = 'RobotoVariable';
const String robotoVariableFontUrl = '/assets/fonts/RobotoSlab-VariableFont_wght.ttf';

/// The list of test fonts, in the form of font family name - font file url pairs.
/// This list does not include embedded test fonts, which need to be loaded and
/// registered separately in [FontCollection.debugDownloadTestFonts].
const Map<String, String> testFontUrls = <String, String>{
  ahemFontFamily: ahemFontUrl,
  robotoFontFamily: robotoTestFontUrl,
  robotoVariableFontFamily: robotoVariableFontUrl,
};

FakeAssetScope configureDebugFontsAssetScope(FakeAssetManager manager) {
  final FakeAssetScope scope = manager.pushAssetScope();
  scope.setAsset('AssetManifest.json', stringAsUtf8Data('{}'));
  scope.setAsset(
    'FontManifest.json',
    stringAsUtf8Data('''
  [
   {
      "family":"$robotoFontFamily",
      "fonts":[{"asset":"$robotoTestFontUrl"}]
   },
   {
      "family":"$ahemFontFamily",
      "fonts":[{"asset":"$ahemFontUrl"}]
   },
   {
      "family":"$robotoVariableFontFamily",
      "fonts":[{"asset":"$robotoVariableFontUrl"}]
    }
  ]'''),
  );
  scope.setAssetPassthrough(robotoTestFontUrl);
  scope.setAssetPassthrough(ahemFontUrl);
  scope.setAssetPassthrough(robotoVariableFontUrl);
  return scope;
}
