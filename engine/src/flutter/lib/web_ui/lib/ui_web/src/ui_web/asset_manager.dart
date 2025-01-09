// Copyright 2013 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

import 'dart:convert';
import 'dart:typed_data';

import 'package:ui/src/engine.dart';

/// Provides the [AssetManager] used by the Flutter Engine.
AssetManager get assetManager => engineAssetManager;

/// This class downloads assets over the network.
///
/// Assets are resolved relative to [assetsDir] inside the absolute base
/// specified by [assetBase] (optional).
///
/// By default, URLs are relative to the `<base>` of the current website.
class AssetManager {
  /// Initializes [AssetManager] with paths.
  AssetManager({this.assetsDir = _defaultAssetsDir, String? assetBase})
    : assert(
        assetBase == null || assetBase.endsWith('/'),
        '`assetBase` must end with a `/` character.',
      ),
      _assetBase = assetBase;

  static const String _defaultAssetsDir = 'assets';

  /// The directory containing the assets.
  final String assetsDir;

  /// The absolute base URL for assets.
  String? _assetBase;

  // Cache a value for `_assetBase` so we don't hit the DOM multiple times.
  String get _baseUrl => _assetBase ??= _deprecatedAssetBase ?? '';

  // Retrieves the `assetBase` value from the DOM.
  //
  // This warns the user and points them to the new initializeEngine style.
  String? get _deprecatedAssetBase {
    final DomHTMLMetaElement? meta =
        domWindow.document.querySelector('meta[name=assetBase]') as DomHTMLMetaElement?;

    final String? fallbackBaseUrl = meta?.content;

    if (fallbackBaseUrl != null) {
      // Warn users that they're using a deprecated configuration style...
      domWindow.console.warn(
        'The `assetBase` meta tag is now deprecated.\n'
        'Use engineInitializer.initializeEngine(config) instead.\n'
        'See: https://docs.flutter.dev/development/platform-integration/web/initialization',
      );
    }
    return fallbackBaseUrl;
  }

  /// Returns the URL to load the asset from, given the asset key.
  ///
  /// We URL-encode the asset URL in order to correctly issue the right
  /// HTTP request to the server.
  ///
  /// For example, if you have an asset in the file "assets/hello world.png",
  /// two things will happen. When the app is built, the asset will be copied
  /// to an asset directory with the file name URL-encoded. So our asset will
  /// be copied to something like "assets/hello%20world.png". To account for
  /// the assets being copied over with a URL-encoded name, the Flutter
  /// framework URL-encodes the asset key  so when it sends a request to the
  /// engine to load "assets/hello world.png", it actually sends a request to
  /// load "assets/hello%20world.png". However, on the web, if we try to load
  /// "assets/hello%20world.png", the request will be URL-decoded, we will
  /// request "assets/hello world.png", and the request will 404. Therefore, we
  /// must URL-encode the asset key *again* so when it is decoded, it is
  /// requesting the once-URL-encoded asset key.
  String getAssetUrl(String asset) {
    if (Uri.parse(asset).hasScheme) {
      return Uri.encodeFull(asset);
    }
    return Uri.encodeFull('$_baseUrl$assetsDir/$asset');
  }

  /// Loads an asset and returns the server response.
  Future<Object> loadAsset(String asset) {
    return httpFetch(getAssetUrl(asset));
  }

  /// Loads an asset using an [XMLHttpRequest] and returns data as [ByteData].
  Future<ByteData> load(String asset) async {
    final String url = getAssetUrl(asset);
    final HttpFetchResponse response = await httpFetch(url);

    if (response.status == 404 && asset == 'AssetManifest.json') {
      printWarning('Asset manifest does not exist at `$url` - ignoring.');
      return ByteData.sublistView(utf8.encode('{}'));
    }

    return (await response.payload.asByteBuffer()).asByteData();
  }
}
