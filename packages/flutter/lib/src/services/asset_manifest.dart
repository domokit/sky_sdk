// Copyright 2014 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

import 'dart:convert';

import 'package:flutter/foundation.dart';

import 'asset_bundle.dart';
import 'message_codecs.dart';

// We use .bin as the extension since it is well-known to represent
// data in some arbitrary binary format.
const String _kAssetManifestFilename = 'AssetManifest.bin';

// We use the same bin file for the web, but re-encoded as JSON(base64(bytes))
// so it can be downloaded by even the dumbest of browsers.
// See https://github.com/flutter/flutter/issues/128456
const String _kAssetManifestWebFilename = 'AssetManifest.bin.json';

/// Contains details about available assets and their variants.
/// See [Resolution-aware image assets](https://flutter.dev/to/resolution-aware-images)
/// to learn about asset variants and how to declare them.
abstract class AssetManifest {
  /// Loads asset manifest data from an [AssetBundle] object and creates an
  /// [AssetManifest] object from that data.
  static Future<AssetManifest> loadFromAssetBundle(AssetBundle bundle) {
    // The AssetManifest file contains binary data.
    //
    // On the web, the build process wraps this binary data in json+base64 so
    // it can be transmitted over the network without special configuration
    // (see #131382).
    if (kIsWeb) {
      // On the web, the AssetManifest is downloaded as a String, then
      // json+base64-decoded to get to the binary data.
      return bundle.loadStructuredData(_kAssetManifestWebFilename, (String jsonData) async {
        // Decode the manifest JSON file to the underlying BIN, and convert to ByteData.
        final ByteData message = ByteData.sublistView(
          base64.decode(json.decode(jsonData) as String),
        );
        // Now we can keep operating as usual.
        return _AssetManifestBin.fromStandardMessageCodecMessage(message);
      });
    }
    // On every other platform, the binary file contents are used directly.
    return bundle.loadStructuredBinaryData(
      _kAssetManifestFilename,
      _AssetManifestBin.fromStandardMessageCodecMessage,
    );
  }

  /// Lists the keys of all main assets. This does not include assets
  /// that are variants of other assets.
  ///
  /// The logical key maps to the path of an asset specified in the pubspec.yaml
  /// file at build time.
  ///
  /// See [Specifying assets](https://docs.flutter.dev/ui/assets/assets-and-images#specifying-assets)
  /// and [Loading assets](https://docs.flutter.dev/ui/assets/assets-and-images#loading-assets)
  /// for more information.
  List<String> listAssets();

  /// Retrieves metadata about an asset and its variants. Returns null if the
  /// key was not found in the asset manifest.
  ///
  /// This method considers a main asset to be a variant of itself. The returned
  /// list will include it if it exists.
  List<AssetMetadata>? getAssetVariants(String key);
}

// Lazily parses the binary asset manifest into a data structure that's easier to work
// with.
//
// The binary asset manifest is a map of asset keys to a list of objects
// representing the asset's variants.
//
// The entries with each variant object are:
//  - "asset": the location of this variant to load it from.
//  - "dpr": The device-pixel-ratio that the asset is best-suited for.
//
// New fields could be added to this object schema to support new asset variation
// features, such as themes, locale/region support, reading directions, and so on.
class _AssetManifestBin implements AssetManifest {
  _AssetManifestBin(Map<Object?, Object?> standardMessageData) : _data = standardMessageData;

  factory _AssetManifestBin.fromStandardMessageCodecMessage(ByteData message) {
    final dynamic data = const StandardMessageCodec().decodeMessage(message);
    return _AssetManifestBin(data as Map<Object?, Object?>);
  }

  final Map<Object?, Object?> _data;
  final Map<String, List<AssetMetadata>> _typeCastedData = <String, List<AssetMetadata>>{};

  @override
  List<AssetMetadata>? getAssetVariants(String key) {
    // We lazily delay typecasting to prevent a performance hiccup when parsing
    // large asset manifests. This is important to keep an app's first asset
    // load fast.
    if (!_typeCastedData.containsKey(key)) {
      final Object? variantData = _data[key];
      if (variantData == null) {
        return null;
      }
      _typeCastedData[key] =
          ((_data[key] ?? <Object?>[]) as Iterable<Object?>).cast<Map<Object?, Object?>>().map((
            Map<Object?, Object?> data,
          ) {
            final String asset = data['asset']! as String;
            final Object? dpr = data['dpr'];
            return AssetMetadata(
              key: data['asset']! as String,
              targetDevicePixelRatio: dpr as double?,
              main: key == asset,
            );
          }).toList();

      _data.remove(key);
    }

    return _typeCastedData[key]!;
  }

  @override
  List<String> listAssets() {
    return <String>[..._data.keys.cast<String>(), ..._typeCastedData.keys];
  }
}

/// Contains information about an asset.
@immutable
class AssetMetadata {
  /// Creates an object containing information about an asset.
  const AssetMetadata({
    required this.key,
    required this.targetDevicePixelRatio,
    required this.main,
  });

  /// The device pixel ratio that this asset is most ideal for. This is determined
  /// by the name of the parent folder of the asset file. For example, if the
  /// parent folder is named "3.0x", the target device pixel ratio of that
  /// asset will be interpreted as 3.
  ///
  /// This will be null if the parent folder name is not a ratio value followed
  /// by an "x".
  ///
  /// See [Resolution-aware image assets](https://flutter.dev/to/resolution-aware-images)
  /// for more information.
  final double? targetDevicePixelRatio;

  /// The asset's key, which is the path to the asset specified in the pubspec.yaml
  /// file at build time.
  final String key;

  /// Whether or not this is a main asset. In other words, this is true if
  /// this asset is not a variant of another asset.
  final bool main;
}
