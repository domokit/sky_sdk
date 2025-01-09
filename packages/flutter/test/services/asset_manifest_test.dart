// Copyright 2014 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

import 'dart:convert';

import 'package:flutter/services.dart';
import 'package:flutter_test/flutter_test.dart';

class TestAssetBundle extends AssetBundle {
  static const Map<String, List<Object>> _binManifestData = <String, List<Object>>{
    'assets/foo.png': <Object>[
      <String, Object>{'asset': 'assets/foo.png'},
      <String, Object>{'asset': 'assets/2x/foo.png', 'dpr': 2.0},
    ],
    'assets/bar.png': <Object>[
      <String, Object>{'asset': 'assets/bar.png'},
    ],
  };

  @override
  Future<ByteData> load(String key) async {
    if (key == 'AssetManifest.bin') {
      final ByteData data = const StandardMessageCodec().encodeMessage(_binManifestData)!;
      return data;
    }

    if (key == 'AssetManifest.bin.json') {
      // Encode the manifest data that will be used by the app
      final ByteData data = const StandardMessageCodec().encodeMessage(_binManifestData)!;
      // Simulate the behavior of NetworkAssetBundle.load here, for web tests
      return ByteData.sublistView(
        utf8.encode(
          json.encode(
            base64.encode(
              // Encode only the actual bytes of the buffer, and no more...
              data.buffer.asUint8List(data.offsetInBytes, data.lengthInBytes),
            ),
          ),
        ),
      );
    }

    throw ArgumentError('Unexpected key');
  }

  @override
  Future<T> loadStructuredData<T>(String key, Future<T> Function(String value) parser) async {
    return parser(await loadString(key));
  }
}

void main() {
  TestWidgetsFlutterBinding.ensureInitialized();

  test('loadFromBundle correctly parses a binary asset manifest', () async {
    final AssetManifest manifest = await AssetManifest.loadFromAssetBundle(TestAssetBundle());

    expect(manifest.listAssets(), unorderedEquals(<String>['assets/foo.png', 'assets/bar.png']));

    final List<AssetMetadata> fooVariants = manifest.getAssetVariants('assets/foo.png')!;
    expect(fooVariants.length, 2);
    final AssetMetadata firstFooVariant = fooVariants[0];
    expect(firstFooVariant.key, 'assets/foo.png');
    expect(firstFooVariant.targetDevicePixelRatio, null);
    expect(firstFooVariant.main, true);
    final AssetMetadata secondFooVariant = fooVariants[1];
    expect(secondFooVariant.key, 'assets/2x/foo.png');
    expect(secondFooVariant.targetDevicePixelRatio, 2.0);
    expect(secondFooVariant.main, false);

    final List<AssetMetadata> barVariants = manifest.getAssetVariants('assets/bar.png')!;
    expect(barVariants.length, 1);
    final AssetMetadata firstBarVariant = barVariants[0];
    expect(firstBarVariant.key, 'assets/bar.png');
    expect(firstBarVariant.targetDevicePixelRatio, null);
    expect(firstBarVariant.main, true);
  });

  test('getAssetVariants returns null if the key not contained in the asset manifest', () async {
    final AssetManifest manifest = await AssetManifest.loadFromAssetBundle(TestAssetBundle());
    expect(manifest.getAssetVariants('invalid asset key'), isNull);
  });
}
