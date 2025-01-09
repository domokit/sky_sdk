// Copyright 2014 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

import 'dart:convert';
import 'dart:ui' as ui;

import 'package:flutter/foundation.dart';
import 'package:flutter/painting.dart';
import 'package:flutter/services.dart';
import 'package:flutter_test/flutter_test.dart';

class TestAssetBundle extends CachingAssetBundle {
  TestAssetBundle(this._assetBundleMap);

  final Map<String, List<Map<Object?, Object?>>> _assetBundleMap;

  Map<String, int> loadCallCount = <String, int>{};

  @override
  Future<ByteData> load(String key) async {
    if (key == 'AssetManifest.bin') {
      return const StandardMessageCodec().encodeMessage(_assetBundleMap)!;
    }

    if (key == 'AssetManifest.bin.json') {
      // Encode the manifest data that will be used by the app
      final ByteData data = const StandardMessageCodec().encodeMessage(_assetBundleMap)!;
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

    loadCallCount[key] = loadCallCount[key] ?? 0 + 1;
    if (key == 'one') {
      return ByteData(1)..setInt8(0, 49);
    }
    throw FlutterError('key not found');
  }

  @override
  Future<ui.ImmutableBuffer> loadBuffer(String key) async {
    final ByteData data = await load(key);
    return ui.ImmutableBuffer.fromUint8List(data.buffer.asUint8List());
  }
}

void main() {
  group('1.0 scale device tests', () {
    void buildAndTestWithOneAsset(String mainAssetPath) {
      final Map<String, List<Map<Object?, Object?>>> assetBundleMap =
          <String, List<Map<Object?, Object?>>>{};

      final AssetImage assetImage = AssetImage(
        mainAssetPath,
        bundle: TestAssetBundle(assetBundleMap),
      );
      const ImageConfiguration configuration = ImageConfiguration.empty;

      assetImage
          .obtainKey(configuration)
          .then(
            expectAsync1((AssetBundleImageKey bundleKey) {
              expect(bundleKey.name, mainAssetPath);
              expect(bundleKey.scale, 1.0);
            }),
          );
    }

    test('When asset is main variant check scale is 1.0', () {
      buildAndTestWithOneAsset('assets/normalFolder/normalFile.png');
    });

    test(
      'When asset path and key are the same string even though it could be took as a 3.0x variant',
      () async {
        buildAndTestWithOneAsset('assets/parentFolder/3.0x/normalFile.png');
      },
    );

    test(
      'When asset path contains variant identifier as part of parent folder name scale is 1.0',
      () {
        buildAndTestWithOneAsset('assets/parentFolder/__3.0x__/leafFolder/normalFile.png');
      },
    );

    test(
      'When asset path contains variant identifier as part of leaf folder name scale is 1.0',
      () {
        buildAndTestWithOneAsset('assets/parentFolder/__3.0x_leaf_folder_/normalFile.png');
      },
    );

    test(
      'When asset path contains variant identifier as part of parent folder name scale is 1.0',
      () {
        buildAndTestWithOneAsset('assets/parentFolder/__3.0x__/leafFolder/normalFile.png');
      },
    );

    test('When asset path contains variant identifier in parent folder scale is 1.0', () {
      buildAndTestWithOneAsset('assets/parentFolder/3.0x/leafFolder/normalFile.png');
    });
  });

  group('High-res device behavior tests', () {
    test('When asset is not main variant check scale is not 1.0', () {
      const String mainAssetPath = 'assets/normalFolder/normalFile.png';
      const String variantPath = 'assets/normalFolder/3.0x/normalFile.png';

      final Map<String, List<Map<Object?, Object?>>> assetBundleMap =
          <String, List<Map<Object?, Object?>>>{};

      final Map<Object?, Object?> mainAssetVariantManifestEntry = <Object?, Object?>{};
      mainAssetVariantManifestEntry['asset'] = variantPath;
      mainAssetVariantManifestEntry['dpr'] = 3.0;
      assetBundleMap[mainAssetPath] = <Map<Object?, Object?>>[mainAssetVariantManifestEntry];
      final TestAssetBundle testAssetBundle = TestAssetBundle(assetBundleMap);

      final AssetImage assetImage = AssetImage(mainAssetPath, bundle: testAssetBundle);

      assetImage
          .obtainKey(ImageConfiguration.empty)
          .then(
            expectAsync1((AssetBundleImageKey bundleKey) {
              expect(bundleKey.name, mainAssetPath);
              expect(bundleKey.scale, 1.0);
            }),
          );

      assetImage
          .obtainKey(ImageConfiguration(bundle: testAssetBundle, devicePixelRatio: 3.0))
          .then(
            expectAsync1((AssetBundleImageKey bundleKey) {
              expect(bundleKey.name, variantPath);
              expect(bundleKey.scale, 3.0);
            }),
          );
    });

    test(
      'When high-res device and high-res asset not present in bundle then return main variant',
      () {
        const String mainAssetPath = 'assets/normalFolder/normalFile.png';

        final Map<String, List<Map<Object?, Object?>>> assetBundleMap =
            <String, List<Map<Object?, Object?>>>{};

        assetBundleMap[mainAssetPath] = <Map<Object?, Object?>>[];

        final TestAssetBundle testAssetBundle = TestAssetBundle(assetBundleMap);

        final AssetImage assetImage = AssetImage(
          mainAssetPath,
          bundle: TestAssetBundle(assetBundleMap),
        );

        assetImage
            .obtainKey(ImageConfiguration.empty)
            .then(
              expectAsync1((AssetBundleImageKey bundleKey) {
                expect(bundleKey.name, mainAssetPath);
                expect(bundleKey.scale, 1.0);
              }),
            );

        assetImage
            .obtainKey(ImageConfiguration(bundle: testAssetBundle, devicePixelRatio: 3.0))
            .then(
              expectAsync1((AssetBundleImageKey bundleKey) {
                expect(bundleKey.name, mainAssetPath);
                expect(bundleKey.scale, 1.0);
              }),
            );
      },
    );
  });

  group(
    'Regression - When assets available are 1.0 and 3.0 check devices with a range of scales',
    () {
      const String mainAssetPath = 'assets/normalFolder/normalFile.png';
      const String variantPath = 'assets/normalFolder/3.0x/normalFile.png';

      void buildBundleAndTestVariantLogic(
        double deviceRatio,
        double chosenAssetRatio,
        String expectedAssetPath,
      ) {
        const Map<String, List<Map<Object?, Object?>>> assetManifest =
            <String, List<Map<Object?, Object?>>>{
              'assets/normalFolder/normalFile.png': <Map<Object?, Object?>>[
                <Object?, Object?>{'asset': 'assets/normalFolder/normalFile.png'},
                <Object?, Object?>{'asset': 'assets/normalFolder/3.0x/normalFile.png', 'dpr': 3.0},
              ],
            };
        final TestAssetBundle testAssetBundle = TestAssetBundle(assetManifest);

        final AssetImage assetImage = AssetImage(mainAssetPath, bundle: testAssetBundle);

        // we have 1.0 and 3.0, asking for 1.5 should give
        assetImage
            .obtainKey(ImageConfiguration(bundle: testAssetBundle, devicePixelRatio: deviceRatio))
            .then(
              expectAsync1((AssetBundleImageKey bundleKey) {
                expect(bundleKey.name, expectedAssetPath);
                expect(bundleKey.scale, chosenAssetRatio);
              }),
            );
      }

      test('Obvious case 1.0 - we have exact asset', () {
        buildBundleAndTestVariantLogic(1.0, 1.0, mainAssetPath);
      });

      test('Obvious case 3.0 - we have exact asset', () {
        buildBundleAndTestVariantLogic(3.0, 3.0, variantPath);
      });

      test('Typical case 2.0', () {
        buildBundleAndTestVariantLogic(2.0, 1.0, mainAssetPath);
      });

      test('Borderline case 2.01', () {
        buildBundleAndTestVariantLogic(2.01, 3.0, variantPath);
      });
      test('Borderline case 2.9', () {
        buildBundleAndTestVariantLogic(2.9, 3.0, variantPath);
      });

      test('Typical case 4.0', () {
        buildBundleAndTestVariantLogic(4.0, 3.0, variantPath);
      });
    },
  );
}
