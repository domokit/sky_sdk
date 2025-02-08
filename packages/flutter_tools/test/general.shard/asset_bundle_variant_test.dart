// Copyright 2014 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

import 'dart:convert';
import 'dart:typed_data';

import 'package:file/file.dart';
import 'package:file/memory.dart';

import 'package:flutter_tools/src/asset.dart';
import 'package:flutter_tools/src/base/file_system.dart';
import 'package:flutter_tools/src/base/logger.dart';
import 'package:flutter_tools/src/base/platform.dart';
import 'package:flutter_tools/src/base/user_messages.dart';
import 'package:flutter_tools/src/cache.dart';

import 'package:flutter_tools/src/project.dart';
import 'package:standard_message_codec/standard_message_codec.dart';

import '../src/common.dart';
import '../src/package_config.dart';

void main() {
  Future<Map<String, List<String>>> extractAssetManifestJsonFromBundle(
    ManifestAssetBundle bundle,
  ) async {
    final String manifestJson = utf8.decode(
      await bundle.entries['AssetManifest.json']!.contentsAsBytes(),
    );
    final Map<String, dynamic> parsedJson = json.decode(manifestJson) as Map<String, dynamic>;
    final Iterable<String> keys = parsedJson.keys;
    final Map<String, List<String>> parsedManifest = <String, List<String>>{
      for (final String key in keys) key: List<String>.from(parsedJson[key] as List<dynamic>),
    };
    return parsedManifest;
  }

  Future<Map<Object?, Object?>> extractAssetManifestSmcBinFromBundle(
    ManifestAssetBundle bundle,
  ) async {
    final List<int> manifest = await bundle.entries['AssetManifest.bin']!.contentsAsBytes();
    final ByteData asByteData = ByteData.view(Uint8List.fromList(manifest).buffer);
    final Map<Object?, Object?> decoded =
        const StandardMessageCodec().decodeMessage(asByteData)! as Map<Object?, Object?>;
    return decoded;
  }

  group('AssetBundle asset variants (with Unix-style paths)', () {
    late Platform platform;
    late FileSystem fs;
    late String flutterRoot;

    setUp(() {
      platform = FakePlatform();
      fs = MemoryFileSystem.test();
      flutterRoot = Cache.defaultFlutterRoot(
        platform: platform,
        fileSystem: fs,
        userMessages: UserMessages(),
      );
      writePackageConfigFile(directory: fs.currentDirectory, mainLibName: 'test');
    });

    void createPubspec({required List<String> assets}) {
      fs.file('pubspec.yaml').writeAsStringSync('''
name: test
dependencies:
  flutter:
    sdk: flutter
flutter:
  assets:
${assets.map((String entry) => '    - $entry').join('\n')}
''');
    }

    testWithoutContext(
      'Only images in folders named with device pixel ratios (e.g. 2x, 3.0x) should be considered as variants of other images',
      () async {
        createPubspec(assets: <String>['assets/', 'assets/notAVariant/']);

        const String image = 'assets/image.jpg';
        const String image2xVariant = 'assets/2x/image.jpg';
        const String imageNonVariant = 'assets/notAVariant/image.jpg';

        final List<String> assets = <String>[image, image2xVariant, imageNonVariant];

        for (final String asset in assets) {
          final File assetFile = fs.file(asset);
          assetFile.createSync(recursive: true);
          assetFile.writeAsStringSync(asset);
        }

        final ManifestAssetBundle bundle = ManifestAssetBundle(
          logger: BufferLogger.test(),
          fileSystem: fs,
          platform: platform,
          flutterRoot: flutterRoot,
        );

        await bundle.build(
          packageConfigPath: '.dart_tool/package_config.json',
          flutterProject: FlutterProject.fromDirectoryTest(fs.currentDirectory),
        );

        final Map<String, List<String>> jsonManifest = await extractAssetManifestJsonFromBundle(
          bundle,
        );
        final Map<Object?, Object?> smcBinManifest = await extractAssetManifestSmcBinFromBundle(
          bundle,
        );

        final Map<String, List<Map<String, Object>>> expectedAssetManifest =
            <String, List<Map<String, Object>>>{
              image: <Map<String, Object>>[
                <String, String>{'asset': image},
                <String, Object>{'asset': image2xVariant, 'dpr': 2.0},
              ],
              imageNonVariant: <Map<String, String>>[
                <String, String>{'asset': imageNonVariant},
              ],
            };

        expect(smcBinManifest, equals(expectedAssetManifest));
        expect(jsonManifest, equals(_assetManifestBinToJson(expectedAssetManifest)));
      },
    );

    testWithoutContext(
      'Asset directories have their subdirectories searched for asset variants',
      () async {
        createPubspec(assets: <String>['assets/', 'assets/folder/']);

        const String topLevelImage = 'assets/image.jpg';
        const String secondLevelImage = 'assets/folder/secondLevel.jpg';
        const String secondLevel2xVariant = 'assets/folder/2x/secondLevel.jpg';

        final List<String> assets = <String>[topLevelImage, secondLevelImage, secondLevel2xVariant];

        for (final String asset in assets) {
          final File assetFile = fs.file(asset);
          assetFile.createSync(recursive: true);
          assetFile.writeAsStringSync(asset);
        }

        final ManifestAssetBundle bundle = ManifestAssetBundle(
          logger: BufferLogger.test(),
          fileSystem: fs,
          flutterRoot: flutterRoot,
          platform: platform,
        );

        await bundle.build(
          packageConfigPath: '.dart_tool/package_config.json',
          flutterProject: FlutterProject.fromDirectoryTest(fs.currentDirectory),
        );

        final Map<String, List<String>> jsonManifest = await extractAssetManifestJsonFromBundle(
          bundle,
        );
        expect(jsonManifest, hasLength(2));
        expect(jsonManifest[topLevelImage], equals(<String>[topLevelImage]));
        expect(
          jsonManifest[secondLevelImage],
          equals(<String>[secondLevelImage, secondLevel2xVariant]),
        );

        final Map<Object?, Object?> smcBinManifest = await extractAssetManifestSmcBinFromBundle(
          bundle,
        );

        final Map<String, List<Map<String, Object>>> expectedAssetManifest =
            <String, List<Map<String, Object>>>{
              topLevelImage: <Map<String, Object>>[
                <String, String>{'asset': topLevelImage},
              ],
              secondLevelImage: <Map<String, Object>>[
                <String, String>{'asset': secondLevelImage},
                <String, Object>{'asset': secondLevel2xVariant, 'dpr': 2.0},
              ],
            };
        expect(jsonManifest, equals(_assetManifestBinToJson(expectedAssetManifest)));
        expect(smcBinManifest, equals(expectedAssetManifest));
      },
    );

    testWithoutContext('Asset paths should never be URI-encoded', () async {
      createPubspec(assets: <String>['assets/normalFolder/']);

      const String image = 'assets/normalFolder/i have URI-reserved_characters.jpg';
      const String imageVariant = 'assets/normalFolder/3x/i have URI-reserved_characters.jpg';

      final List<String> assets = <String>[image, imageVariant];

      for (final String asset in assets) {
        final File assetFile = fs.file(asset);
        assetFile.createSync(recursive: true);
        assetFile.writeAsStringSync(asset);
      }

      final ManifestAssetBundle bundle = ManifestAssetBundle(
        logger: BufferLogger.test(),
        fileSystem: fs,
        platform: platform,
        flutterRoot: flutterRoot,
      );

      await bundle.build(
        packageConfigPath: '.dart_tool/package_config.json',
        flutterProject: FlutterProject.fromDirectoryTest(fs.currentDirectory),
      );

      final Map<String, List<String>> jsonManifest = await extractAssetManifestJsonFromBundle(
        bundle,
      );
      final Map<Object?, Object?> smcBinManifest = await extractAssetManifestSmcBinFromBundle(
        bundle,
      );

      final Map<String, List<Map<String, Object>>> expectedAssetManifest =
          <String, List<Map<String, Object>>>{
            image: <Map<String, Object>>[
              <String, Object>{'asset': image},
              <String, Object>{'asset': imageVariant, 'dpr': 3.0},
            ],
          };

      expect(jsonManifest, equals(_assetManifestBinToJson(expectedAssetManifest)));
      expect(smcBinManifest, equals(expectedAssetManifest));
    });

    testWithoutContext('Main assets are not included if the file does not exist', () async {
      createPubspec(assets: <String>['assets/image.png']);

      // We intentionally do not add a 'assets/image.png'.
      const String imageVariant = 'assets/2x/image.png';
      final List<String> assets = <String>[imageVariant];

      for (final String asset in assets) {
        final File assetFile = fs.file(asset);
        assetFile.createSync(recursive: true);
        assetFile.writeAsStringSync(asset);
      }

      final ManifestAssetBundle bundle = ManifestAssetBundle(
        logger: BufferLogger.test(),
        fileSystem: fs,
        platform: platform,
        flutterRoot: flutterRoot,
      );

      await bundle.build(
        packageConfigPath: '.dart_tool/package_config.json',
        flutterProject: FlutterProject.fromDirectoryTest(fs.currentDirectory),
      );

      final Map<String, List<Map<String, Object>>> expectedManifest =
          <String, List<Map<String, Object>>>{
            'assets/image.png': <Map<String, Object>>[
              <String, Object>{'asset': imageVariant, 'dpr': 2.0},
            ],
          };
      final Map<String, List<String>> jsonManifest = await extractAssetManifestJsonFromBundle(
        bundle,
      );
      final Map<Object?, Object?> smcBinManifest = await extractAssetManifestSmcBinFromBundle(
        bundle,
      );

      expect(jsonManifest, equals(_assetManifestBinToJson(expectedManifest)));
      expect(smcBinManifest, equals(expectedManifest));
    });
  });

  group('AssetBundle asset variants (with Windows-style filepaths)', () {
    late final Platform platform;
    late final FileSystem fs;
    late final String flutterRoot;

    setUp(() {
      platform = FakePlatform(operatingSystem: 'windows');
      fs = MemoryFileSystem.test(style: FileSystemStyle.windows);
      flutterRoot = Cache.defaultFlutterRoot(
        platform: platform,
        fileSystem: fs,
        userMessages: UserMessages(),
      );

      writePackageConfigFile(directory: fs.currentDirectory, mainLibName: 'test');

      fs.file('pubspec.yaml').writeAsStringSync('''
name: test
dependencies:
  flutter:
    sdk: flutter
flutter:
  assets:
    - assets/
    - assets/somewhereElse/
''');
    });

    testWithoutContext('Variant detection works with windows-style filepaths', () async {
      const List<String> assets = <String>[
        r'assets\foo.jpg',
        r'assets\2x\foo.jpg',
        r'assets\somewhereElse\bar.jpg',
        r'assets\somewhereElse\2x\bar.jpg',
      ];

      for (final String asset in assets) {
        final File assetFile = fs.file(asset);
        assetFile.createSync(recursive: true);
        assetFile.writeAsStringSync(asset);
      }

      final ManifestAssetBundle bundle = ManifestAssetBundle(
        logger: BufferLogger.test(),
        fileSystem: fs,
        platform: platform,
        flutterRoot: flutterRoot,
      );

      await bundle.build(
        packageConfigPath: '.dart_tool/package_config.json',
        flutterProject: FlutterProject.fromDirectoryTest(fs.currentDirectory),
      );

      final Map<String, List<Map<String, Object>>> expectedAssetManifest =
          <String, List<Map<String, Object>>>{
            'assets/foo.jpg': <Map<String, Object>>[
              <String, Object>{'asset': 'assets/foo.jpg'},
              <String, Object>{'asset': 'assets/2x/foo.jpg', 'dpr': 2.0},
            ],
            'assets/somewhereElse/bar.jpg': <Map<String, Object>>[
              <String, Object>{'asset': 'assets/somewhereElse/bar.jpg'},
              <String, Object>{'asset': 'assets/somewhereElse/2x/bar.jpg', 'dpr': 2.0},
            ],
          };

      final Map<String, List<String>> jsonManifest = await extractAssetManifestJsonFromBundle(
        bundle,
      );
      final Map<Object?, Object?> smcBinManifest = await extractAssetManifestSmcBinFromBundle(
        bundle,
      );

      expect(jsonManifest, equals(_assetManifestBinToJson(expectedAssetManifest)));
      expect(smcBinManifest, equals(expectedAssetManifest));
    });
  });
}

Map<Object, Object> _assetManifestBinToJson(Map<Object, Object> manifest) {
  List<Object> convertList(List<Object> variants) =>
      variants.map((Object variant) => (variant as Map<Object?, Object?>)['asset']!).toList();

  return manifest.map(
    (Object key, Object value) => MapEntry<Object, Object>(key, convertList(value as List<Object>)),
  );
}
