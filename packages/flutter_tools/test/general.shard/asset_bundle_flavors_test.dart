// Copyright 2014 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

import 'package:file/memory.dart';
import 'package:flutter_tools/src/asset.dart';
import 'package:flutter_tools/src/base/file_system.dart';
import 'package:flutter_tools/src/base/logger.dart';
import 'package:flutter_tools/src/base/platform.dart';
import 'package:flutter_tools/src/base/user_messages.dart';
import 'package:flutter_tools/src/cache.dart';
import 'package:flutter_tools/src/project.dart';

import '../src/common.dart';
import '../src/package_config.dart';

void main() {
  Future<ManifestAssetBundle> buildBundleWithFlavor(
    String? flavor, {
    required Logger logger,
    required FileSystem fileSystem,
    required Platform platform,
  }) async {
    final ManifestAssetBundle bundle = ManifestAssetBundle(
      logger: logger,
      fileSystem: fileSystem,
      platform: platform,
      flutterRoot: Cache.defaultFlutterRoot(
        platform: platform,
        fileSystem: fileSystem,
        userMessages: UserMessages(),
      ),
      splitDeferredAssets: true,
    );

    await bundle.build(
      packageConfigPath: '.dart_tool/package_config.json',
      flutterProject: FlutterProject.fromDirectoryTest(fileSystem.currentDirectory),
      flavor: flavor,
    );
    return bundle;
  }

  testWithoutContext(
    'correctly bundles assets given a simple asset manifest with flavors',
    () async {
      final MemoryFileSystem fileSystem = MemoryFileSystem();
      fileSystem.currentDirectory = fileSystem.systemTempDirectory.createTempSync(
        'flutter_asset_bundle_test.',
      );
      final BufferLogger logger = BufferLogger.test();
      final FakePlatform platform = FakePlatform();

      writePackageConfigFile(directory: fileSystem.currentDirectory, mainLibName: 'example');

      fileSystem
          .file(fileSystem.path.join('assets', 'common', 'image.png'))
          .createSync(recursive: true);
      fileSystem
          .file(fileSystem.path.join('assets', 'vanilla', 'ice-cream.png'))
          .createSync(recursive: true);
      fileSystem
          .file(fileSystem.path.join('assets', 'strawberry', 'ice-cream.png'))
          .createSync(recursive: true);
      fileSystem
          .file(fileSystem.path.join('assets', 'orange', 'ice-cream.png'))
          .createSync(recursive: true);
      fileSystem.file('pubspec.yaml')
        ..createSync()
        ..writeAsStringSync(r'''
name: example
flutter:
  assets:
  - assets/common/
  - path: assets/vanilla/
    flavors:
      - vanilla
  - path: assets/strawberry/
    flavors:
      - strawberry
  - path: assets/orange/ice-cream.png
    flavors:
      - orange
''');

      ManifestAssetBundle bundle;
      bundle = await buildBundleWithFlavor(
        null,
        logger: logger,
        fileSystem: fileSystem,
        platform: platform,
      );
      expect(bundle.entries.keys, contains('assets/common/image.png'));
      expect(bundle.entries.keys, isNot(contains('assets/vanilla/ice-cream.png')));
      expect(bundle.entries.keys, isNot(contains('assets/strawberry/ice-cream.png')));
      expect(bundle.entries.keys, isNot(contains('assets/orange/ice-cream.png')));

      bundle = await buildBundleWithFlavor(
        'strawberry',
        logger: logger,
        fileSystem: fileSystem,
        platform: platform,
      );
      expect(bundle.entries.keys, contains('assets/common/image.png'));
      expect(bundle.entries.keys, isNot(contains('assets/vanilla/ice-cream.png')));
      expect(bundle.entries.keys, contains('assets/strawberry/ice-cream.png'));
      expect(bundle.entries.keys, isNot(contains('assets/orange/ice-cream.png')));

      bundle = await buildBundleWithFlavor(
        'orange',
        logger: logger,
        fileSystem: fileSystem,
        platform: platform,
      );
      expect(bundle.entries.keys, contains('assets/common/image.png'));
      expect(bundle.entries.keys, isNot(contains('assets/vanilla/ice-cream.png')));
      expect(bundle.entries.keys, isNot(contains('assets/strawberry/ice-cream.png')));
      expect(bundle.entries.keys, contains('assets/orange/ice-cream.png'));
    },
  );

  testWithoutContext(
    'throws a tool exit when a non-flavored folder contains a flavored asset',
    () async {
      final MemoryFileSystem fileSystem = MemoryFileSystem();
      fileSystem.currentDirectory = fileSystem.systemTempDirectory.createTempSync(
        'flutter_asset_bundle_test.',
      );
      final BufferLogger logger = BufferLogger.test();
      final FakePlatform platform = FakePlatform();
      writePackageConfigFile(directory: fileSystem.currentDirectory, mainLibName: 'example');

      fileSystem.file(fileSystem.path.join('assets', 'unflavored.png')).createSync(recursive: true);
      fileSystem
          .file(fileSystem.path.join('assets', 'vanillaOrange.png'))
          .createSync(recursive: true);

      fileSystem.file('pubspec.yaml')
        ..createSync()
        ..writeAsStringSync(r'''
name: example
flutter:
  assets:
    - assets/
    - path: assets/vanillaOrange.png
      flavors:
        - vanilla
        - orange
''');

      expect(
        buildBundleWithFlavor(null, logger: logger, fileSystem: fileSystem, platform: platform),
        throwsToolExit(
          message:
              'Multiple assets entries include the file '
              '"assets/vanillaOrange.png", but they specify different lists of flavors.\n'
              'An entry with the path "assets/" does not specify any flavors.\n'
              'An entry with the path "assets/vanillaOrange.png" specifies the flavor(s): "vanilla", "orange".\n\n'
              'Consider organizing assets with different flavors into different directories.',
        ),
      );
    },
  );

  testWithoutContext(
    'throws a tool exit when a flavored folder contains a flavorless asset',
    () async {
      final MemoryFileSystem fileSystem = MemoryFileSystem();
      fileSystem.currentDirectory = fileSystem.systemTempDirectory.createTempSync(
        'flutter_asset_bundle_test.',
      );
      final BufferLogger logger = BufferLogger.test();
      final FakePlatform platform = FakePlatform();
      writePackageConfigFile(directory: fileSystem.currentDirectory, mainLibName: 'example');
      fileSystem.file(fileSystem.path.join('vanilla', 'vanilla.png')).createSync(recursive: true);
      fileSystem
          .file(fileSystem.path.join('vanilla', 'flavorless.png'))
          .createSync(recursive: true);

      fileSystem.file('pubspec.yaml')
        ..createSync()
        ..writeAsStringSync(r'''
name: example
flutter:
  assets:
    - path: vanilla/
      flavors:
        - vanilla
    - vanilla/flavorless.png
''');
      expect(
        buildBundleWithFlavor(null, logger: logger, fileSystem: fileSystem, platform: platform),
        throwsToolExit(
          message:
              'Multiple assets entries include the file '
              '"vanilla/flavorless.png", but they specify different lists of flavors.\n'
              'An entry with the path "vanilla/" specifies the flavor(s): "vanilla".\n'
              'An entry with the path "vanilla/flavorless.png" does not specify any flavors.\n\n'
              'Consider organizing assets with different flavors into different directories.',
        ),
      );
    },
  );

  testWithoutContext(
    'tool exits when two file-explicit entries give the same asset different flavors',
    () {
      final MemoryFileSystem fileSystem = MemoryFileSystem();
      fileSystem.currentDirectory = fileSystem.systemTempDirectory.createTempSync(
        'flutter_asset_bundle_test.',
      );
      final BufferLogger logger = BufferLogger.test();
      final FakePlatform platform = FakePlatform();
      writePackageConfigFile(directory: fileSystem.currentDirectory, mainLibName: 'example');
      fileSystem.file('orange.png').createSync(recursive: true);
      fileSystem.file('pubspec.yaml')
        ..createSync()
        ..writeAsStringSync(r'''
name: example
flutter:
  assets:
    - path: orange.png
      flavors:
        - orange
    - path: orange.png
      flavors:
        - mango
''');

      expect(
        buildBundleWithFlavor(null, logger: logger, fileSystem: fileSystem, platform: platform),
        throwsToolExit(
          message:
              'Multiple assets entries include the file '
              '"orange.png", but they specify different lists of flavors.\n'
              'An entry with the path "orange.png" specifies the flavor(s): "orange".\n'
              'An entry with the path "orange.png" specifies the flavor(s): "mango".',
        ),
      );
    },
  );

  testWithoutContext(
    'throws ToolExit when flavor from file-level declaration has different flavor from containing folder flavor declaration',
    () async {
      final MemoryFileSystem fileSystem = MemoryFileSystem();
      fileSystem.currentDirectory = fileSystem.systemTempDirectory.createTempSync(
        'flutter_asset_bundle_test.',
      );
      final BufferLogger logger = BufferLogger.test();
      final FakePlatform platform = FakePlatform();
      writePackageConfigFile(directory: fileSystem.currentDirectory, mainLibName: 'example');
      fileSystem
          .file(fileSystem.path.join('vanilla', 'actually-strawberry.png'))
          .createSync(recursive: true);
      fileSystem.file(fileSystem.path.join('vanilla', 'vanilla.png')).createSync(recursive: true);

      fileSystem.file('pubspec.yaml')
        ..createSync()
        ..writeAsStringSync(r'''
name: example
flutter:
  assets:
    - path: vanilla/
      flavors:
        - vanilla
    - path: vanilla/actually-strawberry.png
      flavors:
        - strawberry
''');
      expect(
        buildBundleWithFlavor(null, logger: logger, fileSystem: fileSystem, platform: platform),
        throwsToolExit(
          message:
              'Multiple assets entries include the file '
              '"vanilla/actually-strawberry.png", but they specify different lists of flavors.\n'
              'An entry with the path "vanilla/" specifies the flavor(s): "vanilla".\n'
              'An entry with the path "vanilla/actually-strawberry.png" '
              'specifies the flavor(s): "strawberry".',
        ),
      );
    },
  );
}
