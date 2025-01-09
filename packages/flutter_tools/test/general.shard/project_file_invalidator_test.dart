// Copyright 2014 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

import 'package:file/memory.dart';
import 'package:flutter_tools/src/base/file_system.dart';
import 'package:flutter_tools/src/base/logger.dart';
import 'package:flutter_tools/src/base/multi_root_file_system.dart';
import 'package:flutter_tools/src/base/platform.dart';
import 'package:flutter_tools/src/run_hot.dart';
import 'package:package_config/package_config.dart';

import '../src/common.dart';

// assumption: tests have a timeout less than 100 days
final DateTime inFuture = DateTime.now().add(const Duration(days: 100));

void main() {
  for (final bool asyncScanning in <bool>[true, false]) {
    testWithoutContext('No last compile, asyncScanning: $asyncScanning', () async {
      final FileSystem fileSystem = MemoryFileSystem.test();
      final ProjectFileInvalidator projectFileInvalidator = ProjectFileInvalidator(
        fileSystem: fileSystem,
        platform: FakePlatform(),
        logger: BufferLogger.test(),
      );
      fileSystem.file('.dart_tool/package_config.json')
        ..createSync(recursive: true)
        ..writeAsStringSync('''
{
  "packages": [],
  "configVersion": 2
}
''');

      expect(
        (await projectFileInvalidator.findInvalidated(
          lastCompiled: null,
          urisToMonitor: <Uri>[],
          packagesPath: '.dart_tool/package_config.json',
          asyncScanning: asyncScanning,
          packageConfig: PackageConfig.empty,
        )).uris,
        isEmpty,
      );
    });

    testWithoutContext('Empty project, asyncScanning: $asyncScanning', () async {
      final FileSystem fileSystem = MemoryFileSystem.test();
      final ProjectFileInvalidator projectFileInvalidator = ProjectFileInvalidator(
        fileSystem: fileSystem,
        platform: FakePlatform(),
        logger: BufferLogger.test(),
      );
      fileSystem.file('.dart_tool/package_config.json')
        ..createSync(recursive: true)
        ..writeAsStringSync('''
{
  "packages": [],
  "configVersion": 2
}
''');

      expect(
        (await projectFileInvalidator.findInvalidated(
          lastCompiled: inFuture,
          urisToMonitor: <Uri>[],
          packagesPath: '.dart_tool/package_config.json',
          asyncScanning: asyncScanning,
          packageConfig: PackageConfig.empty,
        )).uris,
        isEmpty,
      );
    });

    testWithoutContext('Non-existent files are ignored, asyncScanning: $asyncScanning', () async {
      final FileSystem fileSystem = MemoryFileSystem.test();
      final ProjectFileInvalidator projectFileInvalidator = ProjectFileInvalidator(
        fileSystem: MemoryFileSystem.test(),
        platform: FakePlatform(),
        logger: BufferLogger.test(),
      );
      fileSystem.file('.dart_tool/package_config.json')
        ..createSync(recursive: true)
        ..writeAsStringSync('''
{
  "packages": [],
  "configVersion": 2
}
''');

      expect(
        (await projectFileInvalidator.findInvalidated(
          lastCompiled: inFuture,
          urisToMonitor: <Uri>[Uri.parse('/not-there-anymore')],
          packagesPath: '.dart_tool/package_config.json',
          asyncScanning: asyncScanning,
          packageConfig: PackageConfig.empty,
        )).uris,
        isEmpty,
      );
    });

    testWithoutContext(
      'Works with MultiRootFileSystem uris, asyncScanning: $asyncScanning',
      () async {
        final FileSystem fileSystem = MemoryFileSystem.test();
        final FileSystem multiRootFileSystem = MultiRootFileSystem(
          delegate: fileSystem,
          scheme: 'scheme',
          roots: <String>['/root'],
        );
        final ProjectFileInvalidator projectFileInvalidator = ProjectFileInvalidator(
          fileSystem: multiRootFileSystem,
          platform: FakePlatform(),
          logger: BufferLogger.test(),
        );

        expect(
          (await projectFileInvalidator.findInvalidated(
            lastCompiled: inFuture,
            urisToMonitor: <Uri>[
              Uri.parse('file1'),
              Uri.parse('file:///file2'),
              Uri.parse('scheme:///file3'),
            ],
            packagesPath: '.dart_tool/package_config.json',
            asyncScanning: asyncScanning,
            packageConfig: PackageConfig.empty,
          )).uris,
          isEmpty,
        );
      },
    );
  }
}
