// Copyright 2014 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

import 'package:file/file.dart';
import 'package:file/memory.dart';
import 'package:flutter_tools/src/dart/package_map.dart';

import '../../src/common.dart';

void main() {
  group('findPackageConfigFile', () {
    late FileSystem fileSystem;

    setUp(() {
      fileSystem = MemoryFileSystem.test();
    });

    test('should find an immediate .dart_tool/package_config.json', () {
      fileSystem.file('.dart_tool/package_config.json').createSync(recursive: true);
      expect(findPackageConfigFile(fileSystem.currentDirectory), isNotNull);
    });

    test('should find a parent .dart_tool/package_config.json', () {
      fileSystem.file('.dart_tool/package_config.json').createSync(recursive: true);
      fileSystem.currentDirectory.childDirectory('child').createSync(recursive: true);
      expect(findPackageConfigFile(fileSystem.currentDirectory.childDirectory('child')), isNotNull);
    });

    test('should not find a .dart_tool/package_config.json in an existing directory', () {
      fileSystem.currentDirectory.childDirectory('child').createSync(recursive: true);
      expect(findPackageConfigFile(fileSystem.currentDirectory.childDirectory('child')), isNull);
    });

    // Regression test: https://github.com/flutter/flutter/issues/163901.
    test('should not find a .dart_tool/package_config.json in a missing directory', () {
      expect(findPackageConfigFile(fileSystem.currentDirectory.childDirectory('missing')), isNull);
    });

    // Regression test: https://github.com/flutter/flutter/issues/163901.
    test('should find a .dart_tool/package_config.json in a parent of a missing directory', () {
      fileSystem.file('.dart_tool/package_config.json').createSync(recursive: true);
      expect(
        findPackageConfigFile(fileSystem.currentDirectory.childDirectory('missing')),
        isNotNull,
      );
    });
  });
}
