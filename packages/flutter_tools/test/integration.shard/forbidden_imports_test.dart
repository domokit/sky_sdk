// Copyright 2014 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

import 'package:file/file.dart';

import '../src/common.dart';
import 'test_utils.dart';

void main() {
  final String flutterTools = fileSystem.path.join(getFlutterRoot(), 'packages', 'flutter_tools');

  test('no imports of commands/* or test/* in lib/src/*', () {
    final List<String> skippedPaths = <String>[
      fileSystem.path.join(flutterTools, 'lib', 'src', 'commands'),
      fileSystem.path.join(flutterTools, 'lib', 'src', 'test'),
    ];
    bool isNotSkipped(FileSystemEntity entity) =>
        skippedPaths.every((String path) => !entity.path.startsWith(path));

    final Iterable<File> files = fileSystem
        .directory(fileSystem.path.join(flutterTools, 'lib', 'src'))
        .listSync(recursive: true)
        .where(_isDartFile)
        .where(isNotSkipped)
        .map(_asFile);
    for (final File file in files) {
      for (final String line in file.readAsLinesSync()) {
        if (line.startsWith(RegExp(r'import.*package:'))) {
          continue;
        }
        if (line.startsWith(RegExp(r'import.*commands/')) ||
            line.startsWith(RegExp(r'import.*test/'))) {
          final String relativePath = fileSystem.path.relative(file.path, from: flutterTools);
          fail(
            '$relativePath imports $line. This import introduces a layering violation. '
            'Please find another way to access the information you are using.',
          );
        }
      }
    }
  });

  test('no imports of globals without a global prefix', () {
    final List<String> skippedPaths = <String>[];
    bool isNotSkipped(FileSystemEntity entity) =>
        skippedPaths.every((String path) => !entity.path.startsWith(path));

    final Iterable<File> files = fileSystem
        .directory(fileSystem.path.join(flutterTools, 'lib', 'src'))
        .listSync(recursive: true)
        .followedBy(
          fileSystem
              .directory(fileSystem.path.join(flutterTools, 'test'))
              .listSync(recursive: true),
        )
        .where(_isDartFile)
        .where(isNotSkipped)
        .map(_asFile);
    for (final File file in files) {
      for (final String line in file.readAsLinesSync()) {
        if ((line.startsWith(RegExp(r'import.*globals.dart')) ||
                line.startsWith(RegExp(r'import.*globals_null_migrated.dart'))) &&
            !line.contains(r'as globals')) {
          final String relativePath = fileSystem.path.relative(file.path, from: flutterTools);
          fail(
            '$relativePath imports globals_null_migrated.dart or globals.dart without a globals prefix.',
          );
        }
      }
    }
  });

  test('no unauthorized imports of dart:io', () {
    final List<String> allowedPaths = <String>[
      // This is a standalone script invoked by xcode, not part of the tool
      fileSystem.path.join(flutterTools, 'bin', 'xcode_backend.dart'),
      fileSystem.path.join(flutterTools, 'lib', 'src', 'base', 'io.dart'),
      fileSystem.path.join(flutterTools, 'lib', 'src', 'base', 'platform.dart'),
      fileSystem.path.join(flutterTools, 'lib', 'src', 'base', 'error_handling_io.dart'),
      fileSystem.path.join(flutterTools, 'lib', 'src', 'base', 'multi_root_file_system.dart'),
    ];
    bool isNotAllowed(FileSystemEntity entity) =>
        allowedPaths.every((String path) => path != entity.path);

    for (final String dirName in <String>['lib', 'bin']) {
      final Iterable<File> files = fileSystem
          .directory(fileSystem.path.join(flutterTools, dirName))
          .listSync(recursive: true)
          .where(_isDartFile)
          .where(isNotAllowed)
          .map(_asFile);
      for (final File file in files) {
        for (final String line in file.readAsLinesSync()) {
          if (line.startsWith(RegExp(r'import.*dart:io')) &&
              !line.contains('flutter_ignore: dart_io_import')) {
            final String relativePath = fileSystem.path.relative(file.path, from: flutterTools);
            fail("$relativePath imports 'dart:io'; import 'lib/src/base/io.dart' instead");
          }
        }
      }
    }
  });

  test('no unauthorized imports of package:http', () {
    final List<String> allowedPaths = <String>[
      // Used only for multi-part file uploads, which are non-trivial to reimplement.
      fileSystem.path.join(flutterTools, 'lib', 'src', 'reporting', 'crash_reporting.dart'),
    ];
    bool isNotAllowed(FileSystemEntity entity) =>
        allowedPaths.every((String path) => path != entity.path);

    for (final String dirName in <String>['lib', 'bin']) {
      final Iterable<File> files = fileSystem
          .directory(fileSystem.path.join(flutterTools, dirName))
          .listSync(recursive: true)
          .where(_isDartFile)
          .where(isNotAllowed)
          .map(_asFile);
      for (final File file in files) {
        for (final String line in file.readAsLinesSync()) {
          if (line.startsWith(RegExp(r'import.*package:http/')) &&
              !line.contains('flutter_ignore: package_http_import')) {
            final String relativePath = fileSystem.path.relative(file.path, from: flutterTools);
            fail("$relativePath imports 'package:http'; import 'lib/src/base/io.dart' instead");
          }
        }
      }
    }
  });

  test('no unauthorized imports of test_api', () {
    final List<String> allowedPaths = <String>[
      fileSystem.path.join(flutterTools, 'lib', 'src', 'test', 'flutter_platform.dart'),
      fileSystem.path.join(flutterTools, 'lib', 'src', 'test', 'flutter_web_platform.dart'),
      fileSystem.path.join(flutterTools, 'lib', 'src', 'test', 'test_wrapper.dart'),
    ];
    bool isNotAllowed(FileSystemEntity entity) =>
        allowedPaths.every((String path) => path != entity.path);

    for (final String dirName in <String>['lib']) {
      final Iterable<File> files = fileSystem
          .directory(fileSystem.path.join(flutterTools, dirName))
          .listSync(recursive: true)
          .where(_isDartFile)
          .where(isNotAllowed)
          .map(_asFile);
      for (final File file in files) {
        for (final String line in file.readAsLinesSync()) {
          if (line.startsWith(RegExp(r'import.*package:test_api')) &&
              !line.contains('flutter_ignore: test_api_import')) {
            final String relativePath = fileSystem.path.relative(file.path, from: flutterTools);
            fail("$relativePath imports 'package:test_api/test_api.dart';");
          }
        }
      }
    }
  });

  test('no unauthorized imports of package:path', () {
    final List<String> allowedPath = <String>[
      fileSystem.path.join(flutterTools, 'lib', 'src', 'isolated', 'web_compilation_delegate.dart'),
      fileSystem.path.join(flutterTools, 'test', 'general.shard', 'platform_plugins_test.dart'),
    ];
    for (final String dirName in <String>['lib', 'bin', 'test']) {
      final Iterable<File> files = fileSystem
          .directory(fileSystem.path.join(flutterTools, dirName))
          .listSync(recursive: true)
          .where(_isDartFile)
          .where((FileSystemEntity entity) => !allowedPath.contains(entity.path))
          .map(_asFile);
      for (final File file in files) {
        for (final String line in file.readAsLinesSync()) {
          if (line.startsWith(RegExp(r'import.*package:path/path.dart')) &&
              !line.contains('flutter_ignore: package_path_import')) {
            final String relativePath = fileSystem.path.relative(file.path, from: flutterTools);
            fail("$relativePath imports 'package:path/path.dart'; use 'fileSystem.path' instead");
          }
        }
      }
    }
  });

  test('no unauthorized imports of package:file/local.dart', () {
    final List<String> allowedPath = <String>[
      fileSystem.path.join(flutterTools, 'test', 'integration.shard', 'test_utils.dart'),
      fileSystem.path.join(flutterTools, 'lib', 'src', 'base', 'file_system.dart'),
    ];
    for (final String dirName in <String>['lib', 'bin', 'test']) {
      final Iterable<File> files = fileSystem
          .directory(fileSystem.path.join(flutterTools, dirName))
          .listSync(recursive: true)
          .where(_isDartFile)
          .where((FileSystemEntity entity) => !allowedPath.contains(entity.path))
          .map(_asFile);
      for (final File file in files) {
        for (final String line in file.readAsLinesSync()) {
          if (line.startsWith(RegExp(r'import.*package:file/local.dart'))) {
            final String relativePath = fileSystem.path.relative(file.path, from: flutterTools);
            fail(
              "$relativePath imports 'package:file/local.dart'; use 'lib/src/base/file_system.dart' instead",
            );
          }
        }
      }
    }
  });

  test('no unauthorized imports of dart:convert', () {
    final List<String> allowedPaths = <String>[
      fileSystem.path.join(flutterTools, 'lib', 'src', 'convert.dart'),
      fileSystem.path.join(flutterTools, 'lib', 'src', 'base', 'error_handling_io.dart'),
    ];
    bool isNotAllowed(FileSystemEntity entity) =>
        allowedPaths.every((String path) => path != entity.path);

    for (final String dirName in <String>['lib']) {
      final Iterable<File> files = fileSystem
          .directory(fileSystem.path.join(flutterTools, dirName))
          .listSync(recursive: true)
          .where(_isDartFile)
          .where(isNotAllowed)
          .map(_asFile);
      for (final File file in files) {
        for (final String line in file.readAsLinesSync()) {
          if (line.startsWith(RegExp(r'import.*dart:convert')) &&
              !line.contains('flutter_ignore: dart_convert_import')) {
            final String relativePath = fileSystem.path.relative(file.path, from: flutterTools);
            fail("$relativePath imports 'dart:convert'; import 'lib/src/convert.dart' instead");
          }
        }
      }
    }
  });

  test('no unauthorized imports of build_runner/dwds/devtools', () {
    final List<String> allowedPaths = <String>[
      fileSystem.path.join(flutterTools, 'test', 'src', 'isolated'),
      fileSystem.path.join(flutterTools, 'lib', 'src', 'isolated'),
      fileSystem.path.join(flutterTools, 'lib', 'executable.dart'),
      fileSystem.path.join(flutterTools, 'lib', 'devfs_web.dart'),
      fileSystem.path.join(flutterTools, 'lib', 'resident_web_runner.dart'),
    ];
    bool isNotAllowed(FileSystemEntity entity) =>
        allowedPaths.every((String path) => !entity.path.contains(path));

    for (final String dirName in <String>['lib']) {
      final Iterable<File> files = fileSystem
          .directory(fileSystem.path.join(flutterTools, dirName))
          .listSync(recursive: true)
          .where(_isDartFile)
          .where(isNotAllowed)
          .map(_asFile);
      for (final File file in files) {
        for (final String line in file.readAsLinesSync()) {
          if (line.startsWith(
                RegExp(r'import.*package:build_runner_core/build_runner_core.dart'),
              ) ||
              line.startsWith(RegExp(r'import.*package:build_runner/build_runner.dart')) ||
              line.startsWith(RegExp(r'import.*package:build_config/build_config.dart')) ||
              line.startsWith(RegExp(r'import.*dwds:*.dart')) ||
              line.startsWith(RegExp(r'import.*devtools_server:*.dart')) ||
              line.startsWith(RegExp(r'import.*build_runner/.*.dart'))) {
            final String relativePath = fileSystem.path.relative(file.path, from: flutterTools);
            fail('$relativePath imports a build_runner/dwds/devtools package');
          }
        }
      }
    }
  });

  test('no import of packages in tool_backend.dart', () {
    final File file = fileSystem.file(
      fileSystem.path.join(flutterTools, 'bin', 'tool_backend.dart'),
    );
    for (final String line in file.readAsLinesSync()) {
      if (line.startsWith(RegExp(r'import.*package:.*'))) {
        final String relativePath = fileSystem.path.relative(file.path, from: flutterTools);
        fail('$relativePath imports a package');
      }
    }
  });
}

bool _isDartFile(FileSystemEntity entity) => entity is File && entity.path.endsWith('.dart');

File _asFile(FileSystemEntity entity) => entity as File;
