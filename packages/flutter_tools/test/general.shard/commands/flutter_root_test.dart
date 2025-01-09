// Copyright 2014 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

import 'package:file/memory.dart';
import 'package:flutter_tools/src/base/file_system.dart';
import 'package:flutter_tools/src/base/platform.dart';
import 'package:flutter_tools/src/base/user_messages.dart';
import 'package:flutter_tools/src/cache.dart';

import '../../src/common.dart';

void main() {
  testWithoutContext('Cache can initialize flutter root from environment variable', () {
    final String defaultFlutterRoot = Cache.defaultFlutterRoot(
      fileSystem: MemoryFileSystem.test(),
      userMessages: UserMessages(),
      platform: FakePlatform(environment: <String, String>{'FLUTTER_ROOT': 'path/to/flutter'}),
    );

    expect(defaultFlutterRoot, '/path/to/flutter');
  });

  testWithoutContext('Cache can initialize flutter root data-scheme platform script', () {
    final FileSystem fileSystem = MemoryFileSystem.test();
    // For data-uri, the root is initialized to ../.. and then normalized. Change the
    // current directory to verify this.
    final Directory directory = fileSystem.directory('foo/bar/baz/')..createSync(recursive: true);
    fileSystem.currentDirectory = directory;

    final String defaultFlutterRoot = Cache.defaultFlutterRoot(
      fileSystem: fileSystem,
      userMessages: UserMessages(),
      platform: FakePlatform(
        environment: <String, String>{},
        script: Uri.parse('data:,Hello%2C%20World!'),
      ),
    );

    expect(defaultFlutterRoot, '/foo');
  });

  testWithoutContext('Cache can initialize flutter root package-scheme platform script', () {
    final FileSystem fileSystem = MemoryFileSystem.test();
    final String defaultFlutterRoot = Cache.defaultFlutterRoot(
      fileSystem: fileSystem,
      userMessages: UserMessages(),
      platform: FakePlatform(
        environment: <String, String>{},
        script: Uri.parse('package:flutter_tools/flutter_tools.dart'),
        packageConfig: 'flutter/packages/flutter_tools/.dart_tool/package_config.json',
      ),
    );

    expect(defaultFlutterRoot, '/flutter');
  });

  testWithoutContext('Cache can initialize flutter root from snapshot location', () {
    final FileSystem fileSystem = MemoryFileSystem.test();
    final String defaultFlutterRoot = Cache.defaultFlutterRoot(
      fileSystem: fileSystem,
      userMessages: UserMessages(),
      platform: FakePlatform(
        environment: <String, String>{},
        script: Uri.parse('file:///flutter/bin/cache/flutter_tools.snapshot'),
      ),
    );

    expect(defaultFlutterRoot, '/flutter');
  });

  testWithoutContext('Cache can initialize flutter root from script file', () {
    final FileSystem fileSystem = MemoryFileSystem.test();
    final String defaultFlutterRoot = Cache.defaultFlutterRoot(
      fileSystem: fileSystem,
      userMessages: UserMessages(),
      platform: FakePlatform(
        environment: <String, String>{},
        script: Uri.parse('file:///flutter/packages/flutter_tools/bin/flutter_tools.dart'),
      ),
    );

    expect(defaultFlutterRoot, '/flutter');
  });

  testWithoutContext('Cache will default to current directory if there are no matches', () {
    final FileSystem fileSystem = MemoryFileSystem.test();
    final String defaultFlutterRoot = Cache.defaultFlutterRoot(
      fileSystem: fileSystem,
      userMessages: UserMessages(),
      platform: FakePlatform(
        environment: <String, String>{},
        script: Uri.parse('http://foo.bar'), // does not match any heuristics.
      ),
    );

    expect(defaultFlutterRoot, '/');
  });
}
