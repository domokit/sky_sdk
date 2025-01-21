// Copyright 2014 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

import 'package:file/file.dart';
import 'package:file/local.dart';
import 'package:path/path.dart' as p;
import 'package:test/test.dart';

import '../bin/generate_gradle_lockfiles.dart' as bin;

void main() {
  const FileSystem localFs = LocalFileSystem();
  late Directory tmpFlutterRoot;

  setUp(() {
    tmpFlutterRoot = localFs.systemTempDirectory.createTempSync('generate_gradle_lockfiles_test.');

    // Simulate a version of the top-level directory structure.
    // It's not critical that it is a "real" structure, as long as it includes:
    // - At least one top-level directory that is "engine"
    // - At least one top-level directory that is not "engine"
    // - At least one nested directory (in not "engine") that is "engine"
    final List<String> directoriesToCreate = <String>[
      p.join('dev', 'integration_tests', 'android_test', 'android'),
      p.join('engine', 'src', 'flutter', 'third_party', 'some_package', 'android'),
      p.join('packages', 'flutter_tools', 'test', 'fixtures', 'engine', 'android'),
    ];

    for (final String path in directoriesToCreate) {
      localFs.directory(p.join(tmpFlutterRoot.path, path)).createSync(recursive: true);
    }
  });

  tearDown(() {
    tmpFlutterRoot.deleteSync(recursive: true);
  });

  test('discoverAndroidDirectories resolves a total of 2 directories of the possible 3', () {
    expect(bin.discoverAndroidDirectories(tmpFlutterRoot), hasLength(2));
  });

  test('discoverAndroidDirectories does not traverse into the top-level "engine" directory', () {
    for (final Directory directory in bin.discoverAndroidDirectories(tmpFlutterRoot)) {
      if (directory.path.contains(p.join('engine', 'src', 'flutter'))) {
        fail('Unexpected: ${directory.path} should be excluded (top-level engine directory)');
      }
    }
  });

  test('discoverAndroidDirectories does traverse into a non top-level "engine" directory', () {
    final Iterable<String> paths = bin
        .discoverAndroidDirectories(tmpFlutterRoot)
        .map((Directory r) => r.path);
    expect(
      paths,
      containsAll(<Matcher>[contains(p.join('test', 'fixtures', 'engine', 'android'))]),
      reason: 'A directory named "engine" that is not a root directory should be traversed',
    );
  });
}
