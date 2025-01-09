// Copyright 2014 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

import 'package:file/memory.dart';
import 'package:flutter_tools/src/base/file_system.dart';
import 'package:flutter_tools/src/dart/language_version.dart';
import 'package:package_config/package_config.dart';

import '../../src/common.dart';

const String flutterRoot = '';
const String testVersionString = '2.13';
final LanguageVersion testCurrentLanguageVersion = LanguageVersion(2, 13);

void setUpLanguageVersion(FileSystem fileSystem, [String version = testVersionString]) {
  fileSystem.file(fileSystem.path.join('bin', 'cache', 'dart-sdk', 'version'))
    ..createSync(recursive: true)
    ..writeAsStringSync(version);
}

void main() {
  testWithoutContext('detects language version in comment', () {
    final FileSystem fileSystem = MemoryFileSystem.test();
    setUpLanguageVersion(fileSystem);
    final File file = fileSystem.file('example.dart')..writeAsStringSync('''
// Some license

// @dart = 2.9
''');

    expect(determineLanguageVersion(file, null, flutterRoot), LanguageVersion(2, 9));
  });

  testWithoutContext('detects language version in comment without spacing', () {
    final FileSystem fileSystem = MemoryFileSystem.test();
    setUpLanguageVersion(fileSystem);
    final File file = fileSystem.file('example.dart')..writeAsStringSync('''
// Some license

// @dart=2.9
''');

    expect(determineLanguageVersion(file, null, flutterRoot), LanguageVersion(2, 9));
  });

  testWithoutContext('detects language version in comment with more numbers', () {
    final FileSystem fileSystem = MemoryFileSystem.test();
    setUpLanguageVersion(fileSystem);
    final File file = fileSystem.file('example.dart')..writeAsStringSync('''
// Some license

// @dart=2.12
''');

    expect(determineLanguageVersion(file, null, flutterRoot), nullSafeVersion);
  });

  testWithoutContext('does not detect invalid language version', () {
    final FileSystem fileSystem = MemoryFileSystem.test();
    setUpLanguageVersion(fileSystem);
    final File file = fileSystem.file('example.dart')..writeAsStringSync('''
// Some license

// @dart
''');

    expect(determineLanguageVersion(file, null, flutterRoot), testCurrentLanguageVersion);
  });

  testWithoutContext('detects language version with leading whitespace', () {
    final FileSystem fileSystem = MemoryFileSystem.test();
    final File file = fileSystem.file('example.dart')..writeAsStringSync('''
// Some license

    // @dart = 2.9
''');

    expect(determineLanguageVersion(file, null, flutterRoot), LanguageVersion(2, 9));
  });

  testWithoutContext('detects language version with tabs', () {
    final FileSystem fileSystem = MemoryFileSystem.test();
    final File file = fileSystem.file('example.dart')..writeAsStringSync('''
// Some license

//\t@dart = 2.9
''');

    expect(determineLanguageVersion(file, null, flutterRoot), LanguageVersion(2, 9));
  });

  testWithoutContext('detects language version with tons of whitespace', () {
    final FileSystem fileSystem = MemoryFileSystem.test();
    final File file = fileSystem.file('example.dart')..writeAsStringSync('''
// Some license

//        @dart       = 2.23
''');

    expect(determineLanguageVersion(file, null, flutterRoot), LanguageVersion(2, 23));
  });

  testWithoutContext('does not detect language version in dartdoc', () {
    final FileSystem fileSystem = MemoryFileSystem.test();
    setUpLanguageVersion(fileSystem);
    final File file = fileSystem.file('example.dart')..writeAsStringSync('''
// Some license

/// @dart = 2.9
''');

    expect(determineLanguageVersion(file, null, flutterRoot), testCurrentLanguageVersion);
  });

  testWithoutContext('does not detect language version in block comment', () {
    final FileSystem fileSystem = MemoryFileSystem.test();
    setUpLanguageVersion(fileSystem);
    final File file = fileSystem.file('example.dart')..writeAsStringSync('''
// Some license

/*
// @dart = 2.9
*/
''');

    expect(determineLanguageVersion(file, null, flutterRoot), testCurrentLanguageVersion);
  });

  testWithoutContext('does not detect language version in nested block comment', () {
    final FileSystem fileSystem = MemoryFileSystem.test();
    setUpLanguageVersion(fileSystem);
    final File file = fileSystem.file('example.dart')..writeAsStringSync('''
// Some license

/*
/*
// @dart = 2.9
*/
*/
''');

    expect(determineLanguageVersion(file, null, flutterRoot), testCurrentLanguageVersion);
  });

  testWithoutContext('detects language version after nested block comment', () {
    final FileSystem fileSystem = MemoryFileSystem.test();
    final File file = fileSystem.file('example.dart')..writeAsStringSync('''
// Some license

/* /*
*/
*/
// @dart = 2.9
''');

    expect(determineLanguageVersion(file, null, flutterRoot), LanguageVersion(2, 9));
  });

  testWithoutContext('does not crash with unbalanced opening block comments', () {
    final FileSystem fileSystem = MemoryFileSystem.test();
    setUpLanguageVersion(fileSystem);
    final File file = fileSystem.file('example.dart')..writeAsStringSync('''
// Some license

/*
/*
*/
// @dart = 2.9
''');

    expect(determineLanguageVersion(file, null, flutterRoot), testCurrentLanguageVersion);
  });

  testWithoutContext('does not crash with unbalanced closing block comments', () {
    final FileSystem fileSystem = MemoryFileSystem.test();
    setUpLanguageVersion(fileSystem);
    final File file = fileSystem.file('example.dart')..writeAsStringSync('''
// Some license

/*
*/
*/
// @dart = 2.9
''');

    expect(determineLanguageVersion(file, null, flutterRoot), testCurrentLanguageVersion);
  });

  testWithoutContext('does not detect language version in single line block comment', () {
    final FileSystem fileSystem = MemoryFileSystem.test();
    setUpLanguageVersion(fileSystem);
    final File file = fileSystem.file('example.dart')..writeAsStringSync('''
// Some license

/* // @dart = 2.9 */
''');

    expect(determineLanguageVersion(file, null, flutterRoot), testCurrentLanguageVersion);
  });

  testWithoutContext('does not detect language version after import declaration', () {
    final FileSystem fileSystem = MemoryFileSystem.test();
    setUpLanguageVersion(fileSystem);
    final File file = fileSystem.file('example.dart')..writeAsStringSync('''
// Some license

import 'dart:ui' as ui;

// @dart = 2.9
''');

    expect(determineLanguageVersion(file, null, flutterRoot), testCurrentLanguageVersion);
  });

  testWithoutContext('does not detect language version after part declaration', () {
    final FileSystem fileSystem = MemoryFileSystem.test();
    setUpLanguageVersion(fileSystem);
    final File file = fileSystem.file('example.dart')..writeAsStringSync('''
// Some license

part of 'foo.dart';

// @dart = 2.9
''');

    expect(determineLanguageVersion(file, null, flutterRoot), testCurrentLanguageVersion);
  });

  testWithoutContext('does not detect language version after library declaration', () {
    final FileSystem fileSystem = MemoryFileSystem.test();
    setUpLanguageVersion(fileSystem);
    final File file = fileSystem.file('example.dart')..writeAsStringSync('''
// Some license

library funstuff;

// @dart = 2.9
''');

    expect(determineLanguageVersion(file, null, flutterRoot), testCurrentLanguageVersion);
  });

  testWithoutContext('looks up language version from package if not found in file', () {
    final FileSystem fileSystem = MemoryFileSystem.test();
    final File file = fileSystem.file('example.dart')..writeAsStringSync('''
// Some license
''');
    final Package package = Package(
      'foo',
      Uri.parse('file://foo/'),
      languageVersion: LanguageVersion(2, 7),
    );

    expect(determineLanguageVersion(file, package, flutterRoot), LanguageVersion(2, 7));
  });

  testWithoutContext('defaults to current version if package lookup returns null', () {
    final FileSystem fileSystem = MemoryFileSystem.test();
    setUpLanguageVersion(fileSystem);
    final File file = fileSystem.file('example.dart')..writeAsStringSync('''
// Some license
''');
    final Package package = Package('foo', Uri.parse('file://foo/'));

    expect(determineLanguageVersion(file, package, flutterRoot), testCurrentLanguageVersion);
  });

  testWithoutContext(
    'Returns null safe error if reading the file throws a FileSystemException',
    () {
      final FileExceptionHandler handler = FileExceptionHandler();
      final FileSystem fileSystem = MemoryFileSystem.test(opHandle: handler.opHandle);
      setUpLanguageVersion(fileSystem);
      final File errorFile = fileSystem.file('foo');
      handler.addError(errorFile, FileSystemOp.read, const FileSystemException());

      final Package package = Package(
        'foo',
        Uri.parse('file://foo/'),
        languageVersion: LanguageVersion(2, 7),
      );

      expect(determineLanguageVersion(errorFile, package, flutterRoot), testCurrentLanguageVersion);
    },
  );

  testWithoutContext('Can parse Dart language version with pre/post suffix', () {
    final FileSystem fileSystem = MemoryFileSystem.test();
    setUpLanguageVersion(fileSystem, '2.13.0-150.0.dev');

    expect(currentLanguageVersion(fileSystem, flutterRoot), LanguageVersion(2, 13));
  });
}
