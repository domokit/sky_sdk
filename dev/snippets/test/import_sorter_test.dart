// Copyright 2014 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

import 'package:file/file.dart';
import 'package:file/memory.dart';
import 'package:path/path.dart' as path;
import 'package:pub_semver/pub_semver.dart';
import 'package:snippets/snippets.dart';
import 'package:test/test.dart' hide TypeMatcher, isInstanceOf;

class FakeFlutterInformation extends FlutterInformation {
  FakeFlutterInformation(this.flutterRoot);

  final Directory flutterRoot;

  @override
  Map<String, dynamic> getFlutterInformation() {
    return <String, dynamic>{
      'flutterRoot': flutterRoot,
      'frameworkVersion': Version(2, 10, 0),
      'dartSdkVersion': Version(2, 12, 1),
    };
  }
}

void main() {
  late MemoryFileSystem memoryFileSystem = MemoryFileSystem();
  late Directory tmpDir;

  setUp(() {
    // Create a new filesystem.
    memoryFileSystem = MemoryFileSystem();
    tmpDir = memoryFileSystem.systemTempDirectory.createTempSync('flutter_snippets_test.');
    final Directory flutterRoot = memoryFileSystem.directory(
      path.join(tmpDir.absolute.path, 'flutter'),
    );
    FlutterInformation.instance = FakeFlutterInformation(flutterRoot);
  });

  test('Sorting packages works', () async {
    final String result = sortImports('''
// Unit comment

// third import
import 'packages:gamma/gamma.dart'; // third

// second import
import 'packages:beta/beta.dart'; // second

// first import
import 'packages:alpha/alpha.dart'; // first

void main() {}
''');
    expect(
      result,
      equals('''
// Unit comment

// first import
import 'packages:alpha/alpha.dart'; // first
// second import
import 'packages:beta/beta.dart'; // second
// third import
import 'packages:gamma/gamma.dart'; // third

void main() {}
'''),
    );
  });
  test('Sorting dart and packages works', () async {
    final String result = sortImports('''
// Unit comment

// third import
import 'packages:gamma/gamma.dart'; // third

// second import
import 'packages:beta/beta.dart'; // second

// first import
import 'packages:alpha/alpha.dart'; // first

// first dart
import 'dart:async';

void main() {}
''');
    expect(
      result,
      equals('''
// Unit comment

// first dart
import 'dart:async';

// first import
import 'packages:alpha/alpha.dart'; // first
// second import
import 'packages:beta/beta.dart'; // second
// third import
import 'packages:gamma/gamma.dart'; // third

void main() {}
'''),
    );
  });
}
