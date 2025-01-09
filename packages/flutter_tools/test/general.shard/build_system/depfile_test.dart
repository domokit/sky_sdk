// Copyright 2014 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

import 'package:file/memory.dart';
import 'package:flutter_tools/src/base/file_system.dart';
import 'package:flutter_tools/src/base/logger.dart';
import 'package:flutter_tools/src/build_system/depfile.dart';

import '../../src/common.dart';

void main() {
  late FileSystem fileSystem;
  late DepfileService depfileService;

  setUp(() {
    fileSystem = MemoryFileSystem.test();
    depfileService = DepfileService(logger: BufferLogger.test(), fileSystem: fileSystem);
  });
  testWithoutContext('Can parse depfile from file', () {
    final File depfileSource = fileSystem.file('example.d')..writeAsStringSync('''
a.txt: b.txt
''');
    final Depfile depfile = depfileService.parse(depfileSource);

    expect(depfile.inputs.single.path, 'b.txt');
    expect(depfile.outputs.single.path, 'a.txt');
  });

  testWithoutContext('Can parse depfile with multiple inputs', () {
    final File depfileSource = fileSystem.file('example.d')..writeAsStringSync('''
a.txt: b.txt c.txt d.txt
''');
    final Depfile depfile = depfileService.parse(depfileSource);

    expect(depfile.inputs.map((File file) => file.path), <String>['b.txt', 'c.txt', 'd.txt']);
    expect(depfile.outputs.single.path, 'a.txt');
  });

  testWithoutContext('Can parse depfile with multiple outputs', () {
    final File depfileSource = fileSystem.file('example.d')..writeAsStringSync('''
a.txt c.txt d.txt: b.txt
''');
    final Depfile depfile = depfileService.parse(depfileSource);

    expect(depfile.inputs.single.path, 'b.txt');
    expect(depfile.outputs.map((File file) => file.path), <String>['a.txt', 'c.txt', 'd.txt']);
  });

  testWithoutContext('Can parse depfile with windows file paths', () {
    fileSystem = MemoryFileSystem.test(style: FileSystemStyle.windows);
    depfileService = DepfileService(logger: BufferLogger.test(), fileSystem: fileSystem);
    final File depfileSource = fileSystem.file('example.d')..writeAsStringSync(r'''
C:\\a.txt: C:\\b.txt
''');
    final Depfile depfile = depfileService.parse(depfileSource);

    expect(depfile.inputs.single.path, r'C:\b.txt');
    expect(depfile.outputs.single.path, r'C:\a.txt');
  });

  testWithoutContext(
    'Can escape depfile with windows file paths and spaces in directory names',
    () {
      fileSystem = MemoryFileSystem.test(style: FileSystemStyle.windows);
      depfileService = DepfileService(logger: BufferLogger.test(), fileSystem: fileSystem);
      final File inputFile =
          fileSystem.directory(r'Hello Flutter').childFile('a.txt').absolute
            ..createSync(recursive: true);
      final File outputFile =
          fileSystem.directory(r'Hello Flutter').childFile('b.txt').absolute..createSync();
      final Depfile depfile = Depfile(<File>[inputFile], <File>[outputFile]);
      final File outputDepfile = fileSystem.file('depfile');
      depfileService.writeToFile(depfile, outputDepfile);

      expect(outputDepfile.readAsStringSync(), contains(r'C:\\Hello\ Flutter\\a.txt'));
      expect(outputDepfile.readAsStringSync(), contains(r'C:\\Hello\ Flutter\\b.txt'));
    },
  );

  testWithoutContext('Can escape depfile with spaces in directory names', () {
    final File inputFile =
        fileSystem.directory(r'Hello Flutter').childFile('a.txt').absolute
          ..createSync(recursive: true);
    final File outputFile =
        fileSystem.directory(r'Hello Flutter').childFile('b.txt').absolute..createSync();
    final Depfile depfile = Depfile(<File>[inputFile], <File>[outputFile]);
    final File outputDepfile = fileSystem.file('depfile');
    depfileService.writeToFile(depfile, outputDepfile);

    expect(outputDepfile.readAsStringSync(), contains(r'/Hello\ Flutter/a.txt'));
    expect(outputDepfile.readAsStringSync(), contains(r'/Hello\ Flutter/b.txt'));
  });

  testWithoutContext('Resilient to weird whitespace', () {
    final File depfileSource = fileSystem.file('example.d')..writeAsStringSync(r'''
a.txt
  : b.txt    c.txt


''');
    final Depfile depfile = depfileService.parse(depfileSource);

    expect(depfile.inputs, hasLength(2));
    expect(depfile.outputs.single.path, 'a.txt');
  });

  testWithoutContext('Resilient to duplicate files', () {
    final File depfileSource = fileSystem.file('example.d')..writeAsStringSync(r'''
a.txt: b.txt b.txt
''');
    final Depfile depfile = depfileService.parse(depfileSource);

    expect(depfile.inputs.single.path, 'b.txt');
    expect(depfile.outputs.single.path, 'a.txt');
  });

  testWithoutContext('Resilient to malformed file, missing :', () {
    final File depfileSource = fileSystem.file('example.d')..writeAsStringSync(r'''
a.text b.txt
''');
    final Depfile depfile = depfileService.parse(depfileSource);

    expect(depfile.inputs, isEmpty);
    expect(depfile.outputs, isEmpty);
  });

  testWithoutContext('Can parse dart2js output format', () {
    final File dart2jsDependencyFile = fileSystem.file('main.dart.js.deps')..writeAsStringSync(r'''
file:///Users/foo/collection.dart
file:///Users/foo/algorithms.dart
file:///Users/foo/canonicalized_map.dart
''');

    final Depfile depfile = depfileService.parseDart2js(
      dart2jsDependencyFile,
      fileSystem.file('foo.dart.js'),
    );

    expect(depfile.inputs.map((File file) => file.path), <String>[
      fileSystem.path.absolute(fileSystem.path.join('Users', 'foo', 'collection.dart')),
      fileSystem.path.absolute(fileSystem.path.join('Users', 'foo', 'algorithms.dart')),
      fileSystem.path.absolute(fileSystem.path.join('Users', 'foo', 'canonicalized_map.dart')),
    ]);
    expect(depfile.outputs.single.path, 'foo.dart.js');
  });

  testWithoutContext('Can parse handle invalid uri', () {
    final File dart2jsDependencyFile = fileSystem.file('main.dart.js.deps')..writeAsStringSync('''
file:///Users/foo/collection.dart
abcdevf
file:///Users/foo/canonicalized_map.dart
''');

    final Depfile depfile = depfileService.parseDart2js(
      dart2jsDependencyFile,
      fileSystem.file('foo.dart.js'),
    );

    expect(depfile.inputs.map((File file) => file.path), <String>[
      fileSystem.path.absolute(fileSystem.path.join('Users', 'foo', 'collection.dart')),
      fileSystem.path.absolute(fileSystem.path.join('Users', 'foo', 'canonicalized_map.dart')),
    ]);
    expect(depfile.outputs.single.path, 'foo.dart.js');
  });
}
