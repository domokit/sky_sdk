// Copyright 2014 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

import 'dart:io' as io;

import 'package:path/path.dart' as path;
import 'package:test/test.dart';

class FileContentPair {
  FileContentPair({required this.name, required this.original, required this.formatted});

  final String name;
  final String original;
  final String formatted;
}

final FileContentPair dartContentPair = FileContentPair(
  name: 'format_test.dart',
  original: 'enum \n\nfoo {\n  entry1,\n  entry2,\n}',
  formatted: 'enum foo { entry1, entry2 }\n',
);

class TestFileFixture {
  TestFileFixture(this.filePairs, this.baseDir) {
    for (final FileContentPair filePair in filePairs) {
      final io.File file = io.File(path.join(baseDir.path, filePair.name));
      file.writeAsStringSync(filePair.original);
      files.add(file);
    }
  }

  final List<io.File> files = <io.File>[];
  final io.Directory baseDir;
  final List<FileContentPair> filePairs;

  void gitAdd() {
    final List<String> args = <String>['add'];
    for (final io.File file in files) {
      args.add(file.path);
    }

    io.Process.runSync('git', args);
  }

  void gitRemove() {
    final List<String> args = <String>['rm', '-f'];
    for (final io.File file in files) {
      args.add(file.path);
    }
    io.Process.runSync('git', args);
  }

  Iterable<FileContentPair> getFileContents() {
    final List<FileContentPair> results = <FileContentPair>[];
    for (int i = 0; i < files.length; i++) {
      final io.File file = files[i];
      final FileContentPair filePair = filePairs[i];
      final String content = file.readAsStringSync().replaceAll('\r\n', '\n');
      results.add(
        FileContentPair(name: filePair.name, original: content, formatted: filePair.formatted),
      );
    }
    return results;
  }
}

void main() {
  final io.File script = io.File(path.current).absolute;
  final io.Directory flutterRoot = script.parent.parent;
  final String formatterPath = path.join(
    flutterRoot.path,
    'dev',
    'tools',
    'format.${io.Platform.isWindows ? 'bat' : 'sh'}',
  );

  test(
    'Can fix Dart formatting errors',
    () {
      final TestFileFixture fixture = TestFileFixture(<FileContentPair>[
        dartContentPair,
      ], flutterRoot);
      try {
        fixture.gitAdd();
        io.Process.runSync(formatterPath, <String>['--fix'], workingDirectory: flutterRoot.path);

        final Iterable<FileContentPair> files = fixture.getFileContents();
        for (final FileContentPair pair in files) {
          expect(pair.original, equals(pair.formatted));
        }
      } finally {
        fixture.gitRemove();
      }
    },
    // TODO(goderbauer): Re-enable after the formatting changes have landed.
    skip: true,
  );

  test(
    'Prints error if dart formatter fails',
    () {
      final TestFileFixture fixture = TestFileFixture(<FileContentPair>[], flutterRoot);
      final io.File dartFile = io.File('${flutterRoot.path}/format_test2.dart');
      dartFile.writeAsStringSync('P\n');
      fixture.files.add(dartFile);

      try {
        fixture.gitAdd();
        final io.ProcessResult result = io.Process.runSync(formatterPath, <String>[
          '--fix',
        ], workingDirectory: flutterRoot.path);
        expect(result.stdout, contains('format_test2.dart produced the following error'));
        expect(result.exitCode, isNot(0));
      } finally {
        fixture.gitRemove();
      }
    }, // TODO(goderbauer): Re-enable after the formatting changes have landed.
    skip: true,
  );
}
