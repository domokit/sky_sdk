// Copyright 2013 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

import 'dart:io' as io;

import 'package:engine_repo_tools/engine_repo_tools.dart';
import 'package:path/path.dart' as path;
import 'package:test/test.dart';

import '../bin/format.dart' as target;

final io.Directory repoDir = Engine.findWithin().flutterDir;

class FileContentPair {
  FileContentPair(this.original, this.formatted);

  final String original;
  final String formatted;
}

final FileContentPair ccContentPair = FileContentPair(
  'int main(){return 0;}\n',
  'int main() {\n  return 0;\n}\n',
);
final FileContentPair hContentPair = FileContentPair('int\nmain\n()\n;\n', 'int main();\n');
final FileContentPair dartContentPair = FileContentPair(
  'enum \n\nfoo {\n  entry1,\n  entry2,\n}',
  'enum foo { entry1, entry2 }\n',
);
final FileContentPair gnContentPair = FileContentPair(
  'test\n(){testvar=true}\n',
  'test() {\n  testvar = true\n}\n',
);
final FileContentPair javaContentPair = FileContentPair(
  'class Test{public static void main(String args[]){System.out.println("Test");}}\n',
  'class Test {\n  public static void main(String args[]) {\n    System.out.println("Test");\n  }\n}\n',
);
final FileContentPair pythonContentPair = FileContentPair(
  "if __name__=='__main__':\n  sys.exit(\nMain(sys.argv)\n)\n",
  "if __name__ == '__main__':\n  sys.exit(Main(sys.argv))\n",
);
final FileContentPair whitespaceContentPair = FileContentPair(
  'int main() {\n  return 0;       \n}\n',
  'int main() {\n  return 0;\n}\n',
);
final FileContentPair headerContentPair = FileContentPair(
  <String>['#ifndef FOO_H_', '#define FOO_H_', '', '#endif  // FOO_H_'].join('\n'),
  <String>[
    '#ifndef FLUTTER_FORMAT_TEST_H_',
    '#define FLUTTER_FORMAT_TEST_H_',
    '',
    '#endif  // FLUTTER_FORMAT_TEST_H_',
    '',
  ].join('\n'),
);

class TestFileFixture {
  TestFileFixture(this.type) {
    switch (type) {
      case target.FormatCheck.clang:
        final io.File ccFile = io.File('${repoDir.path}/format_test.cc');
        ccFile.writeAsStringSync(ccContentPair.original);
        files.add(ccFile);

        final io.File hFile = io.File('${repoDir.path}/format_test.h');
        hFile.writeAsStringSync(hContentPair.original);
        files.add(hFile);
      case target.FormatCheck.dart:
        final io.File dartFile = io.File('${repoDir.path}/format_test.dart');
        dartFile.writeAsStringSync(dartContentPair.original);
        files.add(dartFile);
      case target.FormatCheck.gn:
        final io.File gnFile = io.File('${repoDir.path}/format_test.gn');
        gnFile.writeAsStringSync(gnContentPair.original);
        files.add(gnFile);
      case target.FormatCheck.java:
        final io.File javaFile = io.File('${repoDir.path}/format_test.java');
        javaFile.writeAsStringSync(javaContentPair.original);
        files.add(javaFile);
      case target.FormatCheck.python:
        final io.File pyFile = io.File('${repoDir.path}/format_test.py');
        pyFile.writeAsStringSync(pythonContentPair.original);
        files.add(pyFile);
      case target.FormatCheck.whitespace:
        final io.File whitespaceFile = io.File('${repoDir.path}/format_test.c');
        whitespaceFile.writeAsStringSync(whitespaceContentPair.original);
        files.add(whitespaceFile);
      case target.FormatCheck.header:
        final io.File headerFile = io.File('${repoDir.path}/format_test.h');
        headerFile.writeAsStringSync(headerContentPair.original);
        files.add(headerFile);
    }
  }

  final target.FormatCheck type;
  final List<io.File> files = <io.File>[];

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
    return files.map((io.File file) {
      String content = file.readAsStringSync();
      // Avoid clang-tidy formatting CRLF EOL on Windows
      content = content.replaceAll('\r\n', '\n');
      switch (type) {
        case target.FormatCheck.clang:
          return FileContentPair(
            content,
            path.extension(file.path) == '.cc' ? ccContentPair.formatted : hContentPair.formatted,
          );
        case target.FormatCheck.dart:
          return FileContentPair(content, dartContentPair.formatted);
        case target.FormatCheck.gn:
          return FileContentPair(content, gnContentPair.formatted);
        case target.FormatCheck.java:
          return FileContentPair(content, javaContentPair.formatted);
        case target.FormatCheck.python:
          return FileContentPair(content, pythonContentPair.formatted);
        case target.FormatCheck.whitespace:
          return FileContentPair(content, whitespaceContentPair.formatted);
        case target.FormatCheck.header:
          return FileContentPair(content, headerContentPair.formatted);
      }
    });
  }
}

void main() {
  final String formatterPath = '${repoDir.path}/ci/format.${io.Platform.isWindows ? 'bat' : 'sh'}';

  test('Can fix C++ formatting errors', () {
    final TestFileFixture fixture = TestFileFixture(target.FormatCheck.clang);
    try {
      fixture.gitAdd();
      io.Process.runSync(formatterPath, <String>[
        '--check',
        'clang',
        '--fix',
      ], workingDirectory: repoDir.path);

      final Iterable<FileContentPair> files = fixture.getFileContents();
      for (final FileContentPair pair in files) {
        expect(pair.original, equals(pair.formatted));
      }
    } finally {
      fixture.gitRemove();
    }
  });

  test('Can fix Dart formatting errors', () {
    final TestFileFixture fixture = TestFileFixture(target.FormatCheck.dart);
    try {
      fixture.gitAdd();
      io.Process.runSync(formatterPath, <String>[
        '--check',
        'dart',
        '--fix',
      ], workingDirectory: repoDir.path);

      final Iterable<FileContentPair> files = fixture.getFileContents();
      for (final FileContentPair pair in files) {
        expect(pair.original, equals(pair.formatted));
      }
    } finally {
      fixture.gitRemove();
    }
  });

  test('Prints error if dart formatter fails', () {
    final TestFileFixture fixture = TestFileFixture(target.FormatCheck.dart);
    final io.File dartFile = io.File('${repoDir.path}/format_test2.dart');
    dartFile.writeAsStringSync('P\n');
    fixture.files.add(dartFile);

    try {
      fixture.gitAdd();
      final io.ProcessResult result = io.Process.runSync(formatterPath, <String>[
        '--check',
        'dart',
        '--fix',
      ], workingDirectory: repoDir.path);
      expect(result.stdout, contains('format_test2.dart produced the following error'));
      expect(result.exitCode, isNot(0));
    } finally {
      fixture.gitRemove();
    }
  });

  test('Can fix GN formatting errors', () {
    final TestFileFixture fixture = TestFileFixture(target.FormatCheck.gn);
    try {
      fixture.gitAdd();
      io.Process.runSync(formatterPath, <String>[
        '--check',
        'gn',
        '--fix',
      ], workingDirectory: repoDir.path);

      final Iterable<FileContentPair> files = fixture.getFileContents();
      for (final FileContentPair pair in files) {
        expect(pair.original, equals(pair.formatted));
      }
    } finally {
      fixture.gitRemove();
    }
  });

  test('Can fix Java formatting errors', () {
    final TestFileFixture fixture = TestFileFixture(target.FormatCheck.java);
    try {
      fixture.gitAdd();
      io.Process.runSync(formatterPath, <String>[
        '--check',
        'java',
        '--fix',
      ], workingDirectory: repoDir.path);

      final Iterable<FileContentPair> files = fixture.getFileContents();
      for (final FileContentPair pair in files) {
        expect(pair.original, equals(pair.formatted));
      }
    } finally {
      fixture.gitRemove();
    }
  });

  test('Can fix Python formatting errors', () {
    final TestFileFixture fixture = TestFileFixture(target.FormatCheck.python);
    try {
      fixture.gitAdd();
      io.Process.runSync(formatterPath, <String>[
        '--check',
        'python',
        '--fix',
      ], workingDirectory: repoDir.path);

      final Iterable<FileContentPair> files = fixture.getFileContents();
      for (final FileContentPair pair in files) {
        expect(pair.original, equals(pair.formatted));
      }
    } finally {
      fixture.gitRemove();
    }
  });

  test('Can fix whitespace formatting errors', () {
    final TestFileFixture fixture = TestFileFixture(target.FormatCheck.whitespace);
    try {
      fixture.gitAdd();
      io.Process.runSync(formatterPath, <String>[
        '--check',
        'whitespace',
        '--fix',
      ], workingDirectory: repoDir.path);

      final Iterable<FileContentPair> files = fixture.getFileContents();
      for (final FileContentPair pair in files) {
        expect(pair.original, equals(pair.formatted));
      }
    } finally {
      fixture.gitRemove();
    }
  });

  test('Can fix header guard formatting errors', () {
    final TestFileFixture fixture = TestFileFixture(target.FormatCheck.header);
    try {
      fixture.gitAdd();
      io.Process.runSync(formatterPath, <String>[
        '--check',
        'header',
        '--fix',
      ], workingDirectory: repoDir.path);
      final Iterable<FileContentPair> files = fixture.getFileContents();
      for (final FileContentPair pair in files) {
        expect(pair.original, equals(pair.formatted));
      }
    } finally {
      fixture.gitRemove();
    }
  });
}
