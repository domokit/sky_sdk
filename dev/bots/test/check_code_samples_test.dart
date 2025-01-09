// Copyright 2014 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

import 'dart:io';

import 'package:file/file.dart';
import 'package:file/memory.dart';
import 'package:path/path.dart' as path;

import '../check_code_samples.dart';
import '../utils.dart';
import 'common.dart';

void main() {
  late SampleChecker checker;
  late FileSystem fs;
  late Directory examples;
  late Directory packages;
  late Directory dartUIPath;
  late Directory flutterRoot;

  String getRelativePath(File file, [Directory? from]) {
    from ??= flutterRoot;
    return path.relative(file.absolute.path, from: flutterRoot.absolute.path);
  }

  void writeLink({required File source, required File example, String? alternateLink}) {
    final String link = alternateLink ?? ' ** See code in ${getRelativePath(example)} **';
    source
      ..createSync(recursive: true)
      ..writeAsStringSync('''
/// Class documentation
///
/// {@tool dartpad}
/// Example description
///
///$link
/// {@end-tool}
''');
  }

  void buildTestFiles({
    bool missingLinks = false,
    bool missingTests = false,
    bool malformedLinks = false,
  }) {
    final Directory examplesLib = examples.childDirectory('lib').childDirectory('layer')
      ..createSync(recursive: true);
    final File fooExample =
        examplesLib.childFile('foo_example.0.dart')
          ..createSync(recursive: true)
          ..writeAsStringSync('// Example for foo');
    final File barExample =
        examplesLib.childFile('bar_example.0.dart')
          ..createSync(recursive: true)
          ..writeAsStringSync('// Example for bar');
    if (missingLinks) {
      examplesLib.childFile('missing_example.0.dart')
        ..createSync(recursive: true)
        ..writeAsStringSync('// Example that is not linked');
    }
    final Directory examplesTests = examples.childDirectory('test').childDirectory('layer')
      ..createSync(recursive: true);
    examplesTests.childFile('foo_example.0_test.dart')
      ..createSync(recursive: true)
      ..writeAsStringSync('// test for foo example');
    if (!missingTests) {
      examplesTests.childFile('bar_example.0_test.dart')
        ..createSync(recursive: true)
        ..writeAsStringSync('// test for bar example');
    }
    if (missingLinks) {
      examplesTests.childFile('missing_example.0_test.dart')
        ..createSync(recursive: true)
        ..writeAsStringSync('// test for foo example');
    }
    final Directory flutterPackage = packages
      .childDirectory('flutter')
      .childDirectory('lib')
      .childDirectory('src')..createSync(recursive: true);
    if (malformedLinks) {
      writeLink(
        source: flutterPackage.childDirectory('layer').childFile('foo.dart'),
        example: fooExample,
        alternateLink: '*See Code *',
      );
      writeLink(
        source: flutterPackage.childDirectory('layer').childFile('bar.dart'),
        example: barExample,
        alternateLink: ' ** See code examples/api/lib/layer/bar_example.0.dart **',
      );
    } else {
      writeLink(
        source: flutterPackage.childDirectory('layer').childFile('foo.dart'),
        example: fooExample,
      );
      writeLink(
        source: flutterPackage.childDirectory('layer').childFile('bar.dart'),
        example: barExample,
      );
    }
  }

  setUp(() {
    fs = MemoryFileSystem(
      style: Platform.isWindows ? FileSystemStyle.windows : FileSystemStyle.posix,
    );
    // Get the root prefix of the current directory so that on Windows we get a
    // correct root prefix.
    flutterRoot = fs.directory(
      path.join(path.rootPrefix(fs.currentDirectory.absolute.path), 'flutter sdk'),
    )..createSync(recursive: true);
    fs.currentDirectory = flutterRoot;
    examples = flutterRoot.childDirectory('examples').childDirectory('api')
      ..createSync(recursive: true);
    packages = flutterRoot.childDirectory('packages')..createSync(recursive: true);
    dartUIPath = flutterRoot
      .childDirectory('bin')
      .childDirectory('cache')
      .childDirectory('pkg')
      .childDirectory('sky_engine')
      .childDirectory('lib')..createSync(recursive: true);
    checker = SampleChecker(
      examples: examples,
      packages: packages,
      dartUIPath: dartUIPath,
      flutterRoot: flutterRoot,
      filesystem: fs,
    );
  });

  test('check_code_samples.dart - checkCodeSamples catches missing links', () async {
    buildTestFiles(missingLinks: true);
    bool? success;
    final String result = await capture(() async {
      success = checker.checkCodeSamples();
    }, shouldHaveErrors: true);
    final String lines = <String>[
          '╔═╡ERROR #1╞════════════════════════════════════════════════════════════════════',
          '║ The following examples are not linked from any source file API doc comments:',
          '║   examples/api/lib/layer/missing_example.0.dart',
          '║ Either link them to a source file API doc comment, or remove them.',
          '╚═══════════════════════════════════════════════════════════════════════════════',
        ]
        .map((String line) {
          return line.replaceAll('/', Platform.isWindows ? r'\' : '/');
        })
        .join('\n');
    expect(result, equals('$lines\n'));
    expect(success, equals(false));
  });

  test('check_code_samples.dart - checkCodeSamples catches malformed links', () async {
    buildTestFiles(malformedLinks: true);
    bool? success;
    final String result = await capture(() async {
      success = checker.checkCodeSamples();
    }, shouldHaveErrors: true);
    final bool isWindows = Platform.isWindows;
    final String lines = <String>[
      '╔═╡ERROR #1╞════════════════════════════════════════════════════════════════════',
      '║ The following examples are not linked from any source file API doc comments:',
      if (!isWindows) '║   examples/api/lib/layer/foo_example.0.dart',
      if (!isWindows) '║   examples/api/lib/layer/bar_example.0.dart',
      if (isWindows) r'║   examples\api\lib\layer\foo_example.0.dart',
      if (isWindows) r'║   examples\api\lib\layer\bar_example.0.dart',
      '║ Either link them to a source file API doc comment, or remove them.',
      '╚═══════════════════════════════════════════════════════════════════════════════',
      '╔═╡ERROR #2╞════════════════════════════════════════════════════════════════════',
      '║ The following malformed links were found in API doc comments:',
      if (!isWindows) '║   /flutter sdk/packages/flutter/lib/src/layer/foo.dart:6: ///*See Code *',
      if (!isWindows)
        '║   /flutter sdk/packages/flutter/lib/src/layer/bar.dart:6: /// ** See code examples/api/lib/layer/bar_example.0.dart **',
      if (isWindows)
        r'║   C:\flutter sdk\packages\flutter\lib\src\layer\foo.dart:6: ///*See Code *',
      if (isWindows)
        r'║   C:\flutter sdk\packages\flutter\lib\src\layer\bar.dart:6: /// ** See code examples/api/lib/layer/bar_example.0.dart **',
      '║ Correct the formatting of these links so that they match the exact pattern:',
      r"║   r'\*\* See code in (?<path>.+) \*\*'",
      '╚═══════════════════════════════════════════════════════════════════════════════',
    ].join('\n');
    expect(result, equals('$lines\n'));
    expect(success, equals(false));
  });

  test('check_code_samples.dart - checkCodeSamples catches missing tests', () async {
    buildTestFiles(missingTests: true);
    bool? success;
    final String result = await capture(() async {
      success = checker.checkCodeSamples();
    }, shouldHaveErrors: true);
    final String lines = <String>[
          '╔═╡ERROR #1╞════════════════════════════════════════════════════════════════════',
          '║ The following example test files are missing:',
          '║   examples/api/test/layer/bar_example.0_test.dart',
          '╚═══════════════════════════════════════════════════════════════════════════════',
        ]
        .map((String line) {
          return line.replaceAll('/', Platform.isWindows ? r'\' : '/');
        })
        .join('\n');
    expect(result, equals('$lines\n'));
    expect(success, equals(false));
  });

  test('check_code_samples.dart - checkCodeSamples succeeds', () async {
    buildTestFiles();
    bool? success;
    final String result = await capture(() async {
      success = checker.checkCodeSamples();
    });
    expect(result, isEmpty);
    expect(success, equals(true));
  });
}

typedef AsyncVoidCallback = Future<void> Function();

Future<String> capture(AsyncVoidCallback callback, {bool shouldHaveErrors = false}) async {
  final StringBuffer buffer = StringBuffer();
  final PrintCallback oldPrint = print;
  try {
    print = (Object? line) {
      buffer.writeln(line);
    };
    await callback();
    expect(
      hasError,
      shouldHaveErrors,
      reason:
          buffer.isEmpty
              ? '(No output to report.)'
              : hasError
              ? 'Unexpected errors:\n$buffer'
              : 'Unexpected success:\n$buffer',
    );
  } finally {
    print = oldPrint;
    resetErrorStatus();
  }
  if (stdout.supportsAnsiEscapes) {
    // Remove ANSI escapes when this test is running on a terminal.
    return buffer.toString().replaceAll(RegExp(r'(\x9B|\x1B\[)[0-?]{1,3}[ -/]*[@-~]'), '');
  } else {
    return buffer.toString();
  }
}
