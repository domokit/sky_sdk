// Copyright 2014 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

import 'dart:convert';
import 'dart:io';

import 'package:path/path.dart' as path;
import 'package:process_runner/process_runner.dart';

// This program enables testing of private interfaces in the flutter package.
//
// See README.md for more information.

final Directory flutterRoot =
    Directory(path.fromUri(Platform.script)).absolute.parent.parent.parent.parent.parent;
final Directory flutterPackageDir = Directory(path.join(flutterRoot.path, 'packages', 'flutter'));
final Directory testPrivateDir = Directory(path.join(flutterPackageDir.path, 'test_private'));
final Directory privateTestsDir = Directory(path.join(testPrivateDir.path, 'test'));

void _usage() {
  print('Usage: test_private.dart [--help] [--temp-dir=<temp_dir>]');
  print('''
    --help      Print a usage message.
    --temp-dir  A location where temporary files may be written. Defaults to a
                directory in the system temp folder. If a temp_dir is not
                specified, then the default temp_dir will be created, used, and
                removed automatically.
    ''');
}

Future<void> main(List<String> args) async {
  // TODO(gspencergoog): Convert to using the args package once it has been
  // converted to be non-nullable by default.
  if (args.firstOrNull == '--help') {
    _usage();
    exit(0);
  }

  void errorExit(String message, {int exitCode = -1}) {
    stderr.write('Error: $message\n\n');
    _usage();
    exit(exitCode);
  }

  if (args.length > 2) {
    errorExit('Too many arguments.');
  }

  String? tempDirArg;
  if (args.isNotEmpty) {
    if (args[0].startsWith('--temp-dir')) {
      if (args[0].startsWith('--temp-dir=')) {
        tempDirArg = args[0].replaceFirst('--temp-dir=', '');
      } else {
        if (args.length < 2) {
          errorExit('Not enough arguments to --temp-dir');
        }
        tempDirArg = args[1];
      }
    } else {
      errorExit('Invalid arguments ${args.join(' ')}.');
    }
  }

  Directory tempDir;
  bool removeTempDir = false;
  if (tempDirArg == null || tempDirArg.isEmpty) {
    tempDir = Directory.systemTemp.createTempSync('flutter_package.');
    removeTempDir = true;
  } else {
    tempDir = Directory(tempDirArg);
    if (!tempDir.existsSync()) {
      errorExit("Temporary directory $tempDirArg doesn't exist.");
    }
  }

  bool success = true;
  try {
    await for (final TestCase testCase in getTestCases(tempDir)) {
      stderr.writeln('Analyzing test case $testCase');
      if (!testCase.setUp()) {
        stderr.writeln('Unable to set up $testCase');
        success = false;
        break;
      }
      if (!await testCase.runAnalyzer()) {
        stderr.writeln('Test case $testCase failed analysis.');
        success = false;
        break;
      } else {
        stderr.writeln('Test case $testCase passed analysis.');
      }
      stderr.writeln('Running test case $testCase');
      if (!await testCase.runTests()) {
        stderr.writeln('Test case $testCase failed.');
        success = false;
        break;
      } else {
        stderr.writeln('Test case $testCase succeeded.');
      }
    }
  } finally {
    if (removeTempDir) {
      tempDir.deleteSync(recursive: true);
    }
  }
  exit(success ? 0 : 1);
}

File makeAbsolute(File file, {Directory? workingDirectory}) {
  workingDirectory ??= Directory.current;
  return File(path.join(workingDirectory.absolute.path, file.path));
}

/// A test case representing a private test file that should be run.
///
/// It is loaded from a JSON manifest file that contains a list of dependencies
/// to copy, a list of test files themselves, and a pubspec file.
///
/// The dependencies are copied into the test area with the same relative path.
///
/// The test files are copied to the root of the test area.
///
/// The pubspec file is copied to the root of the test area too, but renamed to
/// "pubspec.yaml".
class TestCase {
  TestCase.fromManifest(this.manifest, this.tmpdir) {
    _json = jsonDecode(manifest.readAsStringSync()) as Map<String, dynamic>;
    tmpdir.createSync(recursive: true);
    assert(tmpdir.existsSync());
  }

  final File manifest;
  final Directory tmpdir;

  Map<String, dynamic> _json = <String, dynamic>{};

  Iterable<File> _getList(String name) sync* {
    for (final dynamic entry in _json[name] as List<dynamic>) {
      final String name = entry as String;
      yield File(path.joinAll(name.split('/')));
    }
  }

  Iterable<File> get dependencies => _getList('deps');
  Iterable<File> get testDependencies => _getList('test_deps');
  Iterable<File> get tests => _getList('tests');
  File get pubspec => File(_json['pubspec'] as String);

  bool setUp() {
    // Copy the manifest tests and deps to the same relative path under the
    // tmpdir.
    for (final File file in dependencies) {
      try {
        final Directory destDir = Directory(path.join(tmpdir.absolute.path, file.parent.path));
        destDir.createSync(recursive: true);
        final File absFile = makeAbsolute(file, workingDirectory: flutterPackageDir);
        final String destination = path.join(tmpdir.absolute.path, file.path);
        absFile.copySync(destination);
      } on FileSystemException catch (e) {
        stderr.writeln('Problem copying manifest dep file ${file.path} to ${tmpdir.path}: $e');
        return false;
      }
    }
    for (final File file in testDependencies) {
      try {
        final Directory destDir = Directory(
          path.join(tmpdir.absolute.path, 'lib', file.parent.path),
        );
        destDir.createSync(recursive: true);
        final File absFile = makeAbsolute(file, workingDirectory: flutterPackageDir);
        final String destination = path.join(tmpdir.absolute.path, 'lib', file.path);
        absFile.copySync(destination);
      } on FileSystemException catch (e) {
        stderr.writeln('Problem copying manifest test_dep file ${file.path} to ${tmpdir.path}: $e');
        return false;
      }
    }
    // Copy the test files into the tmpdir's lib directory.
    for (final File file in tests) {
      String destination = tmpdir.path;
      try {
        final File absFile = makeAbsolute(file, workingDirectory: privateTestsDir);
        // Copy the file, but without the ".tmpl" extension.
        destination = path.join(
          tmpdir.absolute.path,
          'lib',
          path.basenameWithoutExtension(file.path),
        );
        absFile.copySync(destination);
      } on FileSystemException catch (e) {
        stderr.writeln('Problem copying test ${file.path} to $destination: $e');
        return false;
      }
    }

    // Copy the pubspec to the right place.
    makeAbsolute(
      pubspec,
      workingDirectory: privateTestsDir,
    ).copySync(path.join(tmpdir.absolute.path, 'pubspec.yaml'));

    // Use Flutter's analysis_options.yaml file from packages/flutter.
    File(path.join(tmpdir.absolute.path, 'analysis_options.yaml')).writeAsStringSync(
      'include: ${path.toUri(path.join(flutterRoot.path, 'packages', 'flutter', 'analysis_options.yaml'))}\n'
      'linter:\n'
      '  rules:\n'
      // The code does wonky things with the part-of directive that cause false positives.
      '    unreachable_from_main: false',
    );

    return true;
  }

  Future<bool> runAnalyzer() async {
    final String flutter = path.join(flutterRoot.path, 'bin', 'flutter');
    final ProcessRunner runner = ProcessRunner(
      defaultWorkingDirectory: tmpdir.absolute,
      printOutputDefault: true,
    );
    final ProcessRunnerResult result = await runner.runProcess(<String>[
      flutter,
      'analyze',
      '--current-package',
      '--pub',
      '--congratulate',
      '.',
    ], failOk: true);
    if (result.exitCode != 0) {
      return false;
    }
    return true;
  }

  Future<bool> runTests() async {
    final ProcessRunner runner = ProcessRunner(
      defaultWorkingDirectory: tmpdir.absolute,
      printOutputDefault: true,
    );
    final String flutter = path.join(flutterRoot.path, 'bin', 'flutter');
    for (final File test in tests) {
      final String testPath = path.join(
        path.dirname(test.path),
        'lib',
        path.basenameWithoutExtension(test.path),
      );
      final ProcessRunnerResult result = await runner.runProcess(<String>[
        flutter,
        'test',
        testPath,
      ], failOk: true);
      if (result.exitCode != 0) {
        return false;
      }
    }
    return true;
  }

  @override
  String toString() {
    return path.basenameWithoutExtension(manifest.path);
  }
}

Stream<TestCase> getTestCases(Directory tmpdir) async* {
  final Directory testDir = Directory(path.join(testPrivateDir.path, 'test'));
  await for (final FileSystemEntity entity in testDir.list(recursive: true)) {
    if (path.split(entity.path).where((String element) => element.startsWith('.')).isNotEmpty) {
      // Skip hidden files, directories, and the files inside them, like
      // .dart_tool, which contains a (non-hidden) .json file.
      continue;
    }
    if (entity is File && path.basename(entity.path).endsWith('_test.json')) {
      print('Found manifest ${entity.path}');
      final Directory testTmpDir = Directory(
        path.join(tmpdir.absolute.path, path.basenameWithoutExtension(entity.path)),
      );
      yield TestCase.fromManifest(entity, testTmpDir);
    }
  }
}
