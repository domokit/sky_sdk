// Copyright 2014 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

import 'package:file/file.dart';
import 'package:flutter_tools/src/base/io.dart';

import '../src/common.dart';
import 'test_data/project.dart';
import 'test_utils.dart';

void main() {
  late Directory tempDir;

  setUp(() async {
    tempDir = fileSystem.systemTempDirectory.createTempSync('driver_environment_test.');
  });

  tearDown(() async {
    tryToDelete(tempDir);
  });

  testWithoutContext('environment variables are passed to the drive script', () async {
    final Project project = _PrintEnvironmentVariablesInTestDriverProject();
    await project.setUpIn(tempDir);

    final ProcessResult result = await processManager.run(
      <String>[flutterBin, 'drive', '-d', 'flutter-tester'],
      workingDirectory: tempDir.path,
      environment: <String, String>{'FOO': 'BAR'},
    );

    printOnFailure('stdout: ${result.stdout}');
    printOnFailure('stderr: ${result.stderr}');
    expect(result.exitCode, 0);

    expect(result.stdout.toString(), contains('FOO=BAR'));
  });
}

final class _PrintEnvironmentVariablesInTestDriverProject extends Project {
  @override
  final String pubspec = '''
  name: test
  environment:
    sdk: ^3.7.0-0

  dependencies:
    flutter:
      sdk: flutter
  ''';

  @override
  final String main = r'void main() {}';

  @override
  Future<void> setUpIn(Directory dir) async {
    await super.setUpIn(dir);
    writeFile(fileSystem.path.join(dir.path, 'test_driver', 'main_test.dart'), r'''
import 'dart:io' as io;

void main() {
  print('FOO=${io.Platform.environment['FOO']}');
}
''');
  }
}
