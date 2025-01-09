// Copyright 2014 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

import 'dart:ffi' show Abi;
import 'dart:io' as io;

import 'package:file/file.dart';
import 'package:file_testing/file_testing.dart';
import 'package:flutter_tools/src/base/io.dart';

import '../src/common.dart';
import 'test_utils.dart';

void main() {
  late Directory tempDir;
  late Directory projectRoot;
  late Directory releaseDir;
  late File exeFile;

  group('flutter build windows command', () {
    setUpAll(() {
      tempDir = createResolvedTempDirectorySync('build_windows_test.');
      ProcessResult result = processManager.runSync(<String>[
        flutterBin,
        'config',
        '--enable-windows-desktop',
      ]);
      expect(result, const ProcessResultMatcher());

      result = processManager.runSync(<String>[
        flutterBin,
        ...getLocalEngineArguments(),
        'create',
        'hello',
      ], workingDirectory: tempDir.path);
      expect(result, const ProcessResultMatcher());

      projectRoot = tempDir.childDirectory('hello');

      final String arch;
      if (Abi.current() == Abi.windowsArm64) {
        arch = 'arm64';
      } else {
        arch = 'x64';
      }

      releaseDir = fileSystem.directory(
        fileSystem.path.join(projectRoot.path, 'build', 'windows', arch, 'runner', 'Release'),
      );

      exeFile = fileSystem.file(fileSystem.path.join(releaseDir.path, 'hello.exe'));
    });

    tearDownAll(() {
      tryToDelete(tempDir);
    });

    testWithoutContext('flutter build windows creates exe', () {
      final ProcessResult result = processManager.runSync(<String>[
        flutterBin,
        ...getLocalEngineArguments(),
        'build',
        'windows',
        '--no-pub',
      ], workingDirectory: projectRoot.path);
      expect(result, const ProcessResultMatcher());

      expect(releaseDir, exists);
      expect(exeFile, exists);

      // Default exe has build name 1.0.0 and build number 1.
      final String fileVersion = _getFileVersion(exeFile);
      final String productVersion = _getProductVersion(exeFile);

      expect(fileVersion, equals('1.0.0.1'));
      expect(productVersion, equals('1.0.0+1'));
    });

    testWithoutContext('flutter build windows sets build name', () {
      final ProcessResult result = processManager.runSync(<String>[
        flutterBin,
        ...getLocalEngineArguments(),
        'build',
        'windows',
        '--no-pub',
        '--build-name',
        '1.2.3',
      ], workingDirectory: projectRoot.path);
      expect(result, const ProcessResultMatcher());

      final String fileVersion = _getFileVersion(exeFile);
      final String productVersion = _getProductVersion(exeFile);

      expect(fileVersion, equals('1.2.3.0'));
      expect(productVersion, equals('1.2.3'));
    });

    testWithoutContext('flutter build windows sets build name and build number', () {
      final ProcessResult result = processManager.runSync(<String>[
        flutterBin,
        ...getLocalEngineArguments(),
        'build',
        'windows',
        '--no-pub',
        '--build-name',
        '1.2.3',
        '--build-number',
        '4',
      ], workingDirectory: projectRoot.path);
      expect(result, const ProcessResultMatcher());

      final String fileVersion = _getFileVersion(exeFile);
      final String productVersion = _getProductVersion(exeFile);

      expect(fileVersion, equals('1.2.3.4'));
      expect(productVersion, equals('1.2.3+4'));
    });
  }, skip: !io.Platform.isWindows); // [intended] Windows integration build.
}

String _getFileVersion(File file) {
  // FileVersionInfo's FileVersion property excludes the private part,
  // so this recreates the file version using the individual parts.
  final ProcessResult result = Process.runSync(
    'powershell.exe -command " '
    '\$v = [System.Diagnostics.FileVersionInfo]::GetVersionInfo(\\"${file.path}\\"); '
    r'Write-Output \"$($v.FileMajorPart).$($v.FileMinorPart).$($v.FileBuildPart).$($v.FilePrivatePart)\" '
    '"',
    <String>[],
  );

  expect(result, const ProcessResultMatcher());

  // Trim trailing new line.
  final String output = result.stdout as String;
  return output.trim();
}

String _getProductVersion(File file) {
  final ProcessResult result = Process.runSync(
    'powershell.exe -command "[System.Diagnostics.FileVersionInfo]::GetVersionInfo(\\"${file.path}\\").ProductVersion"',
    <String>[],
  );

  expect(result, const ProcessResultMatcher());

  // Trim trailing new line.
  final String output = result.stdout as String;
  return output.trim();
}
