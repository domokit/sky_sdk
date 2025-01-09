// Copyright 2014 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

import 'package:file/file.dart';
import 'package:flutter_tools/src/base/io.dart';
import 'package:flutter_tools/src/base/logger.dart';
import 'package:flutter_tools/src/base/os.dart';

import '../integration.shard/test_utils.dart';
import '../src/common.dart';

Future<void> main() async {
  test('verify the dart binary arch matches the host arch', () async {
    final HostPlatform dartArch = _identifyMacBinaryArch(_dartBinary.path);
    final OperatingSystemUtils os = OperatingSystemUtils(
      processManager: processManager,
      fileSystem: fileSystem,
      platform: platform,
      logger: BufferLogger.test(),
    );
    expect(dartArch, os.hostPlatform);
  }, skip: !platform.isMacOS); // [intended] Calls macOS-specific commands
}

// Call `file` on the path and parse the output.
HostPlatform _identifyMacBinaryArch(String path) {
  // Expect STDOUT like:
  //   bin/cache/dart-sdk/bin/dart: Mach-O 64-bit executable x86_64
  final RegExp pattern = RegExp(r'Mach-O 64-bit executable (\w+)');
  final ProcessResult result = processManager.runSync(<String>['file', _dartBinary.path]);
  expect(
    result,
    ProcessResultMatcher(stdoutPattern: '${_dartBinary.path}: Mach-O 64-bit executable'),
  );
  final RegExpMatch? match = pattern.firstMatch(result.stdout as String);
  if (match == null) {
    fail('Unrecognized STDOUT from `file`: "${result.stdout}"');
  }
  switch (match.group(1)) {
    case 'x86_64':
      return HostPlatform.darwin_x64;
    case 'arm64':
      return HostPlatform.darwin_arm64;
    default:
      fail('Unexpected architecture ${match.group(1)}');
  }
}

final String _flutterRootPath = getFlutterRoot();
final Directory _flutterRoot = fileSystem.directory(_flutterRootPath);
final File _dartBinary =
    _flutterRoot
        .childDirectory('bin')
        .childDirectory('cache')
        .childDirectory('dart-sdk')
        .childDirectory('bin')
        .childFile('dart')
        .absolute;
