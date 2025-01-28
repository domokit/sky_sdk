// Copyright 2014 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

import 'dart:io' as io;

import 'package:path/path.dart' as p;

/// To run this script, either:
///
/// ```sh
/// cd dev/customer_testing
/// dart ci.dart [sha]
/// ```
///
/// Or:
///
/// ```sh
/// ./dev/customer_testing/ci.sh
/// ./dev/customer_testing/ci.bat
/// ```
void main(List<String> args) async {
  final String sha;
  if (args.isEmpty) {
    sha = io.File('tests.version').readAsStringSync().trim();
  } else if (args.length == 1) {
    sha = args.first;
  } else {
    io.stderr.writeln('Usage: dart ci.dart [sha]');
    io.exitCode = 1;
    return;
  }

  final String flutterRootPath = p.canonicalize('../../');
  final io.Directory testsCacheDir = io.Directory(
    p.join(flutterRootPath, 'bin', 'cache', 'pkg', 'tests'),
  );

  if (testsCacheDir.existsSync()) {
    io.stderr.writeln('Cleaning up existing repo: ${testsCacheDir.path}');
    testsCacheDir.deleteSync(recursive: true);
  }

  io.stderr.writeln('Cloning flutter/tests');
  final io.Process clone = await io.Process.start('git', <String>[
    'clone',
    '--depth',
    '1',
    'https://github.com/flutter/tests.git',
    testsCacheDir.path,
  ], mode: io.ProcessStartMode.inheritStdio);
  if ((await clone.exitCode) != 0) {
    io.exitCode = 1;
    return;
  }

  io.stderr.writeln('Fetching/checking out $sha');
  final io.Process fetch = await io.Process.start(
    'git',
    <String>['fetch', 'origin', sha],
    mode: io.ProcessStartMode.inheritStdio,
    workingDirectory: testsCacheDir.path,
  );
  if ((await fetch.exitCode) != 0) {
    io.exitCode = 1;
    return;
  }
  final io.Process checkout = await io.Process.start(
    'git',
    <String>['checkout', sha],
    mode: io.ProcessStartMode.inheritStdio,
    workingDirectory: testsCacheDir.path,
  );
  if ((await checkout.exitCode) != 0) {
    io.exitCode = 1;
    return;
  }

  io.stderr.writeln('Running tests...');
  final io.Process test = await io.Process.start('dart', <String>[
    '--enable-asserts',
    'run_tests.dart',
    '--skip-on-fetch-failure',
    '--skip-template',
    p.posix.joinAll(<String>[...p.split(testsCacheDir.path), 'registry', '*.test']),
  ], mode: io.ProcessStartMode.inheritStdio);
  if ((await test.exitCode) != 0) {
    io.exitCode = 1;
    return;
  }
}
