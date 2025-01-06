// Copyright 2014 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

import 'dart:async';
import 'dart:convert';

import 'package:file/file.dart';
import 'package:flutter_tools/src/base/io.dart';

import '../src/common.dart';
import 'test_utils.dart';

final String flutterRootPath = getFlutterRoot();
final Directory flutterRoot = fileSystem.directory(flutterRootPath);

Future<void> main() async {
  test(
    'verify terminating flutter/bin/dart terminates the underlying dart process',
    () async {
      final Completer<void> childReadyCompleter = Completer<void>();
      String stdout = '';
      final Process process = await processManager.start(<String>[
        dartBash.path,
        listenForSigtermScript.path,
      ]);
      final Future<Object?> stdoutFuture = process.stdout.transform<String>(utf8.decoder).forEach((
        String str,
      ) {
        stdout += str;
        if (stdout.contains('Ready to receive signals') && !childReadyCompleter.isCompleted) {
          childReadyCompleter.complete();
        }
      });
      // Ensure that the child app has registered its signal handler
      await childReadyCompleter.future;
      final bool killSuccess = process.kill();
      expect(killSuccess, true);
      // Wait for stdout to complete
      await stdoutFuture;
      // Ensure child exited successfully
      expect(
        await process.exitCode,
        0,
        reason:
            'child process exited with code ${await process.exitCode}, and '
            'stdout:\n$stdout',
      );
      expect(stdout, contains('Successfully received SIGTERM!'));
    },
    // [intended] Windows does not use the bash entrypoint
    skip: platform.isWindows,
  );

  test('shared.sh does not compile flutter tool if PROG_NAME=dart', () async {
    final Directory tempDir = fileSystem.systemTempDirectory.createTempSync('bash_entrypoint_test');
    try {
      // bash script checks it is in a git repo
      ProcessResult result = await processManager.run(<String>[
        'git',
        'init',
      ], workingDirectory: tempDir.path);
      expect(result, const ProcessResultMatcher());
      result = await processManager.run(<String>[
        'git',
        'commit',
        '--allow-empty',
        '-m',
        'init commit',
      ], workingDirectory: tempDir.path);
      expect(result, const ProcessResultMatcher());

      // copy dart and shared.sh to temp dir
      final File trueSharedSh = flutterRoot
          .childDirectory('bin')
          .childDirectory('internal')
          .childFile('shared.sh');
      final File fakeSharedSh = (tempDir.childDirectory('bin').childDirectory('internal')
        ..createSync(recursive: true)).childFile('shared.sh');
      trueSharedSh.copySync(fakeSharedSh.path);
      final File fakeDartBash = tempDir.childDirectory('bin').childFile('dart');
      dartBash.copySync(fakeDartBash.path);
      // mark dart executable
      makeExecutable(fakeDartBash);

      // create no-op fake update_dart_sdk.sh script
      final File updateDartSdk = tempDir
        .childDirectory('bin')
        .childDirectory('internal')
        .childFile('update_dart_sdk.sh')..writeAsStringSync('''
#!/usr/bin/env bash

echo downloaded dart sdk
''');
      makeExecutable(updateDartSdk);

      final File udpateEngine = tempDir
        .childDirectory('bin')
        .childDirectory('internal')
        .childFile('update_engine_version.sh')..writeAsStringSync('''
#!/usr/bin/env bash

echo engine version
''');
      makeExecutable(udpateEngine);

      // create a fake dart runtime
      final File dartBin = (tempDir
        .childDirectory('bin')
        .childDirectory('cache')
        .childDirectory('dart-sdk')
        .childDirectory('bin')..createSync(recursive: true)).childFile('dart');
      dartBin.writeAsStringSync('''
#!/usr/bin/env bash

echo executed dart binary
''');
      makeExecutable(dartBin);

      result = await processManager.run(<String>[fakeDartBash.path]);
      expect(result, const ProcessResultMatcher());
      expect(
        (result.stdout as String).split('\n'),
        // verify we ran updateDartSdk and dartBin
        containsAll(<String>['downloaded dart sdk', 'executed dart binary']),
      );

      // Verify we did not try to compile the flutter_tool
      expect(result.stderr, isNot(contains('Building flutter tool...')));
    } finally {
      tryToDelete(tempDir);
    }
  }, skip: true); // [intended] https://github.com/flutter/flutter/issues/160689
}

// A test Dart app that will run until it receives SIGTERM
File get listenForSigtermScript {
  return flutterRoot
      .childDirectory('packages')
      .childDirectory('flutter_tools')
      .childDirectory('test')
      .childDirectory('integration.shard')
      .childDirectory('test_data')
      .childFile('listen_for_sigterm.dart')
      .absolute;
}

// The executable bash entrypoint for the Dart binary.
File get dartBash {
  return flutterRoot.childDirectory('bin').childFile('dart').absolute;
}

void makeExecutable(File file) {
  final ProcessResult result = processManager.runSync(<String>['chmod', '+x', file.path]);
  expect(result, const ProcessResultMatcher());
}
