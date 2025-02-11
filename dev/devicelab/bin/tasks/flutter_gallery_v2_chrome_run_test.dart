// Copyright 2014 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

import 'dart:async';
import 'dart:convert';
import 'dart:io';

import 'package:flutter_devicelab/framework/framework.dart';
import 'package:flutter_devicelab/framework/task_result.dart';
import 'package:flutter_devicelab/framework/utils.dart';

Future<void> main() async {
  await task(const NewGalleryChromeRunTest().run);
}

/// After the gallery loads, a duration of [durationToWaitForError]
/// is waited, allowing any possible exceptions to be thrown.
const Duration durationToWaitForError = Duration(seconds: 5);

/// Flutter prints this string when an app is successfully loaded.
/// Used to check when the app is successfully loaded.
const String successfullyLoadedString = 'To hot restart';

/// Flutter prints this string when an exception is caught.
/// Used to check if there are any exceptions.
const String exceptionString = 'EXCEPTION CAUGHT';

/// Checks that the New Flutter Gallery runs successfully on Chrome.
class NewGalleryChromeRunTest {
  const NewGalleryChromeRunTest();

  /// Runs the test.
  Future<TaskResult> run() async {
    final TaskResult result = await inDirectory<TaskResult>(
      '${flutterDirectory.path}/dev/integration_tests/new_gallery/',
      () async {
        await flutter(
          'create',
          options: <String>['--platforms', 'web,android,ios', '--no-overwrite', '.'],
        );
        await flutter('doctor');
        await flutter('packages', options: <String>['get']);

        await flutter('build', options: <String>['web', '-v', '--release', '--no-pub']);

        final List<String> options = <String>['-d', 'chrome', '--resident'];
        final Process process = await startFlutter('run', options: options);

        final Completer<void> stdoutDone = Completer<void>();
        final Completer<void> stderrDone = Completer<void>();

        bool success = true;

        process.stdout
            .transform<String>(utf8.decoder)
            .transform<String>(const LineSplitter())
            .listen(
              (String line) {
                if (line.contains(successfullyLoadedString)) {
                  // Successfully started.
                  Future<void>.delayed(durationToWaitForError, () {
                    process.stdin.write('q');
                  });
                }
                if (line.contains(exceptionString)) {
                  success = false;
                }
                print('stdout: $line');
              },
              onDone: () {
                stdoutDone.complete();
              },
            );

        process.stderr
            .transform<String>(utf8.decoder)
            .transform<String>(const LineSplitter())
            .listen(
              (String line) {
                print('stderr: $line');
              },
              onDone: () {
                stderrDone.complete();
              },
            );

        await Future.wait<void>(<Future<void>>[stdoutDone.future, stderrDone.future]);

        await process.exitCode;

        if (success) {
          return TaskResult.success(<String, dynamic>{});
        } else {
          return TaskResult.failure('An exception was thrown.');
        }
      },
    );

    return result;
  }
}
