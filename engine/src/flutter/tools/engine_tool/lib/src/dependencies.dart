// Copyright 2013 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

import 'dart:io' as io;

import 'package:process_runner/process_runner.dart';

import 'environment.dart';
import 'logger.dart';

/// Update Flutter engine dependencies. Returns an exit code.
Future<int> fetchDependencies(Environment environment) async {
  if (!environment.processRunner.processManager.canRun('gclient')) {
    environment.logger.error('Cannot find the gclient command in your path');
    return 1;
  }

  environment.logger.status('Fetching dependencies... ', newline: environment.verbose);

  Spinner? spinner;
  ProcessRunnerResult result;
  try {
    if (!environment.verbose) {
      spinner = environment.logger.startSpinner();
    }

    result = await environment.processRunner.runProcess(
      <String>['gclient', 'sync', '-D'],
      runInShell: true,
      startMode:
          environment.verbose ? io.ProcessStartMode.inheritStdio : io.ProcessStartMode.normal,
    );
  } finally {
    spinner?.finish();
  }

  if (result.exitCode != 0) {
    environment.logger.error('Fetching dependencies failed.');

    // Verbose mode already logged output by making the child process inherit
    // this process's stdio handles.
    if (!environment.verbose) {
      environment.logger.error('Output:\n${result.output}');
    }
  }

  return result.exitCode;
}
