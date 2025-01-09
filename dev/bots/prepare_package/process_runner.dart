// Copyright 2014 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

import 'dart:async';
import 'dart:convert';
import 'dart:io' hide Platform;

import 'package:platform/platform.dart' show LocalPlatform, Platform;
import 'package:process/process.dart';

import 'common.dart';

/// A helper class for classes that want to run a process.
///
/// The stderr and stdout can optionally be reported as the process runs, and
/// capture the stdout properly without dropping any.
class ProcessRunner {
  ProcessRunner({
    ProcessManager? processManager,
    this.subprocessOutput = true,
    this.defaultWorkingDirectory,
    this.platform = const LocalPlatform(),
  }) : processManager = processManager ?? const LocalProcessManager() {
    environment = Map<String, String>.from(platform.environment);
  }

  /// The platform to use for a starting environment.
  final Platform platform;

  /// Set [subprocessOutput] to show output as processes run. Stdout from the
  /// process will be printed to stdout, and stderr printed to stderr.
  final bool subprocessOutput;

  /// Set the [processManager] in order to inject a test instance to perform
  /// testing.
  final ProcessManager processManager;

  /// Sets the default directory used when `workingDirectory` is not specified
  /// to [runProcess].
  final Directory? defaultWorkingDirectory;

  /// The environment to run processes with.
  late Map<String, String> environment;

  /// Run the command and arguments in `commandLine` as a sub-process from
  /// `workingDirectory` if set, or the [defaultWorkingDirectory] if not. Uses
  /// [Directory.current] if [defaultWorkingDirectory] is not set.
  ///
  /// Set `failOk` if [runProcess] should not throw an exception when the
  /// command completes with a non-zero exit code.
  Future<String> runProcess(
    List<String> commandLine, {
    Directory? workingDirectory,
    bool failOk = false,
  }) async {
    workingDirectory ??= defaultWorkingDirectory ?? Directory.current;
    if (subprocessOutput) {
      stderr.write('Running "${commandLine.join(' ')}" in ${workingDirectory.path}.\n');
    }
    final List<int> output = <int>[];
    final Completer<void> stdoutComplete = Completer<void>();
    final Completer<void> stderrComplete = Completer<void>();
    late Process process;
    Future<int> allComplete() async {
      await stderrComplete.future;
      await stdoutComplete.future;
      return process.exitCode;
    }

    try {
      process = await processManager.start(
        commandLine,
        workingDirectory: workingDirectory.absolute.path,
        environment: environment,
      );
      process.stdout.listen((List<int> event) {
        output.addAll(event);
        if (subprocessOutput) {
          stdout.add(event);
        }
      }, onDone: () async => stdoutComplete.complete());
      if (subprocessOutput) {
        process.stderr.listen((List<int> event) {
          stderr.add(event);
        }, onDone: () async => stderrComplete.complete());
      } else {
        stderrComplete.complete();
      }
    } on ProcessException catch (e) {
      final String message =
          'Running "${commandLine.join(' ')}" in ${workingDirectory.path} '
          'failed with:\n$e';
      throw PreparePackageException(message);
    } on ArgumentError catch (e) {
      final String message =
          'Running "${commandLine.join(' ')}" in ${workingDirectory.path} '
          'failed with:\n$e';
      throw PreparePackageException(message);
    }

    final int exitCode = await allComplete();
    if (exitCode != 0 && !failOk) {
      final String message =
          'Running "${commandLine.join(' ')}" in ${workingDirectory.path} failed';
      throw PreparePackageException(
        message,
        ProcessResult(0, exitCode, null, 'returned $exitCode'),
      );
    }
    return utf8.decoder.convert(output).trim();
  }
}
