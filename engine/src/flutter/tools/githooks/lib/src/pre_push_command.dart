// Copyright 2013 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

import 'dart:io' as io;

import 'package:args/command_runner.dart';
import 'package:clang_tidy/clang_tidy.dart';
import 'package:path/path.dart' as path;

/// The command that implements the pre-push githook
class PrePushCommand extends Command<bool> {
  @override
  final String name = 'pre-push';

  @override
  final String description = 'Checks to run before a "git push"';

  @override
  Future<bool> run() async {
    final Stopwatch sw = Stopwatch()..start();
    final bool verbose = globalResults!['verbose']! as bool;
    final bool enableClangTidy = globalResults!['enable-clang-tidy']! as bool;
    final String flutterRoot = globalResults!['flutter']! as String;

    if (!enableClangTidy) {
      print(
        'The clang-tidy check is disabled. To enable set the environment '
        'variable PRE_PUSH_CLANG_TIDY to any value.',
      );
    }

    final List<bool> checkResults = <bool>[
      await _runFormatter(flutterRoot, verbose),
      if (enableClangTidy) await _runClangTidy(flutterRoot, verbose),
    ];
    sw.stop();
    io.stdout.writeln('pre-push checks finished in ${sw.elapsed}');
    return !checkResults.contains(false);
  }

  Future<bool> _runClangTidy(String flutterRoot, bool verbose) async {
    io.stdout.writeln('Starting clang-tidy checks.');
    final Stopwatch sw = Stopwatch()..start();
    // First ensure that out/host_debug/compile_commands.json exists by running
    // //flutter/tools/gn.
    io.File compileCommands = io.File(
      path.join(flutterRoot, '..', 'out', 'host_debug', 'compile_commands.json'),
    );
    if (!compileCommands.existsSync()) {
      compileCommands = io.File(
        path.join(flutterRoot, '..', 'out', 'host_debug_unopt', 'compile_commands.json'),
      );
      if (!compileCommands.existsSync()) {
        io.stderr.writeln(
          'clang-tidy requires a fully built host_debug or '
          'host_debug_unopt build directory',
        );
        return false;
      }
    }
    final StringBuffer outBuffer = StringBuffer();
    final StringBuffer errBuffer = StringBuffer();
    final ClangTidy clangTidy = ClangTidy(
      buildCommandsPath: compileCommands,
      outSink: outBuffer,
      errSink: errBuffer,
    );
    final int clangTidyResult = await clangTidy.run();
    sw.stop();
    io.stdout.writeln('clang-tidy checks finished in ${sw.elapsed}');
    if (clangTidyResult != 0) {
      io.stderr.write(errBuffer);
      return false;
    }
    return true;
  }

  Future<bool> _runFormatter(String flutterRoot, bool verbose) async {
    io.stdout.writeln('Starting formatting checks.');
    final Stopwatch sw = Stopwatch()..start();
    final String ext = io.Platform.isWindows ? '.bat' : '.sh';
    final bool result = await _runCheck(
      flutterRoot,
      path.join(flutterRoot, 'ci', 'format$ext'),
      <String>[],
      'Formatting check',
      verbose: verbose,
    );
    sw.stop();
    io.stdout.writeln('formatting checks finished in ${sw.elapsed}');
    return result;
  }

  Future<bool> _runCheck(
    String flutterRoot,
    String scriptPath,
    List<String> scriptArgs,
    String checkName, {
    bool verbose = false,
  }) async {
    if (verbose) {
      io.stdout.writeln('Starting "$checkName": $scriptPath');
    }
    final io.ProcessResult result = await io.Process.run(
      scriptPath,
      scriptArgs,
      workingDirectory: flutterRoot,
    );
    if (result.exitCode != 0) {
      final StringBuffer message = StringBuffer();
      message.writeln('Check "$checkName" failed.');
      message.writeln('command: $scriptPath ${scriptArgs.join(" ")}');
      message.writeln('working directory: $flutterRoot');
      message.writeln('exit code: ${result.exitCode}');
      message.writeln('stdout:');
      message.writeln(result.stdout);
      message.writeln('stderr:');
      message.writeln(result.stderr);
      io.stderr.write(message.toString());
      return false;
    }
    if (verbose) {
      io.stdout.writeln('Check "$checkName" finished successfully.');
    }
    return true;
  }
}
