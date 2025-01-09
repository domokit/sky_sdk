// Copyright 2013 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

import 'dart:io' as io;

import 'package:args/args.dart';
import 'package:args/command_runner.dart';

import 'src/post_checkout_command.dart';
import 'src/post_merge_command.dart';
import 'src/pre_push_command.dart';
import 'src/pre_rebase_command.dart';

/// Runs the githooks
Future<int> run(List<String> args) async {
  final CommandRunner<bool> runner =
      CommandRunner<bool>('githooks', 'Githooks implementation for the flutter/engine repo.')
        ..addCommand(PostCheckoutCommand())
        ..addCommand(PostMergeCommand())
        ..addCommand(PrePushCommand())
        ..addCommand(PreRebaseCommand());

  // Add top-level arguments.
  runner.argParser
    ..addFlag('enable-clang-tidy', help: 'Enable running clang-tidy on changed files.')
    ..addOption(
      'flutter',
      abbr: 'f',
      help: 'The absolute path to the root of the flutter/engine checkout.',
    )
    ..addFlag('verbose', abbr: 'v', help: 'Runs with verbose logging');

  if (args.isEmpty) {
    // The tool was invoked with no arguments. Print usage.
    runner.printUsage();
    return 1;
  }

  final ArgResults argResults = runner.parse(args);
  final String? argMessage = _checkArgs(argResults);
  if (argMessage != null) {
    io.stderr.writeln(argMessage);
    runner.printUsage();
    return 1;
  }

  final bool commandResult = await runner.runCommand(argResults) ?? false;
  return commandResult ? 0 : 1;
}

String? _checkArgs(ArgResults argResults) {
  if (argResults.command?.name == 'help') {
    return null;
  }

  if (argResults['help'] as bool) {
    return null;
  }

  if (argResults['flutter'] == null) {
    return 'The --flutter option is required';
  }

  final io.Directory dir = io.Directory(argResults['flutter'] as String);
  if (!dir.isAbsolute) {
    return 'The --flutter option must be an absolute path';
  }

  if (!dir.existsSync()) {
    return 'The directory specified by the --flutter option must exist';
  }

  return null;
}
