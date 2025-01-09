// Copyright 2013 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

import 'package:args/command_runner.dart';
import 'package:engine_build_configs/engine_build_configs.dart';

import '../environment.dart';
import 'build_command.dart';
import 'cleanup_command.dart';
import 'fetch_command.dart';
import 'flags.dart';
import 'format_command.dart';
import 'lint_command.dart';
import 'query_command.dart';
import 'run_command.dart';
import 'test_command.dart';

const int _usageLineLength = 100;

/// The root command runner.
final class ToolCommandRunner extends CommandRunner<int> {
  /// Constructs the runner and populates commands, subcommands, and global
  /// options and flags.
  ToolCommandRunner({required this.environment, required this.configs, this.help = false})
    : super(
        'et',
        ''
            'A command line tool for working on '
            'the Flutter Engine.\n\nThis is a community supported project, '
            'for more information see https://flutter.dev/to/et.',
        usageLineLength: _usageLineLength,
      ) {
    final List<Command<int>> commands = <Command<int>>[
      CleanupCommand(environment: environment, usageLineLength: _usageLineLength),
      FetchCommand(environment: environment, usageLineLength: _usageLineLength),
      FormatCommand(environment: environment, usageLineLength: _usageLineLength),
      QueryCommand(
        environment: environment,
        configs: configs,
        help: help,
        usageLineLength: _usageLineLength,
      ),
      BuildCommand(
        environment: environment,
        configs: configs,
        help: help,
        usageLineLength: _usageLineLength,
      ),
      RunCommand(environment: environment, configs: configs, usageLineLength: _usageLineLength),
      LintCommand(environment: environment, usageLineLength: _usageLineLength),
      TestCommand(
        environment: environment,
        configs: configs,
        help: help,
        usageLineLength: _usageLineLength,
      ),
    ];
    commands.forEach(addCommand);

    argParser.addFlag(verboseFlag, abbr: 'v', help: 'Prints verbose output', negatable: false);
  }

  /// The host system environment.
  final Environment environment;

  /// Build configurations loaded from the engine from under ci/builders.
  final Map<String, BuilderConfig> configs;

  /// Whether the invocation is for a help command
  final bool help;

  @override
  Future<int> run(Iterable<String> args) async {
    try {
      return await runCommand(parse(args)) ?? 0;
    } on FormatException catch (e, s) {
      environment.logger.error('$e\n$s');
      return 1;
    } on UsageException catch (e) {
      environment.logger.error(e);
      return 1;
    }
  }

  @override
  void printUsage() {
    environment.logger.status(usage);
  }
}
