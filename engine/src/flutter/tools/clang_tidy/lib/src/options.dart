// Copyright 2013 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

import 'dart:io' as io show Directory, File, Platform, stderr;

import 'package:args/args.dart';
import 'package:engine_repo_tools/engine_repo_tools.dart';
import 'package:path/path.dart' as path;

import 'lint_target.dart';

final Engine _engineRoot = Engine.findWithin();

/// Adds warnings as errors for only specific runs.  This is helpful if migrating one platform at a time.
String? _platformSpecificWarningsAsErrors(ArgResults options) {
  if (options['target-variant'] == 'host_debug' && io.Platform.isMacOS) {
    return options['mac-host-warnings-as-errors'] as String?;
  }
  return null;
}

/// A class for organizing the options to the Engine linter, and the files
/// that it operates on.
class Options {
  /// Builds an instance of [Options] from the arguments.
  Options({
    required this.buildCommandsPath,
    this.help = false,
    this.verbose = false,
    this.checksArg = '',
    this.lintTarget = const LintChanged(),
    this.fix = false,
    this.errorMessage,
    this.warningsAsErrors,
    this.shardId,
    this.shardCommandsPaths = const <io.File>[],
    this.enableCheckProfile = false,
    StringSink? errSink,
    this.clangTidyPath,
  }) : checks = checksArg.isNotEmpty ? '--checks=$checksArg' : null,
       _errSink = errSink ?? io.stderr;

  factory Options._error(String message, {StringSink? errSink}) {
    return Options(errorMessage: message, buildCommandsPath: io.File('none'), errSink: errSink);
  }

  factory Options._help({StringSink? errSink}) {
    return Options(help: true, buildCommandsPath: io.File('none'), errSink: errSink);
  }

  /// Builds an [Options] instance with an [ArgResults] instance.
  factory Options._fromArgResults(
    ArgResults options, {
    required io.File buildCommandsPath,
    StringSink? errSink,
    required List<io.File> shardCommandsPaths,
    int? shardId,
    io.File? clangTidyPath,
  }) {
    final LintTarget lintTarget;
    if (options.wasParsed('lint-all') || io.Platform.environment['FLUTTER_LINT_ALL'] != null) {
      lintTarget = const LintAll();
    } else if (options.wasParsed('lint-regex')) {
      lintTarget = LintRegex(options['lint-regex'] as String? ?? '');
    } else if (options.wasParsed('lint-head')) {
      lintTarget = const LintHead();
    } else {
      lintTarget = const LintChanged();
    }
    return Options(
      help: options['help'] as bool,
      verbose: options['verbose'] as bool,
      buildCommandsPath: buildCommandsPath,
      checksArg: options.wasParsed('checks') ? options['checks'] as String : '',
      lintTarget: lintTarget,
      fix: options['fix'] as bool,
      errSink: errSink,
      warningsAsErrors: _platformSpecificWarningsAsErrors(options),
      shardCommandsPaths: shardCommandsPaths,
      shardId: shardId,
      enableCheckProfile: options['enable-check-profile'] as bool,
      clangTidyPath: clangTidyPath,
    );
  }

  /// Builds an instance of [Options] from the given `arguments`.
  factory Options.fromCommandLine(List<String> arguments, {StringSink? errSink, Engine? engine}) {
    // TODO(matanlurey): Refactor this further, ideally moving all of the engine
    // resolution logic (i.e. --src-dir, --target-variant, --compile-commands)
    // into a separate method, and perhaps also adding `engine.output(name)`
    // to engine_repo_tools instead of path manipulation inlined below.
    final ArgResults argResults = _argParser(defaultEngine: engine).parse(arguments);

    String? buildCommandsPath = argResults['compile-commands'] as String?;

    String variantToBuildCommandsFilePath(String variant) =>
        path.join(argResults['src-dir'] as String, 'out', variant, 'compile_commands.json');
    // path/to/engine/src/out/variant/compile_commands.json
    buildCommandsPath ??= variantToBuildCommandsFilePath(argResults['target-variant'] as String);
    final io.File buildCommands = io.File(buildCommandsPath);
    final List<io.File> shardCommands =
        (argResults['shard-variants'] as String? ?? '')
            .split(',')
            .where((String element) => element.isNotEmpty)
            .map((String variant) => io.File(variantToBuildCommandsFilePath(variant)))
            .toList();
    final String? message = _checkArguments(argResults, buildCommands);
    if (message != null) {
      return Options._error(message, errSink: errSink);
    }
    if (argResults['help'] as bool) {
      return Options._help(errSink: errSink);
    }
    final String? shardIdString = argResults['shard-id'] as String?;
    final int? shardId = shardIdString == null ? null : int.parse(shardIdString);
    if (shardId != null && (shardId > shardCommands.length || shardId < 0)) {
      return Options._error('Invalid shard-id value: $shardId.', errSink: errSink);
    }
    final io.File? clangTidyPath = ((String? path) => path == null ? null : io.File(path))(
      argResults['clang-tidy'] as String?,
    );
    return Options._fromArgResults(
      argResults,
      buildCommandsPath: buildCommands,
      errSink: errSink,
      shardCommandsPaths: shardCommands,
      shardId: shardId,
      clangTidyPath: clangTidyPath,
    );
  }

  static ArgParser _argParser({required Engine? defaultEngine}) {
    defaultEngine ??= _engineRoot;
    final io.Directory? latestBuild = defaultEngine.latestOutput()?.path;
    return ArgParser()
      ..addFlag('help', abbr: 'h', help: 'Print help.', negatable: false)
      ..addOption(
        'lint-regex',
        help:
            'Lint all files, regardless of FLUTTER_NOLINT. Provide a regex '
            'to filter files. For example, `--lint-regex=".*impeller.*"` will '
            'lint all files within a path that contains "impeller".',
        valueHelp: 'regex',
      )
      ..addFlag('lint-all', help: 'Lint all files, regardless of FLUTTER_NOLINT.')
      ..addFlag('lint-head', help: 'Lint files changed in the tip-of-tree commit.')
      ..addFlag('fix', help: 'Apply suggested fixes.')
      ..addFlag('verbose', help: 'Print verbose output.')
      ..addOption(
        'shard-id',
        help: 'When used with the shard-commands option this identifies which shard will execute.',
        valueHelp: 'A number less than 1 + the number of shard-commands arguments.',
      )
      ..addOption(
        'shard-variants',
        help:
            'Comma separated list of other targets, this invocation '
            'will only execute a subset of the intersection and the difference of the '
            'compile commands. Use with `shard-id`.',
      )
      ..addOption(
        'compile-commands',
        help:
            'Use the given path as the source of compile_commands.json. This '
            'file is created by running "tools/gn". Cannot be used with --target-variant '
            'or --src-dir.',
      )
      ..addOption(
        'target-variant',
        aliases: <String>['variant'],
        help:
            'The engine variant directory name containing compile_commands.json '
            'created by running "tools/gn".\n\nIf not provided, the default is '
            'the latest build in the engine defined by --src-dir (or the '
            'default path, see --src-dir for details).\n\n'
            'Cannot be used with --compile-commands.',
        valueHelp: 'host_debug|android_debug_unopt|ios_debug|ios_debug_sim_unopt',
        defaultsTo: latestBuild == null ? 'host_debug' : path.basename(latestBuild.path),
      )
      ..addOption(
        'mac-host-warnings-as-errors',
        help: 'checks that will be treated as errors when running debug_host on mac.',
      )
      ..addOption(
        'src-dir',
        help:
            'Path to the engine src directory.\n\n'
            'If not provided, the default is the engine root directory that '
            'contains the `clang_tidy` tool.\n\n'
            'Cannot be used with --compile-commands.',
        valueHelp: 'path/to/engine/src',
        defaultsTo: _engineRoot.srcDir.path,
      )
      ..addOption(
        'checks',
        help:
            'Perform the given checks on the code. Defaults to the empty '
            'string, indicating all checks should be performed.',
        defaultsTo: '',
      )
      ..addOption(
        'clang-tidy',
        help:
            'Path to the clang-tidy executable. Defaults to deriving the path\n'
            'from compile_commands.json.',
      )
      ..addFlag(
        'enable-check-profile',
        help: 'Enable per-check timing profiles and print a report to stderr.',
        negatable: false,
      );
  }

  /// Whether to print a help message and exit.
  final bool help;

  /// Whether to run with verbose output.
  final bool verbose;

  /// The location of the compile_commands.json file.
  final io.File buildCommandsPath;

  /// The location of shard compile_commands.json files.
  final List<io.File> shardCommandsPaths;

  /// The identifier of the shard.
  final int? shardId;

  /// The root of the flutter/engine repository.
  final io.Directory repoPath = _engineRoot.flutterDir;

  /// Argument sent as `warnings-as-errors` to clang-tidy.
  final String? warningsAsErrors;

  /// Checks argument as supplied to the command-line.
  final String checksArg;

  /// Check argument to be supplied to the clang-tidy subprocess.
  final String? checks;

  /// What files to lint.
  final LintTarget lintTarget;

  /// Whether checks should apply available fix-ups to the working copy.
  final bool fix;

  /// Whether to enable per-check timing profiles and print a report to stderr.
  final bool enableCheckProfile;

  /// If there was a problem with the command line arguments, this string
  /// contains the error message.
  final String? errorMessage;

  final StringSink _errSink;

  /// Override for which clang-tidy to use. If it is null it will be derived
  /// instead.
  final io.File? clangTidyPath;

  /// Print command usage with an additional message.
  void printUsage({String? message, required Engine? engine}) {
    if (message != null) {
      _errSink.writeln(message);
    }
    _errSink.writeln(
      'Usage: bin/main.dart [--help] [--lint-all] [--lint-head] [--fix] [--verbose] '
      '[--diff-branch] [--target-variant variant] [--src-dir path/to/engine/src]',
    );
    _errSink.writeln(_argParser(defaultEngine: engine).usage);
  }

  /// Command line argument validation.
  static String? _checkArguments(ArgResults argResults, io.File buildCommandsPath) {
    if (argResults.wasParsed('help')) {
      return null;
    }

    final bool compileCommandsParsed = argResults.wasParsed('compile-commands');
    if (compileCommandsParsed && argResults.wasParsed('target-variant')) {
      return 'ERROR: --compile-commands option cannot be used with --target-variant.';
    }

    if (compileCommandsParsed && argResults.wasParsed('src-dir')) {
      return 'ERROR: --compile-commands option cannot be used with --src-dir.';
    }

    if (const <String>['lint-all', 'lint-head', 'lint-regex'].where(argResults.wasParsed).length >
        1) {
      return 'ERROR: At most one of --lint-all, --lint-head, --lint-regex can be passed.';
    }

    if (!buildCommandsPath.existsSync()) {
      return "ERROR: Build commands path ${buildCommandsPath.absolute.path} doesn't exist.";
    }

    if (argResults.wasParsed('shard-variants') && !argResults.wasParsed('shard-id')) {
      return 'ERROR: a `shard-id` must be specified with `shard-variants`.';
    }

    return null;
  }
}
