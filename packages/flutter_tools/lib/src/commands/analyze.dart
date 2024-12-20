// Copyright 2014 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

import 'package:meta/meta.dart';
import 'package:process/process.dart';

import '../artifacts.dart';
import '../base/common.dart';
import '../base/file_system.dart';
import '../base/logger.dart';
import '../base/platform.dart';
import '../base/terminal.dart';
import '../project.dart';
import '../project_validator.dart';
import '../runner/flutter_command.dart';
import 'analyze_base.dart';
import 'analyze_continuously.dart';
import 'analyze_once.dart';
import 'android_analyze.dart';
import 'ios_analyze.dart';
import 'validate_project.dart';

class AnalyzeCommand extends FlutterCommand {
  AnalyzeCommand({
    bool verboseHelp = false,
    this.workingDirectory,
    required FileSystem fileSystem,
    required Platform platform,
    required Terminal terminal,
    required Logger logger,
    required ProcessManager processManager,
    required Artifacts artifacts,
    required List<ProjectValidator> allProjectValidators,
    required bool suppressAnalytics,
  }) : _artifacts = artifacts,
       _fileSystem = fileSystem,
       _processManager = processManager,
       _logger = logger,
       _terminal = terminal,
       _allProjectValidators = allProjectValidators,
       _platform = platform,
       _suppressAnalytics = suppressAnalytics {
    argParser.addFlag(
      'flutter-repo',
      negatable: false,
      help: 'Include all the examples and tests from the Flutter repository.',
      hide: !verboseHelp,
    );
    argParser.addFlag(
      'current-package',
      help: 'Analyze the current project, if applicable.',
      defaultsTo: true,
    );
    argParser.addFlag(
      'dartdocs',
      negatable: false,
      help:
          '(deprecated) List every public member that is lacking documentation. '
          'This command will be removed in a future version of Flutter.',
      hide: !verboseHelp,
    );
    argParser.addFlag(
      'watch',
      help: 'Run analysis continuously, watching the filesystem for changes.',
      negatable: false,
    );
    argParser.addOption(
      'write',
      valueHelp: 'file',
      help:
          'Also output the results to a file. This is useful with "--watch" '
          'if you want a file to always contain the latest results.',
    );
    argParser.addOption(
      'dart-sdk',
      valueHelp: 'path-to-sdk',
      help: 'The path to the Dart SDK.',
      hide: !verboseHelp,
    );
    argParser.addOption(
      'protocol-traffic-log',
      valueHelp: 'path-to-protocol-traffic-log',
      help:
          'The path to write the request and response protocol. This is '
          'only intended to be used for debugging the tooling.',
      hide: !verboseHelp,
    );
    argParser.addFlag('suggestions', help: 'Show suggestions about the current flutter project.');
    argParser.addFlag(
      'machine',
      negatable: false,
      help:
          'Dumps a JSON with a subset of relevant data about the tool, project, '
          'and environment.',
      hide: !verboseHelp,
    );

    // Hidden option to enable a benchmarking mode.
    argParser.addFlag(
      'benchmark',
      negatable: false,
      hide: !verboseHelp,
      help: 'Also output the analysis time.',
    );

    usesPubOption();

    // Not used by analyze --watch
    argParser.addFlag(
      'congratulate',
      help:
          'Show output even when there are no errors, warnings, hints, or lints. '
          'Ignored if "--watch" is specified.',
      defaultsTo: true,
    );
    argParser.addFlag(
      'preamble',
      defaultsTo: true,
      help:
          'When analyzing the flutter repository, display the number of '
          'files that will be analyzed.\n'
          'Ignored if "--watch" is specified.',
    );
    argParser.addFlag('fatal-infos', help: 'Treat info level issues as fatal.', defaultsTo: true);
    argParser.addFlag(
      'fatal-warnings',
      help: 'Treat warning level issues as fatal.',
      defaultsTo: true,
    );

    argParser.addFlag(
      'android',
      negatable: false,
      help: 'Analyze Android sub-project. Used by internal tools only.',
      hide: !verboseHelp,
    );

    argParser.addFlag(
      'ios',
      negatable: false,
      help: 'Analyze iOS Xcode sub-project. Used by internal tools only.',
      hide: !verboseHelp,
    );

    if (verboseHelp) {
      argParser.addSeparator('Usage: flutter analyze --android [arguments]');
    }

    argParser.addFlag(
      'list-build-variants',
      negatable: false,
      help:
          'Print out a list of available build variants for the '
          'Android sub-project.',
      hide: !verboseHelp,
    );

    argParser.addFlag(
      'output-app-link-settings',
      negatable: false,
      help:
          'Output a JSON with Android app link settings into a file. '
          'The "--build-variant" must also be set.',
      hide: !verboseHelp,
    );

    argParser.addOption(
      'build-variant',
      help: 'Sets the Android build variant to be analyzed.',
      valueHelp: 'build variant',
      hide: !verboseHelp,
    );

    if (verboseHelp) {
      argParser.addSeparator('Usage: flutter analyze --ios [arguments]');
    }

    argParser.addFlag(
      'list-build-options',
      help:
          'Print out a list of available build options for the '
          'iOS Xcode sub-project.',
      hide: !verboseHelp,
    );

    argParser.addFlag(
      'output-universal-link-settings',
      negatable: false,
      help:
          'Output a JSON with iOS Xcode universal link settings into a file. '
          'The "--configuration" and "--target" must be set.',
      hide: !verboseHelp,
    );

    argParser.addOption(
      'configuration',
      help: 'Sets the iOS build configuration to be analyzed.',
      valueHelp: 'configuration',
      hide: !verboseHelp,
    );

    argParser.addOption(
      'target',
      help: 'Sets the iOS build target to be analyzed.',
      valueHelp: 'target',
      hide: !verboseHelp,
    );
  }

  /// The working directory for testing analysis using dartanalyzer.
  final Directory? workingDirectory;

  final Artifacts _artifacts;
  final FileSystem _fileSystem;
  final Logger _logger;
  final Terminal _terminal;
  final ProcessManager _processManager;
  final Platform _platform;
  final List<ProjectValidator> _allProjectValidators;
  final bool _suppressAnalytics;

  @override
  String get name => 'analyze';

  @override
  String get description => "Analyze the project's Dart code.";

  @override
  String get category => FlutterCommandCategory.project;

  @visibleForTesting
  List<ProjectValidator> allProjectValidators() => _allProjectValidators;

  @override
  bool get shouldRunPub {
    // If they're not analyzing the current project.
    if (!boolArg('current-package')) {
      return false;
    }

    // Or we're not in a project directory.
    if (!_fileSystem.file('pubspec.yaml').existsSync()) {
      return false;
    }

    // Don't run pub if asking for machine output.
    if (boolArg('machine')) {
      return false;
    }

    // Android analyze needs to process resource, i.e. evaluating build
    // settings and assets, and thus needs to run pub.
    if (boolArg('android')) {
      return true;
    }

    return super.shouldRunPub;
  }

  @override
  Future<FlutterCommandResult> runCommand() async {
    if (boolArg('android')) {
      final AndroidAnalyzeOption option;
      final String? buildVariant;
      if (argResults!['list-build-variants'] as bool &&
          argResults!['output-app-link-settings'] as bool) {
        throwToolExit(
          'Only one of "--list-build-variants" or "--output-app-link-settings" can be provided',
        );
      }
      if (argResults!['list-build-variants'] as bool) {
        option = AndroidAnalyzeOption.listBuildVariant;
        buildVariant = null;
      } else if (argResults!['output-app-link-settings'] as bool) {
        option = AndroidAnalyzeOption.outputAppLinkSettings;
        buildVariant = argResults!['build-variant'] as String?;
        if (buildVariant == null) {
          throwToolExit('"--build-variant" must be provided');
        }
      } else {
        throwToolExit('No argument is provided to analyze. Use -h to see available commands.');
      }
      final Set<String> items = findDirectories(argResults!, _fileSystem);
      final String directoryPath;
      if (items.isEmpty) {
        // user did not specify any path
        directoryPath = _fileSystem.currentDirectory.path;
      } else if (items.length > 1) {
        // if the user sends more than one path
        throwToolExit('The Android analyze can process only one directory path');
      } else {
        directoryPath = items.first;
      }
      await AndroidAnalyze(
        fileSystem: _fileSystem,
        option: option,
        userPath: directoryPath,
        buildVariant: buildVariant,
        logger: _logger,
      ).analyze();
    } else if (boolArg('ios')) {
      final IOSAnalyzeOption option;
      final String? configuration;
      final String? target;
      if (argResults!['list-build-options'] as bool &&
          argResults!['output-universal-link-settings'] as bool) {
        throwToolExit(
          'Only one of "--list-build-options" or "--output-universal-link-settings" can be provided',
        );
      }
      if (argResults!['list-build-options'] as bool) {
        option = IOSAnalyzeOption.listBuildOptions;
        configuration = null;
        target = null;
      } else if (argResults!['output-universal-link-settings'] as bool) {
        option = IOSAnalyzeOption.outputUniversalLinkSettings;
        configuration = argResults!['configuration'] as String?;
        if (configuration == null) {
          throwToolExit('"--configuration" must be provided');
        }
        target = argResults!['target'] as String?;
        if (target == null) {
          throwToolExit('"--target" must be provided');
        }
      } else {
        throwToolExit('No argument is provided to analyze. Use -h to see available commands.');
      }
      final Set<String> items = findDirectories(argResults!, _fileSystem);
      final String directoryPath;
      if (items.isEmpty) {
        // user did not specify any path
        directoryPath = _fileSystem.currentDirectory.path;
      } else if (items.length > 1) {
        // if the user sends more than one path
        throwToolExit('The iOS analyze can process only one directory path');
      } else {
        directoryPath = items.first;
      }
      await IOSAnalyze(
        project: FlutterProject.fromDirectory(_fileSystem.directory(directoryPath)),
        option: option,
        configuration: configuration,
        target: target,
        logger: _logger,
      ).analyze();
    } else if (boolArg('suggestions')) {
      final String directoryPath;
      if (boolArg('watch')) {
        throwToolExit('flag --watch is not compatible with --suggestions');
      }
      if (workingDirectory == null) {
        final Set<String> items = findDirectories(argResults!, _fileSystem);
        if (items.isEmpty) {
          // user did not specify any path
          directoryPath = _fileSystem.currentDirectory.path;
          _logger.printTrace('Showing suggestions for current directory: $directoryPath');
        } else if (items.length > 1) {
          // if the user sends more than one path
          throwToolExit('The suggestions flag can process only one directory path');
        } else {
          directoryPath = items.first;
        }
      } else {
        directoryPath = workingDirectory!.path;
      }
      return ValidateProject(
        fileSystem: _fileSystem,
        logger: _logger,
        allProjectValidators: _allProjectValidators,
        userPath: directoryPath,
        processManager: _processManager,
        machine: boolArg('machine'),
      ).run();
    } else if (boolArg('watch')) {
      await AnalyzeContinuously(
        argResults!,
        runner!.getRepoPackages(),
        fileSystem: _fileSystem,
        logger: _logger,
        platform: _platform,
        processManager: _processManager,
        terminal: _terminal,
        artifacts: _artifacts,
        suppressAnalytics: _suppressAnalytics,
      ).analyze();
    } else {
      await AnalyzeOnce(
        argResults!,
        runner!.getRepoPackages(),
        workingDirectory: workingDirectory,
        fileSystem: _fileSystem,
        logger: _logger,
        platform: _platform,
        processManager: _processManager,
        terminal: _terminal,
        artifacts: _artifacts,
        suppressAnalytics: _suppressAnalytics,
      ).analyze();
    }
    return FlutterCommandResult.success();
  }
}
