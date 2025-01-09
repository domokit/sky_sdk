// Copyright 2014 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

import 'package:args/args.dart';
import 'package:args/command_runner.dart';
import 'package:completion/completion.dart';
import 'package:file/file.dart';
import 'package:unified_analytics/unified_analytics.dart';

import '../artifacts.dart';
import '../base/common.dart';
import '../base/context.dart';
import '../base/file_system.dart';
import '../base/terminal.dart';
import '../base/utils.dart';
import '../cache.dart';
import '../convert.dart';
import '../globals.dart' as globals;
import '../resident_runner.dart';
import '../tester/flutter_tester.dart';
import '../version.dart';
import '../web/web_device.dart';

/// Common flutter command line options.
abstract final class FlutterGlobalOptions {
  static const String kColorFlag = 'color';
  static const String kContinuousIntegrationFlag = 'ci';
  static const String kDeviceIdOption = 'device-id';
  static const String kDisableAnalyticsFlag = 'disable-analytics';
  static const String kEnableAnalyticsFlag = 'enable-analytics';
  static const String kLocalEngineOption = 'local-engine';
  static const String kLocalEngineSrcPathOption = 'local-engine-src-path';
  static const String kLocalEngineHostOption = 'local-engine-host';
  static const String kLocalWebSDKOption = 'local-web-sdk';
  static const String kMachineFlag = 'machine';
  static const String kPackagesOption = 'packages';
  static const String kPrefixedErrorsFlag = 'prefixed-errors';
  static const String kPrintDtd = 'print-dtd';
  static const String kQuietFlag = 'quiet';
  static const String kShowTestDeviceFlag = 'show-test-device';
  static const String kShowWebServerDeviceFlag = 'show-web-server-device';
  static const String kSuppressAnalyticsFlag = 'suppress-analytics';
  static const String kVerboseFlag = 'verbose';
  static const String kVersionCheckFlag = 'version-check';
  static const String kVersionFlag = 'version';
  static const String kWrapColumnOption = 'wrap-column';
  static const String kWrapFlag = 'wrap';
  static const String kDebugLogsDirectoryFlag = 'debug-logs-dir';
}

class FlutterCommandRunner extends CommandRunner<void> {
  FlutterCommandRunner({bool verboseHelp = false})
    : super(
        'flutter',
        'Manage your Flutter app development.\n'
            '\n'
            'Common commands:\n'
            '\n'
            '  flutter create <output directory>\n'
            '    Create a new Flutter project in the specified directory.\n'
            '\n'
            '  flutter run [options]\n'
            '    Run your Flutter application on an attached device or in an emulator.',
      ) {
    argParser.addFlag(
      FlutterGlobalOptions.kVerboseFlag,
      abbr: 'v',
      negatable: false,
      help:
          'Noisy logging, including all shell commands executed.\n'
          'If used with "--help", shows hidden options. '
          'If used with "flutter doctor", shows additional diagnostic information. '
          '(Use "-vv" to force verbose logging in those cases.)',
    );
    argParser.addFlag(
      FlutterGlobalOptions.kPrefixedErrorsFlag,
      negatable: false,
      help: 'Causes lines sent to stderr to be prefixed with "ERROR:".',
      hide: !verboseHelp,
    );
    argParser.addFlag(
      FlutterGlobalOptions.kQuietFlag,
      negatable: false,
      hide: !verboseHelp,
      help: 'Reduce the amount of output from some commands.',
    );
    argParser.addFlag(
      FlutterGlobalOptions.kWrapFlag,
      hide: !verboseHelp,
      help: 'Toggles output word wrapping, regardless of whether or not the output is a terminal.',
      defaultsTo: true,
    );
    argParser.addOption(
      FlutterGlobalOptions.kWrapColumnOption,
      hide: !verboseHelp,
      help:
          'Sets the output wrap column. If not set, uses the width of the terminal. No '
          'wrapping occurs if not writing to a terminal. Use "--no-wrap" to turn off wrapping '
          'when connected to a terminal.',
    );
    argParser.addOption(
      FlutterGlobalOptions.kDeviceIdOption,
      abbr: 'd',
      help: 'Target device id or name (prefixes allowed).',
    );
    argParser.addFlag(
      FlutterGlobalOptions.kVersionFlag,
      negatable: false,
      help: 'Reports the version of this tool.',
    );
    argParser.addFlag(
      FlutterGlobalOptions.kMachineFlag,
      negatable: false,
      hide: !verboseHelp,
      help: 'When used with the "--version" flag, outputs the information using JSON.',
    );
    argParser.addFlag(
      FlutterGlobalOptions.kColorFlag,
      hide: !verboseHelp,
      help: 'Whether to use terminal colors (requires support for ANSI escape sequences).',
      defaultsTo: true,
    );
    argParser.addFlag(
      FlutterGlobalOptions.kVersionCheckFlag,
      defaultsTo: true,
      hide: !verboseHelp,
      help: 'Allow Flutter to check for updates when this command runs.',
    );
    argParser.addFlag(
      FlutterGlobalOptions.kEnableAnalyticsFlag,
      negatable: false,
      help:
          'Enable telemetry reporting each time a flutter or dart '
          'command runs.',
    );
    argParser.addFlag(
      FlutterGlobalOptions.kDisableAnalyticsFlag,
      negatable: false,
      help:
          'Disable telemetry reporting each time a flutter or dart '
          'command runs, until it is re-enabled.',
    );
    argParser.addFlag(
      FlutterGlobalOptions.kSuppressAnalyticsFlag,
      negatable: false,
      help: 'Suppress analytics reporting for the current CLI invocation.',
    );
    argParser.addOption(
      FlutterGlobalOptions.kPackagesOption,
      hide: !verboseHelp,
      help: 'Path to your "package_config.json" file.',
    );
    argParser.addFlag(
      FlutterGlobalOptions.kPrintDtd,
      negatable: false,
      help: 'Print the address of the Dart Tooling Daemon, if one is hosted by the Flutter CLI.',
      hide: !verboseHelp,
    );

    if (verboseHelp) {
      argParser.addSeparator('Local build selection options (not normally required):');
    }

    argParser.addOption(
      FlutterGlobalOptions.kLocalEngineSrcPathOption,
      hide: !verboseHelp,
      help:
          'Path to your engine src directory, if you are building Flutter locally.\n'
          'Defaults to \$$kFlutterEngineEnvironmentVariableName if set, otherwise defaults to '
          'the path given in your pubspec.yaml dependency_overrides for $kFlutterEnginePackageName, '
          'if any.',
    );

    argParser.addOption(
      FlutterGlobalOptions.kLocalEngineOption,
      hide: !verboseHelp,
      help:
          'Name of a build output within the engine out directory, if you are building Flutter locally.\n'
          'Use this to select a specific version of the engine if you have built multiple engine targets.\n'
          'This path is relative to "--local-engine-src-path" (see above).',
    );

    argParser.addOption(
      FlutterGlobalOptions.kLocalEngineHostOption,
      hide: !verboseHelp,
      help:
          'The host operating system for which engine artifacts should be selected, if you are building Flutter locally.\n'
          'This is only used when "--local-engine" is also specified.\n'
          'By default, the host is determined automatically, but you may need to specify this if you are building on one '
          'platform (e.g. MacOS ARM64) but intend to run Flutter on another (e.g. Android).',
    );

    argParser.addOption(
      FlutterGlobalOptions.kLocalWebSDKOption,
      hide: !verboseHelp,
      help:
          'Name of a build output within the engine out directory, if you are building Flutter locally.\n'
          'Use this to select a specific version of the web sdk if you have built multiple engine targets.\n'
          'This path is relative to "--local-engine-src-path" (see above).',
    );

    if (verboseHelp) {
      argParser.addSeparator('Options for testing the "flutter" tool itself:');
    }
    argParser.addFlag(
      FlutterGlobalOptions.kShowTestDeviceFlag,
      negatable: false,
      hide: !verboseHelp,
      help:
          'List the special "flutter-tester" device in device listings. '
          'This headless device is used to test Flutter tooling.',
    );
    argParser.addFlag(
      FlutterGlobalOptions.kShowWebServerDeviceFlag,
      negatable: false,
      hide: !verboseHelp,
      help: 'List the special "web-server" device in device listings.',
    );
    argParser.addFlag(
      FlutterGlobalOptions.kContinuousIntegrationFlag,
      negatable: false,
      help: 'Enable a set of CI-specific test debug settings.',
      hide: !verboseHelp,
    );
    argParser.addOption(
      FlutterGlobalOptions.kDebugLogsDirectoryFlag,
      help: 'Path to a directory where logs for debugging may be added.',
      hide: !verboseHelp,
    );
  }

  @override
  ArgParser get argParser => _argParser;
  final ArgParser _argParser = ArgParser(
    allowTrailingOptions: false,
    usageLineLength:
        globals.outputPreferences.wrapText ? globals.outputPreferences.wrapColumn : null,
  );

  @override
  String get usageFooter {
    return wrapText(
      'Run "flutter help -v" for verbose help output, including less commonly used options.',
      columnWidth: globals.outputPreferences.wrapColumn,
      shouldWrap: globals.outputPreferences.wrapText,
    );
  }

  @override
  String get usage {
    final String usageWithoutDescription = super.usage.substring(description.length + 2);
    final String prefix = wrapText(
      description,
      shouldWrap: globals.outputPreferences.wrapText,
      columnWidth: globals.outputPreferences.wrapColumn,
    );
    return '$prefix\n\n$usageWithoutDescription';
  }

  @override
  ArgResults parse(Iterable<String> args) {
    try {
      // This is where the CommandRunner would call argParser.parse(args). We
      // override this function so we can call tryArgsCompletion instead, so the
      // completion package can interrogate the argParser, and as part of that,
      // it calls argParser.parse(args) itself and returns the result.
      return tryArgsCompletion(args.toList(), argParser);
    } on ArgParserException catch (error) {
      if (error.commands.isEmpty) {
        usageException(error.message);
      }

      Command<void>? command = commands[error.commands.first];
      for (final String commandName in error.commands.skip(1)) {
        command = command?.subcommands[commandName];
      }

      command!.usageException(error.message);
    }
  }

  // See https://github.com/flutter/flutter/issues/145158.
  late bool _machineFlagPresentInAnyCliArg;

  @override
  Future<void> run(Iterable<String> args) {
    // Have invocations of 'build', 'custom-devices', and 'pub' print out
    // their sub-commands.
    // TODO(ianh): Move this to the Build command itself somehow.
    if (args.length == 1) {
      if (args.first == 'build') {
        args = <String>['build', '-h'];
      } else if (args.first == 'custom-devices') {
        args = <String>['custom-devices', '-h'];
      } else if (args.first == 'pub') {
        args = <String>['pub', '-h'];
      }
    }

    _machineFlagPresentInAnyCliArg = args.contains('--${FlutterGlobalOptions.kMachineFlag}');
    return super.run(args);
  }

  /// Whether to perform a flutter version check, which prints a warning if old.
  ///
  /// This method should be narrowly used in the following manner:
  /// ```dart
  /// final bool topLevelMachineFlag = topLevelResults[FlutterGlobalOptions.kMachineFlag] as bool? ?? false;
  /// if (await _shouldCheckForUpdates(topLevelResults, topLevelMachineFlag: topLevelMachineFlag)) {
  ///   await globals.flutterVersion.checkFlutterVersionFreshness();
  /// }
  /// ```
  Future<bool> _shouldCheckForUpdates(
    ArgResults topLevelResults, {
    required bool topLevelMachineFlag,
  }) async {
    // Check if the user has explicitly requested a version check.
    final bool versionCheckFlag =
        topLevelResults[FlutterGlobalOptions.kVersionCheckFlag] as bool? ?? false;
    final bool explicitVersionCheckPassed =
        topLevelResults.wasParsed(FlutterGlobalOptions.kVersionCheckFlag) && versionCheckFlag;
    if (explicitVersionCheckPassed) {
      return true;
    }

    // If the top level --machine flag is set, we don't want to check for updates.
    if (topLevelMachineFlag) {
      return false;
    }

    // Running the "upgrade" command is already checking, don't check twice.
    if (topLevelResults.command?.name == 'upgrade') {
      return false;
    }

    // If the same flag appears in any subcommand, we don't want to check for updates.
    //
    // A better solution would be the flag not being in any specific subcommand, just
    // in the top level command, but that would require a more significant refactor
    // and deprecation of the current behaviors.
    //
    // See https://github.com/flutter/flutter/issues/145158.
    if (_machineFlagPresentInAnyCliArg) {
      return false;
    }

    // e.g. `flutter bash-completion` or `flutter zsh-completion`
    final bool isShellCompletionCommand =
        !globals.stdio.hasTerminal && (topLevelResults.command?.name ?? '').endsWith('-completion');
    if (isShellCompletionCommand || await globals.botDetector.isRunningOnBot) {
      return false;
    }

    // Otherwise, check for updates based on the flag which is typically set by default.
    return versionCheckFlag;
  }

  @override
  Future<void> runCommand(ArgResults topLevelResults) async {
    final Map<Type, Object?> contextOverrides = <Type, Object?>{};

    // If the flag for enabling or disabling telemetry is passed in,
    // we will return out
    if (topLevelResults.wasParsed(FlutterGlobalOptions.kDisableAnalyticsFlag) ||
        topLevelResults.wasParsed(FlutterGlobalOptions.kEnableAnalyticsFlag)) {
      return;
    }

    // Don't set wrapColumns unless the user said to: if it's set, then all
    // wrapping will occur at this width explicitly, and won't adapt if the
    // terminal size changes during a run.
    int? wrapColumn;
    if (topLevelResults.wasParsed(FlutterGlobalOptions.kWrapColumnOption)) {
      try {
        wrapColumn = int.parse(topLevelResults[FlutterGlobalOptions.kWrapColumnOption] as String);
        if (wrapColumn < 0) {
          throwToolExit(
            globals.userMessages.runnerWrapColumnInvalid(
              topLevelResults[FlutterGlobalOptions.kWrapColumnOption],
            ),
          );
        }
      } on FormatException {
        throwToolExit(
          globals.userMessages.runnerWrapColumnParseError(
            topLevelResults[FlutterGlobalOptions.kWrapColumnOption],
          ),
        );
      }
    }

    // If we're not writing to a terminal with a defined width, then don't wrap
    // anything, unless the user explicitly said to.
    final bool useWrapping =
        topLevelResults.wasParsed(FlutterGlobalOptions.kWrapFlag)
            ? topLevelResults[FlutterGlobalOptions.kWrapFlag] as bool
            : globals.stdio.terminalColumns != null &&
                topLevelResults[FlutterGlobalOptions.kWrapFlag] as bool;
    contextOverrides[OutputPreferences] = OutputPreferences(
      wrapText: useWrapping,
      showColor: topLevelResults[FlutterGlobalOptions.kColorFlag] as bool?,
      wrapColumn: wrapColumn,
    );

    if (((topLevelResults[FlutterGlobalOptions.kShowTestDeviceFlag] as bool?) ?? false) ||
        topLevelResults[FlutterGlobalOptions.kDeviceIdOption] ==
            FlutterTesterDevices.kTesterDeviceId) {
      FlutterTesterDevices.showFlutterTesterDevice = true;
    }
    if (((topLevelResults[FlutterGlobalOptions.kShowWebServerDeviceFlag] as bool?) ?? false) ||
        topLevelResults[FlutterGlobalOptions.kDeviceIdOption] ==
            WebServerDevice.kWebServerDeviceId) {
      WebServerDevice.showWebServerDevice = true;
    }

    // Set up the tooling configuration.
    final EngineBuildPaths? engineBuildPaths = await globals.localEngineLocator?.findEnginePath(
      engineSourcePath: topLevelResults[FlutterGlobalOptions.kLocalEngineSrcPathOption] as String?,
      localEngine: topLevelResults[FlutterGlobalOptions.kLocalEngineOption] as String?,
      localHostEngine: topLevelResults[FlutterGlobalOptions.kLocalEngineHostOption] as String?,
      localWebSdk: topLevelResults[FlutterGlobalOptions.kLocalWebSDKOption] as String?,
      packagePath: topLevelResults[FlutterGlobalOptions.kPackagesOption] as String?,
    );
    if (engineBuildPaths != null) {
      contextOverrides.addAll(<Type, Object?>{
        Artifacts: Artifacts.getLocalEngine(engineBuildPaths),
      });
    }

    await context.run<void>(
      overrides: contextOverrides.map<Type, Generator>((Type type, Object? value) {
        return MapEntry<Type, Generator>(type, () => value);
      }),
      body: () async {
        globals.logger.quiet = (topLevelResults[FlutterGlobalOptions.kQuietFlag] as bool?) ?? false;

        if (globals.platform.environment['FLUTTER_ALREADY_LOCKED'] != 'true') {
          await globals.cache.lock();
        }

        if ((topLevelResults[FlutterGlobalOptions.kSuppressAnalyticsFlag] as bool?) ?? false) {
          globals.flutterUsage.suppressAnalytics = true;
          globals.analytics.suppressTelemetry();
        }

        globals.flutterVersion.ensureVersionFile();
        final bool machineFlag =
            topLevelResults[FlutterGlobalOptions.kMachineFlag] as bool? ?? false;
        if (await _shouldCheckForUpdates(topLevelResults, topLevelMachineFlag: machineFlag)) {
          await globals.flutterVersion.checkFlutterVersionFreshness();
        }

        // See if the user specified a specific device.
        final String? specifiedDeviceId =
            topLevelResults[FlutterGlobalOptions.kDeviceIdOption] as String?;
        if (specifiedDeviceId != null) {
          globals.deviceManager?.specifiedDeviceId = specifiedDeviceId;
        }

        if ((topLevelResults[FlutterGlobalOptions.kVersionFlag] as bool?) ?? false) {
          globals.flutterUsage.sendCommand(FlutterGlobalOptions.kVersionFlag);
          globals.analytics.send(
            Event.flutterCommandResult(
              commandPath: 'version',
              result: 'success',
              commandHasTerminal: globals.stdio.hasTerminal,
            ),
          );
          final FlutterVersion version = globals.flutterVersion.fetchTagsAndGetVersion(
            clock: globals.systemClock,
          );
          final String status;
          if (machineFlag) {
            final Map<String, Object> jsonOut = version.toJson();
            jsonOut['flutterRoot'] = Cache.flutterRoot!;
            status = const JsonEncoder.withIndent('  ').convert(jsonOut);
          } else {
            status = version.toString();
          }
          globals.printStatus(status);
          return;
        }
        if (machineFlag && topLevelResults.command?.name != 'analyze') {
          throwToolExit(
            'The "--machine" flag is only valid with the "--version" flag or the "analyze --suggestions" command.',
            exitCode: 2,
          );
        }

        // TODO(bkonyi): can this be removed and passed solely via DebuggingOptions?
        final bool shouldPrintDtdUri =
            topLevelResults[FlutterGlobalOptions.kPrintDtd] as bool? ?? false;
        DevtoolsLauncher.instance!.printDtdUri = shouldPrintDtdUri;

        await super.runCommand(topLevelResults);
      },
    );
  }

  /// Get the root directories of the repo - the directories containing Dart packages.
  List<String> getRepoRoots() {
    final String root = globals.fs.path.absolute(Cache.flutterRoot!);
    // not bin, and not the root
    return <String>['dev', 'examples', 'packages'].map<String>((String item) {
      return globals.fs.path.join(root, item);
    }).toList();
  }

  /// Get all pub packages in the Flutter repo.
  List<Directory> getRepoPackages() {
    return getRepoRoots()
        .expand<String>((String root) => _gatherProjectPaths(root))
        .map<Directory>((String dir) => globals.fs.directory(dir))
        .toList();
  }

  static List<String> _gatherProjectPaths(String rootPath) {
    if (globals.fs.isFileSync(globals.fs.path.join(rootPath, '.dartignore'))) {
      return <String>[];
    }

    final List<String> projectPaths =
        globals.fs.directory(rootPath).listSync(followLinks: false).expand((
          FileSystemEntity entity,
        ) {
          if (entity is Directory && !globals.fs.path.split(entity.path).contains('.dart_tool')) {
            return _gatherProjectPaths(entity.path);
          }
          return <String>[];
        }).toList();

    if (globals.fs.isFileSync(globals.fs.path.join(rootPath, 'pubspec.yaml'))) {
      projectPaths.add(rootPath);
    }

    return projectPaths;
  }
}
