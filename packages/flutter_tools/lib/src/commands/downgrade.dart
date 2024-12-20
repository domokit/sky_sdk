// Copyright 2014 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

import 'package:process/process.dart';

import '../base/common.dart';
import '../base/file_system.dart';
import '../base/io.dart';
import '../base/logger.dart';
import '../base/process.dart';
import '../base/terminal.dart';
import '../cache.dart';
import '../globals.dart' as globals;
import '../persistent_tool_state.dart';
import '../runner/flutter_command.dart';
import '../version.dart';

/// The flutter downgrade command returns the SDK to the last recorded version
/// for a particular branch.
///
/// For example, suppose a user on the beta channel upgrades from 1.2.3 to 1.4.6.
/// The tool will record that sha "abcdefg" was the last active beta channel in the
/// persistent tool state. If the user is still on the beta channel and runs
/// flutter downgrade, this will take the user back to "abcdefg". They will not be
/// able to downgrade again, since the tool only records one prior version.
/// Additionally, if they had switched channels to stable before trying to downgrade,
/// the command would fail since there was no previously recorded stable version.
class DowngradeCommand extends FlutterCommand {
  DowngradeCommand({
    bool verboseHelp = false,
    PersistentToolState? persistentToolState,
    required Logger logger,
    ProcessManager? processManager,
    FlutterVersion? flutterVersion,
    Terminal? terminal,
    Stdio? stdio,
    FileSystem? fileSystem,
  }) : _terminal = terminal,
       _flutterVersion = flutterVersion,
       _persistentToolState = persistentToolState,
       _processManager = processManager,
       _stdio = stdio,
       _logger = logger,
       _fileSystem = fileSystem {
    argParser.addOption(
      'working-directory',
      hide: !verboseHelp,
      help:
          'Override the downgrade working directory. '
          'This is only intended to enable integration testing of the tool itself. '
          'It allows one to use the flutter tool from one checkout to downgrade a '
          'different checkout.',
    );
    argParser.addFlag(
      'prompt',
      defaultsTo: true,
      hide: !verboseHelp,
      help: 'Show the downgrade prompt.',
    );
  }

  Terminal? _terminal;
  FlutterVersion? _flutterVersion;
  PersistentToolState? _persistentToolState;
  ProcessUtils? _processUtils;
  ProcessManager? _processManager;
  final Logger _logger;
  Stdio? _stdio;
  FileSystem? _fileSystem;

  @override
  String get description => 'Downgrade Flutter to the last active version for the current channel.';

  @override
  String get name => 'downgrade';

  @override
  final String category = FlutterCommandCategory.sdk;

  @override
  Future<FlutterCommandResult> runCommand() async {
    // Commands do not necessarily have access to the correct zone injected
    // values when being created. Fields must be lazily instantiated in runCommand,
    // at least until the zone injection is refactored.
    _terminal ??= globals.terminal;
    _flutterVersion ??= globals.flutterVersion;
    _persistentToolState ??= globals.persistentToolState;
    _processManager ??= globals.processManager;
    _processUtils ??= ProcessUtils(processManager: _processManager!, logger: _logger);
    _stdio ??= globals.stdio;
    _fileSystem ??= globals.fs;
    String workingDirectory = Cache.flutterRoot!;
    if (argResults!.wasParsed('working-directory')) {
      workingDirectory = stringArg('working-directory')!;
      _flutterVersion = FlutterVersion(fs: _fileSystem!, flutterRoot: workingDirectory);
    }

    final String currentChannel = _flutterVersion!.channel;
    final Channel? channel = getChannelForName(currentChannel);
    if (channel == null) {
      throwToolExit(
        'Flutter is not currently on a known channel. '
        'Use "flutter channel" to switch to an official channel. ',
      );
    }
    final PersistentToolState persistentToolState = _persistentToolState!;
    final String? lastFlutterVersion = persistentToolState.lastActiveVersion(channel);
    final String? currentFlutterVersion = _flutterVersion?.frameworkRevision;
    if (lastFlutterVersion == null || currentFlutterVersion == lastFlutterVersion) {
      final String trailing = await _createErrorMessage(workingDirectory, channel);
      throwToolExit(
        "It looks like you haven't run "
        '"flutter upgrade" on channel "$currentChannel".\n'
        '\n'
        '"flutter downgrade" undoes the last "flutter upgrade".\n'
        '\n'
        'To switch to a specific Flutter version, see: '
        'https://flutter.dev/to/switch-flutter-version'
        '$trailing',
      );
    }

    // Detect unknown versions.
    final ProcessUtils processUtils = _processUtils!;
    final RunResult parseResult = await processUtils.run(<String>[
      'git',
      'describe',
      '--tags',
      lastFlutterVersion,
    ], workingDirectory: workingDirectory);
    if (parseResult.exitCode != 0) {
      throwToolExit('Failed to parse version for downgrade:\n${parseResult.stderr}');
    }
    final String humanReadableVersion = parseResult.stdout;

    // If there is a terminal attached, prompt the user to confirm the downgrade.
    final Stdio stdio = _stdio!;
    final Terminal terminal = _terminal!;
    if (stdio.hasTerminal && boolArg('prompt')) {
      terminal.usesTerminalUi = true;
      final String result = await terminal.promptForCharInput(
        const <String>['y', 'n'],
        prompt: 'Downgrade flutter to version $humanReadableVersion?',
        logger: _logger,
      );
      if (result == 'n') {
        return FlutterCommandResult.success();
      }
    } else {
      _logger.printStatus('Downgrading Flutter to version $humanReadableVersion');
    }

    // To downgrade the tool, we perform a git checkout --hard, and then
    // switch channels. The version recorded must have existed on that branch
    // so this operation is safe.
    try {
      await processUtils.run(
        <String>['git', 'reset', '--hard', lastFlutterVersion],
        throwOnError: true,
        workingDirectory: workingDirectory,
      );
    } on ProcessException catch (error) {
      throwToolExit(
        'Unable to downgrade Flutter: The tool could not update to the version '
        '$humanReadableVersion.\n'
        'Error: $error',
      );
    }
    try {
      await processUtils.run(
        // The `--` bit (because it's followed by nothing) means that we don't actually change
        // anything in the working tree, which avoids the need to first go into detached HEAD mode.
        <String>['git', 'checkout', currentChannel, '--'],
        throwOnError: true,
        workingDirectory: workingDirectory,
      );
    } on ProcessException catch (error) {
      throwToolExit(
        'Unable to downgrade Flutter: The tool could not switch to the channel '
        '$currentChannel.\n'
        'Error: $error',
      );
    }
    await FlutterVersion.resetFlutterVersionFreshnessCheck();
    _logger.printStatus('Success');
    return FlutterCommandResult.success();
  }

  // Formats an error message that lists the currently stored versions.
  Future<String> _createErrorMessage(String workingDirectory, Channel currentChannel) async {
    final StringBuffer buffer = StringBuffer();
    for (final Channel channel in Channel.values) {
      if (channel == currentChannel) {
        continue;
      }
      final String? sha = _persistentToolState?.lastActiveVersion(channel);
      if (sha == null) {
        continue;
      }
      final RunResult parseResult = await _processUtils!.run(<String>[
        'git',
        'describe',
        '--tags',
        sha,
      ], workingDirectory: workingDirectory);
      if (parseResult.exitCode == 0) {
        if (buffer.isEmpty) {
          buffer.writeln();
        }
        buffer.writeln();
        buffer.writeln(
          'Channel "${getNameForChannel(channel)}" was previously on: '
          '${parseResult.stdout}.',
        );
      }
    }
    return buffer.toString();
  }
}
