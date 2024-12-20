// Copyright 2014 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

import '../base/common.dart';
import '../base/process.dart';
import '../cache.dart';
import '../globals.dart' as globals;
import '../runner/flutter_command.dart';
import '../runner/flutter_command_runner.dart';
import '../version.dart';

import 'upgrade.dart' show precacheArtifacts;

class ChannelCommand extends FlutterCommand {
  ChannelCommand({bool verboseHelp = false}) {
    argParser.addFlag(
      'all',
      abbr: 'a',
      help: 'Include all the available branches (including local branches) when listing channels.',
      hide: !verboseHelp,
    );
    argParser.addFlag(
      'cache-artifacts',
      help:
          'After switching channels, download all required binary artifacts. '
          'This is the equivalent of running "flutter precache" with the "--all-platforms" flag.',
      defaultsTo: true,
    );
  }

  @override
  String get name => 'channel';

  @override
  String get description =>
      'List or switch Flutter channels.\n'
      '\n'
      'Common commands:\n'
      '\n'
      ' flutter channel\n'
      '   List Flutter channels.\n'
      '\n'
      ' flutter channel main\n'
      "   Switch to Flutter's main channel.";

  @override
  String get category => FlutterCommandCategory.sdk;

  @override
  String get invocation => '${runner?.executableName} $name [<channel-name>]';

  @override
  Future<Set<DevelopmentArtifact>> get requiredArtifacts async => const <DevelopmentArtifact>{};

  @override
  Future<FlutterCommandResult> runCommand() async {
    final List<String> rest = argResults?.rest ?? <String>[];
    switch (rest.length) {
      case 0:
        await _listChannels(
          showAll: boolArg('all'),
          verbose: globalResults?[FlutterGlobalOptions.kVerboseFlag] == true,
        );
        return FlutterCommandResult.success();
      case 1:
        await _switchChannel(rest[0]);
        return FlutterCommandResult.success();
      default:
        throwToolExit('Too many arguments.\n$usage');
    }
  }

  Future<void> _listChannels({required bool showAll, required bool verbose}) async {
    // Beware: currentBranch could contain PII. See getBranchName().
    final String currentChannel = globals.flutterVersion.channel; // limited to known branch names
    assert(
      kOfficialChannels.contains(currentChannel) ||
          kObsoleteBranches.containsKey(currentChannel) ||
          currentChannel == kUserBranch,
      'potential PII leak in channel name: "$currentChannel"',
    );
    final String currentBranch = globals.flutterVersion.getBranchName();
    final Set<String> seenUnofficialChannels = <String>{};
    final List<String> rawOutput = <String>[];

    globals.printStatus('Flutter channels:');
    final int result = await globals.processUtils.stream(
      <String>['git', 'branch', '-r'],
      workingDirectory: Cache.flutterRoot,
      mapFunction: (String line) {
        rawOutput.add(line);
        return null;
      },
    );
    if (result != 0) {
      final String details = verbose ? '\n${rawOutput.join('\n')}' : '';
      throwToolExit('List channels failed: $result$details', exitCode: result);
    }

    final Set<String> availableChannels = <String>{};

    for (final String line in rawOutput) {
      final List<String> split = line.split('/');
      if (split.length != 2) {
        // We don't know how to parse this line, skip it.
        continue;
      }
      final String branch = split[1];
      if (kOfficialChannels.contains(branch)) {
        availableChannels.add(branch);
      } else if (showAll) {
        seenUnofficialChannels.add(branch);
      }
    }

    bool currentChannelIsOfficial = false;

    // print all available official channels in sorted manner
    for (final String channel in kOfficialChannels) {
      // only print non-missing channels
      if (availableChannels.contains(channel)) {
        String currentIndicator = ' ';
        if (channel == currentChannel) {
          currentIndicator = '*';
          currentChannelIsOfficial = true;
        }
        globals.printStatus('$currentIndicator $channel (${kChannelDescriptions[channel]})');
      }
    }

    // print all remaining channels if showAll is true
    if (showAll) {
      for (final String branch in seenUnofficialChannels) {
        if (currentBranch == branch) {
          globals.printStatus('* $branch');
        } else if (!branch.startsWith('HEAD ')) {
          globals.printStatus('  $branch');
        }
      }
    } else if (!currentChannelIsOfficial) {
      globals.printStatus('* $currentBranch');
    }

    if (!currentChannelIsOfficial) {
      assert(
        currentChannel == kUserBranch,
        'Current channel is "$currentChannel", which is not an official branch. (Current branch is "$currentBranch".)',
      );
      globals.printStatus('');
      globals.printStatus('Currently not on an official channel.');
    }
  }

  Future<void> _switchChannel(String branchName) async {
    globals.printStatus("Switching to flutter channel '$branchName'...");
    if (kObsoleteBranches.containsKey(branchName)) {
      final String alternative = kObsoleteBranches[branchName]!;
      globals.printStatus(
        "This channel is obsolete. Consider switching to the '$alternative' channel instead.",
      );
    } else if (!kOfficialChannels.contains(branchName)) {
      globals.printStatus(
        'This is not an official channel. For a list of available channels, try "flutter channel".',
      );
    }
    await _checkout(branchName);
    if (boolArg('cache-artifacts')) {
      await precacheArtifacts(Cache.flutterRoot);
    }
    globals.printStatus("Successfully switched to flutter channel '$branchName'.");
    globals.printStatus(
      "To ensure that you're on the latest build from this channel, run 'flutter upgrade'",
    );
  }

  static Future<void> upgradeChannel(FlutterVersion currentVersion) async {
    final String channel = currentVersion.channel;
    if (kObsoleteBranches.containsKey(channel)) {
      final String alternative = kObsoleteBranches[channel]!;
      globals.printStatus("Transitioning from '$channel' to '$alternative'...");
      return _checkout(alternative);
    }
  }

  static Future<void> _checkout(String branchName) async {
    // Get latest refs from upstream.
    RunResult runResult = await globals.processUtils.run(<String>[
      'git',
      'fetch',
    ], workingDirectory: Cache.flutterRoot);

    if (runResult.processResult.exitCode == 0) {
      runResult = await globals.processUtils.run(<String>[
        'git',
        'show-ref',
        '--verify',
        '--quiet',
        'refs/heads/$branchName',
      ], workingDirectory: Cache.flutterRoot);
      if (runResult.processResult.exitCode == 0) {
        // branch already exists, try just switching to it
        runResult = await globals.processUtils.run(<String>[
          'git',
          'checkout',
          branchName,
          '--',
        ], workingDirectory: Cache.flutterRoot);
      } else {
        // branch does not exist, we have to create it
        runResult = await globals.processUtils.run(<String>[
          'git',
          'checkout',
          '--track',
          '-b',
          branchName,
          'origin/$branchName',
        ], workingDirectory: Cache.flutterRoot);
      }
    }
    if (runResult.processResult.exitCode != 0) {
      throwToolExit(
        'Switching channels failed\n$runResult.',
        exitCode: runResult.processResult.exitCode,
      );
    } else {
      // Remove the version check stamp, since it could contain out-of-date
      // information that pertains to the previous channel.
      await FlutterVersion.resetFlutterVersionFreshnessCheck();
    }
  }
}
