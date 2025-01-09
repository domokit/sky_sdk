// Copyright 2014 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

import 'package:args/args.dart';
import 'package:args/command_runner.dart';
import 'package:file/file.dart';

import './git.dart';
import './globals.dart' show releaseCandidateBranchRegex;
import './repository.dart';
import './stdio.dart';
import './version.dart';

const String kRemote = 'remote';

class CandidatesCommand extends Command<void> {
  CandidatesCommand({required this.flutterRoot, required this.checkouts})
    : git = Git(checkouts.processManager),
      stdio = checkouts.stdio {
    argParser.addOption(
      kRemote,
      help: 'Which remote name to query for branches.',
      defaultsTo: 'upstream',
    );
  }

  final Checkouts checkouts;
  final Directory flutterRoot;
  final Git git;
  final Stdio stdio;

  @override
  String get name => 'candidates';

  @override
  String get description => 'List release candidates.';

  @override
  Future<void> run() async {
    final ArgResults results = argResults!;
    await git.run(
      <String>['fetch', results[kRemote] as String],
      'Fetch from remote ${results[kRemote]}',
      workingDirectory: flutterRoot.path,
    );

    final FrameworkRepository framework = HostFrameworkRepository(
      checkouts: checkouts,
      name: 'framework-for-candidates',
      upstreamPath: flutterRoot.path,
    );

    final Version currentVersion = await framework.flutterVersion();
    stdio.printStatus('currentVersion = $currentVersion');

    final List<String> branches = (await git.getOutput(
      <String>['branch', '--no-color', '--remotes', '--list', '${results[kRemote]}/*'],
      'List all remote branches',
      workingDirectory: flutterRoot.path,
    )).split('\n');

    // Pattern for extracting only the branch name via sub-group 1
    final RegExp remotePattern = RegExp('${results[kRemote]}\\/(.*)');
    for (final String branchName in branches) {
      final RegExpMatch? candidateMatch = releaseCandidateBranchRegex.firstMatch(branchName);
      if (candidateMatch == null) {
        continue;
      }
      final int currentX = currentVersion.x;
      final int currentY = currentVersion.y;
      final int currentZ = currentVersion.z;
      final int currentM = currentVersion.m ?? 0;
      final int x = int.parse(candidateMatch.group(1)!);
      final int y = int.parse(candidateMatch.group(2)!);
      final int m = int.parse(candidateMatch.group(3)!);

      final RegExpMatch? match = remotePattern.firstMatch(branchName);
      // If this is not the correct remote
      if (match == null) {
        continue;
      }
      if (x < currentVersion.x) {
        continue;
      }
      if (x == currentVersion.x && y < currentVersion.y) {
        continue;
      }
      if (x == currentX && y == currentY && currentZ == 0 && m <= currentM) {
        continue;
      }
      stdio.printStatus(match.group(1)!);
    }
  }
}
