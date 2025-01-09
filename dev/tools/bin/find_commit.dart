// Copyright 2014 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

// This script looks at the current commit and branch of the git repository in
// which it was run, and finds the contemporary commit in the master branch of
// another git repository, whose path is provided on the command line. The
// contemporary commit is the one that was public at the time of the last commit
// on the master branch before the current commit's branch was created.

import 'dart:io';

const bool debugLogging = false;

void log(String message) {
  if (debugLogging) {
    print(message);
  }
}

const String _commitTimestampFormat = '--format=%cI';
DateTime _parseTimestamp(String line) => DateTime.parse(line.trim());
int _countLines(String output) =>
    output.trim().split('/n').where((String line) => line.isNotEmpty).length;

String findCommit({
  required String primaryRepoDirectory,
  required String primaryBranch,
  required String primaryTrunk,
  required String secondaryRepoDirectory,
  required String secondaryBranch,
}) {
  final DateTime anchor;
  if (primaryBranch == primaryTrunk) {
    log('on $primaryTrunk, using last commit time');
    anchor = _parseTimestamp(
      git(primaryRepoDirectory, <String>[
        'log',
        _commitTimestampFormat,
        '--max-count=1',
        primaryBranch,
        '--',
      ]),
    );
  } else {
    final String mergeBase =
        git(primaryRepoDirectory, <String>[
          'merge-base',
          primaryBranch,
          primaryTrunk,
        ], allowFailure: true).trim();
    if (mergeBase.isEmpty) {
      throw StateError(
        'Branch $primaryBranch does not seem to have a common history with trunk $primaryTrunk.',
      );
    }
    anchor = _parseTimestamp(
      git(primaryRepoDirectory, <String>[
        'log',
        _commitTimestampFormat,
        '--max-count=1',
        mergeBase,
        '--',
      ]),
    );
    if (debugLogging) {
      final int missingTrunkCommits = _countLines(
        git(primaryRepoDirectory, <String>['rev-list', primaryTrunk, '^$primaryBranch', '--']),
      );
      final int extraCommits = _countLines(
        git(primaryRepoDirectory, <String>['rev-list', primaryBranch, '^$primaryTrunk', '--']),
      );
      if (missingTrunkCommits == 0 && extraCommits == 0) {
        log('$primaryBranch is even with $primaryTrunk at $mergeBase');
      } else {
        log(
          '$primaryBranch branched from $primaryTrunk $missingTrunkCommits commits ago, trunk has advanced by $extraCommits commits since then.',
        );
      }
    }
  }
  return git(secondaryRepoDirectory, <String>[
    'log',
    '--format=%H',
    '--until=${anchor.toIso8601String()}',
    '--max-count=1',
    secondaryBranch,
    '--',
  ]);
}

String git(String workingDirectory, List<String> arguments, {bool allowFailure = false}) {
  final ProcessResult result = Process.runSync(
    'git',
    arguments,
    workingDirectory: workingDirectory,
  );
  if (!allowFailure && result.exitCode != 0 || '${result.stderr}'.isNotEmpty) {
    throw ProcessException('git', arguments, '${result.stdout}${result.stderr}', result.exitCode);
  }
  return '${result.stdout}';
}

void main(List<String> arguments) {
  if (arguments.isEmpty ||
      arguments.length != 4 ||
      arguments.contains('--help') ||
      arguments.contains('-h')) {
    print(
      'Usage: dart find_commit.dart <path-to-primary-repo> <primary-trunk> <path-to-secondary-repo> <secondary-branch>\n'
      'This script will find the commit in the secondary repo that was contemporary\n'
      'when the commit in the primary repo was created. If that commit is on a\n'
      "branch, then the date of the branch's last merge is used instead.",
    );
  } else {
    final String primaryRepo = arguments.first;
    final String primaryTrunk = arguments[1];
    final String secondaryRepo = arguments[2];
    final String secondaryBranch = arguments.last;
    print(
      findCommit(
        primaryRepoDirectory: primaryRepo,
        primaryBranch: git(primaryRepo, <String>['rev-parse', '--abbrev-ref', 'HEAD']).trim(),
        primaryTrunk: primaryTrunk,
        secondaryRepoDirectory: secondaryRepo,
        secondaryBranch: secondaryBranch,
      ).trim(),
    );
  }
}
