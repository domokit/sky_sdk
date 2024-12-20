// Copyright 2014 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

import 'package:args/args.dart';
import 'package:conductor_core/src/globals.dart';
import 'package:conductor_core/src/proto/conductor_state.pb.dart' as pb;

import './common.dart';

void main() {
  test('assertsEnabled returns true in test suite', () {
    expect(assertsEnabled(), true);
  });

  group('getNewPrLink', () {
    const String userName = 'flutterer';
    const String releaseChannel = 'beta';
    const String releaseVersion = '1.2.0-3.4.pre';
    const String candidateBranch = 'flutter-1.2-candidate.3';
    const String workingBranch = 'cherrypicks-$candidateBranch';
    const String dartRevision = 'fe9708ab688dcda9923f584ba370a66fcbc3811f';
    const String engineCherrypick1 = 'a5a25cd702b062c24b2c67b8d30b5cb33e0ef6f0';
    const String engineCherrypick2 = '94d06a2e1d01a3b0c693b94d70c5e1df9d78d249';
    const String frameworkCherrypick = 'a5a25cd702b062c24b2c67b8d30b5cb33e0ef6f0';

    final RegExp titlePattern = RegExp(r'&title=(.*)&');
    final RegExp bodyPattern = RegExp(r'&body=(.*)$');

    late pb.ConductorState state;

    setUp(() {
      state =
          (pb.ConductorState.create()
            ..engine =
                (pb.Repository.create()
                  ..candidateBranch = candidateBranch
                  ..cherrypicks.addAll(<pb.Cherrypick>[
                    pb.Cherrypick.create()..trunkRevision = engineCherrypick1,
                    pb.Cherrypick.create()..trunkRevision = engineCherrypick2,
                  ])
                  ..dartRevision = dartRevision
                  ..workingBranch = workingBranch)
            ..framework =
                (pb.Repository.create()
                  ..candidateBranch = candidateBranch
                  ..cherrypicks.add(pb.Cherrypick.create()..trunkRevision = frameworkCherrypick)
                  ..workingBranch = workingBranch)
            ..releaseChannel = releaseChannel
            ..releaseVersion = releaseVersion);
    });

    test('throws on an invalid repoName', () {
      expect(
        () => getNewPrLink(repoName: 'flooter', userName: userName, state: state),
        throwsExceptionWith('Expected repoName to be one of flutter or engine but got flooter.'),
      );
    });

    test('returns a valid URL for engine', () {
      final String link = getNewPrLink(repoName: 'engine', userName: userName, state: state);
      expect(link, contains('https://github.com/flutter/engine/compare/'));
      expect(link, contains('$candidateBranch...$userName:$workingBranch?expand=1'));
      expect(
        Uri.decodeQueryComponent(titlePattern.firstMatch(link)?.group(1) ?? ''),
        '[flutter_releases] Flutter $releaseChannel $releaseVersion Engine Cherrypicks',
      );
      final String expectedBody = '''
# Flutter $releaseChannel $releaseVersion Engine

## Scheduled Cherrypicks

- Roll dart revision: dart-lang/sdk@${dartRevision.substring(0, 9)}
- commit: flutter/engine@${engineCherrypick1.substring(0, 9)}
- commit: flutter/engine@${engineCherrypick2.substring(0, 9)}
''';
      expect(Uri.decodeQueryComponent(bodyPattern.firstMatch(link)?.group(1) ?? ''), expectedBody);
    });

    test('returns a valid URL for framework', () {
      final String link = getNewPrLink(repoName: 'flutter', userName: userName, state: state);
      expect(link, contains('https://github.com/flutter/flutter/compare/'));
      expect(link, contains('$candidateBranch...$userName:$workingBranch?expand=1'));
      expect(
        Uri.decodeQueryComponent(titlePattern.firstMatch(link)?.group(1) ?? ''),
        '[flutter_releases] Flutter $releaseChannel $releaseVersion Framework Cherrypicks',
      );
      final String expectedBody = '''
# Flutter $releaseChannel $releaseVersion Framework

## Scheduled Cherrypicks

- commit: ${frameworkCherrypick.substring(0, 9)}
''';
      expect(Uri.decodeQueryComponent(bodyPattern.firstMatch(link)?.group(1) ?? ''), expectedBody);
    });
  });

  group('getBoolFromEnvOrArgs', () {
    const String flagName = 'a-cli-flag';

    test('prefers env over argResults', () {
      final ArgResults argResults = FakeArgs(results: <String, Object>{flagName: false});
      final Map<String, String> env = <String, String>{'A_CLI_FLAG': 'TRUE'};
      final bool result = getBoolFromEnvOrArgs(flagName, argResults, env);
      expect(result, true);
    });

    test('falls back to argResults if env is empty', () {
      final ArgResults argResults = FakeArgs(results: <String, Object>{flagName: false});
      final Map<String, String> env = <String, String>{};
      final bool result = getBoolFromEnvOrArgs(flagName, argResults, env);
      expect(result, false);
    });
  });
}

class FakeArgs extends Fake implements ArgResults {
  FakeArgs({
    this.arguments = const <String>[],
    this.name = 'fake-command',
    this.results = const <String, Object>{},
  });

  final Map<String, Object> results;

  @override
  final List<String> arguments;

  @override
  final String name;

  @override
  bool wasParsed(String name) {
    return results[name] != null;
  }

  @override
  Object? operator [](String name) {
    return results[name];
  }
}
