// Copyright 2014 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

import 'package:args/args.dart';

import 'proto/conductor_state.pb.dart' as pb;
import 'repository.dart';

const String gsutilBinary = 'gsutil.py';

const String kFrameworkDefaultBranch = 'master';
const String kForceFlag = 'force';

const List<String> kBaseReleaseChannels = <String>['stable', 'beta'];

const List<String> kReleaseChannels = <String>[
  ...kBaseReleaseChannels,
  FrameworkRepository.defaultBranch,
];

const String kReleaseDocumentationUrl =
    'https://github.com/flutter/flutter/blob/main/docs/releases/Flutter-Cherrypick-Process.md';

const String kLuciPackagingConsoleLink =
    'https://ci.chromium.org/p/dart-internal/g/flutter_packaging/console';

const String kWebsiteReleasesUrl = 'https://docs.flutter.dev/development/tools/sdk/releases';

const String discordReleaseChannel =
    'https://discord.com/channels/608014603317936148/783492179922124850';

const String flutterReleaseHotline = 'https://mail.google.com/chat/u/0/#chat/space/AAAA6RKcK2k';

const String hotfixToStableWiki = 'https://github.com/flutter/flutter/blob/main/CHANGELOG.md';

const String flutterAnnounceGroup = 'https://groups.google.com/g/flutter-announce';

const String hotfixDocumentationBestPractices =
    'https://github.com/flutter/flutter/blob/main/docs/releases/Hotfix-Documentation-Best-Practices.md';

final RegExp releaseCandidateBranchRegex = RegExp(r'flutter-(\d+)\.(\d+)-candidate\.(\d+)');

/// Cast a dynamic to String and trim.
String stdoutToString(dynamic input) {
  final String str = input as String;
  return str.trim();
}

class ConductorException implements Exception {
  ConductorException(this.message);

  final String message;

  @override
  String toString() => 'Exception: $message';
}

bool assertsEnabled() {
  // Verify asserts enabled
  bool assertsEnabled = false;

  assert(() {
    assertsEnabled = true;
    return true;
  }());
  return assertsEnabled;
}

/// Either return the value from [env] or fall back to [argResults].
///
/// If the key does not exist in either the environment or CLI args, throws a
/// [ConductorException].
///
/// The environment is favored over CLI args since the latter can have a default
/// value, which the environment should be able to override.
String? getValueFromEnvOrArgs(
  String name,
  ArgResults argResults,
  Map<String, String> env, {
  bool allowNull = false,
}) {
  final String envName = fromArgToEnvName(name);
  if (env[envName] != null) {
    return env[envName];
  }
  final String? argValue = argResults[name] as String?;
  if (argValue != null) {
    return argValue;
  }

  if (allowNull) {
    return null;
  }
  throw ConductorException(
    'Expected either the CLI arg --$name or the environment variable $envName '
    'to be provided!',
  );
}

bool getBoolFromEnvOrArgs(String name, ArgResults argResults, Map<String, String> env) {
  final String envName = fromArgToEnvName(name);
  if (env[envName] != null) {
    return env[envName]?.toUpperCase() == 'TRUE';
  }
  return argResults[name] as bool;
}

/// Return multiple values from the environment or fall back to [argResults].
///
/// Values read from an environment variable are assumed to be comma-delimited.
///
/// If the key does not exist in either the CLI args or environment, throws a
/// [ConductorException].
///
/// The environment is favored over CLI args since the latter can have a default
/// value, which the environment should be able to override.
List<String> getValuesFromEnvOrArgs(String name, ArgResults argResults, Map<String, String> env) {
  final String envName = fromArgToEnvName(name);
  if (env[envName] != null && env[envName] != '') {
    return env[envName]!.split(',');
  }
  final List<String>? argValues = argResults[name] as List<String>?;
  if (argValues != null) {
    return argValues;
  }

  throw ConductorException(
    'Expected either the CLI arg --$name or the environment variable $envName '
    'to be provided!',
  );
}

/// Translate CLI arg names to env variable names.
///
/// For example, 'state-file' -> 'STATE_FILE'.
String fromArgToEnvName(String argName) {
  return argName.toUpperCase().replaceAll(r'-', r'_');
}

/// Return a web link for the user to open a new PR.
///
/// Includes PR title and body via query params.
String getNewPrLink({
  required String userName,
  required String repoName,
  required pb.ConductorState state,
}) {
  assert(state.releaseChannel.isNotEmpty);
  assert(state.releaseVersion.isNotEmpty);
  final (pb.Repository repository, String repoLabel) = switch (repoName) {
    'flutter' => (state.framework, 'Framework'),
    'engine' => (state.engine, 'Engine'),
    _ =>
      throw ConductorException(
        'Expected repoName to be one of flutter or engine but got $repoName.',
      ),
  };
  final String candidateBranch = repository.candidateBranch;
  final String workingBranch = repository.workingBranch;
  assert(candidateBranch.isNotEmpty);
  assert(workingBranch.isNotEmpty);
  final String title =
      '[flutter_releases] Flutter ${state.releaseChannel} '
      '${state.releaseVersion} $repoLabel Cherrypicks';
  final StringBuffer body = StringBuffer();
  body.write('''
# Flutter ${state.releaseChannel} ${state.releaseVersion} $repoLabel

## Scheduled Cherrypicks

''');
  if (repoName == 'engine') {
    if (state.engine.dartRevision.isNotEmpty) {
      // shorten hashes to make final link manageable
      // prefix with github org/repo so GitHub will auto-generate a hyperlink
      body.writeln(
        '- Roll dart revision: dart-lang/sdk@${state.engine.dartRevision.substring(0, 9)}',
      );
    }
    for (final pb.Cherrypick cp in state.engine.cherrypicks) {
      // Only list commits that map to a commit that exists upstream.
      if (cp.trunkRevision.isNotEmpty) {
        body.writeln('- commit: flutter/engine@${cp.trunkRevision.substring(0, 9)}');
      }
    }
  } else {
    for (final pb.Cherrypick cp in state.framework.cherrypicks) {
      // Only list commits that map to a commit that exists upstream.
      if (cp.trunkRevision.isNotEmpty) {
        body.writeln('- commit: ${cp.trunkRevision.substring(0, 9)}');
      }
    }
  }
  return 'https://github.com/flutter/$repoName/compare/'
      '$candidateBranch...$userName:$workingBranch?'
      'expand=1'
      '&title=${Uri.encodeQueryComponent(title)}'
      '&body=${Uri.encodeQueryComponent(body.toString())}';
}
