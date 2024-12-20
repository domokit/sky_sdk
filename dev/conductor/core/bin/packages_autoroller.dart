// Copyright 2014 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

import 'dart:io' as io;

import 'package:args/args.dart';
import 'package:conductor_core/conductor_core.dart';
import 'package:conductor_core/packages_autoroller.dart';
import 'package:file/file.dart';
import 'package:file/local.dart';
import 'package:meta/meta.dart' show visibleForTesting;
import 'package:platform/platform.dart';
import 'package:process/process.dart';

const String kTokenOption = 'token';
const String kGithubClient = 'github-client';
const String kUpstreamRemote = 'upstream-remote';
const String kGithubAccountName = 'flutter-pub-roller-bot';

Future<void> main(List<String> args) {
  return run(args);
}

@visibleForTesting
Future<void> run(
  List<String> args, {
  FileSystem fs = const LocalFileSystem(),
  ProcessManager processManager = const LocalProcessManager(),
}) async {
  final ArgParser parser = ArgParser();
  parser.addOption(kTokenOption, help: 'Path to GitHub access token file.', mandatory: true);
  parser.addOption(
    kGithubClient,
    help:
        'Path to GitHub CLI client. If not provided, it is assumed `gh` is '
        'present on the PATH.',
  );
  parser.addOption(
    kUpstreamRemote,
    help: 'The upstream git remote that the feature branch will be merged to.',
    hide: true,
    defaultsTo: 'https://github.com/flutter/flutter.git',
  );

  final ArgResults results;
  try {
    results = parser.parse(args);
  } on FormatException {
    io.stdout.writeln('''
Usage:

${parser.usage}
''');
    rethrow;
  }

  const String mirrorUrl = 'https://github.com/flutter-pub-roller-bot/flutter.git';
  final String upstreamUrl = results[kUpstreamRemote]! as String;
  final String tokenPath = results[kTokenOption]! as String;
  final File tokenFile = fs.file(tokenPath);
  if (!tokenFile.existsSync()) {
    throw ArgumentError(
      'Provided token path $tokenPath but no file exists at ${tokenFile.absolute.path}',
    );
  }
  final String token = tokenFile.readAsStringSync().trim();
  if (token.isEmpty) {
    throw ArgumentError(
      'Tried to read a GitHub access token from file ${tokenFile.path} but it was empty',
    );
  }

  final FrameworkRepository framework = FrameworkRepository(
    _localCheckouts(token),
    mirrorRemote: const Remote.mirror(mirrorUrl),
    upstreamRemote: Remote.upstream(upstreamUrl),
  );

  await PackageAutoroller(
    framework: framework,
    githubClient: results[kGithubClient] as String? ?? 'gh',
    orgName: _parseOrgName(mirrorUrl),
    token: token,
    processManager: processManager,
    githubUsername: kGithubAccountName,
  ).roll();
}

String _parseOrgName(String remoteUrl) {
  final RegExp pattern = RegExp(r'^https:\/\/github\.com\/(.*)\/');
  final RegExpMatch? match = pattern.firstMatch(remoteUrl);
  if (match == null) {
    throw FormatException(
      'Malformed upstream URL "$remoteUrl", should start with "https://github.com/"',
    );
  }
  return match.group(1)!;
}

Checkouts _localCheckouts(String token) {
  const FileSystem fileSystem = LocalFileSystem();
  const ProcessManager processManager = LocalProcessManager();
  const Platform platform = LocalPlatform();
  final Stdio stdio = VerboseStdio(
    stdout: io.stdout,
    stderr: io.stderr,
    stdin: io.stdin,
    filter: (String message) => message.replaceAll(token, '[GitHub TOKEN]'),
  );
  return Checkouts(
    fileSystem: fileSystem,
    parentDirectory: _localFlutterRoot.parent,
    platform: platform,
    processManager: processManager,
    stdio: stdio,
  );
}

Directory get _localFlutterRoot {
  String filePath;
  const FileSystem fileSystem = LocalFileSystem();
  const Platform platform = LocalPlatform();

  filePath = platform.script.toFilePath();
  final String checkoutsDirname = fileSystem.path.normalize(
    fileSystem.path.join(
      fileSystem.path.dirname(filePath), // flutter/dev/conductor/core/bin
      '..', // flutter/dev/conductor/core
      '..', // flutter/dev/conductor
      '..', // flutter/dev
      '..', // flutter
    ),
  );
  return fileSystem.directory(checkoutsDirname);
}

@visibleForTesting
void validateTokenFile(String filePath, [FileSystem fs = const LocalFileSystem()]) {}
