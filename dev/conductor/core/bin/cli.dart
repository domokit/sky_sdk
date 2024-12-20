// Copyright 2014 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

// See: https://github.com/flutter/flutter/blob/main/docs/releases/Release-process.md

import 'dart:io' as io;

import 'package:args/command_runner.dart';
import 'package:conductor_core/conductor_core.dart';
import 'package:file/file.dart';
import 'package:file/local.dart';
import 'package:platform/platform.dart';
import 'package:process/process.dart';

const String readmeUrl = 'https://github.com/flutter/flutter/tree/main/dev/conductor/README.md';

Future<void> main(List<String> args) async {
  const FileSystem fileSystem = LocalFileSystem();
  const ProcessManager processManager = LocalProcessManager();
  const Platform platform = LocalPlatform();
  final Stdio stdio = VerboseStdio(stdout: io.stdout, stderr: io.stderr, stdin: io.stdin);
  final Checkouts checkouts = Checkouts(
    fileSystem: fileSystem,
    parentDirectory: _localFlutterRoot.parent,
    platform: platform,
    processManager: processManager,
    stdio: stdio,
  );

  final CommandRunner<void> runner = CommandRunner<void>(
    'conductor',
    'A tool for coordinating Flutter releases. For more documentation on '
        'usage, please see $readmeUrl.',
    usageLineLength: 80,
  );

  final String conductorVersion =
      (await const Git(processManager).getOutput(
        <String>['rev-parse'],
        'Get the revision of the current Flutter SDK',
        workingDirectory: _localFlutterRoot.path,
      )).trim();

  <Command<void>>[
    StatusCommand(checkouts: checkouts),
    StartCommand(checkouts: checkouts, conductorVersion: conductorVersion),
    CleanCommand(checkouts: checkouts),
    CandidatesCommand(checkouts: checkouts, flutterRoot: _localFlutterRoot),
    NextCommand(checkouts: checkouts),
  ].forEach(runner.addCommand);

  if (!assertsEnabled()) {
    stdio.printError('The conductor tool must be run with --enable-asserts.');
    io.exit(1);
  }

  try {
    await runner.run(args);
  } on Exception catch (e, stacktrace) {
    stdio.printError('$e\n\n$stacktrace');
    io.exit(1);
  }
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
