// Copyright 2014 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

import 'package:args/command_runner.dart';
import 'package:file/file.dart';
import 'package:platform/platform.dart';

import './proto/conductor_state.pb.dart' as pb;
import './repository.dart';
import './state.dart';
import './stdio.dart';

const String kVerboseFlag = 'verbose';
const String kStateOption = 'state-file';

/// Command to print the status of the current Flutter release.
class StatusCommand extends Command<void> {
  StatusCommand({required this.checkouts})
    : platform = checkouts.platform,
      fileSystem = checkouts.fileSystem,
      stdio = checkouts.stdio {
    final String defaultPath = defaultStateFilePath(platform);
    argParser.addOption(
      kStateOption,
      defaultsTo: defaultPath,
      help: 'Path to persistent state file. Defaults to $defaultPath',
    );
    argParser.addFlag(kVerboseFlag, abbr: 'v', help: 'Also print logs.');
  }

  final Checkouts checkouts;
  final FileSystem fileSystem;
  final Platform platform;
  final Stdio stdio;

  @override
  String get name => 'status';

  @override
  String get description => 'Print status of current release.';

  @override
  void run() {
    final File stateFile = checkouts.fileSystem.file(argResults![kStateOption]);
    if (!stateFile.existsSync()) {
      stdio.printStatus('No persistent state file found at ${argResults![kStateOption]}.');
      return;
    }
    final pb.ConductorState state = readStateFromFile(stateFile);

    stdio.printStatus(presentState(state));
    if (argResults![kVerboseFlag] as bool) {
      stdio.printStatus('\nLogs:');
      state.logs.forEach(stdio.printStatus);
    }
  }
}
