// Copyright 2014 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

import 'package:args/args.dart';
import 'package:args/command_runner.dart';
import 'package:file/file.dart';
import 'package:platform/platform.dart';

import './globals.dart';
import './repository.dart';
import './state.dart';
import './stdio.dart';

const String kYesFlag = 'yes';
const String kStateOption = 'state-file';

/// Command to clean up persistent state file.
///
/// If the release was not completed, this command will abort the release.
class CleanCommand extends Command<void> {
  CleanCommand({required this.checkouts})
    : platform = checkouts.platform,
      fileSystem = checkouts.fileSystem,
      stdio = checkouts.stdio {
    final String defaultPath = defaultStateFilePath(platform);
    argParser.addFlag(kYesFlag, help: 'Override confirmation checks.');
    argParser.addOption(
      kStateOption,
      defaultsTo: defaultPath,
      help: 'Path to persistent state file. Defaults to $defaultPath',
    );
  }

  final Checkouts checkouts;
  final FileSystem fileSystem;
  final Platform platform;
  final Stdio stdio;

  @override
  String get name => 'clean';

  @override
  String get description =>
      'Cleanup persistent state file. '
      'This will abort a work in progress release.';

  @override
  Future<void> run() {
    final ArgResults argumentResults = argResults!;
    final File stateFile = checkouts.fileSystem.file(argumentResults[kStateOption]);
    if (!stateFile.existsSync()) {
      throw ConductorException('No persistent state file found at ${stateFile.path}!');
    }

    if (!(argumentResults[kYesFlag] as bool)) {
      stdio.printStatus(
        'Are you sure you want to clean up the persistent state file at\n'
        '${stateFile.path} (y/n)?',
      );
      final String response = stdio.readLineSync();

      // Only proceed if the first character of stdin is 'y' or 'Y'
      if (response.isEmpty || response[0].toLowerCase() != 'y') {
        stdio.printStatus('Aborting clean operation.');
      }
    }
    stdio.printStatus('Deleting persistent state file ${stateFile.path}...');

    final CleanContext cleanContext = CleanContext(stateFile: stateFile);
    return cleanContext.run();
  }
}

/// Context for cleaning up persistent state file.
///
/// This is a frontend-agnostic implementation.
class CleanContext {
  CleanContext({required this.stateFile});

  final File stateFile;

  Future<void> run() {
    return stateFile.delete();
  }
}
