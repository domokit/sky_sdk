// Copyright 2014 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

import 'dart:async';
import 'dart:io' show exit, stderr;

import 'package:args/args.dart';
import 'package:file/file.dart';
import 'package:file/local.dart';

import 'prepare_package/archive_creator.dart';
import 'prepare_package/archive_publisher.dart';
import 'prepare_package/common.dart';

const FileSystem fs = LocalFileSystem();

/// Prepares a flutter git repo to be packaged up for distribution. It mainly
/// serves to populate the .pub-preload-cache with any appropriate Dart
/// packages, and the flutter cache in bin/cache with the appropriate
/// dependencies and snapshots.
///
/// Archives contain the executables and customizations for the platform that
/// they are created on.
Future<void> main(List<String> rawArguments) async {
  final ArgParser argParser = ArgParser();
  argParser.addOption(
    'temp_dir',
    help:
        'A location where temporary files may be written. Defaults to a '
        'directory in the system temp folder. Will write a few GiB of data, '
        'so it should have sufficient free space. If a temp_dir is not '
        'specified, then the default temp_dir will be created, used, and '
        'removed automatically.',
  );
  argParser.addOption(
    'revision',
    help:
        'The Flutter git repo revision to build the '
        'archive with. Must be the full 40-character hash. Required.',
  );
  argParser.addOption(
    'branch',
    allowed: Branch.values.map<String>((Branch branch) => branch.name),
    help: 'The Flutter branch to build the archive with. Required.',
  );
  argParser.addOption(
    'output',
    help:
        'The path to the directory where the output archive should be '
        'written. If --output is not specified, the archive will be written to '
        "the current directory. If the output directory doesn't exist, it, and "
        'the path to it, will be created.',
  );
  argParser.addFlag(
    'publish',
    help:
        'If set, will publish the archive to Google Cloud Storage upon '
        'successful creation of the archive. Will publish under this '
        'directory: $baseUrl$releaseFolder',
  );
  argParser.addFlag('force', abbr: 'f', help: 'Overwrite a previously uploaded package.');
  argParser.addFlag(
    'dry_run',
    negatable: false,
    help: 'Prints gsutil commands instead of executing them.',
  );
  argParser.addFlag('help', negatable: false, help: 'Print help for this command.');

  final ArgResults parsedArguments = argParser.parse(rawArguments);

  if (parsedArguments['help'] as bool) {
    print(argParser.usage);
    exit(0);
  }

  void errorExit(String message, {int exitCode = -1}) {
    stderr.write('Error: $message\n\n');
    stderr.write('${argParser.usage}\n');
    exit(exitCode);
  }

  if (!parsedArguments.wasParsed('revision')) {
    errorExit('Invalid argument: --revision must be specified.');
  }
  final String revision = parsedArguments['revision'] as String;
  if (revision.length != 40) {
    errorExit('Invalid argument: --revision must be the entire hash, not just a prefix.');
  }

  if (!parsedArguments.wasParsed('branch')) {
    errorExit('Invalid argument: --branch must be specified.');
  }

  final String? tempDirArg = parsedArguments['temp_dir'] as String?;
  final Directory tempDir;
  bool removeTempDir = false;
  if (tempDirArg == null || tempDirArg.isEmpty) {
    tempDir = fs.systemTempDirectory.createTempSync('flutter_package.');
    removeTempDir = true;
  } else {
    tempDir = fs.directory(tempDirArg);
    if (!tempDir.existsSync()) {
      errorExit("Temporary directory $tempDirArg doesn't exist.");
    }
  }

  final Directory outputDir;
  if (parsedArguments['output'] == null) {
    outputDir = tempDir;
  } else {
    outputDir = fs.directory(parsedArguments['output'] as String);
    if (!outputDir.existsSync()) {
      outputDir.createSync(recursive: true);
    }
  }

  final bool publish = parsedArguments['publish'] as bool;
  final bool dryRun = parsedArguments['dry_run'] as bool;
  final Branch branch = Branch.values.byName(parsedArguments['branch'] as String);
  final ArchiveCreator creator = ArchiveCreator(
    tempDir,
    outputDir,
    revision,
    branch,
    fs: fs,
    strict: publish && !dryRun,
  );
  int exitCode = 0;
  late String message;
  try {
    final Map<String, String> version = await creator.initializeRepo();
    final File outputFile = await creator.createArchive();
    final ArchivePublisher publisher = ArchivePublisher(
      tempDir,
      revision,
      branch,
      version,
      outputFile,
      dryRun,
      fs: fs,
    );
    await publisher.generateLocalMetadata();
    if (parsedArguments['publish'] as bool) {
      await publisher.publishArchive(parsedArguments['force'] as bool);
    }
  } on PreparePackageException catch (e) {
    exitCode = e.exitCode;
    message = e.message;
  } catch (e) {
    exitCode = -1;
    message = e.toString();
  } finally {
    if (removeTempDir) {
      tempDir.deleteSync(recursive: true);
    }
    if (exitCode != 0) {
      errorExit(message, exitCode: exitCode);
    }
    exit(0);
  }
}
