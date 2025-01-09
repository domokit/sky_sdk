// Copyright 2014 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

import 'dart:convert';

import 'package:flutter_tools/src/artifacts.dart';
import 'package:flutter_tools/src/base/file_system.dart';
import 'package:flutter_tools/src/base/io.dart';
import 'package:flutter_tools/src/globals.dart' as globals;
import 'package:process/process.dart';

import '../../../src/test_flutter_command_runner.dart';

/// A ProcessManager that invokes a real process manager, but keeps
/// track of all commands sent to it.
class LoggingProcessManager extends LocalProcessManager {
  List<List<String>> commands = <List<String>>[];

  @override
  Future<Process> start(
    List<Object> command, {
    String? workingDirectory,
    Map<String, String>? environment,
    bool includeParentEnvironment = true,
    bool runInShell = false,
    ProcessStartMode mode = ProcessStartMode.normal,
  }) {
    commands.add(command.map((Object arg) => arg.toString()).toList());
    return super.start(
      command,
      workingDirectory: workingDirectory,
      environment: environment,
      includeParentEnvironment: includeParentEnvironment,
      runInShell: runInShell,
      mode: mode,
    );
  }

  void clear() {
    commands.clear();
  }
}

Future<void> analyzeProject(
  String workingDir, {
  List<String> expectedFailures = const <String>[],
}) async {
  final String flutterToolsSnapshotPath = globals.fs.path.absolute(
    globals.fs.path.join('..', '..', 'bin', 'cache', 'flutter_tools.snapshot'),
  );

  final List<String> args = <String>[flutterToolsSnapshotPath, 'analyze'];

  final ProcessResult exec = await Process.run(
    globals.artifacts!.getArtifactPath(Artifact.engineDartBinary),
    args,
    workingDirectory: workingDir,
  );
  if (expectedFailures.isEmpty) {
    printOnFailure('Results of running analyzer:');
    printOnFailure(exec.stdout.toString());
    printOnFailure(exec.stderr.toString());
    expect(exec.exitCode, 0);
    return;
  }
  expect(exec.exitCode, isNot(0));
  String lineParser(String line) {
    try {
      final String analyzerSeparator = globals.platform.isWindows ? ' - ' : ' • ';
      final List<String> lineComponents = line.trim().split(analyzerSeparator);
      final String lintName = lineComponents.removeLast();
      final String location = lineComponents.removeLast();
      return '$location: $lintName';
    } on RangeError catch (err) {
      throw RangeError('Received "$err" while trying to parse: "$line".');
    }
  }

  final String stdout = exec.stdout.toString();
  final List<String> errors = <String>[];
  try {
    bool analyzeLineFound = false;
    const LineSplitter().convert(stdout).forEach((String line) {
      // Conditional to filter out any stdout from `pub get`
      if (!analyzeLineFound && line.startsWith('Analyzing')) {
        analyzeLineFound = true;
        return;
      }

      if (analyzeLineFound && line.trim().isNotEmpty) {
        errors.add(lineParser(line.trim()));
      }
    });
  } on Exception catch (err) {
    fail('$err\n\nComplete STDOUT was:\n\n$stdout');
  }
  expect(errors, unorderedEquals(expectedFailures), reason: 'Failed with stdout:\n\n$stdout');
}

Future<void> ensureFlutterToolsSnapshot() async {
  final String flutterToolsPath = globals.fs.path.absolute(
    globals.fs.path.join('bin', 'flutter_tools.dart'),
  );
  final String flutterToolsSnapshotPath = globals.fs.path.absolute(
    globals.fs.path.join('..', '..', 'bin', 'cache', 'flutter_tools.snapshot'),
  );
  final String packageConfig = globals.fs.path.absolute(
    globals.fs.path.join('.dart_tool', 'package_config.json'),
  );

  final File snapshotFile = globals.fs.file(flutterToolsSnapshotPath);
  if (snapshotFile.existsSync()) {
    snapshotFile.renameSync('$flutterToolsSnapshotPath.bak');
  }

  final List<String> snapshotArgs = <String>[
    '--snapshot=$flutterToolsSnapshotPath',
    '--packages=$packageConfig',
    flutterToolsPath,
  ];
  final ProcessResult snapshotResult = await Process.run(
    '../../bin/cache/dart-sdk/bin/dart',
    snapshotArgs,
  );
  printOnFailure('Results of generating snapshot:');
  printOnFailure(snapshotResult.stdout.toString());
  printOnFailure(snapshotResult.stderr.toString());
  expect(snapshotResult.exitCode, 0);
}

Future<void> restoreFlutterToolsSnapshot() async {
  final String flutterToolsSnapshotPath = globals.fs.path.absolute(
    globals.fs.path.join('..', '..', 'bin', 'cache', 'flutter_tools.snapshot'),
  );

  final File snapshotBackup = globals.fs.file('$flutterToolsSnapshotPath.bak');
  if (!snapshotBackup.existsSync()) {
    // No backup to restore.
    return;
  }

  snapshotBackup.renameSync(flutterToolsSnapshotPath);
}
