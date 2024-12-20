// Copyright 2014 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

import 'dart:io' show Directory, File;

import 'package:file/file.dart' as fs;
import 'package:file/local.dart';
import 'package:path/path.dart' as path;

import '../run_command.dart';
import '../utils.dart';

/// Executes the test suite for the flutter/packages repo.
Future<void> flutterPackagesRunner() async {
  Future<void> runAnalyze() async {
    printProgress('${green}Running analysis for flutter/packages$reset');
    final Directory checkout = Directory.systemTemp.createTempSync('flutter_packages.');
    await runCommand('git', const <String>[
      '-c',
      'core.longPaths=true',
      'clone',
      'https://github.com/flutter/packages.git',
      '.',
    ], workingDirectory: checkout.path);
    final String packagesCommit = await getFlutterPackagesVersion(flutterRoot: flutterRoot);
    await runCommand('git', <String>[
      '-c',
      'core.longPaths=true',
      'checkout',
      packagesCommit,
    ], workingDirectory: checkout.path);
    // Prep the repository tooling.
    // This test does not use tool_runner.sh because in this context the test
    // should always run on the entire packages repo, while tool_runner.sh
    // is designed for flutter/packages CI and only analyzes changed repository
    // files when run for anything but master.
    final String toolDir = path.join(checkout.path, 'script', 'tool');
    await runCommand('dart', const <String>['pub', 'get'], workingDirectory: toolDir);
    final String toolScript = path.join(toolDir, 'bin', 'flutter_plugin_tools.dart');
    await runCommand('dart', <String>[
      'run',
      toolScript,
      'analyze',
      // Fetch the oldest possible dependencies, rather than the newest, to
      // insulate flutter/flutter from out-of-band failures when new versions
      // of dependencies are published. This compensates for the fact that
      // flutter/packages doesn't use pinned dependencies, and for the
      // purposes of this test using old dependencies is fine. See
      // https://github.com/flutter/flutter/issues/129633
      '--downgrade',
      '--custom-analysis=script/configs/custom_analysis.yaml',
    ], workingDirectory: checkout.path);
  }

  await selectSubshard(<String, ShardRunner>{'analyze': runAnalyze});
}

/// Returns the commit hash of the flutter/packages repository that's rolled in.
///
/// The flutter/packages repository is a downstream dependency, it is only used
/// by flutter/flutter for testing purposes, to assure stable tests for a given
/// flutter commit the flutter/packages commit hash to test against is coded in
/// the bin/internal/flutter_packages.version file.
///
/// The `filesystem` parameter specified filesystem to read the packages version file from.
/// The `packagesVersionFile` parameter allows specifying an alternative path for the
/// packages version file, when null [flutterPackagesVersionFile] is used.
Future<String> getFlutterPackagesVersion({
  fs.FileSystem fileSystem = const LocalFileSystem(),
  String? packagesVersionFile,
  required String flutterRoot,
}) async {
  final String flutterPackagesVersionFile = path.join(
    flutterRoot,
    'bin',
    'internal',
    'flutter_packages.version',
  );

  final File versionFile = fileSystem.file(packagesVersionFile ?? flutterPackagesVersionFile);
  final String versionFileContents = await versionFile.readAsString();
  return versionFileContents.trim();
}
