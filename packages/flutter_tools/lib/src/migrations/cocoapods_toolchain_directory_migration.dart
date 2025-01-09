// Copyright 2014 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

import '../base/file_system.dart';
import '../base/project_migrator.dart';
import '../base/version.dart';
import '../ios/xcodeproj.dart';
import '../xcode_project.dart';

/// Starting in Xcode 15, when building macOS, DT_TOOLCHAIN_DIR cannot be used
/// to evaluate LD_RUNPATH_SEARCH_PATHS or LIBRARY_SEARCH_PATHS. `xcodebuild`
/// error message recommend using TOOLCHAIN_DIR instead.
///
/// This has been fixed upstream in CocoaPods, but migrate a copy of their
/// workaround so users don't need to update.
class CocoaPodsToolchainDirectoryMigration extends ProjectMigrator {
  CocoaPodsToolchainDirectoryMigration(
    XcodeBasedProject project,
    XcodeProjectInterpreter xcodeProjectInterpreter,
    super.logger,
  ) : _podRunnerTargetSupportFiles = project.podRunnerTargetSupportFiles,
      _xcodeProjectInterpreter = xcodeProjectInterpreter;

  final Directory _podRunnerTargetSupportFiles;
  final XcodeProjectInterpreter _xcodeProjectInterpreter;

  @override
  Future<void> migrate() async {
    if (!_podRunnerTargetSupportFiles.existsSync()) {
      logger.printTrace(
        'CocoaPods Pods-Runner Target Support Files not found, skipping TOOLCHAIN_DIR workaround.',
      );
      return;
    }

    final Version? version = _xcodeProjectInterpreter.version;

    // If Xcode not installed or less than 15, skip this migration.
    if (version == null || version < Version(15, 0, 0)) {
      logger.printTrace(
        'Detected Xcode version is $version, below 15.0, skipping TOOLCHAIN_DIR workaround.',
      );
      return;
    }

    final List<FileSystemEntity> files = _podRunnerTargetSupportFiles.listSync();
    for (final FileSystemEntity file in files) {
      if (file.basename.endsWith('xcconfig') && file is File) {
        processFileLines(file);
      }
    }
  }

  @override
  String? migrateLine(String line) {
    final String trimmedString = line.trim();
    if (trimmedString.startsWith('LD_RUNPATH_SEARCH_PATHS') ||
        trimmedString.startsWith('LIBRARY_SEARCH_PATHS')) {
      const String originalReadLinkLine = r'{DT_TOOLCHAIN_DIR}';
      const String replacementReadLinkLine = r'{TOOLCHAIN_DIR}';

      return line.replaceAll(originalReadLinkLine, replacementReadLinkLine);
    }
    return line;
  }
}
