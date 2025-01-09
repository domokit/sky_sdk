// Copyright 2014 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

import '../../base/file_system.dart';
import '../../base/project_migrator.dart';
import '../../xcode_project.dart';

/// Update the minimum macOS deployment version to the minimum allowed by Xcode without causing a warning.
class MacOSDeploymentTargetMigration extends ProjectMigrator {
  MacOSDeploymentTargetMigration(MacOSProject project, super.logger)
    : _xcodeProjectInfoFile = project.xcodeProjectInfoFile,
      _podfile = project.podfile;

  final File _xcodeProjectInfoFile;
  final File _podfile;

  @override
  Future<void> migrate() async {
    if (_xcodeProjectInfoFile.existsSync()) {
      processFileLines(_xcodeProjectInfoFile);
    } else {
      logger.printTrace(
        'Xcode project not found, skipping macOS deployment target version migration.',
      );
    }

    if (_podfile.existsSync()) {
      processFileLines(_podfile);
    } else {
      logger.printTrace('Podfile not found, skipping global platform macOS version migration.');
    }
  }

  @override
  String? migrateLine(String line) {
    // Xcode project file changes.
    const String deploymentTargetOriginal1011 = 'MACOSX_DEPLOYMENT_TARGET = 10.11;';
    const String deploymentTargetOriginal1013 = 'MACOSX_DEPLOYMENT_TARGET = 10.13;';

    // Podfile changes.
    const String podfilePlatformVersionOriginal1011 = "platform :osx, '10.11'";
    const String podfilePlatformVersionOriginal1013 = "platform :osx, '10.13'";

    if (line.contains(deploymentTargetOriginal1011) ||
        line.contains(deploymentTargetOriginal1013) ||
        line.contains(podfilePlatformVersionOriginal1011) ||
        line.contains(podfilePlatformVersionOriginal1013)) {
      if (!migrationRequired) {
        // Only print for the first discovered change found.
        logger.printStatus('Updating minimum macOS deployment target to 10.14.');
      }

      const String deploymentTargetReplacement = 'MACOSX_DEPLOYMENT_TARGET = 10.14;';
      const String podfilePlatformVersionReplacement = "platform :osx, '10.14'";
      return line
          .replaceAll(deploymentTargetOriginal1011, deploymentTargetReplacement)
          .replaceAll(deploymentTargetOriginal1013, deploymentTargetReplacement)
          .replaceAll(podfilePlatformVersionOriginal1011, podfilePlatformVersionReplacement)
          .replaceAll(podfilePlatformVersionOriginal1013, podfilePlatformVersionReplacement);
    }

    return line;
  }
}
