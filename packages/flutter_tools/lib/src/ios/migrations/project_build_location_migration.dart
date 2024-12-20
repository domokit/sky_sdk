// Copyright 2014 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

import '../../base/file_system.dart';
import '../../base/project_migrator.dart';
import '../../xcode_project.dart';

// Update the xcodeproj build location. Legacy build location does not work with Swift Packages.
class ProjectBuildLocationMigration extends ProjectMigrator {
  ProjectBuildLocationMigration(IosProject project, super.logger)
    : _xcodeProjectWorkspaceData = project.xcodeProjectWorkspaceData;

  final File _xcodeProjectWorkspaceData;

  @override
  Future<void> migrate() async {
    if (!_xcodeProjectWorkspaceData.existsSync()) {
      logger.printTrace(
        'Xcode project workspace data not found, skipping build location migration.',
      );
      return;
    }

    processFileLines(_xcodeProjectWorkspaceData);
  }

  @override
  String migrateLine(String line) {
    const String legacyBuildLocation = 'location = "group:Runner.xcodeproj"';
    const String defaultBuildLocation = 'location = "self:"';

    return line.replaceAll(legacyBuildLocation, defaultBuildLocation);
  }

  @override
  String migrateFileContents(String fileContents) {
    const String podLocation = '''
   <FileRef
      location = "group:Pods/Pods.xcodeproj">
   </FileRef>
''';

    return fileContents.replaceAll(podLocation, '');
  }
}
