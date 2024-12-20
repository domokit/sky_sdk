// Copyright 2014 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

import '../../base/file_system.dart';
import '../../base/project_migrator.dart';
import '../../project.dart';

const String _eagerCleanTaskDeclaration = '''
task clean(type: Delete) {
    delete rootProject.buildDir
}
''';

const String _lazyCleanTaskDeclaration = '''
tasks.register("clean", Delete) {
    delete rootProject.layout.buildDirectory
}
''';

/// Migrate the Gradle "clean" task to use modern, lazy declaration style.
class TopLevelGradleBuildFileMigration extends ProjectMigrator {
  TopLevelGradleBuildFileMigration(AndroidProject project, super.logger)
    : _topLevelGradleBuildFile = project.hostAppGradleRoot.childFile('build.gradle');

  final File _topLevelGradleBuildFile;

  @override
  Future<void> migrate() async {
    if (!_topLevelGradleBuildFile.existsSync()) {
      logger.printTrace(
        'Top-level Gradle build file not found, skipping migration of task "clean".',
      );
      return;
    }

    processFileLines(_topLevelGradleBuildFile);
  }

  @override
  String migrateFileContents(String fileContents) {
    final String newContents = fileContents.replaceAll(
      _eagerCleanTaskDeclaration,
      _lazyCleanTaskDeclaration,
    );

    if (newContents != fileContents) {
      logger.printTrace('Migrating "clean" Gradle task to lazy declaration style.');
    }

    return newContents;
  }
}
