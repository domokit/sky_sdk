// Copyright 2014 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

import 'package:meta/meta.dart';

import 'file_system.dart';
import 'logger.dart';

/// Project is generated from a template on Flutter project creation.
/// Sometimes (due to behavior changes in Xcode, Gradle, etc) these files need to be altered
/// from the original template.
abstract class ProjectMigrator {
  ProjectMigrator(this.logger);

  @protected
  final Logger logger;

  Future<void> migrate();

  /// Return null if the line should be deleted.
  @protected
  String? migrateLine(String line) {
    return line;
  }

  @protected
  String migrateFileContents(String fileContents) {
    return fileContents;
  }

  @protected
  bool get migrationRequired => _migrationRequired;
  bool _migrationRequired = false;

  @protected
  /// Calls [migrateLine] per line, then [migrateFileContents]
  /// including the line migrations.
  void processFileLines(File file) {
    final List<String> lines = file.readAsLinesSync();

    final StringBuffer newProjectContents = StringBuffer();
    final String basename = file.basename;

    for (final String line in lines) {
      final String? newProjectLine = migrateLine(line);
      if (newProjectLine == null) {
        logger.printTrace('Migrating $basename, removing:');
        logger.printTrace('    $line');
        _migrationRequired = true;
        continue;
      }
      if (newProjectLine != line) {
        logger.printTrace('Migrating $basename, replacing:');
        logger.printTrace('    $line');
        logger.printTrace('with:');
        logger.printTrace('    $newProjectLine');
        _migrationRequired = true;
      }
      newProjectContents.writeln(newProjectLine);
    }

    final String projectContentsWithMigratedLines = newProjectContents.toString();
    final String projectContentsWithMigratedContents = migrateFileContents(
      projectContentsWithMigratedLines,
    );
    if (projectContentsWithMigratedLines != projectContentsWithMigratedContents) {
      logger.printTrace('Migrating $basename contents');
      _migrationRequired = true;
    }

    if (migrationRequired) {
      logger.printStatus('Upgrading $basename');
      file.writeAsStringSync(projectContentsWithMigratedContents);
    }
  }
}

class ProjectMigration {
  ProjectMigration(this.migrators);

  final List<ProjectMigrator> migrators;

  Future<void> run() async {
    for (final ProjectMigrator migrator in migrators) {
      await migrator.migrate();
    }
  }
}
