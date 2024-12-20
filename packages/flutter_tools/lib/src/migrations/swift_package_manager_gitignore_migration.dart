// Copyright 2014 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

import '../base/file_system.dart';
import '../base/project_migrator.dart';
import '../project.dart';

const String _gitignoreBefore = '''
.DS_Store
.atom/
.buildlog/
.history
.svn/
migrate_working_dir/
''';

const String _gitignoreAfter = '''
.DS_Store
.atom/
.build/
.buildlog/
.history
.svn/
.swiftpm/
migrate_working_dir/
''';

/// Adds `.build/` and `.swiftpm/` to the .gitignore file.
class SwiftPackageManagerGitignoreMigration extends ProjectMigrator {
  SwiftPackageManagerGitignoreMigration(FlutterProject project, super.logger)
    : _gitignoreFile = project.gitignoreFile;

  final File _gitignoreFile;

  @override
  Future<void> migrate() async {
    if (!_gitignoreFile.existsSync()) {
      logger.printTrace(
        '.gitignore file not found, skipping Swift Package Manager .gitignore migration.',
      );
      return;
    }

    final String originalContent = _gitignoreFile.readAsStringSync();

    // Skip if .gitignore is already migrated.
    if (originalContent.contains('.build/') && originalContent.contains('.swiftpm/')) {
      return;
    }

    final String newContent = originalContent.replaceFirst(_gitignoreBefore, _gitignoreAfter);
    if (newContent != originalContent) {
      logger.printWarning(
        '.gitignore does not ignore Swift Package Manager build directories, updating.',
      );
      _gitignoreFile.writeAsStringSync(newContent);
    }
  }
}
