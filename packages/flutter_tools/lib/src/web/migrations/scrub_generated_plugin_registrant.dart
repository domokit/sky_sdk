// Copyright 2014 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

import '../../base/file_system.dart';
import '../../base/logger.dart';
import '../../base/project_migrator.dart';
import '../../project.dart';

/// Remove lib/generated_plugin_registrant.dart if it exists.
class ScrubGeneratedPluginRegistrant extends ProjectMigrator {
  ScrubGeneratedPluginRegistrant(WebProject project, super.logger)
    : _project = project,
      _logger = logger;

  final WebProject _project;
  final Logger _logger;

  @override
  Future<void> migrate() async {
    final File registrant = _project.libDirectory.childFile('generated_plugin_registrant.dart');
    final File gitignore = _project.parent.directory.childFile('.gitignore');

    if (!removeFile(registrant)) {
      return;
    }
    if (gitignore.existsSync()) {
      processFileLines(gitignore);
    }
  }

  // Cleans up the .gitignore by removing the line that mentions generated_plugin_registrant.
  @override
  String? migrateLine(String line) {
    return line.contains('lib/generated_plugin_registrant.dart') ? null : line;
  }

  bool removeFile(File file) {
    if (!file.existsSync()) {
      _logger.printTrace('${file.basename} not found. Skipping.');
      return true;
    }

    try {
      file.deleteSync();
      _logger.printStatus('${file.basename} found. Deleted.');
      return true;
    } on FileSystemException catch (e, s) {
      _logger.printError(e.message, stackTrace: s);
    }

    return false;
  }
}
