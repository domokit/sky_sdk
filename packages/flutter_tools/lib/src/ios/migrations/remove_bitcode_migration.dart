// Copyright 2014 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

import '../../base/file_system.dart';
import '../../base/project_migrator.dart';
import '../../xcode_project.dart';

/// Remove deprecated bitcode build setting.
class RemoveBitcodeMigration extends ProjectMigrator {
  RemoveBitcodeMigration(IosProject project, super.logger)
    : _xcodeProjectInfoFile = project.xcodeProjectInfoFile;

  final File _xcodeProjectInfoFile;

  @override
  Future<void> migrate() async {
    if (_xcodeProjectInfoFile.existsSync()) {
      processFileLines(_xcodeProjectInfoFile);
    } else {
      logger.printTrace('Xcode project not found, skipping removing bitcode migration.');
    }
  }

  @override
  String? migrateLine(String line) {
    if (line.contains('ENABLE_BITCODE = YES;')) {
      if (!migrationRequired) {
        // Only print for the first discovered change found.
        logger.printWarning(
          'Disabling deprecated bitcode Xcode build setting. See https://github.com/flutter/flutter/issues/107887 for additional details.',
        );
      }
      return line.replaceAll('ENABLE_BITCODE = YES', 'ENABLE_BITCODE = NO');
    }

    return line;
  }
}
