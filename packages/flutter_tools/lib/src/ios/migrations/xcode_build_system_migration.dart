// Copyright 2014 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

import '../../base/file_system.dart';
import '../../base/project_migrator.dart';
import '../../xcode_project.dart';

// Xcode legacy build system no longer supported by Xcode.
// Set in https://github.com/flutter/flutter/pull/21901/.
// Removed in https://github.com/flutter/flutter/pull/33684.
class XcodeBuildSystemMigration extends ProjectMigrator {
  XcodeBuildSystemMigration(IosProject project, super.logger)
    : _xcodeWorkspaceSharedSettings = project.xcodeWorkspaceSharedSettings;

  final File? _xcodeWorkspaceSharedSettings;

  @override
  Future<void> migrate() async {
    final File? xcodeWorkspaceSharedSettings = _xcodeWorkspaceSharedSettings;
    if (xcodeWorkspaceSharedSettings == null || !xcodeWorkspaceSharedSettings.existsSync()) {
      logger.printTrace('Xcode workspace settings not found, skipping build system migration');
      return;
    }

    final String contents = xcodeWorkspaceSharedSettings.readAsStringSync();

    // Only delete this file when it is pointing to the legacy build system.
    const String legacyBuildSettingsWorkspace = '''
	<key>BuildSystemType</key>
	<string>Original</string>''';

    // contains instead of equals to ignore newline file ending variance.
    if (contents.contains(legacyBuildSettingsWorkspace)) {
      logger.printStatus(
        'Legacy build system detected, removing ${xcodeWorkspaceSharedSettings.path}',
      );
      xcodeWorkspaceSharedSettings.deleteSync();
    }
  }
}
