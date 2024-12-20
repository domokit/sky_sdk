// Copyright 2014 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

import '../../base/file_system.dart';
import '../../base/project_migrator.dart';
import '../../project.dart';

/// This migrator deletes an old file that was created to support enabling
/// Multidex, which is no longer needed and causes builds to fail if not
/// deleted.
class MultidexRemovalMigration extends ProjectMigrator {
  MultidexRemovalMigration(AndroidProject project, super.logger) : _project = project;

  final AndroidProject _project;

  static const String deletionMessage = 'Deleted obsolete FlutterMultiDexApplication.java file.';

  File _getMultiDexApplicationFile() {
    return _project.hostAppGradleRoot
        .childDirectory('src')
        .childDirectory('main')
        .childDirectory('java')
        .childDirectory('io')
        .childDirectory('flutter')
        .childDirectory('app')
        .childFile('FlutterMultiDexApplication.java');
  }

  @override
  Future<void> migrate() async {
    final File multiDexApplicationFile = _getMultiDexApplicationFile();
    if (multiDexApplicationFile.existsSync()) {
      multiDexApplicationFile.deleteSync();
      logger.printTrace(deletionMessage);
    }
  }
}
