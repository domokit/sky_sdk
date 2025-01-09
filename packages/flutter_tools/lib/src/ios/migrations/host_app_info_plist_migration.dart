// Copyright 2014 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

import '../../base/file_system.dart';
import '../../base/project_migrator.dart';
import '../../xcode_project.dart';

const String _kDisableMinimumFrameDurationKey = 'CADisableMinimumFrameDurationOnPhone';
const String _kIndirectInputEventsKey = 'UIApplicationSupportsIndirectInputEvents';

/// Update Info.plist.
class HostAppInfoPlistMigration extends ProjectMigrator {
  HostAppInfoPlistMigration(IosProject project, super.logger)
    : _infoPlist = project.defaultHostInfoPlist;

  final File _infoPlist;

  @override
  Future<void> migrate() async {
    if (!_infoPlist.existsSync()) {
      logger.printTrace('Info.plist not found, skipping host app Info.plist migration.');
      return;
    }

    processFileLines(_infoPlist);
  }

  @override
  String migrateFileContents(String fileContents) {
    String newContents = fileContents;
    if (!newContents.contains(_kDisableMinimumFrameDurationKey)) {
      logger.printTrace('Adding $_kDisableMinimumFrameDurationKey to Info.plist');
      const String plistEnd = '''
</dict>
</plist>
''';
      const String plistWithKey = '''
	<key>$_kDisableMinimumFrameDurationKey</key>
	<true/>
</dict>
</plist>
''';
      newContents = newContents.replaceAll(plistEnd, plistWithKey);
    }

    if (!newContents.contains(_kIndirectInputEventsKey)) {
      logger.printTrace('Adding $_kIndirectInputEventsKey to Info.plist');
      const String plistEnd = '''
</dict>
</plist>
''';
      const String plistWithKey = '''
	<key>$_kIndirectInputEventsKey</key>
	<true/>
</dict>
</plist>
''';
      newContents = newContents.replaceAll(plistEnd, plistWithKey);
    }

    return newContents;
  }
}
