// Copyright 2014 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

import '../../base/file_system.dart';
import '../../base/project_migrator.dart';
import '../../xcode_project.dart';

const String _appDelegateFileBefore = r'''
class AppDelegate: FlutterAppDelegate {
  override func applicationShouldTerminateAfterLastWindowClosed(_ sender: NSApplication) -> Bool {
    return true
  }
}''';

const String _appDelegateFileAfter = r'''
class AppDelegate: FlutterAppDelegate {
  override func applicationShouldTerminateAfterLastWindowClosed(_ sender: NSApplication) -> Bool {
    return true
  }

  override func applicationSupportsSecureRestorableState(_ app: NSApplication) -> Bool {
    return true
  }
}''';

/// Add `applicationSupportsSecureRestorableState` if not already present.
///
/// In all new AppKit apps since Xcode 13.2, the AppDelegate template includes
/// this method, which opts in to requiring safe deserialization via the
/// `NSSecureCoding` protocol. Because this required new API, existing apps
/// need to opt in to this behavior.
///
/// Since nearly all Flutter macOS apps will be doing serialization of Flutter
/// state via Dart code, it's a very safe bet that the vast majority of
/// existing Flutter apps can safely enable this flag. The few apps that
/// are doing serialization via older insecure APIs can update the migrated
/// code to return false.
///
/// See:
/// https://developer.apple.com/documentation/foundation/nssecurecoding?language=objc
class SecureRestorableStateMigration extends ProjectMigrator {
  SecureRestorableStateMigration(MacOSProject project, super.logger)
    : _appDelegateSwift = project.appDelegateSwift;

  final File _appDelegateSwift;

  @override
  Future<void> migrate() async {
    // Skip this migration if the project uses Objective-C.
    if (!_appDelegateSwift.existsSync()) {
      logger.printTrace(
        'macos/Runner/AppDelegate.swift not found. Skipping applicationSupportsSecureRestorableState migration.',
      );
      return;
    }
    final String original = _appDelegateSwift.readAsStringSync();

    // If we have an AppDelegate.swift, but can't migrate, log a warning.
    if (!original.contains(_appDelegateFileBefore)) {
      if (original.contains('applicationSupportsSecureRestorableState')) {
        // User has already overridden this method. Exit quietly.
        return;
      }

      logger.printWarning('''
macos/Runner/AppDelegate.swift has been modified and cannot be automatically migrated.
We recommend developers override applicationSupportsSecureRestorableState in AppDelegate.swift as follows:
override func applicationSupportsSecureRestorableState(_ app: NSApplication) -> Bool {
  return true
}
''');
    }

    // Migrate the macos/Runner/AppDelegate.swift file.
    final String migrated = original.replaceFirst(_appDelegateFileBefore, _appDelegateFileAfter);
    if (original == migrated) {
      return;
    }

    logger.printWarning(
      'macos/Runner/AppDelegate.swift does not override applicationSupportsSecureRestorableState. Updating.',
    );
    _appDelegateSwift.writeAsStringSync(migrated);
  }
}
