// Copyright 2014 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

import '../../base/file_system.dart';
import '../../base/project_migrator.dart';
import '../../xcode_project.dart';

const String _appDelegateFileBefore = r'''
@UIApplicationMain
@objc class AppDelegate''';

const String _appDelegateFileAfter = r'''
@main
@objc class AppDelegate''';

/// Replace the deprecated `@UIApplicationMain` attribute with `@main`.
///
/// See:
/// https://github.com/apple/swift-evolution/blob/main/proposals/0383-deprecate-uiapplicationmain-and-nsapplicationmain.md
class UIApplicationMainDeprecationMigration extends ProjectMigrator {
  UIApplicationMainDeprecationMigration(IosProject project, super.logger)
    : _appDelegateSwift = project.appDelegateSwift;

  final File _appDelegateSwift;

  @override
  Future<void> migrate() async {
    // Skip this migration if the project uses Objective-C.
    if (!_appDelegateSwift.existsSync()) {
      logger.printTrace('ios/Runner/AppDelegate.swift not found, skipping @main migration.');
      return;
    }

    // Migrate the ios/Runner/AppDelegate.swift file.
    final String original = _appDelegateSwift.readAsStringSync();
    final String migrated = original.replaceFirst(_appDelegateFileBefore, _appDelegateFileAfter);
    if (original == migrated) {
      return;
    }

    logger.printWarning(
      'ios/Runner/AppDelegate.swift uses the deprecated @UIApplicationMain attribute, updating.',
    );
    _appDelegateSwift.writeAsStringSync(migrated);
  }
}
