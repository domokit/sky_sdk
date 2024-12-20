// Copyright 2014 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

import 'package:flutter_tools/src/base/error_handling_io.dart';
import 'package:flutter_tools/src/base/file_system.dart';

import '../src/common.dart';
import 'swift_package_manager_utils.dart';
import 'test_utils.dart';

void main() {
  final List<String> platforms = <String>['ios', 'macos'];
  for (final String platformName in platforms) {
    final List<String> iosLanguages = <String>[if (platformName == 'ios') 'objc', 'swift'];

    for (final String iosLanguage in iosLanguages) {
      test(
        'Create $platformName $iosLanguage app with Swift Package Manager disabled',
        () async {
          final Directory workingDirectory = fileSystem.systemTempDirectory.createTempSync(
            'swift_package_manager_create_app_disabled.',
          );
          final String workingDirectoryPath = workingDirectory.path;
          try {
            await SwiftPackageManagerUtils.disableSwiftPackageManager(
              flutterBin,
              workingDirectoryPath,
            );

            final String appDirectoryPath = await SwiftPackageManagerUtils.createApp(
              flutterBin,
              workingDirectoryPath,
              iosLanguage: iosLanguage,
              platform: platformName,
              options: <String>['--platforms=$platformName'],
            );

            final File pbxprojFile = fileSystem
                .directory(appDirectoryPath)
                .childDirectory(platformName)
                .childDirectory('Runner.xcodeproj')
                .childFile('project.pbxproj');
            expect(pbxprojFile.existsSync(), isTrue);
            expect(
              pbxprojFile.readAsStringSync().contains('FlutterGeneratedPluginSwiftPackage'),
              isFalse,
            );

            final File xcschemeFile = fileSystem
                .directory(appDirectoryPath)
                .childDirectory(platformName)
                .childDirectory('Runner.xcodeproj')
                .childDirectory('xcshareddata')
                .childDirectory('xcschemes')
                .childFile('Runner.xcscheme');
            expect(xcschemeFile.existsSync(), isTrue);
            expect(
              xcschemeFile.readAsStringSync().contains('Run Prepare Flutter Framework Script'),
              isFalse,
            );

            await SwiftPackageManagerUtils.buildApp(
              flutterBin,
              appDirectoryPath,
              options: <String>[platformName, '--debug', '-v'],
              expectedLines: SwiftPackageManagerUtils.expectedLines(
                platform: platformName,
                appDirectoryPath: appDirectoryPath,
              ),
              unexpectedLines: SwiftPackageManagerUtils.unexpectedLines(
                platform: platformName,
                appDirectoryPath: appDirectoryPath,
              ),
            );
          } finally {
            await SwiftPackageManagerUtils.disableSwiftPackageManager(
              flutterBin,
              workingDirectoryPath,
            );
            ErrorHandlingFileSystem.deleteIfExists(workingDirectory, recursive: true);
          }
        },
        skip: !platform.isMacOS, // [intended] Swift Package Manager only works on macos.
      );

      test(
        'Create $platformName $iosLanguage app with Swift Package Manager enabled',
        () async {
          final Directory workingDirectory = fileSystem.systemTempDirectory.createTempSync(
            'swift_package_manager_create_app_enabled.',
          );
          final String workingDirectoryPath = workingDirectory.path;
          try {
            await SwiftPackageManagerUtils.enableSwiftPackageManager(
              flutterBin,
              workingDirectoryPath,
            );

            final String appDirectoryPath = await SwiftPackageManagerUtils.createApp(
              flutterBin,
              workingDirectoryPath,
              iosLanguage: iosLanguage,
              platform: platformName,
              options: <String>['--platforms=$platformName'],
            );

            final File pbxprojFile = fileSystem
                .directory(appDirectoryPath)
                .childDirectory(platformName)
                .childDirectory('Runner.xcodeproj')
                .childFile('project.pbxproj');
            expect(pbxprojFile.existsSync(), isTrue);
            expect(pbxprojFile.readAsStringSync(), contains('FlutterGeneratedPluginSwiftPackage'));

            final File xcschemeFile = fileSystem
                .directory(appDirectoryPath)
                .childDirectory(platformName)
                .childDirectory('Runner.xcodeproj')
                .childDirectory('xcshareddata')
                .childDirectory('xcschemes')
                .childFile('Runner.xcscheme');
            expect(xcschemeFile.existsSync(), isTrue);
            expect(
              xcschemeFile.readAsStringSync(),
              contains('Run Prepare Flutter Framework Script'),
            );

            await SwiftPackageManagerUtils.buildApp(
              flutterBin,
              appDirectoryPath,
              options: <String>[platformName, '--debug', '-v'],
              expectedLines: SwiftPackageManagerUtils.expectedLines(
                platform: platformName,
                appDirectoryPath: appDirectoryPath,
              ),
              unexpectedLines: SwiftPackageManagerUtils.unexpectedLines(
                platform: platformName,
                appDirectoryPath: appDirectoryPath,
              ),
            );
          } finally {
            await SwiftPackageManagerUtils.disableSwiftPackageManager(
              flutterBin,
              workingDirectoryPath,
            );
            ErrorHandlingFileSystem.deleteIfExists(workingDirectory, recursive: true);
          }
        },
        skip: !platform.isMacOS, // [intended] Swift Package Manager only works on macos.
      );
    }
  }
}
