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
        'Create $platformName $iosLanguage plugin with Swift Package Manager disabled',
        () async {
          final Directory workingDirectory = fileSystem.systemTempDirectory.createTempSync(
            'swift_package_manager_create_plugin_disabled.',
          );
          final String workingDirectoryPath = workingDirectory.path;
          try {
            await SwiftPackageManagerUtils.disableSwiftPackageManager(
              flutterBin,
              workingDirectoryPath,
            );

            final SwiftPackageManagerPlugin createdCocoaPodsPlugin =
                await SwiftPackageManagerUtils.createPlugin(
                  flutterBin,
                  workingDirectoryPath,
                  platform: platformName,
                  iosLanguage: iosLanguage,
                );

            final String appDirectoryPath = createdCocoaPodsPlugin.exampleAppPath;

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

            final File podspec = fileSystem
                .directory(createdCocoaPodsPlugin.pluginPath)
                .childDirectory(platformName)
                .childFile('${createdCocoaPodsPlugin.pluginName}.podspec');
            expect(podspec.existsSync(), isTrue);
            expect(podspec.readAsStringSync(), contains('Classes'));
            expect(podspec.readAsStringSync().contains('Sources'), isFalse);

            await SwiftPackageManagerUtils.buildApp(
              flutterBin,
              appDirectoryPath,
              options: <String>[platformName, '--debug', '-v'],
              expectedLines: SwiftPackageManagerUtils.expectedLines(
                platform: platformName,
                appDirectoryPath: appDirectoryPath,
                cocoaPodsPlugin: createdCocoaPodsPlugin,
              ),
              unexpectedLines: SwiftPackageManagerUtils.unexpectedLines(
                platform: platformName,
                appDirectoryPath: appDirectoryPath,
                cocoaPodsPlugin: createdCocoaPodsPlugin,
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
        // [intended] Swift Package Manager only works on macos.
        skip: !platform.isMacOS,
      );

      test(
        'Create $platformName $iosLanguage plugin with Swift Package Manager enabled',
        () async {
          final Directory workingDirectory = fileSystem.systemTempDirectory.createTempSync(
            'swift_package_manager_create_plugin_enabled.',
          );
          final String workingDirectoryPath = workingDirectory.path;
          try {
            await SwiftPackageManagerUtils.enableSwiftPackageManager(
              flutterBin,
              workingDirectoryPath,
            );

            final SwiftPackageManagerPlugin createdSwiftPackagePlugin =
                await SwiftPackageManagerUtils.createPlugin(
                  flutterBin,
                  workingDirectoryPath,
                  platform: platformName,
                  iosLanguage: iosLanguage,
                );

            final String appDirectoryPath = createdSwiftPackagePlugin.exampleAppPath;

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

            final File podspec = fileSystem
                .directory(createdSwiftPackagePlugin.pluginPath)
                .childDirectory(platformName)
                .childFile('${createdSwiftPackagePlugin.pluginName}.podspec');
            expect(podspec.existsSync(), isTrue);
            expect(podspec.readAsStringSync(), contains('Sources'));
            expect(podspec.readAsStringSync().contains('Classes'), isFalse);

            await SwiftPackageManagerUtils.buildApp(
              flutterBin,
              appDirectoryPath,
              options: <String>[platformName, '--debug', '-v'],
              expectedLines: SwiftPackageManagerUtils.expectedLines(
                platform: platformName,
                appDirectoryPath: appDirectoryPath,
                swiftPackagePlugin: createdSwiftPackagePlugin,
                swiftPackageMangerEnabled: true,
              ),
              unexpectedLines: SwiftPackageManagerUtils.unexpectedLines(
                platform: platformName,
                appDirectoryPath: appDirectoryPath,
                swiftPackagePlugin: createdSwiftPackagePlugin,
                swiftPackageMangerEnabled: true,
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
        // [intended] Swift Package Manager only works on macos.
        skip: !platform.isMacOS,
      );
    }
  }
}
