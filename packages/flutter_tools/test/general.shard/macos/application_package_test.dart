// Copyright 2014 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

import 'dart:convert';

import 'package:file/file.dart';
import 'package:file/memory.dart';
import 'package:flutter_tools/src/base/file_system.dart';
import 'package:flutter_tools/src/base/logger.dart';
import 'package:flutter_tools/src/base/os.dart';
import 'package:flutter_tools/src/base/utils.dart';
import 'package:flutter_tools/src/build_info.dart';
import 'package:flutter_tools/src/globals.dart' as globals;
import 'package:flutter_tools/src/ios/plist_parser.dart';
import 'package:flutter_tools/src/macos/application_package.dart';
import 'package:flutter_tools/src/project.dart';
import 'package:test/fake.dart';

import '../../src/common.dart';
import '../../src/context.dart';

void main() {
  group('PrebuiltMacOSApp', () {
    late FakeOperatingSystemUtils os;
    late FileSystem fileSystem;
    late BufferLogger logger;

    final Map<Type, Generator> overrides = <Type, Generator>{
      FileSystem: () => fileSystem,
      ProcessManager: () => FakeProcessManager.any(),
      PlistParser: () => FakePlistUtils(fileSystem),
      OperatingSystemUtils: () => os,
      Logger: () => logger,
    };

    setUp(() {
      fileSystem = MemoryFileSystem.test();
      os = FakeOperatingSystemUtils();
      logger = BufferLogger.test();
    });

    testUsingContext('Error on non-existing file', () {
      final PrebuiltMacOSApp? macosApp =
          MacOSApp.fromPrebuiltApp(fileSystem.file('not_existing.app')) as PrebuiltMacOSApp?;

      expect(macosApp, isNull);
      expect(logger.errorText, contains('File "not_existing.app" does not exist.'));
    }, overrides: overrides);

    testUsingContext('Error on non-app-bundle folder', () {
      fileSystem.directory('regular_folder').createSync();
      final PrebuiltMacOSApp? macosApp =
          MacOSApp.fromPrebuiltApp(fileSystem.file('regular_folder')) as PrebuiltMacOSApp?;

      expect(macosApp, isNull);
      expect(logger.errorText, contains('Folder "regular_folder" is not an app bundle.'));
    }, overrides: overrides);

    testUsingContext('Error on no info.plist', () {
      fileSystem.directory('bundle.app').createSync();
      final PrebuiltMacOSApp? macosApp =
          MacOSApp.fromPrebuiltApp(fileSystem.file('bundle.app')) as PrebuiltMacOSApp?;

      expect(macosApp, isNull);
      expect(
        logger.errorText,
        contains('Invalid prebuilt macOS app. Does not contain Info.plist.'),
      );
    }, overrides: overrides);

    testUsingContext('Error on info.plist missing bundle identifier', () {
      final String contentsDirectory = fileSystem.path.join('bundle.app', 'Contents');
      fileSystem.directory(contentsDirectory).createSync(recursive: true);
      fileSystem
          .file(fileSystem.path.join('bundle.app', 'Contents', 'Info.plist'))
          .writeAsStringSync(badPlistData);
      final PrebuiltMacOSApp? macosApp =
          MacOSApp.fromPrebuiltApp(fileSystem.file('bundle.app')) as PrebuiltMacOSApp?;

      expect(macosApp, isNull);
      expect(
        logger.errorText,
        contains('Invalid prebuilt macOS app. Info.plist does not contain bundle identifier'),
      );
    }, overrides: overrides);

    testUsingContext('Error on info.plist missing executable', () {
      final String contentsDirectory = fileSystem.path.join('bundle.app', 'Contents');
      fileSystem.directory(contentsDirectory).createSync(recursive: true);
      fileSystem
          .file(fileSystem.path.join('bundle.app', 'Contents', 'Info.plist'))
          .writeAsStringSync(badPlistDataNoExecutable);
      final PrebuiltMacOSApp? macosApp =
          MacOSApp.fromPrebuiltApp(fileSystem.file('bundle.app')) as PrebuiltMacOSApp?;

      expect(macosApp, isNull);
      expect(
        logger.errorText,
        contains('Invalid prebuilt macOS app. Info.plist does not contain bundle executable'),
      );
    }, overrides: overrides);

    testUsingContext('Success with app bundle', () {
      final String appDirectory = fileSystem.path.join('bundle.app', 'Contents', 'MacOS');
      fileSystem.directory(appDirectory).createSync(recursive: true);
      fileSystem
          .file(fileSystem.path.join('bundle.app', 'Contents', 'Info.plist'))
          .writeAsStringSync(plistData);
      fileSystem.file(fileSystem.path.join(appDirectory, executableName)).createSync();
      final PrebuiltMacOSApp macosApp =
          MacOSApp.fromPrebuiltApp(fileSystem.file('bundle.app'))! as PrebuiltMacOSApp;

      expect(logger.errorText, isEmpty);
      expect(macosApp.uncompressedBundle.path, 'bundle.app');
      expect(macosApp.id, 'fooBundleId');
      expect(macosApp.bundleName, 'bundle.app');
    }, overrides: overrides);

    testUsingContext('Bad zipped app, no payload dir', () {
      fileSystem.file('app.zip').createSync();
      final PrebuiltMacOSApp? macosApp =
          MacOSApp.fromPrebuiltApp(fileSystem.file('app.zip')) as PrebuiltMacOSApp?;

      expect(macosApp, isNull);
      expect(logger.errorText, contains('Archive "app.zip" does not contain a single app bundle.'));
    }, overrides: overrides);

    testUsingContext('Bad zipped app, two app bundles', () {
      fileSystem.file('app.zip').createSync();
      os.unzipOverride = (File zipFile, Directory targetDirectory) {
        if (zipFile.path != 'app.zip') {
          return;
        }
        final String bundlePath1 = fileSystem.path.join(targetDirectory.path, 'bundle1.app');
        final String bundlePath2 = fileSystem.path.join(targetDirectory.path, 'bundle2.app');
        fileSystem.directory(bundlePath1).createSync(recursive: true);
        fileSystem.directory(bundlePath2).createSync(recursive: true);
      };
      final PrebuiltMacOSApp? macosApp =
          MacOSApp.fromPrebuiltApp(fileSystem.file('app.zip')) as PrebuiltMacOSApp?;

      expect(macosApp, isNull);
      expect(logger.errorText, contains('Archive "app.zip" does not contain a single app bundle.'));
    }, overrides: overrides);

    testUsingContext('Success with zipped app', () {
      fileSystem.file('app.zip').createSync();
      os.unzipOverride = (File zipFile, Directory targetDirectory) {
        if (zipFile.path != 'app.zip') {
          return;
        }
        final Directory bundleAppContentsDir = fileSystem.directory(
          fileSystem.path.join(targetDirectory.path, 'bundle.app', 'Contents'),
        );
        bundleAppContentsDir.createSync(recursive: true);
        fileSystem
            .file(fileSystem.path.join(bundleAppContentsDir.path, 'Info.plist'))
            .writeAsStringSync(plistData);
        fileSystem.directory(fileSystem.path.join(bundleAppContentsDir.path, 'MacOS')).createSync();
        fileSystem
            .file(fileSystem.path.join(bundleAppContentsDir.path, 'MacOS', executableName))
            .createSync();
      };
      final PrebuiltMacOSApp macosApp =
          MacOSApp.fromPrebuiltApp(fileSystem.file('app.zip'))! as PrebuiltMacOSApp;

      expect(logger.errorText, isEmpty);
      expect(macosApp.uncompressedBundle.path, endsWith('bundle.app'));
      expect(macosApp.id, 'fooBundleId');
      expect(macosApp.bundleName, endsWith('bundle.app'));
    }, overrides: overrides);

    testUsingContext('Success with project', () {
      final MacOSApp macosApp = MacOSApp.fromMacOSProject(
        FlutterProject.fromDirectory(globals.fs.currentDirectory).macos,
      );

      expect(logger.errorText, isEmpty);
      expect(macosApp.id, 'com.example.placeholder');
      expect(macosApp.name, 'macOS');
    }, overrides: overrides);

    testUsingContext('Chooses the correct directory for application.', () {
      final MacOSProject project = FlutterProject.fromDirectory(globals.fs.currentDirectory).macos;
      final BuildableMacOSApp macosApp = MacOSApp.fromMacOSProject(project) as BuildableMacOSApp;

      const BuildInfo vanillaApp = BuildInfo(
        BuildMode.debug,
        null,
        treeShakeIcons: false,
        packageConfigPath: '.dart_tool/package_config.json',
      );
      String? applicationBundle = macosApp.bundleDirectory(vanillaApp);
      expect(applicationBundle, 'Debug');

      const BuildInfo flavoredApp = BuildInfo(
        BuildMode.release,
        'flavor',
        treeShakeIcons: false,
        packageConfigPath: '.dart_tool/package_config.json',
      );
      applicationBundle = macosApp.bundleDirectory(flavoredApp);
      expect(applicationBundle, 'Release-flavor');
    }, overrides: overrides);
  });
}

class FakeOperatingSystemUtils extends Fake implements OperatingSystemUtils {
  FakeOperatingSystemUtils();

  void Function(File, Directory)? unzipOverride;

  @override
  void unzip(File file, Directory targetDirectory) {
    unzipOverride?.call(file, targetDirectory);
  }
}

class FakePlistUtils extends Fake implements PlistParser {
  FakePlistUtils(this.fileSystem);

  final FileSystem? fileSystem;

  @override
  Map<String, Object> parseFile(String plistFilePath) {
    final File file = fileSystem!.file(plistFilePath);
    if (!file.existsSync()) {
      return <String, Object>{};
    }
    return castStringKeyedMap(json.decode(file.readAsStringSync()))!.cast();
  }
}

// Contains no bundle identifier.
const String badPlistData = '''
{}
''';

// Contains no bundle executable.
const String badPlistDataNoExecutable = '''
{"CFBundleIdentifier": "fooBundleId"}
''';

const String executableName = 'foo';

const String plistData = '''
{"CFBundleIdentifier": "fooBundleId", "CFBundleExecutable": "$executableName"}
''';
