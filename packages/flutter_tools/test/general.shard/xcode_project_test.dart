// Copyright 2014 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

import 'package:file/file.dart';
import 'package:file/memory.dart';
import 'package:flutter_tools/src/base/file_system.dart';
import 'package:flutter_tools/src/base/logger.dart';
import 'package:flutter_tools/src/base/version.dart';
import 'package:flutter_tools/src/features.dart';
import 'package:flutter_tools/src/flutter_manifest.dart';
import 'package:flutter_tools/src/ios/xcodeproj.dart';
import 'package:flutter_tools/src/project.dart';
import 'package:test/fake.dart';

import '../src/common.dart';
import '../src/context.dart';
import '../src/fakes.dart';

void main() {
  group('IosProject', () {
    testWithoutContext('managedDirectory', () {
      final MemoryFileSystem fs = MemoryFileSystem.test();
      final IosProject project = IosProject.fromFlutter(FakeFlutterProject(fileSystem: fs));
      expect(project.managedDirectory.path, 'app_name/ios/Flutter');
    });

    testWithoutContext('module managedDirectory', () {
      final MemoryFileSystem fs = MemoryFileSystem.test();
      final IosProject project = IosProject.fromFlutter(
        FakeFlutterProject(fileSystem: fs, isModule: true),
      );
      expect(project.managedDirectory.path, 'app_name/.ios/Flutter');
    });

    testWithoutContext('ephemeralDirectory', () {
      final MemoryFileSystem fs = MemoryFileSystem.test();
      final IosProject project = IosProject.fromFlutter(FakeFlutterProject(fileSystem: fs));
      expect(project.ephemeralDirectory.path, 'app_name/ios/Flutter/ephemeral');
    });

    testWithoutContext('module ephemeralDirectory', () {
      final MemoryFileSystem fs = MemoryFileSystem.test();
      final IosProject project = IosProject.fromFlutter(
        FakeFlutterProject(fileSystem: fs, isModule: true),
      );
      expect(project.ephemeralDirectory.path, 'app_name/.ios/Flutter/ephemeral');
    });

    testWithoutContext('flutterPluginSwiftPackageDirectory', () {
      final MemoryFileSystem fs = MemoryFileSystem.test();
      final IosProject project = IosProject.fromFlutter(FakeFlutterProject(fileSystem: fs));
      expect(
        project.flutterPluginSwiftPackageDirectory.path,
        'app_name/ios/Flutter/ephemeral/Packages/FlutterGeneratedPluginSwiftPackage',
      );
    });

    testWithoutContext('module flutterPluginSwiftPackageDirectory', () {
      final MemoryFileSystem fs = MemoryFileSystem.test();
      final IosProject project = IosProject.fromFlutter(
        FakeFlutterProject(fileSystem: fs, isModule: true),
      );
      expect(
        project.flutterPluginSwiftPackageDirectory.path,
        'app_name/.ios/Flutter/ephemeral/Packages/FlutterGeneratedPluginSwiftPackage',
      );
    });

    testWithoutContext('xcodeConfigFor', () {
      final MemoryFileSystem fs = MemoryFileSystem.test();
      final IosProject project = IosProject.fromFlutter(FakeFlutterProject(fileSystem: fs));
      expect(project.xcodeConfigFor('Debug').path, 'app_name/ios/Flutter/Debug.xcconfig');
    });

    group('projectInfo', () {
      testUsingContext(
        'is null if XcodeProjectInterpreter is null',
        () async {
          final MemoryFileSystem fs = MemoryFileSystem.test();
          final IosProject project = IosProject.fromFlutter(FakeFlutterProject(fileSystem: fs));
          project.xcodeProject.createSync(recursive: true);
          expect(await project.projectInfo(), isNull);
        },
        overrides: <Type, Generator>{XcodeProjectInterpreter: () => null},
      );

      testUsingContext(
        'is null if XcodeProjectInterpreter is not installed',
        () async {
          final MemoryFileSystem fs = MemoryFileSystem.test();
          final IosProject project = IosProject.fromFlutter(FakeFlutterProject(fileSystem: fs));
          project.xcodeProject.createSync(recursive: true);
          expect(await project.projectInfo(), isNull);
        },
        overrides: <Type, Generator>{
          XcodeProjectInterpreter: () => FakeXcodeProjectInterpreter(isInstalled: false),
        },
      );

      testUsingContext(
        'is null if xcodeproj does not exist',
        () async {
          final MemoryFileSystem fs = MemoryFileSystem.test();
          final IosProject project = IosProject.fromFlutter(FakeFlutterProject(fileSystem: fs));
          expect(await project.projectInfo(), isNull);
        },
        overrides: <Type, Generator>{XcodeProjectInterpreter: () => FakeXcodeProjectInterpreter()},
      );

      testUsingContext(
        'returns XcodeProjectInfo',
        () async {
          final MemoryFileSystem fs = MemoryFileSystem.test();
          final IosProject project = IosProject.fromFlutter(FakeFlutterProject(fileSystem: fs));
          project.xcodeProject.createSync(recursive: true);
          expect(await project.projectInfo(), isNotNull);
        },
        overrides: <Type, Generator>{XcodeProjectInterpreter: () => FakeXcodeProjectInterpreter()},
      );
    });

    group('usesSwiftPackageManager', () {
      testUsingContext(
        'is true when iOS project exists',
        () async {
          final MemoryFileSystem fs = MemoryFileSystem.test();
          final Directory projectDirectory = fs.directory('path');
          projectDirectory.childDirectory('ios').createSync(recursive: true);
          final FlutterManifest manifest = FakeFlutterManifest();
          final FlutterProject project = FlutterProject(projectDirectory, manifest, manifest);
          expect(project.ios.usesSwiftPackageManager, isTrue);
        },
        overrides: <Type, Generator>{
          FeatureFlags: () => TestFeatureFlags(isSwiftPackageManagerEnabled: true),
          XcodeProjectInterpreter: () => FakeXcodeProjectInterpreter(version: Version(15, 0, 0)),
        },
      );

      testUsingContext(
        "is false when iOS project doesn't exist",
        () async {
          final MemoryFileSystem fs = MemoryFileSystem.test();
          final Directory projectDirectory = fs.directory('path');
          final FlutterManifest manifest = FakeFlutterManifest();
          final FlutterProject project = FlutterProject(projectDirectory, manifest, manifest);
          expect(project.ios.usesSwiftPackageManager, isFalse);
        },
        overrides: <Type, Generator>{
          FeatureFlags: () => TestFeatureFlags(isSwiftPackageManagerEnabled: true),
          XcodeProjectInterpreter: () => FakeXcodeProjectInterpreter(version: Version(15, 0, 0)),
        },
      );

      testUsingContext(
        'is false when disabled via manifest',
        () async {
          final MemoryFileSystem fs = MemoryFileSystem.test();
          final Directory projectDirectory = fs.directory('path');
          projectDirectory.childDirectory('ios').createSync(recursive: true);
          final FlutterManifest manifest = FakeFlutterManifest(disabledSwiftPackageManager: true);
          final FlutterProject project = FlutterProject(projectDirectory, manifest, manifest);
          expect(project.ios.usesSwiftPackageManager, isFalse);
        },
        overrides: <Type, Generator>{
          FeatureFlags: () => TestFeatureFlags(isSwiftPackageManagerEnabled: true),
          XcodeProjectInterpreter: () => FakeXcodeProjectInterpreter(version: Version(15, 0, 0)),
        },
      );

      testUsingContext(
        'is false when Xcode is less than 15',
        () async {
          final MemoryFileSystem fs = MemoryFileSystem.test();
          final Directory projectDirectory = fs.directory('path');
          projectDirectory.childDirectory('ios').createSync(recursive: true);
          final FlutterManifest manifest = FakeFlutterManifest();
          final FlutterProject project = FlutterProject(projectDirectory, manifest, manifest);
          expect(project.ios.usesSwiftPackageManager, isFalse);
        },
        overrides: <Type, Generator>{
          FeatureFlags: () => TestFeatureFlags(isSwiftPackageManagerEnabled: true),
          XcodeProjectInterpreter: () => FakeXcodeProjectInterpreter(version: Version(14, 0, 0)),
        },
      );

      testUsingContext(
        'is false when Swift Package Manager feature is not enabled',
        () async {
          final MemoryFileSystem fs = MemoryFileSystem.test();
          final Directory projectDirectory = fs.directory('path');
          projectDirectory.childDirectory('ios').createSync(recursive: true);
          final FlutterManifest manifest = FakeFlutterManifest();
          final FlutterProject project = FlutterProject(projectDirectory, manifest, manifest);
          expect(project.ios.usesSwiftPackageManager, isFalse);
        },
        overrides: <Type, Generator>{
          FeatureFlags: () => TestFeatureFlags(),
          XcodeProjectInterpreter: () => FakeXcodeProjectInterpreter(version: Version(15, 0, 0)),
        },
      );

      testUsingContext(
        'is false when project is a module',
        () async {
          final MemoryFileSystem fs = MemoryFileSystem.test();
          final Directory projectDirectory = fs.directory('path');
          projectDirectory.childDirectory('ios').createSync(recursive: true);
          final FlutterManifest manifest = FakeFlutterManifest(isModule: true);
          final FlutterProject project = FlutterProject(projectDirectory, manifest, manifest);
          expect(project.ios.usesSwiftPackageManager, isFalse);
        },
        overrides: <Type, Generator>{
          FeatureFlags: () => TestFeatureFlags(isSwiftPackageManagerEnabled: true),
          XcodeProjectInterpreter: () => FakeXcodeProjectInterpreter(version: Version(15, 0, 0)),
        },
      );
    });
  });

  group('MacOSProject', () {
    testWithoutContext('managedDirectory', () {
      final MemoryFileSystem fs = MemoryFileSystem.test();
      final MacOSProject project = MacOSProject.fromFlutter(FakeFlutterProject(fileSystem: fs));
      expect(project.managedDirectory.path, 'app_name/macos/Flutter');
    });

    testWithoutContext('module managedDirectory', () {
      final MemoryFileSystem fs = MemoryFileSystem.test();
      final MacOSProject project = MacOSProject.fromFlutter(FakeFlutterProject(fileSystem: fs));
      expect(project.managedDirectory.path, 'app_name/macos/Flutter');
    });

    testWithoutContext('ephemeralDirectory', () {
      final MemoryFileSystem fs = MemoryFileSystem.test();
      final MacOSProject project = MacOSProject.fromFlutter(FakeFlutterProject(fileSystem: fs));
      expect(project.ephemeralDirectory.path, 'app_name/macos/Flutter/ephemeral');
    });

    testWithoutContext('flutterPluginSwiftPackageDirectory', () {
      final MemoryFileSystem fs = MemoryFileSystem.test();
      final MacOSProject project = MacOSProject.fromFlutter(FakeFlutterProject(fileSystem: fs));
      expect(
        project.flutterPluginSwiftPackageDirectory.path,
        'app_name/macos/Flutter/ephemeral/Packages/FlutterGeneratedPluginSwiftPackage',
      );
    });

    testWithoutContext('xcodeConfigFor', () {
      final MemoryFileSystem fs = MemoryFileSystem.test();
      final MacOSProject project = MacOSProject.fromFlutter(FakeFlutterProject(fileSystem: fs));
      expect(project.xcodeConfigFor('Debug').path, 'app_name/macos/Flutter/Flutter-Debug.xcconfig');
    });

    group('usesSwiftPackageManager', () {
      testUsingContext(
        'is true when macOS project exists',
        () async {
          final MemoryFileSystem fs = MemoryFileSystem.test();
          final Directory projectDirectory = fs.directory('path');
          projectDirectory.childDirectory('macos').createSync(recursive: true);
          final FlutterManifest manifest = FakeFlutterManifest();
          final FlutterProject project = FlutterProject(projectDirectory, manifest, manifest);
          expect(project.macos.usesSwiftPackageManager, isTrue);
        },
        overrides: <Type, Generator>{
          FeatureFlags: () => TestFeatureFlags(isSwiftPackageManagerEnabled: true),
          XcodeProjectInterpreter: () => FakeXcodeProjectInterpreter(version: Version(15, 0, 0)),
        },
      );

      testUsingContext(
        "is false when macOS project doesn't exist",
        () async {
          final MemoryFileSystem fs = MemoryFileSystem.test();
          final Directory projectDirectory = fs.directory('path');
          final FlutterManifest manifest = FakeFlutterManifest();
          final FlutterProject project = FlutterProject(projectDirectory, manifest, manifest);
          expect(project.ios.usesSwiftPackageManager, isFalse);
          expect(project.macos.usesSwiftPackageManager, isFalse);
        },
        overrides: <Type, Generator>{
          FeatureFlags: () => TestFeatureFlags(isSwiftPackageManagerEnabled: true),
          XcodeProjectInterpreter: () => FakeXcodeProjectInterpreter(version: Version(15, 0, 0)),
        },
      );

      testUsingContext(
        'is false when disabled via manifest',
        () async {
          final MemoryFileSystem fs = MemoryFileSystem.test();
          final Directory projectDirectory = fs.directory('path');
          projectDirectory.childDirectory('macos').createSync(recursive: true);
          final FlutterManifest manifest = FakeFlutterManifest(disabledSwiftPackageManager: true);
          final FlutterProject project = FlutterProject(projectDirectory, manifest, manifest);
          expect(project.macos.usesSwiftPackageManager, isFalse);
        },
        overrides: <Type, Generator>{
          FeatureFlags: () => TestFeatureFlags(isSwiftPackageManagerEnabled: true),
          XcodeProjectInterpreter: () => FakeXcodeProjectInterpreter(version: Version(15, 0, 0)),
        },
      );

      testUsingContext(
        'is false when Xcode is less than 15',
        () async {
          final MemoryFileSystem fs = MemoryFileSystem.test();
          final Directory projectDirectory = fs.directory('path');
          projectDirectory.childDirectory('macos').createSync(recursive: true);
          final FlutterManifest manifest = FakeFlutterManifest();
          final FlutterProject project = FlutterProject(projectDirectory, manifest, manifest);
          expect(project.macos.usesSwiftPackageManager, isFalse);
        },
        overrides: <Type, Generator>{
          FeatureFlags: () => TestFeatureFlags(isSwiftPackageManagerEnabled: true),
          XcodeProjectInterpreter: () => FakeXcodeProjectInterpreter(version: Version(14, 0, 0)),
        },
      );

      testUsingContext(
        'is false when Swift Package Manager feature is not enabled',
        () async {
          final MemoryFileSystem fs = MemoryFileSystem.test();
          final Directory projectDirectory = fs.directory('path');
          projectDirectory.childDirectory('macos').createSync(recursive: true);
          final FlutterManifest manifest = FakeFlutterManifest();
          final FlutterProject project = FlutterProject(projectDirectory, manifest, manifest);
          expect(project.macos.usesSwiftPackageManager, isFalse);
        },
        overrides: <Type, Generator>{
          FeatureFlags: () => TestFeatureFlags(),
          XcodeProjectInterpreter: () => FakeXcodeProjectInterpreter(version: Version(15, 0, 0)),
        },
      );

      testUsingContext(
        'is false when project is a module',
        () async {
          final MemoryFileSystem fs = MemoryFileSystem.test();
          final Directory projectDirectory = fs.directory('path');
          projectDirectory.childDirectory('macos').createSync(recursive: true);
          final FlutterManifest manifest = FakeFlutterManifest(isModule: true);
          final FlutterProject project = FlutterProject(projectDirectory, manifest, manifest);
          expect(project.macos.usesSwiftPackageManager, isFalse);
        },
        overrides: <Type, Generator>{
          FeatureFlags: () => TestFeatureFlags(isSwiftPackageManagerEnabled: true),
          XcodeProjectInterpreter: () => FakeXcodeProjectInterpreter(version: Version(15, 0, 0)),
        },
      );
    });
  });
}

class FakeFlutterProject extends Fake implements FlutterProject {
  FakeFlutterProject({required this.fileSystem, this.isModule = false});

  MemoryFileSystem fileSystem;

  @override
  late final Directory directory = fileSystem.directory('app_name');

  @override
  bool isModule = false;
}

class FakeXcodeProjectInterpreter extends Fake implements XcodeProjectInterpreter {
  FakeXcodeProjectInterpreter({this.isInstalled = true, this.version});

  @override
  final bool isInstalled;

  @override
  final Version? version;

  @override
  Future<XcodeProjectInfo?> getInfo(String projectPath, {String? projectFilename}) async {
    return XcodeProjectInfo(<String>[], <String>[], <String>['Runner'], BufferLogger.test());
  }
}

class FakeFlutterManifest extends Fake implements FlutterManifest {
  FakeFlutterManifest({this.disabledSwiftPackageManager = false, this.isModule = false});

  @override
  bool disabledSwiftPackageManager;

  @override
  bool isModule;
}
