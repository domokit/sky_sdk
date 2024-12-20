// Copyright 2014 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

import 'dart:convert';

import 'package:file/file.dart';
import 'package:file/memory.dart';
import 'package:flutter_tools/src/base/logger.dart';
import 'package:flutter_tools/src/build_info.dart';
import 'package:flutter_tools/src/dart/pub.dart';
import 'package:flutter_tools/src/flutter_manifest.dart';
import 'package:flutter_tools/src/macos/cocoapod_utils.dart';
import 'package:flutter_tools/src/macos/cocoapods.dart';
import 'package:flutter_tools/src/project.dart';
import 'package:test/fake.dart';

import '../../src/common.dart';
import '../../src/context.dart';
import '../../src/fake_pub_deps.dart';

void main() {
  group('processPodsIfNeeded', () {
    late MemoryFileSystem fs;
    late FakeCocoaPods cocoaPods;
    late BufferLogger logger;

    // Adds basic properties to the flutterProject and its subprojects.
    void setUpProject(FakeFlutterProject flutterProject, MemoryFileSystem fileSystem) {
      flutterProject
        ..manifest = FakeFlutterManifest()
        ..directory = fileSystem.systemTempDirectory.childDirectory('app')
        ..flutterPluginsFile = flutterProject.directory.childFile('.flutter-plugins')
        ..flutterPluginsDependenciesFile = flutterProject.directory.childFile(
          '.flutter-plugins-dependencies',
        )
        ..ios = FakeIosProject(fileSystem: fileSystem, parent: flutterProject)
        ..macos = FakeMacOSProject(fileSystem: fileSystem, parent: flutterProject)
        ..android = FakeAndroidProject()
        ..web = FakeWebProject()
        ..windows = FakeWindowsProject()
        ..linux = FakeLinuxProject();
      flutterProject.directory.childDirectory('.dart_tool').childFile('package_config.json')
        ..createSync(recursive: true)
        ..writeAsStringSync('''
{
  "packages": [],
  "configVersion": 2
}
''');
    }

    setUp(() async {
      fs = MemoryFileSystem.test();
      cocoaPods = FakeCocoaPods();
      logger = BufferLogger.test();
    });

    void createFakePlugins(
      FlutterProject flutterProject,
      FileSystem fileSystem,
      List<String> pluginNames,
    ) {
      const String pluginYamlTemplate = '''
      flutter:
        plugin:
          platforms:
            ios:
              pluginClass: PLUGIN_CLASS
            macos:
              pluginClass: PLUGIN_CLASS
      ''';

      final Directory fakePubCache = fileSystem.systemTempDirectory.childDirectory('cache');
      final File packageConfigFile = flutterProject.directory
        .childDirectory('.dart_tool')
        .childFile('package_config.json')..createSync(recursive: true);
      final Map<String, Object?> packageConfig = <String, Object?>{
        'packages': <Object?>[],
        'configVersion': 2,
      };
      for (final String name in pluginNames) {
        final Directory pluginDirectory = fakePubCache.childDirectory(name);
        (packageConfig['packages']! as List<Object?>).add(<String, Object?>{
          'name': name,
          'rootUri': pluginDirectory.uri.toString(),
          'packageUri': 'lib/',
        });
        pluginDirectory.childFile('pubspec.yaml')
          ..createSync(recursive: true)
          ..writeAsStringSync(pluginYamlTemplate.replaceAll('PLUGIN_CLASS', name));
      }

      packageConfigFile.writeAsStringSync(jsonEncode(packageConfig));
    }

    group('for iOS', () {
      group('using CocoaPods only', () {
        testUsingContext(
          'processes when there are plugins',
          () async {
            final FakeFlutterProject flutterProject = FakeFlutterProject();
            setUpProject(flutterProject, fs);
            createFakePlugins(flutterProject, fs, <String>['plugin_one', 'plugin_two']);

            await processPodsIfNeeded(
              flutterProject.ios,
              fs.currentDirectory.childDirectory('build').path,
              BuildMode.debug,
            );
            expect(cocoaPods.processedPods, isTrue);
          },
          overrides: <Type, Generator>{
            FileSystem: () => fs,
            ProcessManager: FakeProcessManager.empty,
            Pub: FakePubWithPrimedDeps.new,
            CocoaPods: () => cocoaPods,
          },
        );

        testUsingContext(
          'processes when no plugins but the project is a module and podfile exists',
          () async {
            final FakeFlutterProject flutterProject = FakeFlutterProject();
            setUpProject(flutterProject, fs);
            flutterProject.isModule = true;
            flutterProject.ios.podfile.createSync(recursive: true);

            await processPodsIfNeeded(
              flutterProject.ios,
              fs.currentDirectory.childDirectory('build').path,
              BuildMode.debug,
            );
            expect(cocoaPods.processedPods, isTrue);
          },
          overrides: <Type, Generator>{
            FileSystem: () => fs,
            ProcessManager: FakeProcessManager.empty,
            Pub: FakePubWithPrimedDeps.new,
            CocoaPods: () => cocoaPods,
          },
        );

        testUsingContext(
          "skips when no plugins and the project is a module but podfile doesn't exist",
          () async {
            final FakeFlutterProject flutterProject = FakeFlutterProject();
            setUpProject(flutterProject, fs);
            flutterProject.isModule = true;

            await processPodsIfNeeded(
              flutterProject.ios,
              fs.currentDirectory.childDirectory('build').path,
              BuildMode.debug,
            );
            expect(cocoaPods.processedPods, isFalse);
          },
          overrides: <Type, Generator>{
            FileSystem: () => fs,
            ProcessManager: FakeProcessManager.empty,
            Pub: FakePubWithPrimedDeps.new,
            CocoaPods: () => cocoaPods,
          },
        );

        testUsingContext(
          'skips when no plugins and project is not a module',
          () async {
            final FakeFlutterProject flutterProject = FakeFlutterProject();
            setUpProject(flutterProject, fs);

            await processPodsIfNeeded(
              flutterProject.ios,
              fs.currentDirectory.childDirectory('build').path,
              BuildMode.debug,
            );
            expect(cocoaPods.processedPods, isFalse);
          },
          overrides: <Type, Generator>{
            FileSystem: () => fs,
            ProcessManager: FakeProcessManager.empty,
            Pub: FakePubWithPrimedDeps.new,
            CocoaPods: () => cocoaPods,
          },
        );
      });

      group('using Swift Package Manager', () {
        testUsingContext(
          'processes if podfile exists',
          () async {
            final FakeFlutterProject flutterProject = FakeFlutterProject();
            setUpProject(flutterProject, fs);
            createFakePlugins(flutterProject, fs, <String>['plugin_one', 'plugin_two']);
            flutterProject.ios.usesSwiftPackageManager = true;
            flutterProject.ios.podfile.createSync(recursive: true);

            await processPodsIfNeeded(
              flutterProject.ios,
              fs.currentDirectory.childDirectory('build').path,
              BuildMode.debug,
            );
            expect(cocoaPods.processedPods, isTrue);
          },
          overrides: <Type, Generator>{
            FileSystem: () => fs,
            ProcessManager: FakeProcessManager.empty,
            Pub: FakePubWithPrimedDeps.new,
            CocoaPods: () => cocoaPods,
          },
        );

        testUsingContext(
          'skip if podfile does not exists',
          () async {
            final FakeFlutterProject flutterProject = FakeFlutterProject();
            setUpProject(flutterProject, fs);
            createFakePlugins(flutterProject, fs, <String>['plugin_one', 'plugin_two']);
            flutterProject.ios.usesSwiftPackageManager = true;

            await processPodsIfNeeded(
              flutterProject.ios,
              fs.currentDirectory.childDirectory('build').path,
              BuildMode.debug,
            );
            expect(cocoaPods.processedPods, isFalse);
          },
          overrides: <Type, Generator>{
            FileSystem: () => fs,
            ProcessManager: FakeProcessManager.empty,
            CocoaPods: () => cocoaPods,
          },
        );

        testUsingContext(
          'process if podfile does not exists but forceCocoaPodsOnly is true',
          () async {
            final FakeFlutterProject flutterProject = FakeFlutterProject();
            setUpProject(flutterProject, fs);
            createFakePlugins(flutterProject, fs, <String>['plugin_one', 'plugin_two']);
            flutterProject.ios.usesSwiftPackageManager = true;
            flutterProject.ios.flutterPluginSwiftPackageManifest.createSync(recursive: true);

            await processPodsIfNeeded(
              flutterProject.ios,
              fs.currentDirectory.childDirectory('build').path,
              BuildMode.debug,
              forceCocoaPodsOnly: true,
            );
            expect(cocoaPods.processedPods, isTrue);
            expect(cocoaPods.podfileSetup, isTrue);
            expect(
              logger.warningText,
              'Swift Package Manager does not yet support this command. '
              'CocoaPods will be used instead.\n',
            );
            expect(flutterProject.ios.flutterPluginSwiftPackageManifest.existsSync(), isFalse);
          },
          overrides: <Type, Generator>{
            FileSystem: () => fs,
            ProcessManager: FakeProcessManager.empty,
            Pub: FakePubWithPrimedDeps.new,
            CocoaPods: () => cocoaPods,
            Logger: () => logger,
          },
        );
      });
    });

    group('for macOS', () {
      group('using CocoaPods only', () {
        testUsingContext(
          'processes when there are plugins',
          () async {
            final FakeFlutterProject flutterProject = FakeFlutterProject();
            setUpProject(flutterProject, fs);
            createFakePlugins(flutterProject, fs, <String>['plugin_one', 'plugin_two']);

            await processPodsIfNeeded(
              flutterProject.macos,
              fs.currentDirectory.childDirectory('build').path,
              BuildMode.debug,
            );
            expect(cocoaPods.processedPods, isTrue);
          },
          overrides: <Type, Generator>{
            FileSystem: () => fs,
            ProcessManager: FakeProcessManager.empty,
            Pub: FakePubWithPrimedDeps.new,
            CocoaPods: () => cocoaPods,
          },
        );

        testUsingContext(
          'processes when no plugins but the project is a module and podfile exists',
          () async {
            final FakeFlutterProject flutterProject = FakeFlutterProject();
            setUpProject(flutterProject, fs);
            flutterProject.isModule = true;
            flutterProject.macos.podfile.createSync(recursive: true);

            await processPodsIfNeeded(
              flutterProject.macos,
              fs.currentDirectory.childDirectory('build').path,
              BuildMode.debug,
            );
            expect(cocoaPods.processedPods, isTrue);
          },
          overrides: <Type, Generator>{
            FileSystem: () => fs,
            ProcessManager: FakeProcessManager.empty,
            Pub: FakePubWithPrimedDeps.new,
            CocoaPods: () => cocoaPods,
          },
        );

        testUsingContext(
          "skips when no plugins and the project is a module but podfile doesn't exist",
          () async {
            final FakeFlutterProject flutterProject = FakeFlutterProject();
            setUpProject(flutterProject, fs);
            flutterProject.isModule = true;

            await processPodsIfNeeded(
              flutterProject.macos,
              fs.currentDirectory.childDirectory('build').path,
              BuildMode.debug,
            );
            expect(cocoaPods.processedPods, isFalse);
          },
          overrides: <Type, Generator>{
            FileSystem: () => fs,
            ProcessManager: FakeProcessManager.empty,
            Pub: FakePubWithPrimedDeps.new,
            CocoaPods: () => cocoaPods,
          },
        );

        testUsingContext(
          'skips when no plugins and project is not a module',
          () async {
            final FakeFlutterProject flutterProject = FakeFlutterProject();
            setUpProject(flutterProject, fs);

            await processPodsIfNeeded(
              flutterProject.macos,
              fs.currentDirectory.childDirectory('build').path,
              BuildMode.debug,
            );
            expect(cocoaPods.processedPods, isFalse);
          },
          overrides: <Type, Generator>{
            FileSystem: () => fs,
            ProcessManager: FakeProcessManager.empty,
            Pub: FakePubWithPrimedDeps.new,
            CocoaPods: () => cocoaPods,
          },
        );
      });

      group('using Swift Package Manager', () {
        testUsingContext(
          'processes if podfile exists',
          () async {
            final FakeFlutterProject flutterProject = FakeFlutterProject();
            setUpProject(flutterProject, fs);
            createFakePlugins(flutterProject, fs, <String>['plugin_one', 'plugin_two']);
            flutterProject.macos.usesSwiftPackageManager = true;
            flutterProject.macos.podfile.createSync(recursive: true);

            await processPodsIfNeeded(
              flutterProject.macos,
              fs.currentDirectory.childDirectory('build').path,
              BuildMode.debug,
            );
            expect(cocoaPods.processedPods, isTrue);
          },
          overrides: <Type, Generator>{
            FileSystem: () => fs,
            ProcessManager: FakeProcessManager.empty,
            Pub: FakePubWithPrimedDeps.new,
            CocoaPods: () => cocoaPods,
          },
        );

        testUsingContext(
          'skip if podfile does not exists',
          () async {
            final FakeFlutterProject flutterProject = FakeFlutterProject();
            setUpProject(flutterProject, fs);
            createFakePlugins(flutterProject, fs, <String>['plugin_one', 'plugin_two']);
            flutterProject.macos.usesSwiftPackageManager = true;

            await processPodsIfNeeded(
              flutterProject.macos,
              fs.currentDirectory.childDirectory('build').path,
              BuildMode.debug,
            );
            expect(cocoaPods.processedPods, isFalse);
          },
          overrides: <Type, Generator>{
            FileSystem: () => fs,
            ProcessManager: FakeProcessManager.empty,
            CocoaPods: () => cocoaPods,
          },
        );

        testUsingContext(
          'process if podfile does not exists but forceCocoaPodsOnly is true',
          () async {
            final FakeFlutterProject flutterProject = FakeFlutterProject();
            setUpProject(flutterProject, fs);
            createFakePlugins(flutterProject, fs, <String>['plugin_one', 'plugin_two']);
            flutterProject.macos.usesSwiftPackageManager = true;
            flutterProject.macos.flutterPluginSwiftPackageManifest.createSync(recursive: true);

            await processPodsIfNeeded(
              flutterProject.macos,
              fs.currentDirectory.childDirectory('build').path,
              BuildMode.debug,
              forceCocoaPodsOnly: true,
            );
            expect(cocoaPods.processedPods, isTrue);
            expect(cocoaPods.podfileSetup, isTrue);
            expect(
              logger.warningText,
              'Swift Package Manager does not yet support this command. '
              'CocoaPods will be used instead.\n',
            );
            expect(flutterProject.macos.flutterPluginSwiftPackageManifest.existsSync(), isFalse);
          },
          overrides: <Type, Generator>{
            FileSystem: () => fs,
            ProcessManager: FakeProcessManager.empty,
            Pub: FakePubWithPrimedDeps.new,
            CocoaPods: () => cocoaPods,
            Logger: () => logger,
          },
        );
      });
    });
  });
}

class FakeFlutterManifest extends Fake implements FlutterManifest {
  @override
  Set<String> get dependencies => <String>{};
}

class FakeFlutterProject extends Fake implements FlutterProject {
  @override
  bool isModule = false;

  @override
  late FlutterManifest manifest;

  @override
  late Directory directory;

  @override
  late File flutterPluginsFile;

  @override
  late File flutterPluginsDependenciesFile;

  @override
  late FakeIosProject ios;

  @override
  late FakeMacOSProject macos;

  @override
  late AndroidProject android;

  @override
  late WebProject web;

  @override
  late LinuxProject linux;

  @override
  late WindowsProject windows;
}

class FakeMacOSProject extends Fake implements MacOSProject {
  FakeMacOSProject({required MemoryFileSystem fileSystem, required this.parent})
    : hostAppRoot = fileSystem.directory('app_name').childDirectory('ios');

  @override
  String pluginConfigKey = 'macos';

  @override
  final FlutterProject parent;

  @override
  Directory hostAppRoot;

  bool exists = true;

  @override
  bool existsSync() => exists;

  @override
  File get podfile => hostAppRoot.childFile('Podfile');

  @override
  File get xcodeProjectInfoFile =>
      hostAppRoot.childDirectory('Runner.xcodeproj').childFile('project.pbxproj');

  @override
  File get flutterPluginSwiftPackageManifest => hostAppRoot
      .childDirectory('Flutter')
      .childDirectory('ephemeral')
      .childDirectory('Packages')
      .childDirectory('FlutterGeneratedPluginSwiftPackage')
      .childFile('Package.swift');

  @override
  bool usesSwiftPackageManager = false;
}

class FakeIosProject extends Fake implements IosProject {
  FakeIosProject({required MemoryFileSystem fileSystem, required this.parent})
    : hostAppRoot = fileSystem.directory('app_name').childDirectory('ios');

  @override
  String pluginConfigKey = 'ios';

  @override
  final FlutterProject parent;

  @override
  Directory hostAppRoot;

  @override
  bool exists = true;

  @override
  bool existsSync() => exists;

  @override
  File get podfile => hostAppRoot.childFile('Podfile');

  @override
  File get xcodeProjectInfoFile =>
      hostAppRoot.childDirectory('Runner.xcodeproj').childFile('project.pbxproj');

  @override
  File get flutterPluginSwiftPackageManifest => hostAppRoot
      .childDirectory('Flutter')
      .childDirectory('ephemeral')
      .childDirectory('Packages')
      .childDirectory('FlutterGeneratedPluginSwiftPackage')
      .childFile('Package.swift');

  @override
  bool usesSwiftPackageManager = false;
}

class FakeAndroidProject extends Fake implements AndroidProject {
  @override
  String pluginConfigKey = 'android';

  @override
  bool existsSync() => false;
}

class FakeWebProject extends Fake implements WebProject {
  @override
  String pluginConfigKey = 'web';

  @override
  bool existsSync() => false;
}

class FakeWindowsProject extends Fake implements WindowsProject {
  @override
  String pluginConfigKey = 'windows';

  @override
  bool existsSync() => false;
}

class FakeLinuxProject extends Fake implements LinuxProject {
  @override
  String pluginConfigKey = 'linux';

  @override
  bool existsSync() => false;
}

class FakeCocoaPods extends Fake implements CocoaPods {
  bool podfileSetup = false;
  bool processedPods = false;

  @override
  Future<bool> processPods({
    required XcodeBasedProject xcodeProject,
    required BuildMode buildMode,
    bool dependenciesChanged = true,
  }) async {
    processedPods = true;
    return true;
  }

  @override
  Future<void> setupPodfile(XcodeBasedProject xcodeProject) async {
    podfileSetup = true;
  }

  @override
  void invalidatePodInstallOutput(XcodeBasedProject xcodeProject) {}
}
