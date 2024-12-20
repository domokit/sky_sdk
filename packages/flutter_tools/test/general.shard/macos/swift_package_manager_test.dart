// Copyright 2014 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

import 'package:file/file.dart';
import 'package:file/memory.dart';
import 'package:flutter_tools/src/isolated/mustache_template.dart';
import 'package:flutter_tools/src/macos/swift_package_manager.dart';
import 'package:flutter_tools/src/platform_plugins.dart';
import 'package:flutter_tools/src/plugins.dart';
import 'package:flutter_tools/src/project.dart';
import 'package:test/fake.dart';

import '../../src/common.dart';

const String _doubleIndent = '        ';

void main() {
  const List<SupportedPlatform> supportedPlatforms = <SupportedPlatform>[
    SupportedPlatform.ios,
    SupportedPlatform.macos,
  ];

  group('SwiftPackageManager', () {
    for (final SupportedPlatform platform in supportedPlatforms) {
      group('for ${platform.name}', () {
        group('generatePluginsSwiftPackage', () {
          testWithoutContext('throw if invalid platform', () async {
            final MemoryFileSystem fs = MemoryFileSystem();
            final FakeXcodeProject project = FakeXcodeProject(
              platform: platform.name,
              fileSystem: fs,
            );

            final SwiftPackageManager spm = SwiftPackageManager(
              fileSystem: fs,
              templateRenderer: const MustacheTemplateRenderer(),
            );

            await expectLater(
              () => spm.generatePluginsSwiftPackage(<Plugin>[], SupportedPlatform.android, project),
              throwsToolExit(
                message:
                    'The platform android is not compatible with Swift Package Manager. Only iOS and macOS are allowed.',
              ),
            );
          });

          testWithoutContext('skip if no dependencies and not already migrated', () async {
            final MemoryFileSystem fs = MemoryFileSystem();
            final FakeXcodeProject project = FakeXcodeProject(
              platform: platform.name,
              fileSystem: fs,
            );

            final SwiftPackageManager spm = SwiftPackageManager(
              fileSystem: fs,
              templateRenderer: const MustacheTemplateRenderer(),
            );
            await spm.generatePluginsSwiftPackage(<Plugin>[], platform, project);

            expect(project.flutterPluginSwiftPackageManifest.existsSync(), isFalse);
          });

          testWithoutContext('generate if no dependencies and already migrated', () async {
            final MemoryFileSystem fs = MemoryFileSystem();
            final FakeXcodeProject project = FakeXcodeProject(
              platform: platform.name,
              fileSystem: fs,
            );
            project.xcodeProjectInfoFile.createSync(recursive: true);
            project.xcodeProjectInfoFile.writeAsStringSync('''
'		78A318202AECB46A00862997 /* FlutterGeneratedPluginSwiftPackage in Frameworks */ = {isa = PBXBuildFile; productRef = 78A3181F2AECB46A00862997 /* FlutterGeneratedPluginSwiftPackage */; };';
''');

            final SwiftPackageManager spm = SwiftPackageManager(
              fileSystem: fs,
              templateRenderer: const MustacheTemplateRenderer(),
            );
            await spm.generatePluginsSwiftPackage(<Plugin>[], platform, project);

            final String supportedPlatform =
                platform == SupportedPlatform.ios ? '.iOS("12.0")' : '.macOS("10.14")';
            expect(project.flutterPluginSwiftPackageManifest.existsSync(), isTrue);
            expect(project.flutterPluginSwiftPackageManifest.readAsStringSync(), '''
// swift-tools-version: 5.9
// The swift-tools-version declares the minimum version of Swift required to build this package.
//
//  Generated file. Do not edit.
//

import PackageDescription

let package = Package(
    name: "FlutterGeneratedPluginSwiftPackage",
    platforms: [
        $supportedPlatform
    ],
    products: [
        .library(name: "FlutterGeneratedPluginSwiftPackage", type: .static, targets: ["FlutterGeneratedPluginSwiftPackage"])
    ],
    dependencies: [
$_doubleIndent
    ],
    targets: [
        .target(
            name: "FlutterGeneratedPluginSwiftPackage"
        )
    ]
)
''');
          });

          testWithoutContext('generate with single dependency', () async {
            final MemoryFileSystem fs = MemoryFileSystem();
            final FakeXcodeProject project = FakeXcodeProject(
              platform: platform.name,
              fileSystem: fs,
            );

            final File validPlugin1Manifest = fs.file(
              '/local/path/to/plugins/valid_plugin_1/Package.swift',
            )..createSync(recursive: true);
            final FakePlugin validPlugin1 = FakePlugin(
              name: 'valid_plugin_1',
              platforms: <String, PluginPlatform>{platform.name: FakePluginPlatform()},
              pluginSwiftPackageManifestPath: validPlugin1Manifest.path,
            );
            final SwiftPackageManager spm = SwiftPackageManager(
              fileSystem: fs,
              templateRenderer: const MustacheTemplateRenderer(),
            );
            await spm.generatePluginsSwiftPackage(<Plugin>[validPlugin1], platform, project);

            final String supportedPlatform =
                platform == SupportedPlatform.ios ? '.iOS("12.0")' : '.macOS("10.14")';
            expect(project.flutterPluginSwiftPackageManifest.existsSync(), isTrue);
            expect(project.flutterPluginSwiftPackageManifest.readAsStringSync(), '''
// swift-tools-version: 5.9
// The swift-tools-version declares the minimum version of Swift required to build this package.
//
//  Generated file. Do not edit.
//

import PackageDescription

let package = Package(
    name: "FlutterGeneratedPluginSwiftPackage",
    platforms: [
        $supportedPlatform
    ],
    products: [
        .library(name: "FlutterGeneratedPluginSwiftPackage", type: .static, targets: ["FlutterGeneratedPluginSwiftPackage"])
    ],
    dependencies: [
        .package(name: "valid_plugin_1", path: "/local/path/to/plugins/valid_plugin_1")
    ],
    targets: [
        .target(
            name: "FlutterGeneratedPluginSwiftPackage",
            dependencies: [
                .product(name: "valid-plugin-1", package: "valid_plugin_1")
            ]
        )
    ]
)
''');
          });

          testWithoutContext('generate with multiple dependencies', () async {
            final MemoryFileSystem fs = MemoryFileSystem();
            final FakeXcodeProject project = FakeXcodeProject(
              platform: platform.name,
              fileSystem: fs,
            );
            final FakePlugin nonPlatformCompatiblePlugin = FakePlugin(
              name: 'invalid_plugin_due_to_incompatible_platform',
              platforms: <String, PluginPlatform>{},
              pluginSwiftPackageManifestPath: '/some/path',
            );
            final FakePlugin pluginSwiftPackageManifestIsNull = FakePlugin(
              name: 'invalid_plugin_due_to_null_plugin_swift_package_path',
              platforms: <String, PluginPlatform>{platform.name: FakePluginPlatform()},
              pluginSwiftPackageManifestPath: null,
            );
            final FakePlugin pluginSwiftPackageManifestNotExists = FakePlugin(
              name: 'invalid_plugin_due_to_plugin_swift_package_path_does_not_exist',
              platforms: <String, PluginPlatform>{platform.name: FakePluginPlatform()},
              pluginSwiftPackageManifestPath: '/some/path',
            );

            final File validPlugin1Manifest = fs.file(
              '/local/path/to/plugins/valid_plugin_1/Package.swift',
            )..createSync(recursive: true);
            final FakePlugin validPlugin1 = FakePlugin(
              name: 'valid_plugin_1',
              platforms: <String, PluginPlatform>{platform.name: FakePluginPlatform()},
              pluginSwiftPackageManifestPath: validPlugin1Manifest.path,
            );
            final File validPlugin2Manifest = fs.file(
              '/.pub-cache/plugins/valid_plugin_2/Package.swift',
            )..createSync(recursive: true);
            final FakePlugin validPlugin2 = FakePlugin(
              name: 'valid_plugin_2',
              platforms: <String, PluginPlatform>{platform.name: FakePluginPlatform()},
              pluginSwiftPackageManifestPath: validPlugin2Manifest.path,
            );

            final SwiftPackageManager spm = SwiftPackageManager(
              fileSystem: fs,
              templateRenderer: const MustacheTemplateRenderer(),
            );
            await spm.generatePluginsSwiftPackage(
              <Plugin>[
                nonPlatformCompatiblePlugin,
                pluginSwiftPackageManifestIsNull,
                pluginSwiftPackageManifestNotExists,
                validPlugin1,
                validPlugin2,
              ],
              platform,
              project,
            );

            final String supportedPlatform =
                platform == SupportedPlatform.ios ? '.iOS("12.0")' : '.macOS("10.14")';
            expect(project.flutterPluginSwiftPackageManifest.existsSync(), isTrue);
            expect(project.flutterPluginSwiftPackageManifest.readAsStringSync(), '''
// swift-tools-version: 5.9
// The swift-tools-version declares the minimum version of Swift required to build this package.
//
//  Generated file. Do not edit.
//

import PackageDescription

let package = Package(
    name: "FlutterGeneratedPluginSwiftPackage",
    platforms: [
        $supportedPlatform
    ],
    products: [
        .library(name: "FlutterGeneratedPluginSwiftPackage", type: .static, targets: ["FlutterGeneratedPluginSwiftPackage"])
    ],
    dependencies: [
        .package(name: "valid_plugin_1", path: "/local/path/to/plugins/valid_plugin_1"),
        .package(name: "valid_plugin_2", path: "/.pub-cache/plugins/valid_plugin_2")
    ],
    targets: [
        .target(
            name: "FlutterGeneratedPluginSwiftPackage",
            dependencies: [
                .product(name: "valid-plugin-1", package: "valid_plugin_1"),
                .product(name: "valid-plugin-2", package: "valid_plugin_2")
            ]
        )
    ]
)
''');
          });
        });

        group('updateMinimumDeployment', () {
          testWithoutContext('return if invalid deploymentTarget', () {
            final MemoryFileSystem fs = MemoryFileSystem();
            final FakeXcodeProject project = FakeXcodeProject(
              platform: platform.name,
              fileSystem: fs,
            );
            final String supportedPlatform =
                platform == SupportedPlatform.ios ? '.iOS("12.0")' : '.macOS("10.14")';
            project.flutterPluginSwiftPackageManifest.createSync(recursive: true);
            project.flutterPluginSwiftPackageManifest.writeAsStringSync(supportedPlatform);
            SwiftPackageManager.updateMinimumDeployment(
              project: project,
              platform: platform,
              deploymentTarget: '',
            );
            expect(
              project.flutterPluginSwiftPackageManifest.readAsLinesSync(),
              contains(supportedPlatform),
            );
          });

          testWithoutContext('return if deploymentTarget is lower than default', () {
            final MemoryFileSystem fs = MemoryFileSystem();
            final FakeXcodeProject project = FakeXcodeProject(
              platform: platform.name,
              fileSystem: fs,
            );
            final String supportedPlatform =
                platform == SupportedPlatform.ios ? '.iOS("12.0")' : '.macOS("10.14")';
            project.flutterPluginSwiftPackageManifest.createSync(recursive: true);
            project.flutterPluginSwiftPackageManifest.writeAsStringSync(supportedPlatform);
            SwiftPackageManager.updateMinimumDeployment(
              project: project,
              platform: platform,
              deploymentTarget: '9.0',
            );
            expect(
              project.flutterPluginSwiftPackageManifest.readAsLinesSync(),
              contains(supportedPlatform),
            );
          });

          testWithoutContext('return if deploymentTarget is same than default', () {
            final MemoryFileSystem fs = MemoryFileSystem();
            final FakeXcodeProject project = FakeXcodeProject(
              platform: platform.name,
              fileSystem: fs,
            );
            final String supportedPlatform =
                platform == SupportedPlatform.ios ? '.iOS("12.0")' : '.macOS("10.14")';
            project.flutterPluginSwiftPackageManifest.createSync(recursive: true);
            project.flutterPluginSwiftPackageManifest.writeAsStringSync(supportedPlatform);
            SwiftPackageManager.updateMinimumDeployment(
              project: project,
              platform: platform,
              deploymentTarget: platform == SupportedPlatform.ios ? '12.0' : '10.14',
            );
            expect(
              project.flutterPluginSwiftPackageManifest.readAsLinesSync(),
              contains(supportedPlatform),
            );
          });

          testWithoutContext('update if deploymentTarget is higher than default', () {
            final MemoryFileSystem fs = MemoryFileSystem();
            final FakeXcodeProject project = FakeXcodeProject(
              platform: platform.name,
              fileSystem: fs,
            );
            final String supportedPlatform =
                platform == SupportedPlatform.ios ? '.iOS("12.0")' : '.macOS("10.14")';
            project.flutterPluginSwiftPackageManifest.createSync(recursive: true);
            project.flutterPluginSwiftPackageManifest.writeAsStringSync(supportedPlatform);
            SwiftPackageManager.updateMinimumDeployment(
              project: project,
              platform: platform,
              deploymentTarget: '14.0',
            );
            expect(
              project.flutterPluginSwiftPackageManifest.readAsLinesSync().contains(
                supportedPlatform,
              ),
              isFalse,
            );
            expect(
              project.flutterPluginSwiftPackageManifest.readAsLinesSync(),
              contains(platform == SupportedPlatform.ios ? '.iOS("14.0")' : '.macOS("14.0")'),
            );
          });
        });
      });
    }
  });
}

class FakeXcodeProject extends Fake implements IosProject {
  FakeXcodeProject({required MemoryFileSystem fileSystem, required String platform})
    : hostAppRoot = fileSystem.directory('app_name').childDirectory(platform);

  @override
  Directory hostAppRoot;

  @override
  Directory get xcodeProject => hostAppRoot.childDirectory('$hostAppProjectName.xcodeproj');

  @override
  File get xcodeProjectInfoFile => xcodeProject.childFile('project.pbxproj');

  @override
  String hostAppProjectName = 'Runner';

  @override
  Directory get flutterPluginSwiftPackageDirectory => hostAppRoot
      .childDirectory('Flutter')
      .childDirectory('ephemeral')
      .childDirectory('Packages')
      .childDirectory('FlutterGeneratedPluginSwiftPackage');

  @override
  File get flutterPluginSwiftPackageManifest =>
      flutterPluginSwiftPackageDirectory.childFile('Package.swift');

  @override
  bool get flutterPluginSwiftPackageInProjectSettings {
    return xcodeProjectInfoFile.existsSync() &&
        xcodeProjectInfoFile.readAsStringSync().contains('FlutterGeneratedPluginSwiftPackage');
  }
}

class FakePlugin extends Fake implements Plugin {
  FakePlugin({
    required this.name,
    required this.platforms,
    required String? pluginSwiftPackageManifestPath,
  }) : _pluginSwiftPackageManifestPath = pluginSwiftPackageManifestPath;

  final String? _pluginSwiftPackageManifestPath;

  @override
  final String name;

  @override
  final Map<String, PluginPlatform> platforms;

  @override
  String? pluginSwiftPackageManifestPath(FileSystem fileSystem, String platform) {
    return _pluginSwiftPackageManifestPath;
  }
}

class FakePluginPlatform extends Fake implements PluginPlatform {}
