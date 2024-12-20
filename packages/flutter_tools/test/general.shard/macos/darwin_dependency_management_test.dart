// Copyright 2014 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

import 'package:file/file.dart';
import 'package:file/memory.dart';
import 'package:flutter_tools/src/base/logger.dart';
import 'package:flutter_tools/src/macos/cocoapods.dart';
import 'package:flutter_tools/src/macos/darwin_dependency_management.dart';
import 'package:flutter_tools/src/macos/swift_package_manager.dart';
import 'package:flutter_tools/src/platform_plugins.dart';
import 'package:flutter_tools/src/plugins.dart';
import 'package:flutter_tools/src/project.dart';
import 'package:test/fake.dart';

import '../../src/common.dart';

void main() {
  const List<SupportedPlatform> supportedPlatforms = <SupportedPlatform>[
    SupportedPlatform.ios,
    SupportedPlatform.macos,
  ];

  group('DarwinDependencyManagement', () {
    for (final SupportedPlatform platform in supportedPlatforms) {
      group('for ${platform.name}', () {
        group('generatePluginsSwiftPackage', () {
          testWithoutContext('throw if invalid platform', () async {
            final MemoryFileSystem fs = MemoryFileSystem();
            final BufferLogger testLogger = BufferLogger.test();

            final DarwinDependencyManagement dependencyManagement = DarwinDependencyManagement(
              project: FakeFlutterProject(fileSystem: fs),
              plugins: <Plugin>[],
              cocoapods: FakeCocoaPods(),
              swiftPackageManager: FakeSwiftPackageManager(),
              fileSystem: fs,
              logger: testLogger,
            );

            await expectLater(
              () => dependencyManagement.setUp(platform: SupportedPlatform.android),
              throwsToolExit(
                message:
                    'The platform android is incompatible with Darwin Dependency Managers. Only iOS and macOS are allowed.',
              ),
            );
          });
          group('when using Swift Package Manager', () {
            testWithoutContext('with only CocoaPod plugins', () async {
              final MemoryFileSystem fs = MemoryFileSystem();
              final BufferLogger testLogger = BufferLogger.test();
              final File cocoapodPluginPodspec = fs.file(
                '/path/to/cocoapod_plugin_1/darwin/cocoapod_plugin_1.podspec',
              )..createSync(recursive: true);
              final List<Plugin> plugins = <Plugin>[
                FakePlugin(
                  name: 'cocoapod_plugin_1',
                  platforms: <String, PluginPlatform>{platform.name: FakePluginPlatform()},
                  pluginPodspecPath: cocoapodPluginPodspec.path,
                ),
              ];
              final FakeSwiftPackageManager swiftPackageManager = FakeSwiftPackageManager(
                expectedPlugins: plugins,
              );
              final FakeCocoaPods cocoaPods = FakeCocoaPods();

              final DarwinDependencyManagement dependencyManagement = DarwinDependencyManagement(
                project: FakeFlutterProject(usesSwiftPackageManager: true, fileSystem: fs),
                plugins: plugins,
                cocoapods: cocoaPods,
                swiftPackageManager: swiftPackageManager,
                fileSystem: fs,
                logger: testLogger,
              );
              await dependencyManagement.setUp(platform: platform);
              expect(swiftPackageManager.generated, isTrue);
              expect(testLogger.warningText, isEmpty);
              expect(testLogger.statusText, isEmpty);
              expect(cocoaPods.podfileSetup, isTrue);
            });

            testWithoutContext(
              'with only Swift Package Manager plugins and no pod integration',
              () async {
                final MemoryFileSystem fs = MemoryFileSystem();
                final BufferLogger testLogger = BufferLogger.test();
                final File swiftPackagePluginPodspec = fs.file(
                  '/path/to/cocoapod_plugin_1/darwin/cocoapod_plugin_1/Package.swift',
                )..createSync(recursive: true);
                final List<Plugin> plugins = <Plugin>[
                  FakePlugin(
                    name: 'swift_package_plugin_1',
                    platforms: <String, PluginPlatform>{platform.name: FakePluginPlatform()},
                    pluginSwiftPackageManifestPath: swiftPackagePluginPodspec.path,
                  ),
                ];
                final FakeSwiftPackageManager swiftPackageManager = FakeSwiftPackageManager(
                  expectedPlugins: plugins,
                );
                final FakeCocoaPods cocoaPods = FakeCocoaPods();

                final DarwinDependencyManagement dependencyManagement = DarwinDependencyManagement(
                  project: FakeFlutterProject(usesSwiftPackageManager: true, fileSystem: fs),
                  plugins: plugins,
                  cocoapods: cocoaPods,
                  swiftPackageManager: swiftPackageManager,
                  fileSystem: fs,
                  logger: testLogger,
                );
                await dependencyManagement.setUp(platform: platform);
                expect(swiftPackageManager.generated, isTrue);
                expect(testLogger.warningText, isEmpty);
                expect(testLogger.statusText, isEmpty);
                expect(cocoaPods.podfileSetup, isFalse);
              },
            );

            testWithoutContext(
              'with only Swift Package Manager plugins but project not migrated',
              () async {
                final MemoryFileSystem fs = MemoryFileSystem();
                final BufferLogger testLogger = BufferLogger.test();
                final File swiftPackagePluginPodspec = fs.file(
                  '/path/to/cocoapod_plugin_1/darwin/cocoapod_plugin_1/Package.swift',
                )..createSync(recursive: true);
                final List<Plugin> plugins = <Plugin>[
                  FakePlugin(
                    name: 'swift_package_plugin_1',
                    platforms: <String, PluginPlatform>{platform.name: FakePluginPlatform()},
                    pluginSwiftPackageManifestPath: swiftPackagePluginPodspec.path,
                  ),
                ];
                final FakeSwiftPackageManager swiftPackageManager = FakeSwiftPackageManager(
                  expectedPlugins: plugins,
                );
                final File projectPodfile = fs.file('/path/to/Podfile')
                  ..createSync(recursive: true);
                projectPodfile.writeAsStringSync('Standard Podfile template');
                final FakeCocoaPods cocoaPods = FakeCocoaPods(podFile: projectPodfile);
                final FakeFlutterProject project = FakeFlutterProject(
                  usesSwiftPackageManager: true,
                  fileSystem: fs,
                );
                final XcodeBasedProject xcodeProject =
                    platform == SupportedPlatform.ios ? project.ios : project.macos;
                xcodeProject.podfile.createSync(recursive: true);
                xcodeProject.podfile.writeAsStringSync('Standard Podfile template');

                final DarwinDependencyManagement dependencyManagement = DarwinDependencyManagement(
                  project: project,
                  plugins: plugins,
                  cocoapods: cocoaPods,
                  swiftPackageManager: swiftPackageManager,
                  fileSystem: fs,
                  logger: testLogger,
                );
                await dependencyManagement.setUp(platform: platform);
                expect(swiftPackageManager.generated, isTrue);
                expect(testLogger.warningText, isEmpty);
                expect(testLogger.statusText, isEmpty);
                expect(cocoaPods.podfileSetup, isFalse);
              },
            );

            testWithoutContext(
              'with only Swift Package Manager plugins with preexisting standard CocoaPods Podfile',
              () async {
                final MemoryFileSystem fs = MemoryFileSystem();
                final BufferLogger testLogger = BufferLogger.test();
                final File swiftPackagePluginPodspec = fs.file(
                  '/path/to/cocoapod_plugin_1/darwin/cocoapod_plugin_1/Package.swift',
                )..createSync(recursive: true);
                final List<Plugin> plugins = <Plugin>[
                  FakePlugin(
                    name: 'swift_package_plugin_1',
                    platforms: <String, PluginPlatform>{platform.name: FakePluginPlatform()},
                    pluginSwiftPackageManifestPath: swiftPackagePluginPodspec.path,
                  ),
                ];
                final FakeSwiftPackageManager swiftPackageManager = FakeSwiftPackageManager(
                  expectedPlugins: plugins,
                );
                final File projectPodfile = fs.file('/path/to/Podfile')
                  ..createSync(recursive: true);
                projectPodfile.writeAsStringSync('Standard Podfile template');
                final FakeCocoaPods cocoaPods = FakeCocoaPods(podFile: projectPodfile);
                final FakeFlutterProject project = FakeFlutterProject(
                  usesSwiftPackageManager: true,
                  fileSystem: fs,
                );
                final XcodeBasedProject xcodeProject =
                    platform == SupportedPlatform.ios ? project.ios : project.macos;
                xcodeProject.podfile.createSync(recursive: true);
                xcodeProject.podfile.writeAsStringSync('Standard Podfile template');
                xcodeProject.xcodeProjectInfoFile.createSync(recursive: true);
                xcodeProject.xcodeProjectInfoFile.writeAsStringSync(
                  'FlutterGeneratedPluginSwiftPackage',
                );

                final DarwinDependencyManagement dependencyManagement = DarwinDependencyManagement(
                  project: project,
                  plugins: plugins,
                  cocoapods: cocoaPods,
                  swiftPackageManager: swiftPackageManager,
                  fileSystem: fs,
                  logger: testLogger,
                );
                await dependencyManagement.setUp(platform: platform);
                expect(swiftPackageManager.generated, isTrue);
                final String xcconfigPrefix = platform == SupportedPlatform.macos ? 'Flutter-' : '';
                expect(
                  testLogger.warningText,
                  contains(
                    'All plugins found for ${platform.name} are Swift Packages, '
                    'but your project still has CocoaPods integration. To remove '
                    'CocoaPods integration, complete the following steps:\n'
                    '  * In the ${platform.name}/ directory run "pod deintegrate"\n'
                    '  * Also in the ${platform.name}/ directory, delete the Podfile\n'
                    '  * Remove the include to "Pods/Target Support Files/Pods-Runner/Pods-Runner.debug.xcconfig" '
                    'in your ${platform.name}/Flutter/${xcconfigPrefix}Debug.xcconfig\n'
                    '  * Remove the include to "Pods/Target Support Files/Pods-Runner/Pods-Runner.release.xcconfig" '
                    'in your ${platform.name}/Flutter/${xcconfigPrefix}Release.xcconfig\n\n'
                    "Removing CocoaPods integration will improve the project's build time.\n",
                  ),
                );
                expect(testLogger.statusText, isEmpty);
                expect(cocoaPods.podfileSetup, isFalse);
              },
            );

            testWithoutContext(
              'with only Swift Package Manager plugins with preexisting custom CocoaPods Podfile',
              () async {
                final MemoryFileSystem fs = MemoryFileSystem();
                final BufferLogger testLogger = BufferLogger.test();
                final File swiftPackagePluginPodspec = fs.file(
                  '/path/to/cocoapod_plugin_1/darwin/cocoapod_plugin_1/Package.swift',
                )..createSync(recursive: true);
                final List<Plugin> plugins = <Plugin>[
                  FakePlugin(
                    name: 'swift_package_plugin_1',
                    platforms: <String, PluginPlatform>{platform.name: FakePluginPlatform()},
                    pluginSwiftPackageManifestPath: swiftPackagePluginPodspec.path,
                  ),
                ];
                final FakeSwiftPackageManager swiftPackageManager = FakeSwiftPackageManager(
                  expectedPlugins: plugins,
                );
                final File projectPodfile = fs.file('/path/to/Podfile')
                  ..createSync(recursive: true);
                projectPodfile.writeAsStringSync('Standard Podfile template');
                final FakeCocoaPods cocoaPods = FakeCocoaPods(podFile: projectPodfile);
                final FakeFlutterProject project = FakeFlutterProject(
                  usesSwiftPackageManager: true,
                  fileSystem: fs,
                );
                final XcodeBasedProject xcodeProject =
                    platform == SupportedPlatform.ios ? project.ios : project.macos;
                xcodeProject.podfile.createSync(recursive: true);
                xcodeProject.podfile.writeAsStringSync('Non-Standard Podfile template');
                xcodeProject.xcodeProjectInfoFile.createSync(recursive: true);
                xcodeProject.xcodeProjectInfoFile.writeAsStringSync(
                  'FlutterGeneratedPluginSwiftPackage',
                );

                final DarwinDependencyManagement dependencyManagement = DarwinDependencyManagement(
                  project: project,
                  plugins: plugins,
                  cocoapods: cocoaPods,
                  swiftPackageManager: swiftPackageManager,
                  fileSystem: fs,
                  logger: testLogger,
                );
                await dependencyManagement.setUp(platform: platform);
                expect(swiftPackageManager.generated, isTrue);
                final String xcconfigPrefix = platform == SupportedPlatform.macos ? 'Flutter-' : '';
                expect(
                  testLogger.warningText,
                  contains(
                    'All plugins found for ${platform.name} are Swift Packages, '
                    'but your project still has CocoaPods integration. Your '
                    'project uses a non-standard Podfile and will need to be '
                    'migrated to Swift Package Manager manually. Some steps you '
                    'may need to complete include:\n'
                    '  * In the ${platform.name}/ directory run "pod deintegrate"\n'
                    '  * Transition any Pod dependencies to Swift Package equivalents. '
                    'See https://developer.apple.com/documentation/xcode/adding-package-dependencies-to-your-app\n'
                    '  * Transition any custom logic\n'
                    '  * Remove the include to "Pods/Target Support Files/Pods-Runner/Pods-Runner.debug.xcconfig" '
                    'in your ${platform.name}/Flutter/${xcconfigPrefix}Debug.xcconfig\n'
                    '  * Remove the include to "Pods/Target Support Files/Pods-Runner/Pods-Runner.release.xcconfig" '
                    'in your ${platform.name}/Flutter/${xcconfigPrefix}Release.xcconfig\n\n'
                    "Removing CocoaPods integration will improve the project's build time.\n",
                  ),
                );
                expect(testLogger.statusText, isEmpty);
                expect(cocoaPods.podfileSetup, isFalse);
              },
            );

            testWithoutContext('with mixed plugins', () async {
              final MemoryFileSystem fs = MemoryFileSystem();
              final BufferLogger testLogger = BufferLogger.test();
              final File cocoapodPluginPodspec = fs.file(
                '/path/to/cocoapod_plugin_1/darwin/cocoapod_plugin_1.podspec',
              )..createSync(recursive: true);
              final File swiftPackagePluginPodspec = fs.file(
                '/path/to/cocoapod_plugin_1/darwin/cocoapod_plugin_1/Package.swift',
              )..createSync(recursive: true);
              final List<Plugin> plugins = <Plugin>[
                FakePlugin(
                  name: 'cocoapod_plugin_1',
                  platforms: <String, PluginPlatform>{platform.name: FakePluginPlatform()},
                  pluginPodspecPath: cocoapodPluginPodspec.path,
                ),
                FakePlugin(
                  name: 'swift_package_plugin_1',
                  platforms: <String, PluginPlatform>{platform.name: FakePluginPlatform()},
                  pluginSwiftPackageManifestPath: swiftPackagePluginPodspec.path,
                ),
                FakePlugin(
                  name: 'neither_plugin_1',
                  platforms: <String, PluginPlatform>{platform.name: FakePluginPlatform()},
                ),
              ];
              final FakeSwiftPackageManager swiftPackageManager = FakeSwiftPackageManager(
                expectedPlugins: plugins,
              );
              final FakeCocoaPods cocoaPods = FakeCocoaPods();

              final DarwinDependencyManagement dependencyManagement = DarwinDependencyManagement(
                project: FakeFlutterProject(usesSwiftPackageManager: true, fileSystem: fs),
                plugins: plugins,
                cocoapods: cocoaPods,
                swiftPackageManager: swiftPackageManager,
                fileSystem: fs,
                logger: testLogger,
              );
              await dependencyManagement.setUp(platform: platform);
              expect(swiftPackageManager.generated, isTrue);
              expect(testLogger.warningText, isEmpty);
              expect(testLogger.statusText, isEmpty);
              expect(cocoaPods.podfileSetup, isTrue);
            });
          });

          group('when not using Swift Package Manager', () {
            testWithoutContext('but project already migrated', () async {
              final MemoryFileSystem fs = MemoryFileSystem();
              final BufferLogger testLogger = BufferLogger.test();
              final File cocoapodPluginPodspec = fs.file(
                '/path/to/cocoapod_plugin_1/darwin/cocoapod_plugin_1.podspec',
              )..createSync(recursive: true);
              final List<Plugin> plugins = <Plugin>[
                FakePlugin(
                  name: 'cocoapod_plugin_1',
                  platforms: <String, PluginPlatform>{platform.name: FakePluginPlatform()},
                  pluginPodspecPath: cocoapodPluginPodspec.path,
                ),
              ];
              final FakeSwiftPackageManager swiftPackageManager = FakeSwiftPackageManager(
                expectedPlugins: plugins,
              );
              final FakeCocoaPods cocoaPods = FakeCocoaPods();
              final FakeFlutterProject project = FakeFlutterProject(
                usesSwiftPackageManager: true,
                fileSystem: fs,
              );
              final XcodeBasedProject xcodeProject =
                  platform == SupportedPlatform.ios ? project.ios : project.macos;
              xcodeProject.xcodeProjectInfoFile.createSync(recursive: true);
              xcodeProject.xcodeProjectInfoFile.writeAsStringSync(
                'FlutterGeneratedPluginSwiftPackage',
              );

              final DarwinDependencyManagement dependencyManagement = DarwinDependencyManagement(
                project: project,
                plugins: plugins,
                cocoapods: cocoaPods,
                swiftPackageManager: swiftPackageManager,
                fileSystem: fs,
                logger: testLogger,
              );
              await dependencyManagement.setUp(platform: platform);
              expect(swiftPackageManager.generated, isTrue);
              expect(testLogger.warningText, isEmpty);
              expect(testLogger.statusText, isEmpty);
              expect(cocoaPods.podfileSetup, isTrue);
            });

            testWithoutContext('with only CocoaPod plugins', () async {
              final MemoryFileSystem fs = MemoryFileSystem();
              final BufferLogger testLogger = BufferLogger.test();
              final File cocoapodPluginPodspec = fs.file(
                '/path/to/cocoapod_plugin_1/darwin/cocoapod_plugin_1.podspec',
              )..createSync(recursive: true);
              final List<Plugin> plugins = <Plugin>[
                FakePlugin(
                  name: 'cocoapod_plugin_1',
                  platforms: <String, PluginPlatform>{platform.name: FakePluginPlatform()},
                  pluginPodspecPath: cocoapodPluginPodspec.path,
                ),
              ];
              final FakeSwiftPackageManager swiftPackageManager = FakeSwiftPackageManager(
                expectedPlugins: plugins,
              );
              final FakeCocoaPods cocoaPods = FakeCocoaPods();

              final DarwinDependencyManagement dependencyManagement = DarwinDependencyManagement(
                project: FakeFlutterProject(fileSystem: fs),
                plugins: plugins,
                cocoapods: cocoaPods,
                swiftPackageManager: swiftPackageManager,
                fileSystem: fs,
                logger: testLogger,
              );
              await dependencyManagement.setUp(platform: platform);
              expect(swiftPackageManager.generated, isFalse);
              expect(testLogger.warningText, isEmpty);
              expect(testLogger.statusText, isEmpty);
              expect(cocoaPods.podfileSetup, isTrue);
            });

            testWithoutContext('with only Swift Package Manager plugins', () async {
              final MemoryFileSystem fs = MemoryFileSystem();
              final BufferLogger testLogger = BufferLogger.test();
              final File swiftPackagePluginPodspec = fs.file(
                '/path/to/cocoapod_plugin_1/darwin/cocoapod_plugin_1/Package.swift',
              )..createSync(recursive: true);
              final List<Plugin> plugins = <Plugin>[
                FakePlugin(
                  name: 'swift_package_plugin_1',
                  platforms: <String, PluginPlatform>{platform.name: FakePluginPlatform()},
                  pluginSwiftPackageManifestPath: swiftPackagePluginPodspec.path,
                ),
              ];
              final FakeSwiftPackageManager swiftPackageManager = FakeSwiftPackageManager(
                expectedPlugins: plugins,
              );
              final FakeCocoaPods cocoaPods = FakeCocoaPods();

              final DarwinDependencyManagement dependencyManagement = DarwinDependencyManagement(
                project: FakeFlutterProject(fileSystem: fs),
                plugins: plugins,
                cocoapods: cocoaPods,
                swiftPackageManager: swiftPackageManager,
                fileSystem: fs,
                logger: testLogger,
              );
              await expectLater(
                () => dependencyManagement.setUp(platform: platform),
                throwsToolExit(
                  message:
                      'Plugin swift_package_plugin_1 is only Swift Package Manager compatible. Try '
                      'enabling Swift Package Manager by running '
                      '"flutter config --enable-swift-package-manager" or remove the '
                      'plugin as a dependency.',
                ),
              );
              expect(swiftPackageManager.generated, isFalse);
              expect(cocoaPods.podfileSetup, isFalse);
            });

            testWithoutContext('when project is a module', () async {
              final MemoryFileSystem fs = MemoryFileSystem();
              final BufferLogger testLogger = BufferLogger.test();
              final File cocoapodPluginPodspec = fs.file(
                '/path/to/cocoapod_plugin_1/darwin/cocoapod_plugin_1.podspec',
              )..createSync(recursive: true);
              final List<Plugin> plugins = <Plugin>[
                FakePlugin(
                  name: 'cocoapod_plugin_1',
                  platforms: <String, PluginPlatform>{platform.name: FakePluginPlatform()},
                  pluginPodspecPath: cocoapodPluginPodspec.path,
                ),
              ];
              final FakeSwiftPackageManager swiftPackageManager = FakeSwiftPackageManager(
                expectedPlugins: plugins,
              );
              final FakeCocoaPods cocoaPods = FakeCocoaPods();

              final DarwinDependencyManagement dependencyManagement = DarwinDependencyManagement(
                project: FakeFlutterProject(fileSystem: fs, isModule: true),
                plugins: plugins,
                cocoapods: cocoaPods,
                swiftPackageManager: swiftPackageManager,
                fileSystem: fs,
                logger: testLogger,
              );
              await dependencyManagement.setUp(platform: platform);
              expect(swiftPackageManager.generated, isFalse);
              expect(testLogger.warningText, isEmpty);
              expect(testLogger.statusText, isEmpty);
              expect(cocoaPods.podfileSetup, isFalse);
            });
          });
        });
      });
    }
  });
}

class FakeIosProject extends Fake implements IosProject {
  FakeIosProject({required MemoryFileSystem fileSystem, required this.usesSwiftPackageManager})
    : hostAppRoot = fileSystem.directory('app_name').childDirectory('ios');

  @override
  Directory hostAppRoot;

  @override
  File get podfile => hostAppRoot.childFile('Podfile');

  @override
  File get podfileLock => hostAppRoot.childFile('Podfile.lock');

  @override
  Directory get xcodeProject => hostAppRoot.childDirectory('Runner.xcodeproj');

  @override
  File get xcodeProjectInfoFile => xcodeProject.childFile('project.pbxproj');

  @override
  bool get flutterPluginSwiftPackageInProjectSettings {
    return xcodeProjectInfoFile.existsSync() &&
        xcodeProjectInfoFile.readAsStringSync().contains('FlutterGeneratedPluginSwiftPackage');
  }

  @override
  bool usesSwiftPackageManager;

  @override
  Directory get managedDirectory => hostAppRoot.childDirectory('Flutter');

  @override
  File xcodeConfigFor(String mode) => managedDirectory.childFile('$mode.xcconfig');
}

class FakeMacOSProject extends Fake implements MacOSProject {
  FakeMacOSProject({required MemoryFileSystem fileSystem, required this.usesSwiftPackageManager})
    : hostAppRoot = fileSystem.directory('app_name').childDirectory('macos');

  @override
  Directory hostAppRoot;

  @override
  File get podfile => hostAppRoot.childFile('Podfile');

  @override
  File get podfileLock => hostAppRoot.childFile('Podfile.lock');

  @override
  Directory get xcodeProject => hostAppRoot.childDirectory('Runner.xcodeproj');

  @override
  File get xcodeProjectInfoFile => xcodeProject.childFile('project.pbxproj');

  @override
  bool get flutterPluginSwiftPackageInProjectSettings {
    return xcodeProjectInfoFile.existsSync() &&
        xcodeProjectInfoFile.readAsStringSync().contains('FlutterGeneratedPluginSwiftPackage');
  }

  @override
  bool usesSwiftPackageManager;

  @override
  Directory get managedDirectory => hostAppRoot.childDirectory('Flutter');

  @override
  File xcodeConfigFor(String mode) => managedDirectory.childFile('Flutter-$mode.xcconfig');
}

class FakeFlutterProject extends Fake implements FlutterProject {
  FakeFlutterProject({
    required this.fileSystem,
    this.usesSwiftPackageManager = false,
    this.isModule = false,
  });

  final MemoryFileSystem fileSystem;
  final bool usesSwiftPackageManager;

  @override
  late final FakeIosProject ios = FakeIosProject(
    fileSystem: fileSystem,
    usesSwiftPackageManager: usesSwiftPackageManager,
  );

  @override
  late final FakeMacOSProject macos = FakeMacOSProject(
    fileSystem: fileSystem,
    usesSwiftPackageManager: usesSwiftPackageManager,
  );

  @override
  final bool isModule;
}

class FakeSwiftPackageManager extends Fake implements SwiftPackageManager {
  FakeSwiftPackageManager({this.expectedPlugins});

  bool generated = false;
  final List<Plugin>? expectedPlugins;

  @override
  Future<void> generatePluginsSwiftPackage(
    List<Plugin> plugins,
    SupportedPlatform platform,
    XcodeBasedProject project,
  ) async {
    generated = true;
    expect(plugins, expectedPlugins);
  }
}

class FakeCocoaPods extends Fake implements CocoaPods {
  FakeCocoaPods({this.podFile, this.configIncludesPods = true});

  File? podFile;

  bool podfileSetup = false;
  bool addedPodDependencyToFlutterXcconfig = false;
  bool configIncludesPods;

  @override
  Future<void> setupPodfile(XcodeBasedProject xcodeProject) async {
    podfileSetup = true;
  }

  @override
  void addPodsDependencyToFlutterXcconfig(XcodeBasedProject xcodeProject) {
    addedPodDependencyToFlutterXcconfig = true;
  }

  @override
  Future<File> getPodfileTemplate(XcodeBasedProject xcodeProject, Directory runnerProject) async {
    return podFile!;
  }

  @override
  bool xcconfigIncludesPods(File xcodeConfig) {
    return configIncludesPods;
  }

  @override
  String includePodsXcconfig(String mode) {
    return 'Pods/Target Support Files/Pods-Runner/Pods-Runner.${mode.toLowerCase()}.xcconfig';
  }
}

class FakePlugin extends Fake implements Plugin {
  FakePlugin({
    required this.name,
    required this.platforms,
    String? pluginSwiftPackageManifestPath,
    String? pluginPodspecPath,
  }) : _pluginSwiftPackageManifestPath = pluginSwiftPackageManifestPath,
       _pluginPodspecPath = pluginPodspecPath;

  final String? _pluginSwiftPackageManifestPath;

  final String? _pluginPodspecPath;

  @override
  final String name;

  @override
  final Map<String, PluginPlatform> platforms;

  @override
  String? pluginSwiftPackageManifestPath(FileSystem fileSystem, String platform) {
    return _pluginSwiftPackageManifestPath;
  }

  @override
  String? pluginPodspecPath(FileSystem fileSystem, String platform) {
    return _pluginPodspecPath;
  }
}

class FakePluginPlatform extends Fake implements PluginPlatform {}
