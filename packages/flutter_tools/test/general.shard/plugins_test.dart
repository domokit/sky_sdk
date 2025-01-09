// Copyright 2014 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

import 'dart:convert';
import 'dart:io';

import 'package:file/file.dart';
import 'package:file/memory.dart';
import 'package:file_testing/file_testing.dart';
import 'package:flutter_tools/src/base/error_handling_io.dart';
import 'package:flutter_tools/src/base/logger.dart';
import 'package:flutter_tools/src/base/os.dart';
import 'package:flutter_tools/src/base/platform.dart';
import 'package:flutter_tools/src/base/time.dart';
import 'package:flutter_tools/src/base/utils.dart';
import 'package:flutter_tools/src/dart/pub.dart';
import 'package:flutter_tools/src/features.dart';
import 'package:flutter_tools/src/flutter_manifest.dart';
import 'package:flutter_tools/src/flutter_plugins.dart';
import 'package:flutter_tools/src/globals.dart' as globals;
import 'package:flutter_tools/src/ios/xcodeproj.dart';
import 'package:flutter_tools/src/macos/darwin_dependency_management.dart';
import 'package:flutter_tools/src/platform_plugins.dart';
import 'package:flutter_tools/src/plugins.dart';
import 'package:flutter_tools/src/preview_device.dart';
import 'package:flutter_tools/src/project.dart';
import 'package:flutter_tools/src/version.dart';
import 'package:test/fake.dart';
import 'package:yaml/yaml.dart';

import '../src/common.dart';
import '../src/context.dart';
import '../src/fake_pub_deps.dart';
import '../src/fakes.dart' hide FakeOperatingSystemUtils;
import '../src/pubspec_schema.dart';

/// Information for a platform entry in the 'platforms' section of a plugin's
/// pubspec.yaml.
class _PluginPlatformInfo {
  const _PluginPlatformInfo({
    this.pluginClass,
    this.dartPluginClass,
    this.androidPackage,
    this.sharedDarwinSource = false,
    this.fileName,
  }) : assert(pluginClass != null || dartPluginClass != null),
       assert(androidPackage == null || pluginClass != null);

  /// The pluginClass entry, if any.
  final String? pluginClass;

  /// The dartPluginClass entry, if any.
  final String? dartPluginClass;

  /// The package entry for an Android plugin implementation using pluginClass.
  final String? androidPackage;

  final bool sharedDarwinSource;

  /// The fileName entry for a web plugin implementation.
  final String? fileName;

  /// Returns the body of a platform section for a plugin's pubspec, properly
  /// indented.
  String get indentedPubspecSection {
    const String indentation = '        ';
    return <String>[
      if (pluginClass != null) '${indentation}pluginClass: $pluginClass',
      if (dartPluginClass != null) '${indentation}dartPluginClass: $dartPluginClass',
      if (androidPackage != null) '${indentation}package: $androidPackage',
      if (sharedDarwinSource) '${indentation}sharedDarwinSource: true',
      if (fileName != null) '${indentation}fileName: $fileName',
    ].join('\n');
  }
}

void main() {
  // TODO(matanlurey): Remove after `explicit-package-dependencies` is enabled by default.
  // See https://github.com/flutter/flutter/issues/160257 for details.
  FeatureFlags enableExplicitPackageDependencies() {
    return TestFeatureFlags(isExplicitPackageDependenciesEnabled: true);
  }

  // TODO(matanlurey): Remove after `explicit-package-dependencies` is removed.
  // See https://github.com/flutter/flutter/issues/48918 for details.
  FeatureFlags disableExplicitPackageDependencies() {
    return TestFeatureFlags(
      // ignore: avoid_redundant_argument_values
      isExplicitPackageDependenciesEnabled: false,
    );
  }

  group('plugins', () {
    late FileSystem fs;
    late FakeFlutterProject flutterProject;
    late FakeFlutterManifest flutterManifest;
    late FakeIosProject iosProject;
    late FakeMacOSProject macosProject;
    late FakeAndroidProject androidProject;
    late FakeWebProject webProject;
    late FakeWindowsProject windowsProject;
    late FakeLinuxProject linuxProject;
    late FakeSystemClock systemClock;
    late FlutterVersion flutterVersion;
    // A Windows-style filesystem. This is not populated by default, so tests
    // using it instead of fs must re-run any necessary setup (e.g.,
    // setUpProject).
    late FileSystem fsWindows;
    const String pubCachePath = '/path/to/.pub-cache/hosted/pub.dartlang.org/foo-1.2.3';
    const String ephemeralPackagePath = '/path/to/app/linux/flutter/ephemeral/foo-1.2.3';

    // Adds basic properties to the flutterProject and its subprojects.
    void setUpProject(FileSystem fileSystem) {
      flutterProject = FakeFlutterProject();
      flutterManifest = FakeFlutterManifest();

      flutterProject
        ..manifest = flutterManifest
        ..directory = fileSystem.systemTempDirectory.childDirectory('app')
        ..flutterPluginsFile = flutterProject.directory.childFile('.flutter-plugins')
        ..flutterPluginsDependenciesFile = flutterProject.directory.childFile(
          '.flutter-plugins-dependencies',
        );

      iosProject = FakeIosProject();
      flutterProject.ios = iosProject;
      final Directory iosDirectory = flutterProject.directory.childDirectory('ios');
      iosProject
        ..pluginRegistrantHost = flutterProject.directory.childDirectory('Runner')
        ..podfile = iosDirectory.childFile('Podfile')
        ..podManifestLock = iosDirectory.childFile('Podfile.lock')
        ..testExists = false;

      macosProject = FakeMacOSProject();
      flutterProject.macos = macosProject;
      final Directory macosDirectory = flutterProject.directory.childDirectory('macos');
      final Directory macosManagedDirectory = macosDirectory.childDirectory('Flutter');
      macosProject
        ..podfile = macosDirectory.childFile('Podfile')
        ..podManifestLock = macosDirectory.childFile('Podfile.lock')
        ..managedDirectory = macosManagedDirectory
        ..exists = false;

      androidProject = FakeAndroidProject();
      flutterProject.android = androidProject;
      final Directory androidDirectory = flutterProject.directory.childDirectory('android');
      androidProject
        ..pluginRegistrantHost = androidDirectory.childDirectory('app')
        ..hostAppGradleRoot = androidDirectory
        ..exists = false
        ..embeddingVersion = AndroidEmbeddingVersion.v2;

      webProject = FakeWebProject();
      flutterProject.web = webProject;
      webProject
        ..libDirectory = flutterProject.directory.childDirectory('lib')
        ..exists = false;

      windowsProject = FakeWindowsProject();
      flutterProject.windows = windowsProject;
      final Directory windowsManagedDirectory = flutterProject.directory
          .childDirectory('windows')
          .childDirectory('flutter');
      windowsProject
        ..managedDirectory = windowsManagedDirectory
        ..cmakeFile = windowsManagedDirectory.parent.childFile('CMakeLists.txt')
        ..generatedPluginCmakeFile = windowsManagedDirectory.childFile('generated_plugins.mk')
        ..pluginSymlinkDirectory = windowsManagedDirectory
            .childDirectory('ephemeral')
            .childDirectory('.plugin_symlinks')
        ..exists = false;

      linuxProject = FakeLinuxProject();
      flutterProject.linux = linuxProject;
      final Directory linuxManagedDirectory = flutterProject.directory
          .childDirectory('linux')
          .childDirectory('flutter');
      final Directory linuxEphemeralDirectory = linuxManagedDirectory.childDirectory('ephemeral');
      linuxProject
        ..managedDirectory = linuxManagedDirectory
        ..ephemeralDirectory = linuxEphemeralDirectory
        ..pluginSymlinkDirectory = linuxEphemeralDirectory.childDirectory('.plugin_symlinks')
        ..cmakeFile = linuxManagedDirectory.parent.childFile('CMakeLists.txt')
        ..generatedPluginCmakeFile = linuxManagedDirectory.childFile('generated_plugins.mk')
        ..exists = false;
    }

    setUp(() async {
      fs = MemoryFileSystem.test();
      fsWindows = MemoryFileSystem(style: FileSystemStyle.windows);
      systemClock = FakeSystemClock()..currentTime = DateTime(1970);
      flutterVersion = FakeFlutterVersion(frameworkVersion: '1.0.0');

      // Add basic properties to the Flutter project and subprojects
      setUpProject(fs);
      flutterProject.directory.childDirectory('.dart_tool').childFile('package_config.json')
        ..createSync(recursive: true)
        ..writeAsStringSync('''
{
  "packages": [],
  "configVersion": 2
}
''');
    });

    void addToPackageConfig(String name, Directory packageDir) {
      final File packageConfigFile = flutterProject.directory
          .childDirectory('.dart_tool')
          .childFile('package_config.json');

      final Map<String, Object?> packageConfig =
          jsonDecode(packageConfigFile.readAsStringSync()) as Map<String, Object?>;

      (packageConfig['packages']! as List<Object?>).add(<String, Object?>{
        'name': name,
        'rootUri': packageDir.uri.toString(),
        'packageUri': 'lib/',
      });

      packageConfigFile.writeAsStringSync(jsonEncode(packageConfig));
    }

    // Makes fake plugin packages for each plugin, adds them to flutterProject,
    // and returns their directories.
    //
    // If an entry contains a path separator, it will be treated as a path for
    // the location of the package, with the name being the last component.
    // Otherwise it will be treated as a name, and put in a default location
    // (a fake pub cache).
    List<Directory> createFakePlugins(FileSystem fileSystem, List<String> pluginNamesOrPaths) {
      const String pluginYamlTemplate = '''
  flutter:
    plugin:
      platforms:
        ios:
          pluginClass: PLUGIN_CLASS
        macos:
          pluginClass: PLUGIN_CLASS
        windows:
          pluginClass: PLUGIN_CLASS
        linux:
          pluginClass: PLUGIN_CLASS
        web:
          pluginClass: PLUGIN_CLASS
          fileName: lib/PLUGIN_CLASS.dart
        android:
          pluginClass: PLUGIN_CLASS
          package: AndroidPackage
  ''';

      final List<Directory> directories = <Directory>[];
      final Directory fakePubCache = fileSystem.systemTempDirectory.childDirectory('cache');
      flutterProject.directory.childDirectory('.dart_tool').childFile('package_config.json')
        ..createSync(recursive: true)
        ..writeAsStringSync('''
{
  "packages": [],
  "configVersion": 2
}
''');
      for (final String nameOrPath in pluginNamesOrPaths) {
        final String name = fileSystem.path.basename(nameOrPath);
        final Directory pluginDirectory =
            (nameOrPath == name)
                ? fakePubCache.childDirectory(name)
                : fileSystem.directory(nameOrPath);
        addToPackageConfig(name, pluginDirectory);
        pluginDirectory.childFile('pubspec.yaml')
          ..createSync(recursive: true)
          ..writeAsStringSync(
            pluginYamlTemplate.replaceAll('PLUGIN_CLASS', sentenceCase(camelCase(name))),
          );
        directories.add(pluginDirectory);
      }
      return directories;
    }

    // Makes a fake plugin package, adds it to flutterProject, and returns its directory.
    Directory createFakePlugin(FileSystem fileSystem) {
      return createFakePlugins(fileSystem, <String>['some_plugin'])[0];
    }

    void createNewJavaPlugin1() {
      final Directory pluginUsingJavaAndNewEmbeddingDir = fs.systemTempDirectory.createTempSync(
        'flutter_plugin_using_java_and_new_embedding_dir.',
      );
      pluginUsingJavaAndNewEmbeddingDir.childFile('pubspec.yaml').writeAsStringSync('''
flutter:
  plugin:
    androidPackage: plugin1
    pluginClass: UseNewEmbedding
              ''');
      pluginUsingJavaAndNewEmbeddingDir
          .childDirectory('android')
          .childDirectory('src')
          .childDirectory('main')
          .childDirectory('java')
          .childDirectory('plugin1')
          .childFile('UseNewEmbedding.java')
        ..createSync(recursive: true)
        ..writeAsStringSync('import io.flutter.embedding.engine.plugins.FlutterPlugin;');
      addToPackageConfig('plugin1', pluginUsingJavaAndNewEmbeddingDir);
    }

    Directory createPluginWithInvalidAndroidPackage() {
      final Directory pluginUsingJavaAndNewEmbeddingDir = fs.systemTempDirectory.createTempSync(
        'flutter_plugin_invalid_package.',
      );
      pluginUsingJavaAndNewEmbeddingDir.childFile('pubspec.yaml').writeAsStringSync('''
flutter:
  plugin:
    androidPackage: plugin1.invalid
    pluginClass: UseNewEmbedding
              ''');
      pluginUsingJavaAndNewEmbeddingDir
          .childDirectory('android')
          .childDirectory('src')
          .childDirectory('main')
          .childDirectory('java')
          .childDirectory('plugin1')
          .childDirectory('correct')
          .childFile('UseNewEmbedding.java')
        ..createSync(recursive: true)
        ..writeAsStringSync('import io.flutter.embedding.engine.plugins.FlutterPlugin;');

      addToPackageConfig('plugin1', pluginUsingJavaAndNewEmbeddingDir);
      return pluginUsingJavaAndNewEmbeddingDir;
    }

    void createDualSupportJavaPlugin4() {
      final Directory pluginUsingJavaAndNewEmbeddingDir = fs.systemTempDirectory.createTempSync(
        'flutter_plugin_using_java_and_new_embedding_dir.',
      );
      pluginUsingJavaAndNewEmbeddingDir.childFile('pubspec.yaml').writeAsStringSync('''
flutter:
  plugin:
    androidPackage: plugin4
    pluginClass: UseBothEmbedding
''');
      pluginUsingJavaAndNewEmbeddingDir
          .childDirectory('android')
          .childDirectory('src')
          .childDirectory('main')
          .childDirectory('java')
          .childDirectory('plugin4')
          .childFile('UseBothEmbedding.java')
        ..createSync(recursive: true)
        ..writeAsStringSync(
          'import io.flutter.embedding.engine.plugins.FlutterPlugin;\n'
          'PluginRegistry\n'
          'registerWith(Irrelevant registrar)\n',
        );

      addToPackageConfig('plugin4', pluginUsingJavaAndNewEmbeddingDir);
    }

    Directory createLegacyPluginWithDependencies({
      required String name,
      required List<String> dependencies,
    }) {
      final Directory pluginDirectory = fs.systemTempDirectory.createTempSync('flutter_plugin.');
      pluginDirectory.childFile('pubspec.yaml').writeAsStringSync('''
name: $name
flutter:
  plugin:
    androidPackage: plugin2
    pluginClass: UseNewEmbedding
dependencies:
''');
      for (final String dependency in dependencies) {
        pluginDirectory
            .childFile('pubspec.yaml')
            .writeAsStringSync('  $dependency:\n', mode: FileMode.append);
      }
      addToPackageConfig(name, pluginDirectory);
      return pluginDirectory;
    }

    Directory createPlugin({
      required String name,
      required Map<String, _PluginPlatformInfo> platforms,
      List<String> dependencies = const <String>[],
    }) {
      final Iterable<String> platformSections = platforms.entries.map(
        (MapEntry<String, _PluginPlatformInfo> entry) => '''
      ${entry.key}:
${entry.value.indentedPubspecSection}
''',
      );
      final Directory pluginDirectory = fs.systemTempDirectory.createTempSync('flutter_plugin.');
      pluginDirectory.childFile('pubspec.yaml').writeAsStringSync('''
name: $name
flutter:
  plugin:
    platforms:
${platformSections.join('\n')}

dependencies:
''');
      for (final String dependency in dependencies) {
        pluginDirectory
            .childFile('pubspec.yaml')
            .writeAsStringSync('  $dependency:\n', mode: FileMode.append);
      }
      addToPackageConfig(name, pluginDirectory);
      return pluginDirectory;
    }

    // Creates the files that would indicate that pod install has run for the
    // given project.
    void simulatePodInstallRun(XcodeBasedProject project) {
      project.podManifestLock.createSync(recursive: true);
    }

    group('refreshPlugins', () {
      testUsingContext(
        'Refreshing the plugin list is a no-op when the plugins list stays empty',
        () async {
          await refreshPluginsList(flutterProject);

          expect(flutterProject.flutterPluginsDependenciesFile.existsSync(), false);
        },
        overrides: <Type, Generator>{
          FileSystem: () => fs,
          ProcessManager: () => FakeProcessManager.any(),
          FeatureFlags: enableExplicitPackageDependencies,
          Pub: FakePubWithPrimedDeps.new,
        },
      );

      testUsingContext(
        'Refreshing the plugin list deletes the plugin file when there were plugins but no longer are',
        () async {
          flutterProject.flutterPluginsFile.createSync();

          await refreshPluginsList(flutterProject);

          expect(flutterProject.flutterPluginsDependenciesFile.existsSync(), false);
        },
        overrides: <Type, Generator>{
          FileSystem: () => fs,
          ProcessManager: () => FakeProcessManager.any(),
          FeatureFlags: enableExplicitPackageDependencies,
          Pub: FakePubWithPrimedDeps.new,
        },
      );

      testUsingContext(
        'Refreshing the plugin list creates a sorted plugin directory when there are plugins',
        () async {
          createFakePlugins(fs, <String>[
            'plugin_d',
            'plugin_a',
            '/local_plugins/plugin_c',
            '/local_plugins/plugin_b',
          ]);

          iosProject.testExists = true;

          await refreshPluginsList(flutterProject);

          expect(
            flutterProject.flutterPluginsFile.existsSync(),
            false,
            reason: 'No longer emitted',
          );
          expect(flutterProject.flutterPluginsDependenciesFile.existsSync(), true);

          final String pluginsFileContents =
              flutterProject.flutterPluginsDependenciesFile.readAsStringSync();
          expect(
            pluginsFileContents.indexOf('plugin_a'),
            lessThan(pluginsFileContents.indexOf('plugin_b')),
          );
          expect(
            pluginsFileContents.indexOf('plugin_b'),
            lessThan(pluginsFileContents.indexOf('plugin_c')),
          );
          expect(
            pluginsFileContents.indexOf('plugin_c'),
            lessThan(pluginsFileContents.indexOf('plugin_d')),
          );
        },
        overrides: <Type, Generator>{
          FileSystem: () => fs,
          ProcessManager: () => FakeProcessManager.any(),
          FeatureFlags: enableExplicitPackageDependencies,
          Pub: FakePubWithPrimedDeps.new,
        },
      );

      testUsingContext(
        'Opting in to explicit-package-dependencies omits .flutter-plugins',
        () async {
          createFakePlugins(fs, <String>[
            'plugin_d',
            'plugin_a',
            '/local_plugins/plugin_c',
            '/local_plugins/plugin_b',
          ]);

          await refreshPluginsList(flutterProject);

          expect(flutterProject.flutterPluginsFile, isNot(exists));
          expect(flutterProject.flutterPluginsDependenciesFile, exists);
        },
        overrides: <Type, Generator>{
          FileSystem: () => fs,
          ProcessManager: FakeProcessManager.empty,
          FeatureFlags: enableExplicitPackageDependencies,
          Pub: FakePubWithPrimedDeps.new,
        },
      );

      testUsingContext(
        'Refreshing the plugin list modifies .flutter-plugins '
        'and .flutter-plugins-dependencies when there are plugins',
        () async {
          final Directory pluginA = createLegacyPluginWithDependencies(
            name: 'plugin-a',
            dependencies: const <String>['plugin-b', 'plugin-c', 'random-package'],
          );
          final Directory pluginB = createLegacyPluginWithDependencies(
            name: 'plugin-b',
            dependencies: const <String>['plugin-c'],
          );
          final Directory pluginC = createLegacyPluginWithDependencies(
            name: 'plugin-c',
            dependencies: const <String>[],
          );
          iosProject.testExists = true;

          final DateTime dateCreated = DateTime(1970);
          systemClock.currentTime = dateCreated;

          await refreshPluginsList(flutterProject);

          // Verify .flutter-plugins-dependencies is configured correctly.
          expect(flutterProject.flutterPluginsFile.existsSync(), true);
          expect(flutterProject.flutterPluginsDependenciesFile.existsSync(), true);
          expect(
            flutterProject.flutterPluginsFile.readAsStringSync(),
            '# This is a generated file; do not edit or check into version control.\n'
            'plugin-a=${pluginA.path}/\n'
            'plugin-b=${pluginB.path}/\n'
            'plugin-c=${pluginC.path}/\n',
          );

          final String pluginsString =
              flutterProject.flutterPluginsDependenciesFile.readAsStringSync();
          final Map<String, dynamic> jsonContent =
              json.decode(pluginsString) as Map<String, dynamic>;
          expect(
            jsonContent['info'],
            'This is a generated file; do not edit or check into version control.',
          );

          final Map<String, dynamic> plugins = jsonContent['plugins'] as Map<String, dynamic>;
          final List<dynamic> expectedPlugins = <dynamic>[
            <String, dynamic>{
              'name': 'plugin-a',
              'path': '${pluginA.path}/',
              'native_build': true,
              'dependencies': <String>['plugin-b', 'plugin-c'],
              'dev_dependency': false,
            },
            <String, dynamic>{
              'name': 'plugin-b',
              'path': '${pluginB.path}/',
              'native_build': true,
              'dependencies': <String>['plugin-c'],
              'dev_dependency': false,
            },
            <String, dynamic>{
              'name': 'plugin-c',
              'path': '${pluginC.path}/',
              'native_build': true,
              'dependencies': <String>[],
              'dev_dependency': false,
            },
          ];
          expect(plugins['ios'], expectedPlugins);
          expect(plugins['android'], expectedPlugins);
          expect(plugins['macos'], <dynamic>[]);
          expect(plugins['windows'], <dynamic>[]);
          expect(plugins['linux'], <dynamic>[]);
          expect(plugins['web'], <dynamic>[]);

          final List<dynamic> expectedDependencyGraph = <dynamic>[
            <String, dynamic>{
              'name': 'plugin-a',
              'dependencies': <String>['plugin-b', 'plugin-c'],
            },
            <String, dynamic>{
              'name': 'plugin-b',
              'dependencies': <String>['plugin-c'],
            },
            <String, dynamic>{'name': 'plugin-c', 'dependencies': <String>[]},
          ];

          expect(jsonContent['dependencyGraph'], expectedDependencyGraph);
          expect(jsonContent['date_created'], dateCreated.toString());
          expect(jsonContent['version'], '1.0.0');

          final Map<String, dynamic> expectedSwiftPackageManagerEnabled = <String, bool>{
            'ios': false,
            'macos': false,
          };
          expect(jsonContent['swift_package_manager_enabled'], expectedSwiftPackageManagerEnabled);

          // Make sure tests are updated if a new object is added/removed.
          final List<String> expectedKeys = <String>[
            'info',
            'plugins',
            'dependencyGraph',
            'date_created',
            'version',
            'swift_package_manager_enabled',
          ];
          expect(jsonContent.keys, expectedKeys);
        },
        overrides: <Type, Generator>{
          FileSystem: () => fs,
          ProcessManager: () => FakeProcessManager.any(),
          SystemClock: () => systemClock,
          FlutterVersion: () => flutterVersion,
          FeatureFlags: disableExplicitPackageDependencies,
        },
      );

      testUsingContext(
        '.flutter-plugins-dependencies contains plugin platform info',
        () async {
          createPlugin(
            name: 'plugin-a',
            platforms: const <String, _PluginPlatformInfo>{
              // Native-only; should include native build.
              'android': _PluginPlatformInfo(pluginClass: 'Foo', androidPackage: 'bar.foo'),
              // Hybrid native and Dart; should include native build.
              'ios': _PluginPlatformInfo(
                pluginClass: 'Foo',
                dartPluginClass: 'Bar',
                sharedDarwinSource: true,
              ),
              // Web; should not have the native build key at all since it doesn't apply.
              'web': _PluginPlatformInfo(pluginClass: 'Foo', fileName: 'lib/foo.dart'),
              // Dart-only; should not include native build.
              'windows': _PluginPlatformInfo(dartPluginClass: 'Foo'),
            },
          );
          iosProject.testExists = true;

          final DateTime dateCreated = DateTime(1970);
          systemClock.currentTime = dateCreated;

          await refreshPluginsList(flutterProject);

          expect(flutterProject.flutterPluginsDependenciesFile.existsSync(), true);
          final String pluginsString =
              flutterProject.flutterPluginsDependenciesFile.readAsStringSync();
          final Map<String, dynamic> jsonContent =
              json.decode(pluginsString) as Map<String, dynamic>;
          final Map<String, dynamic>? actualPlugins =
              jsonContent['plugins'] as Map<String, dynamic>?;

          final Map<String, Object> expectedPlugins = <String, Object>{
            'ios': <Map<String, Object>>[
              <String, Object>{
                'name': 'plugin-a',
                'path': '/.tmp_rand0/flutter_plugin.rand0/',
                'shared_darwin_source': true,
                'native_build': true,
                'dependencies': <String>[],
                'dev_dependency': false,
              },
            ],
            'android': <Map<String, Object>>[
              <String, Object>{
                'name': 'plugin-a',
                'path': '/.tmp_rand0/flutter_plugin.rand0/',
                'native_build': true,
                'dependencies': <String>[],
                'dev_dependency': false,
              },
            ],
            'macos': <Map<String, Object>>[],
            'linux': <Map<String, Object>>[],
            'windows': <Map<String, Object>>[
              <String, Object>{
                'name': 'plugin-a',
                'path': '/.tmp_rand0/flutter_plugin.rand0/',
                'native_build': false,
                'dependencies': <String>[],
                'dev_dependency': false,
              },
            ],
            'web': <Map<String, Object>>[
              <String, Object>{
                'name': 'plugin-a',
                'path': '/.tmp_rand0/flutter_plugin.rand0/',
                'dependencies': <String>[],
                'dev_dependency': false,
              },
            ],
          };
          expect(actualPlugins, expectedPlugins);
        },
        overrides: <Type, Generator>{
          FileSystem: () => fs,
          ProcessManager: () => FakeProcessManager.any(),
          SystemClock: () => systemClock,
          FlutterVersion: () => flutterVersion,
          FeatureFlags: enableExplicitPackageDependencies,
          Pub: FakePubWithPrimedDeps.new,
        },
      );

      testUsingContext(
        '.flutter-plugins-dependencies contains swift_package_manager_enabled true when project is using Swift Package Manager',
        () async {
          createPlugin(
            name: 'plugin-a',
            platforms: const <String, _PluginPlatformInfo>{
              // Native-only; should include native build.
              'android': _PluginPlatformInfo(pluginClass: 'Foo', androidPackage: 'bar.foo'),
              // Hybrid native and Dart; should include native build.
              'ios': _PluginPlatformInfo(
                pluginClass: 'Foo',
                dartPluginClass: 'Bar',
                sharedDarwinSource: true,
              ),
              // Web; should not have the native build key at all since it doesn't apply.
              'web': _PluginPlatformInfo(pluginClass: 'Foo', fileName: 'lib/foo.dart'),
              // Dart-only; should not include native build.
              'windows': _PluginPlatformInfo(dartPluginClass: 'Foo'),
            },
          );
          iosProject.testExists = true;

          final DateTime dateCreated = DateTime(1970);
          systemClock.currentTime = dateCreated;

          iosProject.usesSwiftPackageManager = true;
          macosProject.usesSwiftPackageManager = true;

          await refreshPluginsList(flutterProject, iosPlatform: true, macOSPlatform: true);

          expect(flutterProject.flutterPluginsDependenciesFile.existsSync(), true);
          final String pluginsString =
              flutterProject.flutterPluginsDependenciesFile.readAsStringSync();
          final Map<String, dynamic> jsonContent =
              json.decode(pluginsString) as Map<String, dynamic>;

          final Map<String, dynamic> expectedSwiftPackageManagerEnabled = <String, dynamic>{
            'ios': true,
            'macos': true,
          };
          expect(jsonContent['swift_package_manager_enabled'], expectedSwiftPackageManagerEnabled);
        },
        overrides: <Type, Generator>{
          FileSystem: () => fs,
          ProcessManager: () => FakeProcessManager.any(),
          SystemClock: () => systemClock,
          FlutterVersion: () => flutterVersion,
          FeatureFlags: enableExplicitPackageDependencies,
          Pub: FakePubWithPrimedDeps.new,
        },
      );

      testUsingContext(
        '.flutter-plugins-dependencies contains swift_package_manager_enabled false when project is using Swift Package Manager but forceCocoaPodsOnly is true',
        () async {
          createPlugin(
            name: 'plugin-a',
            platforms: const <String, _PluginPlatformInfo>{
              // Native-only; should include native build.
              'android': _PluginPlatformInfo(pluginClass: 'Foo', androidPackage: 'bar.foo'),
              // Hybrid native and Dart; should include native build.
              'ios': _PluginPlatformInfo(
                pluginClass: 'Foo',
                dartPluginClass: 'Bar',
                sharedDarwinSource: true,
              ),
              // Web; should not have the native build key at all since it doesn't apply.
              'web': _PluginPlatformInfo(pluginClass: 'Foo', fileName: 'lib/foo.dart'),
              // Dart-only; should not include native build.
              'windows': _PluginPlatformInfo(dartPluginClass: 'Foo'),
            },
          );
          iosProject.testExists = true;

          final DateTime dateCreated = DateTime(1970);
          systemClock.currentTime = dateCreated;

          iosProject.usesSwiftPackageManager = true;
          macosProject.usesSwiftPackageManager = true;

          await refreshPluginsList(flutterProject, forceCocoaPodsOnly: true);

          expect(flutterProject.flutterPluginsDependenciesFile.existsSync(), true);
          final String pluginsString =
              flutterProject.flutterPluginsDependenciesFile.readAsStringSync();
          final Map<String, dynamic> jsonContent =
              json.decode(pluginsString) as Map<String, dynamic>;

          final Map<String, dynamic> expectedSwiftPackageManagerEnabled = <String, dynamic>{
            'ios': false,
            'macos': false,
          };
          expect(jsonContent['swift_package_manager_enabled'], expectedSwiftPackageManagerEnabled);
        },
        overrides: <Type, Generator>{
          FileSystem: () => fs,
          ProcessManager: () => FakeProcessManager.any(),
          SystemClock: () => systemClock,
          FlutterVersion: () => flutterVersion,
          FeatureFlags: enableExplicitPackageDependencies,
          Pub: FakePubWithPrimedDeps.new,
        },
      );

      testUsingContext(
        '.flutter-plugins-dependencies can have different swift_package_manager_enabled values for iOS and macoS',
        () async {
          createPlugin(
            name: 'plugin-a',
            platforms: const <String, _PluginPlatformInfo>{
              // Native-only; should include native build.
              'android': _PluginPlatformInfo(pluginClass: 'Foo', androidPackage: 'bar.foo'),
              // Hybrid native and Dart; should include native build.
              'ios': _PluginPlatformInfo(
                pluginClass: 'Foo',
                dartPluginClass: 'Bar',
                sharedDarwinSource: true,
              ),
              // Web; should not have the native build key at all since it doesn't apply.
              'web': _PluginPlatformInfo(pluginClass: 'Foo', fileName: 'lib/foo.dart'),
              // Dart-only; should not include native build.
              'windows': _PluginPlatformInfo(dartPluginClass: 'Foo'),
            },
          );
          iosProject.testExists = true;

          final DateTime dateCreated = DateTime(1970);
          systemClock.currentTime = dateCreated;

          iosProject.usesSwiftPackageManager = true;
          macosProject.usesSwiftPackageManager = false;

          await refreshPluginsList(flutterProject, iosPlatform: true, macOSPlatform: true);

          expect(flutterProject.flutterPluginsDependenciesFile.existsSync(), true);
          final String pluginsString =
              flutterProject.flutterPluginsDependenciesFile.readAsStringSync();
          final Map<String, dynamic> jsonContent =
              json.decode(pluginsString) as Map<String, dynamic>;

          final Map<String, dynamic> expectedSwiftPackageManagerEnabled = <String, dynamic>{
            'ios': true,
            'macos': false,
          };
          expect(jsonContent['swift_package_manager_enabled'], expectedSwiftPackageManagerEnabled);
        },
        overrides: <Type, Generator>{
          FileSystem: () => fs,
          ProcessManager: () => FakeProcessManager.any(),
          SystemClock: () => systemClock,
          FlutterVersion: () => flutterVersion,
          FeatureFlags: enableExplicitPackageDependencies,
          Pub: FakePubWithPrimedDeps.new,
        },
      );

      testUsingContext(
        'Changes to the plugin list invalidates the Cocoapod lockfiles',
        () async {
          simulatePodInstallRun(iosProject);
          simulatePodInstallRun(macosProject);
          createFakePlugin(fs);
          iosProject.testExists = true;
          macosProject.exists = true;

          await refreshPluginsList(flutterProject, iosPlatform: true, macOSPlatform: true);
          expect(iosProject.podManifestLock.existsSync(), false);
          expect(macosProject.podManifestLock.existsSync(), false);
        },
        overrides: <Type, Generator>{
          FileSystem: () => fs,
          ProcessManager: () => FakeProcessManager.any(),
          SystemClock: () => systemClock,
          FlutterVersion: () => flutterVersion,
          FeatureFlags: enableExplicitPackageDependencies,
          Pub: FakePubWithPrimedDeps.new,
        },
      );

      testUsingContext(
        'No changes to the plugin list does not invalidate the Cocoapod lockfiles',
        () async {
          createFakePlugin(fs);
          iosProject.testExists = true;
          macosProject.exists = true;

          // First call will create the .flutter-plugins-dependencies and the legacy .flutter-plugins file.
          // Since there was no plugins list, the lock files will be invalidated.
          // The second call is where the plugins list is compared to the existing one, and if there is no change,
          // the podfiles shouldn't be invalidated.
          await refreshPluginsList(flutterProject, iosPlatform: true, macOSPlatform: true);
          simulatePodInstallRun(iosProject);
          simulatePodInstallRun(macosProject);

          await refreshPluginsList(flutterProject);
          expect(iosProject.podManifestLock.existsSync(), true);
          expect(macosProject.podManifestLock.existsSync(), true);
        },
        overrides: <Type, Generator>{
          FileSystem: () => fs,
          ProcessManager: () => FakeProcessManager.any(),
          SystemClock: () => systemClock,
          FlutterVersion: () => flutterVersion,
          FeatureFlags: enableExplicitPackageDependencies,
          Pub: FakePubWithPrimedDeps.new,
        },
      );
    });

    group('injectPlugins', () {
      FakeXcodeProjectInterpreter? xcodeProjectInterpreter;

      setUp(() {
        xcodeProjectInterpreter = FakeXcodeProjectInterpreter();
      });

      testUsingContext(
        'Registrant uses new embedding if app uses new embedding',
        () async {
          androidProject.embeddingVersion = AndroidEmbeddingVersion.v2;

          await injectPlugins(flutterProject, androidPlatform: true);

          final File registrant = flutterProject.directory
              .childDirectory(
                fs.path.join('android', 'app', 'src', 'main', 'java', 'io', 'flutter', 'plugins'),
              )
              .childFile('GeneratedPluginRegistrant.java');

          expect(registrant.existsSync(), isTrue);
          expect(registrant.readAsStringSync(), contains('package io.flutter.plugins'));
          expect(registrant.readAsStringSync(), contains('class GeneratedPluginRegistrant'));
          expect(
            registrant.readAsStringSync(),
            contains('public static void registerWith(@NonNull FlutterEngine flutterEngine)'),
          );
        },
        overrides: <Type, Generator>{
          FileSystem: () => fs,
          ProcessManager: () => FakeProcessManager.any(),
          FeatureFlags: enableExplicitPackageDependencies,
          Pub: FakePubWithPrimedDeps.new,
        },
      );

      // Issue: https://github.com/flutter/flutter/issues/47803
      testUsingContext(
        'exits the tool if a plugin sets an invalid android package in pubspec.yaml',
        () async {
          androidProject.embeddingVersion = AndroidEmbeddingVersion.v1;

          final Directory pluginDir = createPluginWithInvalidAndroidPackage();

          await expectLater(
            () async {
              await injectPlugins(flutterProject, androidPlatform: true);
            },
            throwsToolExit(
              message:
                  "The plugin `plugin1` doesn't have a main class defined in "
                  '${pluginDir.path}/android/src/main/java/plugin1/invalid/UseNewEmbedding.java or '
                  '${pluginDir.path}/android/src/main/kotlin/plugin1/invalid/UseNewEmbedding.kt. '
                  "This is likely to due to an incorrect `androidPackage: plugin1.invalid` or `mainClass` entry in the plugin's pubspec.yaml.\n"
                  'If you are the author of this plugin, fix the `androidPackage` entry or move the main class to any of locations used above. '
                  'Otherwise, please contact the author of this plugin and consider using a different plugin in the meanwhile.',
            ),
          );
        },
        overrides: <Type, Generator>{
          FileSystem: () => fs,
          ProcessManager: () => FakeProcessManager.any(),
          XcodeProjectInterpreter: () => xcodeProjectInterpreter,
          FeatureFlags: enableExplicitPackageDependencies,
          Pub: FakePubWithPrimedDeps.new,
        },
      );

      testUsingContext(
        'new embedding app uses a plugin that supports v1 and v2 embedding',
        () async {
          androidProject.embeddingVersion = AndroidEmbeddingVersion.v2;

          createDualSupportJavaPlugin4();

          await injectPlugins(flutterProject, androidPlatform: true);

          final File registrant = flutterProject.directory
              .childDirectory(
                fs.path.join('android', 'app', 'src', 'main', 'java', 'io', 'flutter', 'plugins'),
              )
              .childFile('GeneratedPluginRegistrant.java');

          expect(registrant.existsSync(), isTrue);
          expect(registrant.readAsStringSync(), contains('package io.flutter.plugins'));
          expect(registrant.readAsStringSync(), contains('class GeneratedPluginRegistrant'));
          expect(
            registrant.readAsStringSync(),
            contains('flutterEngine.getPlugins().add(new plugin4.UseBothEmbedding());'),
          );
        },
        overrides: <Type, Generator>{
          FileSystem: () => fs,
          ProcessManager: () => FakeProcessManager.any(),
          XcodeProjectInterpreter: () => xcodeProjectInterpreter,
          FeatureFlags: enableExplicitPackageDependencies,
          Pub: FakePubWithPrimedDeps.new,
        },
      );

      testUsingContext(
        'Modules use new embedding',
        () async {
          flutterProject.isModule = true;
          androidProject.embeddingVersion = AndroidEmbeddingVersion.v2;

          await injectPlugins(flutterProject, androidPlatform: true);

          final File registrant = flutterProject.directory
              .childDirectory(
                fs.path.join('android', 'app', 'src', 'main', 'java', 'io', 'flutter', 'plugins'),
              )
              .childFile('GeneratedPluginRegistrant.java');

          expect(registrant.existsSync(), isTrue);
          expect(registrant.readAsStringSync(), contains('package io.flutter.plugins'));
          expect(registrant.readAsStringSync(), contains('class GeneratedPluginRegistrant'));
          expect(
            registrant.readAsStringSync(),
            contains('public static void registerWith(@NonNull FlutterEngine flutterEngine)'),
          );
        },
        overrides: <Type, Generator>{
          FileSystem: () => fs,
          ProcessManager: () => FakeProcessManager.any(),
          FeatureFlags: enableExplicitPackageDependencies,
          Pub: FakePubWithPrimedDeps.new,
        },
      );

      testUsingContext(
        'Module using new plugin shows no warnings',
        () async {
          flutterProject.isModule = true;
          androidProject.embeddingVersion = AndroidEmbeddingVersion.v2;

          createNewJavaPlugin1();

          await injectPlugins(flutterProject, androidPlatform: true);

          final File registrant = flutterProject.directory
              .childDirectory(
                fs.path.join('android', 'app', 'src', 'main', 'java', 'io', 'flutter', 'plugins'),
              )
              .childFile('GeneratedPluginRegistrant.java');
          expect(
            registrant.readAsStringSync(),
            contains('flutterEngine.getPlugins().add(new plugin1.UseNewEmbedding());'),
          );

          expect(testLogger.errorText, isNot(contains('go/android-plugin-migration')));
        },
        overrides: <Type, Generator>{
          FileSystem: () => fs,
          ProcessManager: () => FakeProcessManager.any(),
          XcodeProjectInterpreter: () => xcodeProjectInterpreter,
          FeatureFlags: enableExplicitPackageDependencies,
          Pub: FakePubWithPrimedDeps.new,
        },
      );

      testUsingContext(
        'Module using plugin with v1 and v2 support shows no warning',
        () async {
          flutterProject.isModule = true;
          androidProject.embeddingVersion = AndroidEmbeddingVersion.v2;

          createDualSupportJavaPlugin4();

          await injectPlugins(flutterProject, androidPlatform: true);

          final File registrant = flutterProject.directory
              .childDirectory(
                fs.path.join('android', 'app', 'src', 'main', 'java', 'io', 'flutter', 'plugins'),
              )
              .childFile('GeneratedPluginRegistrant.java');
          expect(
            registrant.readAsStringSync(),
            contains('flutterEngine.getPlugins().add(new plugin4.UseBothEmbedding());'),
          );

          expect(testLogger.errorText, isNot(contains('go/android-plugin-migration')));
        },
        overrides: <Type, Generator>{
          FileSystem: () => fs,
          ProcessManager: () => FakeProcessManager.any(),
          XcodeProjectInterpreter: () => xcodeProjectInterpreter,
          FeatureFlags: enableExplicitPackageDependencies,
          Pub: FakePubWithPrimedDeps.new,
        },
      );

      testUsingContext(
        'App using plugin with v1 and v2 support shows no warning',
        () async {
          flutterProject.isModule = false;
          androidProject.embeddingVersion = AndroidEmbeddingVersion.v2;

          createDualSupportJavaPlugin4();

          await injectPlugins(flutterProject, androidPlatform: true);

          final File registrant = flutterProject.directory
              .childDirectory(
                fs.path.join('android', 'app', 'src', 'main', 'java', 'io', 'flutter', 'plugins'),
              )
              .childFile('GeneratedPluginRegistrant.java');
          expect(
            registrant.readAsStringSync(),
            contains('flutterEngine.getPlugins().add(new plugin4.UseBothEmbedding());'),
          );

          expect(testLogger.errorText, isNot(contains('go/android-plugin-migration')));
        },
        overrides: <Type, Generator>{
          FileSystem: () => fs,
          ProcessManager: () => FakeProcessManager.any(),
          XcodeProjectInterpreter: () => xcodeProjectInterpreter,
          FeatureFlags: enableExplicitPackageDependencies,
          Pub: FakePubWithPrimedDeps.new,
        },
      );

      testUsingContext(
        'Does not throw when AndroidManifest.xml is not found',
        () async {
          final File manifest = fs.file('AndroidManifest.xml');
          androidProject.appManifestFile = manifest;
          await injectPlugins(flutterProject, androidPlatform: true);
        },
        overrides: <Type, Generator>{
          FileSystem: () => fs,
          ProcessManager: () => FakeProcessManager.any(),
          FeatureFlags: enableExplicitPackageDependencies,
          Pub: FakePubWithPrimedDeps.new,
        },
      );

      group('Build time plugin injection', () {
        testUsingContext(
          "Registrant for web doesn't escape slashes in imports",
          () async {
            flutterProject.isModule = true;
            final Directory webPluginWithNestedFile = fs.systemTempDirectory.createTempSync(
              'flutter_web_plugin_with_nested.',
            );
            webPluginWithNestedFile.childFile('pubspec.yaml').writeAsStringSync('''
  flutter:
    plugin:
      platforms:
        web:
          pluginClass: WebPlugin
          fileName: src/web_plugin.dart
  ''');
            webPluginWithNestedFile
                .childDirectory('lib')
                .childDirectory('src')
                .childFile('web_plugin.dart')
                .createSync(recursive: true);

            addToPackageConfig('web_plugin_with_nested', webPluginWithNestedFile);

            final Directory destination = flutterProject.directory.childDirectory('lib');
            await injectBuildTimePluginFilesForWebPlatform(
              flutterProject,
              destination: destination,
            );

            final File registrant = flutterProject.directory
                .childDirectory('lib')
                .childFile('web_plugin_registrant.dart');

            expect(registrant.existsSync(), isTrue);
            expect(
              registrant.readAsStringSync(),
              contains("import 'package:web_plugin_with_nested/src/web_plugin.dart';"),
            );
          },
          overrides: <Type, Generator>{
            FileSystem: () => fs,
            ProcessManager: () => FakeProcessManager.any(),
            FeatureFlags: enableExplicitPackageDependencies,
            Pub: FakePubWithPrimedDeps.new,
          },
        );

        testUsingContext(
          'user-selected implementation overrides inline implementation on web',
          () async {
            final List<Directory> directories = createFakePlugins(fs, <String>[
              'user_selected_url_launcher_implementation',
              'url_launcher',
            ]);

            // Add inline web implementation to `user_selected_url_launcher_implementation`
            directories[0].childFile('pubspec.yaml').writeAsStringSync('''
flutter:
  plugin:
    implements: url_launcher
    platforms:
      web:
        pluginClass: UserSelectedUrlLauncherWeb
        fileName: src/web_plugin.dart
    ''');

            // Add inline native implementation to `url_launcher`
            directories[1].childFile('pubspec.yaml').writeAsStringSync('''
flutter:
  plugin:
    platforms:
      web:
        pluginClass: InlineUrlLauncherWeb
        fileName: src/web_plugin.dart
    ''');

            final FlutterManifest manifest =
                FlutterManifest.createFromString('''
name: test
version: 1.0.0

dependencies:
  url_launcher: ^1.0.0
  user_selected_url_launcher_implementation: ^1.0.0
    ''', logger: BufferLogger.test())!;

            flutterProject.manifest = manifest;
            flutterProject.isModule = true;

            final Directory destination = flutterProject.directory.childDirectory('lib');
            await injectBuildTimePluginFilesForWebPlatform(
              flutterProject,
              destination: destination,
            );

            final File registrant = flutterProject.directory
                .childDirectory('lib')
                .childFile('web_plugin_registrant.dart');

            expect(registrant.existsSync(), isTrue);
            expect(
              registrant.readAsStringSync(),
              contains(
                "import 'package:user_selected_url_launcher_implementation/src/web_plugin.dart';",
              ),
            );
            expect(
              registrant.readAsStringSync(),
              isNot(contains("import 'package:url_launcher/src/web_plugin.dart';")),
            );
          },
          overrides: <Type, Generator>{
            FileSystem: () => fs,
            ProcessManager: () => FakeProcessManager.any(),
            FeatureFlags: enableExplicitPackageDependencies,
            Pub: FakePubWithPrimedDeps.new,
          },
        );
      });

      testUsingContext(
        'Injecting creates generated Android registrant, but does not include Dart-only plugins',
        () async {
          // Create a plugin without a pluginClass.
          final Directory pluginDirectory = createFakePlugin(fs);
          pluginDirectory.childFile('pubspec.yaml').writeAsStringSync('''
flutter:
  plugin:
    platforms:
      android:
        dartPluginClass: SomePlugin
    ''');

          await injectPlugins(flutterProject, androidPlatform: true);

          final File registrantFile = androidProject.pluginRegistrantHost
              .childDirectory(fs.path.join('src', 'main', 'java', 'io', 'flutter', 'plugins'))
              .childFile('GeneratedPluginRegistrant.java');

          expect(registrantFile, exists);
          expect(registrantFile, isNot(contains('SomePlugin')));
        },
        overrides: <Type, Generator>{
          FileSystem: () => fs,
          ProcessManager: () => FakeProcessManager.any(),
          FeatureFlags: enableExplicitPackageDependencies,
          Pub: FakePubWithPrimedDeps.new,
        },
      );

      testUsingContext(
        'Injecting creates generated iOS registrant, but does not include Dart-only plugins',
        () async {
          flutterProject.isModule = true;
          // Create a plugin without a pluginClass.
          final Directory pluginDirectory = createFakePlugin(fs);
          pluginDirectory.childFile('pubspec.yaml').writeAsStringSync('''
flutter:
  plugin:
    platforms:
      ios:
        dartPluginClass: SomePlugin
    ''');
          final FakeDarwinDependencyManagement dependencyManagement =
              FakeDarwinDependencyManagement();
          await injectPlugins(
            flutterProject,
            iosPlatform: true,
            darwinDependencyManagement: dependencyManagement,
          );

          final File registrantFile = iosProject.pluginRegistrantImplementation;

          expect(registrantFile, exists);
          expect(registrantFile, isNot(contains('SomePlugin')));
        },
        overrides: <Type, Generator>{
          FileSystem: () => fs,
          ProcessManager: () => FakeProcessManager.any(),
          FeatureFlags: enableExplicitPackageDependencies,
          Pub: FakePubWithPrimedDeps.new,
        },
      );

      testUsingContext(
        'Injecting creates generated macos registrant, but does not include Dart-only plugins',
        () async {
          flutterProject.isModule = true;
          // Create a plugin without a pluginClass.
          final Directory pluginDirectory = createFakePlugin(fs);
          pluginDirectory.childFile('pubspec.yaml').writeAsStringSync('''
flutter:
  plugin:
    platforms:
      macos:
        dartPluginClass: SomePlugin
    ''');
          final FakeDarwinDependencyManagement dependencyManagement =
              FakeDarwinDependencyManagement();
          await injectPlugins(
            flutterProject,
            macOSPlatform: true,
            darwinDependencyManagement: dependencyManagement,
          );

          final File registrantFile = macosProject.managedDirectory.childFile(
            'GeneratedPluginRegistrant.swift',
          );

          expect(registrantFile, exists);
          expect(registrantFile, isNot(contains('SomePlugin')));
        },
        overrides: <Type, Generator>{
          FileSystem: () => fs,
          ProcessManager: () => FakeProcessManager.any(),
          FeatureFlags: enableExplicitPackageDependencies,
          Pub: FakePubWithPrimedDeps.new,
        },
      );

      testUsingContext(
        "pluginClass: none doesn't trigger registrant entry on macOS",
        () async {
          flutterProject.isModule = true;
          // Create a plugin without a pluginClass.
          final Directory pluginDirectory = createFakePlugin(fs);
          pluginDirectory.childFile('pubspec.yaml').writeAsStringSync('''
flutter:
  plugin:
    platforms:
      macos:
        pluginClass: none
        dartPluginClass: SomePlugin
    ''');
          final FakeDarwinDependencyManagement dependencyManagement =
              FakeDarwinDependencyManagement();
          await injectPlugins(
            flutterProject,
            macOSPlatform: true,
            darwinDependencyManagement: dependencyManagement,
          );

          final File registrantFile = macosProject.managedDirectory.childFile(
            'GeneratedPluginRegistrant.swift',
          );

          expect(registrantFile, exists);
          expect(registrantFile, isNot(contains('SomePlugin')));
          expect(registrantFile, isNot(contains('none')));
        },
        overrides: <Type, Generator>{
          FileSystem: () => fs,
          ProcessManager: () => FakeProcessManager.any(),
          FeatureFlags: enableExplicitPackageDependencies,
          Pub: FakePubWithPrimedDeps.new,
        },
      );

      testUsingContext(
        'Invalid yaml does not crash plugin lookup.',
        () async {
          flutterProject.isModule = true;
          // Create a plugin without a pluginClass.
          final Directory pluginDirectory = createFakePlugin(fs);
          pluginDirectory.childFile('pubspec.yaml').writeAsStringSync(r'''
"aws ... \"Branch\": $BITBUCKET_BRANCH, \"Date\": $(date +"%m-%d-%y"), \"Time\": $(date +"%T")}\"
    ''');
          final FakeDarwinDependencyManagement dependencyManagement =
              FakeDarwinDependencyManagement();
          await injectPlugins(
            flutterProject,
            macOSPlatform: true,
            darwinDependencyManagement: dependencyManagement,
          );

          final File registrantFile = macosProject.managedDirectory.childFile(
            'GeneratedPluginRegistrant.swift',
          );

          expect(registrantFile, exists);
        },
        overrides: <Type, Generator>{
          FileSystem: () => fs,
          ProcessManager: () => FakeProcessManager.any(),
          FeatureFlags: enableExplicitPackageDependencies,
          Pub: FakePubWithPrimedDeps.new,
        },
      );

      testUsingContext(
        'Injecting creates generated Linux registrant',
        () async {
          createFakePlugin(fs);

          await injectPlugins(flutterProject, linuxPlatform: true);

          final File registrantHeader = linuxProject.managedDirectory.childFile(
            'generated_plugin_registrant.h',
          );
          final File registrantImpl = linuxProject.managedDirectory.childFile(
            'generated_plugin_registrant.cc',
          );

          expect(registrantHeader.existsSync(), isTrue);
          expect(registrantImpl.existsSync(), isTrue);
          expect(
            registrantImpl.readAsStringSync(),
            contains('some_plugin_register_with_registrar'),
          );
        },
        overrides: <Type, Generator>{
          FileSystem: () => fs,
          ProcessManager: () => FakeProcessManager.any(),
          FeatureFlags: enableExplicitPackageDependencies,
          Pub: FakePubWithPrimedDeps.new,
        },
      );

      testUsingContext(
        'user-selected implementation overrides inline implementation',
        () async {
          final List<Directory> directories = createFakePlugins(fs, <String>[
            'user_selected_url_launcher_implementation',
            'url_launcher',
          ]);

          // Add inline native implementation to `user_selected_url_launcher_implementation`
          directories[0].childFile('pubspec.yaml').writeAsStringSync('''
flutter:
  plugin:
    implements: url_launcher
    platforms:
      linux:
        pluginClass: UserSelectedUrlLauncherLinux
    ''');

          // Add inline native implementation to `url_launcher`
          directories[1].childFile('pubspec.yaml').writeAsStringSync('''
flutter:
  plugin:
    platforms:
      linux:
        pluginClass: InlineUrlLauncherLinux
    ''');

          final FlutterManifest manifest =
              FlutterManifest.createFromString('''
name: test
version: 1.0.0

dependencies:
  url_launcher: ^1.0.0
  user_selected_url_launcher_implementation: ^1.0.0
    ''', logger: BufferLogger.test())!;

          flutterProject.manifest = manifest;

          await injectPlugins(flutterProject, linuxPlatform: true);

          final File registrantImpl = linuxProject.managedDirectory.childFile(
            'generated_plugin_registrant.cc',
          );

          expect(registrantImpl.existsSync(), isTrue);
          expect(
            registrantImpl.readAsStringSync(),
            contains('user_selected_url_launcher_linux_register_with_registrar'),
          );
          expect(
            registrantImpl.readAsStringSync(),
            isNot(contains('inline_url_launcher_linux_register_with_registrar')),
          );
        },
        overrides: <Type, Generator>{
          FileSystem: () => fs,
          ProcessManager: () => FakeProcessManager.any(),
          FeatureFlags: enableExplicitPackageDependencies,
          Pub: FakePubWithPrimedDeps.new,
        },
      );

      testUsingContext(
        'user-selected implementation overrides default implementation',
        () async {
          final List<Directory> directories = createFakePlugins(fs, <String>[
            'user_selected_url_launcher_implementation',
            'url_launcher',
            'url_launcher_linux',
          ]);

          // Add inline native implementation to `user_selected_url_launcher_implementation`
          directories[0].childFile('pubspec.yaml').writeAsStringSync('''
flutter:
  plugin:
    implements: url_launcher
    platforms:
      linux:
        pluginClass: UserSelectedUrlLauncherLinux
    ''');

          // Add default native implementation to `url_launcher`
          directories[1].childFile('pubspec.yaml').writeAsStringSync('''
flutter:
  plugin:
    platforms:
      linux:
        default_package: url_launcher_linux
    ''');

          // Add inline native implementation to `url_launcher_linux`
          directories[1].childFile('pubspec.yaml').writeAsStringSync('''
flutter:
  plugin:
    platforms:
      linux:
        pluginClass: InlineUrlLauncherLinux
    ''');

          final FlutterManifest manifest =
              FlutterManifest.createFromString('''
name: test
version: 1.0.0

dependencies:
  url_launcher: ^1.0.0
  user_selected_url_launcher_implementation: ^1.0.0
    ''', logger: BufferLogger.test())!;

          flutterProject.manifest = manifest;

          await injectPlugins(flutterProject, linuxPlatform: true);

          final File registrantImpl = linuxProject.managedDirectory.childFile(
            'generated_plugin_registrant.cc',
          );

          expect(registrantImpl.existsSync(), isTrue);
          expect(
            registrantImpl.readAsStringSync(),
            contains('user_selected_url_launcher_linux_register_with_registrar'),
          );
          expect(
            registrantImpl.readAsStringSync(),
            isNot(contains('inline_url_launcher_linux_register_with_registrar')),
          );
        },
        overrides: <Type, Generator>{
          FileSystem: () => fs,
          ProcessManager: () => FakeProcessManager.any(),
          FeatureFlags: enableExplicitPackageDependencies,
          Pub: FakePubWithPrimedDeps.new,
        },
      );

      testUsingContext(
        'Injecting creates generated Linux registrant, but does not include Dart-only plugins',
        () async {
          // Create a plugin without a pluginClass.
          final Directory pluginDirectory = createFakePlugin(fs);
          pluginDirectory.childFile('pubspec.yaml').writeAsStringSync('''
flutter:
  plugin:
    platforms:
      linux:
        dartPluginClass: SomePlugin
    ''');

          await injectPlugins(flutterProject, linuxPlatform: true);

          final File registrantImpl = linuxProject.managedDirectory.childFile(
            'generated_plugin_registrant.cc',
          );

          expect(registrantImpl, exists);
          expect(registrantImpl, isNot(contains('SomePlugin')));
          expect(registrantImpl, isNot(contains('some_plugin')));
        },
        overrides: <Type, Generator>{
          FileSystem: () => fs,
          ProcessManager: () => FakeProcessManager.any(),
          FeatureFlags: enableExplicitPackageDependencies,
          Pub: FakePubWithPrimedDeps.new,
        },
      );

      testUsingContext(
        "pluginClass: none doesn't trigger registrant entry on Linux",
        () async {
          // Create a plugin without a pluginClass.
          final Directory pluginDirectory = createFakePlugin(fs);
          pluginDirectory.childFile('pubspec.yaml').writeAsStringSync('''
flutter:
  plugin:
    platforms:
      linux:
        pluginClass: none
        dartPluginClass: SomePlugin
    ''');

          await injectPlugins(flutterProject, linuxPlatform: true);

          final File registrantImpl = linuxProject.managedDirectory.childFile(
            'generated_plugin_registrant.cc',
          );

          expect(registrantImpl, exists);
          expect(registrantImpl, isNot(contains('SomePlugin')));
          expect(registrantImpl, isNot(contains('none')));
        },
        overrides: <Type, Generator>{
          FileSystem: () => fs,
          ProcessManager: () => FakeProcessManager.any(),
          FeatureFlags: enableExplicitPackageDependencies,
          Pub: FakePubWithPrimedDeps.new,
        },
      );

      testUsingContext(
        'Injecting creates generated Linux plugin Cmake file',
        () async {
          createFakePlugin(fs);

          await injectPlugins(flutterProject, linuxPlatform: true);

          final File pluginMakefile = linuxProject.generatedPluginCmakeFile;

          expect(pluginMakefile.existsSync(), isTrue);
          final String contents = pluginMakefile.readAsStringSync();
          expect(contents, contains('some_plugin'));
          expect(
            contents,
            contains(r'target_link_libraries(${BINARY_NAME} PRIVATE ${plugin}_plugin)'),
          );
          expect(
            contents,
            contains(r'list(APPEND PLUGIN_BUNDLED_LIBRARIES $<TARGET_FILE:${plugin}_plugin>)'),
          );
          expect(
            contents,
            contains(r'list(APPEND PLUGIN_BUNDLED_LIBRARIES ${${plugin}_bundled_libraries})'),
          );
        },
        overrides: <Type, Generator>{
          FileSystem: () => fs,
          ProcessManager: () => FakeProcessManager.any(),
          FeatureFlags: enableExplicitPackageDependencies,
          Pub: FakePubWithPrimedDeps.new,
        },
      );

      testUsingContext(
        'Generated Linux plugin files sorts by plugin name',
        () async {
          createFakePlugins(fs, <String>[
            'plugin_d',
            'plugin_a',
            '/local_plugins/plugin_c',
            '/local_plugins/plugin_b',
          ]);

          await injectPlugins(flutterProject, linuxPlatform: true);

          final File pluginCmakeFile = linuxProject.generatedPluginCmakeFile;
          final File pluginRegistrant = linuxProject.managedDirectory.childFile(
            'generated_plugin_registrant.cc',
          );
          for (final File file in <File>[pluginCmakeFile, pluginRegistrant]) {
            final String contents = file.readAsStringSync();
            expect(contents.indexOf('plugin_a'), lessThan(contents.indexOf('plugin_b')));
            expect(contents.indexOf('plugin_b'), lessThan(contents.indexOf('plugin_c')));
            expect(contents.indexOf('plugin_c'), lessThan(contents.indexOf('plugin_d')));
          }
        },
        overrides: <Type, Generator>{
          FileSystem: () => fs,
          ProcessManager: () => FakeProcessManager.any(),
          FeatureFlags: enableExplicitPackageDependencies,
          Pub: FakePubWithPrimedDeps.new,
        },
      );

      testUsingContext(
        'Injecting creates generated Windows registrant',
        () async {
          createFakePlugin(fs);

          await injectPlugins(flutterProject, windowsPlatform: true);

          final File registrantHeader = windowsProject.managedDirectory.childFile(
            'generated_plugin_registrant.h',
          );
          final File registrantImpl = windowsProject.managedDirectory.childFile(
            'generated_plugin_registrant.cc',
          );

          expect(registrantHeader.existsSync(), isTrue);
          expect(registrantImpl.existsSync(), isTrue);
          expect(registrantImpl.readAsStringSync(), contains('SomePluginRegisterWithRegistrar'));
        },
        overrides: <Type, Generator>{
          FileSystem: () => fs,
          ProcessManager: () => FakeProcessManager.any(),
          FeatureFlags: enableExplicitPackageDependencies,
          Pub: FakePubWithPrimedDeps.new,
        },
      );

      testUsingContext(
        'Injecting creates generated Windows registrant, but does not include Dart-only plugins',
        () async {
          // Create a plugin without a pluginClass.
          final Directory pluginDirectory = createFakePlugin(fs);
          pluginDirectory.childFile('pubspec.yaml').writeAsStringSync('''
flutter:
  plugin:
    platforms:
      windows:
        dartPluginClass: SomePlugin
    ''');

          await injectPlugins(flutterProject, windowsPlatform: true);

          final File registrantImpl = windowsProject.managedDirectory.childFile(
            'generated_plugin_registrant.cc',
          );

          expect(registrantImpl, exists);
          expect(registrantImpl, isNot(contains('SomePlugin')));
        },
        overrides: <Type, Generator>{
          FileSystem: () => fs,
          ProcessManager: () => FakeProcessManager.any(),
          FeatureFlags: enableExplicitPackageDependencies,
          Pub: FakePubWithPrimedDeps.new,
        },
      );

      testUsingContext(
        "pluginClass: none doesn't trigger registrant entry on Windows",
        () async {
          // Create a plugin without a pluginClass.
          final Directory pluginDirectory = createFakePlugin(fs);
          pluginDirectory.childFile('pubspec.yaml').writeAsStringSync('''
flutter:
  plugin:
    platforms:
      windows:
        pluginClass: none
        dartPluginClass: SomePlugin
    ''');

          await injectPlugins(flutterProject, windowsPlatform: true);

          final File registrantImpl = windowsProject.managedDirectory.childFile(
            'generated_plugin_registrant.cc',
          );

          expect(registrantImpl, exists);
          expect(registrantImpl, isNot(contains('SomePlugin')));
          expect(registrantImpl, isNot(contains('none')));
        },
        overrides: <Type, Generator>{
          FileSystem: () => fs,
          ProcessManager: () => FakeProcessManager.any(),
          FeatureFlags: enableExplicitPackageDependencies,
          Pub: FakePubWithPrimedDeps.new,
        },
      );

      testUsingContext(
        'Generated Windows plugin files sorts by plugin name',
        () async {
          createFakePlugins(fs, <String>[
            'plugin_d',
            'plugin_a',
            '/local_plugins/plugin_c',
            '/local_plugins/plugin_b',
          ]);

          await injectPlugins(flutterProject, windowsPlatform: true);

          final File pluginCmakeFile = windowsProject.generatedPluginCmakeFile;
          final File pluginRegistrant = windowsProject.managedDirectory.childFile(
            'generated_plugin_registrant.cc',
          );
          for (final File file in <File>[pluginCmakeFile, pluginRegistrant]) {
            final String contents = file.readAsStringSync();
            expect(contents.indexOf('plugin_a'), lessThan(contents.indexOf('plugin_b')));
            expect(contents.indexOf('plugin_b'), lessThan(contents.indexOf('plugin_c')));
            expect(contents.indexOf('plugin_c'), lessThan(contents.indexOf('plugin_d')));
          }
        },
        overrides: <Type, Generator>{
          FileSystem: () => fs,
          ProcessManager: () => FakeProcessManager.any(),
          FeatureFlags: enableExplicitPackageDependencies,
          Pub: FakePubWithPrimedDeps.new,
        },
      );

      testUsingContext(
        'Generated plugin CMake files always use posix-style paths',
        () async {
          // Re-run the setup using the Windows filesystem.
          setUpProject(fsWindows);
          createFakePlugin(fsWindows);

          await injectPlugins(flutterProject, linuxPlatform: true, windowsPlatform: true);

          for (final CmakeBasedProject? project in <CmakeBasedProject?>[
            linuxProject,
            windowsProject,
          ]) {
            final File pluginCmakefile = project!.generatedPluginCmakeFile;

            expect(pluginCmakefile.existsSync(), isTrue);
            final String contents = pluginCmakefile.readAsStringSync();
            expect(contents, contains('add_subdirectory(flutter/ephemeral/.plugin_symlinks'));
          }
        },
        overrides: <Type, Generator>{
          FileSystem: () => fsWindows,
          ProcessManager: () => FakeProcessManager.any(),
          FeatureFlags: enableExplicitPackageDependencies,
          Pub: FakePubWithPrimedDeps.new,
        },
      );

      testUsingContext(
        'injectPlugins will validate if all plugins in the project are part of the passed allowedPlugins',
        () async {
          // Re-run the setup using the Windows filesystem.
          setUpProject(fsWindows);
          createFakePlugins(fsWindows, const <String>['plugin_one', 'plugin_two']);

          expect(
            () => injectPlugins(
              flutterProject,
              linuxPlatform: true,
              windowsPlatform: true,
              allowedPlugins: PreviewDevice.supportedPubPlugins,
            ),
            throwsToolExit(
              message: '''
The Flutter Preview device does not support the following plugins from your pubspec.yaml:

[plugin_one, plugin_two]
''',
            ),
          );
        },
        overrides: <Type, Generator>{
          FileSystem: () => fsWindows,
          ProcessManager: () => FakeProcessManager.empty(),
          FeatureFlags: enableExplicitPackageDependencies,
          Pub: FakePubWithPrimedDeps.new,
        },
      );

      testUsingContext(
        'iOS and macOS project setup up Darwin Dependency Management',
        () async {
          final FakeDarwinDependencyManagement dependencyManagement =
              FakeDarwinDependencyManagement();
          await injectPlugins(
            flutterProject,
            iosPlatform: true,
            macOSPlatform: true,
            darwinDependencyManagement: dependencyManagement,
          );
          expect(dependencyManagement.setupPlatforms, <SupportedPlatform>[
            SupportedPlatform.ios,
            SupportedPlatform.macos,
          ]);
        },
        overrides: <Type, Generator>{
          FileSystem: () => fs,
          ProcessManager: () => FakeProcessManager.any(),
          FeatureFlags: enableExplicitPackageDependencies,
          Pub: FakePubWithPrimedDeps.new,
        },
      );

      testUsingContext(
        'non-iOS or macOS project does not setup up Darwin Dependency Management',
        () async {
          final FakeDarwinDependencyManagement dependencyManagement =
              FakeDarwinDependencyManagement();
          await injectPlugins(flutterProject, darwinDependencyManagement: dependencyManagement);
          expect(dependencyManagement.setupPlatforms, <SupportedPlatform>[]);
        },
        overrides: <Type, Generator>{
          FileSystem: () => fs,
          ProcessManager: () => FakeProcessManager.any(),
          FeatureFlags: enableExplicitPackageDependencies,
          Pub: FakePubWithPrimedDeps.new,
        },
      );
    });

    group('createPluginSymlinks', () {
      FeatureFlags? featureFlags;

      setUp(() {
        featureFlags = TestFeatureFlags(isLinuxEnabled: true, isWindowsEnabled: true);
      });

      testUsingContext(
        'Symlinks are created for Linux plugins',
        () async {
          linuxProject.exists = true;
          createFakePlugin(fs);
          // refreshPluginsList should call createPluginSymlinks.
          await refreshPluginsList(flutterProject);

          expect(linuxProject.pluginSymlinkDirectory.childLink('some_plugin').existsSync(), true);
        },
        overrides: <Type, Generator>{
          FileSystem: () => fs,
          ProcessManager: () => FakeProcessManager.any(),
          FeatureFlags: () => featureFlags,
        },
      );

      testUsingContext(
        'Symlinks are created for Windows plugins',
        () async {
          windowsProject.exists = true;
          createFakePlugin(fs);
          // refreshPluginsList should call createPluginSymlinks.
          await refreshPluginsList(flutterProject);

          expect(windowsProject.pluginSymlinkDirectory.childLink('some_plugin').existsSync(), true);
        },
        overrides: <Type, Generator>{
          FileSystem: () => fs,
          ProcessManager: () => FakeProcessManager.any(),
          FeatureFlags: () => featureFlags,
        },
      );

      testUsingContext(
        'Existing symlinks are removed when no longer in use with force',
        () {
          linuxProject.exists = true;
          windowsProject.exists = true;

          final List<File> dummyFiles = <File>[
            flutterProject.linux.pluginSymlinkDirectory.childFile('dummy'),
            flutterProject.windows.pluginSymlinkDirectory.childFile('dummy'),
          ];
          for (final File file in dummyFiles) {
            file.createSync(recursive: true);
          }

          createPluginSymlinks(flutterProject, force: true);

          for (final File file in dummyFiles) {
            expect(file.existsSync(), false);
          }
        },
        overrides: <Type, Generator>{
          FileSystem: () => fs,
          ProcessManager: () => FakeProcessManager.any(),
          FeatureFlags: () => featureFlags,
        },
      );

      testUsingContext(
        'Existing symlinks are removed automatically on refresh when no longer in use',
        () async {
          linuxProject.exists = true;
          windowsProject.exists = true;

          final List<File> dummyFiles = <File>[
            flutterProject.linux.pluginSymlinkDirectory.childFile('dummy'),
            flutterProject.windows.pluginSymlinkDirectory.childFile('dummy'),
          ];
          for (final File file in dummyFiles) {
            file.createSync(recursive: true);
          }

          // refreshPluginsList should remove existing links and recreate on changes.
          createFakePlugin(fs);
          await refreshPluginsList(flutterProject);

          for (final File file in dummyFiles) {
            expect(file.existsSync(), false);
          }
        },
        overrides: <Type, Generator>{
          FileSystem: () => fs,
          ProcessManager: () => FakeProcessManager.any(),
          FeatureFlags: () => featureFlags,
        },
      );

      testUsingContext(
        'createPluginSymlinks is a no-op without force when up to date',
        () {
          linuxProject.exists = true;
          windowsProject.exists = true;

          final List<File> dummyFiles = <File>[
            flutterProject.linux.pluginSymlinkDirectory.childFile('dummy'),
            flutterProject.windows.pluginSymlinkDirectory.childFile('dummy'),
          ];
          for (final File file in dummyFiles) {
            file.createSync(recursive: true);
          }

          // Without force, this should do nothing to existing files.
          createPluginSymlinks(flutterProject);

          for (final File file in dummyFiles) {
            expect(file.existsSync(), true);
          }
        },
        overrides: <Type, Generator>{
          FileSystem: () => fs,
          ProcessManager: () => FakeProcessManager.any(),
          FeatureFlags: () => featureFlags,
        },
      );

      testUsingContext(
        'createPluginSymlinks repairs missing links',
        () async {
          linuxProject.exists = true;
          windowsProject.exists = true;
          createFakePlugin(fs);
          await refreshPluginsList(flutterProject);

          final List<Link> links = <Link>[
            linuxProject.pluginSymlinkDirectory.childLink('some_plugin'),
            windowsProject.pluginSymlinkDirectory.childLink('some_plugin'),
          ];
          for (final Link link in links) {
            link.deleteSync();
          }
          createPluginSymlinks(flutterProject);

          for (final Link link in links) {
            expect(link.existsSync(), true);
          }
        },
        overrides: <Type, Generator>{
          FileSystem: () => fs,
          ProcessManager: () => FakeProcessManager.any(),
          FeatureFlags: () => featureFlags,
        },
      );
    });

    group('pubspec', () {
      late Directory projectDir;
      late Directory tempDir;

      setUp(() {
        tempDir = globals.fs.systemTempDirectory.createTempSync('flutter_plugin_test.');
        projectDir = tempDir.childDirectory('flutter_project');
      });

      tearDown(() {
        tryToDelete(tempDir);
      });

      void createPubspecFile(String yamlString) {
        projectDir.childFile('pubspec.yaml')
          ..createSync(recursive: true)
          ..writeAsStringSync(yamlString);
      }

      testUsingContext('validatePubspecForPlugin works', () async {
        const String pluginYaml = '''
  flutter:
    plugin:
      platforms:
        ios:
          pluginClass: SomePlugin
        macos:
          pluginClass: SomePlugin
        windows:
          pluginClass: SomePlugin
        linux:
          pluginClass: SomePlugin
        web:
          pluginClass: SomePlugin
          fileName: lib/SomeFile.dart
        android:
          pluginClass: SomePlugin
          package: AndroidPackage
  ''';
        createPubspecFile(pluginYaml);
        validatePubspecForPlugin(
          projectDir: projectDir.absolute.path,
          pluginClass: 'SomePlugin',
          expectedPlatforms: <String>['ios', 'macos', 'windows', 'linux', 'android', 'web'],
          androidIdentifier: 'AndroidPackage',
          webFileName: 'lib/SomeFile.dart',
        );
      });

      testUsingContext('createPlatformsYamlMap should create the correct map', () async {
        final YamlMap map = Plugin.createPlatformsYamlMap(
          <String>['ios', 'android', 'linux'],
          'PluginClass',
          'some.android.package',
        );
        expect(map['ios'], <String, String>{'pluginClass': 'PluginClass'});
        expect(map['android'], <String, String>{
          'pluginClass': 'PluginClass',
          'package': 'some.android.package',
        });
        expect(map['linux'], <String, String>{'pluginClass': 'PluginClass'});
      });

      testUsingContext('createPlatformsYamlMap should create empty map', () async {
        final YamlMap map = Plugin.createPlatformsYamlMap(<String>[], 'foo', 'bar');
        expect(map.isEmpty, true);
      });
    });

    group('Plugin files', () {
      testWithoutContext('pluginSwiftPackageManifestPath for iOS and macOS plugins', () async {
        final MemoryFileSystem fs = MemoryFileSystem.test();
        final Plugin plugin = Plugin(
          name: 'test',
          path: '/path/to/test/',
          defaultPackagePlatforms: const <String, String>{},
          pluginDartClassPlatforms: const <String, DartPluginClassAndFilePair>{},
          platforms: const <String, PluginPlatform>{
            IOSPlugin.kConfigKey: IOSPlugin(name: 'test', classPrefix: ''),
            MacOSPlugin.kConfigKey: MacOSPlugin(name: 'test'),
          },
          dependencies: <String>[],
          isDirectDependency: true,
          isDevDependency: false,
        );

        expect(
          plugin.pluginSwiftPackageManifestPath(fs, IOSPlugin.kConfigKey),
          '/path/to/test/ios/test/Package.swift',
        );
        expect(
          plugin.pluginSwiftPackageManifestPath(fs, MacOSPlugin.kConfigKey),
          '/path/to/test/macos/test/Package.swift',
        );
      });

      testWithoutContext('pluginSwiftPackageManifestPath for darwin plugins', () async {
        final MemoryFileSystem fs = MemoryFileSystem.test();
        final Plugin plugin = Plugin(
          name: 'test',
          path: '/path/to/test/',
          defaultPackagePlatforms: const <String, String>{},
          pluginDartClassPlatforms: const <String, DartPluginClassAndFilePair>{},
          platforms: const <String, PluginPlatform>{
            IOSPlugin.kConfigKey: IOSPlugin(
              name: 'test',
              classPrefix: '',
              sharedDarwinSource: true,
            ),
            MacOSPlugin.kConfigKey: MacOSPlugin(name: 'test', sharedDarwinSource: true),
          },
          dependencies: <String>[],
          isDirectDependency: true,
          isDevDependency: false,
        );

        expect(
          plugin.pluginSwiftPackageManifestPath(fs, IOSPlugin.kConfigKey),
          '/path/to/test/darwin/test/Package.swift',
        );
        expect(
          plugin.pluginSwiftPackageManifestPath(fs, MacOSPlugin.kConfigKey),
          '/path/to/test/darwin/test/Package.swift',
        );
      });

      testWithoutContext('pluginSwiftPackageManifestPath for non darwin plugins', () async {
        final MemoryFileSystem fs = MemoryFileSystem.test();
        final Plugin plugin = Plugin(
          name: 'test',
          path: '/path/to/test/',
          defaultPackagePlatforms: const <String, String>{},
          pluginDartClassPlatforms: const <String, DartPluginClassAndFilePair>{},
          platforms: const <String, PluginPlatform>{
            WindowsPlugin.kConfigKey: WindowsPlugin(name: 'test', pluginClass: ''),
          },
          dependencies: <String>[],
          isDirectDependency: true,
          isDevDependency: false,
        );

        expect(plugin.pluginSwiftPackageManifestPath(fs, IOSPlugin.kConfigKey), isNull);
        expect(plugin.pluginSwiftPackageManifestPath(fs, MacOSPlugin.kConfigKey), isNull);
        expect(plugin.pluginSwiftPackageManifestPath(fs, WindowsPlugin.kConfigKey), isNull);
      });

      testWithoutContext('pluginPodspecPath for iOS and macOS plugins', () async {
        final MemoryFileSystem fs = MemoryFileSystem.test();
        final Plugin plugin = Plugin(
          name: 'test',
          path: '/path/to/test/',
          defaultPackagePlatforms: const <String, String>{},
          pluginDartClassPlatforms: const <String, DartPluginClassAndFilePair>{},
          platforms: const <String, PluginPlatform>{
            IOSPlugin.kConfigKey: IOSPlugin(name: 'test', classPrefix: ''),
            MacOSPlugin.kConfigKey: MacOSPlugin(name: 'test'),
          },
          dependencies: <String>[],
          isDirectDependency: true,
          isDevDependency: false,
        );

        expect(
          plugin.pluginPodspecPath(fs, IOSPlugin.kConfigKey),
          '/path/to/test/ios/test.podspec',
        );
        expect(
          plugin.pluginPodspecPath(fs, MacOSPlugin.kConfigKey),
          '/path/to/test/macos/test.podspec',
        );
      });

      testWithoutContext('pluginPodspecPath for darwin plugins', () async {
        final MemoryFileSystem fs = MemoryFileSystem.test();
        final Plugin plugin = Plugin(
          name: 'test',
          path: '/path/to/test/',
          defaultPackagePlatforms: const <String, String>{},
          pluginDartClassPlatforms: const <String, DartPluginClassAndFilePair>{},
          platforms: const <String, PluginPlatform>{
            IOSPlugin.kConfigKey: IOSPlugin(
              name: 'test',
              classPrefix: '',
              sharedDarwinSource: true,
            ),
            MacOSPlugin.kConfigKey: MacOSPlugin(name: 'test', sharedDarwinSource: true),
          },
          dependencies: <String>[],
          isDirectDependency: true,
          isDevDependency: false,
        );

        expect(
          plugin.pluginPodspecPath(fs, IOSPlugin.kConfigKey),
          '/path/to/test/darwin/test.podspec',
        );
        expect(
          plugin.pluginPodspecPath(fs, MacOSPlugin.kConfigKey),
          '/path/to/test/darwin/test.podspec',
        );
      });

      testWithoutContext('pluginPodspecPath for non darwin plugins', () async {
        final MemoryFileSystem fs = MemoryFileSystem.test();
        final Plugin plugin = Plugin(
          name: 'test',
          path: '/path/to/test/',
          defaultPackagePlatforms: const <String, String>{},
          pluginDartClassPlatforms: const <String, DartPluginClassAndFilePair>{},
          platforms: const <String, PluginPlatform>{
            WindowsPlugin.kConfigKey: WindowsPlugin(name: 'test', pluginClass: ''),
          },
          dependencies: <String>[],
          isDirectDependency: true,
          isDevDependency: false,
        );

        expect(plugin.pluginPodspecPath(fs, IOSPlugin.kConfigKey), isNull);
        expect(plugin.pluginPodspecPath(fs, MacOSPlugin.kConfigKey), isNull);
        expect(plugin.pluginPodspecPath(fs, WindowsPlugin.kConfigKey), isNull);
      });
    });

    testWithoutContext(
      'Symlink failures give developer mode instructions on recent versions of Windows',
      () async {
        final Platform platform = FakePlatform(operatingSystem: 'windows');
        final FakeOperatingSystemUtils os = FakeOperatingSystemUtils(
          'Microsoft Windows [Version 10.0.14972.1]',
        );

        const FileSystemException e = FileSystemException('', '', OSError('', 1314));

        expect(
          () => handleSymlinkException(
            e,
            platform: platform,
            os: os,
            source: pubCachePath,
            destination: ephemeralPackagePath,
          ),
          throwsToolExit(message: 'start ms-settings:developers'),
        );
      },
    );

    testUsingContext(
      'Symlink ERROR_ACCESS_DENIED failures show developers paths that were used',
      () async {
        final FakeFlutterProject flutterProject =
            FakeFlutterProject()..directory = globals.fs.currentDirectory.childDirectory('app');
        final Directory windowsManagedDirectory = flutterProject.directory
            .childDirectory('windows')
            .childDirectory('flutter');
        final FakeWindowsProject windowsProject =
            FakeWindowsProject()
              ..managedDirectory = windowsManagedDirectory
              ..pluginSymlinkDirectory = windowsManagedDirectory
                  .childDirectory('ephemeral')
                  .childDirectory('.plugin_symlinks')
              ..exists = true;

        final File dependenciesFile = flutterProject.directory.childFile(
          '.flutter-plugins-dependencies',
        );
        flutterProject
          ..flutterPluginsDependenciesFile = dependenciesFile
          ..windows = windowsProject;

        flutterProject.directory
            .childDirectory('.dart_tool')
            .childFile('package_config.json')
            .createSync(recursive: true);

        const String dependenciesFileContents = r'''
{
  "plugins": {
    "windows": [
      {
        "name": "some_plugin",
        "path": "C:\\some_plugin"
      }
    ]
  }
}
''';
        dependenciesFile.writeAsStringSync(dependenciesFileContents);

        const String expectedMessage =
            'ERROR_ACCESS_DENIED file system exception thrown while trying to '
            r'create a symlink from C:\some_plugin to '
            r'C:\app\windows\flutter\ephemeral\.plugin_symlinks\some_plugin';

        expect(
          () => createPluginSymlinks(
            flutterProject,
            featureFlagsOverride: TestFeatureFlags(isWindowsEnabled: true),
          ),
          throwsToolExit(message: expectedMessage),
        );
      },
      overrides: <Type, Generator>{
        FileSystem: () {
          final FileExceptionHandler handle = FileExceptionHandler();
          final ErrorHandlingFileSystem fileSystem = ErrorHandlingFileSystem(
            platform: FakePlatform(),
            delegate: MemoryFileSystem.test(
              style: FileSystemStyle.windows,
              opHandle: handle.opHandle,
            ),
          );
          const String pluginSymlinkPath =
              r'C:\app\windows\flutter\ephemeral\.plugin_symlinks\some_plugin';
          handle.addError(
            fileSystem.link(pluginSymlinkPath),
            FileSystemOp.create,
            const FileSystemException('', '', OSError('', 5)),
          );
          return fileSystem;
        },
        Platform: () => FakePlatform(operatingSystem: 'windows'),
        ProcessManager: () => FakeProcessManager.empty(),
      },
    );

    testWithoutContext(
      'Symlink failures instruct developers to run as administrator on older versions of Windows',
      () async {
        final Platform platform = FakePlatform(operatingSystem: 'windows');
        final FakeOperatingSystemUtils os = FakeOperatingSystemUtils(
          'Microsoft Windows [Version 10.0.14393]',
        );

        const FileSystemException e = FileSystemException('', '', OSError('', 1314));

        expect(
          () => handleSymlinkException(
            e,
            platform: platform,
            os: os,
            source: pubCachePath,
            destination: ephemeralPackagePath,
          ),
          throwsToolExit(message: 'administrator'),
        );
      },
    );

    testWithoutContext(
      'Symlink failures instruct developers to have their project on the same drive as their SDK',
      () async {
        final Platform platform = FakePlatform(operatingSystem: 'windows');
        final FakeOperatingSystemUtils os = FakeOperatingSystemUtils(
          'Microsoft Windows [Version 10.0.14972]',
        );

        const FileSystemException e = FileSystemException('', '', OSError('', 1));

        expect(
          () => handleSymlinkException(
            e,
            platform: platform,
            os: os,
            source: pubCachePath,
            destination: ephemeralPackagePath,
          ),
          throwsToolExit(
            message: 'Try moving your Flutter project to the same drive as your Flutter SDK',
          ),
        );
      },
    );

    testWithoutContext('Symlink failures only give instructions for specific errors', () async {
      final Platform platform = FakePlatform(operatingSystem: 'windows');
      final FakeOperatingSystemUtils os = FakeOperatingSystemUtils(
        'Microsoft Windows [Version 10.0.14393]',
      );

      const FileSystemException e = FileSystemException('', '', OSError('', 999));

      expect(
        () => handleSymlinkException(
          e,
          platform: platform,
          os: os,
          source: pubCachePath,
          destination: ephemeralPackagePath,
        ),
        returnsNormally,
      );
    });
  });

  testUsingContext(
    'exits tool when deleting .plugin_symlinks fails',
    () async {
      final FakeFlutterProject flutterProject =
          FakeFlutterProject()..directory = globals.fs.currentDirectory.childDirectory('app');
      final FakeFlutterManifest flutterManifest = FakeFlutterManifest();
      final Directory windowsManagedDirectory = flutterProject.directory
          .childDirectory('windows')
          .childDirectory('flutter');
      final FakeWindowsProject windowsProject =
          FakeWindowsProject()
            ..managedDirectory = windowsManagedDirectory
            ..cmakeFile = windowsManagedDirectory.parent.childFile('CMakeLists.txt')
            ..generatedPluginCmakeFile = windowsManagedDirectory.childFile('generated_plugins.mk')
            ..pluginSymlinkDirectory = windowsManagedDirectory
                .childDirectory('ephemeral')
                .childDirectory('.plugin_symlinks')
            ..exists = true;

      flutterProject
        ..manifest = flutterManifest
        ..flutterPluginsFile = flutterProject.directory.childFile('.flutter-plugins')
        ..flutterPluginsDependenciesFile = flutterProject.directory.childFile(
          '.flutter-plugins-dependencies',
        )
        ..windows = windowsProject;

      flutterProject.directory
          .childDirectory('.dart_tool')
          .childFile('package_config.json')
          .createSync(recursive: true);

      createPluginSymlinks(
        flutterProject,
        force: true,
        featureFlagsOverride: TestFeatureFlags(isWindowsEnabled: true),
      );

      expect(
        () => createPluginSymlinks(
          flutterProject,
          force: true,
          featureFlagsOverride: TestFeatureFlags(isWindowsEnabled: true),
        ),
        throwsToolExit(
          message: RegExp(
            'Unable to delete file or directory at '
            r'"C:\\app\\windows\\flutter\\ephemeral\\\.plugin_symlinks"',
          ),
        ),
      );
    },
    overrides: <Type, Generator>{
      FileSystem: () {
        final FileExceptionHandler handle = FileExceptionHandler();
        final ErrorHandlingFileSystem fileSystem = ErrorHandlingFileSystem(
          platform: FakePlatform(),
          delegate: MemoryFileSystem.test(
            style: FileSystemStyle.windows,
            opHandle: handle.opHandle,
          ),
        );
        const String symlinkDirectoryPath = r'C:\app\windows\flutter\ephemeral\.plugin_symlinks';
        handle.addError(
          fileSystem.directory(symlinkDirectoryPath),
          FileSystemOp.delete,
          const PathNotFoundException(
            symlinkDirectoryPath,
            OSError('The system cannot find the path specified.', 3),
          ),
        );
        return fileSystem;
      },
      ProcessManager: () => FakeProcessManager.empty(),
    },
  );
}

class FakeFlutterManifest extends Fake implements FlutterManifest {
  @override
  Set<String> get dependencies => <String>{};
}

class FakeXcodeProjectInterpreter extends Fake implements XcodeProjectInterpreter {
  @override
  bool get isInstalled => false;
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
  late IosProject ios;

  @override
  late AndroidProject android;

  @override
  late WebProject web;

  @override
  late MacOSProject macos;

  @override
  late LinuxProject linux;

  @override
  late WindowsProject windows;
}

class FakeMacOSProject extends Fake implements MacOSProject {
  @override
  String pluginConfigKey = 'macos';

  bool exists = false;

  @override
  late File podfile;

  @override
  late File podManifestLock;

  @override
  bool usesSwiftPackageManager = false;

  @override
  late Directory managedDirectory;

  @override
  bool existsSync() => exists;
}

class FakeIosProject extends Fake implements IosProject {
  @override
  String pluginConfigKey = 'ios';

  bool testExists = false;

  @override
  bool existsSync() => testExists;

  @override
  bool get exists => testExists;

  @override
  late Directory pluginRegistrantHost;

  @override
  File get pluginRegistrantHeader => pluginRegistrantHost.childFile('GeneratedPluginRegistrant.h');

  @override
  File get pluginRegistrantImplementation =>
      pluginRegistrantHost.childFile('GeneratedPluginRegistrant.m');

  @override
  late File podfile;

  @override
  late File podManifestLock;

  @override
  bool usesSwiftPackageManager = false;
}

class FakeAndroidProject extends Fake implements AndroidProject {
  @override
  String pluginConfigKey = 'android';

  bool exists = false;

  @override
  late Directory pluginRegistrantHost;

  @override
  late Directory hostAppGradleRoot;

  @override
  late File appManifestFile;

  late AndroidEmbeddingVersion embeddingVersion;

  @override
  bool existsSync() => exists;

  @override
  AndroidEmbeddingVersion getEmbeddingVersion() {
    return embeddingVersion;
  }

  @override
  AndroidEmbeddingVersionResult computeEmbeddingVersion() {
    return AndroidEmbeddingVersionResult(embeddingVersion, 'reasons for version');
  }
}

class FakeWebProject extends Fake implements WebProject {
  @override
  String pluginConfigKey = 'web';

  @override
  late Directory libDirectory;

  bool exists = false;

  @override
  bool existsSync() => exists;
}

class FakeWindowsProject extends Fake implements WindowsProject {
  @override
  String pluginConfigKey = 'windows';

  @override
  late Directory managedDirectory;

  @override
  late Directory ephemeralDirectory;

  @override
  late Directory pluginSymlinkDirectory;

  @override
  late File cmakeFile;

  @override
  late File generatedPluginCmakeFile;
  bool exists = false;

  @override
  bool existsSync() => exists;
}

class FakeLinuxProject extends Fake implements LinuxProject {
  @override
  String pluginConfigKey = 'linux';

  @override
  late Directory managedDirectory;

  @override
  late Directory ephemeralDirectory;

  @override
  late Directory pluginSymlinkDirectory;

  @override
  late File cmakeFile;

  @override
  late File generatedPluginCmakeFile;
  bool exists = false;

  @override
  bool existsSync() => exists;
}

class FakeOperatingSystemUtils extends Fake implements OperatingSystemUtils {
  FakeOperatingSystemUtils(this.name);

  @override
  final String name;
}

class FakeSystemClock extends Fake implements SystemClock {
  late DateTime currentTime;

  @override
  DateTime now() {
    return currentTime;
  }
}

class FakeDarwinDependencyManagement extends Fake implements DarwinDependencyManagement {
  List<SupportedPlatform> setupPlatforms = <SupportedPlatform>[];

  @override
  Future<void> setUp({required SupportedPlatform platform}) async {
    setupPlatforms.add(platform);
  }
}
