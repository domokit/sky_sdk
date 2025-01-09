// Copyright 2014 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

import 'package:file/memory.dart';
import 'package:flutter_tools/src/artifacts.dart';
import 'package:flutter_tools/src/base/file_system.dart';
import 'package:flutter_tools/src/base/logger.dart';
import 'package:flutter_tools/src/build_info.dart';
import 'package:flutter_tools/src/build_system/build_system.dart';
import 'package:flutter_tools/src/build_system/targets/dart_plugin_registrant.dart';
import 'package:flutter_tools/src/dart/pub.dart';
import 'package:flutter_tools/src/features.dart';
import 'package:flutter_tools/src/project.dart';

import '../../../src/common.dart';
import '../../../src/context.dart';
import '../../../src/fake_pub_deps.dart';
import '../../../src/fakes.dart';

const String _kEmptyPubspecFile = '''
name: path_provider_example

dependencies:
  flutter:
    sdk: flutter
''';

const String _kEmptyPackageJson = '''
{
  "configVersion": 2,
  "packages": [
     {
      "name": "path_provider_example",
      "rootUri": "../",
      "packageUri": "lib/",
      "languageVersion": "2.12"
    }
  ]
}
''';

const String _kSamplePackageJson = '''
{
  "configVersion": 2,
  "packages": [
    {
      "name": "path_provider_linux",
      "rootUri": "/path_provider_linux",
      "packageUri": "lib/",
      "languageVersion": "2.12"
    },
    {
      "name": "path_provider_example",
      "rootUri": "../",
      "packageUri": "lib/",
      "languageVersion": "2.12"
    }
  ]
}
''';

const String _kSamplePubspecFile = '''
name: path_provider_example
description: Demonstrates how to use the path_provider plugin.

dependencies:
  flutter:
    sdk: flutter
  path_provider_linux: 1.0.0
''';

const String _kSamplePluginPubspec = '''
name: path_provider_linux
description: linux implementation of the path_provider plugin
// version: 2.0.1
// homepage: https://github.com/flutter/plugins/tree/main/packages/path_provider/path_provider_linux

flutter:
  plugin:
    implements: path_provider
    platforms:
      linux:
        dartPluginClass: PathProviderLinux
        pluginClass: none

environment:
  sdk: ^3.7.0-0
  flutter: ">=1.20.0"
''';

void main() {
  // TODO(matanlurey): Remove after `explicit-package-dependencies` is enabled by default.
  // See https://github.com/flutter/flutter/issues/160257 for details.
  FeatureFlags enableExplicitPackageDependencies() {
    return TestFeatureFlags(isExplicitPackageDependenciesEnabled: true);
  }

  group('Dart plugin registrant', () {
    late FileSystem fileSystem;

    setUp(() {
      fileSystem = MemoryFileSystem.test();
    });

    testWithoutContext('skipped based on environment.generateDartPluginRegistry', () async {
      final Environment environment = Environment.test(
        fileSystem.currentDirectory,
        artifacts: Artifacts.test(),
        fileSystem: fileSystem,
        logger: BufferLogger.test(),
        processManager: FakeProcessManager.any(),
      );

      expect(const DartPluginRegistrantTarget().canSkip(environment), isTrue);

      final Environment environment2 = Environment.test(
        fileSystem.currentDirectory,
        artifacts: Artifacts.test(),
        fileSystem: fileSystem,
        logger: BufferLogger.test(),
        processManager: FakeProcessManager.any(),
        generateDartPluginRegistry: true,
      );

      expect(const DartPluginRegistrantTarget().canSkip(environment2), isFalse);
    });

    testWithoutContext('skipped based on platform', () async {
      const Map<String, bool> canSkip = <String, bool>{
        'darwin-x64': false,
        'linux-x64': false,
        'linux-arm64': false,
        'windows-x64': false,
        'web-javascript': true,
        'ios': false,
        'android': false,
        'fuchsia-arm64': true,
        'fuchsia-x64': true,
      };

      for (final String targetPlatform in canSkip.keys) {
        expect(
          const DartPluginRegistrantTarget().canSkip(
            Environment.test(
              fileSystem.currentDirectory,
              artifacts: Artifacts.test(),
              fileSystem: fileSystem,
              logger: BufferLogger.test(),
              processManager: FakeProcessManager.any(),
              generateDartPluginRegistry: true,
              defines: <String, String>{kTargetPlatform: targetPlatform},
            ),
          ),
          canSkip[targetPlatform],
        );
      }
    });

    testUsingContext(
      "doesn't generate dart_plugin_registrant.dart if there aren't Dart plugins",
      () async {
        final Directory projectDir = fileSystem.directory('project')..createSync();
        final Environment environment = Environment.test(
          fileSystem.currentDirectory,
          projectDir: projectDir,
          artifacts: Artifacts.test(),
          fileSystem: fileSystem,
          logger: BufferLogger.test(),
          processManager: FakeProcessManager.any(),
          defines: <String, String>{
            kTargetFile: projectDir.childDirectory('lib').childFile('main.dart').absolute.path,
          },
          generateDartPluginRegistry: true,
        );

        projectDir.childDirectory('.dart_tool').childFile('package_config.json')
          ..createSync(recursive: true)
          ..writeAsStringSync(_kSamplePackageJson);

        projectDir.childFile('pubspec.yaml').createSync();

        final FlutterProject testProject = FlutterProject.fromDirectoryTest(projectDir);
        await DartPluginRegistrantTarget.test(testProject).build(environment);

        final File generatedMain = projectDir
            .childDirectory('.dart_tool')
            .childDirectory('flutter_build')
            .childFile('dart_plugin_registrant.dart');
        expect(generatedMain.existsSync(), isFalse);
      },
      overrides: <Type, Generator>{
        ProcessManager: () => FakeProcessManager.any(),
        FeatureFlags: enableExplicitPackageDependencies,
        Pub: FakePubWithPrimedDeps.new,
      },
    );

    testUsingContext(
      'regenerates dart_plugin_registrant.dart',
      () async {
        final Directory projectDir = fileSystem.directory('project')..createSync();
        final Environment environment = Environment.test(
          fileSystem.currentDirectory,
          projectDir: projectDir,
          artifacts: Artifacts.test(),
          fileSystem: fileSystem,
          logger: BufferLogger.test(),
          processManager: FakeProcessManager.any(),
          defines: <String, String>{
            kTargetFile: projectDir.childDirectory('lib').childFile('main.dart').absolute.path,
          },
          generateDartPluginRegistry: true,
        );

        projectDir.childDirectory('.dart_tool').childFile('package_config.json')
          ..createSync(recursive: true)
          ..writeAsStringSync(_kSamplePackageJson);

        projectDir.childFile('pubspec.yaml').writeAsStringSync(_kSamplePubspecFile);

        projectDir.childDirectory('lib').childFile('main.dart').createSync(recursive: true);

        environment.fileSystem.currentDirectory
            .childDirectory('path_provider_linux')
            .childFile('pubspec.yaml')
          ..createSync(recursive: true)
          ..writeAsStringSync(_kSamplePluginPubspec);

        final FlutterProject testProject = FlutterProject.fromDirectoryTest(projectDir);
        await DartPluginRegistrantTarget.test(testProject).build(environment);

        final File generatedMain = projectDir
            .childDirectory('.dart_tool')
            .childDirectory('flutter_build')
            .childFile('dart_plugin_registrant.dart');
        final String mainContent = generatedMain.readAsStringSync();
        expect(
          mainContent,
          equals(
            '//\n'
            '// Generated file. Do not edit.\n'
            '// This file is generated from template in file `flutter_tools/lib/src/flutter_plugins.dart`.\n'
            '//\n'
            '\n'
            '// @dart = 2.12\n'
            '\n'
            "import 'dart:io'; // flutter_ignore: dart_io_import.\n"
            "import 'package:path_provider_linux/path_provider_linux.dart';\n"
            '\n'
            "@pragma('vm:entry-point')\n"
            'class _PluginRegistrant {\n'
            '\n'
            "  @pragma('vm:entry-point')\n"
            '  static void register() {\n'
            '    if (Platform.isAndroid) {\n'
            '    } else if (Platform.isIOS) {\n'
            '    } else if (Platform.isLinux) {\n'
            '      try {\n'
            '        PathProviderLinux.registerWith();\n'
            '      } catch (err) {\n'
            '        print(\n'
            "          '`path_provider_linux` threw an error: \$err. '\n"
            "          'The app may not function as expected until you remove this plugin from pubspec.yaml'\n"
            '        );\n'
            '      }\n'
            '\n'
            '    } else if (Platform.isMacOS) {\n'
            '    } else if (Platform.isWindows) {\n'
            '    }\n'
            '  }\n'
            '}\n',
          ),
        );
      },
      overrides: <Type, Generator>{
        ProcessManager: () => FakeProcessManager.any(),
        FeatureFlags: enableExplicitPackageDependencies,
        Pub: FakePubWithPrimedDeps.new,
      },
    );

    testUsingContext(
      'removes dart_plugin_registrant.dart if plugins are removed from pubspec.yaml',
      () async {
        final Directory projectDir = fileSystem.directory('project')..createSync();
        final Environment environment = Environment.test(
          fileSystem.currentDirectory,
          projectDir: projectDir,
          artifacts: Artifacts.test(),
          fileSystem: fileSystem,
          logger: BufferLogger.test(),
          processManager: FakeProcessManager.any(),
          defines: <String, String>{
            kTargetFile: projectDir.childDirectory('lib').childFile('main.dart').absolute.path,
          },
          generateDartPluginRegistry: true,
        );
        final File config =
            projectDir.childDirectory('.dart_tool').childFile('package_config.json')
              ..createSync(recursive: true)
              ..writeAsStringSync(_kSamplePackageJson);

        final File pubspec = projectDir.childFile('pubspec.yaml')
          ..writeAsStringSync(_kSamplePubspecFile);

        environment.fileSystem.currentDirectory
            .childDirectory('path_provider_linux')
            .childFile('pubspec.yaml')
          ..createSync(recursive: true)
          ..writeAsStringSync(_kSamplePluginPubspec);

        final File generatedMain = projectDir
            .childDirectory('.dart_tool')
            .childDirectory('flutter_build')
            .childFile('dart_plugin_registrant.dart');

        final FlutterProject testProject = FlutterProject.fromDirectoryTest(projectDir);
        await DartPluginRegistrantTarget.test(testProject).build(environment);
        expect(generatedMain.existsSync(), isTrue);

        // Simulate a user removing everything from pubspec.yaml.
        pubspec.writeAsStringSync(_kEmptyPubspecFile);
        config.writeAsStringSync(_kEmptyPackageJson);

        await DartPluginRegistrantTarget.test(testProject).build(environment);
        expect(generatedMain.existsSync(), isFalse);
      },
      overrides: <Type, Generator>{
        ProcessManager: () => FakeProcessManager.any(),
        FeatureFlags: enableExplicitPackageDependencies,
        Pub: FakePubWithPrimedDeps.new,
      },
    );

    testUsingContext(
      'target file is outside the current project package',
      () async {
        final Directory projectDir = fileSystem.directory('project')..createSync();
        final Environment environment = Environment.test(
          fileSystem.currentDirectory,
          projectDir: projectDir,
          artifacts: Artifacts.test(),
          fileSystem: fileSystem,
          logger: BufferLogger.test(),
          processManager: FakeProcessManager.any(),
          defines: <String, String>{
            kTargetFile: fileSystem.directory('root').childFile('external.dart').absolute.path,
          },
          generateDartPluginRegistry: true,
        );

        projectDir.childDirectory('.dart_tool').childFile('package_config.json')
          ..createSync(recursive: true)
          ..writeAsStringSync(_kSamplePackageJson);

        projectDir.childFile('pubspec.yaml').writeAsStringSync(_kSamplePubspecFile);

        projectDir.childDirectory('lib').childFile('main.dart').createSync(recursive: true);

        environment.fileSystem.currentDirectory
            .childDirectory('path_provider_linux')
            .childFile('pubspec.yaml')
          ..createSync(recursive: true)
          ..writeAsStringSync(_kSamplePluginPubspec);

        final FlutterProject testProject = FlutterProject.fromDirectoryTest(projectDir);
        await DartPluginRegistrantTarget.test(testProject).build(environment);

        final File generatedMain = projectDir
            .childDirectory('.dart_tool')
            .childDirectory('flutter_build')
            .childFile('dart_plugin_registrant.dart');

        final String mainContent = generatedMain.readAsStringSync();
        expect(
          mainContent,
          equals(
            '//\n'
            '// Generated file. Do not edit.\n'
            '// This file is generated from template in file `flutter_tools/lib/src/flutter_plugins.dart`.\n'
            '//\n'
            '\n'
            '// @dart = 2.12\n'
            '\n'
            "import 'dart:io'; // flutter_ignore: dart_io_import.\n"
            "import 'package:path_provider_linux/path_provider_linux.dart';\n"
            '\n'
            "@pragma('vm:entry-point')\n"
            'class _PluginRegistrant {\n'
            '\n'
            "  @pragma('vm:entry-point')\n"
            '  static void register() {\n'
            '    if (Platform.isAndroid) {\n'
            '    } else if (Platform.isIOS) {\n'
            '    } else if (Platform.isLinux) {\n'
            '      try {\n'
            '        PathProviderLinux.registerWith();\n'
            '      } catch (err) {\n'
            '        print(\n'
            "          '`path_provider_linux` threw an error: \$err. '\n"
            "          'The app may not function as expected until you remove this plugin from pubspec.yaml'\n"
            '        );\n'
            '      }\n'
            '\n'
            '    } else if (Platform.isMacOS) {\n'
            '    } else if (Platform.isWindows) {\n'
            '    }\n'
            '  }\n'
            '}\n',
          ),
        );
      },
      overrides: <Type, Generator>{
        ProcessManager: () => FakeProcessManager.any(),
        FeatureFlags: enableExplicitPackageDependencies,
        Pub: FakePubWithPrimedDeps.new,
      },
    );
  });
}
