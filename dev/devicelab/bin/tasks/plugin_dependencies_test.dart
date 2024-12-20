// Copyright 2014 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

import 'dart:convert';
import 'dart:io';

import 'package:flutter_devicelab/framework/framework.dart';
import 'package:flutter_devicelab/framework/ios.dart';
import 'package:flutter_devicelab/framework/task_result.dart';
import 'package:flutter_devicelab/framework/utils.dart';
import 'package:path/path.dart' as path;

/// Tests that a plugin A can depend on platform code from a plugin B
/// as long as plugin B is defined as a pub dependency of plugin A.
///
/// This test fails when `flutter build apk` fails and the stderr from this command
/// contains "Unresolved reference: plugin_b".
Future<void> main() async {
  await task(() async {
    section('Find Java');

    final String? javaHome = await findJavaHome();
    if (javaHome == null) {
      return TaskResult.failure('Could not find Java');
    }

    print('\nUsing JAVA_HOME=$javaHome');

    final Directory tempDir = Directory.systemTemp.createTempSync('flutter_plugin_dependencies.');
    try {
      section('Create plugin A');

      final Directory pluginADirectory = Directory(path.join(tempDir.path, 'plugin_a'));
      await inDirectory(tempDir, () async {
        await flutter(
          'create',
          options: <String>[
            '--org',
            'io.flutter.devicelab.plugin_a',
            '--template=plugin',
            '--platforms=android,ios',
            pluginADirectory.path,
          ],
        );
      });

      section('Create plugin B');

      final Directory pluginBDirectory = Directory(path.join(tempDir.path, 'plugin_b'));
      await inDirectory(tempDir, () async {
        await flutter(
          'create',
          options: <String>[
            '--org',
            'io.flutter.devicelab.plugin_b',
            '--template=plugin',
            '--platforms=android,ios',
            pluginBDirectory.path,
          ],
        );
      });

      section('Create plugin C without android/ directory');

      final Directory pluginCDirectory = Directory(path.join(tempDir.path, 'plugin_c'));
      await inDirectory(tempDir, () async {
        await flutter(
          'create',
          options: <String>[
            '--org',
            'io.flutter.devicelab.plugin_c',
            '--template=plugin',
            '--platforms=ios',
            pluginCDirectory.path,
          ],
        );
      });

      checkDirectoryNotExists(path.join(pluginCDirectory.path, 'android'));

      final File pluginCpubspec = File(path.join(pluginCDirectory.path, 'pubspec.yaml'));
      await pluginCpubspec.writeAsString('''
name: plugin_c
version: 0.0.1

flutter:
  plugin:
    platforms:
      ios:
        pluginClass: Plugin_cPlugin

dependencies:
  flutter:
    sdk: flutter

environment:
  sdk: ^3.7.0-0
  flutter: ">=1.5.0"
''', flush: true);

      section('Create plugin D without ios/ directory');

      final Directory pluginDDirectory = Directory(path.join(tempDir.path, 'plugin_d'));
      await inDirectory(tempDir, () async {
        await flutter(
          'create',
          options: <String>[
            '--org',
            'io.flutter.devicelab.plugin_d',
            '--template=plugin',
            '--platforms=android',
            pluginDDirectory.path,
          ],
        );
      });

      checkDirectoryNotExists(path.join(pluginDDirectory.path, 'ios'));

      section('Write dummy Kotlin code in plugin B');

      final File pluginBKotlinClass = File(
        path.join(
          pluginBDirectory.path,
          'android',
          'src',
          'main',
          'kotlin',
          'DummyPluginBClass.kt',
        ),
      );

      await pluginBKotlinClass.writeAsString('''
package io.flutter.devicelab.plugin_b

public class DummyPluginBClass {
  companion object {
    fun dummyStaticMethod() {
    }
  }
}
''', flush: true);

      section('Make plugin A depend on plugin B, C, and D');

      final File pluginApubspec = File(path.join(pluginADirectory.path, 'pubspec.yaml'));
      String pluginApubspecContent = await pluginApubspec.readAsString();
      pluginApubspecContent = pluginApubspecContent.replaceFirst(
        '${Platform.lineTerminator}dependencies:${Platform.lineTerminator}',
        '${Platform.lineTerminator}dependencies:${Platform.lineTerminator}'
            '  plugin_b:${Platform.lineTerminator}'
            '    path: ${pluginBDirectory.path}${Platform.lineTerminator}'
            '  plugin_c:${Platform.lineTerminator}'
            '    path: ${pluginCDirectory.path}${Platform.lineTerminator}'
            '  plugin_d:${Platform.lineTerminator}'
            '    path: ${pluginDDirectory.path}${Platform.lineTerminator}',
      );
      await pluginApubspec.writeAsString(pluginApubspecContent, flush: true);

      section('Write Kotlin code in plugin A that references Kotlin code from plugin B');

      final File pluginAKotlinClass = File(
        path.join(
          pluginADirectory.path,
          'android',
          'src',
          'main',
          'kotlin',
          'DummyPluginAClass.kt',
        ),
      );

      await pluginAKotlinClass.writeAsString('''
package io.flutter.devicelab.plugin_a

import io.flutter.devicelab.plugin_b.DummyPluginBClass

public class DummyPluginAClass {
  constructor() {
    // Call a method from plugin b.
    DummyPluginBClass.dummyStaticMethod();
  }
}
''', flush: true);

      section('Verify .flutter-plugins-dependencies');

      final Directory exampleApp = Directory(path.join(pluginADirectory.path, 'example'));

      await inDirectory(exampleApp, () async {
        await flutter('packages', options: <String>['get']);
      });

      final File flutterPluginsDependenciesFile = File(
        path.join(exampleApp.path, '.flutter-plugins-dependencies'),
      );

      if (!flutterPluginsDependenciesFile.existsSync()) {
        return TaskResult.failure("${flutterPluginsDependenciesFile.path} doesn't exist");
      }

      final String flutterPluginsDependenciesFileContent =
          flutterPluginsDependenciesFile.readAsStringSync();

      final Map<String, dynamic> jsonContent =
          json.decode(flutterPluginsDependenciesFileContent) as Map<String, dynamic>;
      final Map<String, dynamic>? swiftPackageManagerJson =
          jsonContent['swift_package_manager_enabled'] as Map<String, dynamic>?;

      // Verify the dependencyGraph object is valid. The rest of the contents of this file are not relevant to the
      // dependency graph and are tested by unit tests.
      final List<dynamic> dependencyGraph = jsonContent['dependencyGraph'] as List<dynamic>;
      const String kExpectedPluginsDependenciesContent =
          '['
          '{'
          '"name":"integration_test",'
          '"dependencies":[]'
          '},'
          '{'
          '"name":"plugin_a",'
          '"dependencies":["plugin_b","plugin_c","plugin_d"]'
          '},'
          '{'
          '"name":"plugin_b",'
          '"dependencies":[]'
          '},'
          '{'
          '"name":"plugin_c",'
          '"dependencies":[]'
          '},'
          '{'
          '"name":"plugin_d",'
          '"dependencies":[]'
          '}'
          ']';
      final String graphString = json.encode(dependencyGraph);
      if (graphString != kExpectedPluginsDependenciesContent) {
        return TaskResult.failure(
          'Unexpected file content in ${flutterPluginsDependenciesFile.path}: '
          'Found "$graphString" instead of "$kExpectedPluginsDependenciesContent"',
        );
      }

      section('Build plugin A example Android app');

      final StringBuffer stderr = StringBuffer();
      await inDirectory(exampleApp, () async {
        await evalFlutter(
          'build',
          options: <String>['apk', '--target-platform', 'android-arm'],
          canFail: true,
          stderr: stderr,
        );
      });

      if (stderr.toString().contains('Unresolved reference: plugin_b')) {
        return TaskResult.failure('plugin_a cannot reference plugin_b');
      }

      final bool pluginAExampleApk = exists(
        File(
          path.join(
            pluginADirectory.path,
            'example',
            'build',
            'app',
            'outputs',
            'apk',
            'release',
            'app-release.apk',
          ),
        ),
      );

      if (!pluginAExampleApk) {
        return TaskResult.failure('Failed to build plugin A example APK');
      }

      if (Platform.isMacOS) {
        section('Build plugin A example iOS app');

        await inDirectory(exampleApp, () async {
          await evalFlutter('build', options: <String>['ios', '--no-codesign', '--verbose']);
        });

        final Directory appBundle = Directory(
          path.join(pluginADirectory.path, 'example', 'build', 'ios', 'iphoneos', 'Runner.app'),
        );

        if (!exists(appBundle)) {
          return TaskResult.failure('Failed to build plugin A example iOS app');
        }

        final bool? swiftPackageManagerEnabled = swiftPackageManagerJson?['ios'] as bool?;
        if (swiftPackageManagerEnabled == null) {
          return TaskResult.failure(
            '${flutterPluginsDependenciesFile.path} is missing the '
            '"swift_package_manager_enabled" > "ios" property.\n'
            '\n'
            '.flutter_plugin_dependencies content:\n'
            '\n'
            '$flutterPluginsDependenciesFileContent',
          );
        }

        if (swiftPackageManagerEnabled) {
          // Check plugins are built statically if using SwiftPM.
          final String executable = path.join(appBundle.path, 'Runner');
          final String symbols = await dumpSymbolTable(executable);

          final bool foundA = symbols.contains('plugin_a');
          final bool foundB = symbols.contains('plugin_b');
          final bool foundC = symbols.contains('plugin_c');
          final bool foundD = symbols.contains('plugin_d');

          if (!foundA || !foundB || !foundC) {
            return TaskResult.failure(
              'Failed to find plugins_a, plugin_b, or plugin_c symbols in the app',
            );
          }

          if (foundD) {
            return TaskResult.failure('Found Android plugin_d symbols in iOS app');
          }
        } else {
          // Check plugins are built dynamically if using CocoaPods.
          checkDirectoryExists(path.join(appBundle.path, 'Frameworks', 'plugin_a.framework'));
          checkDirectoryExists(path.join(appBundle.path, 'Frameworks', 'plugin_b.framework'));
          checkDirectoryExists(path.join(appBundle.path, 'Frameworks', 'plugin_c.framework'));

          checkDirectoryNotExists(path.join(appBundle.path, 'Frameworks', 'plugin_d.framework'));
        }
      }

      return TaskResult.success(null);
    } on TaskResult catch (taskResult) {
      return taskResult;
    } catch (e) {
      return TaskResult.failure(e.toString());
    } finally {
      rmTree(tempDir);
    }
  });
}
