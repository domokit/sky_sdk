// Copyright 2014 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

import 'dart:io';

import 'package:flutter_devicelab/framework/framework.dart';
import 'package:flutter_devicelab/framework/task_result.dart';
import 'package:flutter_devicelab/framework/utils.dart';
import 'package:path/path.dart' as path;

/// Tests that the Flutter plugin template works. Use `pod lib lint`
/// to confirm the plugin module can be imported into an app.
Future<void> main() async {
  await task(() async {
    final Directory tempDir = Directory.systemTemp.createTempSync('flutter_plugin_test.');
    try {
      section('Lint integration_test');

      await inDirectory(tempDir, () async {
        // Update pod repo.
        await exec('pod', <String>['repo', 'update']);

        // Relative to this script.
        final String flutterRoot = path.dirname(
          path.dirname(path.dirname(path.dirname(path.dirname(path.fromUri(Platform.script))))),
        );
        print('Flutter root at $flutterRoot');
        final String integrationTestPackage = path.join(
          flutterRoot,
          'packages',
          'integration_test',
        );
        final String iosintegrationTestPodspec = path.join(
          integrationTestPackage,
          'ios',
          'integration_test.podspec',
        );

        await exec('pod', <String>['lib', 'lint', iosintegrationTestPodspec, '--use-libraries']);

        final String macosintegrationTestPodspec = path.join(
          integrationTestPackage,
          'integration_test_macos',
          'macos',
          'integration_test_macos.podspec',
        );
        await _tryMacOSLint(macosintegrationTestPodspec, <String>[]);
      });

      section('Create Objective-C plugin');

      const String objcPluginName = 'test_plugin_objc';
      await inDirectory(tempDir, () async {
        await flutter(
          'create',
          options: <String>[
            '--org',
            'io.flutter.devicelab',
            '--template=plugin',
            '--platforms=ios,android',
            '--ios-language=objc',
            objcPluginName,
          ],
        );
      });

      section('Lint Objective-C iOS podspec plugin as framework');

      final String objcPluginPath = path.join(tempDir.path, objcPluginName);
      final String objcPodspecPath = path.join(objcPluginPath, 'ios', '$objcPluginName.podspec');
      await inDirectory(tempDir, () async {
        await exec('pod', <String>['lib', 'lint', objcPodspecPath, '--allow-warnings']);
      });

      section('Lint Objective-C iOS podspec plugin as library');

      await inDirectory(tempDir, () async {
        await exec('pod', <String>[
          'lib',
          'lint',
          objcPodspecPath,
          '--allow-warnings',
          '--use-libraries',
        ]);
      });

      section('Create Swift plugin');

      const String swiftPluginName = 'test_plugin_swift';
      await inDirectory(tempDir, () async {
        await flutter(
          'create',
          options: <String>[
            '--org',
            'io.flutter.devicelab',
            '--template=plugin',
            '--platforms=ios,macos',
            '--ios-language=swift',
            swiftPluginName,
          ],
        );
      });

      section('Lint Swift iOS podspec plugin as framework');

      final String swiftPluginPath = path.join(tempDir.path, swiftPluginName);
      final String swiftPodspecPath = path.join(swiftPluginPath, 'ios', '$swiftPluginName.podspec');
      await inDirectory(tempDir, () async {
        await exec('pod', <String>['lib', 'lint', swiftPodspecPath, '--allow-warnings']);
      });

      section('Lint Swift iOS podspec plugin as library');

      await inDirectory(tempDir, () async {
        await exec('pod', <String>[
          'lib',
          'lint',
          swiftPodspecPath,
          '--allow-warnings',
          '--use-libraries',
        ]);
      });

      section('Lint Swift macOS podspec plugin as framework');

      final String macOSPodspecPath = path.join(
        swiftPluginPath,
        'macos',
        '$swiftPluginName.podspec',
      );
      await inDirectory(tempDir, () async {
        await _tryMacOSLint(macOSPodspecPath, <String>['--allow-warnings']);
      });

      section('Lint Swift macOS podspec plugin as library');

      await inDirectory(tempDir, () async {
        await _tryMacOSLint(macOSPodspecPath, <String>['--allow-warnings', '--use-libraries']);
      });

      section('Create Objective-C application');

      const String objcAppName = 'test_app_objc';
      await inDirectory(tempDir, () async {
        await flutter(
          'create',
          options: <String>[
            '--org',
            'io.flutter.devicelab',
            '--platforms=ios',
            '--ios-language=objc',
            objcAppName,
          ],
        );
      });

      section('Build Objective-C application with Swift and Objective-C plugins as libraries');

      final String objcAppPath = path.join(tempDir.path, objcAppName);
      final File objcPubspec = File(path.join(objcAppPath, 'pubspec.yaml'));
      String pubspecContent = objcPubspec.readAsStringSync();
      // Add (randomly selected) first-party plugins that support iOS and macOS.
      // Add the new plugins we just made.
      pubspecContent = pubspecContent.replaceFirst(
        '\ndependencies:\n',
        '\ndependencies:\n  $objcPluginName:\n    path: $objcPluginPath\n  $swiftPluginName:\n    path: $swiftPluginPath\n  url_launcher: 6.0.16\n  url_launcher_macos:\n',
      );
      objcPubspec.writeAsStringSync(pubspecContent, flush: true);

      await inDirectory(objcAppPath, () async {
        await flutter(
          'build',
          options: <String>['ios', '--no-codesign'],
          // TODO(jmagman): Make Objective-C applications handle Swift libraries https://github.com/flutter/flutter/issues/16049
          canFail: true,
        );
      });

      final File objcPodfile = File(path.join(objcAppPath, 'ios', 'Podfile'));
      String objcPodfileContent = objcPodfile.readAsStringSync();
      if (objcPodfileContent.contains('use_frameworks!')) {
        return TaskResult.failure(
          'Expected default Objective-C Podfile to not contain use_frameworks',
        );
      }
      _validateIosPodfile(objcAppPath);

      section('Build Objective-C application with Swift and Objective-C plugins as frameworks');

      objcPodfileContent = 'use_frameworks!\n$objcPodfileContent';
      objcPodfile.writeAsStringSync(objcPodfileContent, flush: true);

      await inDirectory(objcAppPath, () async {
        await flutter('build', options: <String>['ios', '--no-codesign']);
      });

      section('Create Swift application');

      const String swiftAppName = 'test_app_swift';
      await inDirectory(tempDir, () async {
        await flutter(
          'create',
          options: <String>[
            '--org',
            'io.flutter.devicelab',
            '--platforms=ios,macos',
            '--ios-language=swift',
            swiftAppName,
          ],
        );
      });

      section('Build Swift iOS application with Swift and Objective-C plugins as frameworks');

      final String swiftAppPath = path.join(tempDir.path, swiftAppName);

      final File swiftPubspec = File(path.join(swiftAppPath, 'pubspec.yaml'));
      swiftPubspec.writeAsStringSync(pubspecContent, flush: true);

      await inDirectory(swiftAppPath, () async {
        await flutter('build', options: <String>['ios', '--no-codesign']);
      });

      final File swiftPodfile = File(path.join(swiftAppPath, 'ios', 'Podfile'));
      String swiftPodfileContent = swiftPodfile.readAsStringSync();
      if (!swiftPodfileContent.contains('use_frameworks!')) {
        return TaskResult.failure('Expected default Swift Podfile to contain use_frameworks');
      }

      section('Build Swift iOS application with Swift and Objective-C plugins as libraries');

      swiftPodfileContent = swiftPodfileContent.replaceAll('use_frameworks!', '');
      swiftPodfile.writeAsStringSync(swiftPodfileContent, flush: true);

      await inDirectory(swiftAppPath, () async {
        await flutter('build', options: <String>['ios', '--no-codesign']);
      });

      _validateIosPodfile(swiftAppPath);

      section('Build Swift macOS application with plugins as frameworks');
      await inDirectory(swiftAppPath, () async {
        await flutter('build', options: <String>['macos']);
      });

      final File macOSPodfile = File(path.join(swiftAppPath, 'macos', 'Podfile'));
      String macosPodfileContent = macOSPodfile.readAsStringSync();
      if (!macosPodfileContent.contains('use_frameworks!')) {
        return TaskResult.failure('Expected default Swift Podfile to contain use_frameworks');
      }

      _validateMacOSPodfile(swiftAppPath);

      section('Build Swift macOS application with plugins as libraries');

      macosPodfileContent = macosPodfileContent.replaceAll('use_frameworks!', '');
      macOSPodfile.writeAsStringSync(macosPodfileContent, flush: true);

      await inDirectory(swiftAppPath, () async {
        await flutter('build', options: <String>['macos']);
      });

      _validateMacOSPodfile(swiftAppPath);

      section('Remove iOS support from plugin');

      Directory(path.join(objcPluginPath, 'ios')).deleteSync(recursive: true);

      const String iosPlatformMap = '''
      ios:
        pluginClass: TestPluginObjcPlugin''';

      final File pluginPubspec = File(path.join(objcPluginPath, 'pubspec.yaml'));
      String pluginPubspecContent = pluginPubspec.readAsStringSync();
      if (!pluginPubspecContent.contains(iosPlatformMap)) {
        return TaskResult.failure('Plugin pubspec.yaml missing iOS platform map');
      }

      pluginPubspecContent = pluginPubspecContent.replaceFirst(iosPlatformMap, '');
      pluginPubspec.writeAsStringSync(pluginPubspecContent, flush: true);

      await inDirectory(swiftAppPath, () async {
        await flutter('clean');
        await flutter('build', options: <String>['ios', '--no-codesign']);
      });

      section('Validate plugin without iOS platform');

      final File podfileLockFile = File(path.join(swiftAppPath, 'ios', 'Podfile.lock'));
      final String podfileLockOutput = podfileLockFile.readAsStringSync();
      if (!podfileLockOutput.contains(':path: ".symlinks/plugins/url_launcher_ios/ios"') ||
          !podfileLockOutput.contains(':path: Flutter')
          // test_plugin_objc no longer supports iOS, shouldn't be present.
          ||
          podfileLockOutput.contains(':path: ".symlinks/plugins/test_plugin_objc/ios"') ||
          !podfileLockOutput.contains(':path: ".symlinks/plugins/test_plugin_swift/ios"')) {
        print(podfileLockOutput);
        return TaskResult.failure('Podfile.lock does not contain expected pods');
      }

      final String pluginSymlinks = path.join(swiftAppPath, 'ios', '.symlinks', 'plugins');

      checkDirectoryExists(path.join(pluginSymlinks, 'url_launcher_ios', 'ios'));

      checkDirectoryExists(path.join(pluginSymlinks, 'test_plugin_swift', 'ios'));

      // test_plugin_objc no longer supports iOS, shouldn't exist!
      checkDirectoryNotExists(path.join(pluginSymlinks, 'test_plugin_objc'));

      return TaskResult.success(null);
    } catch (e) {
      return TaskResult.failure(e.toString());
    } finally {
      rmTree(tempDir);
    }
  });
}

void _validateIosPodfile(String appPath) {
  section('Validate iOS Podfile');

  final File podfileLockFile = File(path.join(appPath, 'ios', 'Podfile.lock'));
  final String podfileLockOutput = podfileLockFile.readAsStringSync();
  if (!podfileLockOutput.contains(':path: ".symlinks/plugins/url_launcher_ios/ios"') ||
      !podfileLockOutput.contains(':path: Flutter') ||
      !podfileLockOutput.contains(':path: ".symlinks/plugins/test_plugin_objc/ios"') ||
      !podfileLockOutput.contains(':path: ".symlinks/plugins/test_plugin_swift/ios"') ||
      podfileLockOutput.contains('url_launcher_macos')) {
    print(podfileLockOutput);
    throw TaskResult.failure('iOS Podfile.lock does not contain expected pods');
  }

  checkDirectoryNotExists(path.join(appPath, 'ios', 'Flutter', 'Flutter.framework'));

  checkFileExists(path.join(appPath, 'ios', 'Flutter', 'Flutter.podspec'));

  final String pluginSymlinks = path.join(appPath, 'ios', '.symlinks', 'plugins');

  checkDirectoryExists(path.join(pluginSymlinks, 'url_launcher_ios', 'ios'));

  checkDirectoryNotExists(path.join(pluginSymlinks, 'url_launcher_macos'));

  checkDirectoryExists(path.join(pluginSymlinks, 'test_plugin_objc', 'ios'));

  checkDirectoryExists(path.join(pluginSymlinks, 'test_plugin_swift', 'ios'));

  // Make sure no Xcode build settings are leaking derived data/build directory into the ios directory.
  checkDirectoryNotExists(path.join(appPath, 'ios', 'build'));
}

void _validateMacOSPodfile(String appPath) {
  section('Validate macOS Podfile');

  final File podfileLockFile = File(path.join(appPath, 'macos', 'Podfile.lock'));
  final String podfileLockOutput = podfileLockFile.readAsStringSync();
  if (!podfileLockOutput.contains(':path: Flutter/ephemeral\n') ||
      !podfileLockOutput.contains(
        ':path: Flutter/ephemeral/.symlinks/plugins/url_launcher_macos/macos',
      ) ||
      !podfileLockOutput.contains(
        ':path: Flutter/ephemeral/.symlinks/plugins/test_plugin_swift/macos',
      ) ||
      podfileLockOutput.contains('url_launcher_ios/')) {
    print(podfileLockOutput);
    throw TaskResult.failure('macOS Podfile.lock does not contain expected pods');
  }

  checkFileExists(path.join(appPath, 'macos', 'Flutter', 'ephemeral', 'FlutterMacOS.podspec'));

  final String pluginSymlinks = path.join(
    appPath,
    'macos',
    'Flutter',
    'ephemeral',
    '.symlinks',
    'plugins',
  );

  checkDirectoryExists(path.join(pluginSymlinks, 'url_launcher_macos', 'macos'));

  checkDirectoryNotExists(path.join(pluginSymlinks, 'url_launcher_ios'));

  checkDirectoryExists(path.join(pluginSymlinks, 'test_plugin_swift', 'macos'));
}

Future<void> _tryMacOSLint(String podspecPath, List<String> extraArguments) async {
  await eval('pod', <String>['lib', 'lint', podspecPath, ...extraArguments]);
}
