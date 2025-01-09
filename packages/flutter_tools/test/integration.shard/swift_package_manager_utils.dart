// Copyright 2014 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

import 'dart:convert';

import 'package:flutter_tools/src/base/file_system.dart';
import 'package:flutter_tools/src/base/io.dart';

import '../src/common.dart';
import 'test_utils.dart';

class SwiftPackageManagerUtils {
  static Future<void> enableSwiftPackageManager(String flutterBin, String workingDirectory) async {
    final ProcessResult result = await processManager.run(<String>[
      flutterBin,
      ...getLocalEngineArguments(),
      'config',
      '--enable-swift-package-manager',
      '-v',
    ], workingDirectory: workingDirectory);
    expect(
      result.exitCode,
      0,
      reason:
          'Failed to enable Swift Package Manager: \n'
          'stdout: \n${result.stdout}\n'
          'stderr: \n${result.stderr}\n',
      verbose: true,
    );
  }

  static Future<void> disableSwiftPackageManager(String flutterBin, String workingDirectory) async {
    final ProcessResult result = await processManager.run(<String>[
      flutterBin,
      ...getLocalEngineArguments(),
      'config',
      '--no-enable-swift-package-manager',
      '-v',
    ], workingDirectory: workingDirectory);
    expect(
      result.exitCode,
      0,
      reason:
          'Failed to disable Swift Package Manager: \n'
          'stdout: \n${result.stdout}\n'
          'stderr: \n${result.stderr}\n',
      verbose: true,
    );
  }

  static Future<String> createApp(
    String flutterBin,
    String workingDirectory, {
    required String platform,
    required String iosLanguage,
    required List<String> options,
    bool usesSwiftPackageManager = false,
  }) async {
    final String appTemplateType = usesSwiftPackageManager ? 'spm' : 'default';

    final String appName = '${platform}_${iosLanguage}_${appTemplateType}_app';
    final ProcessResult result = await processManager.run(<String>[
      flutterBin,
      ...getLocalEngineArguments(),
      'create',
      '--org',
      'io.flutter.devicelab',
      '-i',
      iosLanguage,
      ...options,
      appName,
    ], workingDirectory: workingDirectory);

    expect(
      result.exitCode,
      0,
      reason:
          'Failed to create app: \n'
          'stdout: \n${result.stdout}\n'
          'stderr: \n${result.stderr}\n',
    );

    return fileSystem.path.join(workingDirectory, appName);
  }

  static Future<void> buildApp(
    String flutterBin,
    String workingDirectory, {
    required List<String> options,
    List<Pattern>? expectedLines,
    List<String>? unexpectedLines,
  }) async {
    final List<Pattern> remainingExpectedLines = expectedLines ?? <Pattern>[];
    final List<String> unexpectedLinesFound = <String>[];
    final List<String> command = <String>[
      flutterBin,
      ...getLocalEngineArguments(),
      'build',
      ...options,
    ];

    final ProcessResult result = await processManager.run(
      command,
      workingDirectory: workingDirectory,
    );

    final List<String> stdout = LineSplitter.split(result.stdout.toString()).toList();
    final List<String> stderr = LineSplitter.split(result.stderr.toString()).toList();
    final List<String> output = stdout + stderr;
    for (final String line in output) {
      // Remove "[   +3 ms] " prefix
      String trimmedLine = line.trim();
      if (trimmedLine.startsWith('[')) {
        final int prefixEndIndex = trimmedLine.indexOf(']');
        if (prefixEndIndex > 0) {
          trimmedLine = trimmedLine.substring(prefixEndIndex + 1, trimmedLine.length).trim();
        }
      }
      remainingExpectedLines.remove(trimmedLine);
      remainingExpectedLines.removeWhere(
        (Pattern expectedLine) => trimmedLine.contains(expectedLine),
      );
      if (unexpectedLines != null) {
        if (unexpectedLines
                .where((String unexpectedLine) => trimmedLine.contains(unexpectedLine))
                .firstOrNull !=
            null) {
          unexpectedLinesFound.add(trimmedLine);
        }
      }
    }
    expect(
      result.exitCode,
      0,
      reason:
          'Failed to build app for "${command.join(' ')}":\n'
          'stdout: \n${result.stdout}\n'
          'stderr: \n${result.stderr}\n',
    );
    expect(
      remainingExpectedLines,
      isEmpty,
      reason:
          'Did not find expected lines for "${command.join(' ')}":\n'
          'stdout: \n${result.stdout}\n'
          'stderr: \n${result.stderr}\n',
    );
    expect(
      unexpectedLinesFound,
      isEmpty,
      reason:
          'Found unexpected lines for "${command.join(' ')}":\n'
          'stdout: \n${result.stdout}\n'
          'stderr: \n${result.stderr}\n',
    );
  }

  static Future<void> cleanApp(String flutterBin, String workingDirectory) async {
    final ProcessResult result = await processManager.run(<String>[
      flutterBin,
      ...getLocalEngineArguments(),
      'clean',
    ], workingDirectory: workingDirectory);
    expect(
      result.exitCode,
      0,
      reason:
          'Failed to clean app: \n'
          'stdout: \n${result.stdout}\n'
          'stderr: \n${result.stderr}\n',
    );
  }

  static Future<SwiftPackageManagerPlugin> createPlugin(
    String flutterBin,
    String workingDirectory, {
    required String platform,
    required String iosLanguage,
    bool usesSwiftPackageManager = false,
  }) async {
    final String dependencyManager = usesSwiftPackageManager ? 'spm' : 'cocoapods';

    // Create plugin
    final String pluginName = '${platform}_${iosLanguage}_${dependencyManager}_plugin';
    final ProcessResult result = await processManager.run(<String>[
      flutterBin,
      ...getLocalEngineArguments(),
      'create',
      '--org',
      'io.flutter.devicelab',
      '--template=plugin',
      '--platforms=$platform',
      '-i',
      iosLanguage,
      pluginName,
    ], workingDirectory: workingDirectory);

    expect(
      result.exitCode,
      0,
      reason:
          'Failed to create plugin: \n'
          'stdout: \n${result.stdout}\n'
          'stderr: \n${result.stderr}\n',
    );

    final Directory pluginDirectory = fileSystem.directory(
      fileSystem.path.join(workingDirectory, pluginName),
    );

    return SwiftPackageManagerPlugin(
      pluginName: pluginName,
      pluginPath: pluginDirectory.path,
      platform: platform,
    );
  }

  static void addDependency({
    required SwiftPackageManagerPlugin plugin,
    required String appDirectoryPath,
  }) {
    final File pubspec = fileSystem.file(fileSystem.path.join(appDirectoryPath, 'pubspec.yaml'));
    final String pubspecContent = pubspec.readAsStringSync();
    pubspec.writeAsStringSync(
      pubspecContent.replaceFirst(
        '\ndependencies:\n',
        '\ndependencies:\n  ${plugin.pluginName}:\n    path: ${plugin.pluginPath}\n',
      ),
    );
  }

  static void removeDependency({
    required SwiftPackageManagerPlugin plugin,
    required String appDirectoryPath,
  }) {
    final File pubspec = fileSystem.file(fileSystem.path.join(appDirectoryPath, 'pubspec.yaml'));
    final String pubspecContent = pubspec.readAsStringSync();
    final String updatedPubspecContent = pubspecContent.replaceFirst(
      '\n  ${plugin.pluginName}:\n    path: ${plugin.pluginPath}\n',
      '\n',
    );

    expect(updatedPubspecContent, isNot(pubspecContent));

    pubspec.writeAsStringSync(updatedPubspecContent);
  }

  static void disableSwiftPackageManagerByPubspec({required String appDirectoryPath}) {
    final File pubspec = fileSystem.file(fileSystem.path.join(appDirectoryPath, 'pubspec.yaml'));
    final String pubspecContent = pubspec.readAsStringSync();
    pubspec.writeAsStringSync(
      pubspecContent.replaceFirst(
        '\n# The following section is specific to Flutter packages.\nflutter:\n',
        '\n# The following section is specific to Flutter packages.\nflutter:\n  disable-swift-package-manager: true',
      ),
    );
  }

  static SwiftPackageManagerPlugin integrationTestPlugin(String platform) {
    final String flutterRoot = getFlutterRoot();
    return SwiftPackageManagerPlugin(
      platform: platform,
      pluginName: (platform == 'ios') ? 'integration_test' : 'integration_test_macos',
      pluginPath:
          (platform == 'ios')
              ? fileSystem.path.join(flutterRoot, 'packages', 'integration_test')
              : fileSystem.path.join(
                flutterRoot,
                'packages',
                'integration_test',
                'integration_test_macos',
              ),
    );
  }

  static List<Pattern> expectedLines({
    required String platform,
    required String appDirectoryPath,
    SwiftPackageManagerPlugin? cocoaPodsPlugin,
    SwiftPackageManagerPlugin? swiftPackagePlugin,
    bool swiftPackageMangerEnabled = false,
    bool migrated = false,
  }) {
    final String frameworkName = platform == 'ios' ? 'Flutter' : 'FlutterMacOS';
    final String appPlatformDirectoryPath = fileSystem.path.join(appDirectoryPath, platform);

    final List<Pattern> expectedLines = <Pattern>[];
    if (swiftPackageMangerEnabled) {
      expectedLines.addAll(<String>[
        'FlutterGeneratedPluginSwiftPackage: $appPlatformDirectoryPath/Flutter/ephemeral/Packages/FlutterGeneratedPluginSwiftPackage',
        "➜ Explicit dependency on target 'FlutterGeneratedPluginSwiftPackage' in project 'FlutterGeneratedPluginSwiftPackage'",
      ]);
    }
    if (swiftPackagePlugin != null) {
      // If using a Swift Package plugin, but Swift Package Manager is not enabled, it falls back to being used as a CocoaPods plugin.
      if (swiftPackageMangerEnabled) {
        expectedLines.addAll(<Pattern>[
          RegExp(
            '${swiftPackagePlugin.pluginName}: [/private]*${swiftPackagePlugin.pluginPath}/$platform/${swiftPackagePlugin.pluginName} @ local',
          ),
          "➜ Explicit dependency on target '${swiftPackagePlugin.pluginName}' in project '${swiftPackagePlugin.pluginName}'",
        ]);
      } else {
        expectedLines.addAll(<String>[
          '-> Installing ${swiftPackagePlugin.pluginName} (0.0.1)',
          "➜ Explicit dependency on target '${swiftPackagePlugin.pluginName}' in project 'Pods'",
        ]);
      }
    }
    if (cocoaPodsPlugin != null) {
      expectedLines.addAll(<String>[
        'Running pod install...',
        '-> Installing $frameworkName (1.0.0)',
        '-> Installing ${cocoaPodsPlugin.pluginName} (0.0.1)',
        "Target 'Pods-Runner' in project 'Pods'",
        "➜ Explicit dependency on target '$frameworkName' in project 'Pods'",
        "➜ Explicit dependency on target '${cocoaPodsPlugin.pluginName}' in project 'Pods'",
      ]);
    }
    if (migrated) {
      expectedLines.addAll(<String>[
        'Adding Swift Package Manager integration...',
        'Running pod install...',
        "Target 'Pods-Runner' in project 'Pods'",
      ]);
    }
    return expectedLines;
  }

  static List<String> unexpectedLines({
    required String platform,
    required String appDirectoryPath,
    SwiftPackageManagerPlugin? cocoaPodsPlugin,
    SwiftPackageManagerPlugin? swiftPackagePlugin,
    bool swiftPackageMangerEnabled = false,
    bool migrated = false,
  }) {
    final String frameworkName = platform == 'ios' ? 'Flutter' : 'FlutterMacOS';
    final List<String> unexpectedLines = <String>[];
    if (cocoaPodsPlugin == null && !migrated) {
      unexpectedLines.addAll(<String>[
        'Running pod install...',
        '-> Installing $frameworkName (1.0.0)',
        "Target 'Pods-Runner' in project 'Pods'",
      ]);
    }
    if (swiftPackagePlugin != null) {
      if (swiftPackageMangerEnabled) {
        unexpectedLines.addAll(<String>[
          '-> Installing ${swiftPackagePlugin.pluginName} (0.0.1)',
          "➜ Explicit dependency on target '${swiftPackagePlugin.pluginName}' in project 'Pods'",
        ]);
      } else {
        unexpectedLines.addAll(<String>[
          '${swiftPackagePlugin.pluginName}: ${swiftPackagePlugin.pluginPath}/$platform/${swiftPackagePlugin.pluginName} @ local',
          "➜ Explicit dependency on target '${swiftPackagePlugin.pluginName}' in project '${swiftPackagePlugin.pluginName}'",
        ]);
      }
    }
    if (!migrated) {
      unexpectedLines.addAll(<String>['Adding Swift Package Manager integration...']);
    }
    return unexpectedLines;
  }
}

class SwiftPackageManagerPlugin {
  SwiftPackageManagerPlugin({
    required this.pluginName,
    required this.pluginPath,
    required this.platform,
  });

  final String pluginName;
  final String pluginPath;
  final String platform;
  String get exampleAppPath => fileSystem.path.join(pluginPath, 'example');
  String get exampleAppPlatformPath => fileSystem.path.join(exampleAppPath, platform);
  String get swiftPackagePlatformPath => fileSystem.path.join(pluginPath, platform, pluginName);
}
