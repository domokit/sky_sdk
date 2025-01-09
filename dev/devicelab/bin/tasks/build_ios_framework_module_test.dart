// Copyright 2014 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

import 'dart:io';

import 'package:flutter_devicelab/framework/framework.dart';
import 'package:flutter_devicelab/framework/ios.dart';
import 'package:flutter_devicelab/framework/task_result.dart';
import 'package:flutter_devicelab/framework/utils.dart';
import 'package:path/path.dart' as path;

/// Tests that iOS and macOS .xcframeworks can be built.
Future<void> main() async {
  await task(() async {
    section('Create module project');

    final Directory tempDir = Directory.systemTemp.createTempSync('flutter_module_test.');
    try {
      await inDirectory(tempDir, () async {
        section('Test iOS module template');

        final Directory moduleProjectDir = Directory(path.join(tempDir.path, 'hello_module'));
        await flutter(
          'create',
          options: <String>[
            '--org',
            'io.flutter.devicelab',
            '--template',
            'module',
            'hello_module',
          ],
        );

        await _addPlugin(moduleProjectDir);
        await _testBuildIosFramework(moduleProjectDir, isModule: true);

        section('Test app template');

        final Directory projectDir = Directory(path.join(tempDir.path, 'hello_project'));
        await flutter(
          'create',
          options: <String>['--org', 'io.flutter.devicelab', 'hello_project'],
        );

        await _addPlugin(projectDir);
        await _testBuildIosFramework(projectDir);
        await _testBuildMacOSFramework(projectDir);
      });

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

Future<void> _addPlugin(Directory projectDir) async {
  section('Add plugins');

  final File pubspec = File(path.join(projectDir.path, 'pubspec.yaml'));
  String content = pubspec.readAsStringSync();
  content = content.replaceFirst(
    '\ndependencies:\n',
    '\ndependencies:\n  package_info: 2.0.2\n  connectivity: 3.0.6\n',
  );
  pubspec.writeAsStringSync(content, flush: true);
  await inDirectory(projectDir, () async {
    await flutter('packages', options: <String>['get']);
  });
}

Future<void> _testBuildIosFramework(Directory projectDir, {bool isModule = false}) async {
  // This builds all build modes' frameworks by default
  section('Build iOS app');

  const String outputDirectoryName = 'flutter-frameworks';

  await inDirectory(projectDir, () async {
    await flutter(
      'build',
      options: <String>[
        'ios-framework',
        '--verbose',
        '--output=$outputDirectoryName',
        '--obfuscate',
        '--split-debug-info=symbols',
      ],
    );
  });

  final String outputPath = path.join(projectDir.path, outputDirectoryName);

  checkFileExists(
    path.join(
      outputPath,
      'Debug',
      'Flutter.xcframework',
      'ios-arm64',
      'Flutter.framework',
      'Flutter',
    ),
  );

  final String debugAppFrameworkPath = path.join(
    outputPath,
    'Debug',
    'App.xcframework',
    'ios-arm64',
    'App.framework',
    'App',
  );
  checkFileExists(debugAppFrameworkPath);

  checkFileExists(
    path.join(outputPath, 'Debug', 'App.xcframework', 'ios-arm64', 'App.framework', 'Info.plist'),
  );

  section('Check debug build has Dart snapshot as asset');

  checkFileExists(
    path.join(
      outputPath,
      'Debug',
      'App.xcframework',
      'ios-arm64_x86_64-simulator',
      'App.framework',
      'flutter_assets',
      'vm_snapshot_data',
    ),
  );

  section('Check obfuscation symbols');

  checkFileExists(path.join(projectDir.path, 'symbols', 'app.ios-arm64.symbols'));

  section('Check debug build has no Dart AOT');

  final String aotSymbols = await dumpSymbolTable(debugAppFrameworkPath);

  if (aotSymbols.contains('architecture') || aotSymbols.contains('_kDartVmSnapshot')) {
    throw TaskResult.failure('Debug App.framework contains AOT');
  }

  section('Check profile, release builds has Dart AOT dylib');

  for (final String mode in <String>['Profile', 'Release']) {
    final String appFrameworkPath = path.join(
      outputPath,
      mode,
      'App.xcframework',
      'ios-arm64',
      'App.framework',
      'App',
    );

    await _checkDylib(appFrameworkPath);

    final String aotSymbols = await dumpSymbolTable(appFrameworkPath);

    if (!aotSymbols.contains('_kDartVmSnapshot')) {
      throw TaskResult.failure('$mode App.framework missing Dart AOT');
    }

    checkFileNotExists(
      path.join(
        outputPath,
        mode,
        'App.xcframework',
        'ios-arm64',
        'App.framework',
        'flutter_assets',
        'vm_snapshot_data',
      ),
    );

    final String dsymPath = path.join(outputPath, mode, 'App.xcframework', 'ios-arm64', 'dSYMs');
    checkDirectoryExists(dsymPath);

    final String appFrameworkDsymPath = path.join(dsymPath, 'App.framework.dSYM');
    checkDirectoryExists(appFrameworkDsymPath);

    if (Directory(dsymPath).listSync().whereType<Directory>().length != 1) {
      throw TaskResult.failure('App.framework/dSYMs should ONLY contain App.xcframework.dSYM');
    }

    await _checkDsym(path.join(appFrameworkDsymPath, 'Contents', 'Resources', 'DWARF', 'App'));

    checkFileExists(
      path.join(
        outputPath,
        mode,
        'App.xcframework',
        'ios-arm64_x86_64-simulator',
        'App.framework',
        'App',
      ),
    );

    checkFileExists(
      path.join(
        outputPath,
        mode,
        'App.xcframework',
        'ios-arm64_x86_64-simulator',
        'App.framework',
        'Info.plist',
      ),
    );
  }

  section("Check all modes' engine dylib");

  for (final String mode in <String>['Debug', 'Profile', 'Release']) {
    checkFileExists(
      path.join(
        outputPath,
        mode,
        'Flutter.xcframework',
        'ios-arm64',
        'Flutter.framework',
        'Flutter',
      ),
    );

    checkFileExists(
      path.join(
        outputPath,
        mode,
        'Flutter.xcframework',
        'ios-arm64_x86_64-simulator',
        'Flutter.framework',
        'Flutter',
      ),
    );

    checkFileExists(
      path.join(
        outputPath,
        mode,
        'Flutter.xcframework',
        'ios-arm64_x86_64-simulator',
        'Flutter.framework',
        'Headers',
        'Flutter.h',
      ),
    );
  }

  section('Check all modes have plugins');

  for (final String mode in <String>['Debug', 'Profile', 'Release']) {
    final String pluginFrameworkPath = path.join(
      outputPath,
      mode,
      'connectivity.xcframework',
      'ios-arm64',
      'connectivity.framework',
      'connectivity',
    );

    await _checkDylib(pluginFrameworkPath);
    if (!await _linksOnFlutter(pluginFrameworkPath)) {
      throw TaskResult.failure('$pluginFrameworkPath does not link on Flutter');
    }

    final String transitiveDependencyFrameworkPath = path.join(
      outputPath,
      mode,
      'Reachability.xcframework',
      'ios-arm64',
      'Reachability.framework',
      'Reachability',
    );

    if (!exists(File(transitiveDependencyFrameworkPath))) {
      throw TaskResult.failure('Expected debug Flutter engine artifact binary to exist');
    }

    if (await _linksOnFlutter(transitiveDependencyFrameworkPath)) {
      throw TaskResult.failure(
        'Transitive dependency $transitiveDependencyFrameworkPath unexpectedly links on Flutter',
      );
    }

    checkFileExists(
      path.join(
        outputPath,
        mode,
        'connectivity.xcframework',
        'ios-arm64',
        'connectivity.framework',
        'Headers',
        'FLTConnectivityPlugin.h',
      ),
    );

    if (mode != 'Debug') {
      checkDirectoryExists(
        path.join(
          outputPath,
          mode,
          'connectivity.xcframework',
          'ios-arm64',
          'dSYMs',
          'connectivity.framework.dSYM',
        ),
      );
    }

    final String simulatorFrameworkPath = path.join(
      outputPath,
      mode,
      'connectivity.xcframework',
      'ios-arm64_x86_64-simulator',
      'connectivity.framework',
      'connectivity',
    );

    final String simulatorFrameworkHeaderPath = path.join(
      outputPath,
      mode,
      'connectivity.xcframework',
      'ios-arm64_x86_64-simulator',
      'connectivity.framework',
      'Headers',
      'FLTConnectivityPlugin.h',
    );

    checkFileExists(simulatorFrameworkPath);
    checkFileExists(simulatorFrameworkHeaderPath);
  }

  section('Check all modes have generated plugin registrant');

  for (final String mode in <String>['Debug', 'Profile', 'Release']) {
    if (!isModule) {
      continue;
    }
    final String registrantFrameworkPath = path.join(
      outputPath,
      mode,
      'FlutterPluginRegistrant.xcframework',
      'ios-arm64',
      'FlutterPluginRegistrant.framework',
      'FlutterPluginRegistrant',
    );
    await _checkStatic(registrantFrameworkPath);

    checkFileExists(
      path.join(
        outputPath,
        mode,
        'FlutterPluginRegistrant.xcframework',
        'ios-arm64',
        'FlutterPluginRegistrant.framework',
        'Headers',
        'GeneratedPluginRegistrant.h',
      ),
    );
    final String simulatorHeaderPath = path.join(
      outputPath,
      mode,
      'FlutterPluginRegistrant.xcframework',
      'ios-arm64_x86_64-simulator',
      'FlutterPluginRegistrant.framework',
      'Headers',
      'GeneratedPluginRegistrant.h',
    );
    checkFileExists(simulatorHeaderPath);
  }

  // This builds all build modes' frameworks by default
  section('Build podspec and static plugins');

  const String cocoapodsOutputDirectoryName = 'flutter-frameworks-cocoapods';

  await inDirectory(projectDir, () async {
    await flutter(
      'build',
      options: <String>[
        'ios-framework',
        '--cocoapods',
        '--force', // Allow podspec creation on master.
        '--output=$cocoapodsOutputDirectoryName',
        '--static',
      ],
    );
  });

  final String cocoapodsOutputPath = path.join(projectDir.path, cocoapodsOutputDirectoryName);
  for (final String mode in <String>['Debug', 'Profile', 'Release']) {
    checkFileExists(path.join(cocoapodsOutputPath, mode, 'Flutter.podspec'));
    await _checkDylib(
      path.join(cocoapodsOutputPath, mode, 'App.xcframework', 'ios-arm64', 'App.framework', 'App'),
    );

    if (mode != 'Debug') {
      final String appFrameworkDsymPath = path.join(
        cocoapodsOutputPath,
        mode,
        'App.xcframework',
        'ios-arm64',
        'dSYMs',
        'App.framework.dSYM',
      );
      checkDirectoryExists(appFrameworkDsymPath);
      await _checkDsym(path.join(appFrameworkDsymPath, 'Contents', 'Resources', 'DWARF', 'App'));
    }

    if (Directory(
          path.join(cocoapodsOutputPath, mode, 'FlutterPluginRegistrant.xcframework'),
        ).existsSync() !=
        isModule) {
      throw TaskResult.failure('Unexpected FlutterPluginRegistrant.xcframework.');
    }

    await _checkStatic(
      path.join(
        cocoapodsOutputPath,
        mode,
        'package_info.xcframework',
        'ios-arm64',
        'package_info.framework',
        'package_info',
      ),
    );

    await _checkStatic(
      path.join(
        cocoapodsOutputPath,
        mode,
        'connectivity.xcframework',
        'ios-arm64',
        'connectivity.framework',
        'connectivity',
      ),
    );

    checkDirectoryExists(path.join(cocoapodsOutputPath, mode, 'Reachability.xcframework'));
  }

  if (File(path.join(outputPath, 'GeneratedPluginRegistrant.h')).existsSync() == isModule) {
    throw TaskResult.failure('Unexpected GeneratedPluginRegistrant.h.');
  }

  if (File(path.join(outputPath, 'GeneratedPluginRegistrant.m')).existsSync() == isModule) {
    throw TaskResult.failure('Unexpected GeneratedPluginRegistrant.m.');
  }

  section('Build frameworks without plugins');
  await _testBuildFrameworksWithoutPlugins(projectDir, platform: 'ios');

  section('check --static cannot be used with the --no-plugins flag');
  await _testStaticAndNoPlugins(projectDir);
}

Future<void> _testBuildMacOSFramework(Directory projectDir) async {
  // This builds all build modes' frameworks by default
  section('Build macOS frameworks');

  const String outputDirectoryName = 'flutter-frameworks';

  await inDirectory(projectDir, () async {
    await flutter(
      'build',
      options: <String>[
        'macos-framework',
        '--verbose',
        '--output=$outputDirectoryName',
        '--obfuscate',
        '--split-debug-info=symbols',
      ],
    );
  });

  final String outputPath = path.join(projectDir.path, outputDirectoryName);
  final String flutterFramework = path.join(
    outputPath,
    'Debug',
    'FlutterMacOS.xcframework',
    'macos-arm64_x86_64',
    'FlutterMacOS.framework',
  );
  checkDirectoryExists(flutterFramework);

  final String debugAppFrameworkPath = path.join(
    outputPath,
    'Debug',
    'App.xcframework',
    'macos-arm64_x86_64',
    'App.framework',
    'App',
  );
  checkSymlinkExists(debugAppFrameworkPath);

  checkFileExists(
    path.join(
      outputPath,
      'Debug',
      'App.xcframework',
      'macos-arm64_x86_64',
      'App.framework',
      'Resources',
      'Info.plist',
    ),
  );

  section('Check debug build has Dart snapshot as asset');

  checkFileExists(
    path.join(
      outputPath,
      'Debug',
      'App.xcframework',
      'macos-arm64_x86_64',
      'App.framework',
      'Resources',
      'flutter_assets',
      'vm_snapshot_data',
    ),
  );

  section('Check obfuscation symbols');

  checkFileExists(path.join(projectDir.path, 'symbols', 'app.darwin-arm64.symbols'));

  checkFileExists(path.join(projectDir.path, 'symbols', 'app.darwin-x86_64.symbols'));

  section('Check debug build has no Dart AOT');

  final String aotSymbols = await dumpSymbolTable(debugAppFrameworkPath);

  if (aotSymbols.contains('architecture') || aotSymbols.contains('_kDartVmSnapshot')) {
    throw TaskResult.failure('Debug App.framework contains AOT');
  }

  section('Check profile, release builds has Dart AOT dylib');

  for (final String mode in <String>['Profile', 'Release']) {
    final String appFrameworkPath = path.join(
      outputPath,
      mode,
      'App.xcframework',
      'macos-arm64_x86_64',
      'App.framework',
      'App',
    );

    await _checkDylib(appFrameworkPath);

    final String aotSymbols = await dumpSymbolTable(appFrameworkPath);

    if (!aotSymbols.contains('_kDartVmSnapshot')) {
      throw TaskResult.failure('$mode App.framework missing Dart AOT');
    }

    checkFileNotExists(
      path.join(
        outputPath,
        mode,
        'App.xcframework',
        'macos-arm64_x86_64',
        'App.framework',
        'Resources',
        'flutter_assets',
        'vm_snapshot_data',
      ),
    );

    checkFileExists(
      path.join(
        outputPath,
        mode,
        'App.xcframework',
        'macos-arm64_x86_64',
        'App.framework',
        'Resources',
        'Info.plist',
      ),
    );

    final String appFrameworkDsymPath = path.join(
      outputPath,
      mode,
      'App.xcframework',
      'macos-arm64_x86_64',
      'dSYMs',
      'App.framework.dSYM',
    );
    checkDirectoryExists(appFrameworkDsymPath);
    await _checkDsym(path.join(appFrameworkDsymPath, 'Contents', 'Resources', 'DWARF', 'App'));
  }

  section("Check all modes' engine dylib");

  for (final String mode in <String>['Debug', 'Profile', 'Release']) {
    final String engineBinary = path.join(
      outputPath,
      mode,
      'FlutterMacOS.xcframework',
      'macos-arm64_x86_64',
      'FlutterMacOS.framework',
      'FlutterMacOS',
    );
    checkSymlinkExists(engineBinary);

    checkFileExists(
      path.join(
        outputPath,
        mode,
        'FlutterMacOS.xcframework',
        'macos-arm64_x86_64',
        'FlutterMacOS.framework',
        'Headers',
        'FlutterMacOS.h',
      ),
    );
  }

  section('Check all modes have plugins');

  for (final String mode in <String>['Debug', 'Profile', 'Release']) {
    final String pluginFrameworkPath = path.join(
      outputPath,
      mode,
      'connectivity_macos.xcframework',
      'macos-arm64_x86_64',
      'connectivity_macos.framework',
      'connectivity_macos',
    );

    await _checkDylib(pluginFrameworkPath);
    if (!await _linksOnFlutterMacOS(pluginFrameworkPath)) {
      throw TaskResult.failure('$pluginFrameworkPath does not link on Flutter');
    }

    final String transitiveDependencyFrameworkPath = path.join(
      outputPath,
      mode,
      'Reachability.xcframework',
      'macos-arm64_x86_64',
      'Reachability.framework',
      'Reachability',
    );
    if (await _linksOnFlutterMacOS(transitiveDependencyFrameworkPath)) {
      throw TaskResult.failure(
        'Transitive dependency $transitiveDependencyFrameworkPath unexpectedly links on Flutter',
      );
    }

    checkFileExists(
      path.join(
        outputPath,
        mode,
        'connectivity_macos.xcframework',
        'macos-arm64_x86_64',
        'connectivity_macos.framework',
        'Headers',
        'connectivity_macos-Swift.h',
      ),
    );

    checkDirectoryExists(
      path.join(
        outputPath,
        mode,
        'connectivity_macos.xcframework',
        'macos-arm64_x86_64',
        'connectivity_macos.framework',
        'Modules',
        'connectivity_macos.swiftmodule',
      ),
    );

    if (mode != 'Debug') {
      checkDirectoryExists(
        path.join(
          outputPath,
          mode,
          'connectivity_macos.xcframework',
          'macos-arm64_x86_64',
          'dSYMs',
          'connectivity_macos.framework.dSYM',
        ),
      );
    }

    checkSymlinkExists(
      path.join(
        outputPath,
        mode,
        'connectivity_macos.xcframework',
        'macos-arm64_x86_64',
        'connectivity_macos.framework',
        'connectivity_macos',
      ),
    );
  }

  // This builds all build modes' frameworks by default
  section('Build podspec and static plugins');

  const String cocoapodsOutputDirectoryName = 'flutter-frameworks-cocoapods';

  await inDirectory(projectDir, () async {
    await flutter(
      'build',
      options: <String>[
        'macos-framework',
        '--cocoapods',
        '--force', // Allow podspec creation on master.
        '--output=$cocoapodsOutputDirectoryName',
        '--static',
      ],
    );
  });

  final String cocoapodsOutputPath = path.join(projectDir.path, cocoapodsOutputDirectoryName);
  for (final String mode in <String>['Debug', 'Profile', 'Release']) {
    checkFileExists(path.join(cocoapodsOutputPath, mode, 'FlutterMacOS.podspec'));
    await _checkDylib(
      path.join(
        cocoapodsOutputPath,
        mode,
        'App.xcframework',
        'macos-arm64_x86_64',
        'App.framework',
        'App',
      ),
    );

    if (mode != 'Debug') {
      final String appFrameworkDsymPath = path.join(
        cocoapodsOutputPath,
        mode,
        'App.xcframework',
        'macos-arm64_x86_64',
        'dSYMs',
        'App.framework.dSYM',
      );
      checkDirectoryExists(appFrameworkDsymPath);
      await _checkDsym(path.join(appFrameworkDsymPath, 'Contents', 'Resources', 'DWARF', 'App'));
    }

    await _checkStatic(
      path.join(
        cocoapodsOutputPath,
        mode,
        'package_info.xcframework',
        'macos-arm64_x86_64',
        'package_info.framework',
        'package_info',
      ),
    );

    await _checkStatic(
      path.join(
        cocoapodsOutputPath,
        mode,
        'connectivity_macos.xcframework',
        'macos-arm64_x86_64',
        'connectivity_macos.framework',
        'connectivity_macos',
      ),
    );

    checkDirectoryExists(path.join(cocoapodsOutputPath, mode, 'Reachability.xcframework'));
  }

  checkFileExists(path.join(outputPath, 'GeneratedPluginRegistrant.swift'));

  section('Validate embed FlutterMacOS.framework with CocoaPods');

  final File podspec = File(path.join(cocoapodsOutputPath, 'Debug', 'FlutterMacOS.podspec'));

  podspec.writeAsStringSync(podspec.readAsStringSync().replaceFirst('null.null.0', '0.0.0'));

  final Directory macosDirectory = Directory(path.join(projectDir.path, 'macos'));
  final File podfile = File(path.join(macosDirectory.path, 'Podfile'));
  final String currentPodfile = podfile.readAsStringSync();

  // Temporarily test Add-to-App Cocoapods podspec for framework
  podfile.writeAsStringSync('''
target 'Runner' do
  # Comment the next line if you don't want to use dynamic frameworks
  use_frameworks!

  pod 'FlutterMacOS', :podspec => '${podspec.path}'
end
''');
  await inDirectory(macosDirectory, () async {
    await eval('pod', <String>['install']);
  });

  // Change podfile back to original
  podfile.writeAsStringSync(currentPodfile);
  await inDirectory(macosDirectory, () async {
    await eval('pod', <String>['install']);
  });

  section('Build frameworks without plugins');
  await _testBuildFrameworksWithoutPlugins(projectDir, platform: 'macos');
}

Future<void> _testBuildFrameworksWithoutPlugins(
  Directory projectDir, {
  required String platform,
}) async {
  const String noPluginsOutputDir = 'flutter-frameworks-no-plugins';

  await inDirectory(projectDir, () async {
    await flutter(
      'build',
      options: <String>[
        '$platform-framework',
        '--cocoapods',
        '--force', // Allow podspec creation on master.
        '--output=$noPluginsOutputDir',
        '--no-plugins',
      ],
    );
  });

  final String noPluginsOutputPath = path.join(projectDir.path, noPluginsOutputDir);
  for (final String mode in <String>['Debug', 'Profile', 'Release']) {
    checkFileExists(
      path.join(noPluginsOutputPath, mode, 'Flutter${platform == 'macos' ? 'MacOS' : ''}.podspec'),
    );
    checkDirectoryExists(path.join(noPluginsOutputPath, mode, 'App.xcframework'));

    checkDirectoryNotExists(path.join(noPluginsOutputPath, mode, 'package_info.xcframework'));

    checkDirectoryNotExists(path.join(noPluginsOutputPath, mode, 'connectivity.xcframework'));

    checkDirectoryNotExists(path.join(noPluginsOutputPath, mode, 'Reachability.xcframework'));
  }
}

Future<void> _testStaticAndNoPlugins(Directory projectDir) async {
  const String noPluginsOutputDir = 'flutter-frameworks-no-plugins-static';
  final ProcessResult result = await inDirectory(projectDir, () async {
    return executeFlutter(
      'build',
      options: <String>[
        'ios-framework',
        '--cocoapods',
        '--force', // Allow podspec creation on master.
        '--output=$noPluginsOutputDir',
        '--no-plugins',
        '--static',
      ],
      canFail: true,
    );
  });
  if (result.exitCode == 0) {
    throw TaskResult.failure('Build framework command did not exit with error as expected');
  }
  final String output = '${result.stdout}\n${result.stderr}';
  if (!output.contains('--static cannot be used with the --no-plugins flag')) {
    throw TaskResult.failure(output);
  }
}

Future<void> _checkDylib(String pathToLibrary) async {
  final String binaryFileType = await fileType(pathToLibrary);
  if (!binaryFileType.contains('dynamically linked')) {
    throw TaskResult.failure('$pathToLibrary is not a dylib, found: $binaryFileType');
  }
}

Future<void> _checkDsym(String pathToSymbolFile) async {
  final String binaryFileType = await fileType(pathToSymbolFile);
  if (!binaryFileType.contains('dSYM companion file')) {
    throw TaskResult.failure('$pathToSymbolFile is not a dSYM, found: $binaryFileType');
  }
}

Future<void> _checkStatic(String pathToLibrary) async {
  final String binaryFileType = await fileType(pathToLibrary);
  if (!binaryFileType.contains('current ar archive random library')) {
    throw TaskResult.failure('$pathToLibrary is not a static library, found: $binaryFileType');
  }
}

Future<bool> _linksOnFlutter(String pathToBinary) async {
  final String loadCommands = await eval('otool', <String>['-l', '-arch', 'arm64', pathToBinary]);
  return loadCommands.contains('Flutter.framework');
}

Future<bool> _linksOnFlutterMacOS(String pathToBinary) async {
  final String loadCommands = await eval('otool', <String>['-l', '-arch', 'arm64', pathToBinary]);
  return loadCommands.contains('FlutterMacOS.framework');
}
