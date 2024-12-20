// Copyright 2014 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

import '../artifacts.dart';
import '../base/common.dart';
import '../base/file_system.dart';
import '../build_info.dart';
import '../cache.dart';
import '../flutter_manifest.dart';
import '../globals.dart' as globals;
import '../project.dart';

String flutterMacOSFrameworkDir(BuildMode mode, FileSystem fileSystem, Artifacts artifacts) {
  final String flutterMacOSFramework = artifacts.getArtifactPath(
    Artifact.flutterMacOSFramework,
    platform: TargetPlatform.darwin,
    mode: mode,
  );
  return fileSystem.path.normalize(fileSystem.path.dirname(flutterMacOSFramework));
}

/// Writes or rewrites Xcode property files with the specified information.
///
/// useMacOSConfig: Optional parameter that controls whether we use the macOS
/// project file instead. Defaults to false.
///
/// targetOverride: Optional parameter, if null or unspecified the default value
/// from xcode_backend.sh is used 'lib/main.dart'.
Future<void> updateGeneratedXcodeProperties({
  required FlutterProject project,
  required BuildInfo buildInfo,
  String? targetOverride,
  bool useMacOSConfig = false,
  String? buildDirOverride,
  String? configurationBuildDir,
}) async {
  final List<String> xcodeBuildSettings = await _xcodeBuildSettingsLines(
    project: project,
    buildInfo: buildInfo,
    targetOverride: targetOverride,
    useMacOSConfig: useMacOSConfig,
    buildDirOverride: buildDirOverride,
    configurationBuildDir: configurationBuildDir,
  );

  _updateGeneratedXcodePropertiesFile(
    project: project,
    xcodeBuildSettings: xcodeBuildSettings,
    useMacOSConfig: useMacOSConfig,
  );

  _updateGeneratedEnvironmentVariablesScript(
    project: project,
    xcodeBuildSettings: xcodeBuildSettings,
    useMacOSConfig: useMacOSConfig,
  );
}

/// Generate a xcconfig file to inherit FLUTTER_ build settings
/// for Xcode targets that need them.
/// See [XcodeBasedProject.generatedXcodePropertiesFile].
void _updateGeneratedXcodePropertiesFile({
  required FlutterProject project,
  required List<String> xcodeBuildSettings,
  bool useMacOSConfig = false,
}) {
  final StringBuffer localsBuffer = StringBuffer();

  localsBuffer.writeln('// This is a generated file; do not edit or check into version control.');
  xcodeBuildSettings.forEach(localsBuffer.writeln);
  final File generatedXcodePropertiesFile =
      useMacOSConfig
          ? project.macos.generatedXcodePropertiesFile
          : project.ios.generatedXcodePropertiesFile;

  generatedXcodePropertiesFile.createSync(recursive: true);
  generatedXcodePropertiesFile.writeAsStringSync(localsBuffer.toString());
}

/// Generate a script to export all the FLUTTER_ environment variables needed
/// as flags for Flutter tools.
/// See [XcodeBasedProject.generatedEnvironmentVariableExportScript].
void _updateGeneratedEnvironmentVariablesScript({
  required FlutterProject project,
  required List<String> xcodeBuildSettings,
  bool useMacOSConfig = false,
}) {
  final StringBuffer localsBuffer = StringBuffer();

  localsBuffer.writeln('#!/bin/sh');
  localsBuffer.writeln('# This is a generated file; do not edit or check into version control.');
  for (final String line in xcodeBuildSettings) {
    if (!line.contains('[')) {
      // Exported conditional Xcode build settings do not work.
      localsBuffer.writeln('export "$line"');
    }
  }

  final File generatedModuleBuildPhaseScript =
      useMacOSConfig
          ? project.macos.generatedEnvironmentVariableExportScript
          : project.ios.generatedEnvironmentVariableExportScript;
  generatedModuleBuildPhaseScript.createSync(recursive: true);
  generatedModuleBuildPhaseScript.writeAsStringSync(localsBuffer.toString());
  globals.os.chmod(generatedModuleBuildPhaseScript, '755');
}

/// Build name parsed and validated from build info and manifest. Used for CFBundleShortVersionString.
String? parsedBuildName({required FlutterManifest manifest, BuildInfo? buildInfo}) {
  final String? buildNameToParse = buildInfo?.buildName ?? manifest.buildName;
  return validatedBuildNameForPlatform(TargetPlatform.ios, buildNameToParse, globals.logger);
}

/// Build number parsed and validated from build info and manifest. Used for CFBundleVersion.
String? parsedBuildNumber({required FlutterManifest manifest, BuildInfo? buildInfo}) {
  String? buildNumberToParse = buildInfo?.buildNumber ?? manifest.buildNumber;
  final String? buildNumber = validatedBuildNumberForPlatform(
    TargetPlatform.ios,
    buildNumberToParse,
    globals.logger,
  );
  if (buildNumber != null && buildNumber.isNotEmpty) {
    return buildNumber;
  }
  // Drop back to parsing build name if build number is not present. Build number is optional in the manifest, but
  // FLUTTER_BUILD_NUMBER is required as the backing value for the required CFBundleVersion.
  buildNumberToParse = buildInfo?.buildName ?? manifest.buildName;
  return validatedBuildNumberForPlatform(TargetPlatform.ios, buildNumberToParse, globals.logger);
}

/// List of lines of build settings. Example: 'FLUTTER_BUILD_DIR=build'
Future<List<String>> _xcodeBuildSettingsLines({
  required FlutterProject project,
  required BuildInfo buildInfo,
  String? targetOverride,
  bool useMacOSConfig = false,
  String? buildDirOverride,
  String? configurationBuildDir,
}) async {
  final List<String> xcodeBuildSettings = <String>[];

  final String flutterRoot = globals.fs.path.normalize(Cache.flutterRoot!);
  xcodeBuildSettings.add('FLUTTER_ROOT=$flutterRoot');

  // This holds because requiresProjectRoot is true for this command
  xcodeBuildSettings.add(
    'FLUTTER_APPLICATION_PATH=${globals.fs.path.normalize(project.directory.path)}',
  );

  // Tell CocoaPods behavior to codesign in parallel with rest of scripts to speed it up.
  // Value must be "true", not "YES". https://github.com/CocoaPods/CocoaPods/pull/6088
  xcodeBuildSettings.add('COCOAPODS_PARALLEL_CODE_SIGN=true');

  // Relative to FLUTTER_APPLICATION_PATH, which is [Directory.current].
  if (targetOverride != null) {
    xcodeBuildSettings.add('FLUTTER_TARGET=$targetOverride');
  }

  // The build outputs directory, relative to FLUTTER_APPLICATION_PATH.
  xcodeBuildSettings.add('FLUTTER_BUILD_DIR=${buildDirOverride ?? getBuildDirectory()}');

  final String buildName =
      parsedBuildName(manifest: project.manifest, buildInfo: buildInfo) ?? '1.0.0';
  xcodeBuildSettings.add('FLUTTER_BUILD_NAME=$buildName');

  final String buildNumber =
      parsedBuildNumber(manifest: project.manifest, buildInfo: buildInfo) ?? '1';
  xcodeBuildSettings.add('FLUTTER_BUILD_NUMBER=$buildNumber');

  // CoreDevices in debug and profile mode are launched, but not built, via Xcode.
  // Set the CONFIGURATION_BUILD_DIR so Xcode knows where to find the app
  // bundle to launch.
  if (configurationBuildDir != null) {
    xcodeBuildSettings.add('CONFIGURATION_BUILD_DIR=$configurationBuildDir');
  }

  final LocalEngineInfo? localEngineInfo = globals.artifacts?.localEngineInfo;
  if (localEngineInfo != null) {
    final String engineOutPath = localEngineInfo.targetOutPath;
    xcodeBuildSettings.add(
      'FLUTTER_ENGINE=${globals.fs.path.dirname(globals.fs.path.dirname(engineOutPath))}',
    );

    final String localEngineName = localEngineInfo.localTargetName;
    xcodeBuildSettings.add('LOCAL_ENGINE=$localEngineName');

    final String localEngineHostName = localEngineInfo.localHostName;
    xcodeBuildSettings.add('LOCAL_ENGINE_HOST=$localEngineHostName');

    // Tell Xcode not to build universal binaries for local engines, which are
    // single-architecture.
    //
    // This assumes that local engine binary paths are consistent with
    // the conventions uses in the engine: 32-bit iOS engines are built to
    // paths ending in _arm, 64-bit builds are not.

    String arch;
    if (useMacOSConfig) {
      if (localEngineName.contains('_arm64')) {
        arch = 'arm64';
      } else {
        arch = 'x86_64';
      }
    } else {
      if (localEngineName.endsWith('_arm')) {
        throwToolExit('32-bit iOS local engine binaries are not supported.');
      } else if (localEngineName.contains('_arm64')) {
        arch = 'arm64';
      } else if (localEngineName.contains('_sim')) {
        arch = 'x86_64';
      } else {
        arch = 'arm64';
      }
    }
    xcodeBuildSettings.add('ARCHS=$arch');
  }

  if (!useMacOSConfig) {
    // If any plugins or their dependencies do not support arm64 simulators
    // (to run natively without Rosetta translation on an ARM Mac),
    // the app will fail to build unless it also excludes arm64 simulators.
    String excludedSimulatorArchs = 'i386';
    if (!(await project.ios.pluginsSupportArmSimulator())) {
      excludedSimulatorArchs += ' arm64';
    }
    xcodeBuildSettings.add('EXCLUDED_ARCHS[sdk=iphonesimulator*]=$excludedSimulatorArchs');
    xcodeBuildSettings.add('EXCLUDED_ARCHS[sdk=iphoneos*]=armv7');
  }

  for (final MapEntry<String, String> config in buildInfo.toEnvironmentConfig().entries) {
    xcodeBuildSettings.add('${config.key}=${config.value}');
  }
  return xcodeBuildSettings;
}
