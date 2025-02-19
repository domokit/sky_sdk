// Copyright 2014 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

import '../base/analyze_size.dart';
import '../base/common.dart';
import '../build_info.dart';
import '../cache.dart';
import '../features.dart';
import '../globals.dart' as globals;
import '../macos/build_macos.dart';
import '../runner/flutter_command.dart' show FlutterCommandResult;
import 'build.dart';

/// A command to build a macOS desktop target through a build shell script.
class BuildMacosCommand extends BuildSubCommand {
  BuildMacosCommand({required super.logger, required bool verboseHelp})
    : super(verboseHelp: verboseHelp) {
    addCommonDesktopBuildOptions(verboseHelp: verboseHelp);
    usesFlavorOption();
    argParser.addFlag(
      'config-only',
      help:
          'Update the project configuration without performing a build. '
          'This can be used in CI/CD process that create an archive to avoid '
          'performing duplicate work.',
    );
  }

  @override
  final String name = 'macos';

  @override
  bool get hidden => !featureFlags.isMacOSEnabled || !globals.platform.isMacOS;

  @override
  Future<Set<DevelopmentArtifact>> get requiredArtifacts async => <DevelopmentArtifact>{
    DevelopmentArtifact.macOS,
  };

  @override
  String get description => 'Build a macOS desktop application.';

  @override
  bool get supported => globals.platform.isMacOS;

  bool get configOnly => boolArg('config-only');

  @override
  Future<FlutterCommandResult> runCommand() async {
    final BuildInfo buildInfo = await getBuildInfo();
    if (!featureFlags.isMacOSEnabled) {
      throwToolExit(
        '"build macos" is not currently supported. To enable, run "flutter config --enable-macos-desktop".',
      );
    }
    if (!supported) {
      throwToolExit('"build macos" only supported on macOS hosts.');
    }
    await buildMacOS(
      flutterProject: project,
      buildInfo: buildInfo,
      targetOverride: targetFile,
      verboseLogging: globals.logger.isVerbose,
      configOnly: configOnly,
      sizeAnalyzer: SizeAnalyzer(
        fileSystem: globals.fs,
        logger: globals.logger,
        appFilenamePattern: 'App',
        analytics: analytics,
      ),
      usingCISystem: usingCISystem,
    );
    return FlutterCommandResult.success();
  }
}
