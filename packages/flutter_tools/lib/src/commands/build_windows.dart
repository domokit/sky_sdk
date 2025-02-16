// Copyright 2014 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

import 'package:meta/meta.dart';

import '../base/analyze_size.dart';
import '../base/common.dart';
import '../base/os.dart';
import '../build_info.dart';
import '../cache.dart';
import '../features.dart';
import '../globals.dart' as globals;
import '../runner/flutter_command.dart' show FlutterCommandResult;
import '../windows/build_windows.dart';
import '../windows/visual_studio.dart';
import 'build.dart';

/// A command to build a windows desktop target through a build shell script.
class BuildWindowsCommand extends BuildSubCommand {
  BuildWindowsCommand({
    required super.logger,
    required OperatingSystemUtils operatingSystemUtils,
    bool verboseHelp = false,
  }) : _operatingSystemUtils = operatingSystemUtils,
       super(verboseHelp: verboseHelp) {
    addCommonDesktopBuildOptions(verboseHelp: verboseHelp);
  }

  final OperatingSystemUtils _operatingSystemUtils;

  @override
  final String name = 'windows';

  @override
  bool get hidden => !featureFlags.isWindowsEnabled || !globals.platform.isWindows;

  @override
  Future<Set<DevelopmentArtifact>> get requiredArtifacts async => <DevelopmentArtifact>{
    DevelopmentArtifact.windows,
  };

  @override
  String get description => 'Build a Windows desktop application.';

  @visibleForTesting
  VisualStudio? visualStudioOverride;

  @override
  Future<FlutterCommandResult> runCommand() async {
    final BuildInfo buildInfo = await getBuildInfo();
    if (!featureFlags.isWindowsEnabled) {
      throwToolExit(
        '"build windows" is not currently supported. To enable, run "flutter config --enable-windows-desktop".',
      );
    }
    if (!globals.platform.isWindows) {
      throwToolExit('"build windows" only supported on Windows hosts.');
    }

    final String defaultTargetPlatform =
        (_operatingSystemUtils.hostPlatform == HostPlatform.windows_arm64)
            ? 'windows-arm64'
            : 'windows-x64';
    final TargetPlatform targetPlatform = getTargetPlatformForName(defaultTargetPlatform);

    await buildWindows(
      project.windows,
      buildInfo,
      targetPlatform,
      target: targetFile,
      visualStudioOverride: visualStudioOverride,
      sizeAnalyzer: SizeAnalyzer(
        fileSystem: globals.fs,
        logger: globals.logger,
        appFilenamePattern: 'app.so',
        analytics: analytics,
      ),
    );
    return FlutterCommandResult.success();
  }
}
