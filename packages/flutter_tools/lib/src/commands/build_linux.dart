// Copyright 2014 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

import '../base/analyze_size.dart';
import '../base/common.dart';
import '../base/logger.dart';
import '../base/os.dart';
import '../build_info.dart';
import '../cache.dart';
import '../features.dart';
import '../globals.dart' as globals;
import '../linux/build_linux.dart';
import '../runner/flutter_command.dart' show FlutterCommandResult;
import 'build.dart';

/// A command to build a linux desktop target through a build shell script.
class BuildLinuxCommand extends BuildSubCommand {
  BuildLinuxCommand({
    required super.logger,
    required OperatingSystemUtils operatingSystemUtils,
    bool verboseHelp = false,
  }) : _operatingSystemUtils = operatingSystemUtils,
       super(verboseHelp: verboseHelp) {
    addCommonDesktopBuildOptions(verboseHelp: verboseHelp);
    final String defaultTargetPlatform =
        (_operatingSystemUtils.hostPlatform == HostPlatform.linux_arm64)
            ? 'linux-arm64'
            : 'linux-x64';
    argParser.addOption(
      'target-platform',
      defaultsTo: defaultTargetPlatform,
      allowed: <String>['linux-arm64', 'linux-x64'],
      help: 'The target platform for which the app is compiled.',
    );
    argParser.addOption(
      'target-sysroot',
      defaultsTo: '/',
      help:
          'The root filesystem path of target platform for which '
          'the app is compiled. This option is valid only '
          'if the current host and target architectures are different.',
    );
  }

  final OperatingSystemUtils _operatingSystemUtils;

  @override
  final String name = 'linux';

  @override
  bool get hidden => !featureFlags.isLinuxEnabled || !globals.platform.isLinux;

  @override
  Future<Set<DevelopmentArtifact>> get requiredArtifacts async => <DevelopmentArtifact>{
    DevelopmentArtifact.linux,
  };

  @override
  String get description => 'Build a Linux desktop application.';

  @override
  Future<FlutterCommandResult> runCommand() async {
    final BuildInfo buildInfo = await getBuildInfo();
    final TargetPlatform targetPlatform = getTargetPlatformForName(stringArg('target-platform')!);
    final bool needCrossBuild =
        _operatingSystemUtils.hostPlatform.platformName != targetPlatform.simpleName;

    if (!featureFlags.isLinuxEnabled) {
      throwToolExit(
        '"build linux" is not currently supported. To enable, run "flutter config --enable-linux-desktop".',
      );
    }
    if (!globals.platform.isLinux) {
      throwToolExit('"build linux" only supported on Linux hosts.');
    }
    // Cross-building for x64 targets on arm64 hosts is not supported.
    if (_operatingSystemUtils.hostPlatform != HostPlatform.linux_x64 &&
        targetPlatform != TargetPlatform.linux_arm64) {
      throwToolExit('"cross-building" only supported on Linux x64 hosts.');
    }
    // TODO(fujino): https://github.com/flutter/flutter/issues/74929
    if (_operatingSystemUtils.hostPlatform == HostPlatform.linux_x64 &&
        targetPlatform == TargetPlatform.linux_arm64) {
      throwToolExit(
        'Cross-build from Linux x64 host to Linux arm64 target is not currently supported.',
      );
    }
    displayNullSafetyMode(buildInfo);
    final Logger logger = globals.logger;
    await buildLinux(
      project.linux,
      buildInfo,
      target: targetFile,
      sizeAnalyzer: SizeAnalyzer(fileSystem: globals.fs, logger: logger, analytics: analytics),
      needCrossBuild: needCrossBuild,
      targetPlatform: targetPlatform,
      targetSysroot: stringArg('target-sysroot')!,
      logger: logger,
    );
    return FlutterCommandResult.success();
  }
}
