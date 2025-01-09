// Copyright 2014 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

import 'package:unified_analytics/unified_analytics.dart';

import '../base/common.dart';
import '../build_info.dart';
import '../bundle.dart';
import '../bundle_builder.dart';
import '../features.dart';
import '../globals.dart' as globals;
import '../project.dart';
import '../reporting/reporting.dart';
import '../runner/flutter_command.dart';
import 'build.dart';

class BuildBundleCommand extends BuildSubCommand {
  BuildBundleCommand({
    required super.logger,
    bool verboseHelp = false,
    BundleBuilder? bundleBuilder,
  }) : _bundleBuilder = bundleBuilder ?? BundleBuilder(),
       super(verboseHelp: verboseHelp) {
    usesTargetOption();
    usesFilesystemOptions(hide: !verboseHelp);
    usesBuildNumberOption();
    addBuildModeFlags(verboseHelp: verboseHelp, defaultToRelease: false);
    usesDartDefineOption();
    usesExtraDartFlagOptions(verboseHelp: verboseHelp);
    argParser
      ..addOption(
        'depfile',
        defaultsTo: defaultDepfilePath,
        help:
            'A file path where a depfile will be written. '
            'This contains all build inputs and outputs in a Make-style syntax.',
      )
      ..addOption(
        'target-platform',
        defaultsTo: 'android-arm',
        allowed: const <String>[
          'android-arm',
          'android-arm64',
          'android-x86',
          'android-x64',
          'ios',
          'darwin',
          'linux-x64',
          'linux-arm64',
          'windows-x64',
          'windows-arm64',
        ],
        help: 'The architecture for which to build the application.',
      )
      ..addOption(
        'asset-dir',
        defaultsTo: getAssetBuildDirectory(),
        help:
            'The output directory for the kernel_blob.bin file, the native snapshot, the assets, etc. '
            'Can be used to redirect the output when driving the Flutter toolchain from another build system.',
      )
      ..addFlag(
        'tree-shake-icons',
        hide: !verboseHelp,
        help: '(deprecated) Icon font tree shaking is not supported by this command.',
      );
    usesPubOption();
    usesTrackWidgetCreation(verboseHelp: verboseHelp);
  }

  final BundleBuilder _bundleBuilder;

  @override
  final String name = 'bundle';

  @override
  final String description = 'Build the Flutter assets directory from your app.';

  @override
  final String usageFooter =
      'The Flutter assets directory contains your '
      'application code and resources; they are used by some Flutter Android and'
      ' iOS runtimes.';

  @override
  Future<CustomDimensions> get usageValues async {
    final String projectDir = globals.fs.file(targetFile).parent.parent.path;
    final FlutterProject flutterProject = FlutterProject.fromDirectory(
      globals.fs.directory(projectDir),
    );
    return CustomDimensions(
      commandBuildBundleTargetPlatform: stringArg('target-platform'),
      commandBuildBundleIsModule: flutterProject.isModule,
    );
  }

  @override
  Future<Event> unifiedAnalyticsUsageValues(String commandPath) async {
    final String projectDir = globals.fs.file(targetFile).parent.parent.path;
    final FlutterProject flutterProject = FlutterProject.fromDirectory(
      globals.fs.directory(projectDir),
    );
    return Event.commandUsageValues(
      workflow: commandPath,
      commandHasTerminal: hasTerminal,
      buildBundleTargetPlatform: stringArg('target-platform'),
      buildBundleIsModule: flutterProject.isModule,
    );
  }

  @override
  Future<void> validateCommand() async {
    if (boolArg('tree-shake-icons')) {
      throwToolExit(
        'The "--tree-shake-icons" flag is deprecated for "build bundle" and will be removed in a future version of Flutter.',
      );
    }
    return super.validateCommand();
  }

  @override
  Future<FlutterCommandResult> runCommand() async {
    final String targetPlatform = stringArg('target-platform')!;
    final TargetPlatform platform = getTargetPlatformForName(targetPlatform);
    // Check for target platforms that are only allowed via feature flags.
    switch (platform) {
      case TargetPlatform.darwin:
        if (!featureFlags.isMacOSEnabled) {
          throwToolExit('macOS is not a supported target platform.');
        }
      case TargetPlatform.windows_x64:
      case TargetPlatform.windows_arm64:
        if (!featureFlags.isWindowsEnabled) {
          throwToolExit('Windows is not a supported target platform.');
        }
      case TargetPlatform.linux_x64:
      case TargetPlatform.linux_arm64:
        if (!featureFlags.isLinuxEnabled) {
          throwToolExit('Linux is not a supported target platform.');
        }
      case TargetPlatform.android:
      case TargetPlatform.android_arm:
      case TargetPlatform.android_arm64:
      case TargetPlatform.android_x64:
      case TargetPlatform.android_x86:
      case TargetPlatform.fuchsia_arm64:
      case TargetPlatform.fuchsia_x64:
      case TargetPlatform.ios:
      case TargetPlatform.tester:
      case TargetPlatform.web_javascript:
        break;
    }

    final BuildInfo buildInfo = await getBuildInfo();
    displayNullSafetyMode(buildInfo);

    await _bundleBuilder.build(
      platform: platform,
      buildInfo: buildInfo,
      mainPath: targetFile,
      depfilePath: stringArg('depfile'),
      assetDirPath: stringArg('asset-dir'),
      buildNativeAssets: false,
    );
    return FlutterCommandResult.success();
  }
}
