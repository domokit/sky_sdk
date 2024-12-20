// Copyright 2014 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

import 'package:unified_analytics/unified_analytics.dart';

import '../android/android_builder.dart';
import '../android/build_validation.dart';
import '../android/gradle_utils.dart';
import '../build_info.dart';
import '../cache.dart';
import '../globals.dart' as globals;
import '../project.dart';
import '../reporting/reporting.dart';
import '../runner/flutter_command.dart' show FlutterCommandResult;
import 'build.dart';

class BuildApkCommand extends BuildSubCommand {
  BuildApkCommand({required super.logger, bool verboseHelp = false})
    : super(verboseHelp: verboseHelp) {
    addTreeShakeIconsFlag();
    usesTargetOption();
    addBuildModeFlags(verboseHelp: verboseHelp);
    usesFlavorOption();
    usesPubOption();
    usesBuildNumberOption();
    usesBuildNameOption();
    addShrinkingFlag(verboseHelp: verboseHelp);
    addSplitDebugInfoOption();
    addDartObfuscationOption();
    usesDartDefineOption();
    usesExtraDartFlagOptions(verboseHelp: verboseHelp);
    addBundleSkSLPathOption(hide: !verboseHelp);
    addEnableExperimentation(hide: !verboseHelp);
    addBuildPerformanceFile(hide: !verboseHelp);
    addNullSafetyModeOptions(hide: !verboseHelp);
    usesAnalyzeSizeFlag();
    addAndroidSpecificBuildOptions(hide: !verboseHelp);
    addIgnoreDeprecationOption();
    argParser
      ..addFlag(
        'split-per-abi',
        negatable: false,
        help:
            'Whether to split the APKs per ABIs. '
            'To learn more, see: https://developer.android.com/studio/build/configure-apk-splits#configure-abi-split',
      )
      ..addFlag(
        'config-only',
        help:
            'Generate build files used by flutter but '
            'do not build any artifacts.',
      )
      ..addMultiOption(
        'target-platform',
        allowed: <String>['android-arm', 'android-arm64', 'android-x86', 'android-x64'],
        help: 'The target platform for which the app is compiled.',
      );
    usesTrackWidgetCreation(verboseHelp: verboseHelp);
  }

  BuildMode get _buildMode {
    if (boolArg('release')) {
      return BuildMode.release;
    } else if (boolArg('profile')) {
      return BuildMode.profile;
    } else if (boolArg('debug')) {
      return BuildMode.debug;
    } else if (boolArg('jit-release')) {
      return BuildMode.jitRelease;
    }
    return BuildMode.release;
  }

  static const List<String> _kDefaultJitArchs = <String>[
    'android-arm',
    'android-arm64',
    'android-x86',
    'android-x64',
  ];
  static const List<String> _kDefaultAotArchs = <String>[
    'android-arm',
    'android-arm64',
    'android-x64',
  ];
  List<String> get _targetArchs =>
      stringsArg('target-platform').isEmpty
          ? switch (_buildMode) {
            BuildMode.release || BuildMode.profile => _kDefaultAotArchs,
            BuildMode.debug || BuildMode.jitRelease => _kDefaultJitArchs,
          }
          : stringsArg('target-platform');

  @override
  final String name = 'apk';

  @override
  DeprecationBehavior get deprecationBehavior =>
      boolArg('ignore-deprecation') ? DeprecationBehavior.ignore : DeprecationBehavior.exit;

  bool get configOnly => boolArg('config-only');

  @override
  Future<Set<DevelopmentArtifact>> get requiredArtifacts async => <DevelopmentArtifact>{
    DevelopmentArtifact.androidGenSnapshot,
  };

  @override
  final String description =
      'Build an Android APK file from your app.\n\n'
      "This command can build debug and release versions of your application. 'debug' builds support "
      "debugging and a quick development cycle. 'release' builds don't support debugging and are "
      'suitable for deploying to app stores. If you are deploying the app to the Play Store, '
      "it's recommended to use app bundles or split the APK to reduce the APK size. Learn more at:\n\n"
      ' * https://developer.android.com/guide/app-bundle\n'
      ' * https://developer.android.com/studio/build/configure-apk-splits#configure-abi-split';

  @override
  Future<CustomDimensions> get usageValues async {
    return CustomDimensions(
      commandBuildApkTargetPlatform: _targetArchs.join(','),
      commandBuildApkBuildMode: _buildMode.cliName,
      commandBuildApkSplitPerAbi: boolArg('split-per-abi'),
    );
  }

  @override
  Future<Event> unifiedAnalyticsUsageValues(String commandPath) async {
    return Event.commandUsageValues(
      workflow: commandPath,
      commandHasTerminal: hasTerminal,
      buildApkTargetPlatform: _targetArchs.join(','),
      buildApkBuildMode: _buildMode.cliName,
      buildApkSplitPerAbi: boolArg('split-per-abi'),
    );
  }

  @override
  Future<FlutterCommandResult> runCommand() async {
    if (globals.androidSdk == null) {
      exitWithNoSdkMessage();
    }
    final BuildInfo buildInfo = await getBuildInfo();

    final AndroidBuildInfo androidBuildInfo = AndroidBuildInfo(
      buildInfo,
      splitPerAbi: boolArg('split-per-abi'),
      targetArchs: _targetArchs.map<AndroidArch>(getAndroidArchForName),
    );
    validateBuild(androidBuildInfo);
    displayNullSafetyMode(androidBuildInfo.buildInfo);
    globals.terminal.usesTerminalUi = true;
    final FlutterProject project = FlutterProject.current();
    await androidBuilder?.buildApk(
      project: project,
      target: targetFile,
      androidBuildInfo: androidBuildInfo,
      configOnly: configOnly,
    );

    // When an app is successfully built, record to analytics whether Impeller
    // is enabled or disabled. Note that 'computeImpellerEnabled' will default
    // to false if not enabled explicitly in the manifest.
    final bool impellerEnabled = project.android.computeImpellerEnabled();
    final String buildLabel =
        impellerEnabled ? 'manifest-impeller-enabled' : 'manifest-impeller-disabled';
    globals.analytics.send(Event.flutterBuildInfo(label: buildLabel, buildType: 'android'));

    return FlutterCommandResult.success();
  }
}
