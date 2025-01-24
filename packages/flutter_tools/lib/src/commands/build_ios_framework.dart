// Copyright 2014 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

import 'package:meta/meta.dart';
import 'package:process/process.dart';

import '../artifacts.dart';
import '../base/common.dart';
import '../base/file_system.dart';
import '../base/io.dart';
import '../base/logger.dart';
import '../base/platform.dart';
import '../base/process.dart';
import '../base/utils.dart';
import '../build_info.dart';
import '../build_system/build_system.dart';
import '../build_system/targets/ios.dart';
import '../cache.dart';
import '../flutter_plugins.dart';
import '../globals.dart' as globals;
import '../macos/cocoapod_utils.dart';
import '../runner/flutter_command.dart' show DevelopmentArtifact, FlutterCommandResult;
import '../version.dart';
import 'build.dart';

abstract class BuildFrameworkCommand extends BuildSubCommand {
  BuildFrameworkCommand({
    // Instantiating FlutterVersion kicks off networking, so delay until it's needed, but allow test injection.
    @visibleForTesting FlutterVersion? flutterVersion,
    required BuildSystem buildSystem,
    required bool verboseHelp,
    Cache? cache,
    Platform? platform,
    required super.logger,
  }) : _injectedFlutterVersion = flutterVersion,
       _buildSystem = buildSystem,
       _injectedCache = cache,
       _injectedPlatform = platform,
       super(verboseHelp: verboseHelp) {
    addTreeShakeIconsFlag();
    usesTargetOption();
    usesPubOption();
    usesDartDefineOption();
    addSplitDebugInfoOption();
    addDartObfuscationOption();
    usesExtraDartFlagOptions(verboseHelp: verboseHelp);
    addNullSafetyModeOptions(hide: !verboseHelp);
    addEnableExperimentation(hide: !verboseHelp);

    argParser
      ..addFlag(
        'debug',
        defaultsTo: true,
        help:
            'Whether to produce a framework for the debug build configuration. '
            'By default, all build configurations are built.',
      )
      ..addFlag(
        'profile',
        defaultsTo: true,
        help:
            'Whether to produce a framework for the profile build configuration. '
            'By default, all build configurations are built.',
      )
      ..addFlag(
        'release',
        defaultsTo: true,
        help:
            'Whether to produce a framework for the release build configuration. '
            'By default, all build configurations are built.',
      )
      ..addFlag(
        'cocoapods',
        help:
            'Produce a Flutter.podspec instead of an engine Flutter.xcframework (recommended if host app uses CocoaPods).',
      )
      ..addFlag(
        'plugins',
        defaultsTo: true,
        help:
            'Whether to produce frameworks for the plugins. '
            'This is intended for cases where plugins are already being built separately.',
      )
      ..addFlag(
        'static',
        help:
            'Build plugins as static frameworks. Link on, but do not embed these frameworks in the existing Xcode project.',
      )
      ..addOption(
        'output',
        abbr: 'o',
        valueHelp: 'path/to/directory/',
        help: 'Location to write the frameworks.',
      )
      ..addFlag(
        'force',
        abbr: 'f',
        help:
            'Force Flutter.podspec creation on the master channel. This is only intended for testing the tool itself.',
        hide: !verboseHelp,
      );
  }

  final BuildSystem? _buildSystem;
  @protected
  BuildSystem get buildSystem => _buildSystem ?? globals.buildSystem;

  @protected
  Cache get cache => _injectedCache ?? globals.cache;
  final Cache? _injectedCache;

  @protected
  Platform get platform => _injectedPlatform ?? globals.platform;
  final Platform? _injectedPlatform;

  // FlutterVersion.instance kicks off git processing which can sometimes fail, so don't try it until needed.
  @protected
  FlutterVersion get flutterVersion => _injectedFlutterVersion ?? globals.flutterVersion;
  final FlutterVersion? _injectedFlutterVersion;

  @override
  bool get reportNullSafety => false;

  Future<List<BuildInfo>> getBuildInfos() async {
    return <BuildInfo>[
      if (boolArg('debug')) await getBuildInfo(forcedBuildMode: BuildMode.debug),
      if (boolArg('profile')) await getBuildInfo(forcedBuildMode: BuildMode.profile),
      if (boolArg('release')) await getBuildInfo(forcedBuildMode: BuildMode.release),
    ];
  }

  @override
  bool get supported => platform.isMacOS;

  @override
  Future<void> validateCommand() async {
    await super.validateCommand();
    if (!supported) {
      throwToolExit('Building frameworks for iOS is only supported on the Mac.');
    }

    if ((await getBuildInfos()).isEmpty) {
      throwToolExit('At least one of "--debug" or "--profile", or "--release" is required.');
    }

    if (!boolArg('plugins') && boolArg('static')) {
      throwToolExit('--static cannot be used with the --no-plugins flag');
    }
  }

  static Future<void> produceXCFramework(
    Iterable<Directory> frameworks,
    String frameworkBinaryName,
    Directory outputDirectory,
    ProcessManager processManager,
  ) async {
    final List<String> xcframeworkCommand = <String>[
      'xcrun',
      'xcodebuild',
      '-create-xcframework',
      for (final Directory framework in frameworks) ...<String>[
        '-framework',
        framework.path,
        ...framework.parent
            .listSync()
            .where(
              (FileSystemEntity entity) =>
                  entity.basename.endsWith('dSYM') && !entity.basename.startsWith('Flutter'),
            )
            .map((FileSystemEntity entity) => <String>['-debug-symbols', entity.path])
            .expand<String>((List<String> parameter) => parameter),
      ],
      '-output',
      outputDirectory.childDirectory('$frameworkBinaryName.xcframework').path,
    ];

    final ProcessResult xcframeworkResult = await processManager.run(xcframeworkCommand);

    if (xcframeworkResult.exitCode != 0) {
      throwToolExit(
        'Unable to create $frameworkBinaryName.xcframework: ${xcframeworkResult.stderr}',
      );
    }
  }
}

/// Produces a .framework for integration into a host iOS app. The .framework
/// contains the Flutter engine and framework code as well as plugins. It can
/// be integrated into plain Xcode projects without using or other package
/// managers.
class BuildIOSFrameworkCommand extends BuildFrameworkCommand {
  BuildIOSFrameworkCommand({
    required super.logger,
    super.flutterVersion,
    required super.buildSystem,
    required bool verboseHelp,
    super.cache,
    super.platform,
  }) : super(verboseHelp: verboseHelp) {
    usesFlavorOption();

    argParser
      ..addFlag(
        'universal',
        help: '(deprecated) Produce universal frameworks that include all valid architectures.',
        hide: !verboseHelp,
      )
      ..addFlag(
        'xcframework',
        help: 'Produce xcframeworks that include all valid architectures.',
        negatable: false,
        defaultsTo: true,
        hide: !verboseHelp,
      );
  }

  @override
  final String name = 'ios-framework';

  @override
  final String description =
      'Produces .xcframeworks for a Flutter project '
      'and its plugins for integration into existing, plain iOS Xcode projects.\n'
      'This can only be run on macOS hosts.';

  @override
  Future<Set<DevelopmentArtifact>> get requiredArtifacts async => const <DevelopmentArtifact>{
    DevelopmentArtifact.iOS,
  };

  @override
  Future<void> validateCommand() async {
    await super.validateCommand();

    if (boolArg('universal')) {
      throwToolExit('--universal has been deprecated, only XCFrameworks are supported.');
    }
  }

  @override
  Future<FlutterCommandResult> runCommand() async {
    final String outputArgument =
        stringArg('output') ??
        globals.fs.path.join(globals.fs.currentDirectory.path, 'build', 'ios', 'framework');

    if (outputArgument.isEmpty) {
      throwToolExit('--output is required.');
    }

    if (!project.ios.existsSync()) {
      throwToolExit('Project does not support iOS');
    }

    final Directory outputDirectory = globals.fs.directory(
      globals.fs.path.absolute(globals.fs.path.normalize(outputArgument)),
    );
    final List<BuildInfo> buildInfos = await getBuildInfos();
    displayNullSafetyMode(buildInfos.first);
    for (final BuildInfo buildInfo in buildInfos) {
      final String? productBundleIdentifier = await project.ios.productBundleIdentifier(buildInfo);
      globals.printStatus(
        'Building frameworks for $productBundleIdentifier in ${buildInfo.mode.cliName} mode...',
      );
      final String xcodeBuildConfiguration = sentenceCase(buildInfo.mode.cliName);
      final Directory modeDirectory = outputDirectory.childDirectory(xcodeBuildConfiguration);

      if (modeDirectory.existsSync()) {
        modeDirectory.deleteSync(recursive: true);
      }

      if (boolArg('cocoapods')) {
        produceFlutterPodspec(buildInfo.mode, modeDirectory, force: boolArg('force'));
      } else {
        // Copy Flutter.xcframework.
        await _produceFlutterFramework(buildInfo, modeDirectory);
      }

      // Build aot, create module.framework and copy.
      final Directory iPhoneBuildOutput = modeDirectory.childDirectory('iphoneos');
      final Directory simulatorBuildOutput = modeDirectory.childDirectory('iphonesimulator');
      await _produceAppFramework(buildInfo, modeDirectory, iPhoneBuildOutput, simulatorBuildOutput);

      // Build and copy plugins.
      await processPodsIfNeeded(
        project.ios,
        getIosBuildDirectory(),
        buildInfo.mode,
        forceCocoaPodsOnly: true,
      );
      if (boolArg('plugins') && hasPlugins(project)) {
        await _producePlugins(
          buildInfo.mode,
          xcodeBuildConfiguration,
          iPhoneBuildOutput,
          simulatorBuildOutput,
          modeDirectory,
        );
      }

      final Status status = globals.logger.startProgress(
        ' └─Moving to ${globals.fs.path.relative(modeDirectory.path)}',
      );

      // Copy the native assets. The native assets have already been signed in
      // buildNativeAssetsMacOS.
      final Directory nativeAssetsDirectory = globals.fs
          .directory(getBuildDirectory())
          .childDirectory('native_assets/ios/');
      if (await nativeAssetsDirectory.exists()) {
        final ProcessResult rsyncResult = await globals.processManager.run(<Object>[
          'rsync',
          '-av',
          '--filter',
          '- .DS_Store',
          '--filter',
          '- native_assets.yaml',
          '--filter',
          '- native_assets.json',
          nativeAssetsDirectory.path,
          modeDirectory.path,
        ]);
        if (rsyncResult.exitCode != 0) {
          throwToolExit('Failed to copy native assets:\n${rsyncResult.stderr}');
        }
      }

      try {
        // Delete the intermediaries since they would have been copied into our
        // output frameworks.
        if (iPhoneBuildOutput.existsSync()) {
          iPhoneBuildOutput.deleteSync(recursive: true);
        }
        if (simulatorBuildOutput.existsSync()) {
          simulatorBuildOutput.deleteSync(recursive: true);
        }
      } finally {
        status.stop();
      }
    }

    globals.printStatus('Frameworks written to ${outputDirectory.path}.');

    if (!project.isModule && hasPlugins(project)) {
      // Apps do not generate a FlutterPluginRegistrant.framework. Users will need
      // to copy the GeneratedPluginRegistrant class to their project manually.
      final File pluginRegistrantHeader = project.ios.pluginRegistrantHeader;
      final File pluginRegistrantImplementation = project.ios.pluginRegistrantImplementation;
      pluginRegistrantHeader.copySync(
        outputDirectory.childFile(pluginRegistrantHeader.basename).path,
      );
      pluginRegistrantImplementation.copySync(
        outputDirectory.childFile(pluginRegistrantImplementation.basename).path,
      );
      globals.printStatus(
        '\nCopy the ${globals.fs.path.basenameWithoutExtension(pluginRegistrantHeader.path)} class into your project.\n'
        'See https://flutter.dev/to/ios-create-flutter-engine for more information.',
      );
    }

    return FlutterCommandResult.success();
  }

  /// Create podspec that will download and unzip remote engine assets so host apps can leverage CocoaPods
  /// vendored framework caching.
  @visibleForTesting
  void produceFlutterPodspec(BuildMode mode, Directory modeDirectory, {bool force = false}) {
    final Status status = globals.logger.startProgress(' ├─Creating Flutter.podspec...');
    try {
      final GitTagVersion gitTagVersion = flutterVersion.gitTagVersion;
      if (!force &&
          (gitTagVersion.x == null ||
              gitTagVersion.y == null ||
              gitTagVersion.z == null ||
              gitTagVersion.commits != 0)) {
        throwToolExit(
          '--cocoapods is only supported on the beta or stable channel. Detected version is ${flutterVersion.frameworkVersion}',
        );
      }

      // Podspecs use semantic versioning, which don't support hotfixes.
      // Fake out a semantic version with major.minor.(patch * 100) + hotfix.
      // A real increasing version is required to prompt CocoaPods to fetch
      // new artifacts when the source URL changes.
      final int minorHotfixVersion = (gitTagVersion.z ?? 0) * 100 + (gitTagVersion.hotfix ?? 0);

      final File license = cache.getLicenseFile();
      if (!license.existsSync()) {
        throwToolExit('Could not find license at ${license.path}');
      }
      final String licenseSource = license.readAsStringSync();
      final String artifactsMode = mode == BuildMode.debug ? 'ios' : 'ios-${mode.cliName}';

      final String podspecContents = '''
Pod::Spec.new do |s|
  s.name                  = 'Flutter'
  s.version               = '${gitTagVersion.x}.${gitTagVersion.y}.$minorHotfixVersion' # ${flutterVersion.frameworkVersion}
  s.summary               = 'A UI toolkit for beautiful and fast apps.'
  s.description           = <<-DESC
Flutter is Google's UI toolkit for building beautiful, fast apps for mobile, web, desktop, and embedded devices from a single codebase.
This pod vends the iOS Flutter engine framework. It is compatible with application frameworks created with this version of the engine and tools.
The pod version matches Flutter version major.minor.(patch * 100) + hotfix.
DESC
  s.homepage              = 'https://flutter.dev'
  s.license               = { :type => 'BSD', :text => <<-LICENSE
$licenseSource
LICENSE
  }
  s.author                = { 'Flutter Dev Team' => 'flutter-dev@googlegroups.com' }
  s.source                = { :http => '${cache.storageBaseUrl}/flutter_infra_release/flutter/${cache.engineRevision}/$artifactsMode/artifacts.zip' }
  s.documentation_url     = 'https://docs.flutter.dev'
  s.platform              = :ios, '12.0'
  s.vendored_frameworks   = 'Flutter.xcframework'
end
''';

      final File podspec = modeDirectory.childFile('Flutter.podspec')..createSync(recursive: true);
      podspec.writeAsStringSync(podspecContents);
    } finally {
      status.stop();
    }
  }

  Future<void> _produceFlutterFramework(BuildInfo buildInfo, Directory modeDirectory) async {
    final Status status = globals.logger.startProgress(' ├─Copying Flutter.xcframework...');
    final String engineCacheFlutterFrameworkDirectory = globals.artifacts!.getArtifactPath(
      Artifact.flutterXcframework,
      platform: TargetPlatform.ios,
      mode: buildInfo.mode,
    );
    final String flutterFrameworkFileName = globals.fs.path.basename(
      engineCacheFlutterFrameworkDirectory,
    );
    final Directory flutterFrameworkCopy = modeDirectory.childDirectory(flutterFrameworkFileName);

    try {
      // Copy xcframework engine cache framework to mode directory.
      copyDirectory(
        globals.fs.directory(engineCacheFlutterFrameworkDirectory),
        flutterFrameworkCopy,
      );
    } finally {
      status.stop();
    }
  }

  Future<void> _produceAppFramework(
    BuildInfo buildInfo,
    Directory outputDirectory,
    Directory iPhoneBuildOutput,
    Directory simulatorBuildOutput,
  ) async {
    const String appFrameworkName = 'App.framework';
    final Status status = globals.logger.startProgress(' ├─Building App.xcframework...');
    final List<Directory> frameworks = <Directory>[];

    try {
      for (final EnvironmentType sdkType in EnvironmentType.values) {
        final Directory outputBuildDirectory = switch (sdkType) {
          EnvironmentType.physical => iPhoneBuildOutput,
          EnvironmentType.simulator => simulatorBuildOutput,
        };
        frameworks.add(outputBuildDirectory.childDirectory(appFrameworkName));
        final Environment environment = Environment(
          projectDir: globals.fs.currentDirectory,
          packageConfigPath: packageConfigPath(),
          outputDir: outputBuildDirectory,
          buildDir: project.dartTool.childDirectory('flutter_build'),
          cacheDir: globals.cache.getRoot(),
          flutterRootDir: globals.fs.directory(Cache.flutterRoot),
          defines: <String, String>{
            kTargetFile: targetFile,
            kTargetPlatform: getNameForTargetPlatform(TargetPlatform.ios),
            kIosArchs: defaultIOSArchsForEnvironment(
              sdkType,
              globals.artifacts!,
            ).map((DarwinArch e) => e.name).join(' '),
            kSdkRoot: await globals.xcode!.sdkLocation(sdkType),
            ...buildInfo.toBuildSystemEnvironment(),
          },
          artifacts: globals.artifacts!,
          fileSystem: globals.fs,
          logger: globals.logger,
          processManager: globals.processManager,
          platform: globals.platform,
          analytics: globals.analytics,
          engineVersion:
              globals.artifacts!.usesLocalArtifacts ? null : globals.flutterVersion.engineRevision,
          generateDartPluginRegistry: true,
        );
        Target target;
        // Always build debug for simulator.
        if (buildInfo.isDebug || sdkType == EnvironmentType.simulator) {
          target = const DebugIosApplicationBundle();
        } else if (buildInfo.isProfile) {
          target = const ProfileIosApplicationBundle();
        } else {
          target = const ReleaseIosApplicationBundle();
        }
        final BuildResult result = await buildSystem.build(target, environment);
        if (!result.success) {
          for (final ExceptionMeasurement measurement in result.exceptions.values) {
            globals.printError(measurement.exception.toString());
          }
          throwToolExit('The App.xcframework build failed.');
        }
      }
    } finally {
      status.stop();
    }

    await BuildFrameworkCommand.produceXCFramework(
      frameworks,
      'App',
      outputDirectory,
      globals.processManager,
    );
  }

  Future<void> _producePlugins(
    BuildMode mode,
    String xcodeBuildConfiguration,
    Directory iPhoneBuildOutput,
    Directory simulatorBuildOutput,
    Directory modeDirectory,
  ) async {
    final Status status = globals.logger.startProgress(' ├─Building plugins...');
    try {
      List<String> pluginsBuildCommand = <String>[
        ...globals.xcode!.xcrunCommand(),
        'xcodebuild',
        '-alltargets',
        '-sdk',
        'iphoneos',
        '-configuration',
        xcodeBuildConfiguration,
        'SYMROOT=${iPhoneBuildOutput.path}',
        'ONLY_ACTIVE_ARCH=NO', // No device targeted, so build all valid architectures.
        'BUILD_LIBRARY_FOR_DISTRIBUTION=YES',
        if (boolArg('static')) 'MACH_O_TYPE=staticlib',
      ];

      RunResult buildPluginsResult = await globals.processUtils.run(
        pluginsBuildCommand,
        workingDirectory: project.ios.hostAppRoot.childDirectory('Pods').path,
      );

      if (buildPluginsResult.exitCode != 0) {
        throwToolExit('Unable to build plugin frameworks: ${buildPluginsResult.stderr}');
      }

      // Always build debug for simulator.
      final String simulatorConfiguration = sentenceCase(BuildMode.debug.cliName);
      pluginsBuildCommand = <String>[
        ...globals.xcode!.xcrunCommand(),
        'xcodebuild',
        '-alltargets',
        '-sdk',
        'iphonesimulator',
        '-configuration',
        simulatorConfiguration,
        'SYMROOT=${simulatorBuildOutput.path}',
        'ONLY_ACTIVE_ARCH=NO', // No device targeted, so build all valid architectures.
        'BUILD_LIBRARY_FOR_DISTRIBUTION=YES',
        if (boolArg('static')) 'MACH_O_TYPE=staticlib',
      ];

      buildPluginsResult = await globals.processUtils.run(
        pluginsBuildCommand,
        workingDirectory: project.ios.hostAppRoot.childDirectory('Pods').path,
      );

      if (buildPluginsResult.exitCode != 0) {
        throwToolExit(
          'Unable to build plugin frameworks for simulator: ${buildPluginsResult.stderr}',
        );
      }

      final Directory iPhoneBuildConfiguration = iPhoneBuildOutput.childDirectory(
        '$xcodeBuildConfiguration-iphoneos',
      );
      final Directory simulatorBuildConfiguration = simulatorBuildOutput.childDirectory(
        '$simulatorConfiguration-iphonesimulator',
      );

      final Iterable<Directory> products =
          iPhoneBuildConfiguration.listSync(followLinks: false).whereType<Directory>();
      for (final Directory builtProduct in products) {
        for (final FileSystemEntity podProduct in builtProduct.listSync(followLinks: false)) {
          final String podFrameworkName = podProduct.basename;
          if (globals.fs.path.extension(podFrameworkName) != '.framework') {
            continue;
          }
          final String binaryName = globals.fs.path.basenameWithoutExtension(podFrameworkName);

          final List<Directory> frameworks = <Directory>[
            podProduct as Directory,
            simulatorBuildConfiguration
                .childDirectory(builtProduct.basename)
                .childDirectory(podFrameworkName),
          ];

          await BuildFrameworkCommand.produceXCFramework(
            frameworks,
            binaryName,
            modeDirectory,
            globals.processManager,
          );
        }
      }
    } finally {
      status.stop();
    }
  }
}
