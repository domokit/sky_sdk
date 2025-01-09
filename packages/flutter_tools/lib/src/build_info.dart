// Copyright 2014 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

import 'package:meta/meta.dart';

import 'package:package_config/package_config_types.dart';

import 'artifacts.dart';
import 'base/config.dart';
import 'base/file_system.dart';
import 'base/logger.dart';
import 'base/os.dart';
import 'base/utils.dart';
import 'convert.dart';
import 'globals.dart' as globals;

/// Whether icon font subsetting is enabled by default.
const bool kIconTreeShakerEnabledDefault = true;

/// Information about a build to be performed or used.
class BuildInfo {
  const BuildInfo(
    this.mode,
    this.flavor, {
    this.trackWidgetCreation = false,
    this.frontendServerStarterPath,
    List<String>? extraFrontEndOptions,
    List<String>? extraGenSnapshotOptions,
    List<String>? fileSystemRoots,
    this.androidProjectArgs = const <String>[],
    this.fileSystemScheme,
    this.buildNumber,
    this.buildName,
    this.splitDebugInfoPath,
    this.dartObfuscation = false,
    List<String>? dartDefines,
    this.bundleSkSLPath,
    List<String>? dartExperiments,
    required this.treeShakeIcons,
    this.performanceMeasurementFile,
    required this.packageConfigPath,
    this.nullSafetyMode = NullSafetyMode.sound,
    this.codeSizeDirectory,
    this.androidGradleDaemon = true,
    this.androidSkipBuildDependencyValidation = false,
    this.packageConfig = PackageConfig.empty,
    this.initializeFromDill,
    this.assumeInitializeFromDillUpToDate = false,
    this.buildNativeAssets = true,
    this.useLocalCanvasKit = false,
  }) : extraFrontEndOptions = extraFrontEndOptions ?? const <String>[],
       extraGenSnapshotOptions = extraGenSnapshotOptions ?? const <String>[],
       fileSystemRoots = fileSystemRoots ?? const <String>[],
       dartDefines = dartDefines ?? const <String>[],
       dartExperiments = dartExperiments ?? const <String>[];

  final BuildMode mode;

  /// The null safety mode the application should be run in.
  ///
  /// If not provided, defaults to [NullSafetyMode.autodetect].
  final NullSafetyMode nullSafetyMode;

  /// Whether the build should subset icon fonts.
  final bool treeShakeIcons;

  /// Represents a custom Android product flavor or an Xcode scheme, null for
  /// using the default.
  ///
  /// If not null, the Gradle build task will be `assembleFlavorMode` (e.g.
  /// `assemblePaidRelease`), and the Xcode build configuration will be
  /// Mode-Flavor (e.g. Release-Paid).
  final String? flavor;

  /// The path to the package configuration file to use for compilation.
  ///
  /// This is used by package:package_config to locate the actual package_config.json
  /// file. If not provided, defaults to `.dart_tool/package_config.json`.
  final String packageConfigPath;

  final List<String> fileSystemRoots;
  final String? fileSystemScheme;

  /// Whether the build should track widget creation locations.
  final bool trackWidgetCreation;

  /// If provided, the frontend server will be started in JIT mode from this
  /// file.
  final String? frontendServerStarterPath;

  /// Extra command-line options for front-end.
  final List<String> extraFrontEndOptions;

  /// Extra command-line options for gen_snapshot.
  final List<String> extraGenSnapshotOptions;

  /// Internal version number (not displayed to users).
  /// Each build must have a unique number to differentiate it from previous builds.
  /// It is used to determine whether one build is more recent than another, with higher numbers indicating more recent build.
  /// On Android it is used as versionCode.
  /// On Xcode builds it is used as CFBundleVersion.
  /// On Windows it is used as the build suffix for the product and file versions.
  final String? buildNumber;

  /// A "x.y.z" string used as the version number shown to users.
  /// For each new version of your app, you will provide a version number to differentiate it from previous versions.
  /// On Android it is used as versionName.
  /// On Xcode builds it is used as CFBundleShortVersionString.
  /// On Windows it is used as the major, minor, and patch parts of the product and file versions.
  final String? buildName;

  /// An optional directory path to save debugging information from dwarf stack
  /// traces. If null, stack trace information is not stripped from the
  /// executable.
  final String? splitDebugInfoPath;

  /// Whether to apply dart source code obfuscation.
  final bool dartObfuscation;

  /// An optional path to a JSON containing object SkSL shaders.
  ///
  /// Currently this is only supported for Android builds.
  final String? bundleSkSLPath;

  /// Additional constant values to be made available in the Dart program.
  ///
  /// These values can be used with the const `fromEnvironment` constructors of
  /// [bool], [String], [int], and [double].
  final List<String> dartDefines;

  /// A list of Dart experiments.
  final List<String> dartExperiments;

  /// The name of a file where flutter assemble will output performance
  /// information in a JSON format.
  ///
  /// This is not considered a build input and will not force assemble to
  /// rerun tasks.
  final String? performanceMeasurementFile;

  /// If provided, an output directory where one or more v8-style heap snapshots
  /// will be written for code size profiling.
  final String? codeSizeDirectory;

  /// Whether to enable the Gradle daemon when performing an Android build.
  ///
  /// Starting the daemon is the default behavior of the gradle wrapper script created
  /// in a Flutter project. Setting this value to false will cause the tool to pass
  /// `--no-daemon` to the gradle wrapper script, preventing it from spawning a daemon
  /// process.
  ///
  /// For one-off builds or CI systems, preventing the daemon from spawning will
  /// reduce system resource usage, at the cost of any subsequent builds starting
  /// up slightly slower.
  ///
  /// The Gradle daemon may also be disabled in the Android application's properties file.
  final bool androidGradleDaemon;

  /// Whether to skip checking of individual versions of our Android build time
  /// dependencies.
  final bool androidSkipBuildDependencyValidation;

  /// Additional key value pairs that are passed directly to the gradle project via the `-P`
  /// flag.
  final List<String> androidProjectArgs;

  /// The package configuration for the loaded application.
  ///
  /// This is captured once during startup, but the actual package configuration
  /// may change during a 'flutter run` workflow.
  final PackageConfig packageConfig;

  /// The kernel file that the resident compiler will be initialized with.
  ///
  /// If this is null, it will be initialized from the default cached location.
  final String? initializeFromDill;

  /// If set, assumes that the file passed in [initializeFromDill] is up to date
  /// and skips the check and potential invalidation of files.
  final bool assumeInitializeFromDillUpToDate;

  /// If set, builds native assets with `build.dart` from all packages.
  final bool buildNativeAssets;

  /// If set, web builds will use the locally built CanvasKit instead of using the CDN
  final bool useLocalCanvasKit;

  /// Can be used when the actual information is not needed.
  static const BuildInfo dummy = BuildInfo(
    BuildMode.debug,
    null,
    trackWidgetCreation: true,
    treeShakeIcons: false,
    packageConfigPath: '.dart_tool/package_config.json',
  );

  @visibleForTesting
  static const BuildInfo debug = BuildInfo(
    BuildMode.debug,
    null,
    trackWidgetCreation: true,
    treeShakeIcons: false,
    packageConfigPath: '.dart_tool/package_config.json',
  );

  @visibleForTesting
  static const BuildInfo profile = BuildInfo(
    BuildMode.profile,
    null,
    treeShakeIcons: kIconTreeShakerEnabledDefault,
    packageConfigPath: '.dart_tool/package_config.json',
  );

  @visibleForTesting
  static const BuildInfo jitRelease = BuildInfo(
    BuildMode.jitRelease,
    null,
    treeShakeIcons: kIconTreeShakerEnabledDefault,
    packageConfigPath: '.dart_tool/package_config.json',
  );

  @visibleForTesting
  static const BuildInfo release = BuildInfo(
    BuildMode.release,
    null,
    treeShakeIcons: kIconTreeShakerEnabledDefault,
    packageConfigPath: '.dart_tool/package_config.json',
  );

  /// Returns whether a debug build is requested.
  ///
  /// Exactly one of [isDebug], [isProfile], or [isRelease] is true.
  bool get isDebug => mode == BuildMode.debug;

  /// Returns whether a profile build is requested.
  ///
  /// Exactly one of [isDebug], [isProfile], [isJitRelease],
  /// or [isRelease] is true.
  bool get isProfile => mode == BuildMode.profile;

  /// Returns whether a release build is requested.
  ///
  /// Exactly one of [isDebug], [isProfile], [isJitRelease],
  /// or [isRelease] is true.
  bool get isRelease => mode == BuildMode.release;

  /// Returns whether a JIT release build is requested.
  ///
  /// Exactly one of [isDebug], [isProfile], [isJitRelease],
  /// or [isRelease] is true.
  bool get isJitRelease => mode == BuildMode.jitRelease;

  bool get usesAot => isAotBuildMode(mode);
  bool get supportsEmulator => isEmulatorBuildMode(mode);
  bool get supportsSimulator => isEmulatorBuildMode(mode);
  String get modeName => mode.cliName;
  String get friendlyModeName => getFriendlyModeName(mode);

  /// the flavor name in the output apk files is lower-cased (see Flutter Gradle Plugin),
  /// so the lower cased flavor name is used to compute the output file name
  String? get lowerCasedFlavor => flavor?.toLowerCase();

  /// the flavor name in the output bundle files has the first character lower-cased,
  /// so the uncapitalized flavor name is used to compute the output file name
  String? get uncapitalizedFlavor => _uncapitalize(flavor);

  /// The module system DDC is targeting, or null if not using DDC.
  // TODO(markzipan): delete this when DDC's AMD module system is deprecated, https://github.com/flutter/flutter/issues/142060.
  DdcModuleFormat? get ddcModuleFormat => _ddcModuleFormatFromFrontEndArgs(extraFrontEndOptions);

  /// Convert to a structured string encoded structure appropriate for usage
  /// in build system [Environment.defines].
  ///
  /// Fields that are `null` are excluded from this configuration.
  Map<String, String> toBuildSystemEnvironment() {
    // packagesPath and performanceMeasurementFile are not passed into
    // the Environment map.
    return <String, String>{
      kBuildMode: mode.cliName,
      if (dartDefines.isNotEmpty) kDartDefines: encodeDartDefines(dartDefines),
      kDartObfuscation: dartObfuscation.toString(),
      if (frontendServerStarterPath != null) kFrontendServerStarterPath: frontendServerStarterPath!,
      if (extraFrontEndOptions.isNotEmpty) kExtraFrontEndOptions: extraFrontEndOptions.join(','),
      if (extraGenSnapshotOptions.isNotEmpty)
        kExtraGenSnapshotOptions: extraGenSnapshotOptions.join(','),
      if (splitDebugInfoPath != null) kSplitDebugInfo: splitDebugInfoPath!,
      kTrackWidgetCreation: trackWidgetCreation.toString(),
      kIconTreeShakerFlag: treeShakeIcons.toString(),
      if (bundleSkSLPath != null) kBundleSkSLPath: bundleSkSLPath!,
      if (codeSizeDirectory != null) kCodeSizeDirectory: codeSizeDirectory!,
      if (fileSystemRoots.isNotEmpty) kFileSystemRoots: fileSystemRoots.join(','),
      if (fileSystemScheme != null) kFileSystemScheme: fileSystemScheme!,
      if (buildName != null) kBuildName: buildName!,
      if (buildNumber != null) kBuildNumber: buildNumber!,
      if (useLocalCanvasKit) kUseLocalCanvasKitFlag: useLocalCanvasKit.toString(),
    };
  }

  /// Convert to a structured string encoded structure appropriate for usage as
  /// environment variables or to embed in other scripts.
  ///
  /// Fields that are `null` are excluded from this configuration.
  Map<String, String> toEnvironmentConfig() {
    return <String, String>{
      if (dartDefines.isNotEmpty) 'DART_DEFINES': encodeDartDefines(dartDefines),
      'DART_OBFUSCATION': dartObfuscation.toString(),
      if (frontendServerStarterPath != null)
        'FRONTEND_SERVER_STARTER_PATH': frontendServerStarterPath!,
      if (extraFrontEndOptions.isNotEmpty)
        'EXTRA_FRONT_END_OPTIONS': extraFrontEndOptions.join(','),
      if (extraGenSnapshotOptions.isNotEmpty)
        'EXTRA_GEN_SNAPSHOT_OPTIONS': extraGenSnapshotOptions.join(','),
      if (splitDebugInfoPath != null) 'SPLIT_DEBUG_INFO': splitDebugInfoPath!,
      'TRACK_WIDGET_CREATION': trackWidgetCreation.toString(),
      'TREE_SHAKE_ICONS': treeShakeIcons.toString(),
      if (performanceMeasurementFile != null)
        'PERFORMANCE_MEASUREMENT_FILE': performanceMeasurementFile!,
      if (bundleSkSLPath != null) 'BUNDLE_SKSL_PATH': bundleSkSLPath!,
      'PACKAGE_CONFIG': packageConfigPath,
      if (codeSizeDirectory != null) 'CODE_SIZE_DIRECTORY': codeSizeDirectory!,
      if (flavor != null) 'FLAVOR': flavor!,
    };
  }

  /// Convert this config to a series of project level arguments to be passed
  /// on the command line to gradle.
  List<String> toGradleConfig() {
    // PACKAGE_CONFIG not currently supported.
    return <String>[
      if (dartDefines.isNotEmpty) '-Pdart-defines=${encodeDartDefines(dartDefines)}',
      '-Pdart-obfuscation=$dartObfuscation',
      if (frontendServerStarterPath != null)
        '-Pfrontend-server-starter-path=$frontendServerStarterPath',
      if (extraFrontEndOptions.isNotEmpty)
        '-Pextra-front-end-options=${extraFrontEndOptions.join(',')}',
      if (extraGenSnapshotOptions.isNotEmpty)
        '-Pextra-gen-snapshot-options=${extraGenSnapshotOptions.join(',')}',
      if (splitDebugInfoPath != null) '-Psplit-debug-info=$splitDebugInfoPath',
      '-Ptrack-widget-creation=$trackWidgetCreation',
      '-Ptree-shake-icons=$treeShakeIcons',
      if (performanceMeasurementFile != null)
        '-Pperformance-measurement-file=$performanceMeasurementFile',
      if (bundleSkSLPath != null) '-Pbundle-sksl-path=$bundleSkSLPath',
      if (codeSizeDirectory != null) '-Pcode-size-directory=$codeSizeDirectory',
      for (final String projectArg in androidProjectArgs) '-P$projectArg',
    ];
  }
}

/// Information about an Android build to be performed or used.
class AndroidBuildInfo {
  const AndroidBuildInfo(
    this.buildInfo, {
    this.targetArchs = const <AndroidArch>[
      AndroidArch.armeabi_v7a,
      AndroidArch.arm64_v8a,
      AndroidArch.x86_64,
    ],
    this.splitPerAbi = false,
    this.fastStart = false,
  });

  // The build info containing the mode and flavor.
  final BuildInfo buildInfo;

  /// Whether to split the shared library per ABI.
  ///
  /// When this is false, multiple ABIs will be contained within one primary
  /// build artifact. When this is true, multiple build artifacts (one per ABI)
  /// will be produced.
  final bool splitPerAbi;

  /// The target platforms for the build.
  final Iterable<AndroidArch> targetArchs;

  bool get containsX86Target => targetArchs.contains(AndroidArch.x86);

  /// Whether to bootstrap an empty application.
  final bool fastStart;
}

/// A summary of the compilation strategy used for Dart.
enum BuildMode {
  /// Built in JIT mode with no optimizations, enabled asserts, and a VM service.
  debug,

  /// Built in AOT mode with some optimizations and a VM service.
  profile,

  /// Built in AOT mode with all optimizations and no VM service.
  release,

  /// Built in JIT mode with all optimizations and no VM service.
  jitRelease;

  factory BuildMode.fromCliName(String value) => values.singleWhere(
    (BuildMode element) => element.cliName == value,
    orElse: () => throw ArgumentError('$value is not a supported build mode'),
  );

  static const Set<BuildMode> releaseModes = <BuildMode>{release, jitRelease};
  static const Set<BuildMode> jitModes = <BuildMode>{debug, jitRelease};

  /// Whether this mode is considered release.
  ///
  /// Useful for determining whether we should enable/disable asserts or
  /// other development features.
  bool get isRelease => releaseModes.contains(this);

  /// Whether this mode is using the JIT runtime.
  bool get isJit => jitModes.contains(this);

  /// Whether this mode is using the precompiled runtime.
  bool get isPrecompiled => !isJit;

  String get cliName => snakeCase(name);

  @override
  String toString() => cliName;
}

/// Environment type of the target device.
enum EnvironmentType { physical, simulator }

String? validatedBuildNumberForPlatform(
  TargetPlatform targetPlatform,
  String? buildNumber,
  Logger logger,
) {
  if (buildNumber == null) {
    return null;
  }
  if (targetPlatform == TargetPlatform.ios || targetPlatform == TargetPlatform.darwin) {
    // See CFBundleVersion at https://developer.apple.com/library/archive/documentation/General/Reference/InfoPlistKeyReference/Articles/CoreFoundationKeys.html
    final RegExp disallowed = RegExp(r'[^\d\.]');
    String tmpBuildNumber = buildNumber.replaceAll(disallowed, '');
    if (tmpBuildNumber.isEmpty) {
      return null;
    }
    final List<String> segments =
        tmpBuildNumber.split('.').where((String segment) => segment.isNotEmpty).toList();
    if (segments.isEmpty) {
      segments.add('0');
    }
    tmpBuildNumber = segments.join('.');
    if (tmpBuildNumber != buildNumber) {
      logger.printTrace(
        'Invalid build-number: $buildNumber for iOS/macOS, overridden by $tmpBuildNumber.\n'
        'See CFBundleVersion at https://developer.apple.com/library/archive/documentation/General/Reference/InfoPlistKeyReference/Articles/CoreFoundationKeys.html',
      );
    }
    return tmpBuildNumber;
  }
  if (targetPlatform == TargetPlatform.android_arm ||
      targetPlatform == TargetPlatform.android_arm64 ||
      targetPlatform == TargetPlatform.android_x64 ||
      targetPlatform == TargetPlatform.android_x86) {
    // See versionCode at https://developer.android.com/studio/publish/versioning
    final RegExp disallowed = RegExp(r'[^\d]');
    String tmpBuildNumberStr = buildNumber.replaceAll(disallowed, '');
    int tmpBuildNumberInt = int.tryParse(tmpBuildNumberStr) ?? 0;
    if (tmpBuildNumberInt < 1) {
      tmpBuildNumberInt = 1;
    }
    tmpBuildNumberStr = tmpBuildNumberInt.toString();
    if (tmpBuildNumberStr != buildNumber) {
      logger.printTrace(
        'Invalid build-number: $buildNumber for Android, overridden by $tmpBuildNumberStr.\n'
        'See versionCode at https://developer.android.com/studio/publish/versioning',
      );
    }
    return tmpBuildNumberStr;
  }
  return buildNumber;
}

String? validatedBuildNameForPlatform(
  TargetPlatform targetPlatform,
  String? buildName,
  Logger logger,
) {
  if (buildName == null) {
    return null;
  }
  if (targetPlatform == TargetPlatform.ios || targetPlatform == TargetPlatform.darwin) {
    // See CFBundleShortVersionString at https://developer.apple.com/library/archive/documentation/General/Reference/InfoPlistKeyReference/Articles/CoreFoundationKeys.html
    final RegExp disallowed = RegExp(r'[^\d\.]');
    String tmpBuildName = buildName.replaceAll(disallowed, '');
    if (tmpBuildName.isEmpty) {
      return null;
    }
    final List<String> segments =
        tmpBuildName.split('.').where((String segment) => segment.isNotEmpty).toList();
    while (segments.length < 3) {
      segments.add('0');
    }
    tmpBuildName = segments.join('.');
    if (tmpBuildName != buildName) {
      logger.printTrace(
        'Invalid build-name: $buildName for iOS/macOS, overridden by $tmpBuildName.\n'
        'See CFBundleShortVersionString at https://developer.apple.com/library/archive/documentation/General/Reference/InfoPlistKeyReference/Articles/CoreFoundationKeys.html',
      );
    }
    return tmpBuildName;
  }
  if (targetPlatform == TargetPlatform.android ||
      targetPlatform == TargetPlatform.android_arm ||
      targetPlatform == TargetPlatform.android_arm64 ||
      targetPlatform == TargetPlatform.android_x64 ||
      targetPlatform == TargetPlatform.android_x86) {
    // See versionName at https://developer.android.com/studio/publish/versioning
    return buildName;
  }
  return buildName;
}

String getFriendlyModeName(BuildMode mode) {
  return snakeCase(mode.cliName).replaceAll('_', ' ');
}

// Returns true if the selected build mode uses ahead-of-time compilation.
bool isAotBuildMode(BuildMode mode) {
  return mode == BuildMode.profile || mode == BuildMode.release;
}

// Returns true if the given build mode can be used on emulators / simulators.
bool isEmulatorBuildMode(BuildMode mode) {
  return mode == BuildMode.debug;
}

enum TargetPlatform {
  android,
  ios,
  darwin,
  linux_x64,
  linux_arm64,
  windows_x64,
  windows_arm64,
  fuchsia_arm64,
  fuchsia_x64,
  tester,
  web_javascript,
  // The arch specific android target platforms are soft-deprecated.
  // Instead of using TargetPlatform as a combination arch + platform
  // the code will be updated to carry arch information in [DarwinArch]
  // and [AndroidArch].
  android_arm,
  android_arm64,
  android_x64,
  android_x86;

  String get fuchsiaArchForTargetPlatform {
    switch (this) {
      case TargetPlatform.fuchsia_arm64:
        return 'arm64';
      case TargetPlatform.fuchsia_x64:
        return 'x64';
      case TargetPlatform.android:
      case TargetPlatform.android_arm:
      case TargetPlatform.android_arm64:
      case TargetPlatform.android_x64:
      case TargetPlatform.android_x86:
      case TargetPlatform.darwin:
      case TargetPlatform.ios:
      case TargetPlatform.linux_arm64:
      case TargetPlatform.linux_x64:
      case TargetPlatform.tester:
      case TargetPlatform.web_javascript:
      case TargetPlatform.windows_x64:
      case TargetPlatform.windows_arm64:
        throw UnsupportedError('Unexpected Fuchsia platform $this');
    }
  }

  String get simpleName {
    switch (this) {
      case TargetPlatform.linux_x64:
      case TargetPlatform.darwin:
      case TargetPlatform.windows_x64:
        return 'x64';
      case TargetPlatform.linux_arm64:
      case TargetPlatform.windows_arm64:
        return 'arm64';
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
        throw UnsupportedError('Unexpected target platform $this');
    }
  }
}

/// iOS and macOS target device architecture.
//
// TODO(cbracken): split TargetPlatform.ios into ios_armv7, ios_arm64.
enum DarwinArch {
  armv7, // Deprecated. Used to display 32-bit unsupported devices.
  arm64,
  x86_64;

  /// Returns the Dart SDK's name for the specified target architecture.
  ///
  /// When building for Darwin platforms, the tool invokes architecture-specific
  /// variants of `gen_snapshot`, one for each target architecture. The output
  /// instructions are then built into architecture-specific binaries, which are
  /// merged into a universal binary using the `lipo` tool.
  String get dartName {
    return switch (this) {
      DarwinArch.armv7 => 'armv7',
      DarwinArch.arm64 => 'arm64',
      DarwinArch.x86_64 => 'x64',
    };
  }
}

// TODO(zanderso): replace all android TargetPlatform usage with AndroidArch.
enum AndroidArch {
  armeabi_v7a,
  arm64_v8a,
  x86,
  x86_64;

  String get archName => switch (this) {
    AndroidArch.armeabi_v7a => 'armeabi-v7a',
    AndroidArch.arm64_v8a => 'arm64-v8a',
    AndroidArch.x86_64 => 'x86_64',
    AndroidArch.x86 => 'x86',
  };

  String get platformName => switch (this) {
    AndroidArch.armeabi_v7a => 'android-arm',
    AndroidArch.arm64_v8a => 'android-arm64',
    AndroidArch.x86_64 => 'android-x64',
    AndroidArch.x86 => 'android-x86',
  };
}

/// The default set of iOS device architectures to build for.
List<DarwinArch> defaultIOSArchsForEnvironment(
  EnvironmentType environmentType,
  Artifacts artifacts,
) {
  // Handle single-arch local engines.
  final LocalEngineInfo? localEngineInfo = artifacts.localEngineInfo;
  if (localEngineInfo != null) {
    final String localEngineName = localEngineInfo.localTargetName;
    if (localEngineName.contains('_arm64')) {
      return <DarwinArch>[DarwinArch.arm64];
    }
    if (localEngineName.contains('_sim')) {
      return <DarwinArch>[DarwinArch.x86_64];
    }
  } else if (environmentType == EnvironmentType.simulator) {
    return <DarwinArch>[DarwinArch.x86_64, DarwinArch.arm64];
  }
  return <DarwinArch>[DarwinArch.arm64];
}

/// The default set of macOS device architectures to build for.
List<DarwinArch> defaultMacOSArchsForEnvironment(Artifacts artifacts) {
  // Handle single-arch local engines.
  final LocalEngineInfo? localEngineInfo = artifacts.localEngineInfo;
  if (localEngineInfo != null) {
    if (localEngineInfo.localTargetName.contains('_arm64')) {
      return <DarwinArch>[DarwinArch.arm64];
    }
    return <DarwinArch>[DarwinArch.x86_64];
  }
  return <DarwinArch>[DarwinArch.x86_64, DarwinArch.arm64];
}

DarwinArch getIOSArchForName(String arch) {
  switch (arch) {
    case 'armv7':
    case 'armv7f': // iPhone 4S.
    case 'armv7s': // iPad 4.
      return DarwinArch.armv7;
    case 'arm64':
    case 'arm64e': // iPhone XS/XS Max/XR and higher. arm64 runs on arm64e devices.
      return DarwinArch.arm64;
    case 'x86_64':
      return DarwinArch.x86_64;
  }
  throw Exception('Unsupported iOS arch name "$arch"');
}

DarwinArch getDarwinArchForName(String arch) {
  return switch (arch) {
    'arm64' => DarwinArch.arm64,
    'x86_64' => DarwinArch.x86_64,
    _ => throw Exception('Unsupported MacOS arch name "$arch"'),
  };
}

String getNameForTargetPlatform(TargetPlatform platform, {DarwinArch? darwinArch}) {
  return switch (platform) {
    TargetPlatform.ios when darwinArch != null => 'ios-${darwinArch.name}',
    TargetPlatform.darwin when darwinArch != null => 'darwin-${darwinArch.name}',
    TargetPlatform.ios => 'ios',
    TargetPlatform.darwin => 'darwin',
    TargetPlatform.android_arm => 'android-arm',
    TargetPlatform.android_arm64 => 'android-arm64',
    TargetPlatform.android_x64 => 'android-x64',
    TargetPlatform.android_x86 => 'android-x86',
    TargetPlatform.linux_x64 => 'linux-x64',
    TargetPlatform.linux_arm64 => 'linux-arm64',
    TargetPlatform.windows_x64 => 'windows-x64',
    TargetPlatform.windows_arm64 => 'windows-arm64',
    TargetPlatform.fuchsia_arm64 => 'fuchsia-arm64',
    TargetPlatform.fuchsia_x64 => 'fuchsia-x64',
    TargetPlatform.tester => 'flutter-tester',
    TargetPlatform.web_javascript => 'web-javascript',
    TargetPlatform.android => 'android',
  };
}

TargetPlatform getTargetPlatformForName(String platform) {
  return switch (platform) {
    'android' => TargetPlatform.android,
    'android-arm' => TargetPlatform.android_arm,
    'android-arm64' => TargetPlatform.android_arm64,
    'android-x64' => TargetPlatform.android_x64,
    'android-x86' => TargetPlatform.android_x86,
    'fuchsia-arm64' => TargetPlatform.fuchsia_arm64,
    'fuchsia-x64' => TargetPlatform.fuchsia_x64,
    'ios' => TargetPlatform.ios,
    // For backward-compatibility and also for Tester, where it must match
    // host platform name (HostPlatform.darwin_x64)
    'darwin' || 'darwin-x64' || 'darwin-arm64' => TargetPlatform.darwin,
    'linux-x64' => TargetPlatform.linux_x64,
    'linux-arm64' => TargetPlatform.linux_arm64,
    'windows-x64' => TargetPlatform.windows_x64,
    'windows-arm64' => TargetPlatform.windows_arm64,
    'web-javascript' => TargetPlatform.web_javascript,
    'flutter-tester' => TargetPlatform.tester,
    _ => throw Exception('Unsupported platform name "$platform"'),
  };
}

AndroidArch getAndroidArchForName(String platform) {
  return switch (platform) {
    'android-arm' => AndroidArch.armeabi_v7a,
    'android-arm64' => AndroidArch.arm64_v8a,
    'android-x64' => AndroidArch.x86_64,
    'android-x86' => AndroidArch.x86,
    _ => throw Exception('Unsupported Android arch name "$platform"'),
  };
}

DarwinArch getCurrentDarwinArch() {
  return switch (globals.os.hostPlatform) {
    HostPlatform.darwin_arm64 => DarwinArch.arm64,
    HostPlatform.darwin_x64 => DarwinArch.x86_64,
    final HostPlatform unsupported =>
      throw Exception('Unsupported Darwin host platform "$unsupported"'),
  };
}

HostPlatform getCurrentHostPlatform() {
  if (globals.platform.isMacOS) {
    return switch (getCurrentDarwinArch()) {
      DarwinArch.arm64 => HostPlatform.darwin_arm64,
      DarwinArch.x86_64 => HostPlatform.darwin_x64,
      DarwinArch.armv7 => throw Exception('Unsupported macOS arch "amv7"'),
    };
  }
  if (globals.platform.isLinux) {
    // support x64 and arm64 architecture.
    return globals.os.hostPlatform;
  }
  if (globals.platform.isWindows) {
    return HostPlatform.windows_x64;
  }

  globals.printWarning('Unsupported host platform, defaulting to Linux');

  return HostPlatform.linux_x64;
}

/// Returns the top-level build output directory.
String getBuildDirectory([Config? config, FileSystem? fileSystem]) {
  // TODO(andrewkolos): Prefer required parameters instead of falling back to globals.
  // TODO(johnmccutchan): Stop calling this function as part of setting
  // up command line argument processing.
  final Config localConfig = config ?? globals.config;
  final FileSystem localFilesystem = fileSystem ?? globals.fs;

  final String buildDir = localConfig.getValue('build-dir') as String? ?? 'build';
  if (localFilesystem.path.isAbsolute(buildDir)) {
    throw Exception('build-dir config setting in ${globals.config.configPath} must be relative');
  }
  return buildDir;
}

/// Returns the Android build output directory.
String getAndroidBuildDirectory() {
  // TODO(cbracken): move to android subdir.
  return getBuildDirectory();
}

/// Returns the AOT build output directory.
String getAotBuildDirectory() {
  return globals.fs.path.join(getBuildDirectory(), 'aot');
}

/// Returns the asset build output directory.
String getAssetBuildDirectory([Config? config, FileSystem? fileSystem]) {
  return (fileSystem ?? globals.fs).path.join(
    getBuildDirectory(config, fileSystem),
    'flutter_assets',
  );
}

/// Returns the iOS build output directory.
String getIosBuildDirectory() {
  return globals.fs.path.join(getBuildDirectory(), 'ios');
}

/// Returns the macOS build output directory.
String getMacOSBuildDirectory() {
  return globals.fs.path.join(getBuildDirectory(), 'macos');
}

/// Returns the web build output directory.
String getWebBuildDirectory() {
  return globals.fs.path.join(getBuildDirectory(), 'web');
}

/// Returns the Linux build output directory.
String getLinuxBuildDirectory([TargetPlatform? targetPlatform]) {
  final String arch =
      (targetPlatform == null) ? _getCurrentHostPlatformArchName() : targetPlatform.simpleName;
  final String subDirs = 'linux/$arch';
  return globals.fs.path.join(getBuildDirectory(), subDirs);
}

/// Returns the Windows build output directory.
String getWindowsBuildDirectory(TargetPlatform targetPlatform) {
  final String arch = targetPlatform.simpleName;
  return globals.fs.path.join(getBuildDirectory(), 'windows', arch);
}

/// Returns the Fuchsia build output directory.
String getFuchsiaBuildDirectory() {
  return globals.fs.path.join(getBuildDirectory(), 'fuchsia');
}

/// Defines specified via the `--dart-define` command-line option.
///
/// These values are URI-encoded and then combined into a comma-separated string.
const String kDartDefines = 'DartDefines';

/// The define to pass a [BuildMode].
const String kBuildMode = 'BuildMode';

/// The define to pass whether we compile 64-bit android-arm code.
const String kTargetPlatform = 'TargetPlatform';

/// The define to control what target file is used.
const String kTargetFile = 'TargetFile';

/// Whether to enable or disable track widget creation.
const String kTrackWidgetCreation = 'TrackWidgetCreation';

/// If provided, the frontend server will be started in JIT mode from this file.
const String kFrontendServerStarterPath = 'FrontendServerStarterPath';

/// Additional configuration passed to the dart front end.
///
/// This is expected to be a comma separated list of strings.
const String kExtraFrontEndOptions = 'ExtraFrontEndOptions';

/// Additional configuration passed to gen_snapshot.
///
/// This is expected to be a comma separated list of strings.
const String kExtraGenSnapshotOptions = 'ExtraGenSnapshotOptions';

/// Whether the build should run gen_snapshot as a split aot build for deferred
/// components.
const String kDeferredComponents = 'DeferredComponents';

/// Whether to strip source code information out of release builds and where to save it.
const String kSplitDebugInfo = 'SplitDebugInfo';

/// Alternative scheme for file URIs.
///
/// May be used along with [kFileSystemRoots] to support a multi-root
/// filesystem.
const String kFileSystemScheme = 'FileSystemScheme';

/// Additional filesystem roots.
///
/// If provided, must be used along with [kFileSystemScheme].
const String kFileSystemRoots = 'FileSystemRoots';

/// The define to control what iOS architectures are built for.
///
/// This is expected to be a space-delimited list of architectures. If not
/// provided, defaults to arm64.
const String kIosArchs = 'IosArchs';

/// The define to control what macOS architectures are built for.
///
/// This is expected to be a space-delimited list of architectures. If not
/// provided, defaults to x86_64 and arm64.
///
/// Supported values are x86_64 and arm64.
const String kDarwinArchs = 'DarwinArchs';

/// The define to control what Android architectures are built for.
///
/// This is expected to be a space-delimited list of architectures.
const String kAndroidArchs = 'AndroidArchs';

/// The define to control what min Android SDK version is built for.
///
/// This is expected to be int.
///
/// If not provided, defaults to `minSdkVersion` from gradle_utils.dart.
///
/// This is passed in by flutter.groovy's invocation of `flutter assemble`.
///
/// For more info, see:
/// https://developer.android.com/ndk/guides/sdk-versions#minsdkversion
/// https://developer.android.com/ndk/guides/other_build_systems#overview
const String kMinSdkVersion = 'MinSdkVersion';

/// Path to the SDK root to be used as the isysroot.
const String kSdkRoot = 'SdkRoot';

/// Whether to enable Dart obfuscation and where to save the symbol map.
const String kDartObfuscation = 'DartObfuscation';

/// Whether to enable Native Assets.
///
/// If true, native assets are built and the mapping for native assets lookup
/// at runtime is embedded in the kernel file.
///
/// If false, native assets are not built, and an empty mapping is embedded in
/// the kernel file. Used for targets that trigger kernel builds but
/// are not OS/architecture specific.
///
/// Supported values are 'true' and 'false'.
///
/// Defaults to 'true'.
const String kNativeAssets = 'NativeAssets';

/// An output directory where one or more code-size measurements may be written.
const String kCodeSizeDirectory = 'CodeSizeDirectory';

/// SHA identifier of the Apple developer code signing identity.
///
/// Same as EXPANDED_CODE_SIGN_IDENTITY Xcode build setting.
/// Also discoverable via `security find-identity -p codesigning`.
const String kCodesignIdentity = 'CodesignIdentity';

/// The build define controlling whether icon fonts should be stripped down to
/// only the glyphs used by the application.
const String kIconTreeShakerFlag = 'TreeShakeIcons';

/// Controls whether a web build should use local canvaskit or the CDN
const String kUseLocalCanvasKitFlag = 'UseLocalCanvasKit';

/// The input key for an SkSL bundle path.
const String kBundleSkSLPath = 'BundleSkSLPath';

/// The define to pass build name
const String kBuildName = 'BuildName';

/// The app flavor to build.
const String kFlavor = 'Flavor';

/// The define to pass build number
const String kBuildNumber = 'BuildNumber';

/// The action Xcode is taking.
///
/// Will be "build" when building and "install" when archiving.
const String kXcodeAction = 'Action';

/// The define of the Xcode build Pre-action.
///
/// Will be "PrepareFramework" when copying the Flutter/FlutterMacOS framework
/// to the BUILT_PRODUCTS_DIR prior to the build.
const String kXcodePreAction = 'PreBuildAction';

final Converter<String, String> _defineEncoder = utf8.encoder.fuse(base64.encoder);
final Converter<String, String> _defineDecoder = base64.decoder.fuse(utf8.decoder);

/// Encode a List of dart defines in a base64 string.
///
/// This encoding does not include `,`, which is used to distinguish
/// the individual entries, nor does it include `%` which is often a
/// control character on windows command lines.
///
/// When decoding this string, it can be safely split on commas, since any
/// user provided commands will still be encoded.
///
/// If the presence of the `/` character ends up being an issue, this can
/// be changed to use base32 instead.
String encodeDartDefines(List<String> defines) {
  return defines.map(_defineEncoder.convert).join(',');
}

List<String> decodeCommaSeparated(Map<String, String> environmentDefines, String key) {
  if (!environmentDefines.containsKey(key) || environmentDefines[key]!.isEmpty) {
    return <String>[];
  }
  return environmentDefines[key]!.split(',').cast<String>().toList();
}

/// Dart defines are encoded inside [environmentDefines] as a comma-separated list.
List<String> decodeDartDefines(Map<String, String> environmentDefines, String key) {
  if (!environmentDefines.containsKey(key) || environmentDefines[key]!.isEmpty) {
    return <String>[];
  }
  return environmentDefines[key]!
      .split(',')
      .map<Object>(_defineDecoder.convert)
      .cast<String>()
      .toList();
}

/// The null safety runtime mode the app should be built in.
enum NullSafetyMode {
  sound,
  unsound,

  /// The null safety mode was not detected. Only supported for 'flutter test'.
  autodetect,
}

/// Indicates the module system DDC is targeting.
enum DdcModuleFormat { amd, ddc }

// TODO(markzipan): delete this when DDC's AMD module system is deprecated, https://github.com/flutter/flutter/issues/142060.
DdcModuleFormat? _ddcModuleFormatFromFrontEndArgs(List<String>? extraFrontEndArgs) {
  if (extraFrontEndArgs == null) {
    return null;
  }
  const String ddcModuleFormatString = '--dartdevc-module-format=';
  for (final String flag in extraFrontEndArgs) {
    if (flag.startsWith(ddcModuleFormatString)) {
      final String moduleFormatString = flag.substring(ddcModuleFormatString.length, flag.length);
      return DdcModuleFormat.values.byName(moduleFormatString);
    }
  }
  return null;
}

String _getCurrentHostPlatformArchName() {
  final HostPlatform hostPlatform = getCurrentHostPlatform();
  return hostPlatform.platformName;
}

String? _uncapitalize(String? s) {
  if (s == null || s.isEmpty) {
    return s;
  }
  return s.substring(0, 1).toLowerCase() + s.substring(1);
}
