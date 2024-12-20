// Copyright 2014 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

// Logic for native assets shared between all host OSes.

import 'package:logging/logging.dart' as logging;
import 'package:native_assets_builder/native_assets_builder.dart';
import 'package:native_assets_cli/code_assets_builder.dart';
import 'package:native_assets_cli/native_assets_cli_internal.dart';
import 'package:package_config/package_config_types.dart';

import '../../base/common.dart';
import '../../base/file_system.dart';
import '../../base/logger.dart';
import '../../base/platform.dart';
import '../../build_info.dart' as build_info;
import '../../build_system/exceptions.dart';
import '../../cache.dart';
import '../../convert.dart';
import '../../features.dart';
import '../../globals.dart' as globals;
import '../../macos/xcode.dart' as xcode;
import 'android/native_assets.dart';
import 'ios/native_assets.dart';
import 'linux/native_assets.dart';
import 'macos/native_assets.dart';
import 'macos/native_assets_host.dart';
import 'windows/native_assets.dart';

/// The assets produced by a Dart build and the dependencies of those assets.
///
/// If any of the dependencies change, then the Dart build should be performed
/// again.
final class DartBuildResult {
  const DartBuildResult(this.codeAssets, this.dependencies);

  const DartBuildResult.empty() : codeAssets = const <CodeAsset>[], dependencies = const <Uri>[];

  factory DartBuildResult.fromJson(Map<String, Object?> json) {
    final List<Uri> dependencies = <Uri>[
      for (final Object? encodedUri in json['dependencies']! as List<Object?>)
        Uri.parse(encodedUri! as String),
    ];
    final List<CodeAsset> codeAssets = <CodeAsset>[
      for (final Object? json in json['code_assets']! as List<Object?>)
        CodeAsset.fromEncoded(EncodedAsset.fromJson(json! as Map<String, Object?>)),
    ];
    return DartBuildResult(codeAssets, dependencies);
  }

  final List<CodeAsset> codeAssets;
  final List<Uri> dependencies;

  Map<String, Object?> toJson() => <String, Object?>{
    'dependencies': <Object?>[for (final Uri dep in dependencies) dep.toString()],
    'code_assets': <Object?>[for (final CodeAsset code in codeAssets) code.encode().toJson()],
  };

  /// The files that eventually should be bundled with the app.
  List<Uri> get filesToBeBundled => <Uri>[
    for (final CodeAsset code in codeAssets)
      if (code.linkMode is DynamicLoadingBundled) code.file!,
  ];
}

/// Invokes the build of all transitive Dart packages and prepares code assets
/// to be included in the native build.
Future<DartBuildResult> runFlutterSpecificDartBuild({
  required Map<String, String> environmentDefines,
  required FlutterNativeAssetsBuildRunner buildRunner,
  required build_info.TargetPlatform targetPlatform,
  required Uri projectUri,
  required FileSystem fileSystem,
}) async {
  final OS targetOS = getNativeOSFromTargetPlatform(targetPlatform);
  final Uri buildUri = nativeAssetsBuildUri(projectUri, targetOS);
  final Directory buildDir = fileSystem.directory(buildUri);

  final bool flutterTester = targetPlatform == build_info.TargetPlatform.tester;

  if (!await buildDir.exists()) {
    // Ensure the folder exists so the native build system can copy it even
    // if there's no native assets.
    await buildDir.create(recursive: true);
  }

  if (!await _nativeBuildRequired(buildRunner)) {
    return const DartBuildResult.empty();
  }

  final build_info.BuildMode buildMode = _getBuildMode(environmentDefines, flutterTester);
  final List<Architecture> architectures =
      flutterTester
          ? <Architecture>[Architecture.current]
          : _architecturesForOS(targetPlatform, targetOS, environmentDefines);
  final DartBuildResult result =
      architectures.isEmpty
          ? const DartBuildResult.empty()
          : await _runDartBuild(
            environmentDefines: environmentDefines,
            buildRunner: buildRunner,
            architectures: architectures,
            projectUri: projectUri,
            buildMode: _nativeAssetsBuildMode(buildMode),
            fileSystem: fileSystem,
            targetOS: targetOS,
          );
  return result;
}

Future<void> installCodeAssets({
  required DartBuildResult dartBuildResult,
  required Map<String, String> environmentDefines,
  required build_info.TargetPlatform targetPlatform,
  required Uri projectUri,
  required FileSystem fileSystem,
  required Uri nativeAssetsFileUri,
}) async {
  final OS targetOS = getNativeOSFromTargetPlatform(targetPlatform);
  final Uri buildUri = nativeAssetsBuildUri(projectUri, targetOS);
  final bool flutterTester = targetPlatform == build_info.TargetPlatform.tester;
  final build_info.BuildMode buildMode = _getBuildMode(environmentDefines, flutterTester);

  final String? codesignIdentity = environmentDefines[build_info.kCodesignIdentity];
  final Map<CodeAsset, KernelAsset> assetTargetLocations = assetTargetLocationsForOS(
    targetOS,
    dartBuildResult.codeAssets,
    flutterTester,
    buildUri,
  );
  await _copyNativeCodeAssetsForOS(
    targetOS,
    buildUri,
    buildMode,
    fileSystem,
    assetTargetLocations,
    codesignIdentity,
    flutterTester,
  );
  await _writeNativeAssetsJson(
    assetTargetLocations.values.toList(),
    nativeAssetsFileUri,
    fileSystem,
  );
}

/// Programmatic API to be used by Dart launchers to invoke native builds.
///
/// It enables mocking `package:native_assets_builder` package.
/// It also enables mocking native toolchain discovery via [cCompilerConfig].
abstract interface class FlutterNativeAssetsBuildRunner {
  /// Whether the project has a `.dart_tools/package_config.json`.
  ///
  /// If there is no package config, [packagesWithNativeAssets], [build] and
  /// [link] must not be invoked.
  Future<bool> hasPackageConfig();

  /// All packages in the transitive dependencies that have a `build.dart`.
  Future<List<Package>> packagesWithNativeAssets();

  /// Runs all [packagesWithNativeAssets] `build.dart`.
  Future<BuildResult?> build({
    required List<String> supportedAssetTypes,
    required BuildConfigValidator configValidator,
    required BuildConfigCreator configCreator,
    required BuildValidator buildValidator,
    required ApplicationAssetValidator applicationAssetValidator,
    required bool includeParentEnvironment,
    required BuildMode buildMode,
    required OS targetOS,
    required Uri workingDirectory,
    required bool linkingEnabled,
  });

  /// Runs all [packagesWithNativeAssets] `link.dart`.
  Future<LinkResult?> link({
    required List<String> supportedAssetTypes,
    required LinkConfigValidator configValidator,
    required LinkConfigCreator configCreator,
    required LinkValidator linkValidator,
    required ApplicationAssetValidator applicationAssetValidator,
    required bool includeParentEnvironment,
    required BuildMode buildMode,
    required OS targetOS,
    required Uri workingDirectory,
    required BuildResult buildResult,
  });

  /// The C compiler config to use for compilation.
  Future<CCompilerConfig> get cCompilerConfig;

  /// The NDK compiler to use to use for compilation for Android.
  Future<CCompilerConfig> get ndkCCompilerConfig;
}

/// Uses `package:native_assets_builder` for its implementation.
class FlutterNativeAssetsBuildRunnerImpl implements FlutterNativeAssetsBuildRunner {
  FlutterNativeAssetsBuildRunnerImpl(
    this.projectUri,
    this.packageConfigPath,
    this.packageConfig,
    this.fileSystem,
    this.logger,
  );

  final Uri projectUri;
  final String packageConfigPath;
  final PackageConfig packageConfig;
  final FileSystem fileSystem;
  final Logger logger;

  late final logging.Logger _logger = logging.Logger('')
    ..onRecord.listen((logging.LogRecord record) {
      final int levelValue = record.level.value;
      final String message = record.message;
      if (levelValue >= logging.Level.SEVERE.value) {
        logger.printError(message);
      } else if (levelValue >= logging.Level.WARNING.value) {
        logger.printWarning(message);
      } else if (levelValue >= logging.Level.INFO.value) {
        logger.printTrace(message);
      } else {
        logger.printTrace(message);
      }
    });

  late final Uri _dartExecutable = fileSystem.directory(Cache.flutterRoot).uri.resolve('bin/dart');

  late final NativeAssetsBuildRunner _buildRunner = NativeAssetsBuildRunner(
    logger: _logger,
    dartExecutable: _dartExecutable,
  );

  @override
  Future<bool> hasPackageConfig() {
    return fileSystem.file(packageConfigPath).exists();
  }

  @override
  Future<List<Package>> packagesWithNativeAssets() async {
    final PackageLayout packageLayout = PackageLayout.fromPackageConfig(
      packageConfig,
      Uri.file(packageConfigPath),
    );
    // It suffices to only check for build hooks. If no packages have a build
    // hook. Then no build hook will output any assets for any link hook, and
    // thus the link hooks will never be run.
    return packageLayout.packagesWithAssets(Hook.build);
  }

  @override
  @override
  Future<BuildResult?> build({
    required List<String> supportedAssetTypes,
    required BuildConfigValidator configValidator,
    required BuildConfigCreator configCreator,
    required BuildValidator buildValidator,
    required ApplicationAssetValidator applicationAssetValidator,
    required bool includeParentEnvironment,
    required BuildMode buildMode,
    required OS targetOS,
    required Uri workingDirectory,
    required bool linkingEnabled,
  }) {
    final PackageLayout packageLayout = PackageLayout.fromPackageConfig(
      packageConfig,
      Uri.file(packageConfigPath),
    );
    return _buildRunner.build(
      supportedAssetTypes: supportedAssetTypes,
      configCreator: configCreator,
      configValidator: configValidator,
      buildValidator: buildValidator,
      applicationAssetValidator: applicationAssetValidator,
      buildMode: buildMode,
      includeParentEnvironment: includeParentEnvironment,
      targetOS: targetOS,
      workingDirectory: workingDirectory,
      packageLayout: packageLayout,
      linkingEnabled: linkingEnabled,
    );
  }

  @override
  Future<LinkResult?> link({
    required List<String> supportedAssetTypes,
    required LinkConfigValidator configValidator,
    required LinkConfigCreator configCreator,
    required LinkValidator linkValidator,
    required ApplicationAssetValidator applicationAssetValidator,
    required bool includeParentEnvironment,
    required BuildMode buildMode,
    required OS targetOS,
    required Uri workingDirectory,
    required BuildResult buildResult,
  }) {
    final PackageLayout packageLayout = PackageLayout.fromPackageConfig(
      packageConfig,
      Uri.file(packageConfigPath),
    );
    return _buildRunner.link(
      supportedAssetTypes: supportedAssetTypes,
      configCreator: configCreator,
      configValidator: configValidator,
      linkValidator: linkValidator,
      applicationAssetValidator: applicationAssetValidator,
      buildMode: buildMode,
      includeParentEnvironment: includeParentEnvironment,
      targetOS: targetOS,
      workingDirectory: workingDirectory,
      packageLayout: packageLayout,
      buildResult: buildResult,
    );
  }

  @override
  late final Future<CCompilerConfig> cCompilerConfig = () {
    if (globals.platform.isMacOS || globals.platform.isIOS) {
      return cCompilerConfigMacOS();
    }
    if (globals.platform.isLinux) {
      return cCompilerConfigLinux();
    }
    if (globals.platform.isWindows) {
      return cCompilerConfigWindows();
    }
    if (globals.platform.isAndroid) {
      throwToolExit('Should use ndkCCompilerConfig for Android.');
    }
    throwToolExit('Unknown target OS.');
  }();

  @override
  late final Future<CCompilerConfig> ndkCCompilerConfig = () {
    return cCompilerConfigAndroid();
  }();
}

Future<Uri> _writeNativeAssetsJson(
  List<KernelAsset> assets,
  Uri nativeAssetsJsonUri,
  FileSystem fileSystem,
) async {
  globals.logger.printTrace('Writing native assets json to $nativeAssetsJsonUri.');
  final String nativeAssetsDartContents = _toNativeAssetsJsonFile(assets);
  final File nativeAssetsFile = fileSystem.file(nativeAssetsJsonUri);
  final Directory parentDirectory = nativeAssetsFile.parent;
  if (!await parentDirectory.exists()) {
    await parentDirectory.create(recursive: true);
  }
  await nativeAssetsFile.writeAsString(nativeAssetsDartContents);
  globals.logger.printTrace('Writing ${nativeAssetsFile.path} done.');
  return nativeAssetsFile.uri;
}

String _toNativeAssetsJsonFile(List<KernelAsset> kernelAssets) {
  final Map<Target, List<KernelAsset>> assetsPerTarget = <Target, List<KernelAsset>>{};
  for (final KernelAsset asset in kernelAssets) {
    assetsPerTarget.putIfAbsent(asset.target, () => <KernelAsset>[]).add(asset);
  }

  const String formatVersionKey = 'format-version';
  const String nativeAssetsKey = 'native-assets';

  // See assets/native_assets.cc in the engine for the expected format.
  final Map<String, Object> jsonContents = <String, Object>{
    formatVersionKey: const <int>[1, 0, 0],
    nativeAssetsKey: <String, Map<String, List<String>>>{
      for (final MapEntry<Target, List<KernelAsset>> entry in assetsPerTarget.entries)
        entry.key.toString(): <String, List<String>>{
          for (final KernelAsset e in entry.value) e.id: e.path.toJson(),
        },
    },
  };

  return jsonEncode(jsonContents);
}

/// Select the native asset build mode for a given Flutter build mode.
BuildMode _nativeAssetsBuildMode(build_info.BuildMode buildMode) {
  switch (buildMode) {
    case build_info.BuildMode.debug:
      return BuildMode.debug;
    case build_info.BuildMode.jitRelease:
    case build_info.BuildMode.profile:
    case build_info.BuildMode.release:
      return BuildMode.release;
  }
}

/// Checks whether this project does not yet have a package config file.
///
/// A project has no package config when `pub get` has not yet been run.
///
/// Native asset builds cannot be run without a package config. If there is
/// no package config, leave a logging trace about that.
Future<bool> _hasNoPackageConfig(FlutterNativeAssetsBuildRunner buildRunner) async {
  final bool packageConfigExists = await buildRunner.hasPackageConfig();
  if (!packageConfigExists) {
    globals.logger.printTrace('No package config found. Skipping native assets compilation.');
  }
  return !packageConfigExists;
}

Future<bool> _nativeBuildRequired(FlutterNativeAssetsBuildRunner buildRunner) async {
  if (await _hasNoPackageConfig(buildRunner)) {
    return false;
  }
  final List<Package> packagesWithNativeAssets = await buildRunner.packagesWithNativeAssets();
  if (packagesWithNativeAssets.isEmpty) {
    globals.logger.printTrace(
      'No packages with native assets. Skipping native assets compilation.',
    );
    return false;
  }

  if (!featureFlags.isNativeAssetsEnabled) {
    final String packageNames = packagesWithNativeAssets.map((Package p) => p.name).join(' ');
    throwToolExit(
      'Package(s) $packageNames require the native assets feature to be enabled. '
      'Enable using `flutter config --enable-native-assets`.',
    );
  }
  return true;
}

/// Ensures that either this project has no native assets, or that native assets
/// are supported on that operating system.
///
/// Exits the tool if the above condition is not satisfied.
Future<void> ensureNoNativeAssetsOrOsIsSupported(
  Uri workingDirectory,
  String os,
  FileSystem fileSystem,
  FlutterNativeAssetsBuildRunner buildRunner,
) async {
  if (await _hasNoPackageConfig(buildRunner)) {
    return;
  }
  final List<Package> packagesWithNativeAssets = await buildRunner.packagesWithNativeAssets();
  if (packagesWithNativeAssets.isEmpty) {
    globals.logger.printTrace(
      'No packages with native assets. Skipping native assets compilation.',
    );
    return;
  }
  final String packageNames = packagesWithNativeAssets.map((Package p) => p.name).join(' ');
  throwToolExit(
    'Package(s) $packageNames require the native assets feature. '
    'This feature has not yet been implemented for `$os`. '
    'For more info see https://github.com/flutter/flutter/issues/129757.',
  );
}

/// This should be the same for different archs, debug/release, etc.
/// It should work for all macOS.
Uri nativeAssetsBuildUri(Uri projectUri, OS os) {
  final String buildDir = build_info.getBuildDirectory();
  return projectUri.resolve('$buildDir/native_assets/$os/');
}

Map<CodeAsset, KernelAsset> _assetTargetLocationsWindowsLinux(
  List<CodeAsset> assets,
  Uri? absolutePath,
) {
  return <CodeAsset, KernelAsset>{
    for (final CodeAsset asset in assets)
      asset: _targetLocationSingleArchitecture(asset, absolutePath),
  };
}

KernelAsset _targetLocationSingleArchitecture(CodeAsset asset, Uri? absolutePath) {
  final LinkMode linkMode = asset.linkMode;
  final KernelAssetPath kernelAssetPath;
  switch (linkMode) {
    case DynamicLoadingSystem _:
      kernelAssetPath = KernelAssetSystemPath(linkMode.uri);
    case LookupInExecutable _:
      kernelAssetPath = KernelAssetInExecutable();
    case LookupInProcess _:
      kernelAssetPath = KernelAssetInProcess();
    case DynamicLoadingBundled _:
      final String fileName = asset.file!.pathSegments.last;
      Uri uri;
      if (absolutePath != null) {
        // Flutter tester needs full host paths.
        uri = absolutePath.resolve(fileName);
      } else {
        // Flutter Desktop needs "absolute" paths inside the app.
        // "relative" in the context of native assets would be relative to the
        // kernel or aot snapshot.
        uri = Uri(path: fileName);
      }
      kernelAssetPath = KernelAssetAbsolutePath(uri);
    default:
      throw Exception('Unsupported asset link mode ${linkMode.runtimeType} in asset $asset');
  }
  return KernelAsset(
    id: asset.id,
    target: Target.fromArchitectureAndOS(asset.architecture!, asset.os),
    path: kernelAssetPath,
  );
}

Map<CodeAsset, KernelAsset> assetTargetLocationsForOS(
  OS targetOS,
  List<CodeAsset> codeAssets,
  bool flutterTester,
  Uri buildUri,
) {
  switch (targetOS) {
    case OS.windows:
    case OS.linux:
      final Uri? absolutePath = flutterTester ? buildUri : null;
      return _assetTargetLocationsWindowsLinux(codeAssets, absolutePath);
    case OS.macOS:
      final Uri? absolutePath = flutterTester ? buildUri : null;
      return assetTargetLocationsMacOS(codeAssets, absolutePath);
    case OS.iOS:
      return assetTargetLocationsIOS(codeAssets);
    case OS.android:
      return assetTargetLocationsAndroid(codeAssets);
    default:
      throw UnimplementedError('This should be unreachable.');
  }
}

Future<void> _copyNativeCodeAssetsForOS(
  OS targetOS,
  Uri buildUri,
  build_info.BuildMode buildMode,
  FileSystem fileSystem,
  Map<CodeAsset, KernelAsset> assetTargetLocations,
  String? codesignIdentity,
  bool flutterTester,
) async {
  // We only have to copy code assets that are bundled within the app.
  // If a code asset that use a linking mode of [LookupInProcess],
  // [LookupInExecutable] or [DynamicLoadingSystem] do not have anything to
  // bundle as part of the app.
  assetTargetLocations = <CodeAsset, KernelAsset>{
    for (final CodeAsset codeAsset in assetTargetLocations.keys)
      if (codeAsset.linkMode is DynamicLoadingBundled) codeAsset: assetTargetLocations[codeAsset]!,
  };

  if (assetTargetLocations.isEmpty) {
    return;
  }

  globals.logger.printTrace('Copying native assets to ${buildUri.toFilePath()}.');
  final List<CodeAsset> codeAssets = assetTargetLocations.keys.toList();
  switch (targetOS) {
    case OS.windows:
    case OS.linux:
      assert(codesignIdentity == null);
      await _copyNativeCodeAssetsToBundleOnWindowsLinux(
        buildUri,
        assetTargetLocations,
        buildMode,
        fileSystem,
      );
    case OS.macOS:
      if (flutterTester) {
        await copyNativeCodeAssetsMacOSFlutterTester(
          buildUri,
          fatAssetTargetLocationsMacOS(codeAssets, buildUri),
          codesignIdentity,
          buildMode,
          fileSystem,
        );
      } else {
        await copyNativeCodeAssetsMacOS(
          buildUri,
          fatAssetTargetLocationsMacOS(codeAssets, null),
          codesignIdentity,
          buildMode,
          fileSystem,
        );
      }
    case OS.iOS:
      await copyNativeCodeAssetsIOS(
        buildUri,
        fatAssetTargetLocationsIOS(codeAssets),
        codesignIdentity,
        buildMode,
        fileSystem,
      );
    case OS.android:
      assert(codesignIdentity == null);
      await copyNativeCodeAssetsAndroid(buildUri, assetTargetLocations, fileSystem);
    default:
      throw StateError('This should be unreachable.');
  }
  globals.logger.printTrace('Copying native assets done.');
}

/// Invokes the build of all transitive Dart packages.
///
/// This will invoke `hook/build.dart` and `hook/link.dart` (if applicable) for
/// all transitive dart packages that define such hooks.
Future<DartBuildResult> _runDartBuild({
  required Map<String, String> environmentDefines,
  required FlutterNativeAssetsBuildRunner buildRunner,
  required List<Architecture> architectures,
  required Uri projectUri,
  required BuildMode buildMode,
  required FileSystem fileSystem,
  required OS? targetOS,
}) async {
  final bool linkingEnabled = buildMode == BuildMode.release;

  final String architectureString =
      architectures.length == 1
          ? architectures.single.toString()
          : architectures.toList().toString();

  globals.logger.printTrace('Building native assets for $targetOS $architectureString $buildMode.');
  final List<EncodedAsset> assets = <EncodedAsset>[];
  final Set<Uri> dependencies = <Uri>{};

  final build_info.EnvironmentType? environmentType;
  if (targetOS == OS.iOS) {
    final String? sdkRoot = environmentDefines[build_info.kSdkRoot];
    if (sdkRoot == null) {
      throw MissingDefineException(build_info.kSdkRoot, 'native_assets');
    }
    environmentType = xcode.environmentTypeFromSdkroot(sdkRoot, fileSystem);
  } else {
    environmentType = null;
  }

  final CCompilerConfig cCompilerConfig =
      targetOS == OS.android
          ? await buildRunner.ndkCCompilerConfig
          : await buildRunner.cCompilerConfig;

  final String? codesignIdentity = environmentDefines[build_info.kCodesignIdentity];
  assert(codesignIdentity == null || targetOS == OS.iOS || targetOS == OS.macOS);

  final int? androidNdkApi =
      targetOS == OS.android ? targetAndroidNdkApi(environmentDefines) : null;
  final int? iOSVersion = targetOS == OS.iOS ? targetIOSVersion : null;
  final int? macOSVersion = targetOS == OS.macOS ? targetMacOSVersion : null;
  final IOSSdk? iOSSdk = targetOS == OS.iOS ? getIOSSdk(environmentType!) : null;

  for (final Architecture architecture in architectures) {
    final BuildResult? buildResult = await buildRunner.build(
      supportedAssetTypes: <String>[CodeAsset.type],
      configCreator:
          () =>
              BuildConfigBuilder()..setupCodeConfig(
                targetArchitecture: architecture,
                linkModePreference: LinkModePreference.dynamic,
                cCompilerConfig: cCompilerConfig,
                targetAndroidNdkApi: androidNdkApi,
                targetIOSVersion: iOSVersion,
                targetMacOSVersion: macOSVersion,
                targetIOSSdk: iOSSdk,
              ),
      configValidator:
          (BuildConfig config) async => <String>[...await validateCodeAssetBuildConfig(config)],
      buildValidator:
          (BuildConfig config, BuildOutput output) async => <String>[
            ...await validateCodeAssetBuildOutput(config, output),
          ],
      applicationAssetValidator:
          (List<EncodedAsset> assets) async => <String>[
            ...await validateCodeAssetInApplication(assets),
          ],
      targetOS: targetOS!,
      buildMode: buildMode,
      workingDirectory: projectUri,
      includeParentEnvironment: true,
      linkingEnabled: linkingEnabled,
    );
    if (buildResult == null) {
      _throwNativeAssetsBuildFailed();
    }
    dependencies.addAll(buildResult.dependencies);
    if (!linkingEnabled) {
      assets.addAll(buildResult.encodedAssets);
    } else {
      final LinkResult? linkResult = await buildRunner.link(
        supportedAssetTypes: <String>[CodeAsset.type],
        configCreator:
            () =>
                LinkConfigBuilder()..setupCodeConfig(
                  targetArchitecture: architecture,
                  linkModePreference: LinkModePreference.dynamic,
                  cCompilerConfig: cCompilerConfig,
                  targetAndroidNdkApi: androidNdkApi,
                  targetIOSVersion: iOSVersion,
                  targetMacOSVersion: macOSVersion,
                  targetIOSSdk: iOSSdk,
                ),
        configValidator:
            (LinkConfig config) async => <String>[...await validateCodeAssetLinkConfig(config)],
        linkValidator:
            (LinkConfig config, LinkOutput output) async => <String>[
              ...await validateCodeAssetLinkOutput(config, output),
            ],
        applicationAssetValidator:
            (List<EncodedAsset> assets) async => <String>[
              ...await validateCodeAssetInApplication(assets),
            ],
        workingDirectory: projectUri,
        includeParentEnvironment: true,
        buildResult: buildResult,
        targetOS: targetOS,
        buildMode: buildMode,
      );
      if (linkResult == null) {
        _throwNativeAssetsLinkFailed();
      }
      assets.addAll(linkResult.encodedAssets);
      dependencies.addAll(linkResult.dependencies);
    }
  }

  final List<CodeAsset> codeAssets =
      assets
          .where((EncodedAsset asset) => asset.type == CodeAsset.type)
          .map<CodeAsset>(CodeAsset.fromEncoded)
          .toList();
  globals.logger.printTrace(
    'Building native assets for $targetOS $architectureString $buildMode done.',
  );
  return DartBuildResult(codeAssets, dependencies.toList());
}

List<Architecture> _architecturesForOS(
  build_info.TargetPlatform targetPlatform,
  OS targetOS,
  Map<String, String> environmentDefines,
) {
  switch (targetOS) {
    case OS.linux:
      return <Architecture>[_getNativeArchitecture(targetPlatform)];
    case OS.windows:
      return <Architecture>[_getNativeArchitecture(targetPlatform)];
    case OS.macOS:
      final List<build_info.DarwinArch> darwinArchs =
          _emptyToNull(
            environmentDefines[build_info.kDarwinArchs],
          )?.split(' ').map(build_info.getDarwinArchForName).toList() ??
          <build_info.DarwinArch>[build_info.DarwinArch.x86_64, build_info.DarwinArch.arm64];
      return darwinArchs.map(getNativeMacOSArchitecture).toList();
    case OS.android:
      final String? androidArchsEnvironment = environmentDefines[build_info.kAndroidArchs];
      final List<build_info.AndroidArch> androidArchs = _androidArchs(
        targetPlatform,
        androidArchsEnvironment,
      );
      return androidArchs.map(getNativeAndroidArchitecture).toList();
    case OS.iOS:
      final List<build_info.DarwinArch> iosArchs =
          _emptyToNull(
            environmentDefines[build_info.kIosArchs],
          )?.split(' ').map(build_info.getIOSArchForName).toList() ??
          <build_info.DarwinArch>[build_info.DarwinArch.arm64];
      return iosArchs.map(getNativeIOSArchitecture).toList();
    default:
      // TODO(dacoharkes): Implement other OSes. https://github.com/flutter/flutter/issues/129757
      // Write the file we claim to have in the [outputs].
      return <Architecture>[];
  }
}

Architecture _getNativeArchitecture(build_info.TargetPlatform targetPlatform) {
  switch (targetPlatform) {
    case build_info.TargetPlatform.linux_x64:
    case build_info.TargetPlatform.windows_x64:
      return Architecture.x64;
    case build_info.TargetPlatform.linux_arm64:
    case build_info.TargetPlatform.windows_arm64:
      return Architecture.arm64;
    case build_info.TargetPlatform.android:
    case build_info.TargetPlatform.ios:
    case build_info.TargetPlatform.darwin:
    case build_info.TargetPlatform.fuchsia_arm64:
    case build_info.TargetPlatform.fuchsia_x64:
    case build_info.TargetPlatform.tester:
    case build_info.TargetPlatform.web_javascript:
    case build_info.TargetPlatform.android_arm:
    case build_info.TargetPlatform.android_arm64:
    case build_info.TargetPlatform.android_x64:
    case build_info.TargetPlatform.android_x86:
      throw Exception('Unknown targetPlatform: $targetPlatform.');
  }
}

Future<void> _copyNativeCodeAssetsToBundleOnWindowsLinux(
  Uri buildUri,
  Map<CodeAsset, KernelAsset> assetTargetLocations,
  build_info.BuildMode buildMode,
  FileSystem fileSystem,
) async {
  assert(assetTargetLocations.isNotEmpty);

  final Directory buildDir = fileSystem.directory(buildUri.toFilePath());
  if (!buildDir.existsSync()) {
    buildDir.createSync(recursive: true);
  }
  for (final MapEntry<CodeAsset, KernelAsset> assetMapping in assetTargetLocations.entries) {
    final Uri source = assetMapping.key.file!;
    final Uri target = (assetMapping.value.path as KernelAssetAbsolutePath).uri;
    final Uri targetUri = buildUri.resolveUri(target);
    final String targetFullPath = targetUri.toFilePath();
    await fileSystem.file(source).copy(targetFullPath);
  }
}

Never _throwNativeAssetsBuildFailed() {
  throwToolExit('Building native assets failed. See the logs for more details.');
}

Never _throwNativeAssetsLinkFailed() {
  throwToolExit('Linking native assets failed. See the logs for more details.');
}

OS getNativeOSFromTargetPlatform(build_info.TargetPlatform platform) {
  switch (platform) {
    case build_info.TargetPlatform.ios:
      return OS.iOS;
    case build_info.TargetPlatform.darwin:
      return OS.macOS;
    case build_info.TargetPlatform.linux_x64:
    case build_info.TargetPlatform.linux_arm64:
      return OS.linux;
    case build_info.TargetPlatform.windows_x64:
    case build_info.TargetPlatform.windows_arm64:
      return OS.windows;
    case build_info.TargetPlatform.fuchsia_arm64:
    case build_info.TargetPlatform.fuchsia_x64:
      return OS.fuchsia;
    case build_info.TargetPlatform.android:
    case build_info.TargetPlatform.android_arm:
    case build_info.TargetPlatform.android_arm64:
    case build_info.TargetPlatform.android_x64:
    case build_info.TargetPlatform.android_x86:
      return OS.android;
    case build_info.TargetPlatform.tester:
      if (const LocalPlatform().isMacOS) {
        return OS.macOS;
      } else if (const LocalPlatform().isLinux) {
        return OS.linux;
      } else if (const LocalPlatform().isWindows) {
        return OS.windows;
      } else {
        throw StateError('Unknown operating system');
      }
    case build_info.TargetPlatform.web_javascript:
      throw StateError('No dart builds for web yet.');
  }
}

List<build_info.AndroidArch> _androidArchs(
  build_info.TargetPlatform targetPlatform,
  String? androidArchsEnvironment,
) {
  switch (targetPlatform) {
    case build_info.TargetPlatform.android_arm:
      return <build_info.AndroidArch>[build_info.AndroidArch.armeabi_v7a];
    case build_info.TargetPlatform.android_arm64:
      return <build_info.AndroidArch>[build_info.AndroidArch.arm64_v8a];
    case build_info.TargetPlatform.android_x64:
      return <build_info.AndroidArch>[build_info.AndroidArch.x86_64];
    case build_info.TargetPlatform.android_x86:
      return <build_info.AndroidArch>[build_info.AndroidArch.x86];
    case build_info.TargetPlatform.android:
      if (androidArchsEnvironment == null) {
        throw MissingDefineException(build_info.kAndroidArchs, 'native_assets');
      }
      return androidArchsEnvironment.split(' ').map(build_info.getAndroidArchForName).toList();
    case build_info.TargetPlatform.darwin:
    case build_info.TargetPlatform.fuchsia_arm64:
    case build_info.TargetPlatform.fuchsia_x64:
    case build_info.TargetPlatform.ios:
    case build_info.TargetPlatform.linux_arm64:
    case build_info.TargetPlatform.linux_x64:
    case build_info.TargetPlatform.tester:
    case build_info.TargetPlatform.web_javascript:
    case build_info.TargetPlatform.windows_x64:
    case build_info.TargetPlatform.windows_arm64:
      throwToolExit('Unsupported Android target platform: $targetPlatform.');
  }
}

String? _emptyToNull(String? input) {
  if (input == null || input.isEmpty) {
    return null;
  }
  return input;
}

extension OSArchitectures on OS {
  Set<Architecture> get architectures => _osTargets[this]!;
}

const Map<OS, Set<Architecture>> _osTargets = <OS, Set<Architecture>>{
  OS.android: <Architecture>{
    Architecture.arm,
    Architecture.arm64,
    Architecture.ia32,
    Architecture.x64,
    Architecture.riscv64,
  },
  OS.fuchsia: <Architecture>{Architecture.arm64, Architecture.x64},
  OS.iOS: <Architecture>{Architecture.arm, Architecture.arm64, Architecture.x64},
  OS.linux: <Architecture>{
    Architecture.arm,
    Architecture.arm64,
    Architecture.ia32,
    Architecture.riscv32,
    Architecture.riscv64,
    Architecture.x64,
  },
  OS.macOS: <Architecture>{Architecture.arm64, Architecture.x64},
  OS.windows: <Architecture>{Architecture.arm64, Architecture.ia32, Architecture.x64},
};

build_info.BuildMode _getBuildMode(Map<String, String> environmentDefines, bool isFlutterTester) {
  if (isFlutterTester) {
    return build_info.BuildMode.debug;
  }
  final String? environmentBuildMode = environmentDefines[build_info.kBuildMode];
  if (environmentBuildMode == null) {
    throw MissingDefineException(build_info.kBuildMode, 'native_assets');
  }
  return build_info.BuildMode.fromCliName(environmentBuildMode);
}
