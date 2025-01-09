// Copyright 2014 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

import 'package:meta/meta.dart';
import 'package:package_config/package_config_types.dart';

import '../../base/common.dart';
import '../../base/file_system.dart';
import '../../build_info.dart';
import '../../convert.dart';
import '../../dart/package_map.dart';
import '../../isolated/native_assets/native_assets.dart';
import '../build_system.dart';
import '../depfile.dart';
import '../exceptions.dart';
import 'common.dart';

/// Runs the dart build of the app.
abstract class DartBuild extends Target {
  const DartBuild({@visibleForTesting FlutterNativeAssetsBuildRunner? buildRunner})
    : _buildRunner = buildRunner;

  final FlutterNativeAssetsBuildRunner? _buildRunner;

  @override
  Future<void> build(Environment environment) async {
    final FileSystem fileSystem = environment.fileSystem;
    final String? nativeAssetsEnvironment = environment.defines[kNativeAssets];

    final DartBuildResult result;
    if (nativeAssetsEnvironment == 'false') {
      result = const DartBuildResult.empty();
    } else {
      final TargetPlatform targetPlatform = _getTargetPlatformFromEnvironment(environment, name);

      final PackageConfig packageConfig = await loadPackageConfigWithLogging(
        fileSystem.file(environment.packageConfigPath),
        logger: environment.logger,
      );
      final Uri projectUri = environment.projectDir.uri;
      final FlutterNativeAssetsBuildRunner buildRunner =
          _buildRunner ??
          FlutterNativeAssetsBuildRunnerImpl(
            projectUri,
            environment.packageConfigPath,
            packageConfig,
            fileSystem,
            environment.logger,
          );
      result = await runFlutterSpecificDartBuild(
        environmentDefines: environment.defines,
        buildRunner: buildRunner,
        targetPlatform: targetPlatform,
        projectUri: projectUri,
        fileSystem: fileSystem,
      );
    }

    final File dartBuildResultJsonFile = environment.buildDir.childFile(dartBuildResultFilename);
    if (!dartBuildResultJsonFile.parent.existsSync()) {
      dartBuildResultJsonFile.parent.createSync(recursive: true);
    }
    dartBuildResultJsonFile.writeAsStringSync(json.encode(result.toJson()));

    final Depfile depfile = Depfile(
      <File>[for (final Uri dependency in result.dependencies) fileSystem.file(dependency)],
      <File>[fileSystem.file(dartBuildResultJsonFile)],
    );
    final File outputDepfile = environment.buildDir.childFile(depFilename);
    if (!outputDepfile.parent.existsSync()) {
      outputDepfile.parent.createSync(recursive: true);
    }
    environment.depFileService.writeToFile(depfile, outputDepfile);
    if (!await outputDepfile.exists()) {
      throwToolExit("${outputDepfile.path} doesn't exist.");
    }
  }

  @override
  List<String> get depfiles => const <String>[depFilename];

  @override
  List<Source> get inputs => const <Source>[
    Source.pattern(
      '{FLUTTER_ROOT}/packages/flutter_tools/lib/src/build_system/targets/native_assets.dart',
    ),
    // If different packages are resolved, different native assets might need to be built.
    Source.pattern('{WORKSPACE_DIR}/.dart_tool/package_config_subset'),
    // TODO(mosuem): Should consume resources.json. https://github.com/flutter/flutter/issues/146263
  ];

  @override
  String get name => 'dart_build';

  @override
  List<Source> get outputs => const <Source>[
    Source.pattern('{BUILD_DIR}/$dartBuildResultFilename'),
  ];

  /// Dependent build [Target]s can use this to consume the result of the
  /// [DartBuild] target.
  static Future<DartBuildResult> loadBuildResult(Environment environment) async {
    final File dartBuildResultJsonFile = environment.buildDir.childFile(
      DartBuild.dartBuildResultFilename,
    );
    return DartBuildResult.fromJson(
      json.decode(dartBuildResultJsonFile.readAsStringSync()) as Map<String, Object?>,
    );
  }

  static const String dartBuildResultFilename = 'dart_build_result.json';
  static const String depFilename = 'dart_build.d';
}

class DartBuildForNative extends DartBuild {
  const DartBuildForNative({@visibleForTesting super.buildRunner});

  @override
  List<Target> get dependencies => const <Target>[KernelSnapshot()];
}

/// Installs the code assets from a [DartBuild] Flutter app.
///
/// The build mode and target architecture can be changed from the
/// native build project (Xcode etc.), so only `flutter assemble` has the
/// information about build-mode and target architecture.
class InstallCodeAssets extends Target {
  const InstallCodeAssets();

  @override
  Future<void> build(Environment environment) async {
    final Uri projectUri = environment.projectDir.uri;
    final FileSystem fileSystem = environment.fileSystem;
    final TargetPlatform targetPlatform = _getTargetPlatformFromEnvironment(environment, name);

    // We fetch the result from the [DartBuild].
    final DartBuildResult dartBuildResult = await DartBuild.loadBuildResult(environment);

    // And install/copy the code assets to the right place and create a
    // native_asset.yaml that can be used by the final AOT compilation.
    final Uri nativeAssetsFileUri = environment.buildDir.childFile(nativeAssetsFilename).uri;
    await installCodeAssets(
      dartBuildResult: dartBuildResult,
      environmentDefines: environment.defines,
      targetPlatform: targetPlatform,
      projectUri: projectUri,
      fileSystem: fileSystem,
      nativeAssetsFileUri: nativeAssetsFileUri,
    );
    assert(await fileSystem.file(nativeAssetsFileUri).exists());

    final Depfile depfile = Depfile(
      <File>[for (final Uri file in dartBuildResult.filesToBeBundled) fileSystem.file(file)],
      <File>[fileSystem.file(nativeAssetsFileUri)],
    );
    final File outputDepfile = environment.buildDir.childFile(depFilename);
    environment.depFileService.writeToFile(depfile, outputDepfile);
    if (!await outputDepfile.exists()) {
      throwToolExit("${outputDepfile.path} doesn't exist.");
    }
  }

  @override
  List<String> get depfiles => <String>[depFilename];

  @override
  List<Target> get dependencies => const <Target>[DartBuildForNative()];

  @override
  List<Source> get inputs => const <Source>[
    Source.pattern(
      '{FLUTTER_ROOT}/packages/flutter_tools/lib/src/build_system/targets/native_assets.dart',
    ),
    // If different packages are resolved, different native assets might need to be built.
    Source.pattern('{WORKSPACE_DIR}/.dart_tool/package_config_subset'),
  ];

  @override
  String get name => 'install_code_assets';

  @override
  List<Source> get outputs => const <Source>[Source.pattern('{BUILD_DIR}/$nativeAssetsFilename')];

  static const String nativeAssetsFilename = 'native_assets.json';
  static const String depFilename = 'install_code_assets.d';
}

TargetPlatform _getTargetPlatformFromEnvironment(Environment environment, String name) {
  final String? targetPlatformEnvironment = environment.defines[kTargetPlatform];
  if (targetPlatformEnvironment == null) {
    throw MissingDefineException(kTargetPlatform, name);
  }
  return getTargetPlatformForName(targetPlatformEnvironment);
}
