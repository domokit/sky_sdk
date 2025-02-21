// Copyright 2014 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

import 'package:meta/meta.dart';
import 'package:pool/pool.dart';
import 'package:process/process.dart';

import 'artifacts.dart';
import 'asset.dart' hide defaultManifestPath;
import 'base/common.dart';
import 'base/file_system.dart';
import 'base/logger.dart';
import 'build_info.dart';
import 'build_system/build_system.dart';
import 'build_system/depfile.dart';
import 'build_system/tools/asset_transformer.dart';
import 'build_system/tools/shader_compiler.dart';
import 'bundle.dart';
import 'cache.dart';
import 'devfs.dart';
import 'device.dart';
import 'globals.dart' as globals;
import 'project.dart';

/// Provides a `build` method that builds the bundle.
class BundleBuilder {
  /// Builds the bundle for the given target platform.
  ///
  /// The default `mainPath` is `lib/main.dart`.
  /// The default `manifestPath` is `pubspec.yaml`.
  ///
  /// If [buildNativeAssets], native assets are built and the mapping for native
  /// assets lookup at runtime is embedded in the kernel file, otherwise an
  /// empty native assets mapping is embedded in the kernel file.
  Future<void> build({
    required TargetPlatform platform,
    required BuildInfo buildInfo,
    FlutterProject? project,
    String? mainPath,
    String manifestPath = defaultManifestPath,
    String? applicationKernelFilePath,
    String? depfilePath,
    String? assetDirPath,
    bool buildNativeAssets = true,
    @visibleForTesting BuildSystem? buildSystem,
  }) async {
    project ??= FlutterProject.current();
    mainPath ??= defaultMainPath;
    depfilePath ??= defaultDepfilePath;
    assetDirPath ??= getAssetBuildDirectory();
    buildSystem ??= globals.buildSystem;

    // If the precompiled flag was not passed, force us into debug mode.
    final Environment environment = Environment(
      projectDir: project.directory,
      packageConfigPath: buildInfo.packageConfigPath,
      outputDir: globals.fs.directory(assetDirPath),
      buildDir: project.dartTool.childDirectory('flutter_build'),
      cacheDir: globals.cache.getRoot(),
      flutterRootDir: globals.fs.directory(Cache.flutterRoot),
      engineVersion:
          globals.artifacts!.usesLocalArtifacts ? null : globals.flutterVersion.engineRevision,
      defines: <String, String>{
        // used by the KernelSnapshot target
        kTargetPlatform: getNameForTargetPlatform(platform),
        kTargetFile: mainPath,
        kDeferredComponents: 'false',
        ...buildInfo.toBuildSystemEnvironment(),
        if (!buildNativeAssets) kNativeAssets: 'false',
      },
      artifacts: globals.artifacts!,
      fileSystem: globals.fs,
      logger: globals.logger,
      processManager: globals.processManager,
      analytics: globals.analytics,
      platform: globals.platform,
      generateDartPluginRegistry: true,
    );
    final Target target =
        buildInfo.mode == BuildMode.debug
            ? globals.buildTargets.copyFlutterBundle
            : globals.buildTargets.releaseCopyFlutterBundle;
    final BuildResult result = await buildSystem.build(target, environment);

    if (!result.success) {
      for (final ExceptionMeasurement measurement in result.exceptions.values) {
        globals.printError(
          'Target ${measurement.target} failed: ${measurement.exception}',
          stackTrace: measurement.fatal ? measurement.stackTrace : null,
        );
      }
      throwToolExit('Failed to build bundle.');
    }
    final Depfile depfile = Depfile(result.inputFiles, result.outputFiles);
    final File outputDepfile = globals.fs.file(depfilePath);
    if (!outputDepfile.parent.existsSync()) {
      outputDepfile.parent.createSync(recursive: true);
    }
    environment.depFileService.writeToFile(depfile, outputDepfile);

    // Work around for flutter_tester placing kernel artifacts in odd places.
    if (applicationKernelFilePath != null) {
      final File outputDill = globals.fs.directory(assetDirPath).childFile('kernel_blob.bin');
      if (outputDill.existsSync()) {
        outputDill.copySync(applicationKernelFilePath);
      }
    }
    return;
  }
}

Future<AssetBundle?> buildAssets({
  required String manifestPath,
  String? assetDirPath,
  required String packageConfigPath,
  TargetPlatform? targetPlatform,
  String? flavor,
}) async {
  assetDirPath ??= getAssetBuildDirectory();

  // Build the asset bundle.
  final AssetBundle assetBundle = AssetBundleFactory.instance.createBundle();
  final int result = await assetBundle.build(
    manifestPath: manifestPath,
    packageConfigPath: packageConfigPath,
    targetPlatform: targetPlatform,
    flavor: flavor,
  );
  if (result != 0) {
    return null;
  }

  return assetBundle;
}

Future<void> writeBundle(
  Directory bundleDir,
  Map<String, AssetBundleEntry> assetEntries, {
  required TargetPlatform targetPlatform,
  required ImpellerStatus impellerStatus,
  required ProcessManager processManager,
  required FileSystem fileSystem,
  required Artifacts artifacts,
  required Logger logger,
  required Directory projectDir,
  required BuildMode buildMode,
}) async {
  if (bundleDir.existsSync()) {
    try {
      bundleDir.deleteSync(recursive: true);
    } on FileSystemException catch (err) {
      logger.printWarning(
        'Failed to clean up asset directory ${bundleDir.path}: $err\n'
        'To clean build artifacts, use the command "flutter clean".',
      );
    }
  }
  bundleDir.createSync(recursive: true);

  final ShaderCompiler shaderCompiler = ShaderCompiler(
    processManager: processManager,
    logger: logger,
    fileSystem: fileSystem,
    artifacts: artifacts,
  );

  final AssetTransformer assetTransformer = AssetTransformer(
    processManager: processManager,
    fileSystem: fileSystem,
    dartBinaryPath: artifacts.getArtifactPath(Artifact.engineDartBinary),
    buildMode: buildMode,
  );

  // Limit number of open files to avoid running out of file descriptors.
  final Pool pool = Pool(64);
  await Future.wait<void>(
    assetEntries.entries.map<Future<void>>((MapEntry<String, AssetBundleEntry> entry) async {
      final PoolResource resource = await pool.request();
      try {
        // This will result in strange looking files, for example files with `/`
        // on Windows or files that end up getting URI encoded such as `#.ext`
        // to `%23.ext`. However, we have to keep it this way since the
        // platform channels in the framework will URI encode these values,
        // and the native APIs will look for files this way.
        final File file = fileSystem.file(fileSystem.path.join(bundleDir.path, entry.key));
        file.parent.createSync(recursive: true);
        final DevFSContent devFSContent = entry.value.content;
        if (devFSContent is DevFSFileContent) {
          final File input = devFSContent.file as File;
          bool doCopy = true;
          switch (entry.value.kind) {
            case AssetKind.regular:
              if (entry.value.transformers.isEmpty) {
                break;
              }
              final AssetTransformationFailure? failure = await assetTransformer.transformAsset(
                asset: input,
                outputPath: file.path,
                workingDirectory: projectDir.path,
                transformerEntries: entry.value.transformers,
                logger: logger,
              );
              doCopy = false;
              if (failure != null) {
                throwToolExit(
                  'User-defined transformation of asset "${entry.key}" failed.\n'
                  '${failure.message}',
                );
              }
            case AssetKind.font:
              break;
            case AssetKind.shader:
              doCopy =
                  !await shaderCompiler.compileShader(
                    input: input,
                    outputPath: file.path,
                    targetPlatform: targetPlatform,
                  );
          }
          if (doCopy) {
            input.copySync(file.path);
          }
        } else {
          await file.writeAsBytes(await entry.value.contentsAsBytes());
        }
      } finally {
        resource.release();
      }
    }),
  );
}
