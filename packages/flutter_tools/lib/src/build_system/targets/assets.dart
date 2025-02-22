// Copyright 2014 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

import 'package:pool/pool.dart';

import '../../artifacts.dart';
import '../../asset.dart';
import '../../base/common.dart';
import '../../base/file_system.dart';
import '../../build_info.dart';
import '../../dart/package_map.dart';
import '../../devfs.dart';
import '../../flutter_manifest.dart';
import '../build_system.dart';
import '../depfile.dart';
import '../exceptions.dart';
import '../tools/asset_transformer.dart';
import '../tools/shader_compiler.dart';
import 'common.dart';
import 'icon_tree_shaker.dart';
import 'native_assets.dart';

/// A helper function to copy an asset bundle into an [environment]'s output
/// directory.
///
/// Throws [Exception] if [AssetBundle.build] returns a non-zero exit code.
///
/// [additionalContent] may contain additional DevFS entries that will be
/// included in the final bundle, but not the AssetManifest.json file.
///
/// Returns a [Depfile] containing all assets used in the build.
Future<Depfile> copyAssets(
  Environment environment,
  Directory outputDirectory, {
  Map<String, DevFSContent> additionalContent = const <String, DevFSContent>{},
  required TargetPlatform targetPlatform,
  required BuildMode buildMode,
  List<File> additionalInputs = const <File>[],
  String? flavor,
}) async {
  final File pubspecFile = environment.projectDir.childFile('pubspec.yaml');
  // Only the default asset bundle style is supported in assemble.
  final AssetBundle assetBundle =
      AssetBundleFactory.defaultInstance(
        logger: environment.logger,
        fileSystem: environment.fileSystem,
        platform: environment.platform,
        splitDeferredAssets: buildMode != BuildMode.debug && buildMode != BuildMode.jitRelease,
      ).createBundle();
  final int resultCode = await assetBundle.build(
    manifestPath: pubspecFile.path,
    packageConfigPath: findPackageConfigFileOrDefault(environment.projectDir).path,
    deferredComponentsEnabled: environment.defines[kDeferredComponents] == 'true',
    targetPlatform: targetPlatform,
    flavor: flavor,
  );
  if (resultCode != 0) {
    throw Exception('Failed to bundle asset files.');
  }
  final Pool pool = Pool(kMaxOpenFiles);
  final List<File> inputs = <File>[
    // An asset manifest with no assets would have zero inputs if not
    // for this pubspec file.
    pubspecFile,
    ...additionalInputs,
  ];
  final List<File> outputs = <File>[];

  final IconTreeShaker iconTreeShaker = IconTreeShaker(
    environment,
    assetBundle.entries[kFontManifestJson]?.content as DevFSStringContent?,
    processManager: environment.processManager,
    logger: environment.logger,
    fileSystem: environment.fileSystem,
    artifacts: environment.artifacts,
    targetPlatform: targetPlatform,
  );
  final ShaderCompiler shaderCompiler = ShaderCompiler(
    processManager: environment.processManager,
    logger: environment.logger,
    fileSystem: environment.fileSystem,
    artifacts: environment.artifacts,
  );
  final AssetTransformer assetTransformer = AssetTransformer(
    processManager: environment.processManager,
    fileSystem: environment.fileSystem,
    dartBinaryPath: environment.artifacts.getArtifactPath(Artifact.engineDartBinary),
    buildMode: buildMode,
  );

  final Map<String, AssetBundleEntry> assetEntries = <String, AssetBundleEntry>{
    ...assetBundle.entries,
    ...additionalContent.map((String key, DevFSContent value) {
      return MapEntry<String, AssetBundleEntry>(
        key,
        AssetBundleEntry(
          value,
          kind: AssetKind.regular,
          transformers: const <AssetTransformerEntry>[],
        ),
      );
    }),
  };

  await Future.wait<void>(
    assetEntries.entries.map<Future<void>>((MapEntry<String, AssetBundleEntry> entry) async {
      final PoolResource resource = await pool.request();
      try {
        // This will result in strange looking files, for example files with `/`
        // on Windows or files that end up getting URI encoded such as `#.ext`
        // to `%23.ext`. However, we have to keep it this way since the
        // platform channels in the framework will URI encode these values,
        // and the native APIs will look for files this way.
        final File file = environment.fileSystem.file(
          environment.fileSystem.path.join(outputDirectory.path, entry.key),
        );
        outputs.add(file);
        file.parent.createSync(recursive: true);
        final DevFSContent content = entry.value.content;
        if (content is DevFSFileContent && content.file is File) {
          inputs.add(content.file as File);
          bool doCopy = true;
          switch (entry.value.kind) {
            case AssetKind.regular:
              if (entry.value.transformers.isNotEmpty) {
                final AssetTransformationFailure? failure = await assetTransformer.transformAsset(
                  asset: content.file as File,
                  outputPath: file.path,
                  workingDirectory: environment.projectDir.path,
                  transformerEntries: entry.value.transformers,
                  logger: environment.logger,
                );
                doCopy = false;
                if (failure != null) {
                  throwToolExit(
                    'User-defined transformation of asset "${entry.key}" failed.\n'
                    '${failure.message}',
                  );
                }
              }
            case AssetKind.font:
              doCopy =
                  !await iconTreeShaker.subsetFont(
                    input: content.file as File,
                    outputPath: file.path,
                    relativePath: entry.key,
                  );
            case AssetKind.shader:
              doCopy =
                  !await shaderCompiler.compileShader(
                    input: content.file as File,
                    outputPath: file.path,
                    targetPlatform: targetPlatform,
                  );
          }
          if (doCopy) {
            await (content.file as File).copy(file.path);
          }
        } else {
          await file.writeAsBytes(await entry.value.content.contentsAsBytes());
        }
      } finally {
        resource.release();
      }
    }),
  );

  // Copy deferred components assets only for release or profile builds.
  // The assets are included in assetBundle.entries as a normal asset when
  // building as debug.
  if (environment.defines[kDeferredComponents] == 'true') {
    await Future.wait<void>(
      assetBundle.deferredComponentsEntries.entries.map<Future<void>>((
        MapEntry<String, Map<String, AssetBundleEntry>> componentEntries,
      ) async {
        final Directory componentOutputDir = environment.projectDir
            .childDirectory('build')
            .childDirectory(componentEntries.key)
            .childDirectory('intermediates')
            .childDirectory('flutter');
        await Future.wait<void>(
          componentEntries.value.entries.map<Future<void>>((
            MapEntry<String, AssetBundleEntry> entry,
          ) async {
            final PoolResource resource = await pool.request();
            try {
              // This will result in strange looking files, for example files with `/`
              // on Windows or files that end up getting URI encoded such as `#.ext`
              // to `%23.ext`. However, we have to keep it this way since the
              // platform channels in the framework will URI encode these values,
              // and the native APIs will look for files this way.

              // If deferred components are disabled, then copy assets to regular location.
              final File file =
                  environment.defines[kDeferredComponents] == 'true'
                      ? environment.fileSystem.file(
                        environment.fileSystem.path.join(
                          componentOutputDir.path,
                          buildMode.cliName,
                          'deferred_assets',
                          'flutter_assets',
                          entry.key,
                        ),
                      )
                      : environment.fileSystem.file(
                        environment.fileSystem.path.join(outputDirectory.path, entry.key),
                      );
              outputs.add(file);
              file.parent.createSync(recursive: true);
              final DevFSContent content = entry.value.content;
              if (content is DevFSFileContent && content.file is File) {
                inputs.add(content.file as File);
                if (!await iconTreeShaker.subsetFont(
                  input: content.file as File,
                  outputPath: file.path,
                  relativePath: entry.key,
                )) {
                  await (content.file as File).copy(file.path);
                }
              } else {
                await file.writeAsBytes(await entry.value.contentsAsBytes());
              }
            } finally {
              resource.release();
            }
          }),
        );
      }),
    );
  }
  final Depfile depfile = Depfile(inputs + assetBundle.additionalDependencies, outputs);
  return depfile;
}

/// Copy the assets defined in the flutter manifest into a build directory.
class CopyAssets extends Target {
  const CopyAssets();

  @override
  String get name => 'copy_assets';

  @override
  List<Target> get dependencies => const <Target>[KernelSnapshot(), InstallCodeAssets()];

  @override
  List<Source> get inputs => const <Source>[
    Source.pattern(
      '{FLUTTER_ROOT}/packages/flutter_tools/lib/src/build_system/targets/assets.dart',
    ),
    ...IconTreeShaker.inputs,
    ...ShaderCompiler.inputs,
  ];

  @override
  List<Source> get outputs => const <Source>[];

  @override
  List<String> get depfiles => const <String>['flutter_assets.d'];

  @override
  Future<void> build(Environment environment) async {
    final String? buildModeEnvironment = environment.defines[kBuildMode];
    if (buildModeEnvironment == null) {
      throw MissingDefineException(kBuildMode, name);
    }
    final BuildMode buildMode = BuildMode.fromCliName(buildModeEnvironment);
    final Directory output = environment.buildDir.childDirectory('flutter_assets');
    output.createSync(recursive: true);
    final Depfile depfile = await copyAssets(
      environment,
      output,
      targetPlatform: TargetPlatform.android,
      buildMode: buildMode,
      flavor: environment.defines[kFlavor],
      additionalContent: <String, DevFSContent>{
        'NativeAssetsManifest.json': DevFSFileContent(
          environment.buildDir.childFile('native_assets.json'),
        ),
      },
    );
    environment.depFileService.writeToFile(
      depfile,
      environment.buildDir.childFile('flutter_assets.d'),
    );
  }
}
