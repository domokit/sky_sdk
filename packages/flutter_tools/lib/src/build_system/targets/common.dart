// Copyright 2014 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

import 'package:package_config/package_config.dart';

import '../../artifacts.dart';
import '../../base/build.dart';
import '../../base/file_system.dart';
import '../../base/io.dart';
import '../../build_info.dart';
import '../../compile.dart';
import '../../dart/package_map.dart';
import '../../devfs.dart';
import '../../globals.dart' as globals show xcode;
import '../build_system.dart';
import '../depfile.dart';
import '../exceptions.dart';
import '../tools/shader_compiler.dart';
import 'assets.dart';
import 'dart_plugin_registrant.dart';
import 'icon_tree_shaker.dart';
import 'localizations.dart';
import 'native_assets.dart';

/// Copies the pre-built flutter bundle.
// This is a one-off rule for implementing build bundle in terms of assemble.
class CopyFlutterBundle extends Target {
  const CopyFlutterBundle();

  @override
  String get name => 'copy_flutter_bundle';

  @override
  List<Source> get inputs => const <Source>[
    Source.artifact(Artifact.vmSnapshotData, mode: BuildMode.debug),
    Source.artifact(Artifact.isolateSnapshotData, mode: BuildMode.debug),
    Source.pattern('{BUILD_DIR}/app.dill'),
    ...IconTreeShaker.inputs,
    ...ShaderCompiler.inputs,
  ];

  @override
  List<Source> get outputs => const <Source>[
    Source.pattern('{OUTPUT_DIR}/vm_snapshot_data'),
    Source.pattern('{OUTPUT_DIR}/isolate_snapshot_data'),
    Source.pattern('{OUTPUT_DIR}/kernel_blob.bin'),
  ];

  @override
  List<String> get depfiles => <String>['flutter_assets.d'];

  @override
  Future<void> build(Environment environment) async {
    final String? buildModeEnvironment = environment.defines[kBuildMode];
    if (buildModeEnvironment == null) {
      throw MissingDefineException(kBuildMode, 'copy_flutter_bundle');
    }
    final String? flavor = environment.defines[kFlavor];

    final BuildMode buildMode = BuildMode.fromCliName(buildModeEnvironment);
    environment.outputDir.createSync(recursive: true);

    // Only copy the prebuilt runtimes and kernel blob in debug mode.
    if (buildMode == BuildMode.debug) {
      final String vmSnapshotData = environment.artifacts.getArtifactPath(
        Artifact.vmSnapshotData,
        mode: BuildMode.debug,
      );
      final String isolateSnapshotData = environment.artifacts.getArtifactPath(
        Artifact.isolateSnapshotData,
        mode: BuildMode.debug,
      );
      environment.buildDir
          .childFile('app.dill')
          .copySync(environment.outputDir.childFile('kernel_blob.bin').path);
      environment.fileSystem
          .file(vmSnapshotData)
          .copySync(environment.outputDir.childFile('vm_snapshot_data').path);
      environment.fileSystem
          .file(isolateSnapshotData)
          .copySync(environment.outputDir.childFile('isolate_snapshot_data').path);
    }
    final Depfile assetDepfile = await copyAssets(
      environment,
      environment.outputDir,
      targetPlatform: TargetPlatform.android,
      buildMode: buildMode,
      flavor: flavor,
      additionalContent: <String, DevFSContent>{
        'NativeAssetsManifest.json': DevFSFileContent(
          environment.buildDir.childFile('native_assets.json'),
        ),
      },
    );
    environment.depFileService.writeToFile(
      assetDepfile,
      environment.buildDir.childFile('flutter_assets.d'),
    );
  }

  @override
  List<Target> get dependencies => const <Target>[KernelSnapshot(), InstallCodeAssets()];
}

/// Copies the pre-built flutter bundle for release mode.
class ReleaseCopyFlutterBundle extends CopyFlutterBundle {
  const ReleaseCopyFlutterBundle();

  @override
  String get name => 'release_flutter_bundle';

  @override
  List<Source> get inputs => const <Source>[];

  @override
  List<Source> get outputs => const <Source>[];

  @override
  List<String> get depfiles => const <String>['flutter_assets.d'];

  @override
  List<Target> get dependencies => const <Target>[];
}

/// Generate a snapshot of the dart code used in the program.
///
/// This target depends on the `.dart_tool/package_config.json` file
/// even though it is not listed as an input. Pub inserts a timestamp into
/// the file which causes unnecessary rebuilds, so instead a subset of the contents
/// are used an input instead.
class KernelSnapshot extends Target {
  const KernelSnapshot();

  @override
  String get name => 'kernel_snapshot_program';

  @override
  List<Source> get inputs => const <Source>[
    Source.pattern('{WORKSPACE_DIR}/.dart_tool/package_config_subset'),
    Source.pattern(
      '{FLUTTER_ROOT}/packages/flutter_tools/lib/src/build_system/targets/common.dart',
    ),
    Source.artifact(Artifact.platformKernelDill),
    Source.artifact(Artifact.engineDartBinary),
    Source.artifact(Artifact.engineDartAotRuntime),
    Source.artifact(Artifact.frontendServerSnapshotForEngineDartSdk),
  ];

  @override
  List<Source> get outputs => const <Source>[
    Source.pattern('{BUILD_DIR}/${KernelSnapshot.dillName}'),
    // TODO(mosuem): Should output resources.json. https://github.com/flutter/flutter/issues/146263
  ];

  static const String depfile = 'kernel_snapshot_program.d';

  @override
  List<String> get depfiles => const <String>[depfile];

  @override
  List<Target> get dependencies => const <Target>[
    GenerateLocalizationsTarget(),
    DartPluginRegistrantTarget(),
  ];

  static const String dillName = 'app.dill';

  @override
  Future<void> build(Environment environment) async {
    final KernelCompiler compiler = KernelCompiler(
      fileSystem: environment.fileSystem,
      logger: environment.logger,
      processManager: environment.processManager,
      artifacts: environment.artifacts,
      fileSystemRoots: <String>[],
    );
    final String? buildModeEnvironment = environment.defines[kBuildMode];
    if (buildModeEnvironment == null) {
      throw MissingDefineException(kBuildMode, 'kernel_snapshot');
    }
    final String? targetPlatformEnvironment = environment.defines[kTargetPlatform];
    if (targetPlatformEnvironment == null) {
      throw MissingDefineException(kTargetPlatform, 'kernel_snapshot');
    }
    final BuildMode buildMode = BuildMode.fromCliName(buildModeEnvironment);
    final String targetFile =
        environment.defines[kTargetFile] ?? environment.fileSystem.path.join('lib', 'main.dart');
    final File packagesFile = findPackageConfigFileOrDefault(environment.projectDir);
    final String targetFileAbsolute = environment.fileSystem.file(targetFile).absolute.path;
    // everything besides 'false' is considered to be enabled.
    final bool trackWidgetCreation = environment.defines[kTrackWidgetCreation] != 'false';
    final TargetPlatform targetPlatform = getTargetPlatformForName(targetPlatformEnvironment);

    // This configuration is all optional.
    final String? frontendServerStarterPath = environment.defines[kFrontendServerStarterPath];
    final List<String> extraFrontEndOptions = decodeCommaSeparated(
      environment.defines,
      kExtraFrontEndOptions,
    );
    final List<String>? fileSystemRoots = environment.defines[kFileSystemRoots]?.split(',');
    final String? fileSystemScheme = environment.defines[kFileSystemScheme];

    TargetModel targetModel = TargetModel.flutter;
    if (targetPlatform == TargetPlatform.fuchsia_x64 ||
        targetPlatform == TargetPlatform.fuchsia_arm64) {
      targetModel = TargetModel.flutterRunner;
    }
    // Force linking of the platform for desktop embedder targets since these
    // do not correctly load the core snapshots in debug mode.
    // See https://github.com/flutter/flutter/issues/44724
    final bool forceLinkPlatform;
    switch (targetPlatform) {
      case TargetPlatform.darwin:
      case TargetPlatform.windows_x64:
      case TargetPlatform.windows_arm64:
      case TargetPlatform.linux_x64:
        forceLinkPlatform = true;
      case TargetPlatform.android:
      case TargetPlatform.android_arm:
      case TargetPlatform.android_arm64:
      case TargetPlatform.android_x64:
      case TargetPlatform.android_x86:
      case TargetPlatform.fuchsia_arm64:
      case TargetPlatform.fuchsia_x64:
      case TargetPlatform.ios:
      case TargetPlatform.linux_arm64:
      case TargetPlatform.tester:
      case TargetPlatform.web_javascript:
        forceLinkPlatform = false;
    }

    final String? targetOS = switch (targetPlatform) {
      TargetPlatform.fuchsia_arm64 || TargetPlatform.fuchsia_x64 => 'fuchsia',
      TargetPlatform.android ||
      TargetPlatform.android_arm ||
      TargetPlatform.android_arm64 ||
      TargetPlatform.android_x64 ||
      TargetPlatform.android_x86 =>
        'android',
      TargetPlatform.darwin => 'macos',
      TargetPlatform.ios => 'ios',
      TargetPlatform.linux_arm64 || TargetPlatform.linux_x64 => 'linux',
      TargetPlatform.windows_arm64 || TargetPlatform.windows_x64 => 'windows',
      TargetPlatform.tester || TargetPlatform.web_javascript => null,
    };

    final PackageConfig packageConfig = await loadPackageConfigWithLogging(
      packagesFile,
      logger: environment.logger,
    );

    final String dillPath = environment.buildDir.childFile(dillName).path;

    final CompilerOutput? output = await compiler.compile(
      sdkRoot: environment.artifacts.getArtifactPath(
        Artifact.flutterPatchedSdkPath,
        platform: targetPlatform,
        mode: buildMode,
      ),
      aot: buildMode.isPrecompiled,
      buildMode: buildMode,
      trackWidgetCreation: trackWidgetCreation && buildMode != BuildMode.release,
      targetModel: targetModel,
      outputFilePath: dillPath,
      initializeFromDill: buildMode.isPrecompiled ? null : dillPath,
      packagesPath: packagesFile.path,
      linkPlatformKernelIn: forceLinkPlatform || buildMode.isPrecompiled,
      mainPath: targetFileAbsolute,
      depFilePath: environment.buildDir.childFile(depfile).path,
      frontendServerStarterPath: frontendServerStarterPath,
      extraFrontEndOptions: extraFrontEndOptions,
      fileSystemRoots: fileSystemRoots,
      fileSystemScheme: fileSystemScheme,
      dartDefines: decodeDartDefines(environment.defines, kDartDefines),
      packageConfig: packageConfig,
      buildDir: environment.buildDir,
      targetOS: targetOS,
      checkDartPluginRegistry: environment.generateDartPluginRegistry,
    );
    if (output == null || output.errorCount != 0) {
      throw Exception();
    }
  }
}

/// Supports compiling a dart kernel file to an ELF binary.
abstract class AotElfBase extends Target {
  const AotElfBase();

  @override
  String get analyticsName => 'android_aot';

  @override
  Future<void> build(Environment environment) async {
    final AOTSnapshotter snapshotter = AOTSnapshotter(
      fileSystem: environment.fileSystem,
      logger: environment.logger,
      xcode: globals.xcode!,
      processManager: environment.processManager,
      artifacts: environment.artifacts,
    );
    final String outputPath = environment.buildDir.path;
    final String? buildModeEnvironment = environment.defines[kBuildMode];
    if (buildModeEnvironment == null) {
      throw MissingDefineException(kBuildMode, 'aot_elf');
    }
    final String? targetPlatformEnvironment = environment.defines[kTargetPlatform];
    if (targetPlatformEnvironment == null) {
      throw MissingDefineException(kTargetPlatform, 'aot_elf');
    }
    final List<String> extraGenSnapshotOptions = decodeCommaSeparated(
      environment.defines,
      kExtraGenSnapshotOptions,
    );
    final BuildMode buildMode = BuildMode.fromCliName(buildModeEnvironment);
    final TargetPlatform targetPlatform = getTargetPlatformForName(targetPlatformEnvironment);
    final String? splitDebugInfo = environment.defines[kSplitDebugInfo];
    final bool dartObfuscation = environment.defines[kDartObfuscation] == 'true';
    final String? codeSizeDirectory = environment.defines[kCodeSizeDirectory];

    if (codeSizeDirectory != null) {
      final File codeSizeFile = environment.fileSystem
          .directory(codeSizeDirectory)
          .childFile('snapshot.${environment.defines[kTargetPlatform]}.json');
      final File precompilerTraceFile = environment.fileSystem
          .directory(codeSizeDirectory)
          .childFile('trace.${environment.defines[kTargetPlatform]}.json');
      extraGenSnapshotOptions.add('--write-v8-snapshot-profile-to=${codeSizeFile.path}');
      extraGenSnapshotOptions.add('--trace-precompiler-to=${precompilerTraceFile.path}');
    }

    final int snapshotExitCode = await snapshotter.build(
      platform: targetPlatform,
      buildMode: buildMode,
      mainPath: environment.buildDir.childFile('app.dill').path,
      outputPath: outputPath,
      extraGenSnapshotOptions: extraGenSnapshotOptions,
      splitDebugInfo: splitDebugInfo,
      dartObfuscation: dartObfuscation,
    );
    if (snapshotExitCode != 0) {
      throw Exception('AOT snapshotter exited with code $snapshotExitCode');
    }
  }
}

/// Generate an ELF binary from a dart kernel file in profile mode.
class AotElfProfile extends AotElfBase {
  const AotElfProfile(this.targetPlatform);

  @override
  String get name => 'aot_elf_profile';

  @override
  List<Source> get inputs => <Source>[
    const Source.pattern(
      '{FLUTTER_ROOT}/packages/flutter_tools/lib/src/build_system/targets/common.dart',
    ),
    const Source.pattern('{BUILD_DIR}/app.dill'),
    const Source.artifact(Artifact.engineDartBinary),
    const Source.artifact(Artifact.skyEnginePath),
    Source.artifact(Artifact.genSnapshot, platform: targetPlatform, mode: BuildMode.profile),
  ];

  @override
  List<Source> get outputs => const <Source>[Source.pattern('{BUILD_DIR}/app.so')];

  @override
  List<Target> get dependencies => const <Target>[KernelSnapshot()];

  final TargetPlatform targetPlatform;
}

/// Generate an ELF binary from a dart kernel file in release mode.
class AotElfRelease extends AotElfBase {
  const AotElfRelease(this.targetPlatform);

  @override
  String get name => 'aot_elf_release';

  @override
  List<Source> get inputs => <Source>[
    const Source.pattern(
      '{FLUTTER_ROOT}/packages/flutter_tools/lib/src/build_system/targets/common.dart',
    ),
    const Source.pattern('{BUILD_DIR}/app.dill'),
    const Source.artifact(Artifact.engineDartBinary),
    const Source.artifact(Artifact.skyEnginePath),
    Source.artifact(Artifact.genSnapshot, platform: targetPlatform, mode: BuildMode.release),
  ];

  @override
  List<Source> get outputs => const <Source>[Source.pattern('{BUILD_DIR}/app.so')];

  @override
  List<Target> get dependencies => const <Target>[KernelSnapshot()];

  final TargetPlatform targetPlatform;
}

/// Copies the pre-built flutter aot bundle.
// This is a one-off rule for implementing build aot in terms of assemble.
abstract class CopyFlutterAotBundle extends Target {
  const CopyFlutterAotBundle();

  @override
  List<Source> get inputs => const <Source>[Source.pattern('{BUILD_DIR}/app.so')];

  @override
  List<Source> get outputs => const <Source>[Source.pattern('{OUTPUT_DIR}/app.so')];

  @override
  Future<void> build(Environment environment) async {
    final File outputFile = environment.outputDir.childFile('app.so');
    if (!outputFile.parent.existsSync()) {
      outputFile.parent.createSync(recursive: true);
    }
    environment.buildDir.childFile('app.so').copySync(outputFile.path);
  }
}

/// Lipo CLI tool wrapper shared by iOS and macOS builds.
abstract final class Lipo {
  /// Create a "fat" binary by combining multiple architecture-specific ones.
  /// `skipMissingInputs` can be changed to `true` to first check whether
  /// the expected input paths exist and ignore the command if they don't.
  /// Otherwise, `lipo` would fail if the given paths didn't exist.
  static Future<void> create(
    Environment environment,
    List<DarwinArch> darwinArchs, {
    required String relativePath,
    required String inputDir,
    bool skipMissingInputs = false,
  }) async {
    final String resultPath = environment.fileSystem.path.join(
      environment.buildDir.path,
      relativePath,
    );
    environment.fileSystem.directory(resultPath).parent.createSync(recursive: true);

    Iterable<String> inputPaths = darwinArchs.map(
      (DarwinArch iosArch) =>
          environment.fileSystem.path.join(inputDir, iosArch.name, relativePath),
    );
    if (skipMissingInputs) {
      inputPaths = inputPaths.where(environment.fileSystem.isFileSync);
      if (inputPaths.isEmpty) {
        return;
      }
    }

    final List<String> lipoArgs = <String>['lipo', ...inputPaths, '-create', '-output', resultPath];

    final ProcessResult result = await environment.processManager.run(lipoArgs);
    if (result.exitCode != 0) {
      throw Exception('lipo exited with code ${result.exitCode}.\n${result.stderr}');
    }
  }
}
