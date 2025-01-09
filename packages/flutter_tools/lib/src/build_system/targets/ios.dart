// Copyright 2014 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

import 'package:meta/meta.dart';
import 'package:unified_analytics/unified_analytics.dart';

import '../../artifacts.dart';
import '../../base/build.dart';
import '../../base/common.dart';
import '../../base/file_system.dart';
import '../../base/io.dart';
import '../../base/process.dart';
import '../../build_info.dart';
import '../../devfs.dart';
import '../../globals.dart' as globals;
import '../../ios/mac.dart';
import '../../macos/xcode.dart';
import '../../project.dart';
import '../../reporting/reporting.dart';
import '../build_system.dart';
import '../depfile.dart';
import '../exceptions.dart';
import '../tools/shader_compiler.dart';
import 'assets.dart';
import 'common.dart';
import 'icon_tree_shaker.dart';
import 'native_assets.dart';

/// Supports compiling a dart kernel file to an assembly file.
///
/// If more than one iOS arch is provided, then this rule will
/// produce a universal binary.
abstract class AotAssemblyBase extends Target {
  const AotAssemblyBase();

  @override
  String get analyticsName => 'ios_aot';

  @override
  Future<void> build(Environment environment) async {
    final AOTSnapshotter snapshotter = AOTSnapshotter(
      fileSystem: environment.fileSystem,
      logger: environment.logger,
      xcode: globals.xcode!,
      artifacts: environment.artifacts,
      processManager: environment.processManager,
    );
    final String buildOutputPath = environment.buildDir.path;
    final String? environmentBuildMode = environment.defines[kBuildMode];
    if (environmentBuildMode == null) {
      throw MissingDefineException(kBuildMode, 'aot_assembly');
    }
    final String? environmentTargetPlatform = environment.defines[kTargetPlatform];
    if (environmentTargetPlatform == null) {
      throw MissingDefineException(kTargetPlatform, 'aot_assembly');
    }
    final String? sdkRoot = environment.defines[kSdkRoot];
    if (sdkRoot == null) {
      throw MissingDefineException(kSdkRoot, 'aot_assembly');
    }

    final List<String> extraGenSnapshotOptions = decodeCommaSeparated(
      environment.defines,
      kExtraGenSnapshotOptions,
    );
    final BuildMode buildMode = BuildMode.fromCliName(environmentBuildMode);
    final TargetPlatform targetPlatform = getTargetPlatformForName(environmentTargetPlatform);
    final String? splitDebugInfo = environment.defines[kSplitDebugInfo];
    final bool dartObfuscation = environment.defines[kDartObfuscation] == 'true';
    final List<DarwinArch> darwinArchs =
        environment.defines[kIosArchs]?.split(' ').map(getIOSArchForName).toList() ??
        <DarwinArch>[DarwinArch.arm64];
    if (targetPlatform != TargetPlatform.ios) {
      throw Exception('aot_assembly is only supported for iOS applications.');
    }

    final EnvironmentType? environmentType = environmentTypeFromSdkroot(
      sdkRoot,
      environment.fileSystem,
    );
    if (environmentType == EnvironmentType.simulator) {
      throw Exception(
        'release/profile builds are only supported for physical devices. '
        'attempted to build for simulator.',
      );
    }
    final String? codeSizeDirectory = environment.defines[kCodeSizeDirectory];

    // If we're building multiple iOS archs the binaries need to be lipo'd
    // together.
    final List<Future<int>> pending = <Future<int>>[];
    for (final DarwinArch darwinArch in darwinArchs) {
      final List<String> archExtraGenSnapshotOptions = List<String>.of(extraGenSnapshotOptions);
      if (codeSizeDirectory != null) {
        final File codeSizeFile = environment.fileSystem
            .directory(codeSizeDirectory)
            .childFile('snapshot.${darwinArch.name}.json');
        final File precompilerTraceFile = environment.fileSystem
            .directory(codeSizeDirectory)
            .childFile('trace.${darwinArch.name}.json');
        archExtraGenSnapshotOptions.add('--write-v8-snapshot-profile-to=${codeSizeFile.path}');
        archExtraGenSnapshotOptions.add('--trace-precompiler-to=${precompilerTraceFile.path}');
      }
      pending.add(
        snapshotter.build(
          platform: targetPlatform,
          buildMode: buildMode,
          mainPath: environment.buildDir.childFile('app.dill').path,
          outputPath: environment.fileSystem.path.join(buildOutputPath, darwinArch.name),
          darwinArch: darwinArch,
          sdkRoot: sdkRoot,
          quiet: true,
          splitDebugInfo: splitDebugInfo,
          dartObfuscation: dartObfuscation,
          extraGenSnapshotOptions: archExtraGenSnapshotOptions,
        ),
      );
    }
    final List<int> results = await Future.wait(pending);
    if (results.any((int result) => result != 0)) {
      throw Exception('AOT snapshotter exited with code ${results.join()}');
    }

    // Combine the app lib into a fat framework.
    await Lipo.create(
      environment,
      darwinArchs,
      relativePath: 'App.framework/App',
      inputDir: buildOutputPath,
    );

    // And combine the dSYM for each architecture too, if it was created.
    await Lipo.create(
      environment,
      darwinArchs,
      relativePath: 'App.framework.dSYM/Contents/Resources/DWARF/App',
      inputDir: buildOutputPath,
      // Don't fail if the dSYM wasn't created (i.e. during a debug build).
      skipMissingInputs: true,
    );
  }
}

/// Generate an assembly target from a dart kernel file in release mode.
class AotAssemblyRelease extends AotAssemblyBase {
  const AotAssemblyRelease();

  @override
  String get name => 'aot_assembly_release';

  @override
  List<Source> get inputs => const <Source>[
    Source.pattern('{FLUTTER_ROOT}/packages/flutter_tools/lib/src/build_system/targets/ios.dart'),
    Source.pattern('{BUILD_DIR}/app.dill'),
    Source.artifact(Artifact.engineDartBinary),
    Source.artifact(Artifact.skyEnginePath),
    // TODO(zanderso): cannot reference gen_snapshot with artifacts since
    // it resolves to a file (ios/gen_snapshot) that never exists. This was
    // split into gen_snapshot_arm64 and gen_snapshot_armv7.
    // Source.artifact(Artifact.genSnapshot,
    //   platform: TargetPlatform.ios,
    //   mode: BuildMode.release,
    // ),
  ];

  @override
  List<Source> get outputs => const <Source>[Source.pattern('{OUTPUT_DIR}/App.framework/App')];

  @override
  List<Target> get dependencies => const <Target>[ReleaseUnpackIOS(), KernelSnapshot()];
}

/// Generate an assembly target from a dart kernel file in profile mode.
class AotAssemblyProfile extends AotAssemblyBase {
  const AotAssemblyProfile();

  @override
  String get name => 'aot_assembly_profile';

  @override
  List<Source> get inputs => const <Source>[
    Source.pattern('{FLUTTER_ROOT}/packages/flutter_tools/lib/src/build_system/targets/ios.dart'),
    Source.pattern('{BUILD_DIR}/app.dill'),
    Source.artifact(Artifact.engineDartBinary),
    Source.artifact(Artifact.skyEnginePath),
    // TODO(zanderso): cannot reference gen_snapshot with artifacts since
    // it resolves to a file (ios/gen_snapshot) that never exists. This was
    // split into gen_snapshot_arm64 and gen_snapshot_armv7.
    // Source.artifact(Artifact.genSnapshot,
    //   platform: TargetPlatform.ios,
    //   mode: BuildMode.profile,
    // ),
  ];

  @override
  List<Source> get outputs => const <Source>[Source.pattern('{OUTPUT_DIR}/App.framework/App')];

  @override
  List<Target> get dependencies => const <Target>[ProfileUnpackIOS(), KernelSnapshot()];
}

/// Create a trivial App.framework file for debug iOS builds.
class DebugUniversalFramework extends Target {
  const DebugUniversalFramework();

  @override
  String get name => 'debug_universal_framework';

  @override
  List<Target> get dependencies => const <Target>[DebugUnpackIOS(), KernelSnapshot()];

  @override
  List<Source> get inputs => const <Source>[
    Source.pattern('{FLUTTER_ROOT}/packages/flutter_tools/lib/src/build_system/targets/ios.dart'),
  ];

  @override
  List<Source> get outputs => const <Source>[Source.pattern('{BUILD_DIR}/App.framework/App')];

  @override
  Future<void> build(Environment environment) async {
    final String? sdkRoot = environment.defines[kSdkRoot];
    if (sdkRoot == null) {
      throw MissingDefineException(kSdkRoot, name);
    }

    // Generate a trivial App.framework.
    final Set<String>? iosArchNames = environment.defines[kIosArchs]?.split(' ').toSet();
    final File output = environment.buildDir.childDirectory('App.framework').childFile('App');
    environment.buildDir.createSync(recursive: true);
    await _createStubAppFramework(output, environment, iosArchNames, sdkRoot);
  }
}

/// Copy the iOS framework to the correct copy dir by invoking 'rsync'.
///
/// This class is abstract to share logic between the three concrete
/// implementations. The shelling out is done to avoid complications with
/// preserving special files (e.g., symbolic links) in the framework structure.
abstract class UnpackIOS extends Target {
  const UnpackIOS();

  @override
  List<Source> get inputs => <Source>[
    const Source.pattern(
      '{FLUTTER_ROOT}/packages/flutter_tools/lib/src/build_system/targets/ios.dart',
    ),
    Source.artifact(Artifact.flutterXcframework, platform: TargetPlatform.ios, mode: buildMode),
  ];

  @override
  List<Source> get outputs => const <Source>[
    Source.pattern('{OUTPUT_DIR}/Flutter.framework/Flutter'),
  ];

  @override
  List<Target> get dependencies => <Target>[];

  @visibleForOverriding
  BuildMode get buildMode;

  @override
  Future<void> build(Environment environment) async {
    final String? sdkRoot = environment.defines[kSdkRoot];
    if (sdkRoot == null) {
      throw MissingDefineException(kSdkRoot, name);
    }
    final String? archs = environment.defines[kIosArchs];
    if (archs == null) {
      throw MissingDefineException(kIosArchs, name);
    }
    await _copyFramework(environment, sdkRoot);

    final File frameworkBinary = environment.outputDir
        .childDirectory('Flutter.framework')
        .childFile('Flutter');
    final String frameworkBinaryPath = frameworkBinary.path;
    if (!await frameworkBinary.exists()) {
      throw Exception('Binary $frameworkBinaryPath does not exist, cannot thin');
    }
    await _thinFramework(environment, frameworkBinaryPath, archs);
    await _signFramework(environment, frameworkBinary, buildMode);
  }

  Future<void> _copyFramework(Environment environment, String sdkRoot) async {
    // Copy Flutter framework.
    final EnvironmentType? environmentType = environmentTypeFromSdkroot(
      sdkRoot,
      environment.fileSystem,
    );
    final String basePath = environment.artifacts.getArtifactPath(
      Artifact.flutterFramework,
      platform: TargetPlatform.ios,
      mode: buildMode,
      environmentType: environmentType,
    );

    final ProcessResult result = await environment.processManager.run(<String>[
      'rsync',
      '-av',
      '--delete',
      '--filter',
      '- .DS_Store/',
      '--chmod=Du=rwx,Dgo=rx,Fu=rw,Fgo=r',
      basePath,
      environment.outputDir.path,
    ]);
    if (result.exitCode != 0) {
      throw Exception(
        'Failed to copy framework (exit ${result.exitCode}:\n'
        '${result.stdout}\n---\n${result.stderr}',
      );
    }

    // Copy Flutter framework dSYM (debug symbol) bundle, if present.
    final Directory frameworkDsym = environment.fileSystem.directory(
      environment.artifacts.getArtifactPath(
        Artifact.flutterFrameworkDsym,
        platform: TargetPlatform.ios,
        mode: buildMode,
        environmentType: environmentType,
      ),
    );
    if (frameworkDsym.existsSync()) {
      final ProcessResult result = await environment.processManager.run(<String>[
        'rsync',
        '-av',
        '--delete',
        '--filter',
        '- .DS_Store/',
        '--chmod=Du=rwx,Dgo=rx,Fu=rw,Fgo=r',
        frameworkDsym.path,
        environment.outputDir.path,
      ]);
      if (result.exitCode != 0) {
        throw Exception(
          'Failed to copy framework dSYM (exit ${result.exitCode}:\n'
          '${result.stdout}\n---\n${result.stderr}',
        );
      }
    }
  }

  /// Destructively thin Flutter.framework to include only the specified architectures.
  Future<void> _thinFramework(
    Environment environment,
    String frameworkBinaryPath,
    String archs,
  ) async {
    final List<String> archList = archs.split(' ').toList();
    final ProcessResult infoResult = await environment.processManager.run(<String>[
      'lipo',
      '-info',
      frameworkBinaryPath,
    ]);
    final String lipoInfo = infoResult.stdout as String;

    final ProcessResult verifyResult = await environment.processManager.run(<String>[
      'lipo',
      frameworkBinaryPath,
      '-verify_arch',
      ...archList,
    ]);

    if (verifyResult.exitCode != 0) {
      throw Exception(
        'Binary $frameworkBinaryPath does not contain architectures "$archs".\n'
        '\n'
        'lipo -info:\n'
        '$lipoInfo',
      );
    }

    // Skip thinning for non-fat executables.
    if (lipoInfo.startsWith('Non-fat file:')) {
      environment.logger.printTrace('Skipping lipo for non-fat file $frameworkBinaryPath');
      return;
    }

    // Thin in-place.
    final ProcessResult extractResult = await environment.processManager.run(<String>[
      'lipo',
      '-output',
      frameworkBinaryPath,
      for (final String arch in archList) ...<String>['-extract', arch],
      ...<String>[frameworkBinaryPath],
    ]);

    if (extractResult.exitCode != 0) {
      throw Exception(
        'Failed to extract architectures "$archs" for $frameworkBinaryPath.\n'
        '\n'
        'stderr:\n'
        '${extractResult.stderr}\n\n'
        'lipo -info:\n'
        '$lipoInfo',
      );
    }
  }
}

/// Unpack the release prebuilt engine framework.
class ReleaseUnpackIOS extends UnpackIOS {
  const ReleaseUnpackIOS();

  @override
  String get name => 'release_unpack_ios';

  @override
  BuildMode get buildMode => BuildMode.release;
}

/// Unpack the profile prebuilt engine framework.
class ProfileUnpackIOS extends UnpackIOS {
  const ProfileUnpackIOS();

  @override
  String get name => 'profile_unpack_ios';

  @override
  BuildMode get buildMode => BuildMode.profile;
}

/// Unpack the debug prebuilt engine framework.
class DebugUnpackIOS extends UnpackIOS {
  const DebugUnpackIOS();

  @override
  String get name => 'debug_unpack_ios';

  @override
  BuildMode get buildMode => BuildMode.debug;
}

/// The base class for all iOS bundle targets.
///
/// This is responsible for setting up the basic App.framework structure, including:
/// * Copying the app.dill/kernel_blob.bin from the build directory to assets (debug)
/// * Copying the precompiled isolate/vm data from the engine (debug)
/// * Copying the flutter assets to App.framework/flutter_assets
/// * Copying either the stub or real App assembly file to App.framework/App
abstract class IosAssetBundle extends Target {
  const IosAssetBundle();

  @override
  List<Target> get dependencies => const <Target>[KernelSnapshot(), InstallCodeAssets()];

  @override
  List<Source> get inputs => const <Source>[
    Source.pattern('{BUILD_DIR}/App.framework/App'),
    Source.pattern('{PROJECT_DIR}/pubspec.yaml'),
    ...IconTreeShaker.inputs,
    ...ShaderCompiler.inputs,
  ];

  @override
  List<Source> get outputs => const <Source>[
    Source.pattern('{OUTPUT_DIR}/App.framework/App'),
    Source.pattern('{OUTPUT_DIR}/App.framework/Info.plist'),
  ];

  @override
  List<String> get depfiles => <String>['flutter_assets.d'];

  @override
  Future<void> build(Environment environment) async {
    final String? environmentBuildMode = environment.defines[kBuildMode];
    if (environmentBuildMode == null) {
      throw MissingDefineException(kBuildMode, name);
    }
    final BuildMode buildMode = BuildMode.fromCliName(environmentBuildMode);
    final Directory frameworkDirectory = environment.outputDir.childDirectory('App.framework');
    final File frameworkBinary = frameworkDirectory.childFile('App');
    final Directory assetDirectory = frameworkDirectory.childDirectory('flutter_assets');
    frameworkDirectory.createSync(recursive: true);
    assetDirectory.createSync();

    // Only copy the prebuilt runtimes and kernel blob in debug mode.
    if (buildMode == BuildMode.debug) {
      // Copy the App.framework to the output directory.
      environment.buildDir
          .childDirectory('App.framework')
          .childFile('App')
          .copySync(frameworkBinary.path);

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
          .copySync(assetDirectory.childFile('kernel_blob.bin').path);
      environment.fileSystem
          .file(vmSnapshotData)
          .copySync(assetDirectory.childFile('vm_snapshot_data').path);
      environment.fileSystem
          .file(isolateSnapshotData)
          .copySync(assetDirectory.childFile('isolate_snapshot_data').path);
    } else {
      environment.buildDir
          .childDirectory('App.framework')
          .childFile('App')
          .copySync(frameworkBinary.path);
    }

    // Copy the dSYM
    if (environment.buildDir.childDirectory('App.framework.dSYM').existsSync()) {
      final File dsymOutputBinary = environment.outputDir
          .childDirectory('App.framework.dSYM')
          .childDirectory('Contents')
          .childDirectory('Resources')
          .childDirectory('DWARF')
          .childFile('App');
      dsymOutputBinary.parent.createSync(recursive: true);
      environment.buildDir
          .childDirectory('App.framework.dSYM')
          .childDirectory('Contents')
          .childDirectory('Resources')
          .childDirectory('DWARF')
          .childFile('App')
          .copySync(dsymOutputBinary.path);
    }

    final FlutterProject flutterProject = FlutterProject.fromDirectory(environment.projectDir);

    // Copy the assets.
    final Depfile assetDepfile = await copyAssets(
      environment,
      assetDirectory,
      targetPlatform: TargetPlatform.ios,
      buildMode: buildMode,
      additionalInputs: <File>[
        flutterProject.ios.infoPlist,
        flutterProject.ios.appFrameworkInfoPlist,
      ],
      additionalContent: <String, DevFSContent>{
        'NativeAssetsManifest.json': DevFSFileContent(
          environment.buildDir.childFile('native_assets.json'),
        ),
      },
      flavor: environment.defines[kFlavor],
    );
    environment.depFileService.writeToFile(
      assetDepfile,
      environment.buildDir.childFile('flutter_assets.d'),
    );

    // Copy the plist from either the project or module.
    flutterProject.ios.appFrameworkInfoPlist.copySync(
      environment.outputDir.childDirectory('App.framework').childFile('Info.plist').path,
    );

    await _signFramework(environment, frameworkBinary, buildMode);
  }
}

/// Build a debug iOS application bundle.
class DebugIosApplicationBundle extends IosAssetBundle {
  const DebugIosApplicationBundle();

  @override
  String get name => 'debug_ios_bundle_flutter_assets';

  @override
  List<Source> get inputs => <Source>[
    const Source.artifact(Artifact.vmSnapshotData, mode: BuildMode.debug),
    const Source.artifact(Artifact.isolateSnapshotData, mode: BuildMode.debug),
    const Source.pattern('{BUILD_DIR}/app.dill'),
    ...super.inputs,
  ];

  @override
  List<Source> get outputs => <Source>[
    const Source.pattern('{OUTPUT_DIR}/App.framework/flutter_assets/vm_snapshot_data'),
    const Source.pattern('{OUTPUT_DIR}/App.framework/flutter_assets/isolate_snapshot_data'),
    const Source.pattern('{OUTPUT_DIR}/App.framework/flutter_assets/kernel_blob.bin'),
    ...super.outputs,
  ];

  @override
  List<Target> get dependencies => <Target>[const DebugUniversalFramework(), ...super.dependencies];
}

/// IosAssetBundle with debug symbols, used for Profile and Release builds.
abstract class _IosAssetBundleWithDSYM extends IosAssetBundle {
  const _IosAssetBundleWithDSYM();

  @override
  List<Source> get inputs => <Source>[
    ...super.inputs,
    const Source.pattern('{BUILD_DIR}/App.framework.dSYM/Contents/Resources/DWARF/App'),
  ];

  @override
  List<Source> get outputs => <Source>[
    ...super.outputs,
    const Source.pattern('{OUTPUT_DIR}/App.framework.dSYM/Contents/Resources/DWARF/App'),
  ];
}

/// Build a profile iOS application bundle.
class ProfileIosApplicationBundle extends _IosAssetBundleWithDSYM {
  const ProfileIosApplicationBundle();

  @override
  String get name => 'profile_ios_bundle_flutter_assets';

  @override
  List<Target> get dependencies => const <Target>[AotAssemblyProfile(), InstallCodeAssets()];
}

/// Build a release iOS application bundle.
class ReleaseIosApplicationBundle extends _IosAssetBundleWithDSYM {
  const ReleaseIosApplicationBundle();

  @override
  String get name => 'release_ios_bundle_flutter_assets';

  @override
  List<Target> get dependencies => const <Target>[AotAssemblyRelease(), InstallCodeAssets()];

  @override
  Future<void> build(Environment environment) async {
    bool buildSuccess = true;
    try {
      await super.build(environment);
    } catch (_) {
      // ignore: avoid_catches_without_on_clauses
      buildSuccess = false;
      rethrow;
    } finally {
      // Send a usage event when the app is being archived.
      // Since assemble is run during a `flutter build`/`run` as well as an out-of-band
      // archive command from Xcode, this is a more accurate count than `flutter build ipa` alone.
      if (environment.defines[kXcodeAction]?.toLowerCase() == 'install') {
        environment.logger.printTrace('Sending archive event if usage enabled.');
        UsageEvent(
          'assemble',
          'ios-archive',
          label: buildSuccess ? 'success' : 'fail',
          flutterUsage: environment.usage,
        ).send();
        environment.analytics.send(
          Event.appleUsageEvent(
            workflow: 'assemble',
            parameter: 'ios-archive',
            result: buildSuccess ? 'success' : 'fail',
          ),
        );
      }
    }
  }
}

/// Create an App.framework for debug iOS targets.
///
/// This framework needs to exist for the Xcode project to link/bundle,
/// but it isn't actually executed. To generate something valid, we compile a trivial
/// constant.
Future<void> _createStubAppFramework(
  File outputFile,
  Environment environment,
  Set<String>? iosArchNames,
  String sdkRoot,
) async {
  try {
    outputFile.createSync(recursive: true);
  } on Exception catch (e) {
    throwToolExit('Failed to create App.framework stub at ${outputFile.path}: $e');
  }

  final FileSystem fileSystem = environment.fileSystem;
  final Directory tempDir = fileSystem.systemTempDirectory.createTempSync(
    'flutter_tools_stub_source.',
  );
  try {
    final File stubSource = tempDir.childFile('debug_app.cc')..writeAsStringSync(r'''
  static const int Moo = 88;
  ''');

    final EnvironmentType? environmentType = environmentTypeFromSdkroot(sdkRoot, fileSystem);

    await globals.xcode!.clang(<String>[
      '-x',
      'c',
      for (final String arch in iosArchNames ?? <String>{}) ...<String>['-arch', arch],
      stubSource.path,
      '-dynamiclib',
      // Keep version in sync with AOTSnapshotter flag
      if (environmentType == EnvironmentType.physical)
        '-miphoneos-version-min=12.0'
      else
        '-miphonesimulator-version-min=12.0',
      '-Xlinker', '-rpath', '-Xlinker', '@executable_path/Frameworks',
      '-Xlinker', '-rpath', '-Xlinker', '@loader_path/Frameworks',
      '-fapplication-extension',
      '-install_name', '@rpath/App.framework/App',
      '-isysroot', sdkRoot,
      '-o', outputFile.path,
    ]);
  } finally {
    try {
      tempDir.deleteSync(recursive: true);
    } on FileSystemException {
      // Best effort. Sometimes we can't delete things from system temp.
    } on Exception catch (e) {
      throwToolExit('Failed to create App.framework stub at ${outputFile.path}: $e');
    }
  }

  await _signFramework(environment, outputFile, BuildMode.debug);
}

Future<void> _signFramework(Environment environment, File binary, BuildMode buildMode) async {
  await removeFinderExtendedAttributes(
    binary,
    ProcessUtils(processManager: environment.processManager, logger: environment.logger),
    environment.logger,
  );

  String? codesignIdentity = environment.defines[kCodesignIdentity];
  if (codesignIdentity == null || codesignIdentity.isEmpty) {
    codesignIdentity = '-';
  }
  final ProcessResult result = environment.processManager.runSync(<String>[
    'codesign',
    '--force',
    '--sign',
    codesignIdentity,
    if (buildMode != BuildMode.release) ...<String>[
      // Mimic Xcode's timestamp codesigning behavior on non-release binaries.
      '--timestamp=none',
    ],
    binary.path,
  ]);
  if (result.exitCode != 0) {
    final String stdout = (result.stdout as String).trim();
    final String stderr = (result.stderr as String).trim();
    final StringBuffer output = StringBuffer();
    output.writeln('Failed to codesign ${binary.path} with identity $codesignIdentity.');
    if (stdout.isNotEmpty) {
      output.writeln(stdout);
    }
    if (stderr.isNotEmpty) {
      output.writeln(stderr);
    }
    throw Exception(output.toString());
  }
}
