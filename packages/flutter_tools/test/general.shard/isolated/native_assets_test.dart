// Copyright 2014 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

import 'package:file/file.dart';
import 'package:file/memory.dart';
import 'package:file_testing/file_testing.dart';
import 'package:flutter_tools/src/artifacts.dart';
import 'package:flutter_tools/src/base/file_system.dart';
import 'package:flutter_tools/src/base/logger.dart';
import 'package:flutter_tools/src/build_info.dart';
import 'package:flutter_tools/src/build_system/build_system.dart';
import 'package:flutter_tools/src/build_system/targets/native_assets.dart';
import 'package:flutter_tools/src/features.dart';
import 'package:flutter_tools/src/globals.dart' as globals;
import 'package:flutter_tools/src/isolated/native_assets/native_assets.dart';
import 'package:native_assets_cli/code_assets_builder.dart';
import 'package:package_config/package_config_types.dart';

import '../../src/common.dart';
import '../../src/context.dart';
import '../../src/fakes.dart';
import 'fake_native_assets_build_runner.dart';

void main() {
  late FakeProcessManager processManager;
  late Environment environment;
  late Artifacts artifacts;
  late FileSystem fileSystem;
  late BufferLogger logger;
  late Uri projectUri;

  setUp(() {
    processManager = FakeProcessManager.empty();
    logger = BufferLogger.test();
    artifacts = Artifacts.test();
    fileSystem = MemoryFileSystem.test();
    environment = Environment.test(
      fileSystem.currentDirectory,
      inputs: <String, String>{},
      artifacts: artifacts,
      processManager: processManager,
      fileSystem: fileSystem,
      logger: logger,
    );
    environment.buildDir.createSync(recursive: true);
    projectUri = environment.projectDir.uri;
  });

  testUsingContext(
    'build with no package config',
    overrides: <Type, Generator>{ProcessManager: () => FakeProcessManager.empty()},
    () async {
      await runFlutterSpecificDartBuild(
        environmentDefines: <String, String>{kBuildMode: BuildMode.debug.cliName},
        targetPlatform: TargetPlatform.windows_x64,
        projectUri: projectUri,
        fileSystem: fileSystem,
        buildRunner: FakeFlutterNativeAssetsBuildRunner(hasPackageConfigResult: false),
      );
      expect(
        (globals.logger as BufferLogger).traceText,
        contains('No package config found. Skipping native assets compilation.'),
      );
    },
  );

  testUsingContext(
    'Native assets: non-bundled libraries require no copying',
    overrides: <Type, Generator>{
      FeatureFlags: () => TestFeatureFlags(isNativeAssetsEnabled: true),
      ProcessManager: () => FakeProcessManager.empty(),
    },
    () async {
      final File packageConfig = environment.projectDir.childFile('.dart_tool/package_config.json');
      final Uri nonFlutterTesterAssetUri = environment.buildDir.childFile('native_assets.json').uri;
      await packageConfig.parent.create();
      await packageConfig.create();

      final File directSoFile = environment.projectDir.childFile('direct.so');
      directSoFile.writeAsBytesSync(<int>[]);

      CodeAsset makeCodeAsset(String name, LinkMode linkMode, [Uri? file]) => CodeAsset(
        package: 'bar',
        name: name,
        linkMode: linkMode,
        os: OS.linux,
        architecture: Architecture.x64,
        file: file,
      );

      final Map<String, String> environmentDefines = <String, String>{
        kBuildMode: BuildMode.release.cliName,
      };
      final List<CodeAsset> codeAssets = <CodeAsset>[
        makeCodeAsset('malloc', LookupInProcess()),
        makeCodeAsset('free', LookupInExecutable()),
        makeCodeAsset('draw', DynamicLoadingSystem(Uri.file('/usr/lib/skia.so'))),
      ];
      final DartBuildResult dartBuildResult = await runFlutterSpecificDartBuild(
        environmentDefines: environmentDefines,
        targetPlatform: TargetPlatform.linux_x64,
        projectUri: projectUri,
        fileSystem: fileSystem,
        buildRunner: FakeFlutterNativeAssetsBuildRunner(
          packagesWithNativeAssetsResult: <Package>[Package('bar', projectUri)],
          buildResult: FakeFlutterNativeAssetsBuilderResult.fromAssets(codeAssets: codeAssets),
          linkResult: FakeFlutterNativeAssetsBuilderResult.fromAssets(codeAssets: codeAssets),
        ),
      );
      await installCodeAssets(
        dartBuildResult: dartBuildResult,
        environmentDefines: environmentDefines,
        targetPlatform: TargetPlatform.windows_x64,
        projectUri: projectUri,
        fileSystem: fileSystem,
        nativeAssetsFileUri: nonFlutterTesterAssetUri,
      );
      expect(testLogger.traceText, isNot(contains('Copying native assets to')));
    },
  );

  testUsingContext(
    'build with assets but not enabled',
    overrides: <Type, Generator>{ProcessManager: () => FakeProcessManager.empty()},
    () async {
      final File packageConfig = environment.projectDir.childFile('.dart_tool/package_config.json');
      await packageConfig.parent.create();
      await packageConfig.create();
      expect(
        () => runFlutterSpecificDartBuild(
          environmentDefines: <String, String>{kBuildMode: BuildMode.debug.cliName},
          targetPlatform: TargetPlatform.windows_x64,
          projectUri: projectUri,
          fileSystem: fileSystem,
          buildRunner: FakeFlutterNativeAssetsBuildRunner(
            packagesWithNativeAssetsResult: <Package>[Package('bar', projectUri)],
          ),
        ),
        throwsToolExit(
          message:
              'Package(s) bar require the native assets feature to be enabled. '
              'Enable using `flutter config --enable-native-assets`.',
        ),
      );
    },
  );

  testUsingContext(
    'build no assets',
    overrides: <Type, Generator>{
      FeatureFlags: () => TestFeatureFlags(isNativeAssetsEnabled: true),
      ProcessManager: () => FakeProcessManager.empty(),
    },
    () async {
      final File packageConfig = environment.projectDir.childFile('.dart_tool/package_config.json');
      final Uri nonFlutterTesterAssetUri =
          environment.buildDir.childFile(InstallCodeAssets.nativeAssetsFilename).uri;
      await packageConfig.parent.create();
      await packageConfig.create();

      final Map<String, String> environmentDefines = <String, String>{
        kBuildMode: BuildMode.debug.cliName,
      };
      final DartBuildResult dartBuildResult = await runFlutterSpecificDartBuild(
        environmentDefines: environmentDefines,
        targetPlatform: TargetPlatform.windows_x64,
        projectUri: projectUri,
        fileSystem: fileSystem,
        buildRunner: FakeFlutterNativeAssetsBuildRunner(
          packagesWithNativeAssetsResult: <Package>[Package('bar', projectUri)],
        ),
      );
      await installCodeAssets(
        dartBuildResult: dartBuildResult,
        environmentDefines: environmentDefines,
        targetPlatform: TargetPlatform.windows_x64,
        projectUri: projectUri,
        fileSystem: fileSystem,
        nativeAssetsFileUri: nonFlutterTesterAssetUri,
      );
      expect(
        await fileSystem.file(nonFlutterTesterAssetUri).readAsString(),
        isNot(contains('package:bar/bar.dart')),
      );
      expect(
        environment.projectDir
            .childDirectory('build')
            .childDirectory('native_assets')
            .childDirectory('windows'),
        exists,
      );
    },
  );

  testUsingContext(
    'Native assets build error',
    overrides: <Type, Generator>{
      FeatureFlags: () => TestFeatureFlags(isNativeAssetsEnabled: true),
      ProcessManager: () => FakeProcessManager.empty(),
    },
    () async {
      final File packageConfig = environment.projectDir.childFile('.dart_tool/package_config.json');
      await packageConfig.parent.create();
      await packageConfig.create();
      expect(
        () => runFlutterSpecificDartBuild(
          environmentDefines: <String, String>{kBuildMode: BuildMode.debug.cliName},
          targetPlatform: TargetPlatform.linux_x64,
          projectUri: projectUri,
          fileSystem: fileSystem,
          buildRunner: FakeFlutterNativeAssetsBuildRunner(
            packagesWithNativeAssetsResult: <Package>[Package('bar', projectUri)],
            buildResult: null,
          ),
        ),
        throwsToolExit(message: 'Building native assets failed. See the logs for more details.'),
      );
    },
  );

  testUsingContext(
    'Native assets: no duplicate assets with linking',
    overrides: <Type, Generator>{
      FeatureFlags: () => TestFeatureFlags(isNativeAssetsEnabled: true),
      ProcessManager: () => FakeProcessManager.empty(),
    },
    () async {
      final File packageConfig = environment.projectDir.childFile('.dart_tool/package_config.json');
      await packageConfig.parent.create();
      await packageConfig.create();

      final File directSoFile = environment.projectDir.childFile('direct.so');
      directSoFile.writeAsBytesSync(<int>[]);
      final File linkableAFile = environment.projectDir.childFile('linkable.a');
      linkableAFile.writeAsBytesSync(<int>[]);
      final File linkedSoFile = environment.projectDir.childFile('linked.so');
      linkedSoFile.writeAsBytesSync(<int>[]);

      CodeAsset makeCodeAsset(String name, Uri file, LinkMode linkMode) => CodeAsset(
        package: 'bar',
        name: name,
        linkMode: linkMode,
        os: OS.linux,
        architecture: Architecture.x64,
        file: file,
      );

      final DartBuildResult result = await runFlutterSpecificDartBuild(
        environmentDefines: <String, String>{
          // Release mode means the dart build has linking enabled.
          kBuildMode: BuildMode.release.cliName,
        },
        targetPlatform: TargetPlatform.linux_x64,
        projectUri: projectUri,
        fileSystem: fileSystem,
        buildRunner: FakeFlutterNativeAssetsBuildRunner(
          packagesWithNativeAssetsResult: <Package>[Package('bar', projectUri)],
          buildResult: FakeFlutterNativeAssetsBuilderResult.fromAssets(
            codeAssets: <CodeAsset>[
              makeCodeAsset('direct', directSoFile.uri, DynamicLoadingBundled()),
            ],
            codeAssetsForLinking: <String, List<CodeAsset>>{
              'package:bar': <CodeAsset>[
                makeCodeAsset('linkable', linkableAFile.uri, StaticLinking()),
              ],
            },
          ),
          linkResult: FakeFlutterNativeAssetsBuilderResult.fromAssets(
            codeAssets: <CodeAsset>[
              makeCodeAsset('direct', directSoFile.uri, DynamicLoadingBundled()),
              makeCodeAsset('linked', linkedSoFile.uri, DynamicLoadingBundled()),
            ],
          ),
        ),
      );
      expect(result.codeAssets.map((CodeAsset c) => c.file!.toString()).toList()..sort(), <String>[
        directSoFile.uri.toString(),
        linkedSoFile.uri.toString(),
      ]);
    },
  );
}
