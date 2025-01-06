// Copyright 2014 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

import 'package:file/file.dart';
import 'package:file/memory.dart';
import 'package:file_testing/file_testing.dart';
import 'package:flutter_tools/src/artifacts.dart';
import 'package:flutter_tools/src/base/file_system.dart';
import 'package:flutter_tools/src/base/logger.dart';
import 'package:flutter_tools/src/base/platform.dart';
import 'package:flutter_tools/src/build_info.dart';
import 'package:flutter_tools/src/build_system/build_system.dart';
import 'package:flutter_tools/src/build_system/targets/native_assets.dart';
import 'package:flutter_tools/src/features.dart';
import 'package:flutter_tools/src/globals.dart' as globals;
import 'package:flutter_tools/src/isolated/native_assets/native_assets.dart';
import 'package:native_assets_cli/code_assets_builder.dart';
import 'package:package_config/package_config_types.dart';

import '../../../src/common.dart';
import '../../../src/context.dart';
import '../../../src/fakes.dart';
import '../fake_native_assets_build_runner.dart';

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

  for (final BuildMode buildMode in <BuildMode>[BuildMode.debug, BuildMode.release]) {
    testUsingContext(
      'build with assets $buildMode',
      overrides: <Type, Generator>{
        FeatureFlags: () => TestFeatureFlags(isNativeAssetsEnabled: true),
        ProcessManager:
            () => FakeProcessManager.list(<FakeCommand>[
              const FakeCommand(
                command: <Pattern>[
                  'lipo',
                  '-create',
                  '-output',
                  '/build/native_assets/ios/bar.framework/bar',
                  'arm64/libbar.dylib',
                  'x64/libbar.dylib',
                ],
              ),
              FakeCommand(
                command: const <Pattern>[
                  'otool',
                  '-D',
                  '/build/native_assets/ios/bar.framework/bar',
                ],
                stdout: <String>[
                  '/build/native_assets/ios/bar.framework/bar (architecture x86_64):',
                  '@rpath/libbar.dylib',
                  '/build/native_assets/ios/bar.framework/bar (architecture arm64):',
                  '@rpath/libbar.dylib',
                ].join('\n'),
              ),
              const FakeCommand(
                command: <Pattern>[
                  'lipo',
                  '-create',
                  '-output',
                  '/build/native_assets/ios/buz.framework/buz',
                  'arm64/libbuz.dylib',
                  'x64/libbuz.dylib',
                ],
              ),
              FakeCommand(
                command: const <Pattern>[
                  'otool',
                  '-D',
                  '/build/native_assets/ios/buz.framework/buz',
                ],
                stdout: <String>[
                  '/build/native_assets/ios/buz.framework/buz (architecture x86_64):',
                  '@rpath/libbuz.dylib',
                  '/build/native_assets/ios/buz.framework/buz (architecture arm64):',
                  '@rpath/libbuz.dylib',
                ].join('\n'),
              ),
              const FakeCommand(
                command: <Pattern>[
                  'install_name_tool',
                  '-id',
                  '@rpath/bar.framework/bar',
                  '-change',
                  '@rpath/libbar.dylib',
                  '@rpath/bar.framework/bar',
                  '-change',
                  '@rpath/libbuz.dylib',
                  '@rpath/buz.framework/buz',
                  '/build/native_assets/ios/bar.framework/bar',
                ],
              ),
              FakeCommand(
                command: <Pattern>[
                  'codesign',
                  '--force',
                  '--sign',
                  '-',
                  if (buildMode == BuildMode.debug) '--timestamp=none',
                  '/build/native_assets/ios/bar.framework',
                ],
              ),
              const FakeCommand(
                command: <Pattern>[
                  'install_name_tool',
                  '-id',
                  '@rpath/buz.framework/buz',
                  '-change',
                  '@rpath/libbar.dylib',
                  '@rpath/bar.framework/bar',
                  '-change',
                  '@rpath/libbuz.dylib',
                  '@rpath/buz.framework/buz',
                  '/build/native_assets/ios/buz.framework/buz',
                ],
              ),
              FakeCommand(
                command: <Pattern>[
                  'codesign',
                  '--force',
                  '--sign',
                  '-',
                  if (buildMode == BuildMode.debug) '--timestamp=none',
                  '/build/native_assets/ios/buz.framework',
                ],
              ),
            ]),
      },
      () async {
        if (const LocalPlatform().isWindows) {
          return; // Backslashes in commands, but we will never run these commands on Windows.
        }
        final File packageConfig = environment.projectDir.childFile(
          '.dart_tool/package_config.json',
        );
        final Uri nonFlutterTesterAssetUri =
            environment.buildDir.childFile(InstallCodeAssets.nativeAssetsFilename).uri;
        await packageConfig.parent.create();
        await packageConfig.create();

        List<CodeAsset> codeAssets(OS targetOS, CodeConfig codeConfig) => <CodeAsset>[
          CodeAsset(
            package: 'bar',
            name: 'bar.dart',
            linkMode: DynamicLoadingBundled(),
            os: targetOS,
            architecture: codeConfig.targetArchitecture,
            file: Uri.file('${codeConfig.targetArchitecture}/libbar.dylib'),
          ),
          CodeAsset(
            package: 'buz',
            name: 'buz.dart',
            linkMode: DynamicLoadingBundled(),
            os: targetOS,
            architecture: codeConfig.targetArchitecture,
            file: Uri.file('${codeConfig.targetArchitecture}/libbuz.dylib'),
          ),
        ];
        final FakeFlutterNativeAssetsBuildRunner buildRunner = FakeFlutterNativeAssetsBuildRunner(
          packagesWithNativeAssetsResult: <Package>[Package('bar', projectUri)],
          onBuild:
              (BuildConfig config) => FakeFlutterNativeAssetsBuilderResult.fromAssets(
                codeAssets: codeAssets(config.codeConfig.targetOS, config.codeConfig),
              ),
          onLink:
              (LinkConfig config) =>
                  buildMode == BuildMode.debug
                      ? null
                      : FakeFlutterNativeAssetsBuilderResult.fromAssets(
                        codeAssets: codeAssets(config.codeConfig.targetOS, config.codeConfig),
                      ),
        );
        final Map<String, String> environmentDefines = <String, String>{
          kBuildMode: buildMode.cliName,
          kSdkRoot: '.../iPhone Simulator',
          kIosArchs: 'arm64 x86_64',
        };
        final DartBuildResult dartBuildResult = await runFlutterSpecificDartBuild(
          environmentDefines: environmentDefines,
          targetPlatform: TargetPlatform.ios,
          projectUri: projectUri,
          fileSystem: fileSystem,
          buildRunner: buildRunner,
        );
        await installCodeAssets(
          dartBuildResult: dartBuildResult,
          environmentDefines: environmentDefines,
          targetPlatform: TargetPlatform.ios,
          projectUri: projectUri,
          fileSystem: fileSystem,
          nativeAssetsFileUri: nonFlutterTesterAssetUri,
        );
        expect(
          (globals.logger as BufferLogger).traceText,
          stringContainsInOrder(<String>[
            'Building native assets for ios [arm64, x64].',
            'Building native assets for ios [arm64, x64] done.',
          ]),
        );
        expect(environment.buildDir.childFile(InstallCodeAssets.nativeAssetsFilename), exists);
        // Two archs.
        expect(buildRunner.buildInvocations, 2);
        expect(buildRunner.linkInvocations, buildMode == BuildMode.release ? 2 : 0);
      },
    );
  }
}
