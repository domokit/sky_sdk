// Copyright 2014 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

import 'package:file/file.dart';
import 'package:file/memory.dart';
import 'package:flutter_tools/src/artifacts.dart';
import 'package:flutter_tools/src/base/file_system.dart';
import 'package:flutter_tools/src/base/logger.dart';
import 'package:flutter_tools/src/base/platform.dart';
import 'package:flutter_tools/src/build_info.dart';
import 'package:flutter_tools/src/build_system/build_system.dart';
import 'package:flutter_tools/src/build_system/targets/native_assets.dart';
import 'package:flutter_tools/src/dart/package_map.dart';
import 'package:flutter_tools/src/features.dart';
import 'package:flutter_tools/src/globals.dart' as globals;
import 'package:flutter_tools/src/isolated/native_assets/native_assets.dart';
import 'package:native_assets_cli/code_assets_builder.dart' hide BuildMode;
import 'package:native_assets_cli/native_assets_cli_internal.dart' as native_assets_cli;
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
    fileSystem = MemoryFileSystem.test(style: FileSystemStyle.windows);
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

  for (final bool flutterTester in <bool>[false, true]) {
    String testName = '';
    if (flutterTester) {
      testName += ' flutter tester';
    }
    for (final BuildMode buildMode in <BuildMode>[
      BuildMode.debug,
      if (!flutterTester) BuildMode.release,
    ]) {
      if (flutterTester && !const LocalPlatform().isWindows) {
        // When calling [runFlutterSpecificDartBuild] with the flutter tester
        // target platform, it will perform a build for the local machine. That
        // means e.g. running this test on MacOS will cause it to run a MacOS
        // build - which in return requires a special [ProcessManager] that can
        // simulate output of `otool` invocations.
        continue;
      }

      testUsingContext(
        'build with assets $buildMode$testName',
        overrides: <Type, Generator>{
          FeatureFlags: () => TestFeatureFlags(isNativeAssetsEnabled: true),
          ProcessManager: () => FakeProcessManager.empty(),
        },
        () async {
          final File packageConfig = environment.projectDir
              .childDirectory('.dart_tool')
              .childFile('package_config.json');
          final Uri nonFlutterTesterAssetUri =
              environment.buildDir.childFile(InstallCodeAssets.nativeAssetsFilename).uri;
          await packageConfig.parent.create();
          await packageConfig.create();
          final File dylibAfterCompiling = fileSystem.file('bar.dll');
          // The mock doesn't create the file, so create it here.
          await dylibAfterCompiling.create();

          final List<CodeAsset> codeAssets = <CodeAsset>[
            CodeAsset(
              package: 'bar',
              name: 'bar.dart',
              linkMode: DynamicLoadingBundled(),
              os: OS.windows,
              architecture: Architecture.x64,
              file: dylibAfterCompiling.uri,
            ),
          ];
          final FakeFlutterNativeAssetsBuildRunner buildRunner = FakeFlutterNativeAssetsBuildRunner(
            packagesWithNativeAssetsResult: <Package>[Package('bar', projectUri)],
            buildResult: FakeFlutterNativeAssetsBuilderResult.fromAssets(codeAssets: codeAssets),
            linkResult:
                buildMode == BuildMode.debug
                    ? null
                    : FakeFlutterNativeAssetsBuilderResult.fromAssets(codeAssets: codeAssets),
          );
          final Map<String, String> environmentDefines = <String, String>{
            kBuildMode: buildMode.cliName,
          };
          final TargetPlatform targetPlatform =
              flutterTester ? TargetPlatform.tester : TargetPlatform.windows_x64;
          final DartBuildResult dartBuildResult = await runFlutterSpecificDartBuild(
            environmentDefines: environmentDefines,
            targetPlatform: targetPlatform,
            projectUri: projectUri,
            fileSystem: fileSystem,
            buildRunner: buildRunner,
          );
          final String expectedDirectory =
              flutterTester ? native_assets_cli.OS.current.toString() : 'windows';
          final Uri nativeAssetsFileUri =
              flutterTester
                  ? projectUri.resolve(
                    'build/native_assets/$expectedDirectory/${InstallCodeAssets.nativeAssetsFilename}',
                  )
                  : nonFlutterTesterAssetUri;
          await installCodeAssets(
            dartBuildResult: dartBuildResult,
            environmentDefines: environmentDefines,
            targetPlatform: targetPlatform,
            projectUri: projectUri,
            fileSystem: fileSystem,
            nativeAssetsFileUri: nativeAssetsFileUri,
          );
          final String expectedOS = flutterTester ? OS.current.toString() : 'windows';
          final String expectedArch = flutterTester ? Architecture.current.toString() : 'x64';
          expect(
            (globals.logger as BufferLogger).traceText,
            stringContainsInOrder(<String>[
              'Building native assets for $expectedOS $expectedArch $buildMode.',
              'Building native assets for $expectedOS $expectedArch $buildMode done.',
            ]),
          );
          expect(
            await fileSystem.file(nativeAssetsFileUri).readAsString(),
            stringContainsInOrder(<String>[
              'package:bar/bar.dart',
              if (flutterTester)
                    // Tests run on host system, so the have the full path on the system.
                    projectUri
                    .resolve('build/native_assets/$expectedDirectory/bar.dll')
                    .toFilePath()
                    .replaceAll(r'\', r'\\') // Undo JSON string escaping.
              else
                // Apps are a bundle with the dylibs on their dlopen path.
                'bar.dll',
            ]),
          );
          expect(buildRunner.buildInvocations, 1);
          expect(buildRunner.linkInvocations, buildMode == BuildMode.release ? 1 : 0);
        },
      );
    }
  }

  // This logic is mocked in the other tests to avoid having test order
  // randomization causing issues with what processes are invoked.
  // Exercise the parsing of the process output in this separate test.
  testUsingContext(
    'NativeAssetsBuildRunnerImpl.cCompilerConfig',
    overrides: <Type, Generator>{
      FeatureFlags: () => TestFeatureFlags(isNativeAssetsEnabled: true),
      ProcessManager:
          () => FakeProcessManager.list(<FakeCommand>[
            FakeCommand(
              command: <Pattern>[
                RegExp(r'(.*)vswhere.exe'),
                '-format',
                'json',
                '-products',
                '*',
                '-utf8',
                '-latest',
                '-version',
                '16',
                '-requires',
                'Microsoft.VisualStudio.Workload.NativeDesktop',
                'Microsoft.VisualStudio.Component.VC.Tools.x86.x64',
                'Microsoft.VisualStudio.Component.VC.CMake.Project',
              ],
              stdout: r'''
[
  {
    "instanceId": "491ec752",
    "installDate": "2023-04-21T08:17:11Z",
    "installationName": "VisualStudio/17.5.4+33530.505",
    "installationPath": "C:\\Program Files\\Microsoft Visual Studio\\2022\\Community",
    "installationVersion": "17.5.33530.505",
    "productId": "Microsoft.VisualStudio.Product.Community",
    "productPath": "C:\\Program Files\\Microsoft Visual Studio\\2022\\Community\\Common7\\IDE\\devenv.exe",
    "state": 4294967295,
    "isComplete": true,
    "isLaunchable": true,
    "isPrerelease": false,
    "isRebootRequired": false,
    "displayName": "Visual Studio Community 2022",
    "description": "Powerful IDE, free for students, open-source contributors, and individuals",
    "channelId": "VisualStudio.17.Release",
    "channelUri": "https://aka.ms/vs/17/release/channel",
    "enginePath": "C:\\Program Files (x86)\\Microsoft Visual Studio\\Installer\\resources\\app\\ServiceHub\\Services\\Microsoft.VisualStudio.Setup.Service",
    "installedChannelId": "VisualStudio.17.Release",
    "installedChannelUri": "https://aka.ms/vs/17/release/channel",
    "releaseNotes": "https://docs.microsoft.com/en-us/visualstudio/releases/2022/release-notes-v17.5#17.5.4",
    "thirdPartyNotices": "https://go.microsoft.com/fwlink/?LinkId=661288",
    "updateDate": "2023-04-21T08:17:11.2249473Z",
    "catalog": {
      "buildBranch": "d17.5",
      "buildVersion": "17.5.33530.505",
      "id": "VisualStudio/17.5.4+33530.505",
      "localBuild": "build-lab",
      "manifestName": "VisualStudio",
      "manifestType": "installer",
      "productDisplayVersion": "17.5.4",
      "productLine": "Dev17",
      "productLineVersion": "2022",
      "productMilestone": "RTW",
      "productMilestoneIsPreRelease": "False",
      "productName": "Visual Studio",
      "productPatchVersion": "4",
      "productPreReleaseMilestoneSuffix": "1.0",
      "productSemanticVersion": "17.5.4+33530.505",
      "requiredEngineVersion": "3.5.2150.18781"
    },
    "properties": {
      "campaignId": "2060:abb99c5d1ecc4013acf2e1814b10b690",
      "channelManifestId": "VisualStudio.17.Release/17.5.4+33530.505",
      "nickname": "",
      "setupEngineFilePath": "C:\\Program Files (x86)\\Microsoft Visual Studio\\Installer\\setup.exe"
    }
  }
]
''', // Newline at the end of the string.
            ),
          ]),
      FileSystem: () => fileSystem,
    },
    () async {
      if (!const LocalPlatform().isWindows) {
        return;
      }

      final Directory msvcBinDir = fileSystem.directory(
        r'C:\Program Files\Microsoft Visual Studio\2022\Community\VC\Tools\MSVC\14.35.32215\bin\Hostx64\x64',
      );
      await msvcBinDir.create(recursive: true);

      final File packageConfigFile = fileSystem
          .directory(projectUri)
          .childDirectory('.dart_tool')
          .childFile('package_config.json');
      await packageConfigFile.parent.create();
      await packageConfigFile.create();
      final PackageConfig packageConfig = await loadPackageConfigWithLogging(
        packageConfigFile,
        logger: environment.logger,
      );
      final FlutterNativeAssetsBuildRunner runner = FlutterNativeAssetsBuildRunnerImpl(
        projectUri,
        packageConfigFile.path,
        packageConfig,
        fileSystem,
        logger,
      );
      final CCompilerConfig result = await runner.cCompilerConfig;
      expect(result.compiler?.toFilePath(), msvcBinDir.childFile('cl.exe').uri.toFilePath());
      expect(result.archiver?.toFilePath(), msvcBinDir.childFile('lib.exe').uri.toFilePath());
      expect(result.linker?.toFilePath(), msvcBinDir.childFile('link.exe').uri.toFilePath());
      expect(result.envScript, isNotNull);
      expect(result.envScriptArgs, isNotNull);
    },
  );
}
