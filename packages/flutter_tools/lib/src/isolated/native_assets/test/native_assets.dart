// Copyright 2014 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

// Logic for native assets shared between all host OSes.

import 'package:native_assets_cli/native_assets_cli.dart';

import '../../../base/platform.dart';
import '../../../build_info.dart';
import '../../../globals.dart' as globals;
import '../../../native_assets.dart';
import '../../../project.dart';
import '../native_assets.dart';

class TestCompilerNativeAssetsBuilderImpl implements TestCompilerNativeAssetsBuilder {
  const TestCompilerNativeAssetsBuilderImpl();

  @override
  Future<Uri?> build(BuildInfo buildInfo) => testCompilerBuildNativeAssets(buildInfo);

  @override
  String windowsBuildDirectory(FlutterProject project) =>
      nativeAssetsBuildUri(project.directory.uri, OS.windows).toFilePath();
}

Future<Uri?> testCompilerBuildNativeAssets(BuildInfo buildInfo) async {
  if (!buildInfo.buildNativeAssets) {
    return null;
  }
  final Uri projectUri = FlutterProject.current().directory.uri;
  final FlutterNativeAssetsBuildRunner buildRunner = FlutterNativeAssetsBuildRunnerImpl(
    projectUri,
    buildInfo.packageConfigPath,
    buildInfo.packageConfig,
    globals.fs,
    globals.logger,
  );

  if (!globals.platform.isMacOS && !globals.platform.isLinux && !globals.platform.isWindows) {
    await ensureNoNativeAssetsOrOsIsSupported(
      projectUri,
      const LocalPlatform().operatingSystem,
      globals.fs,
      buildRunner,
    );
    return null;
  }

  // Only `flutter test` uses the
  // `build/native_assets/<os>/native_assets.json` file which uses absolute
  // paths to the shared libraries.
  final OS targetOS = getNativeOSFromTargetPlatform(TargetPlatform.tester);
  final Uri buildUri = nativeAssetsBuildUri(projectUri, targetOS);
  final Uri nativeAssetsFileUri = buildUri.resolve('native_assets.json');

  final Map<String, String> environmentDefines = <String, String>{
    kBuildMode: buildInfo.mode.cliName,
  };

  // First perform the dart build.
  final DartBuildResult dartBuildResult = await runFlutterSpecificDartBuild(
    environmentDefines: environmentDefines,
    buildRunner: buildRunner,
    targetPlatform: TargetPlatform.tester,
    projectUri: projectUri,
    fileSystem: globals.fs,
  );

  // Then "install" the code assets so they can be used at runtime.
  await installCodeAssets(
    dartBuildResult: dartBuildResult,
    environmentDefines: environmentDefines,
    targetPlatform: TargetPlatform.tester,
    projectUri: projectUri,
    fileSystem: globals.fs,
    nativeAssetsFileUri: nativeAssetsFileUri,
  );
  assert(await globals.fs.file(nativeAssetsFileUri).exists());

  return nativeAssetsFileUri;
}
