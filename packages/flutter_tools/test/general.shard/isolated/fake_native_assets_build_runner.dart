// Copyright 2014 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

import 'package:flutter_tools/src/isolated/native_assets/native_assets.dart';
import 'package:native_assets_builder/native_assets_builder.dart';
import 'package:native_assets_cli/code_assets_builder.dart';

export 'package:native_assets_cli/code_assets_builder.dart' show CodeAsset, DynamicLoadingBundled;

/// Mocks all logic instead of using `package:native_assets_builder`, which
/// relies on doing process calls to `pub` and the local file system.
class FakeFlutterNativeAssetsBuildRunner implements FlutterNativeAssetsBuildRunner {
  FakeFlutterNativeAssetsBuildRunner({
    this.packagesWithNativeAssetsResult = const <String>[],
    this.onBuild,
    this.onLink,
    this.buildResult = const FakeFlutterNativeAssetsBuilderResult(),
    this.linkResult = const FakeFlutterNativeAssetsBuilderResult(),
    this.cCompilerConfigResult,
    this.ndkCCompilerConfigResult,
  });

  // TODO(dcharkes): Cleanup this fake https://github.com/flutter/flutter/issues/162061
  final BuildResult? Function(BuildInput)? onBuild;
  final LinkResult? Function(LinkInput)? onLink;
  final BuildResult? buildResult;
  final LinkResult? linkResult;
  final List<String> packagesWithNativeAssetsResult;
  final CCompilerConfig? cCompilerConfigResult;
  final CCompilerConfig? ndkCCompilerConfigResult;

  int buildInvocations = 0;
  int linkInvocations = 0;
  int packagesWithNativeAssetsInvocations = 0;

  @override
  Future<BuildResult?> build({
    required List<String> buildAssetTypes,
    required BuildInputValidator inputValidator,
    required BuildInputCreator inputCreator,
    required BuildValidator buildValidator,
    required ApplicationAssetValidator applicationAssetValidator,
    required Uri workingDirectory,
    required bool linkingEnabled,
  }) async {
    BuildResult? result = buildResult;
    for (final String package in packagesWithNativeAssetsResult) {
      final BuildInputBuilder configBuilder =
          inputCreator()
            ..setupShared(
              packageRoot: Uri.parse('$package/'),
              packageName: package,
              outputDirectory: Uri.parse('build-out-dir'),
              outputDirectoryShared: Uri.parse('build-out-dir-shared'),
              outputFile: Uri.file('output.json'),
            )
            ..setupBuildInput()
            ..config.setupShared(buildAssetTypes: buildAssetTypes)
            ..config.setupBuild(dryRun: false, linkingEnabled: linkingEnabled);
      final BuildInput buildConfig = BuildInput(configBuilder.json);
      if (onBuild != null) {
        result = onBuild!(buildConfig);
      }
      buildInvocations++;
    }
    return result;
  }

  @override
  Future<LinkResult?> link({
    required List<String> buildAssetTypes,
    required LinkInputCreator inputCreator,
    required LinkInputValidator inputValidator,
    required LinkValidator linkValidator,
    required ApplicationAssetValidator applicationAssetValidator,
    required Uri workingDirectory,
    required BuildResult buildResult,
  }) async {
    LinkResult? result = linkResult;
    for (final String package in packagesWithNativeAssetsResult) {
      final LinkInputBuilder configBuilder =
          inputCreator()
            ..setupShared(
              packageRoot: Uri.parse('$package/'),
              packageName: package,
              outputDirectory: Uri.parse('build-out-dir'),
              outputDirectoryShared: Uri.parse('build-out-dir-shared'),
              outputFile: Uri.file('output.json'),
            )
            ..setupLink(assets: buildResult.encodedAssets, recordedUsesFile: null)
            ..config.setupShared(buildAssetTypes: buildAssetTypes);
      final LinkInput buildConfig = LinkInput(configBuilder.json);
      if (onLink != null) {
        result = onLink!(buildConfig);
      }
      linkInvocations++;
    }
    return result;
  }

  @override
  Future<List<String>> packagesWithNativeAssets() async {
    packagesWithNativeAssetsInvocations++;
    return packagesWithNativeAssetsResult;
  }

  @override
  Future<CCompilerConfig?> get cCompilerConfig async => cCompilerConfigResult;

  @override
  Future<CCompilerConfig?> get ndkCCompilerConfig async => cCompilerConfigResult;
}

final class FakeFlutterNativeAssetsBuilderResult implements BuildResult, LinkResult {
  const FakeFlutterNativeAssetsBuilderResult({
    this.encodedAssets = const <EncodedAsset>[],
    this.encodedAssetsForLinking = const <String, List<EncodedAsset>>{},
    this.dependencies = const <Uri>[],
  });

  factory FakeFlutterNativeAssetsBuilderResult.fromAssets({
    List<CodeAsset> codeAssets = const <CodeAsset>[],
    Map<String, List<CodeAsset>> codeAssetsForLinking = const <String, List<CodeAsset>>{},
    List<Uri> dependencies = const <Uri>[],
  }) {
    return FakeFlutterNativeAssetsBuilderResult(
      encodedAssets: <EncodedAsset>[
        for (final CodeAsset codeAsset in codeAssets) codeAsset.encode(),
      ],
      encodedAssetsForLinking: <String, List<EncodedAsset>>{
        for (final String linkerName in codeAssetsForLinking.keys)
          linkerName: <EncodedAsset>[
            for (final CodeAsset codeAsset in codeAssetsForLinking[linkerName]!) codeAsset.encode(),
          ],
      },
      dependencies: dependencies,
    );
  }

  @override
  final List<EncodedAsset> encodedAssets;

  @override
  final Map<String, List<EncodedAsset>> encodedAssetsForLinking;

  @override
  final List<Uri> dependencies;
}
