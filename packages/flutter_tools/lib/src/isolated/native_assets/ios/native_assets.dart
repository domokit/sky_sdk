// Copyright 2014 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

import 'package:native_assets_builder/native_assets_builder.dart';
import 'package:native_assets_cli/code_assets_builder.dart';

import '../../../base/file_system.dart';
import '../../../build_info.dart';
import '../macos/native_assets_host.dart';

// TODO(dcharkes): Fetch minimum iOS version from somewhere. https://github.com/flutter/flutter/issues/145104
const int targetIOSVersion = 12;

IOSSdk getIOSSdk(EnvironmentType environmentType) {
  return switch (environmentType) {
    EnvironmentType.physical => IOSSdk.iPhoneOS,
    EnvironmentType.simulator => IOSSdk.iPhoneSimulator,
  };
}

/// Extract the [Architecture] from a [DarwinArch].
Architecture getNativeIOSArchitecture(DarwinArch darwinArch) {
  return switch (darwinArch) {
    DarwinArch.armv7 => Architecture.arm,
    DarwinArch.arm64 => Architecture.arm64,
    DarwinArch.x86_64 => Architecture.x64,
  };
}

Map<KernelAssetPath, List<CodeAsset>> fatAssetTargetLocationsIOS(List<CodeAsset> nativeAssets) {
  final Set<String> alreadyTakenNames = <String>{};
  final Map<KernelAssetPath, List<CodeAsset>> result = <KernelAssetPath, List<CodeAsset>>{};
  final Map<String, KernelAssetPath> idToPath = <String, KernelAssetPath>{};
  for (final CodeAsset asset in nativeAssets) {
    // Use same target path for all assets with the same id.
    final KernelAssetPath path =
        idToPath[asset.id] ?? _targetLocationIOS(asset, alreadyTakenNames).path;
    idToPath[asset.id] = path;
    result[path] ??= <CodeAsset>[];
    result[path]!.add(asset);
  }
  return result;
}

Map<CodeAsset, KernelAsset> assetTargetLocationsIOS(List<CodeAsset> nativeAssets) {
  final Set<String> alreadyTakenNames = <String>{};
  final Map<String, KernelAssetPath> idToPath = <String, KernelAssetPath>{};
  final Map<CodeAsset, KernelAsset> result = <CodeAsset, KernelAsset>{};
  for (final CodeAsset asset in nativeAssets) {
    final KernelAssetPath path =
        idToPath[asset.id] ?? _targetLocationIOS(asset, alreadyTakenNames).path;
    idToPath[asset.id] = path;
    result[asset] = KernelAsset(
      id: asset.id,
      target: Target.fromArchitectureAndOS(asset.architecture!, asset.os),
      path: path,
    );
  }
  return result;
}

KernelAsset _targetLocationIOS(CodeAsset asset, Set<String> alreadyTakenNames) {
  final LinkMode linkMode = asset.linkMode;
  final KernelAssetPath kernelAssetPath;
  switch (linkMode) {
    case DynamicLoadingSystem _:
      kernelAssetPath = KernelAssetSystemPath(linkMode.uri);
    case LookupInExecutable _:
      kernelAssetPath = KernelAssetInExecutable();
    case LookupInProcess _:
      kernelAssetPath = KernelAssetInProcess();
    case DynamicLoadingBundled _:
      final String fileName = asset.file!.pathSegments.last;
      kernelAssetPath = KernelAssetAbsolutePath(frameworkUri(fileName, alreadyTakenNames));
    default:
      throw Exception('Unsupported asset link mode $linkMode in asset $asset');
  }
  return KernelAsset(
    id: asset.id,
    target: Target.fromArchitectureAndOS(asset.architecture!, asset.os),
    path: kernelAssetPath,
  );
}

/// Copies native assets into a framework per dynamic library.
///
/// For `flutter run -release` a multi-architecture solution is needed. So,
/// `lipo` is used to combine all target architectures into a single file.
///
/// The install name is set so that it matches with the place it will
/// be bundled in the final app. Install names that are referenced in dependent
/// libraries are updated to match the new install name, so that the referenced
/// library can be found by the dynamic linker.
///
/// Code signing is also done here, so that it doesn't have to be done in
/// in xcode_backend.dart.
Future<void> copyNativeCodeAssetsIOS(
  Uri buildUri,
  Map<KernelAssetPath, List<CodeAsset>> assetTargetLocations,
  String? codesignIdentity,
  BuildMode buildMode,
  FileSystem fileSystem,
) async {
  assert(assetTargetLocations.isNotEmpty);
  final Map<String, String> oldToNewInstallNames = <String, String>{};
  final List<(File, String, Directory)> dylibs = <(File, String, Directory)>[];

  for (final MapEntry<KernelAssetPath, List<CodeAsset>> assetMapping
      in assetTargetLocations.entries) {
    final Uri target = (assetMapping.key as KernelAssetAbsolutePath).uri;
    final List<File> sources = <File>[
      for (final CodeAsset source in assetMapping.value) fileSystem.file(source.file),
    ];
    final Uri targetUri = buildUri.resolveUri(target);
    final File dylibFile = fileSystem.file(targetUri);
    final Directory frameworkDir = dylibFile.parent;
    if (!await frameworkDir.exists()) {
      await frameworkDir.create(recursive: true);
    }
    await lipoDylibs(dylibFile, sources);

    final String dylibFileName = dylibFile.basename;
    final String newInstallName = '@rpath/$dylibFileName.framework/$dylibFileName';
    final Set<String> oldInstallNames = await getInstallNamesDylib(dylibFile);
    for (final String oldInstallName in oldInstallNames) {
      oldToNewInstallNames[oldInstallName] = newInstallName;
    }
    dylibs.add((dylibFile, newInstallName, frameworkDir));

    // TODO(knopp): Wire the value once there is a way to configure that in the hook.
    // https://github.com/dart-lang/native/issues/1133
    await createInfoPlist(targetUri.pathSegments.last, frameworkDir, minimumIOSVersion: '12.0');
  }

  for (final (File dylibFile, String newInstallName, Directory frameworkDir) in dylibs) {
    await setInstallNamesDylib(dylibFile, newInstallName, oldToNewInstallNames);
    await codesignDylib(codesignIdentity, buildMode, frameworkDir);
  }
}
