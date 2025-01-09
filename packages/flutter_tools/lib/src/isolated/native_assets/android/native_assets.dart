// Copyright 2014 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

import 'package:native_assets_builder/native_assets_builder.dart';
import 'package:native_assets_cli/code_assets_builder.dart';

import '../../../android/android_sdk.dart';
import '../../../android/gradle_utils.dart';
import '../../../base/common.dart';
import '../../../base/file_system.dart';
import '../../../build_info.dart';

int targetAndroidNdkApi(Map<String, String> environmentDefines) {
  return int.parse(environmentDefines[kMinSdkVersion] ?? minSdkVersion);
}

Future<void> copyNativeCodeAssetsAndroid(
  Uri buildUri,
  Map<CodeAsset, KernelAsset> assetTargetLocations,
  FileSystem fileSystem,
) async {
  assert(assetTargetLocations.isNotEmpty);
  final List<String> jniArchDirs = <String>[
    for (final AndroidArch androidArch in AndroidArch.values) androidArch.archName,
  ];
  for (final String jniArchDir in jniArchDirs) {
    final Uri archUri = buildUri.resolve('jniLibs/lib/$jniArchDir/');
    await fileSystem.directory(archUri).create(recursive: true);
  }
  for (final MapEntry<CodeAsset, KernelAsset> assetMapping in assetTargetLocations.entries) {
    final Uri source = assetMapping.key.file!;
    final Uri target = (assetMapping.value.path as KernelAssetAbsolutePath).uri;
    final AndroidArch androidArch = _getAndroidArch(assetMapping.value.target.architecture);
    final String jniArchDir = androidArch.archName;
    final Uri archUri = buildUri.resolve('jniLibs/lib/$jniArchDir/');
    final Uri targetUri = archUri.resolveUri(target);
    final String targetFullPath = targetUri.toFilePath();
    await fileSystem.file(source).copy(targetFullPath);
  }
}

/// Get the [Architecture] for [androidArch].
Architecture getNativeAndroidArchitecture(AndroidArch androidArch) {
  return switch (androidArch) {
    AndroidArch.armeabi_v7a => Architecture.arm,
    AndroidArch.arm64_v8a => Architecture.arm64,
    AndroidArch.x86 => Architecture.ia32,
    AndroidArch.x86_64 => Architecture.x64,
  };
}

/// Get the [AndroidArch] for [architecture].
AndroidArch _getAndroidArch(Architecture architecture) {
  return switch (architecture) {
    Architecture.arm => AndroidArch.armeabi_v7a,
    Architecture.arm64 => AndroidArch.arm64_v8a,
    Architecture.ia32 => AndroidArch.x86,
    Architecture.x64 => AndroidArch.x86_64,
    Architecture.riscv64 => throwToolExit('Android RISC-V not yet supported.'),
    _ => throwToolExit('Invalid architecture: $architecture.'),
  };
}

Map<CodeAsset, KernelAsset> assetTargetLocationsAndroid(List<CodeAsset> nativeAssets) {
  return <CodeAsset, KernelAsset>{
    for (final CodeAsset asset in nativeAssets) asset: _targetLocationAndroid(asset),
  };
}

/// Converts the `path` of [asset] as output from a `build.dart` invocation to
/// the path used inside the Flutter app bundle.
KernelAsset _targetLocationAndroid(CodeAsset asset) {
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
      kernelAssetPath = KernelAssetAbsolutePath(Uri(path: fileName));
    default:
      throw Exception('Unsupported asset link mode $linkMode in asset $asset');
  }
  return KernelAsset(
    id: asset.id,
    target: Target.fromArchitectureAndOS(asset.architecture!, asset.os),
    path: kernelAssetPath,
  );
}

/// Looks the NDK clang compiler tools.
///
/// Tool-exits if the NDK cannot be found.
///
/// Should only be invoked if a native assets build is performed. If the native
/// assets feature is disabled, or none of the packages have native assets, a
/// missing NDK is okay.
Future<CCompilerConfig> cCompilerConfigAndroid() async {
  final AndroidSdk? androidSdk = AndroidSdk.locateAndroidSdk();
  if (androidSdk == null) {
    throwToolExit('Android SDK could not be found.');
  }
  final Uri? compiler = _toOptionalFileUri(androidSdk.getNdkClangPath());
  final Uri? archiver = _toOptionalFileUri(androidSdk.getNdkArPath());
  final Uri? linker = _toOptionalFileUri(androidSdk.getNdkLdPath());
  if (compiler == null || archiver == null || linker == null) {
    throwToolExit('Android NDK Clang could not be found.');
  }
  final CCompilerConfig result = CCompilerConfig(
    compiler: compiler,
    archiver: archiver,
    linker: linker,
  );
  return result;
}

Uri? _toOptionalFileUri(String? string) {
  if (string == null) {
    return null;
  }
  return Uri.file(string);
}
