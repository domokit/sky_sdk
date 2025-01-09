// Copyright 2014 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

import 'package:file/memory.dart';
import 'package:file_testing/file_testing.dart';
import 'package:flutter_tools/src/artifacts.dart';
import 'package:flutter_tools/src/base/file_system.dart';
import 'package:flutter_tools/src/base/logger.dart';
import 'package:flutter_tools/src/build_info.dart';
import 'package:flutter_tools/src/build_system/build_system.dart';
import 'package:flutter_tools/src/build_system/targets/common.dart';
import 'package:flutter_tools/src/build_system/targets/linux.dart';
import 'package:flutter_tools/src/convert.dart';

import '../../../src/common.dart';
import '../../../src/context.dart';

void main() {
  testWithoutContext(
    'Copies files to correct cache directory, excluding unrelated code on a x64 host',
    () async {
      final FileSystem fileSystem = MemoryFileSystem.test();
      final Artifacts artifacts = Artifacts.test();
      setUpCacheDirectory(fileSystem, artifacts);

      final Environment testEnvironment = Environment.test(
        fileSystem.currentDirectory,
        defines: <String, String>{kBuildMode: 'debug'},
        artifacts: artifacts,
        processManager: FakeProcessManager.any(),
        fileSystem: fileSystem,
        logger: BufferLogger.test(),
      );
      testEnvironment.buildDir.createSync(recursive: true);

      await const UnpackLinux(TargetPlatform.linux_x64).build(testEnvironment);

      expect(fileSystem.file('linux/flutter/ephemeral/libflutter_linux_gtk.so'), exists);
      expect(fileSystem.file('linux/flutter/ephemeral/unrelated-stuff'), isNot(exists));

      // Check if the target files are copied correctly.
      final String headersPathForX64 = artifacts.getArtifactPath(
        Artifact.linuxHeaders,
        platform: TargetPlatform.linux_x64,
        mode: BuildMode.debug,
      );
      final String headersPathForArm64 = artifacts.getArtifactPath(
        Artifact.linuxHeaders,
        platform: TargetPlatform.linux_arm64,
        mode: BuildMode.debug,
      );
      expect(fileSystem.file('linux/flutter/ephemeral/$headersPathForX64/foo.h'), exists);
      expect(fileSystem.file('linux/flutter/ephemeral/$headersPathForArm64/foo.h'), isNot(exists));

      final String icuDataPathForX64 = artifacts.getArtifactPath(
        Artifact.icuData,
        platform: TargetPlatform.linux_x64,
      );
      final String icuDataPathForArm64 = artifacts.getArtifactPath(
        Artifact.icuData,
        platform: TargetPlatform.linux_arm64,
      );
      expect(fileSystem.file('linux/flutter/ephemeral/$icuDataPathForX64'), exists);
      expect(fileSystem.file('linux/flutter/ephemeral/$icuDataPathForArm64'), isNot(exists));
    },
  );

  // This test is basically the same logic as the above test.
  // The difference is the target CPU architecture.
  testWithoutContext(
    'Copies files to correct cache directory, excluding unrelated code on a arm64 host',
    () async {
      final FileSystem fileSystem = MemoryFileSystem.test();
      final Artifacts artifacts = Artifacts.test();
      setUpCacheDirectory(fileSystem, artifacts);

      final Environment testEnvironment = Environment.test(
        fileSystem.currentDirectory,
        defines: <String, String>{kBuildMode: 'debug'},
        artifacts: artifacts,
        processManager: FakeProcessManager.any(),
        fileSystem: fileSystem,
        logger: BufferLogger.test(),
      );
      testEnvironment.buildDir.createSync(recursive: true);

      await const UnpackLinux(TargetPlatform.linux_arm64).build(testEnvironment);

      expect(fileSystem.file('linux/flutter/ephemeral/libflutter_linux_gtk.so'), exists);
      expect(fileSystem.file('linux/flutter/ephemeral/unrelated-stuff'), isNot(exists));

      // Check if the target files are copied correctly.
      final String headersPathForX64 = artifacts.getArtifactPath(
        Artifact.linuxHeaders,
        platform: TargetPlatform.linux_x64,
        mode: BuildMode.debug,
      );
      final String headersPathForArm64 = artifacts.getArtifactPath(
        Artifact.linuxHeaders,
        platform: TargetPlatform.linux_arm64,
        mode: BuildMode.debug,
      );
      expect(fileSystem.file('linux/flutter/ephemeral/$headersPathForX64/foo.h'), isNot(exists));
      expect(fileSystem.file('linux/flutter/ephemeral/$headersPathForArm64/foo.h'), exists);

      final String icuDataPathForX64 = artifacts.getArtifactPath(
        Artifact.icuData,
        platform: TargetPlatform.linux_x64,
      );
      final String icuDataPathForArm64 = artifacts.getArtifactPath(
        Artifact.icuData,
        platform: TargetPlatform.linux_arm64,
      );
      expect(fileSystem.file('linux/flutter/ephemeral/$icuDataPathForX64'), isNot(exists));
      expect(fileSystem.file('linux/flutter/ephemeral/$icuDataPathForArm64'), exists);
    },
  );

  // Only required for the test below that still depends on the context.
  late FileSystem fileSystem;

  setUp(() {
    fileSystem = MemoryFileSystem.test();
  });

  testUsingContext(
    'DebugBundleLinuxAssets copies artifacts to out directory',
    () async {
      final Environment testEnvironment = Environment.test(
        fileSystem.currentDirectory,
        defines: <String, String>{kBuildMode: 'debug', kBuildName: '2.0.0', kBuildNumber: '22'},
        inputs: <String, String>{kBundleSkSLPath: 'bundle.sksl'},
        artifacts: Artifacts.test(),
        processManager: FakeProcessManager.any(),
        fileSystem: fileSystem,
        logger: BufferLogger.test(),
        engineVersion: '2',
      );

      testEnvironment.buildDir.createSync(recursive: true);

      // Create input files.
      testEnvironment.buildDir.childFile('app.dill').createSync();
      testEnvironment.buildDir.childFile('native_assets.json').createSync();
      fileSystem
          .file('bundle.sksl')
          .writeAsStringSync(
            json.encode(<String, Object>{
              'engineRevision': '2',
              'platform': 'ios',
              'data': <String, Object>{'A': 'B'},
            }),
          );

      await const DebugBundleLinuxAssets(TargetPlatform.linux_x64).build(testEnvironment);

      final Directory output = testEnvironment.outputDir.childDirectory('flutter_assets');

      expect(output.childFile('kernel_blob.bin'), exists);
      expect(output.childFile('AssetManifest.json'), exists);
      expect(output.childFile('version.json'), exists);
      final String versionFile = output.childFile('version.json').readAsStringSync();
      expect(versionFile, contains('"version":"2.0.0"'));
      expect(versionFile, contains('"build_number":"22"'));
      // SkSL
      expect(output.childFile('io.flutter.shaders.json'), exists);
      expect(output.childFile('io.flutter.shaders.json').readAsStringSync(), '{"data":{"A":"B"}}');

      // No bundled fonts
      expect(output.childFile('FontManifest.json'), isNot(exists));
    },
    overrides: <Type, Generator>{
      FileSystem: () => fileSystem,
      ProcessManager: () => FakeProcessManager.any(),
    },
  );

  testWithoutContext("DebugBundleLinuxAssets' name depends on target platforms", () async {
    expect(
      const DebugBundleLinuxAssets(TargetPlatform.linux_x64).name,
      'debug_bundle_linux-x64_assets',
    );
    expect(
      const DebugBundleLinuxAssets(TargetPlatform.linux_arm64).name,
      'debug_bundle_linux-arm64_assets',
    );
  });

  testUsingContext(
    'ProfileBundleLinuxAssets copies artifacts to out directory',
    () async {
      final Environment testEnvironment = Environment.test(
        fileSystem.currentDirectory,
        defines: <String, String>{kBuildMode: 'profile'},
        artifacts: Artifacts.test(),
        processManager: FakeProcessManager.any(),
        fileSystem: fileSystem,
        logger: BufferLogger.test(),
      );

      testEnvironment.buildDir.createSync(recursive: true);

      // Create input files.
      testEnvironment.buildDir.childFile('app.so').createSync();
      testEnvironment.buildDir.childFile('native_assets.json').createSync();

      await const LinuxAotBundle(AotElfProfile(TargetPlatform.linux_x64)).build(testEnvironment);
      await const ProfileBundleLinuxAssets(TargetPlatform.linux_x64).build(testEnvironment);
      final Directory libDir = testEnvironment.outputDir.childDirectory('lib');
      final Directory assetsDir = testEnvironment.outputDir.childDirectory('flutter_assets');

      expect(libDir.childFile('libapp.so'), exists);
      expect(assetsDir.childFile('AssetManifest.json'), exists);
      expect(assetsDir.childFile('version.json'), exists);
      // No bundled fonts
      expect(assetsDir.childFile('FontManifest.json'), isNot(exists));
    },
    overrides: <Type, Generator>{
      FileSystem: () => fileSystem,
      ProcessManager: () => FakeProcessManager.any(),
    },
  );

  testWithoutContext("ProfileBundleLinuxAssets' name depends on target platforms", () async {
    expect(
      const ProfileBundleLinuxAssets(TargetPlatform.linux_x64).name,
      'profile_bundle_linux-x64_assets',
    );
    expect(
      const ProfileBundleLinuxAssets(TargetPlatform.linux_arm64).name,
      'profile_bundle_linux-arm64_assets',
    );
  });

  testUsingContext(
    'ReleaseBundleLinuxAssets copies artifacts to out directory',
    () async {
      final Environment testEnvironment = Environment.test(
        fileSystem.currentDirectory,
        defines: <String, String>{kBuildMode: 'release'},
        artifacts: Artifacts.test(),
        processManager: FakeProcessManager.any(),
        fileSystem: fileSystem,
        logger: BufferLogger.test(),
      );

      testEnvironment.buildDir.createSync(recursive: true);

      // Create input files.
      testEnvironment.buildDir.childFile('app.so').createSync();
      testEnvironment.buildDir.childFile('native_assets.json').createSync();

      await const LinuxAotBundle(AotElfRelease(TargetPlatform.linux_x64)).build(testEnvironment);
      await const ReleaseBundleLinuxAssets(TargetPlatform.linux_x64).build(testEnvironment);
      final Directory libDir = testEnvironment.outputDir.childDirectory('lib');
      final Directory assetsDir = testEnvironment.outputDir.childDirectory('flutter_assets');

      expect(libDir.childFile('libapp.so'), exists);
      expect(assetsDir.childFile('AssetManifest.json'), exists);
      expect(assetsDir.childFile('version.json'), exists);
      // No bundled fonts
      expect(assetsDir.childFile('FontManifest.json'), isNot(exists));
    },
    overrides: <Type, Generator>{
      FileSystem: () => fileSystem,
      ProcessManager: () => FakeProcessManager.any(),
    },
  );

  testWithoutContext("ReleaseBundleLinuxAssets' name depends on target platforms", () async {
    expect(
      const ReleaseBundleLinuxAssets(TargetPlatform.linux_x64).name,
      'release_bundle_linux-x64_assets',
    );
    expect(
      const ReleaseBundleLinuxAssets(TargetPlatform.linux_arm64).name,
      'release_bundle_linux-arm64_assets',
    );
  });
}

void setUpCacheDirectory(FileSystem fileSystem, Artifacts artifacts) {
  final String desktopPathForX64 = artifacts.getArtifactPath(
    Artifact.linuxDesktopPath,
    platform: TargetPlatform.linux_x64,
    mode: BuildMode.debug,
  );
  final String desktopPathForArm64 = artifacts.getArtifactPath(
    Artifact.linuxDesktopPath,
    platform: TargetPlatform.linux_arm64,
    mode: BuildMode.debug,
  );
  fileSystem.file('$desktopPathForX64/unrelated-stuff').createSync(recursive: true);
  fileSystem.file('$desktopPathForX64/libflutter_linux_gtk.so').createSync(recursive: true);
  fileSystem.file('$desktopPathForArm64/unrelated-stuff').createSync(recursive: true);
  fileSystem.file('$desktopPathForArm64/libflutter_linux_gtk.so').createSync(recursive: true);

  final String headersPathForX64 = artifacts.getArtifactPath(
    Artifact.linuxHeaders,
    platform: TargetPlatform.linux_x64,
    mode: BuildMode.debug,
  );
  final String headersPathForArm64 = artifacts.getArtifactPath(
    Artifact.linuxHeaders,
    platform: TargetPlatform.linux_arm64,
    mode: BuildMode.debug,
  );
  fileSystem.file('$headersPathForX64/foo.h').createSync(recursive: true);
  fileSystem.file('$headersPathForArm64/foo.h').createSync(recursive: true);

  fileSystem
      .file(artifacts.getArtifactPath(Artifact.icuData, platform: TargetPlatform.linux_x64))
      .createSync();
  fileSystem
      .file(artifacts.getArtifactPath(Artifact.icuData, platform: TargetPlatform.linux_arm64))
      .createSync();

  fileSystem
      .file('packages/flutter_tools/lib/src/build_system/targets/linux.dart')
      .createSync(recursive: true);
}
