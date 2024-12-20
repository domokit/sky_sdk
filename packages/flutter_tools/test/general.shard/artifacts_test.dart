// Copyright 2014 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

import 'package:file/memory.dart';
import 'package:flutter_tools/src/artifacts.dart';
import 'package:flutter_tools/src/base/file_system.dart';
import 'package:flutter_tools/src/base/logger.dart';
import 'package:flutter_tools/src/base/platform.dart';
import 'package:flutter_tools/src/build_info.dart';
import 'package:flutter_tools/src/cache.dart';

import '../src/common.dart';
import '../src/context.dart';
import '../src/fakes.dart';

void main() {
  group('CachedArtifacts', () {
    late CachedArtifacts artifacts;
    late Cache cache;
    late FileSystem fileSystem;
    late Platform platform;

    setUp(() {
      fileSystem = MemoryFileSystem.test();
      final Directory cacheRoot = fileSystem.directory('root')..createSync();
      platform = FakePlatform();
      cache = Cache(
        rootOverride: cacheRoot,
        fileSystem: fileSystem,
        platform: platform,
        logger: BufferLogger.test(),
        osUtils: FakeOperatingSystemUtils(),
        artifacts: <ArtifactSet>[],
      );
      artifacts = CachedArtifacts(
        fileSystem: fileSystem,
        cache: cache,
        platform: platform,
        operatingSystemUtils: FakeOperatingSystemUtils(),
      );
    });

    testWithoutContext('getArtifactPath', () {
      final String xcframeworkPath = artifacts.getArtifactPath(
        Artifact.flutterXcframework,
        platform: TargetPlatform.ios,
        mode: BuildMode.release,
      );
      expect(
        xcframeworkPath,
        fileSystem.path.join(
          'root',
          'bin',
          'cache',
          'artifacts',
          'engine',
          'ios-release',
          'Flutter.xcframework',
        ),
      );
      expect(
        () => artifacts.getArtifactPath(
          Artifact.flutterFramework,
          platform: TargetPlatform.ios,
          mode: BuildMode.release,
          environmentType: EnvironmentType.simulator,
        ),
        throwsToolExit(message: 'No xcframework found at $xcframeworkPath.'),
      );
      fileSystem.directory(xcframeworkPath).createSync(recursive: true);
      expect(
        () => artifacts.getArtifactPath(
          Artifact.flutterFramework,
          platform: TargetPlatform.ios,
          mode: BuildMode.release,
          environmentType: EnvironmentType.simulator,
        ),
        throwsToolExit(message: 'No iOS frameworks found in $xcframeworkPath'),
      );

      fileSystem
          .directory(xcframeworkPath)
          .childDirectory('ios-arm64_x86_64-simulator')
          .childDirectory('Flutter.framework')
          .createSync(recursive: true);
      fileSystem
          .directory(xcframeworkPath)
          .childDirectory('ios-arm64')
          .childDirectory('Flutter.framework')
          .createSync(recursive: true);

      expect(
        artifacts.getArtifactPath(
          Artifact.flutterFramework,
          platform: TargetPlatform.ios,
          mode: BuildMode.release,
          environmentType: EnvironmentType.simulator,
        ),
        fileSystem.path.join(xcframeworkPath, 'ios-arm64_x86_64-simulator', 'Flutter.framework'),
      );
      final String actualReleaseFrameworkArtifact = artifacts.getArtifactPath(
        Artifact.flutterFramework,
        platform: TargetPlatform.ios,
        mode: BuildMode.release,
        environmentType: EnvironmentType.physical,
      );
      final String expectedArm64ReleaseFrameworkArtifact = fileSystem.path.join(
        xcframeworkPath,
        'ios-arm64',
        'Flutter.framework',
      );
      expect(actualReleaseFrameworkArtifact, expectedArm64ReleaseFrameworkArtifact);
      expect(
        artifacts.getArtifactPath(
          Artifact.flutterXcframework,
          platform: TargetPlatform.ios,
          mode: BuildMode.release,
        ),
        fileSystem.path.join(
          'root',
          'bin',
          'cache',
          'artifacts',
          'engine',
          'ios-release',
          'Flutter.xcframework',
        ),
      );
      expect(
        artifacts.getArtifactPath(Artifact.flutterTester),
        fileSystem.path.join(
          'root',
          'bin',
          'cache',
          'artifacts',
          'engine',
          'linux-x64',
          'flutter_tester',
        ),
      );
      expect(
        artifacts.getArtifactPath(Artifact.flutterTester, platform: TargetPlatform.linux_arm64),
        fileSystem.path.join(
          'root',
          'bin',
          'cache',
          'artifacts',
          'engine',
          'linux-arm64',
          'flutter_tester',
        ),
      );
      expect(
        artifacts.getArtifactPath(Artifact.frontendServerSnapshotForEngineDartSdk),
        fileSystem.path.join(
          'root',
          'bin',
          'cache',
          'dart-sdk',
          'bin',
          'snapshots',
          'frontend_server_aot.dart.snapshot',
        ),
      );
    });

    testWithoutContext(
      'getArtifactPath for FlutterMacOS.framework and FlutterMacOS.xcframework',
      () {
        final String xcframeworkPath = fileSystem.path.join(
          'root',
          'bin',
          'cache',
          'artifacts',
          'engine',
          'darwin-x64-release',
          'FlutterMacOS.xcframework',
        );
        final String xcframeworkPathWithUnsetPlatform = fileSystem.path.join(
          'root',
          'bin',
          'cache',
          'artifacts',
          'engine',
          'linux-x64-release',
          'FlutterMacOS.xcframework',
        );
        expect(
          artifacts.getArtifactPath(
            Artifact.flutterMacOSXcframework,
            platform: TargetPlatform.darwin,
            mode: BuildMode.release,
          ),
          xcframeworkPath,
        );
        expect(
          artifacts.getArtifactPath(Artifact.flutterMacOSXcframework, mode: BuildMode.release),
          xcframeworkPathWithUnsetPlatform,
        );
        expect(
          () => artifacts.getArtifactPath(
            Artifact.flutterMacOSFramework,
            platform: TargetPlatform.darwin,
            mode: BuildMode.release,
          ),
          throwsToolExit(message: 'No xcframework found at $xcframeworkPath.'),
        );
        expect(
          () => artifacts.getArtifactPath(Artifact.flutterMacOSFramework, mode: BuildMode.release),
          throwsToolExit(message: 'No xcframework found at $xcframeworkPathWithUnsetPlatform.'),
        );
        fileSystem.directory(xcframeworkPath).createSync(recursive: true);
        expect(
          () => artifacts.getArtifactPath(
            Artifact.flutterMacOSFramework,
            platform: TargetPlatform.darwin,
            mode: BuildMode.release,
          ),
          throwsToolExit(message: 'No macOS frameworks found in $xcframeworkPath'),
        );
        fileSystem
            .directory(xcframeworkPath)
            .childDirectory('macos-arm64_x86_64')
            .childDirectory('FlutterMacOS.framework')
            .createSync(recursive: true);
        expect(
          artifacts.getArtifactPath(
            Artifact.flutterMacOSFramework,
            platform: TargetPlatform.darwin,
            mode: BuildMode.release,
          ),
          fileSystem.path.join(xcframeworkPath, 'macos-arm64_x86_64', 'FlutterMacOS.framework'),
        );
      },
    );

    testWithoutContext('Precompiled web AMD module system artifact paths are correct', () {
      expect(
        artifacts.getHostArtifact(HostArtifact.webPrecompiledAmdSdk).path,
        'root/bin/cache/flutter_web_sdk/kernel/amd/dart_sdk.js',
      );
      expect(
        artifacts.getHostArtifact(HostArtifact.webPrecompiledAmdSdkSourcemaps).path,
        'root/bin/cache/flutter_web_sdk/kernel/amd/dart_sdk.js.map',
      );
      expect(
        artifacts.getHostArtifact(HostArtifact.webPrecompiledAmdCanvaskitSdk).path,
        'root/bin/cache/flutter_web_sdk/kernel/amd-canvaskit/dart_sdk.js',
      );
      expect(
        artifacts.getHostArtifact(HostArtifact.webPrecompiledAmdCanvaskitSdkSourcemaps).path,
        'root/bin/cache/flutter_web_sdk/kernel/amd-canvaskit/dart_sdk.js.map',
      );
      expect(
        artifacts.getHostArtifact(HostArtifact.webPrecompiledAmdSoundSdk).path,
        'root/bin/cache/flutter_web_sdk/kernel/amd-sound/dart_sdk.js',
      );
      expect(
        artifacts.getHostArtifact(HostArtifact.webPrecompiledAmdSoundSdkSourcemaps).path,
        'root/bin/cache/flutter_web_sdk/kernel/amd-sound/dart_sdk.js.map',
      );
      expect(
        artifacts.getHostArtifact(HostArtifact.webPrecompiledAmdCanvaskitSoundSdk).path,
        'root/bin/cache/flutter_web_sdk/kernel/amd-canvaskit-sound/dart_sdk.js',
      );
      expect(
        artifacts.getHostArtifact(HostArtifact.webPrecompiledAmdCanvaskitSoundSdkSourcemaps).path,
        'root/bin/cache/flutter_web_sdk/kernel/amd-canvaskit-sound/dart_sdk.js.map',
      );
    });

    testWithoutContext('Precompiled web DDC module system artifact paths are correct', () {
      expect(
        artifacts.getHostArtifact(HostArtifact.webPrecompiledDdcSdk).path,
        'root/bin/cache/flutter_web_sdk/kernel/ddc/dart_sdk.js',
      );
      expect(
        artifacts.getHostArtifact(HostArtifact.webPrecompiledDdcSdkSourcemaps).path,
        'root/bin/cache/flutter_web_sdk/kernel/ddc/dart_sdk.js.map',
      );
      expect(
        artifacts.getHostArtifact(HostArtifact.webPrecompiledDdcCanvaskitSdk).path,
        'root/bin/cache/flutter_web_sdk/kernel/ddc-canvaskit/dart_sdk.js',
      );
      expect(
        artifacts.getHostArtifact(HostArtifact.webPrecompiledDdcCanvaskitSdkSourcemaps).path,
        'root/bin/cache/flutter_web_sdk/kernel/ddc-canvaskit/dart_sdk.js.map',
      );
      expect(
        artifacts.getHostArtifact(HostArtifact.webPrecompiledDdcSoundSdk).path,
        'root/bin/cache/flutter_web_sdk/kernel/ddc-sound/dart_sdk.js',
      );
      expect(
        artifacts.getHostArtifact(HostArtifact.webPrecompiledDdcSoundSdkSourcemaps).path,
        'root/bin/cache/flutter_web_sdk/kernel/ddc-sound/dart_sdk.js.map',
      );
      expect(
        artifacts.getHostArtifact(HostArtifact.webPrecompiledDdcCanvaskitSoundSdk).path,
        'root/bin/cache/flutter_web_sdk/kernel/ddc-canvaskit-sound/dart_sdk.js',
      );
      expect(
        artifacts.getHostArtifact(HostArtifact.webPrecompiledDdcCanvaskitSoundSdkSourcemaps).path,
        'root/bin/cache/flutter_web_sdk/kernel/ddc-canvaskit-sound/dart_sdk.js.map',
      );
    });

    testWithoutContext('getEngineType', () {
      expect(artifacts.getEngineType(TargetPlatform.android_arm, BuildMode.debug), 'android-arm');
      expect(artifacts.getEngineType(TargetPlatform.ios, BuildMode.release), 'ios-release');
      expect(artifacts.getEngineType(TargetPlatform.darwin), 'darwin-x64');
    });

    testWithoutContext(
      'CachedLocalWebSdkArtifacts wrapping a versioned engine sets usesLocalArtifacts',
      () {
        final CachedLocalWebSdkArtifacts webArtifacts = CachedLocalWebSdkArtifacts(
          parent: artifacts,
          webSdkPath: fileSystem.path.join(fileSystem.currentDirectory.path, 'out', 'wasm_release'),
          fileSystem: fileSystem,
          platform: platform,
          operatingSystemUtils: FakeOperatingSystemUtils(),
        );
        expect(webArtifacts.usesLocalArtifacts, true);
      },
    );
  });

  group('LocalEngineArtifacts', () {
    late Artifacts artifacts;
    late Cache cache;
    late FileSystem fileSystem;
    late Platform platform;

    setUp(() {
      fileSystem = MemoryFileSystem.test();
      final Directory cacheRoot = fileSystem.directory('root')..createSync();
      platform = FakePlatform();
      cache = Cache(
        rootOverride: cacheRoot,
        fileSystem: fileSystem,
        platform: platform,
        logger: BufferLogger.test(),
        osUtils: FakeOperatingSystemUtils(),
        artifacts: <ArtifactSet>[],
      );
      artifacts = CachedLocalWebSdkArtifacts(
        parent: CachedLocalEngineArtifacts(
          fileSystem.path.join(fileSystem.currentDirectory.path, 'out', 'host_debug_unopt'),
          engineOutPath: fileSystem.path.join(
            fileSystem.currentDirectory.path,
            'out',
            'android_debug_unopt',
          ),
          cache: cache,
          fileSystem: fileSystem,
          platform: platform,
          processManager: FakeProcessManager.any(),
          operatingSystemUtils: FakeOperatingSystemUtils(),
        ),
        webSdkPath: fileSystem.path.join(fileSystem.currentDirectory.path, 'out', 'wasm_release'),
        fileSystem: fileSystem,
        platform: platform,
        operatingSystemUtils: FakeOperatingSystemUtils(),
      );
    });

    testWithoutContext(
      'getArtifactPath for FlutterMacOS.framework and FlutterMacOS.xcframework',
      () {
        final String xcframeworkPath = fileSystem.path.join(
          '/out',
          'android_debug_unopt',
          'FlutterMacOS.xcframework',
        );
        expect(
          artifacts.getArtifactPath(
            Artifact.flutterMacOSXcframework,
            platform: TargetPlatform.darwin,
            mode: BuildMode.release,
          ),
          xcframeworkPath,
        );
        expect(
          artifacts.getArtifactPath(Artifact.flutterMacOSXcframework, mode: BuildMode.release),
          xcframeworkPath,
        );
        expect(
          () => artifacts.getArtifactPath(
            Artifact.flutterMacOSFramework,
            platform: TargetPlatform.darwin,
            mode: BuildMode.release,
          ),
          throwsToolExit(
            message: 'No xcframework found at /out/android_debug_unopt/FlutterMacOS.xcframework',
          ),
        );
        expect(
          () => artifacts.getArtifactPath(Artifact.flutterMacOSFramework, mode: BuildMode.release),
          throwsToolExit(
            message: 'No xcframework found at /out/android_debug_unopt/FlutterMacOS.xcframework',
          ),
        );
        fileSystem.directory(xcframeworkPath).createSync(recursive: true);
        expect(
          () => artifacts.getArtifactPath(
            Artifact.flutterMacOSFramework,
            platform: TargetPlatform.darwin,
            mode: BuildMode.release,
          ),
          throwsToolExit(
            message:
                'No macOS frameworks found in /out/android_debug_unopt/FlutterMacOS.xcframework',
          ),
        );
        expect(
          () => artifacts.getArtifactPath(Artifact.flutterMacOSFramework, mode: BuildMode.release),
          throwsToolExit(
            message:
                'No macOS frameworks found in /out/android_debug_unopt/FlutterMacOS.xcframework',
          ),
        );

        fileSystem
            .directory(xcframeworkPath)
            .childDirectory('macos-arm64_x86_64')
            .childDirectory('FlutterMacOS.framework')
            .createSync(recursive: true);

        expect(
          artifacts.getArtifactPath(
            Artifact.flutterMacOSFramework,
            platform: TargetPlatform.darwin,
            mode: BuildMode.release,
          ),
          fileSystem.path.join(xcframeworkPath, 'macos-arm64_x86_64', 'FlutterMacOS.framework'),
        );
        expect(
          artifacts.getArtifactPath(Artifact.flutterMacOSFramework, mode: BuildMode.release),
          fileSystem.path.join(xcframeworkPath, 'macos-arm64_x86_64', 'FlutterMacOS.framework'),
        );
      },
    );

    testWithoutContext('getArtifactPath', () {
      final String xcframeworkPath = artifacts.getArtifactPath(
        Artifact.flutterXcframework,
        platform: TargetPlatform.ios,
        mode: BuildMode.release,
      );
      expect(
        xcframeworkPath,
        fileSystem.path.join('/out', 'android_debug_unopt', 'Flutter.xcframework'),
      );
      expect(
        () => artifacts.getArtifactPath(
          Artifact.flutterFramework,
          platform: TargetPlatform.ios,
          mode: BuildMode.release,
          environmentType: EnvironmentType.simulator,
        ),
        throwsToolExit(
          message: 'No xcframework found at /out/android_debug_unopt/Flutter.xcframework',
        ),
      );
      fileSystem.directory(xcframeworkPath).createSync(recursive: true);
      expect(
        () => artifacts.getArtifactPath(
          Artifact.flutterFramework,
          platform: TargetPlatform.ios,
          mode: BuildMode.release,
          environmentType: EnvironmentType.simulator,
        ),
        throwsToolExit(
          message: 'No iOS frameworks found in /out/android_debug_unopt/Flutter.xcframework',
        ),
      );

      fileSystem
          .directory(xcframeworkPath)
          .childDirectory('ios-arm64_x86_64-simulator')
          .childDirectory('Flutter.framework')
          .createSync(recursive: true);
      fileSystem
          .directory(xcframeworkPath)
          .childDirectory('ios-arm64')
          .childDirectory('Flutter.framework')
          .createSync(recursive: true);
      fileSystem
          .directory('out')
          .childDirectory('host_debug_unopt')
          .childDirectory('dart-sdk')
          .childDirectory('bin')
          .createSync(recursive: true);

      expect(
        artifacts.getArtifactPath(
          Artifact.flutterFramework,
          platform: TargetPlatform.ios,
          mode: BuildMode.release,
          environmentType: EnvironmentType.simulator,
        ),
        fileSystem.path.join(xcframeworkPath, 'ios-arm64_x86_64-simulator', 'Flutter.framework'),
      );
      expect(
        artifacts.getArtifactPath(
          Artifact.flutterFramework,
          platform: TargetPlatform.ios,
          mode: BuildMode.release,
          environmentType: EnvironmentType.physical,
        ),
        fileSystem.path.join(xcframeworkPath, 'ios-arm64', 'Flutter.framework'),
      );
      expect(
        artifacts.getArtifactPath(
          Artifact.flutterXcframework,
          platform: TargetPlatform.ios,
          mode: BuildMode.release,
        ),
        fileSystem.path.join('/out', 'android_debug_unopt', 'Flutter.xcframework'),
      );
      expect(
        artifacts.getArtifactPath(Artifact.flutterTester),
        fileSystem.path.join('/out', 'android_debug_unopt', 'flutter_tester'),
      );
      expect(
        artifacts.getArtifactPath(Artifact.engineDartSdkPath),
        fileSystem.path.join('/out', 'host_debug_unopt', 'dart-sdk'),
      );
      expect(
        artifacts.getArtifactPath(Artifact.frontendServerSnapshotForEngineDartSdk),
        fileSystem.path.join(
          '/out',
          'host_debug_unopt',
          'dart-sdk',
          'bin',
          'snapshots',
          'frontend_server_aot.dart.snapshot',
        ),
      );

      expect(
        artifacts.getArtifactPath(
          Artifact.flutterPreviewDevice,
          platform: TargetPlatform.windows_x64,
        ),
        fileSystem.path.join(
          'root',
          'bin',
          'cache',
          'artifacts',
          'flutter_preview',
          'flutter_preview.exe',
        ),
      );

      fileSystem
          .file(fileSystem.path.join('/out', 'host_debug_unopt', 'impellerc'))
          .createSync(recursive: true);
      fileSystem
          .file(fileSystem.path.join('/out', 'host_debug_unopt', 'libtessellator.so'))
          .createSync(recursive: true);

      expect(
        artifacts.getHostArtifact(HostArtifact.impellerc).path,
        fileSystem.path.join('/out', 'host_debug_unopt', 'impellerc'),
      );
      expect(
        artifacts.getHostArtifact(HostArtifact.libtessellator).path,
        fileSystem.path.join('/out', 'host_debug_unopt', 'libtessellator.so'),
      );
    });

    testWithoutContext(
      'falls back to bundled impeller artifacts if the files do not exist in the local engine',
      () {
        expect(
          artifacts.getHostArtifact(HostArtifact.impellerc).path,
          fileSystem.path.join(
            'root',
            'bin',
            'cache',
            'artifacts',
            'engine',
            'linux-x64',
            'impellerc',
          ),
        );
        expect(
          artifacts.getHostArtifact(HostArtifact.libtessellator).path,
          fileSystem.path.join(
            'root',
            'bin',
            'cache',
            'artifacts',
            'engine',
            'linux-x64',
            'libtessellator.so',
          ),
        );
      },
    );

    testWithoutContext('uses prebuilt dart sdk for web platform', () {
      final String failureMessage =
          'Unable to find a prebuilt dart sdk at:'
          ' "${fileSystem.path.join('/flutter', 'prebuilts', 'linux-x64', 'dart-sdk')}"';

      expect(
        () => artifacts.getArtifactPath(
          Artifact.frontendServerSnapshotForEngineDartSdk,
          platform: TargetPlatform.web_javascript,
        ),
        throwsToolExit(message: failureMessage),
      );
      expect(
        () => artifacts.getArtifactPath(
          Artifact.engineDartSdkPath,
          platform: TargetPlatform.web_javascript,
        ),
        throwsToolExit(message: failureMessage),
      );
      expect(
        () => artifacts.getArtifactPath(
          Artifact.engineDartBinary,
          platform: TargetPlatform.web_javascript,
        ),
        throwsToolExit(message: failureMessage),
      );

      fileSystem
          .directory('flutter')
          .childDirectory('prebuilts')
          .childDirectory('linux-x64')
          .childDirectory('dart-sdk')
          .childDirectory('bin')
          .createSync(recursive: true);

      expect(
        artifacts.getArtifactPath(
          Artifact.frontendServerSnapshotForEngineDartSdk,
          platform: TargetPlatform.web_javascript,
        ),
        fileSystem.path.join(
          '/flutter',
          'prebuilts',
          'linux-x64',
          'dart-sdk',
          'bin',
          'snapshots',
          'frontend_server_aot.dart.snapshot',
        ),
      );
      expect(
        artifacts.getArtifactPath(
          Artifact.engineDartSdkPath,
          platform: TargetPlatform.web_javascript,
        ),
        fileSystem.path.join('/flutter', 'prebuilts', 'linux-x64', 'dart-sdk'),
      );
      expect(
        artifacts.getArtifactPath(
          Artifact.engineDartBinary,
          platform: TargetPlatform.web_javascript,
        ),
        fileSystem.path.join('/flutter', 'prebuilts', 'linux-x64', 'dart-sdk', 'bin', 'dart'),
      );
      expect(
        artifacts.getArtifactPath(
          Artifact.engineDartAotRuntime,
          platform: TargetPlatform.web_javascript,
        ),
        fileSystem.path.join(
          '/flutter',
          'prebuilts',
          'linux-x64',
          'dart-sdk',
          'bin',
          'dartaotruntime',
        ),
      );
    });

    testWithoutContext('getEngineType', () {
      expect(
        artifacts.getEngineType(TargetPlatform.android_arm, BuildMode.debug),
        'android_debug_unopt',
      );
      expect(artifacts.getEngineType(TargetPlatform.ios, BuildMode.release), 'android_debug_unopt');
      expect(artifacts.getEngineType(TargetPlatform.darwin), 'android_debug_unopt');
    });

    testWithoutContext('Looks up dart.exe on windows platforms', () async {
      artifacts = CachedLocalWebSdkArtifacts(
        parent: CachedLocalEngineArtifacts(
          fileSystem.path.join(fileSystem.currentDirectory.path, 'out', 'host_debug_unopt'),
          engineOutPath: fileSystem.path.join(
            fileSystem.currentDirectory.path,
            'out',
            'android_debug_unopt',
          ),
          cache: cache,
          fileSystem: fileSystem,
          platform: FakePlatform(operatingSystem: 'windows'),
          processManager: FakeProcessManager.any(),
          operatingSystemUtils: FakeOperatingSystemUtils(),
        ),
        webSdkPath: fileSystem.path.join(fileSystem.currentDirectory.path, 'out', 'wasm_release'),
        fileSystem: fileSystem,
        platform: FakePlatform(operatingSystem: 'windows'),
        operatingSystemUtils: FakeOperatingSystemUtils(),
      );

      fileSystem
          .directory('out')
          .childDirectory('host_debug_unopt')
          .childDirectory('dart-sdk')
          .childDirectory('bin')
          .createSync(recursive: true);

      expect(
        artifacts.getArtifactPath(Artifact.engineDartBinary),
        fileSystem.path.join('/out', 'host_debug_unopt', 'dart-sdk', 'bin', 'dart.exe'),
      );
    });

    testWithoutContext('Looks up dart on linux platforms', () async {
      fileSystem
          .directory('/out')
          .childDirectory('host_debug_unopt')
          .childDirectory('dart-sdk')
          .childDirectory('bin')
          .createSync(recursive: true);

      expect(
        artifacts.getArtifactPath(Artifact.engineDartBinary),
        fileSystem.path.join('/out', 'host_debug_unopt', 'dart-sdk', 'bin', 'dart'),
      );
    });

    testWithoutContext('Finds dart-sdk in windows prebuilts for web platform', () async {
      artifacts = CachedLocalWebSdkArtifacts(
        parent: CachedLocalEngineArtifacts(
          fileSystem.path.join(fileSystem.currentDirectory.path, 'out', 'host_debug_unopt'),
          engineOutPath: fileSystem.path.join(
            fileSystem.currentDirectory.path,
            'out',
            'android_debug_unopt',
          ),
          cache: cache,
          fileSystem: fileSystem,
          platform: FakePlatform(operatingSystem: 'windows'),
          processManager: FakeProcessManager.any(),
          operatingSystemUtils: FakeOperatingSystemUtils(),
        ),
        webSdkPath: fileSystem.path.join(fileSystem.currentDirectory.path, 'out', 'wasm_release'),
        fileSystem: fileSystem,
        platform: FakePlatform(operatingSystem: 'windows'),
        operatingSystemUtils: FakeOperatingSystemUtils(),
      );

      fileSystem
          .directory('/flutter')
          .childDirectory('prebuilts')
          .childDirectory('windows-x64')
          .childDirectory('dart-sdk')
          .childDirectory('bin')
          .createSync(recursive: true);

      expect(
        artifacts.getArtifactPath(
          Artifact.engineDartBinary,
          platform: TargetPlatform.web_javascript,
        ),
        fileSystem.path.join('/flutter', 'prebuilts', 'windows-x64', 'dart-sdk', 'bin', 'dart.exe'),
      );
    });

    testWithoutContext('Finds dart-sdk in macos prebuilts for web platform', () async {
      artifacts = CachedLocalWebSdkArtifacts(
        parent: CachedLocalEngineArtifacts(
          fileSystem.path.join(fileSystem.currentDirectory.path, 'out', 'host_debug_unopt'),
          engineOutPath: fileSystem.path.join(
            fileSystem.currentDirectory.path,
            'out',
            'android_debug_unopt',
          ),
          cache: cache,
          fileSystem: fileSystem,
          platform: FakePlatform(operatingSystem: 'macos'),
          processManager: FakeProcessManager.any(),
          operatingSystemUtils: FakeOperatingSystemUtils(),
        ),
        webSdkPath: fileSystem.path.join(fileSystem.currentDirectory.path, 'out', 'wasm_release'),
        fileSystem: fileSystem,
        platform: FakePlatform(operatingSystem: 'macos'),
        operatingSystemUtils: FakeOperatingSystemUtils(),
      );

      fileSystem
          .directory('/flutter')
          .childDirectory('prebuilts')
          .childDirectory('macos-x64')
          .childDirectory('dart-sdk')
          .childDirectory('bin')
          .createSync(recursive: true);

      expect(
        artifacts.getArtifactPath(
          Artifact.engineDartBinary,
          platform: TargetPlatform.web_javascript,
        ),
        fileSystem.path.join('/flutter', 'prebuilts', 'macos-x64', 'dart-sdk', 'bin', 'dart'),
      );
    });
  });

  group('LocalEngineInfo', () {
    late FileSystem fileSystem;
    late LocalEngineInfo localEngineInfo;

    setUp(() {
      fileSystem = MemoryFileSystem.test();
    });

    testUsingContext(
      'determines the target device name from the path',
      () {
        localEngineInfo = LocalEngineInfo(
          targetOutPath: fileSystem.path.join(
            fileSystem.currentDirectory.path,
            'out',
            'android_debug_unopt',
          ),
          hostOutPath: fileSystem.path.join(
            fileSystem.currentDirectory.path,
            'out',
            'host_debug_unopt',
          ),
        );

        expect(localEngineInfo.localTargetName, 'android_debug_unopt');
      },
      overrides: <Type, Generator>{
        FileSystem: () => fileSystem,
        ProcessManager: () => FakeProcessManager.any(),
      },
    );

    testUsingContext(
      'determines the target device name from the path when using a custom engine path',
      () {
        localEngineInfo = LocalEngineInfo(
          targetOutPath: fileSystem.path.join(
            fileSystem.currentDirectory.path,
            'out',
            'android_debug_unopt',
          ),
          hostOutPath: fileSystem.path.join(
            fileSystem.currentDirectory.path,
            'out',
            'host_debug_unopt',
          ),
        );

        expect(localEngineInfo.localHostName, 'host_debug_unopt');
      },
      overrides: <Type, Generator>{
        FileSystem: () => fileSystem,
        ProcessManager: () => FakeProcessManager.any(),
      },
    );
  });
}
