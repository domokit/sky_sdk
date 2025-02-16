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
import 'package:flutter_tools/src/build_system/targets/windows.dart';

import '../../../src/common.dart';
import '../../../src/context.dart';

void main() {
  testWithoutContext(
    'UnpackWindows copies files to the correct windows/ cache directory',
    () async {
      final Artifacts artifacts = Artifacts.test();
      final FileSystem fileSystem = MemoryFileSystem.test(style: FileSystemStyle.windows);
      final Environment environment = Environment.test(
        fileSystem.currentDirectory,
        artifacts: artifacts,
        processManager: FakeProcessManager.any(),
        fileSystem: fileSystem,
        logger: BufferLogger.test(),
        defines: <String, String>{kBuildMode: 'debug'},
      );
      environment.buildDir.createSync(recursive: true);

      final String windowsDesktopPath = artifacts.getArtifactPath(
        Artifact.windowsDesktopPath,
        platform: TargetPlatform.windows_x64,
        mode: BuildMode.debug,
      );
      final String windowsCppClientWrapper = artifacts.getArtifactPath(
        Artifact.windowsCppClientWrapper,
        platform: TargetPlatform.windows_x64,
        mode: BuildMode.debug,
      );
      final String icuData = artifacts.getArtifactPath(
        Artifact.icuData,
        platform: TargetPlatform.windows_x64,
      );
      final List<String> requiredFiles = <String>[
        '$windowsDesktopPath\\flutter_export.h',
        '$windowsDesktopPath\\flutter_messenger.h',
        '$windowsDesktopPath\\flutter_windows.dll',
        '$windowsDesktopPath\\flutter_windows.dll.exp',
        '$windowsDesktopPath\\flutter_windows.dll.lib',
        '$windowsDesktopPath\\flutter_windows.dll.pdb',
        '$windowsDesktopPath\\flutter_plugin_registrar.h',
        '$windowsDesktopPath\\flutter_texture_registrar.h',
        '$windowsDesktopPath\\flutter_windows.h',
        icuData,
        '$windowsCppClientWrapper\\foo',
        r'C:\packages\flutter_tools\lib\src\build_system\targets\windows.dart',
      ];

      for (final String path in requiredFiles) {
        fileSystem.file(path).createSync(recursive: true);
      }
      fileSystem.directory('windows').createSync();

      await const UnpackWindows(TargetPlatform.windows_x64).build(environment);

      // Output files are copied correctly.
      expect(fileSystem.file(r'C:\windows\flutter\ephemeral\flutter_export.h'), exists);
      expect(fileSystem.file(r'C:\windows\flutter\ephemeral\flutter_messenger.h'), exists);
      expect(fileSystem.file(r'C:\windows\flutter\ephemeral\flutter_windows.dll'), exists);
      expect(fileSystem.file(r'C:\windows\flutter\ephemeral\flutter_windows.dll.exp'), exists);
      expect(fileSystem.file(r'C:\windows\flutter\ephemeral\flutter_windows.dll.lib'), exists);
      expect(fileSystem.file(r'C:\windows\flutter\ephemeral\flutter_windows.dll.pdb'), exists);
      expect(fileSystem.file(r'C:\windows\flutter\ephemeral\flutter_export.h'), exists);
      expect(fileSystem.file(r'C:\windows\flutter\ephemeral\flutter_messenger.h'), exists);
      expect(fileSystem.file(r'C:\windows\flutter\ephemeral\flutter_plugin_registrar.h'), exists);
      expect(fileSystem.file(r'C:\windows\flutter\ephemeral\flutter_texture_registrar.h'), exists);
      expect(fileSystem.file(r'C:\windows\flutter\ephemeral\flutter_windows.h'), exists);
      expect(fileSystem.file('C:\\windows\\flutter\\ephemeral\\$icuData'), exists);
      expect(
        fileSystem.file('C:\\windows\\flutter\\ephemeral\\$windowsCppClientWrapper\\foo'),
        exists,
      );

      final File outputDepfile = environment.buildDir.childFile('windows_engine_sources.d');

      // Depfile is created correctly.
      expect(outputDepfile, exists);

      final List<String> inputPaths =
          environment.depFileService
              .parse(outputDepfile)
              .inputs
              .map((File file) => file.path)
              .toList();
      final List<String> outputPaths =
          environment.depFileService
              .parse(outputDepfile)
              .outputs
              .map((File file) => file.path)
              .toList();

      // Depfile has expected sources.
      expect(
        inputPaths,
        unorderedEquals(<String>[
          '$windowsDesktopPath\\flutter_export.h',
          '$windowsDesktopPath\\flutter_messenger.h',
          '$windowsDesktopPath\\flutter_windows.dll',
          '$windowsDesktopPath\\flutter_windows.dll.exp',
          '$windowsDesktopPath\\flutter_windows.dll.lib',
          '$windowsDesktopPath\\flutter_windows.dll.pdb',
          '$windowsDesktopPath\\flutter_plugin_registrar.h',
          '$windowsDesktopPath\\flutter_texture_registrar.h',
          '$windowsDesktopPath\\flutter_windows.h',
          icuData,
          '$windowsCppClientWrapper\\foo',
        ]),
      );
      expect(
        outputPaths,
        unorderedEquals(<String>[
          r'C:\windows\flutter\ephemeral\flutter_export.h',
          r'C:\windows\flutter\ephemeral\flutter_messenger.h',
          r'C:\windows\flutter\ephemeral\flutter_windows.dll',
          r'C:\windows\flutter\ephemeral\flutter_windows.dll.exp',
          r'C:\windows\flutter\ephemeral\flutter_windows.dll.lib',
          r'C:\windows\flutter\ephemeral\flutter_windows.dll.pdb',
          r'C:\windows\flutter\ephemeral\flutter_plugin_registrar.h',
          r'C:\windows\flutter\ephemeral\flutter_texture_registrar.h',
          r'C:\windows\flutter\ephemeral\flutter_windows.h',
          'C:\\windows\\flutter\\ephemeral\\$icuData',
          'C:\\windows\\flutter\\ephemeral\\$windowsCppClientWrapper\\foo',
        ]),
      );
    },
  );

  // AssetBundleFactory still uses context injection
  late FileSystem fileSystem;

  setUp(() {
    fileSystem = MemoryFileSystem.test(style: FileSystemStyle.windows);
  });

  testUsingContext(
    'DebugBundleWindowsAssets creates correct bundle structure',
    () async {
      final Environment environment = Environment.test(
        fileSystem.currentDirectory,
        artifacts: Artifacts.test(),
        processManager: FakeProcessManager.any(),
        fileSystem: fileSystem,
        logger: BufferLogger.test(),
        defines: <String, String>{kBuildMode: 'debug'},
        engineVersion: '2',
      );

      environment.buildDir.childFile('app.dill').createSync(recursive: true);
      environment.buildDir.childFile('native_assets.json').createSync(recursive: true);

      await const DebugBundleWindowsAssets(TargetPlatform.windows_x64).build(environment);

      // Depfile is created and dill is copied.
      expect(environment.buildDir.childFile('flutter_assets.d'), exists);
      expect(fileSystem.file(r'C:\flutter_assets\kernel_blob.bin'), exists);
      expect(fileSystem.file(r'C:\flutter_assets\AssetManifest.json'), exists);
    },
    overrides: <Type, Generator>{
      FileSystem: () => fileSystem,
      ProcessManager: () => FakeProcessManager.any(),
    },
  );

  testUsingContext(
    'ProfileBundleWindowsAssets creates correct bundle structure',
    () async {
      final Environment environment = Environment.test(
        fileSystem.currentDirectory,
        artifacts: Artifacts.test(),
        processManager: FakeProcessManager.any(),
        fileSystem: fileSystem,
        logger: BufferLogger.test(),
        defines: <String, String>{kBuildMode: 'profile'},
      );

      environment.buildDir.childFile('app.so').createSync(recursive: true);
      environment.buildDir.childFile('native_assets.json').createSync(recursive: true);

      await const WindowsAotBundle(AotElfProfile(TargetPlatform.windows_x64)).build(environment);
      await const ProfileBundleWindowsAssets(TargetPlatform.windows_x64).build(environment);

      // Depfile is created and so is copied.
      expect(environment.buildDir.childFile('flutter_assets.d'), exists);
      expect(fileSystem.file(r'C:\windows\app.so'), exists);
      expect(fileSystem.file(r'C:\flutter_assets\kernel_blob.bin').existsSync(), false);
      expect(fileSystem.file(r'C:\flutter_assets\AssetManifest.json'), exists);
    },
    overrides: <Type, Generator>{
      FileSystem: () => fileSystem,
      ProcessManager: () => FakeProcessManager.any(),
    },
  );

  testUsingContext(
    'ReleaseBundleWindowsAssets creates correct bundle structure',
    () async {
      final Environment environment = Environment.test(
        fileSystem.currentDirectory,
        artifacts: Artifacts.test(),
        processManager: FakeProcessManager.any(),
        fileSystem: fileSystem,
        logger: BufferLogger.test(),
        defines: <String, String>{kBuildMode: 'release'},
      );

      environment.buildDir.childFile('app.so').createSync(recursive: true);
      environment.buildDir.childFile('native_assets.json').createSync(recursive: true);

      await const WindowsAotBundle(AotElfRelease(TargetPlatform.windows_x64)).build(environment);
      await const ReleaseBundleWindowsAssets(TargetPlatform.windows_x64).build(environment);

      // Depfile is created and so is copied.
      expect(environment.buildDir.childFile('flutter_assets.d'), exists);
      expect(fileSystem.file(r'C:\windows\app.so'), exists);
      expect(fileSystem.file(r'C:\flutter_assets\kernel_blob.bin').existsSync(), false);
      expect(fileSystem.file(r'C:\flutter_assets\AssetManifest.json'), exists);
    },
    overrides: <Type, Generator>{
      FileSystem: () => fileSystem,
      ProcessManager: () => FakeProcessManager.any(),
    },
  );
}
