// Copyright 2014 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

import 'dart:async';

import 'package:file/memory.dart';
import 'package:flutter_tools/src/application_package.dart';
import 'package:flutter_tools/src/artifacts.dart';
import 'package:flutter_tools/src/base/file_system.dart';
import 'package:flutter_tools/src/base/logger.dart';
import 'package:flutter_tools/src/base/platform.dart';
import 'package:flutter_tools/src/build_info.dart';
import 'package:flutter_tools/src/build_system/build_system.dart';
import 'package:flutter_tools/src/bundle.dart';
import 'package:flutter_tools/src/bundle_builder.dart';
import 'package:flutter_tools/src/cache.dart';
import 'package:flutter_tools/src/device.dart';
import 'package:flutter_tools/src/preview_device.dart';
import 'package:flutter_tools/src/project.dart';
import 'package:meta/meta.dart';
import 'package:test/fake.dart';

import '../src/common.dart';
import '../src/context.dart';
import '../src/fakes.dart';

void main() {
  String? flutterRootBackup;
  late MemoryFileSystem fs;
  late File previewBinary;

  setUp(() {
    fs = MemoryFileSystem.test(style: FileSystemStyle.windows);
    Cache.flutterRoot = r'C:\path\to\flutter';
    previewBinary = fs.file(
      '${Cache.flutterRoot}\\bin\\cache\\artifacts\\flutter_preview\\flutter_preview.exe',
    );
    previewBinary.createSync(recursive: true);
    flutterRootBackup = Cache.flutterRoot;
  });

  tearDown(() {
    Cache.flutterRoot = flutterRootBackup;
  });

  testWithoutContext('PreviewDevice defaults', () async {
    final PreviewDevice device = PreviewDevice(
      artifacts: Artifacts.test(),
      fileSystem: fs,
      processManager: FakeProcessManager.any(),
      previewBinary: previewBinary,
      logger: BufferLogger.test(),
    );

    expect(await device.isLocalEmulator, false);
    expect(device.name, 'Preview');
    expect(await device.sdkNameAndVersion, 'preview');
    expect(await device.targetPlatform, TargetPlatform.windows_x64);
    expect(device.category, Category.desktop);
    expect(device.ephemeral, false);
    expect(device.id, 'preview');

    expect(device.isSupported(), true);
    expect(device.isSupportedForProject(FakeFlutterProject()), true);
    expect(await device.isLatestBuildInstalled(FakeApplicationPackage()), false);
    expect(await device.isAppInstalled(FakeApplicationPackage()), false);
    expect(await device.uninstallApp(FakeApplicationPackage()), true);
  });

  testUsingContext('Can build a simulator app', () async {
    final Completer<void> completer = Completer<void>();
    final BufferLogger logger = BufferLogger.test();
    final PreviewDevice device = PreviewDevice(
      artifacts: Artifacts.test(),
      fileSystem: fs,
      previewBinary: previewBinary,
      processManager: FakeProcessManager.list(<FakeCommand>[
        FakeCommand(
          command: const <String>[r'C:\.tmp_rand0\flutter_preview.rand0\flutter_preview.exe'],
          stdout: 'The Dart VM service is listening on http://127.0.0.1:64494/fZ_B2N6JRwY=/\n',
          completer: completer,
        ),
      ]),
      logger: logger,
      builderFactory: () => FakeBundleBuilder(fs),
    );
    final Directory previewDeviceCacheDir = fs.directory(
      'Artifact.windowsDesktopPath.TargetPlatform.windows_x64.debug',
    )..createSync(recursive: true);
    previewDeviceCacheDir.childFile('flutter_windows.dll').writeAsStringSync('1010101');
    previewDeviceCacheDir.childFile('icudtl.dat').writeAsStringSync('1010101');

    final LaunchResult result = await device.startApp(
      FakeApplicationPackage(),
      debuggingOptions: DebuggingOptions.enabled(BuildInfo.debug),
    );

    expect(result.started, true);
    expect(result.vmServiceUri, Uri.parse('http://127.0.0.1:64494/fZ_B2N6JRwY=/'));
  });

  group('PreviewDeviceDiscovery', () {
    late Artifacts artifacts;
    late ProcessManager processManager;
    final FakePlatform windowsPlatform = FakePlatform(operatingSystem: 'windows');
    final FakePlatform macPlatform = FakePlatform(operatingSystem: 'macos');
    final FakePlatform linuxPlatform = FakePlatform();
    final TestFeatureFlags featureFlags = TestFeatureFlags(isPreviewDeviceEnabled: true);

    setUp(() {
      artifacts = Artifacts.test(fileSystem: fs);
      processManager = FakeProcessManager.empty();
    });

    testWithoutContext('PreviewDeviceDiscovery on linux', () async {
      final PreviewDeviceDiscovery discovery = PreviewDeviceDiscovery(
        artifacts: artifacts,
        fileSystem: fs,
        logger: BufferLogger.test(),
        processManager: processManager,
        platform: linuxPlatform,
        featureFlags: featureFlags,
      );

      final List<Device> devices = await discovery.devices();

      expect(devices, isEmpty);
    });

    testWithoutContext('PreviewDeviceDiscovery on macOS', () async {
      final PreviewDeviceDiscovery discovery = PreviewDeviceDiscovery(
        artifacts: artifacts,
        fileSystem: fs,
        logger: BufferLogger.test(),
        processManager: processManager,
        platform: macPlatform,
        featureFlags: featureFlags,
      );

      final List<Device> devices = await discovery.devices();

      expect(devices, isEmpty);
    });

    testWithoutContext(
      'PreviewDeviceDiscovery on Windows returns preview when binary exists',
      () async {
        // ensure Flutter preview binary exists in cache.
        fs.file(artifacts.getArtifactPath(Artifact.flutterPreviewDevice)).writeAsBytesSync(<int>[
          1,
          0,
          0,
          1,
        ]);
        final PreviewDeviceDiscovery discovery = PreviewDeviceDiscovery(
          artifacts: artifacts,
          fileSystem: fs,
          logger: BufferLogger.test(),
          processManager: processManager,
          platform: windowsPlatform,
          featureFlags: featureFlags,
        );

        final List<Device> devices = await discovery.devices();

        expect(devices, hasLength(1));
        final Device previewDevice = devices.first;
        expect(previewDevice, isA<PreviewDevice>());
      },
    );

    testWithoutContext(
      'PreviewDeviceDiscovery on Windows returns nothing when binary does not exist',
      () async {
        final PreviewDeviceDiscovery discovery = PreviewDeviceDiscovery(
          artifacts: artifacts,
          fileSystem: fs,
          logger: BufferLogger.test(),
          processManager: processManager,
          platform: windowsPlatform,
          featureFlags: featureFlags,
        );

        final List<Device> devices = await discovery.devices();

        expect(devices, isEmpty);
      },
    );
  });
}

class FakeFlutterProject extends Fake implements FlutterProject {}

class FakeApplicationPackage extends Fake implements ApplicationPackage {}

class FakeBundleBuilder extends Fake implements BundleBuilder {
  FakeBundleBuilder(this.fileSystem);

  final FileSystem fileSystem;

  @override
  Future<void> build({
    required TargetPlatform platform,
    required BuildInfo buildInfo,
    FlutterProject? project,
    String? mainPath,
    String manifestPath = defaultManifestPath,
    String? applicationKernelFilePath,
    String? depfilePath,
    String? assetDirPath,
    bool buildNativeAssets = true,
    @visibleForTesting BuildSystem? buildSystem,
  }) async {
    final Directory assetDirectory = fileSystem
      .directory(assetDirPath)
      .childDirectory('flutter_assets')..createSync(recursive: true);
    assetDirectory.childFile('kernel_blob.bin').createSync();
  }
}
