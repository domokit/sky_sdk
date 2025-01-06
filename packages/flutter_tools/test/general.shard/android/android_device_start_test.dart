// Copyright 2014 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

import 'package:file/memory.dart';
import 'package:flutter_tools/src/android/android_device.dart';
import 'package:flutter_tools/src/android/android_sdk.dart';
import 'package:flutter_tools/src/android/application_package.dart';
import 'package:flutter_tools/src/base/file_system.dart';
import 'package:flutter_tools/src/base/logger.dart';
import 'package:flutter_tools/src/base/platform.dart';
import 'package:flutter_tools/src/build_info.dart';
import 'package:flutter_tools/src/device.dart';
import 'package:test/fake.dart';

import '../../src/common.dart';
import '../../src/fake_process_manager.dart';

const FakeCommand kAdbVersionCommand = FakeCommand(
  command: <String>['adb', 'version'],
  stdout: 'Android Debug Bridge version 1.0.39',
);

const FakeCommand kStartServer = FakeCommand(command: <String>['adb', 'start-server']);

const FakeCommand kShaCommand = FakeCommand(
  command: <String>[
    'adb',
    '-s',
    '1234',
    'shell',
    'echo',
    '-n',
    '',
    '>',
    '/data/local/tmp/sky.FlutterApp.sha1',
  ],
);

void main() {
  late FileSystem fileSystem;
  late FakeProcessManager processManager;
  late AndroidSdk androidSdk;

  setUp(() {
    processManager = FakeProcessManager.empty();
    fileSystem = MemoryFileSystem.test();
    androidSdk = FakeAndroidSdk();
  });

  for (final TargetPlatform targetPlatform in <TargetPlatform>[
    TargetPlatform.android_arm,
    TargetPlatform.android_arm64,
    TargetPlatform.android_x64,
  ]) {
    testWithoutContext('AndroidDevice.startApp allows release builds on $targetPlatform', () async {
      final String arch = getAndroidArchForName(getNameForTargetPlatform(targetPlatform)).archName;
      final AndroidDevice device = AndroidDevice(
        '1234',
        modelID: 'TestModel',
        fileSystem: fileSystem,
        processManager: processManager,
        logger: BufferLogger.test(),
        platform: FakePlatform(),
        androidSdk: androidSdk,
      );
      final File apkFile = fileSystem.file('app-debug.apk')..createSync();
      final AndroidApk apk = AndroidApk(
        id: 'FlutterApp',
        applicationPackage: apkFile,
        launchActivity: 'FlutterActivity',
        versionCode: 1,
      );

      processManager.addCommand(kAdbVersionCommand);
      processManager.addCommand(kStartServer);

      // This configures the target platform of the device.
      processManager.addCommand(
        FakeCommand(
          command: const <String>['adb', '-s', '1234', 'shell', 'getprop'],
          stdout: '[ro.product.cpu.abi]: [$arch]',
        ),
      );
      processManager.addCommand(
        const FakeCommand(
          command: <String>['adb', '-s', '1234', 'shell', 'am', 'force-stop', 'FlutterApp'],
        ),
      );
      processManager.addCommand(
        const FakeCommand(
          command: <String>['adb', '-s', '1234', 'install', '-t', '-r', 'app-debug.apk'],
        ),
      );
      processManager.addCommand(kShaCommand);
      processManager.addCommand(
        const FakeCommand(
          command: <String>[
            'adb',
            '-s',
            '1234',
            'shell',
            'am',
            'start',
            '-a',
            'android.intent.action.MAIN',
            '-c',
            'android.intent.category.LAUNCHER',
            '-f',
            '0x20000000',
            '--ez',
            'enable-dart-profiling',
            'true',
            'FlutterActivity',
          ],
        ),
      );

      final LaunchResult launchResult = await device.startApp(
        apk,
        prebuiltApplication: true,
        debuggingOptions: DebuggingOptions.disabled(BuildInfo.release),
        platformArgs: <String, dynamic>{},
      );

      expect(launchResult.started, true);
      expect(processManager, hasNoRemainingExpectations);
    });
  }

  testWithoutContext('AndroidDevice.startApp does not allow release builds on x86', () async {
    final AndroidDevice device = AndroidDevice(
      '1234',
      modelID: 'TestModel',
      fileSystem: fileSystem,
      processManager: processManager,
      logger: BufferLogger.test(),
      platform: FakePlatform(),
      androidSdk: androidSdk,
    );
    final File apkFile = fileSystem.file('app-debug.apk')..createSync();
    final AndroidApk apk = AndroidApk(
      id: 'FlutterApp',
      applicationPackage: apkFile,
      launchActivity: 'FlutterActivity',
      versionCode: 1,
    );

    processManager.addCommand(kAdbVersionCommand);
    processManager.addCommand(kStartServer);

    // This configures the target platform of the device.
    processManager.addCommand(
      const FakeCommand(
        command: <String>['adb', '-s', '1234', 'shell', 'getprop'],
        stdout: '[ro.product.cpu.abi]: [x86]',
      ),
    );

    final LaunchResult launchResult = await device.startApp(
      apk,
      prebuiltApplication: true,
      debuggingOptions: DebuggingOptions.disabled(BuildInfo.release),
      platformArgs: <String, dynamic>{},
    );

    expect(launchResult.started, false);
    expect(processManager, hasNoRemainingExpectations);
  });

  testWithoutContext('AndroidDevice.startApp forwards all supported debugging options', () async {
    final AndroidDevice device = AndroidDevice(
      '1234',
      modelID: 'TestModel',
      fileSystem: fileSystem,
      processManager: processManager,
      logger: BufferLogger.test(),
      platform: FakePlatform(),
      androidSdk: androidSdk,
    );
    final File apkFile = fileSystem.file('app-debug.apk')..createSync();
    final AndroidApk apk = AndroidApk(
      id: 'FlutterApp',
      applicationPackage: apkFile,
      launchActivity: 'FlutterActivity',
      versionCode: 1,
    );

    // These commands are required to install and start the app
    processManager.addCommand(kAdbVersionCommand);
    processManager.addCommand(kStartServer);
    processManager.addCommand(
      const FakeCommand(command: <String>['adb', '-s', '1234', 'shell', 'getprop']),
    );
    processManager.addCommand(
      const FakeCommand(
        command: <String>[
          'adb',
          '-s',
          '1234',
          'shell',
          'am',
          'force-stop',
          '--user',
          '10',
          'FlutterApp',
        ],
      ),
    );
    processManager.addCommand(
      const FakeCommand(
        command: <String>[
          'adb',
          '-s',
          '1234',
          'install',
          '-t',
          '-r',
          '--user',
          '10',
          'app-debug.apk',
        ],
        stdout: '\n\nThe Dart VM service is listening on http://127.0.0.1:456\n\n',
      ),
    );
    processManager.addCommand(kShaCommand);
    processManager.addCommand(
      const FakeCommand(
        command: <String>['adb', '-s', '1234', 'shell', '-x', 'logcat', '-v', 'time'],
      ),
    );

    // This command contains all launch arguments.
    processManager.addCommand(
      const FakeCommand(
        command: <String>[
          'adb',
          '-s',
          '1234',
          'shell',
          'am',
          'start',
          '-a',
          'android.intent.action.MAIN',
          '-c',
          'android.intent.category.LAUNCHER',
          '-f',
          '0x20000000',
          // The DebuggingOptions arguments go here.
          '--ez', 'enable-dart-profiling', 'true',
          '--ez', 'enable-software-rendering', 'true',
          '--ez', 'skia-deterministic-rendering', 'true',
          '--ez', 'trace-skia', 'true',
          '--es', 'trace-allowlist', 'bar,baz',
          '--es', 'trace-skia-allowlist', 'skia.a,skia.b',
          '--ez', 'trace-systrace', 'true',
          '--es', 'trace-to-file', 'path/to/trace.binpb',
          '--ez', 'endless-trace-buffer', 'true',
          '--ez', 'dump-skp-on-shader-compilation', 'true',
          '--ez', 'cache-sksl', 'true',
          '--ez', 'purge-persistent-cache', 'true',
          '--ez', 'enable-impeller', 'true',
          '--ez', 'enable-checked-mode', 'true',
          '--ez', 'verify-entry-points', 'true',
          '--ez', 'start-paused', 'true',
          '--ez', 'disable-service-auth-codes', 'true',
          '--es', 'dart-flags', 'foo,--null_assertions',
          '--ez', 'use-test-fonts', 'true',
          '--ez', 'verbose-logging', 'true',
          '--user', '10',
          'FlutterActivity',
        ],
      ),
    );

    final LaunchResult launchResult = await device.startApp(
      apk,
      prebuiltApplication: true,
      debuggingOptions: DebuggingOptions.enabled(
        BuildInfo.debug,
        startPaused: true,
        disableServiceAuthCodes: true,
        dartFlags: 'foo',
        enableSoftwareRendering: true,
        skiaDeterministicRendering: true,
        traceSkia: true,
        traceAllowlist: 'bar,baz',
        traceSkiaAllowlist: 'skia.a,skia.b',
        traceSystrace: true,
        traceToFile: 'path/to/trace.binpb',
        endlessTraceBuffer: true,
        dumpSkpOnShaderCompilation: true,
        cacheSkSL: true,
        purgePersistentCache: true,
        useTestFonts: true,
        verboseSystemLogs: true,
        enableImpeller: ImpellerStatus.enabled,
        nullAssertions: true,
      ),
      platformArgs: <String, dynamic>{},
      userIdentifier: '10',
    );

    // This fails to start due to VM Service discovery issues.
    expect(launchResult.started, false);
    expect(processManager, hasNoRemainingExpectations);
  });
}

class FakeAndroidSdk extends Fake implements AndroidSdk {
  @override
  String get adbPath => 'adb';

  @override
  bool get licensesAvailable => false;
}
