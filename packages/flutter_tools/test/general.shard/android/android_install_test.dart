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
import 'package:test/fake.dart';

import '../../src/common.dart';
import '../../src/fake_process_manager.dart';

const FakeCommand kAdbVersionCommand = FakeCommand(
  command: <String>['adb', 'version'],
  stdout: 'Android Debug Bridge version 1.0.39',
);
const FakeCommand kAdbStartServerCommand = FakeCommand(command: <String>['adb', 'start-server']);
const FakeCommand kInstallCommand = FakeCommand(
  command: <String>['adb', '-s', '1234', 'install', '-t', '-r', '--user', '10', 'app-debug.apk'],
);
const FakeCommand kStoreShaCommand = FakeCommand(
  command: <String>[
    'adb',
    '-s',
    '1234',
    'shell',
    'echo',
    '-n',
    '',
    '>',
    '/data/local/tmp/sky.app.sha1',
  ],
);

void main() {
  late FileSystem fileSystem;
  late BufferLogger logger;

  setUp(() {
    fileSystem = MemoryFileSystem.test();
    logger = BufferLogger.test();
  });

  AndroidDevice setUpAndroidDevice({AndroidSdk? androidSdk, ProcessManager? processManager}) {
    androidSdk ??= FakeAndroidSdk();
    return AndroidDevice(
      '1234',
      modelID: 'TestModel',
      logger: logger,
      platform: FakePlatform(),
      androidSdk: androidSdk,
      fileSystem: fileSystem,
      processManager: processManager ?? FakeProcessManager.any(),
    );
  }

  testWithoutContext('Cannot install app on API level below 16', () async {
    final FakeProcessManager processManager = FakeProcessManager.list(<FakeCommand>[
      kAdbVersionCommand,
      kAdbStartServerCommand,
      const FakeCommand(
        command: <String>['adb', '-s', '1234', 'shell', 'getprop'],
        stdout: '[ro.build.version.sdk]: [11]',
      ),
    ]);
    final File apk = fileSystem.file('app-debug.apk')..createSync();
    final AndroidApk androidApk = AndroidApk(
      applicationPackage: apk,
      id: 'app',
      versionCode: 22,
      launchActivity: 'Main',
    );
    final AndroidDevice androidDevice = setUpAndroidDevice(processManager: processManager);

    expect(await androidDevice.installApp(androidApk), false);
    expect(processManager, hasNoRemainingExpectations);
  });

  testWithoutContext('Cannot install app if APK file is missing', () async {
    final File apk = fileSystem.file('app-debug.apk');
    final AndroidApk androidApk = AndroidApk(
      applicationPackage: apk,
      id: 'app',
      versionCode: 22,
      launchActivity: 'Main',
    );
    final AndroidDevice androidDevice = setUpAndroidDevice();

    expect(await androidDevice.installApp(androidApk), false);
  });

  testWithoutContext('Can install app on API level 16 or greater', () async {
    final FakeProcessManager processManager = FakeProcessManager.list(<FakeCommand>[
      kAdbVersionCommand,
      kAdbStartServerCommand,
      const FakeCommand(
        command: <String>['adb', '-s', '1234', 'shell', 'getprop'],
        stdout: '[ro.build.version.sdk]: [16]',
      ),
      const FakeCommand(
        command: <String>[
          'adb',
          '-s',
          '1234',
          'shell',
          'pm',
          'list',
          'packages',
          '--user',
          '10',
          'app',
        ],
        stdout: '\n',
      ),
      kInstallCommand,
      kStoreShaCommand,
    ]);
    final File apk = fileSystem.file('app-debug.apk')..createSync();
    final AndroidApk androidApk = AndroidApk(
      applicationPackage: apk,
      id: 'app',
      versionCode: 22,
      launchActivity: 'Main',
    );
    final AndroidDevice androidDevice = setUpAndroidDevice(processManager: processManager);

    expect(await androidDevice.installApp(androidApk, userIdentifier: '10'), true);
    expect(processManager, hasNoRemainingExpectations);
  });

  testWithoutContext('Defaults to API level 16 if adb returns a null response', () async {
    final FakeProcessManager processManager = FakeProcessManager.list(<FakeCommand>[
      kAdbVersionCommand,
      kAdbStartServerCommand,
      const FakeCommand(command: <String>['adb', '-s', '1234', 'shell', 'getprop']),
      const FakeCommand(
        command: <String>[
          'adb',
          '-s',
          '1234',
          'shell',
          'pm',
          'list',
          'packages',
          '--user',
          '10',
          'app',
        ],
        stdout: '\n',
      ),
      kInstallCommand,
      kStoreShaCommand,
    ]);
    final File apk = fileSystem.file('app-debug.apk')..createSync();
    final AndroidApk androidApk = AndroidApk(
      applicationPackage: apk,
      id: 'app',
      versionCode: 22,
      launchActivity: 'Main',
    );
    final AndroidDevice androidDevice = setUpAndroidDevice(processManager: processManager);

    expect(await androidDevice.installApp(androidApk, userIdentifier: '10'), true);
    expect(processManager, hasNoRemainingExpectations);
  });

  testWithoutContext('displays error if user not found', () async {
    final FakeProcessManager processManager = FakeProcessManager.list(<FakeCommand>[
      kAdbVersionCommand,
      kAdbStartServerCommand,
      const FakeCommand(command: <String>['adb', '-s', '1234', 'shell', 'getprop']),
      // This command is run before the user is checked and is allowed to fail.
      const FakeCommand(
        command: <String>[
          'adb',
          '-s',
          '1234',
          'shell',
          'pm',
          'list',
          'packages',
          '--user',
          'jane',
          'app',
        ],
        stderr: 'Blah blah',
        exitCode: 1,
      ),
      const FakeCommand(
        command: <String>[
          'adb',
          '-s',
          '1234',
          'install',
          '-t',
          '-r',
          '--user',
          'jane',
          'app-debug.apk',
        ],
        exitCode: 1,
        stderr:
            'Exception occurred while executing: java.lang.IllegalArgumentException: Bad user number: jane',
      ),
    ]);
    final File apk = fileSystem.file('app-debug.apk')..createSync();
    final AndroidApk androidApk = AndroidApk(
      applicationPackage: apk,
      id: 'app',
      versionCode: 22,
      launchActivity: 'Main',
    );
    final AndroidDevice androidDevice = setUpAndroidDevice(processManager: processManager);

    expect(await androidDevice.installApp(androidApk, userIdentifier: 'jane'), false);
    expect(
      logger.errorText,
      contains(
        'Error: User "jane" not found. Run "adb shell pm list users" to see list of available identifiers.',
      ),
    );
    expect(processManager, hasNoRemainingExpectations);
  });

  testWithoutContext('Will skip install if the correct version is up to date', () async {
    final FakeProcessManager processManager = FakeProcessManager.list(<FakeCommand>[
      kAdbVersionCommand,
      kAdbStartServerCommand,
      const FakeCommand(
        command: <String>['adb', '-s', '1234', 'shell', 'getprop'],
        stdout: '[ro.build.version.sdk]: [16]',
      ),
      const FakeCommand(
        command: <String>[
          'adb',
          '-s',
          '1234',
          'shell',
          'pm',
          'list',
          'packages',
          '--user',
          '10',
          'app',
        ],
        stdout: 'package:app\n',
      ),
      const FakeCommand(
        command: <String>['adb', '-s', '1234', 'shell', 'cat', '/data/local/tmp/sky.app.sha1'],
        stdout: 'example_sha',
      ),
    ]);
    final File apk = fileSystem.file('app-debug.apk')..createSync();
    fileSystem.file('app-debug.apk.sha1').writeAsStringSync('example_sha');
    final AndroidApk androidApk = AndroidApk(
      applicationPackage: apk,
      id: 'app',
      versionCode: 22,
      launchActivity: 'Main',
    );
    final AndroidDevice androidDevice = setUpAndroidDevice(processManager: processManager);

    expect(await androidDevice.installApp(androidApk, userIdentifier: '10'), true);
    expect(processManager, hasNoRemainingExpectations);
  });

  testWithoutContext(
    'Will uninstall if the correct version is not up to date and install fails',
    () async {
      final FakeProcessManager processManager = FakeProcessManager.list(<FakeCommand>[
        kAdbVersionCommand,
        kAdbStartServerCommand,
        const FakeCommand(
          command: <String>['adb', '-s', '1234', 'shell', 'getprop'],
          stdout: '[ro.build.version.sdk]: [16]',
        ),
        const FakeCommand(
          command: <String>[
            'adb',
            '-s',
            '1234',
            'shell',
            'pm',
            'list',
            'packages',
            '--user',
            '10',
            'app',
          ],
          stdout: 'package:app\n',
        ),
        const FakeCommand(
          command: <String>['adb', '-s', '1234', 'shell', 'cat', '/data/local/tmp/sky.app.sha1'],
          stdout: 'different_example_sha',
        ),
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
          exitCode: 1,
          stderr: '[INSTALL_FAILED_INSUFFICIENT_STORAGE]',
        ),
        const FakeCommand(
          command: <String>['adb', '-s', '1234', 'uninstall', '--user', '10', 'app'],
        ),
        kInstallCommand,
        const FakeCommand(
          command: <String>[
            'adb',
            '-s',
            '1234',
            'shell',
            'echo',
            '-n',
            'example_sha',
            '>',
            '/data/local/tmp/sky.app.sha1',
          ],
        ),
      ]);
      final File apk = fileSystem.file('app-debug.apk')..createSync();
      fileSystem.file('app-debug.apk.sha1').writeAsStringSync('example_sha');
      final AndroidApk androidApk = AndroidApk(
        applicationPackage: apk,
        id: 'app',
        versionCode: 22,
        launchActivity: 'Main',
      );
      final AndroidDevice androidDevice = setUpAndroidDevice(processManager: processManager);

      expect(await androidDevice.installApp(androidApk, userIdentifier: '10'), true);
      expect(processManager, hasNoRemainingExpectations);
    },
  );

  testWithoutContext(
    'Will fail to install if the apk was never installed and it fails the first time',
    () async {
      final FakeProcessManager processManager = FakeProcessManager.list(<FakeCommand>[
        kAdbVersionCommand,
        kAdbStartServerCommand,
        const FakeCommand(
          command: <String>['adb', '-s', '1234', 'shell', 'getprop'],
          stdout: '[ro.build.version.sdk]: [16]',
        ),
        const FakeCommand(
          command: <String>[
            'adb',
            '-s',
            '1234',
            'shell',
            'pm',
            'list',
            'packages',
            '--user',
            '10',
            'app',
          ],
          stdout: '\n',
        ),
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
          exitCode: 1,
          stderr: '[INSTALL_FAILED_INSUFFICIENT_STORAGE]',
        ),
      ]);
      final File apk = fileSystem.file('app-debug.apk')..createSync();
      final AndroidApk androidApk = AndroidApk(
        applicationPackage: apk,
        id: 'app',
        versionCode: 22,
        launchActivity: 'Main',
      );
      final AndroidDevice androidDevice = setUpAndroidDevice(processManager: processManager);

      expect(await androidDevice.installApp(androidApk, userIdentifier: '10'), false);
      expect(processManager, hasNoRemainingExpectations);
    },
  );
}

class FakeAndroidSdk extends Fake implements AndroidSdk {
  @override
  String get adbPath => 'adb';
}
