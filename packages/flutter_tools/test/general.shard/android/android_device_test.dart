// Copyright 2014 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

import 'dart:async';
import 'dart:convert';
import 'dart:typed_data';

import 'package:file/memory.dart';
import 'package:flutter_tools/src/android/android_console.dart';
import 'package:flutter_tools/src/android/android_device.dart';
import 'package:flutter_tools/src/android/android_sdk.dart';
import 'package:flutter_tools/src/base/file_system.dart';
import 'package:flutter_tools/src/base/io.dart';
import 'package:flutter_tools/src/base/logger.dart';
import 'package:flutter_tools/src/base/platform.dart';
import 'package:flutter_tools/src/build_info.dart';
import 'package:flutter_tools/src/device.dart';
import 'package:flutter_tools/src/globals.dart' as globals;
import 'package:flutter_tools/src/project.dart';
import 'package:flutter_tools/src/vmservice.dart';
import 'package:test/fake.dart';
import 'package:vm_service/vm_service.dart';

import '../../src/common.dart';
import '../../src/context.dart';
import '../../src/fake_process_manager.dart';

void main() {
  testWithoutContext('AndroidDevice stores the requested id', () {
    final AndroidDevice device = setUpAndroidDevice();

    expect(device.id, '1234');
  });

  testWithoutContext('parseAdbDeviceProperties parses adb shell output', () {
    final Map<String, String> properties = parseAdbDeviceProperties(kAdbShellGetprop);

    expect(properties, isNotNull);
    expect(properties['ro.build.characteristics'], 'emulator');
    expect(properties['ro.product.cpu.abi'], 'x86_64');
    expect(properties['ro.build.version.sdk'], '23');
  });

  testWithoutContext('adb exiting with heap corruption is only allowed on windows', () async {
    final List<FakeCommand> commands = <FakeCommand>[
      const FakeCommand(
        command: <String>['adb', '-s', '1234', 'shell', 'getprop'],
        stdout: '[ro.hardware]: [goldfish]\n[ro.build.characteristics]: [unused]',
        // Heap corruption exit code.
        exitCode: -1073740940,
      ),
    ];

    final AndroidDevice windowsDevice = setUpAndroidDevice(
      processManager: FakeProcessManager.list(commands.toList()),
      platform: FakePlatform(operatingSystem: 'windows'),
    );
    final AndroidDevice linuxDevice = setUpAndroidDevice(
      processManager: FakeProcessManager.list(commands.toList()),
      platform: FakePlatform(),
    );
    final AndroidDevice macOsDevice = setUpAndroidDevice(
      processManager: FakeProcessManager.list(commands.toList()),
      platform: FakePlatform(operatingSystem: 'macos'),
    );

    // Parsing succeeds despite the error.
    expect(await windowsDevice.isLocalEmulator, true);

    // Parsing fails and these default to false.
    expect(await linuxDevice.isLocalEmulator, false);
    expect(await macOsDevice.isLocalEmulator, false);
  });

  testWithoutContext('AndroidDevice can detect TargetPlatform from property '
      'abi and abiList', () async {
    // The format is [ABI, ABI list]: expected target platform.
    final Map<List<String>, TargetPlatform> values = <List<String>, TargetPlatform>{
      <String>['x86_64', 'unknown']: TargetPlatform.android_x64,
      <String>['x86', 'unknown']: TargetPlatform.android_x86,
      // The default ABI is arm32
      <String>['???', 'unknown']: TargetPlatform.android_arm,
      <String>['arm64-v8a', 'arm64-v8a,']: TargetPlatform.android_arm64,
      // The Kindle Fire runs 32 bit apps on 64 bit hardware.
      <String>['arm64-v8a', 'arm']: TargetPlatform.android_arm,
    };

    for (final MapEntry<List<String>, TargetPlatform> entry in values.entries) {
      final AndroidDevice device = setUpAndroidDevice(
        processManager: FakeProcessManager.list(<FakeCommand>[
          FakeCommand(
            command: const <String>['adb', '-s', '1234', 'shell', 'getprop'],
            stdout:
                '[ro.product.cpu.abi]: [${entry.key.first}]\n'
                '[ro.product.cpu.abilist]: [${entry.key.last}]',
          ),
        ]),
      );

      expect(await device.targetPlatform, entry.value);
    }
  });

  testWithoutContext('AndroidDevice supports profile/release mode on arm and x64 targets '
      'abi and abiList', () async {
    // The format is [ABI, ABI list]: expected release mode support.
    final Map<List<String>, bool> values = <List<String>, bool>{
      <String>['x86_64', 'unknown']: true,
      <String>['x86', 'unknown']: false,
      // The default ABI is arm32
      <String>['???', 'unknown']: true,
      <String>['arm64-v8a', 'arm64-v8a,']: true,
      // The Kindle Fire runs 32 bit apps on 64 bit hardware.
      <String>['arm64-v8a', 'arm']: true,
    };

    for (final MapEntry<List<String>, bool> entry in values.entries) {
      final AndroidDevice device = setUpAndroidDevice(
        processManager: FakeProcessManager.list(<FakeCommand>[
          FakeCommand(
            command: const <String>['adb', '-s', '1234', 'shell', 'getprop'],
            stdout:
                '[ro.product.cpu.abi]: [${entry.key.first}]\n'
                '[ro.product.cpu.abilist]: [${entry.key.last}]',
          ),
        ]),
      );

      expect(await device.supportsRuntimeMode(BuildMode.release), entry.value);
      // Debug is always supported.
      expect(await device.supportsRuntimeMode(BuildMode.debug), true);
      // jitRelease is never supported.
      expect(await device.supportsRuntimeMode(BuildMode.jitRelease), false);
    }
  });

  testWithoutContext('AndroidDevice can detect local emulator for known types', () async {
    for (final String hardware in kKnownHardware.keys) {
      final AndroidDevice device = setUpAndroidDevice(
        processManager: FakeProcessManager.list(<FakeCommand>[
          FakeCommand(
            command: const <String>['adb', '-s', '1234', 'shell', 'getprop'],
            stdout:
                '[ro.hardware]: [$hardware]\n'
                '[ro.build.characteristics]: [unused]',
          ),
        ]),
      );

      expect(await device.isLocalEmulator, kKnownHardware[hardware] == HardwareType.emulator);
    }
  });

  testWithoutContext('AndroidDevice can detect unknown hardware', () async {
    final AndroidDevice device = setUpAndroidDevice(
      processManager: FakeProcessManager.list(<FakeCommand>[
        const FakeCommand(
          command: <String>['adb', '-s', '1234', 'shell', 'getprop'],
          stdout:
              '[ro.hardware]: [unknown]\n'
              '[ro.build.characteristics]: [att]',
        ),
      ]),
    );

    expect(await device.isLocalEmulator, false);
  });

  testWithoutContext('AndroidDevice can detect unknown emulator', () async {
    final AndroidDevice device = setUpAndroidDevice(
      processManager: FakeProcessManager.list(<FakeCommand>[
        const FakeCommand(
          command: <String>['adb', '-s', '1234', 'shell', 'getprop'],
          stdout:
              '[ro.hardware]: [unknown]\n'
              '[ro.build.characteristics]: [att,emulator]',
        ),
      ]),
    );

    expect(await device.isLocalEmulator, true);
  });

  testWithoutContext('isSupportedForProject is true on module project', () async {
    final FileSystem fileSystem = MemoryFileSystem.test();
    fileSystem.file('pubspec.yaml')
      ..createSync()
      ..writeAsStringSync(r'''
name: example

flutter:
  module: {}
''');
    final FlutterProject flutterProject = FlutterProjectFactory(
      fileSystem: fileSystem,
      logger: BufferLogger.test(),
    ).fromDirectory(fileSystem.currentDirectory);
    final AndroidDevice device = setUpAndroidDevice(fileSystem: fileSystem);

    expect(device.isSupportedForProject(flutterProject), true);
  });

  testWithoutContext('isSupportedForProject is true with editable host app', () async {
    final FileSystem fileSystem = MemoryFileSystem.test();
    fileSystem.file('pubspec.yaml').createSync();
    fileSystem.directory('android').createSync();
    final FlutterProject flutterProject = FlutterProjectFactory(
      fileSystem: fileSystem,
      logger: BufferLogger.test(),
    ).fromDirectory(fileSystem.currentDirectory);

    final AndroidDevice device = setUpAndroidDevice(fileSystem: fileSystem);

    expect(device.isSupportedForProject(flutterProject), true);
  });

  testWithoutContext('isSupportedForProject is false with no host app and no module', () async {
    final FileSystem fileSystem = MemoryFileSystem.test();
    fileSystem.file('pubspec.yaml').createSync();
    final FlutterProject flutterProject = FlutterProjectFactory(
      fileSystem: fileSystem,
      logger: BufferLogger.test(),
    ).fromDirectory(fileSystem.currentDirectory);

    final AndroidDevice device = setUpAndroidDevice(fileSystem: fileSystem);

    expect(device.isSupportedForProject(flutterProject), false);
  });

  testWithoutContext('AndroidDevice returns correct ID for responsive emulator', () async {
    final AndroidDevice device = setUpAndroidDevice(
      processManager: FakeProcessManager.list(<FakeCommand>[
        const FakeCommand(
          command: <String>['adb', '-s', 'emulator-5555', 'shell', 'getprop'],
          stdout: '[ro.hardware]: [goldfish]',
        ),
      ]),
      id: 'emulator-5555',
      androidConsoleSocketFactory:
          (String host, int port) async => FakeWorkingAndroidConsoleSocket('dummyEmulatorId'),
    );

    expect(await device.emulatorId, equals('dummyEmulatorId'));
  });

  testWithoutContext('AndroidDevice does not create socket for non-emulator devices', () async {
    bool socketWasCreated = false;

    // Still use an emulator-looking ID so we can be sure the failure is due
    // to the isLocalEmulator field and not because the ID doesn't contain a
    // port.
    final AndroidDevice device = setUpAndroidDevice(
      id: 'emulator-5555',
      processManager: FakeProcessManager.list(<FakeCommand>[
        const FakeCommand(
          command: <String>['adb', '-s', 'emulator-5555', 'shell', 'getprop'],
          stdout: '[ro.hardware]: [samsungexynos7420]',
        ),
      ]),
      androidConsoleSocketFactory: (String host, int port) async {
        socketWasCreated = true;
        throw Exception('Socket was created for non-emulator');
      },
    );

    expect(await device.emulatorId, isNull);
    expect(socketWasCreated, isFalse);
  });

  testWithoutContext('AndroidDevice does not create socket for emulators with no port', () async {
    bool socketWasCreated = false;
    final AndroidDevice device = setUpAndroidDevice(
      processManager: FakeProcessManager.list(<FakeCommand>[
        const FakeCommand(
          command: <String>['adb', '-s', '1234', 'shell', 'getprop'],
          stdout: '[ro.hardware]: [goldfish]',
        ),
      ]),
      androidConsoleSocketFactory: (String host, int port) async {
        socketWasCreated = true;
        throw Exception('Socket was created for emulator without port in ID');
      },
    );

    expect(await device.emulatorId, isNull);
    expect(socketWasCreated, isFalse);
  });

  testWithoutContext('AndroidDevice.emulatorId is null for connection error', () async {
    final AndroidDevice device = setUpAndroidDevice(
      processManager: FakeProcessManager.list(<FakeCommand>[
        const FakeCommand(
          command: <String>['adb', '-s', '1234', 'shell', 'getprop'],
          stdout: '[ro.hardware]: [goldfish]',
        ),
      ]),
      androidConsoleSocketFactory: (String host, int port) => throw Exception('Fake socket error'),
    );

    expect(await device.emulatorId, isNull);
  });

  testWithoutContext('AndroidDevice.emulatorId is null for unresponsive device', () async {
    final AndroidDevice device = setUpAndroidDevice(
      processManager: FakeProcessManager.list(<FakeCommand>[
        const FakeCommand(
          command: <String>['adb', '-s', '1234', 'shell', 'getprop'],
          stdout: '[ro.hardware]: [goldfish]',
        ),
      ]),
      androidConsoleSocketFactory:
          (String host, int port) async => FakeUnresponsiveAndroidConsoleSocket(),
    );

    expect(await device.emulatorId, isNull);
  });

  testWithoutContext('AndroidDevice.emulatorId is null on early disconnect', () async {
    final AndroidDevice device = setUpAndroidDevice(
      processManager: FakeProcessManager.list(<FakeCommand>[
        const FakeCommand(
          command: <String>['adb', '-s', '1234', 'shell', 'getprop'],
          stdout: '[ro.hardware]: [goldfish]',
        ),
      ]),
      androidConsoleSocketFactory:
          (String host, int port) async => FakeDisconnectingAndroidConsoleSocket(),
    );

    expect(await device.emulatorId, isNull);
  });

  testWithoutContext('AndroidDevice clearLogs does not crash', () async {
    final AndroidDevice device = setUpAndroidDevice(
      processManager: FakeProcessManager.list(<FakeCommand>[
        const FakeCommand(command: <String>['adb', '-s', '1234', 'logcat', '-c'], exitCode: 1),
      ]),
    );
    device.clearLogs();
  });

  testWithoutContext(
    'AndroidDevice lastLogcatTimestamp returns null if shell command failed',
    () async {
      final AndroidDevice device = setUpAndroidDevice(
        processManager: FakeProcessManager.list(<FakeCommand>[
          const FakeCommand(
            command: <String>[
              'adb',
              '-s',
              '1234',
              'shell',
              '-x',
              'logcat',
              '-v',
              'time',
              '-t',
              '1',
            ],
            exitCode: 1,
          ),
        ]),
      );

      expect(await device.lastLogcatTimestamp(), isNull);
    },
  );

  testWithoutContext(
    'AndroidDevice AdbLogReaders for past+future and future logs are not the same',
    () async {
      final AndroidDevice device = setUpAndroidDevice(
        processManager: FakeProcessManager.list(<FakeCommand>[
          const FakeCommand(
            command: <String>['adb', '-s', '1234', 'shell', 'getprop'],
            stdout: '[ro.build.version.sdk]: [23]',
            exitCode: 1,
          ),
          const FakeCommand(
            command: <String>[
              'adb',
              '-s',
              '1234',
              'shell',
              '-x',
              'logcat',
              '-v',
              'time',
              '-s',
              'flutter',
            ],
          ),
          const FakeCommand(
            command: <String>['adb', '-s', '1234', 'shell', '-x', 'logcat', '-v', 'time'],
          ),
        ]),
      );

      final DeviceLogReader pastLogReader = await device.getLogReader(includePastLogs: true);
      final DeviceLogReader defaultLogReader = await device.getLogReader();
      expect(pastLogReader, isNot(equals(defaultLogReader)));

      // Getting again is cached.
      expect(pastLogReader, equals(await device.getLogReader(includePastLogs: true)));
      expect(defaultLogReader, equals(await device.getLogReader()));
    },
  );

  testWithoutContext('Can parse adb shell dumpsys info', () {
    const String exampleOutput = r'''
Applications Memory Usage (in Kilobytes):
Uptime: 441088659 Realtime: 521464097

** MEMINFO in pid 16141 [io.flutter.demo.gallery] **
                   Pss  Private  Private  SwapPss     Heap     Heap     Heap
                 Total    Dirty    Clean    Dirty     Size    Alloc     Free
                ------   ------   ------   ------   ------   ------   ------
  Native Heap     8648     8620        0       16    20480    12403     8076
  Dalvik Heap      547      424       40       18     2628     1092     1536
 Dalvik Other      464      464        0        0
        Stack      496      496        0        0
       Ashmem        2        0        0        0
      Gfx dev      212      204        0        0
    Other dev       48        0       48        0
     .so mmap    10770      708     9372       25
    .apk mmap      240        0        0        0
    .ttf mmap       35        0       32        0
    .dex mmap     2205        4     1172        0
    .oat mmap       64        0        0        0
    .art mmap     4228     3848       24        2
   Other mmap    20713        4    20704        0
    GL mtrack     2380     2380        0        0
      Unknown    43971    43968        0        1
        TOTAL    95085    61120    31392       62    23108    13495     9612

 App Summary
                       Pss(KB)
                        ------
           Java Heap:     4296
         Native Heap:     8620
                Code:    11288
               Stack:      496
            Graphics:     2584
       Private Other:    65228
              System:     2573

               TOTAL:    95085       TOTAL SWAP PSS:       62

 Objects
               Views:        9         ViewRootImpl:        1
         AppContexts:        3           Activities:        1
              Assets:        4        AssetManagers:        3
       Local Binders:       10        Proxy Binders:       18
       Parcel memory:        6         Parcel count:       24
    Death Recipients:        0      OpenSSL Sockets:        0
            WebViews:        0

 SQL
         MEMORY_USED:        0
  PAGECACHE_OVERFLOW:        0          MALLOC_SIZE:        0
''';

    final AndroidMemoryInfo result = parseMeminfoDump(exampleOutput);

    // Parses correctly
    expect(result.realTime, 521464097);
    expect(result.javaHeap, 4296);
    expect(result.nativeHeap, 8620);
    expect(result.code, 11288);
    expect(result.stack, 496);
    expect(result.graphics, 2584);
    expect(result.privateOther, 65228);
    expect(result.system, 2573);

    // toJson works correctly
    final Map<String, Object> json = result.toJson();

    expect(json, containsPair('Realtime', 521464097));
    expect(json, containsPair('Java Heap', 4296));
    expect(json, containsPair('Native Heap', 8620));
    expect(json, containsPair('Code', 11288));
    expect(json, containsPair('Stack', 496));
    expect(json, containsPair('Graphics', 2584));
    expect(json, containsPair('Private Other', 65228));
    expect(json, containsPair('System', 2573));

    // computed from summation of other fields.
    expect(json, containsPair('Total', 95085));

    // contains identifier for platform in memory info.
    expect(json, containsPair('platform', 'Android'));
  });

  testWithoutContext('AndroidDevice stopApp does nothing if app is not passed', () async {
    final AndroidDevice device = setUpAndroidDevice();

    expect(await device.stopApp(null), isFalse);
  });

  testUsingContext(
    'AdbLogReader.provideVmService catches any RPCError due to VM service disconnection',
    () async {
      final BufferLogger logger = globals.logger as BufferLogger;
      final FlutterVmService vmService = FlutterVmService(_MyFakeVmService());
      final AdbLogReader logReader = AdbLogReader.test(FakeProcess(), 'foo', logger);
      await logReader.provideVmService(vmService);
      expect(
        logger.traceText,
        'VmService.getVm call failed: null: (-32000) '
        'Service connection disposed\n',
      );
      expect(
        logger.errorText,
        'An error occurred when setting up filtering for adb logs. '
        'Unable to communicate with the VM service.\n',
      );
    },
    overrides: <Type, Generator>{Logger: () => BufferLogger.test()},
  );
}

class _MyFakeVmService extends Fake implements VmService {
  @override
  Future<VM> getVM() async {
    throw RPCError(null, RPCErrorKind.kServerError.code, 'Service connection disposed');
  }
}

AndroidDevice setUpAndroidDevice({
  String? id,
  AndroidSdk? androidSdk,
  FileSystem? fileSystem,
  ProcessManager? processManager,
  Platform? platform,
  AndroidConsoleSocketFactory androidConsoleSocketFactory = kAndroidConsoleSocketFactory,
}) {
  androidSdk ??= FakeAndroidSdk();
  return AndroidDevice(
    id ?? '1234',
    modelID: 'TestModel',
    logger: BufferLogger.test(),
    platform: platform ?? FakePlatform(),
    androidSdk: androidSdk,
    fileSystem: fileSystem ?? MemoryFileSystem.test(),
    processManager: processManager ?? FakeProcessManager.any(),
    androidConsoleSocketFactory: androidConsoleSocketFactory,
  );
}

class FakeAndroidSdk extends Fake implements AndroidSdk {
  @override
  String get adbPath => 'adb';
}

const String kAdbShellGetprop = '''
[dalvik.vm.dex2oat-Xms]: [64m]
[dalvik.vm.dex2oat-Xmx]: [512m]
[dalvik.vm.heapsize]: [384m]
[dalvik.vm.image-dex2oat-Xms]: [64m]
[dalvik.vm.image-dex2oat-Xmx]: [64m]
[dalvik.vm.isa.x86.variant]: [dalvik.vm.isa.x86.features=default]
[dalvik.vm.isa.x86_64.features]: [default]
[dalvik.vm.isa.x86_64.variant]: [x86_64]
[dalvik.vm.lockprof.threshold]: [500]
[dalvik.vm.stack-trace-file]: [/data/anr/traces.txt]
[debug.atrace.tags.enableflags]: [0]
[debug.force_rtl]: [0]
[gsm.current.phone-type]: [1]
[gsm.network.type]: [Unknown]
[gsm.nitz.time]: [1473102078793]
[gsm.operator.alpha]: []
[gsm.operator.iso-country]: []
[gsm.operator.isroaming]: [false]
[gsm.operator.numeric]: []
[gsm.sim.operator.alpha]: []
[gsm.sim.operator.iso-country]: []
[gsm.sim.operator.numeric]: []
[gsm.sim.state]: [NOT_READY]
[gsm.version.ril-impl]: [android reference-ril 1.0]
[init.svc.adbd]: [running]
[init.svc.bootanim]: [running]
[init.svc.console]: [running]
[init.svc.debuggerd]: [running]
[init.svc.debuggerd64]: [running]
[init.svc.drm]: [running]
[init.svc.fingerprintd]: [running]
[init.svc.gatekeeperd]: [running]
[init.svc.goldfish-logcat]: [stopped]
[init.svc.goldfish-setup]: [stopped]
[init.svc.healthd]: [running]
[init.svc.installd]: [running]
[init.svc.keystore]: [running]
[init.svc.lmkd]: [running]
[init.svc.logd]: [running]
[init.svc.logd-reinit]: [stopped]
[init.svc.media]: [running]
[init.svc.netd]: [running]
[init.svc.perfprofd]: [running]
[init.svc.qemu-props]: [stopped]
[init.svc.ril-daemon]: [running]
[init.svc.servicemanager]: [running]
[init.svc.surfaceflinger]: [running]
[init.svc.ueventd]: [running]
[init.svc.vold]: [running]
[init.svc.zygote]: [running]
[init.svc.zygote_secondary]: [running]
[net.bt.name]: [Android]
[net.change]: [net.qtaguid_enabled]
[net.eth0.dns1]: [10.0.2.3]
[net.eth0.gw]: [10.0.2.2]
[net.gprs.local-ip]: [10.0.2.15]
[net.hostname]: [android-ccd858aa3d3825ee]
[net.qtaguid_enabled]: [1]
[net.tcp.default_init_rwnd]: [60]
[persist.sys.dalvik.vm.lib.2]: [libart.so]
[persist.sys.profiler_ms]: [0]
[persist.sys.timezone]: [America/Los_Angeles]
[persist.sys.usb.config]: [adb]
[qemu.gles]: [1]
[qemu.hw.mainkeys]: [0]
[qemu.sf.fake_camera]: [none]
[qemu.sf.lcd_density]: [420]
[rild.libargs]: [-d /dev/ttyS0]
[rild.libpath]: [/system/lib/libreference-ril.so]
[ro.allow.mock.location]: [0]
[ro.baseband]: [unknown]
[ro.board.platform]: []
[ro.boot.hardware]: [ranchu]
[ro.bootimage.build.date]: [Wed Jul 20 21:03:09 UTC 2016]
[ro.bootimage.build.date.utc]: [1469048589]
[ro.bootimage.build.fingerprint]: [Android/sdk_google_phone_x86_64/generic_x86_64:6.0/MASTER/3079352:userdebug/test-keys]
[ro.bootloader]: [unknown]
[ro.bootmode]: [unknown]
[ro.build.characteristics]: [emulator]
[ro.build.date]: [Wed Jul 20 21:02:14 UTC 2016]
[ro.build.date.utc]: [1469048534]
[ro.build.description]: [sdk_google_phone_x86_64-userdebug 6.0 MASTER 3079352 test-keys]
[ro.build.display.id]: [sdk_google_phone_x86_64-userdebug 6.0 MASTER 3079352 test-keys]
[ro.build.fingerprint]: [Android/sdk_google_phone_x86_64/generic_x86_64:6.0/MASTER/3079352:userdebug/test-keys]
[ro.build.flavor]: [sdk_google_phone_x86_64-userdebug]
[ro.build.host]: [vpba14.mtv.corp.google.com]
[ro.build.id]: [MASTER]
[ro.build.product]: [generic_x86_64]
[ro.build.tags]: [test-keys]
[ro.build.type]: [userdebug]
[ro.build.user]: [android-build]
[ro.build.version.all_codenames]: [REL]
[ro.build.version.base_os]: []
[ro.build.version.codename]: [REL]
[ro.build.version.incremental]: [3079352]
[ro.build.version.preview_sdk]: [0]
[ro.build.version.release]: [6.0]
[ro.build.version.sdk]: [23]
[ro.build.version.security_patch]: [2015-10-01]
[ro.com.google.locationfeatures]: [1]
[ro.config.alarm_alert]: [Alarm_Classic.ogg]
[ro.config.nocheckin]: [yes]
[ro.config.notification_sound]: [OnTheHunt.ogg]
[ro.crypto.state]: [unencrypted]
[ro.dalvik.vm.native.bridge]: [0]
[ro.debuggable]: [1]
[ro.hardware]: [ranchu]
[ro.hardware.audio.primary]: [goldfish]
[ro.hwui.drop_shadow_cache_size]: [6]
[ro.hwui.gradient_cache_size]: [1]
[ro.hwui.layer_cache_size]: [48]
[ro.hwui.path_cache_size]: [32]
[ro.hwui.r_buffer_cache_size]: [8]
[ro.hwui.text_large_cache_height]: [1024]
[ro.hwui.text_large_cache_width]: [2048]
[ro.hwui.text_small_cache_height]: [1024]
[ro.hwui.text_small_cache_width]: [1024]
[ro.hwui.texture_cache_flushrate]: [0.4]
[ro.hwui.texture_cache_size]: [72]
[ro.kernel.android.checkjni]: [1]
[ro.kernel.android.qemud]: [1]
[ro.kernel.androidboot.hardware]: [ranchu]
[ro.kernel.clocksource]: [pit]
[ro.kernel.qemu]: [1]
[ro.kernel.qemu.gles]: [1]
[ro.opengles.version]: [131072]
[ro.product.board]: []
[ro.product.brand]: [Android]
[ro.product.cpu.abi]: [x86_64]
[ro.product.cpu.abilist]: [x86_64,x86]
[ro.product.cpu.abilist32]: [x86]
[ro.product.cpu.abilist64]: [x86_64]
[ro.product.device]: [generic_x86_64]
[ro.product.locale]: [en-US]
[ro.product.manufacturer]: [unknown]
[ro.product.model]: [Android SDK built for x86_64]
[ro.product.name]: [sdk_google_phone_x86_64]
[ro.radio.use-ppp]: [no]
[ro.revision]: [0]
[ro.secure]: [1]
[ro.serialno]: []
[ro.wifi.channels]: []
[ro.zygote]: [zygote64_32]
[selinux.reload_policy]: [1]
[service.bootanim.exit]: [0]
[status.battery.level]: [5]
[status.battery.level_raw]: [50]
[status.battery.level_scale]: [9]
[status.battery.state]: [Slow]
[sys.sysctl.extra_free_kbytes]: [24300]
[sys.usb.config]: [adb]
[sys.usb.state]: [adb]
[vold.has_adoptable]: [1]
[wlan.driver.status]: [unloaded]
[xmpp.auto-presence]: [true]
''';

/// A mock Android Console that presents a connection banner and responds to
/// "avd name" requests with the supplied name.
class FakeWorkingAndroidConsoleSocket extends Fake implements Socket {
  FakeWorkingAndroidConsoleSocket(this.avdName) {
    _controller.add('Android Console: Welcome!\n');
    // Include OK in the same packet here. In the response to "avd name"
    // it's sent alone to ensure both are handled.
    _controller.add('Android Console: Some intro text\nOK\n');
  }

  final String avdName;
  final StreamController<String> _controller = StreamController<String>();

  @override
  Stream<E> asyncMap<E>(FutureOr<E> Function(Uint8List event) convert) =>
      _controller.stream as Stream<E>;

  @override
  void add(List<int> data) {
    final String text = ascii.decode(data);
    if (text == 'avd name\n') {
      _controller.add('$avdName\n');
      // Include OK in its own packet here. In welcome banner it's included
      // as part of the previous text to ensure both are handled.
      _controller.add('OK\n');
    } else {
      throw Exception('Unexpected command $text');
    }
  }

  @override
  void destroy() {}
}

/// An Android console socket that drops all input and returns no output.
class FakeUnresponsiveAndroidConsoleSocket extends Fake implements Socket {
  final StreamController<String> _controller = StreamController<String>();

  @override
  Stream<E> asyncMap<E>(FutureOr<E> Function(Uint8List event) convert) =>
      _controller.stream as Stream<E>;

  @override
  void add(List<int> data) {}

  @override
  void destroy() {}
}

/// An Android console socket that drops all input and returns no output.
class FakeDisconnectingAndroidConsoleSocket extends Fake implements Socket {
  FakeDisconnectingAndroidConsoleSocket() {
    _controller.add('Android Console: Welcome!\n');
    // Include OK in the same packet here. In the response to "avd name"
    // it's sent alone to ensure both are handled.
    _controller.add('Android Console: Some intro text\nOK\n');
  }

  final StreamController<String> _controller = StreamController<String>();

  @override
  Stream<E> asyncMap<E>(FutureOr<E> Function(Uint8List event) convert) =>
      _controller.stream as Stream<E>;

  @override
  void add(List<int> data) {
    _controller.close();
  }

  @override
  void destroy() {}
}
