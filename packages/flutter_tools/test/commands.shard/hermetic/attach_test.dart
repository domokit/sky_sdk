// Copyright 2014 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

import 'dart:async';

import 'package:file/memory.dart';
import 'package:flutter_tools/src/android/android_device.dart';
import 'package:flutter_tools/src/application_package.dart';
import 'package:flutter_tools/src/artifacts.dart';
import 'package:flutter_tools/src/base/common.dart';
import 'package:flutter_tools/src/base/dds.dart';
import 'package:flutter_tools/src/base/file_system.dart';
import 'package:flutter_tools/src/base/io.dart';
import 'package:flutter_tools/src/base/logger.dart';
import 'package:flutter_tools/src/base/platform.dart';
import 'package:flutter_tools/src/base/signals.dart';
import 'package:flutter_tools/src/base/terminal.dart';
import 'package:flutter_tools/src/build_info.dart';
import 'package:flutter_tools/src/cache.dart';
import 'package:flutter_tools/src/commands/attach.dart';
import 'package:flutter_tools/src/compile.dart';
import 'package:flutter_tools/src/device.dart';
import 'package:flutter_tools/src/device_port_forwarder.dart';
import 'package:flutter_tools/src/device_vm_service_discovery_for_attach.dart';
import 'package:flutter_tools/src/ios/application_package.dart';
import 'package:flutter_tools/src/ios/devices.dart';
import 'package:flutter_tools/src/mdns_discovery.dart';
import 'package:flutter_tools/src/project.dart';
import 'package:flutter_tools/src/reporting/reporting.dart';
import 'package:flutter_tools/src/resident_runner.dart';
import 'package:flutter_tools/src/run_hot.dart';
import 'package:multicast_dns/multicast_dns.dart';
import 'package:test/fake.dart';
import 'package:unified_analytics/unified_analytics.dart';
import 'package:vm_service/vm_service.dart' as vm_service;

import '../../src/common.dart';
import '../../src/context.dart';
import '../../src/fake_devices.dart';
import '../../src/test_flutter_command_runner.dart';

class FakeStdio extends Fake implements Stdio {
  @override
  bool stdinHasTerminal = false;
}

class FakeProcessInfo extends Fake implements ProcessInfo {
  @override
  int maxRss = 0;
}

void main() {
  group('attach', () {
    late StreamLogger logger;
    late FileSystem testFileSystem;
    late TestDeviceManager testDeviceManager;
    late Artifacts artifacts;
    late Stdio stdio;
    late Terminal terminal;
    late Signals signals;
    late Platform platform;
    late ProcessInfo processInfo;

    setUp(() {
      Cache.disableLocking();
      logger = StreamLogger();
      platform = FakePlatform();
      testFileSystem = MemoryFileSystem.test();
      testFileSystem.directory('lib').createSync();
      testFileSystem.file(testFileSystem.path.join('lib', 'main.dart')).createSync();
      artifacts = Artifacts.test(fileSystem: testFileSystem);
      stdio = FakeStdio();
      terminal = FakeTerminal();
      signals = FakeSignals();
      processInfo = FakeProcessInfo();
      testDeviceManager = TestDeviceManager(logger: logger);
    });

    group('with one device and no specified target file', () {
      const int devicePort = 499;
      const int hostPort = 42;
      final int future = DateTime.now().add(const Duration(days: 1)).millisecondsSinceEpoch;

      late FakeDeviceLogReader fakeLogReader;
      late RecordingPortForwarder portForwarder;
      late FakeDartDevelopmentService fakeDds;
      late FakeAndroidDevice device;

      setUp(() {
        fakeLogReader = FakeDeviceLogReader();
        portForwarder = RecordingPortForwarder(hostPort);
        fakeDds = FakeDartDevelopmentService();
        device =
            FakeAndroidDevice(id: '1')
              ..portForwarder = portForwarder
              ..dds = fakeDds;
      });

      tearDown(() {
        fakeLogReader.dispose();
      });

      testUsingContext(
        'succeeds with iOS device with protocol discovery',
        () async {
          final FakeIOSDevice device = FakeIOSDevice(
            portForwarder: portForwarder,
            majorSdkVersion: 12,
            onGetLogReader: () {
              fakeLogReader.addLine('Foo');
              fakeLogReader.addLine(
                'The Dart VM service is listening on http://127.0.0.1:$devicePort',
              );
              return fakeLogReader;
            },
          );
          testDeviceManager.devices = <Device>[device];
          final Completer<void> completer = Completer<void>();
          final StreamSubscription<String> loggerSubscription = logger.stream.listen((
            String message,
          ) {
            if (message == '[verbose] VM Service URL on device: http://127.0.0.1:$devicePort') {
              // The "VM Service URL on device" message is output by the ProtocolDiscovery when it found the VM Service.
              completer.complete();
            }
          });
          final FakeHotRunner hotRunner = FakeHotRunner();
          hotRunner.onAttach =
              (
                Completer<DebugConnectionInfo>? connectionInfoCompleter,
                Completer<void>? appStartedCompleter,
                bool allowExistingDdsInstance,
                bool enableDevTools,
              ) async => 0;
          hotRunner.exited = false;
          hotRunner.isWaitingForVmService = false;
          final FakeHotRunnerFactory hotRunnerFactory =
              FakeHotRunnerFactory()..hotRunner = hotRunner;

          await createTestCommandRunner(
            AttachCommand(
              hotRunnerFactory: hotRunnerFactory,
              stdio: stdio,
              logger: logger,
              terminal: terminal,
              signals: signals,
              platform: platform,
              processInfo: processInfo,
              fileSystem: testFileSystem,
            ),
          ).run(<String>['attach']);
          await completer.future;

          expect(portForwarder.devicePort, devicePort);
          expect(portForwarder.hostPort, hostPort);

          await fakeLogReader.dispose();
          await loggerSubscription.cancel();
        },
        overrides: <Type, Generator>{
          FileSystem: () => testFileSystem,
          ProcessManager: () => FakeProcessManager.any(),
          Logger: () => logger,
          DeviceManager: () => testDeviceManager,
          MDnsVmServiceDiscovery:
              () => MDnsVmServiceDiscovery(
                mdnsClient: FakeMDnsClient(
                  <PtrResourceRecord>[],
                  <String, List<SrvResourceRecord>>{},
                ),
                preliminaryMDnsClient: FakeMDnsClient(
                  <PtrResourceRecord>[],
                  <String, List<SrvResourceRecord>>{},
                ),
                logger: logger,
                flutterUsage: TestUsage(),
                analytics: const NoOpAnalytics(),
              ),
        },
      );

      testUsingContext(
        'restores terminal to singleCharMode == false on command exit',
        () async {
          final FakeIOSDevice device = FakeIOSDevice(
            portForwarder: portForwarder,
            majorSdkVersion: 12,
            onGetLogReader: () {
              fakeLogReader.addLine('Foo');
              fakeLogReader.addLine(
                'The Dart VM service is listening on http://127.0.0.1:$devicePort',
              );
              return fakeLogReader;
            },
          );
          testDeviceManager.devices = <Device>[device];
          final Completer<void> completer = Completer<void>();
          final StreamSubscription<String> loggerSubscription = logger.stream.listen((
            String message,
          ) {
            if (message == '[verbose] VM Service URL on device: http://127.0.0.1:$devicePort') {
              // The "VM Service URL on device" message is output by the ProtocolDiscovery when it found the VM Service.
              completer.complete();
            }
          });
          final FakeHotRunner hotRunner = FakeHotRunner();
          hotRunner.onAttach = (
            Completer<DebugConnectionInfo>? connectionInfoCompleter,
            Completer<void>? appStartedCompleter,
            bool allowExistingDdsInstance,
            bool enableDevTools,
          ) async {
            appStartedCompleter?.complete();
            return 0;
          };
          hotRunner.exited = false;
          hotRunner.isWaitingForVmService = false;
          final FakeHotRunnerFactory hotRunnerFactory =
              FakeHotRunnerFactory()..hotRunner = hotRunner;

          await createTestCommandRunner(
            AttachCommand(
              hotRunnerFactory: hotRunnerFactory,
              stdio: stdio,
              logger: logger,
              terminal: terminal,
              signals: signals,
              platform: platform,
              processInfo: processInfo,
              fileSystem: testFileSystem,
            ),
          ).run(<String>['attach']);
          await completer.future;
          await Future.wait<void>(<Future<void>>[
            fakeLogReader.dispose(),
            loggerSubscription.cancel(),
          ]);

          expect(terminal.singleCharMode, isFalse);
        },
        overrides: <Type, Generator>{
          FileSystem: () => testFileSystem,
          ProcessManager: () => FakeProcessManager.any(),
          Logger: () => logger,
          DeviceManager: () => testDeviceManager,
          MDnsVmServiceDiscovery:
              () => MDnsVmServiceDiscovery(
                mdnsClient: FakeMDnsClient(
                  <PtrResourceRecord>[],
                  <String, List<SrvResourceRecord>>{},
                ),
                preliminaryMDnsClient: FakeMDnsClient(
                  <PtrResourceRecord>[],
                  <String, List<SrvResourceRecord>>{},
                ),
                logger: logger,
                flutterUsage: TestUsage(),
                analytics: const NoOpAnalytics(),
              ),
          Signals: () => FakeSignals(),
        },
      );

      testUsingContext(
        'local engine artifacts are passed to runner',
        () async {
          const String localEngineSrc = '/path/to/local/engine/src';
          const String localEngineDir = 'host_debug_unopt';
          testFileSystem
              .directory('$localEngineSrc/out/$localEngineDir')
              .createSync(recursive: true);
          final FakeIOSDevice device = FakeIOSDevice(
            portForwarder: portForwarder,
            majorSdkVersion: 12,
            onGetLogReader: () {
              fakeLogReader.addLine('Foo');
              fakeLogReader.addLine(
                'The Dart VM service is listening on http://127.0.0.1:$devicePort',
              );
              return fakeLogReader;
            },
          );
          testDeviceManager.devices = <Device>[device];
          final Completer<void> completer = Completer<void>();
          final StreamSubscription<String> loggerSubscription = logger.stream.listen((
            String message,
          ) {
            if (message == '[verbose] VM Service URL on device: http://127.0.0.1:$devicePort') {
              // The "VM Service URL on device" message is output by the ProtocolDiscovery when it found the VM Service.
              completer.complete();
            }
          });
          final FakeHotRunner hotRunner = FakeHotRunner();
          hotRunner.onAttach =
              (
                Completer<DebugConnectionInfo>? connectionInfoCompleter,
                Completer<void>? appStartedCompleter,
                bool allowExistingDdsInstance,
                bool enableDevTools,
              ) async => 0;
          hotRunner.exited = false;
          hotRunner.isWaitingForVmService = false;
          bool passedArtifactTest = false;
          final FakeHotRunnerFactory hotRunnerFactory =
              FakeHotRunnerFactory()
                ..hotRunner = hotRunner
                .._artifactTester = (Artifacts artifacts) {
                  expect(artifacts, isA<CachedLocalEngineArtifacts>());
                  // expecting this to be true ensures this test ran
                  passedArtifactTest = true;
                };

          await createTestCommandRunner(
            AttachCommand(
              hotRunnerFactory: hotRunnerFactory,
              stdio: stdio,
              logger: logger,
              terminal: terminal,
              signals: signals,
              platform: platform,
              processInfo: processInfo,
              fileSystem: testFileSystem,
            ),
          ).run(<String>[
            'attach',
            '--local-engine-src-path=$localEngineSrc',
            '--local-engine=$localEngineDir',
            '--local-engine-host=$localEngineDir',
          ]);
          await completer.future;
          await Future.wait<void>(<Future<void>>[
            fakeLogReader.dispose(),
            loggerSubscription.cancel(),
          ]);
          expect(passedArtifactTest, isTrue);
        },
        overrides: <Type, Generator>{
          Artifacts: () => artifacts,
          DeviceManager: () => testDeviceManager,
          FileSystem: () => testFileSystem,
          Logger: () => logger,
          MDnsVmServiceDiscovery:
              () => MDnsVmServiceDiscovery(
                mdnsClient: FakeMDnsClient(
                  <PtrResourceRecord>[],
                  <String, List<SrvResourceRecord>>{},
                ),
                preliminaryMDnsClient: FakeMDnsClient(
                  <PtrResourceRecord>[],
                  <String, List<SrvResourceRecord>>{},
                ),
                logger: logger,
                flutterUsage: TestUsage(),
                analytics: const NoOpAnalytics(),
              ),
          ProcessManager: () => FakeProcessManager.empty(),
        },
      );

      testUsingContext(
        'succeeds with iOS device with mDNS',
        () async {
          final FakeIOSDevice device = FakeIOSDevice(
            portForwarder: portForwarder,
            majorSdkVersion: 16,
            onGetLogReader: () {
              fakeLogReader.addLine('Foo');
              fakeLogReader.addLine(
                'The Dart VM service is listening on http://127.0.0.1:$devicePort',
              );
              return fakeLogReader;
            },
          );
          testDeviceManager.devices = <Device>[device];
          final FakeHotRunner hotRunner = FakeHotRunner();
          hotRunner.onAttach =
              (
                Completer<DebugConnectionInfo>? connectionInfoCompleter,
                Completer<void>? appStartedCompleter,
                bool allowExistingDdsInstance,
                bool enableDevTools,
              ) async => 0;
          hotRunner.exited = false;
          hotRunner.isWaitingForVmService = false;
          final FakeHotRunnerFactory hotRunnerFactory =
              FakeHotRunnerFactory()..hotRunner = hotRunner;

          await createTestCommandRunner(
            AttachCommand(
              hotRunnerFactory: hotRunnerFactory,
              stdio: stdio,
              logger: logger,
              terminal: terminal,
              signals: signals,
              platform: platform,
              processInfo: processInfo,
              fileSystem: testFileSystem,
            ),
          ).run(<String>['attach']);
          await fakeLogReader.dispose();

          // Listen to the URI before checking port forwarder. Port forwarding
          // is done as a side effect when generating the uri.
          final FlutterDevice flutterDevice = hotRunnerFactory.devices.first;
          final Uri? vmServiceUri = await flutterDevice.vmServiceUris?.first;
          expect(vmServiceUri.toString(), 'http://127.0.0.1:$hostPort/xyz/');

          expect(portForwarder.devicePort, devicePort);
          expect(portForwarder.hostPort, hostPort);
          expect(hotRunnerFactory.devices, hasLength(1));
        },
        overrides: <Type, Generator>{
          FileSystem: () => testFileSystem,
          ProcessManager: () => FakeProcessManager.any(),
          Logger: () => logger,
          DeviceManager: () => testDeviceManager,
          MDnsVmServiceDiscovery:
              () => MDnsVmServiceDiscovery(
                mdnsClient: FakeMDnsClient(
                  <PtrResourceRecord>[],
                  <String, List<SrvResourceRecord>>{},
                ),
                preliminaryMDnsClient: FakeMDnsClient(
                  <PtrResourceRecord>[PtrResourceRecord('foo', future, domainName: 'bar')],
                  <String, List<SrvResourceRecord>>{
                    'bar': <SrvResourceRecord>[
                      SrvResourceRecord(
                        'bar',
                        future,
                        port: devicePort,
                        weight: 1,
                        priority: 1,
                        target: 'appId',
                      ),
                    ],
                  },
                  txtResponse: <String, List<TxtResourceRecord>>{
                    'bar': <TxtResourceRecord>[
                      TxtResourceRecord('bar', future, text: 'authCode=xyz\n'),
                    ],
                  },
                ),
                logger: logger,
                flutterUsage: TestUsage(),
                analytics: const NoOpAnalytics(),
              ),
        },
      );

      testUsingContext(
        'succeeds with iOS device with mDNS wireless device',
        () async {
          final FakeIOSDevice device = FakeIOSDevice(
            portForwarder: portForwarder,
            majorSdkVersion: 16,
            connectionInterface: DeviceConnectionInterface.wireless,
          );
          testDeviceManager.devices = <Device>[device];
          final FakeHotRunner hotRunner = FakeHotRunner();
          hotRunner.onAttach =
              (
                Completer<DebugConnectionInfo>? connectionInfoCompleter,
                Completer<void>? appStartedCompleter,
                bool allowExistingDdsInstance,
                bool enableDevTools,
              ) async => 0;
          hotRunner.exited = false;
          hotRunner.isWaitingForVmService = false;
          final FakeHotRunnerFactory hotRunnerFactory =
              FakeHotRunnerFactory()..hotRunner = hotRunner;

          await createTestCommandRunner(
            AttachCommand(
              hotRunnerFactory: hotRunnerFactory,
              stdio: stdio,
              logger: logger,
              terminal: terminal,
              signals: signals,
              platform: platform,
              processInfo: processInfo,
              fileSystem: testFileSystem,
            ),
          ).run(<String>['attach']);
          await fakeLogReader.dispose();

          // Listen to the URI before checking port forwarder. Port forwarding
          // is done as a side effect when generating the uri.
          final FlutterDevice flutterDevice = hotRunnerFactory.devices.first;
          final Uri? vmServiceUri = await flutterDevice.vmServiceUris?.first;
          expect(vmServiceUri.toString(), 'http://111.111.111.111:123/xyz/');

          expect(portForwarder.devicePort, null);
          expect(portForwarder.hostPort, hostPort);
          expect(hotRunnerFactory.devices, hasLength(1));
        },
        overrides: <Type, Generator>{
          FileSystem: () => testFileSystem,
          ProcessManager: () => FakeProcessManager.any(),
          Logger: () => logger,
          DeviceManager: () => testDeviceManager,
          MDnsVmServiceDiscovery:
              () => MDnsVmServiceDiscovery(
                mdnsClient: FakeMDnsClient(
                  <PtrResourceRecord>[],
                  <String, List<SrvResourceRecord>>{},
                ),
                preliminaryMDnsClient: FakeMDnsClient(
                  <PtrResourceRecord>[PtrResourceRecord('foo', future, domainName: 'srv-foo')],
                  <String, List<SrvResourceRecord>>{
                    'srv-foo': <SrvResourceRecord>[
                      SrvResourceRecord(
                        'srv-foo',
                        future,
                        port: 123,
                        weight: 1,
                        priority: 1,
                        target: 'target-foo',
                      ),
                    ],
                  },
                  ipResponse: <String, List<IPAddressResourceRecord>>{
                    'target-foo': <IPAddressResourceRecord>[
                      IPAddressResourceRecord(
                        'target-foo',
                        0,
                        address: InternetAddress.tryParse('111.111.111.111')!,
                      ),
                    ],
                  },
                  txtResponse: <String, List<TxtResourceRecord>>{
                    'srv-foo': <TxtResourceRecord>[
                      TxtResourceRecord('srv-foo', future, text: 'authCode=xyz\n'),
                    ],
                  },
                ),
                logger: logger,
                flutterUsage: TestUsage(),
                analytics: const NoOpAnalytics(),
              ),
        },
      );

      testUsingContext(
        'succeeds with iOS device with mDNS wireless device with debug-port',
        () async {
          final FakeIOSDevice device = FakeIOSDevice(
            portForwarder: portForwarder,
            majorSdkVersion: 16,
            connectionInterface: DeviceConnectionInterface.wireless,
          );
          testDeviceManager.devices = <Device>[device];
          final FakeHotRunner hotRunner = FakeHotRunner();
          hotRunner.onAttach =
              (
                Completer<DebugConnectionInfo>? connectionInfoCompleter,
                Completer<void>? appStartedCompleter,
                bool allowExistingDdsInstance,
                bool enableDevTools,
              ) async => 0;
          hotRunner.exited = false;
          hotRunner.isWaitingForVmService = false;
          final FakeHotRunnerFactory hotRunnerFactory =
              FakeHotRunnerFactory()..hotRunner = hotRunner;

          await createTestCommandRunner(
            AttachCommand(
              hotRunnerFactory: hotRunnerFactory,
              stdio: stdio,
              logger: logger,
              terminal: terminal,
              signals: signals,
              platform: platform,
              processInfo: processInfo,
              fileSystem: testFileSystem,
            ),
          ).run(<String>['attach', '--debug-port', '123']);
          await fakeLogReader.dispose();

          // Listen to the URI before checking port forwarder. Port forwarding
          // is done as a side effect when generating the uri.
          final FlutterDevice flutterDevice = hotRunnerFactory.devices.first;
          final Uri? vmServiceUri = await flutterDevice.vmServiceUris?.first;
          expect(vmServiceUri.toString(), 'http://111.111.111.111:123/xyz/');

          expect(portForwarder.devicePort, null);
          expect(portForwarder.hostPort, hostPort);
          expect(hotRunnerFactory.devices, hasLength(1));
        },
        overrides: <Type, Generator>{
          FileSystem: () => testFileSystem,
          ProcessManager: () => FakeProcessManager.any(),
          Logger: () => logger,
          DeviceManager: () => testDeviceManager,
          MDnsVmServiceDiscovery:
              () => MDnsVmServiceDiscovery(
                mdnsClient: FakeMDnsClient(
                  <PtrResourceRecord>[],
                  <String, List<SrvResourceRecord>>{},
                ),
                preliminaryMDnsClient: FakeMDnsClient(
                  <PtrResourceRecord>[
                    PtrResourceRecord('bar', future, domainName: 'srv-bar'),
                    PtrResourceRecord('foo', future, domainName: 'srv-foo'),
                  ],
                  <String, List<SrvResourceRecord>>{
                    'srv-bar': <SrvResourceRecord>[
                      SrvResourceRecord(
                        'srv-bar',
                        future,
                        port: 321,
                        weight: 1,
                        priority: 1,
                        target: 'target-bar',
                      ),
                    ],
                    'srv-foo': <SrvResourceRecord>[
                      SrvResourceRecord(
                        'srv-foo',
                        future,
                        port: 123,
                        weight: 1,
                        priority: 1,
                        target: 'target-foo',
                      ),
                    ],
                  },
                  ipResponse: <String, List<IPAddressResourceRecord>>{
                    'target-foo': <IPAddressResourceRecord>[
                      IPAddressResourceRecord(
                        'target-foo',
                        0,
                        address: InternetAddress.tryParse('111.111.111.111')!,
                      ),
                    ],
                  },
                  txtResponse: <String, List<TxtResourceRecord>>{
                    'srv-foo': <TxtResourceRecord>[
                      TxtResourceRecord('srv-foo', future, text: 'authCode=xyz\n'),
                    ],
                  },
                ),
                logger: logger,
                flutterUsage: TestUsage(),
                analytics: const NoOpAnalytics(),
              ),
        },
      );

      testUsingContext(
        'succeeds with iOS device with mDNS wireless device with debug-url',
        () async {
          final FakeIOSDevice device = FakeIOSDevice(
            portForwarder: portForwarder,
            majorSdkVersion: 16,
            connectionInterface: DeviceConnectionInterface.wireless,
          );
          testDeviceManager.devices = <Device>[device];
          final FakeHotRunner hotRunner = FakeHotRunner();
          hotRunner.onAttach =
              (
                Completer<DebugConnectionInfo>? connectionInfoCompleter,
                Completer<void>? appStartedCompleter,
                bool allowExistingDdsInstance,
                bool enableDevTools,
              ) async => 0;
          hotRunner.exited = false;
          hotRunner.isWaitingForVmService = false;
          final FakeHotRunnerFactory hotRunnerFactory =
              FakeHotRunnerFactory()..hotRunner = hotRunner;

          await createTestCommandRunner(
            AttachCommand(
              hotRunnerFactory: hotRunnerFactory,
              stdio: stdio,
              logger: logger,
              terminal: terminal,
              signals: signals,
              platform: platform,
              processInfo: processInfo,
              fileSystem: testFileSystem,
            ),
          ).run(<String>['attach', '--debug-url', 'https://0.0.0.0:123']);
          await fakeLogReader.dispose();

          // Listen to the URI before checking port forwarder. Port forwarding
          // is done as a side effect when generating the uri.
          final FlutterDevice flutterDevice = hotRunnerFactory.devices.first;
          final Uri? vmServiceUri = await flutterDevice.vmServiceUris?.first;
          expect(vmServiceUri.toString(), 'http://111.111.111.111:123/xyz/');

          expect(portForwarder.devicePort, null);
          expect(portForwarder.hostPort, hostPort);
          expect(hotRunnerFactory.devices, hasLength(1));
        },
        overrides: <Type, Generator>{
          FileSystem: () => testFileSystem,
          ProcessManager: () => FakeProcessManager.any(),
          Logger: () => logger,
          DeviceManager: () => testDeviceManager,
          MDnsVmServiceDiscovery:
              () => MDnsVmServiceDiscovery(
                mdnsClient: FakeMDnsClient(
                  <PtrResourceRecord>[],
                  <String, List<SrvResourceRecord>>{},
                ),
                preliminaryMDnsClient: FakeMDnsClient(
                  <PtrResourceRecord>[
                    PtrResourceRecord('bar', future, domainName: 'srv-bar'),
                    PtrResourceRecord('foo', future, domainName: 'srv-foo'),
                  ],
                  <String, List<SrvResourceRecord>>{
                    'srv-bar': <SrvResourceRecord>[
                      SrvResourceRecord(
                        'srv-bar',
                        future,
                        port: 321,
                        weight: 1,
                        priority: 1,
                        target: 'target-bar',
                      ),
                    ],
                    'srv-foo': <SrvResourceRecord>[
                      SrvResourceRecord(
                        'srv-foo',
                        future,
                        port: 123,
                        weight: 1,
                        priority: 1,
                        target: 'target-foo',
                      ),
                    ],
                  },
                  ipResponse: <String, List<IPAddressResourceRecord>>{
                    'target-foo': <IPAddressResourceRecord>[
                      IPAddressResourceRecord(
                        'target-foo',
                        0,
                        address: InternetAddress.tryParse('111.111.111.111')!,
                      ),
                    ],
                  },
                  txtResponse: <String, List<TxtResourceRecord>>{
                    'srv-foo': <TxtResourceRecord>[
                      TxtResourceRecord('srv-foo', future, text: 'authCode=xyz\n'),
                    ],
                  },
                ),
                logger: logger,
                flutterUsage: TestUsage(),
                analytics: const NoOpAnalytics(),
              ),
        },
      );

      testUsingContext(
        'finds VM Service port and forwards',
        () async {
          device.onGetLogReader = () {
            fakeLogReader.addLine('Foo');
            fakeLogReader.addLine(
              'The Dart VM service is listening on http://127.0.0.1:$devicePort',
            );
            return fakeLogReader;
          };
          testDeviceManager.devices = <Device>[device];
          final Completer<void> completer = Completer<void>();
          final StreamSubscription<String> loggerSubscription = logger.stream.listen((
            String message,
          ) {
            if (message == '[verbose] VM Service URL on device: http://127.0.0.1:$devicePort') {
              // The "VM Service URL on device" message is output by the ProtocolDiscovery when it found the VM Service.
              completer.complete();
            }
          });
          final Future<void> task = createTestCommandRunner(
            AttachCommand(
              stdio: stdio,
              logger: logger,
              terminal: terminal,
              signals: signals,
              platform: platform,
              processInfo: processInfo,
              fileSystem: testFileSystem,
            ),
          ).run(<String>['attach']);
          await completer.future;

          expect(portForwarder.devicePort, devicePort);
          expect(portForwarder.hostPort, hostPort);

          await fakeLogReader.dispose();
          await expectLoggerInterruptEndsTask(task, logger);
          await loggerSubscription.cancel();
        },
        overrides: <Type, Generator>{
          FileSystem: () => testFileSystem,
          ProcessManager: () => FakeProcessManager.any(),
          Logger: () => logger,
          DeviceManager: () => testDeviceManager,
        },
      );

      testUsingContext(
        'Fails with tool exit on bad VmService uri',
        () async {
          device.onGetLogReader = () {
            fakeLogReader.addLine('Foo');
            fakeLogReader.addLine(
              'The Dart VM service is listening on http://127.0.0.1:$devicePort',
            );
            fakeLogReader.dispose();
            return fakeLogReader;
          };
          testDeviceManager.devices = <Device>[device];
          expect(
            () => createTestCommandRunner(
              AttachCommand(
                stdio: stdio,
                logger: logger,
                terminal: terminal,
                signals: signals,
                platform: platform,
                processInfo: processInfo,
                fileSystem: testFileSystem,
              ),
            ).run(<String>['attach']),
            throwsToolExit(),
          );
        },
        overrides: <Type, Generator>{
          FileSystem: () => testFileSystem,
          ProcessManager: () => FakeProcessManager.any(),
          Logger: () => logger,
          DeviceManager: () => testDeviceManager,
        },
      );

      testUsingContext(
        'accepts filesystem parameters',
        () async {
          device.onGetLogReader = () {
            fakeLogReader.addLine('Foo');
            fakeLogReader.addLine(
              'The Dart VM service is listening on http://127.0.0.1:$devicePort',
            );
            return fakeLogReader;
          };
          testDeviceManager.devices = <Device>[device];

          const String filesystemScheme = 'foo';
          const String filesystemRoot = '/build-output/';
          const String projectRoot = '/build-output/project-root';
          const String outputDill = '/tmp/output.dill';

          final FakeHotRunner hotRunner = FakeHotRunner();
          hotRunner.onAttach =
              (
                Completer<DebugConnectionInfo>? connectionInfoCompleter,
                Completer<void>? appStartedCompleter,
                bool allowExistingDdsInstance,
                bool enableDevTools,
              ) async => 0;
          hotRunner.exited = false;
          hotRunner.isWaitingForVmService = false;

          final FakeHotRunnerFactory hotRunnerFactory =
              FakeHotRunnerFactory()..hotRunner = hotRunner;

          final AttachCommand command = AttachCommand(
            hotRunnerFactory: hotRunnerFactory,
            stdio: stdio,
            logger: logger,
            terminal: terminal,
            signals: signals,
            platform: platform,
            processInfo: processInfo,
            fileSystem: testFileSystem,
          );
          await createTestCommandRunner(command).run(<String>[
            'attach',
            '--filesystem-scheme',
            filesystemScheme,
            '--filesystem-root',
            filesystemRoot,
            '--project-root',
            projectRoot,
            '--output-dill',
            outputDill,
            '-v', // enables verbose logging
          ]);

          // Validate the attach call built a fake runner with the right
          // project root and output dill.
          expect(hotRunnerFactory.projectRootPath, projectRoot);
          expect(hotRunnerFactory.dillOutputPath, outputDill);
          expect(hotRunnerFactory.devices, hasLength(1));

          // Validate that the attach call built a flutter device with the right
          // output dill, filesystem scheme, and filesystem root.
          final FlutterDevice flutterDevice = hotRunnerFactory.devices.first;

          expect(flutterDevice.buildInfo.fileSystemScheme, filesystemScheme);
          expect(flutterDevice.buildInfo.fileSystemRoots, const <String>[filesystemRoot]);
        },
        overrides: <Type, Generator>{
          FileSystem: () => testFileSystem,
          ProcessManager: () => FakeProcessManager.any(),
          DeviceManager: () => testDeviceManager,
        },
      );

      testUsingContext(
        'exits when ipv6 is specified and debug-port is not on non-iOS device',
        () async {
          testDeviceManager.devices = <Device>[device];

          final AttachCommand command = AttachCommand(
            stdio: stdio,
            logger: logger,
            terminal: terminal,
            signals: signals,
            platform: platform,
            processInfo: processInfo,
            fileSystem: testFileSystem,
          );
          await expectLater(
            createTestCommandRunner(command).run(<String>['attach', '--ipv6']),
            throwsToolExit(
              message:
                  'When the --debug-port or --debug-url is unknown, this command determines '
                  'the value of --ipv6 on its own.',
            ),
          );
        },
        overrides: <Type, Generator>{
          FileSystem: () => testFileSystem,
          ProcessManager: () => FakeProcessManager.any(),
          DeviceManager: () => testDeviceManager,
        },
      );

      testUsingContext(
        'succeeds when ipv6 is specified and debug-port is not on iOS device',
        () async {
          final FakeIOSDevice device = FakeIOSDevice(
            portForwarder: portForwarder,
            majorSdkVersion: 12,
            onGetLogReader: () {
              fakeLogReader.addLine('Foo');
              fakeLogReader.addLine('The Dart VM service is listening on http://[::1]:$devicePort');
              return fakeLogReader;
            },
          );
          testDeviceManager.devices = <Device>[device];
          final Completer<void> completer = Completer<void>();
          final StreamSubscription<String> loggerSubscription = logger.stream.listen((
            String message,
          ) {
            if (message == '[verbose] VM Service URL on device: http://[::1]:$devicePort') {
              // The "VM Service URL on device" message is output by the ProtocolDiscovery when it found the VM Service.
              completer.complete();
            }
          });
          final FakeHotRunner hotRunner = FakeHotRunner();
          hotRunner.onAttach =
              (
                Completer<DebugConnectionInfo>? connectionInfoCompleter,
                Completer<void>? appStartedCompleter,
                bool allowExistingDdsInstance,
                bool enableDevTools,
              ) async => 0;
          hotRunner.exited = false;
          hotRunner.isWaitingForVmService = false;
          final FakeHotRunnerFactory hotRunnerFactory =
              FakeHotRunnerFactory()..hotRunner = hotRunner;

          await createTestCommandRunner(
            AttachCommand(
              hotRunnerFactory: hotRunnerFactory,
              stdio: stdio,
              logger: logger,
              terminal: terminal,
              signals: signals,
              platform: platform,
              processInfo: processInfo,
              fileSystem: testFileSystem,
            ),
          ).run(<String>['attach', '--ipv6']);
          await completer.future;

          expect(portForwarder.devicePort, devicePort);
          expect(portForwarder.hostPort, hostPort);

          await fakeLogReader.dispose();
          await loggerSubscription.cancel();
        },
        overrides: <Type, Generator>{
          FileSystem: () => testFileSystem,
          ProcessManager: () => FakeProcessManager.any(),
          Logger: () => logger,
          DeviceManager: () => testDeviceManager,
          MDnsVmServiceDiscovery:
              () => MDnsVmServiceDiscovery(
                mdnsClient: FakeMDnsClient(
                  <PtrResourceRecord>[],
                  <String, List<SrvResourceRecord>>{},
                ),
                preliminaryMDnsClient: FakeMDnsClient(
                  <PtrResourceRecord>[],
                  <String, List<SrvResourceRecord>>{},
                ),
                logger: logger,
                flutterUsage: TestUsage(),
                analytics: const NoOpAnalytics(),
              ),
        },
      );

      testUsingContext(
        'exits when vm-service-port is specified and debug-port is not',
        () async {
          device.onGetLogReader = () {
            fakeLogReader.addLine('Foo');
            fakeLogReader.addLine(
              'The Dart VM service is listening on http://127.0.0.1:$devicePort',
            );
            return fakeLogReader;
          };
          testDeviceManager.devices = <Device>[device];

          final AttachCommand command = AttachCommand(
            stdio: stdio,
            logger: logger,
            terminal: terminal,
            signals: signals,
            platform: platform,
            processInfo: processInfo,
            fileSystem: testFileSystem,
          );
          await expectLater(
            createTestCommandRunner(command).run(<String>['attach', '--vm-service-port', '100']),
            throwsToolExit(
              message:
                  'When the --debug-port or --debug-url is unknown, this command does not use '
                  'the value of --vm-service-port.',
            ),
          );
        },
        overrides: <Type, Generator>{
          FileSystem: () => testFileSystem,
          ProcessManager: () => FakeProcessManager.any(),
          DeviceManager: () => testDeviceManager,
        },
      );
    });

    group('forwarding to given port', () {
      const int devicePort = 499;
      const int hostPort = 42;
      late RecordingPortForwarder portForwarder;
      late FakeAndroidDevice device;

      setUp(() {
        final FakeDartDevelopmentService fakeDds = FakeDartDevelopmentService();
        portForwarder = RecordingPortForwarder(hostPort);
        device =
            FakeAndroidDevice(id: '1')
              ..portForwarder = portForwarder
              ..dds = fakeDds;
      });

      testUsingContext(
        'succeeds in ipv4 mode',
        () async {
          testDeviceManager.devices = <Device>[device];

          final Completer<void> completer = Completer<void>();
          final StreamSubscription<String> loggerSubscription = logger.stream.listen((
            String message,
          ) {
            if (message == '[verbose] Connecting to service protocol: http://127.0.0.1:42/') {
              // Wait until resident_runner.dart tries to connect.
              // There's nothing to connect _to_, so that's as far as we care to go.
              completer.complete();
            }
          });
          final Future<void> task = createTestCommandRunner(
            AttachCommand(
              stdio: stdio,
              logger: logger,
              terminal: terminal,
              signals: signals,
              platform: platform,
              processInfo: processInfo,
              fileSystem: testFileSystem,
            ),
          ).run(<String>['attach', '--debug-port', '$devicePort']);
          await completer.future;
          expect(portForwarder.devicePort, devicePort);
          expect(portForwarder.hostPort, hostPort);

          await expectLoggerInterruptEndsTask(task, logger);
          await loggerSubscription.cancel();
        },
        overrides: <Type, Generator>{
          FileSystem: () => testFileSystem,
          ProcessManager: () => FakeProcessManager.any(),
          Logger: () => logger,
          DeviceManager: () => testDeviceManager,
        },
      );

      testUsingContext(
        'succeeds in ipv6 mode',
        () async {
          testDeviceManager.devices = <Device>[device];

          final Completer<void> completer = Completer<void>();
          final StreamSubscription<String> loggerSubscription = logger.stream.listen((
            String message,
          ) {
            if (message == '[verbose] Connecting to service protocol: http://[::1]:42/') {
              // Wait until resident_runner.dart tries to connect.
              // There's nothing to connect _to_, so that's as far as we care to go.
              completer.complete();
            }
          });
          final Future<void> task = createTestCommandRunner(
            AttachCommand(
              stdio: stdio,
              logger: logger,
              terminal: terminal,
              signals: signals,
              platform: platform,
              processInfo: processInfo,
              fileSystem: testFileSystem,
            ),
          ).run(<String>['attach', '--debug-port', '$devicePort', '--ipv6']);
          await completer.future;

          expect(portForwarder.devicePort, devicePort);
          expect(portForwarder.hostPort, hostPort);

          await expectLoggerInterruptEndsTask(task, logger);
          await loggerSubscription.cancel();
        },
        overrides: <Type, Generator>{
          FileSystem: () => testFileSystem,
          ProcessManager: () => FakeProcessManager.any(),
          Logger: () => logger,
          DeviceManager: () => testDeviceManager,
        },
      );

      testUsingContext(
        'skips in ipv4 mode with a provided VM Service port',
        () async {
          testDeviceManager.devices = <Device>[device];

          final Completer<void> completer = Completer<void>();
          final StreamSubscription<String> loggerSubscription = logger.stream.listen((
            String message,
          ) {
            if (message == '[verbose] Connecting to service protocol: http://127.0.0.1:42/') {
              // Wait until resident_runner.dart tries to connect.
              // There's nothing to connect _to_, so that's as far as we care to go.
              completer.complete();
            }
          });
          final Future<void> task = createTestCommandRunner(
            AttachCommand(
              stdio: stdio,
              logger: logger,
              terminal: terminal,
              signals: signals,
              platform: platform,
              processInfo: processInfo,
              fileSystem: testFileSystem,
            ),
          ).run(<String>[
            'attach',
            '--debug-port',
            '$devicePort',
            '--vm-service-port',
            '$hostPort',
            // Ensure DDS doesn't use hostPort by binding to a random port.
            '--dds-port',
            '0',
          ]);
          await completer.future;
          expect(portForwarder.devicePort, null);
          expect(portForwarder.hostPort, 42);

          await expectLoggerInterruptEndsTask(task, logger);
          await loggerSubscription.cancel();
        },
        overrides: <Type, Generator>{
          FileSystem: () => testFileSystem,
          ProcessManager: () => FakeProcessManager.any(),
          Logger: () => logger,
          DeviceManager: () => testDeviceManager,
        },
      );

      testUsingContext(
        'skips in ipv6 mode with a provided VM Service port',
        () async {
          testDeviceManager.devices = <Device>[device];

          final Completer<void> completer = Completer<void>();
          final StreamSubscription<String> loggerSubscription = logger.stream.listen((
            String message,
          ) {
            if (message == '[verbose] Connecting to service protocol: http://[::1]:42/') {
              // Wait until resident_runner.dart tries to connect.
              // There's nothing to connect _to_, so that's as far as we care to go.
              completer.complete();
            }
          });
          final Future<void> task = createTestCommandRunner(
            AttachCommand(
              stdio: stdio,
              logger: logger,
              terminal: terminal,
              signals: signals,
              platform: platform,
              processInfo: processInfo,
              fileSystem: testFileSystem,
            ),
          ).run(<String>[
            'attach',
            '--debug-port',
            '$devicePort',
            '--vm-service-port',
            '$hostPort',
            '--ipv6',
            // Ensure DDS doesn't use hostPort by binding to a random port.
            '--dds-port',
            '0',
          ]);
          await completer.future;
          expect(portForwarder.devicePort, null);
          expect(portForwarder.hostPort, 42);

          await expectLoggerInterruptEndsTask(task, logger);
          await loggerSubscription.cancel();
        },
        overrides: <Type, Generator>{
          FileSystem: () => testFileSystem,
          ProcessManager: () => FakeProcessManager.any(),
          Logger: () => logger,
          DeviceManager: () => testDeviceManager,
        },
      );
    });

    testUsingContext(
      'exits when no device connected',
      () async {
        final AttachCommand command = AttachCommand(
          stdio: stdio,
          logger: logger,
          terminal: terminal,
          signals: signals,
          platform: platform,
          processInfo: processInfo,
          fileSystem: testFileSystem,
        );
        await expectLater(
          createTestCommandRunner(command).run(<String>['attach']),
          throwsToolExit(),
        );
        expect(testLogger.statusText, containsIgnoringWhitespace('No supported devices connected'));
      },
      overrides: <Type, Generator>{
        FileSystem: () => testFileSystem,
        ProcessManager: () => FakeProcessManager.any(),
        DeviceManager: () => testDeviceManager,
      },
    );

    testUsingContext(
      'fails when targeted device is not Android with --device-user',
      () async {
        final FakeIOSDevice device = FakeIOSDevice();
        testDeviceManager.devices = <Device>[device];
        expect(
          createTestCommandRunner(
            AttachCommand(
              stdio: stdio,
              logger: logger,
              terminal: terminal,
              signals: signals,
              platform: platform,
              processInfo: processInfo,
              fileSystem: testFileSystem,
            ),
          ).run(<String>['attach', '--device-user', '10']),
          throwsToolExit(message: '--device-user is only supported for Android'),
        );
      },
      overrides: <Type, Generator>{
        FileSystem: () => testFileSystem,
        ProcessManager: () => FakeProcessManager.any(),
        DeviceManager: () => testDeviceManager,
      },
    );

    testUsingContext(
      'exits when multiple devices connected',
      () async {
        final AttachCommand command = AttachCommand(
          stdio: stdio,
          logger: logger,
          terminal: terminal,
          signals: signals,
          platform: platform,
          processInfo: processInfo,
          fileSystem: testFileSystem,
        );
        testDeviceManager.devices = <Device>[
          FakeAndroidDevice(id: 'xx1'),
          FakeAndroidDevice(id: 'yy2'),
        ];
        await expectLater(
          createTestCommandRunner(command).run(<String>['attach']),
          throwsToolExit(),
        );
        expect(testLogger.statusText, containsIgnoringWhitespace('More than one device'));
        expect(testLogger.statusText, contains('xx1'));
        expect(testLogger.statusText, contains('yy2'));
      },
      overrides: <Type, Generator>{
        FileSystem: () => testFileSystem,
        ProcessManager: () => FakeProcessManager.any(),
        DeviceManager: () => testDeviceManager,
        AnsiTerminal: () => FakeTerminal(stdinHasTerminal: false),
      },
    );

    testUsingContext(
      'Catches service disappeared error',
      () async {
        final FakeAndroidDevice device =
            FakeAndroidDevice(id: '1')
              ..portForwarder = const NoOpDevicePortForwarder()
              ..onGetLogReader = () => NoOpDeviceLogReader('test');
        final FakeHotRunner hotRunner = FakeHotRunner();
        final FakeHotRunnerFactory hotRunnerFactory = FakeHotRunnerFactory()..hotRunner = hotRunner;
        hotRunner.onAttach = (
          Completer<DebugConnectionInfo>? connectionInfoCompleter,
          Completer<void>? appStartedCompleter,
          bool allowExistingDdsInstance,
          bool enableDevTools,
        ) async {
          await null;
          throw vm_service.RPCError(
            'flutter._listViews',
            vm_service.RPCErrorKind.kServiceDisappeared.code,
            '',
          );
        };

        testDeviceManager.devices = <Device>[device];
        testFileSystem.file('lib/main.dart').createSync();

        final AttachCommand command = AttachCommand(
          hotRunnerFactory: hotRunnerFactory,
          stdio: stdio,
          logger: logger,
          terminal: terminal,
          signals: signals,
          platform: platform,
          processInfo: processInfo,
          fileSystem: testFileSystem,
        );
        await expectLater(
          createTestCommandRunner(command).run(<String>['attach']),
          throwsToolExit(message: 'Lost connection to device.'),
        );
      },
      overrides: <Type, Generator>{
        FileSystem: () => testFileSystem,
        ProcessManager: () => FakeProcessManager.any(),
        DeviceManager: () => testDeviceManager,
      },
    );

    testUsingContext(
      'Catches "Service connection disposed" error',
      () async {
        final FakeAndroidDevice device =
            FakeAndroidDevice(id: '1')
              ..portForwarder = const NoOpDevicePortForwarder()
              ..onGetLogReader = () => NoOpDeviceLogReader('test');
        final FakeHotRunner hotRunner = FakeHotRunner();
        final FakeHotRunnerFactory hotRunnerFactory = FakeHotRunnerFactory()..hotRunner = hotRunner;
        hotRunner.onAttach = (
          Completer<DebugConnectionInfo>? connectionInfoCompleter,
          Completer<void>? appStartedCompleter,
          bool allowExistingDdsInstance,
          bool enableDevTools,
        ) async {
          await null;
          throw vm_service.RPCError(
            'flutter._listViews',
            vm_service.RPCErrorKind.kServerError.code,
            'Service connection disposed',
          );
        };

        testDeviceManager.devices = <Device>[device];
        testFileSystem.file('lib/main.dart').createSync();

        final AttachCommand command = AttachCommand(
          hotRunnerFactory: hotRunnerFactory,
          stdio: stdio,
          logger: logger,
          terminal: terminal,
          signals: signals,
          platform: platform,
          processInfo: processInfo,
          fileSystem: testFileSystem,
        );
        await expectLater(
          createTestCommandRunner(command).run(<String>['attach']),
          throwsToolExit(message: 'Lost connection to device.'),
        );
      },
      overrides: <Type, Generator>{
        FileSystem: () => testFileSystem,
        ProcessManager: () => FakeProcessManager.any(),
        DeviceManager: () => testDeviceManager,
      },
    );

    testUsingContext(
      'Does not catch generic RPC error',
      () async {
        final FakeAndroidDevice device =
            FakeAndroidDevice(id: '1')
              ..portForwarder = const NoOpDevicePortForwarder()
              ..onGetLogReader = () => NoOpDeviceLogReader('test');
        final FakeHotRunner hotRunner = FakeHotRunner();
        final FakeHotRunnerFactory hotRunnerFactory = FakeHotRunnerFactory()..hotRunner = hotRunner;

        hotRunner.onAttach = (
          Completer<DebugConnectionInfo>? connectionInfoCompleter,
          Completer<void>? appStartedCompleter,
          bool allowExistingDdsInstance,
          bool enableDevTools,
        ) async {
          await null;
          throw vm_service.RPCError(
            'flutter._listViews',
            vm_service.RPCErrorKind.kInvalidParams.code,
            '',
          );
        };

        testDeviceManager.devices = <Device>[device];
        testFileSystem.file('lib/main.dart').createSync();

        final AttachCommand command = AttachCommand(
          hotRunnerFactory: hotRunnerFactory,
          stdio: stdio,
          logger: logger,
          terminal: terminal,
          signals: signals,
          platform: platform,
          processInfo: processInfo,
          fileSystem: testFileSystem,
        );
        await expectLater(
          createTestCommandRunner(command).run(<String>['attach']),
          throwsA(isA<vm_service.RPCError>()),
        );
      },
      overrides: <Type, Generator>{
        FileSystem: () => testFileSystem,
        ProcessManager: () => FakeProcessManager.any(),
        DeviceManager: () => testDeviceManager,
      },
    );
  });
}

class FakeHotRunner extends Fake implements HotRunner {
  late Future<int> Function(Completer<DebugConnectionInfo>?, Completer<void>?, bool, bool) onAttach;

  @override
  bool exited = false;

  @override
  bool isWaitingForVmService = true;

  @override
  Future<int> attach({
    Completer<DebugConnectionInfo>? connectionInfoCompleter,
    Completer<void>? appStartedCompleter,
    bool allowExistingDdsInstance = false,
    bool enableDevTools = false,
    bool needsFullRestart = true,
  }) {
    return onAttach(
      connectionInfoCompleter,
      appStartedCompleter,
      allowExistingDdsInstance,
      enableDevTools,
    );
  }

  @override
  bool supportsServiceProtocol = false;

  @override
  bool stayResident = true;

  @override
  void printHelp({required bool details}) {}
}

class FakeHotRunnerFactory extends Fake implements HotRunnerFactory {
  late HotRunner hotRunner;
  String? dillOutputPath;
  String? projectRootPath;
  late List<FlutterDevice> devices;
  void Function(Artifacts artifacts)? _artifactTester;

  @override
  HotRunner build(
    List<FlutterDevice> devices, {
    required String target,
    required DebuggingOptions debuggingOptions,
    bool benchmarkMode = false,
    File? applicationBinary,
    bool hostIsIde = false,
    String? projectRootPath,
    String? packagesFilePath,
    String? dillOutputPath,
    bool stayResident = true,
    bool ipv6 = false,
    FlutterProject? flutterProject,
    Analytics? analytics,
    String? nativeAssetsYamlFile,
  }) {
    if (_artifactTester != null) {
      for (final FlutterDevice device in devices) {
        _artifactTester!((device.generator! as DefaultResidentCompiler).artifacts);
      }
    }
    this.devices = devices;
    this.dillOutputPath = dillOutputPath;
    this.projectRootPath = projectRootPath;
    return hotRunner;
  }
}

class RecordingPortForwarder implements DevicePortForwarder {
  RecordingPortForwarder([this.hostPort]);

  int? devicePort;
  int? hostPort;

  @override
  Future<void> dispose() async {}

  @override
  Future<int> forward(int devicePort, {int? hostPort}) async {
    this.devicePort = devicePort;
    this.hostPort ??= hostPort;
    return this.hostPort!;
  }

  @override
  List<ForwardedPort> get forwardedPorts => <ForwardedPort>[];

  @override
  Future<void> unforward(ForwardedPort forwardedPort) async {}
}

class StreamLogger extends Logger {
  @override
  bool get isVerbose => true;

  @override
  void printError(
    String message, {
    StackTrace? stackTrace,
    bool? emphasis,
    TerminalColor? color,
    int? indent,
    int? hangingIndent,
    bool? wrap,
  }) {
    hadErrorOutput = true;
    _log('[stderr] $message');
  }

  @override
  void printWarning(
    String message, {
    bool? emphasis,
    TerminalColor? color,
    int? indent,
    int? hangingIndent,
    bool? wrap,
    bool fatal = true,
  }) {
    hadWarningOutput = hadWarningOutput || fatal;
    _log('[stderr] $message');
  }

  @override
  void printStatus(
    String message, {
    bool? emphasis,
    TerminalColor? color,
    bool? newline,
    int? indent,
    int? hangingIndent,
    bool? wrap,
  }) {
    _log('[stdout] $message');
  }

  @override
  void printBox(String message, {String? title}) {
    if (title == null) {
      _log('[stdout] $message');
    } else {
      _log('[stdout] $title: $message');
    }
  }

  @override
  void printTrace(String message) {
    _log('[verbose] $message');
  }

  @override
  Status startProgress(
    String message, {
    Duration? timeout,
    String? progressId,
    bool multilineOutput = false,
    bool includeTiming = true,
    int progressIndicatorPadding = kDefaultStatusPadding,
  }) {
    _log('[progress] $message');
    return SilentStatus(stopwatch: Stopwatch())..start();
  }

  @override
  Status startSpinner({
    VoidCallback? onFinish,
    Duration? timeout,
    SlowWarningCallback? slowWarningCallback,
    TerminalColor? warningColor,
  }) {
    return SilentStatus(stopwatch: Stopwatch(), onFinish: onFinish)..start();
  }

  bool _interrupt = false;

  void interrupt() {
    _interrupt = true;
  }

  final StreamController<String> _controller = StreamController<String>.broadcast();

  void _log(String message) {
    _controller.add(message);
    if (_interrupt) {
      _interrupt = false;
      throw const LoggerInterrupted();
    }
  }

  Stream<String> get stream => _controller.stream;

  @override
  void sendEvent(String name, [Map<String, dynamic>? args]) {}

  @override
  bool get supportsColor => throw UnimplementedError();

  @override
  bool get hasTerminal => false;

  @override
  void clear() => _log('[stdout] ${terminal.clearScreen()}\n');

  @override
  Terminal get terminal => Terminal.test();
}

class LoggerInterrupted implements Exception {
  const LoggerInterrupted();
}

Future<void> expectLoggerInterruptEndsTask(Future<void> task, StreamLogger logger) async {
  logger.interrupt(); // an exception during the task should cause it to fail...
  await expectLater(
    () => task,
    throwsA(isA<ToolExit>().having((ToolExit error) => error.exitCode, 'exitCode', 2)),
  );
}

class FakeDartDevelopmentService extends Fake implements DartDevelopmentService {
  @override
  Future<void> get done => noopCompleter.future;
  final Completer<void> noopCompleter = Completer<void>();

  @override
  Future<void> startDartDevelopmentService(
    Uri vmServiceUri, {
    int? ddsPort,
    FlutterDevice? device,
    bool? ipv6,
    bool? disableServiceAuthCodes,
    bool enableDevTools = false,
    bool cacheStartupProfile = false,
    String? google3WorkspaceRoot,
    Uri? devToolsServerAddress,
  }) async {}

  @override
  Uri get uri => Uri.parse('http://localhost:8181');
}

class FakeAndroidDevice extends Fake implements AndroidDevice {
  FakeAndroidDevice({required this.id});

  @override
  late DartDevelopmentService dds;

  @override
  final String id;

  @override
  String get name => 'd$id';

  @override
  Future<bool> get isLocalEmulator async => false;

  @override
  Future<String> get sdkNameAndVersion async => 'Android 46';

  @override
  Future<String> get targetPlatformDisplayName async => 'android';

  @override
  Future<TargetPlatform> get targetPlatform async => TargetPlatform.android_arm;

  @override
  DeviceConnectionInterface get connectionInterface => DeviceConnectionInterface.attached;

  @override
  bool isSupported() => true;

  @override
  bool get isConnected => true;

  @override
  bool get supportsHotRestart => true;

  @override
  bool get supportsFlutterExit => false;

  @override
  bool isSupportedForProject(FlutterProject flutterProject) => true;

  @override
  DevicePortForwarder? portForwarder;

  DeviceLogReader Function()? onGetLogReader;

  @override
  FutureOr<DeviceLogReader> getLogReader({ApplicationPackage? app, bool includePastLogs = false}) {
    if (onGetLogReader == null) {
      throw UnimplementedError(
        'Called getLogReader but no onGetLogReader callback was supplied in the constructor to FakeAndroidDevice.',
      );
    }
    return onGetLogReader!();
  }

  @override
  final PlatformType platformType = PlatformType.android;

  @override
  Category get category => Category.mobile;

  @override
  bool get ephemeral => true;

  @override
  VMServiceDiscoveryForAttach getVMServiceDiscoveryForAttach({
    String? appId,
    String? fuchsiaModule,
    int? filterDevicePort,
    int? expectedHostPort,
    required bool ipv6,
    required Logger logger,
  }) => LogScanningVMServiceDiscoveryForAttach(
    Future<DeviceLogReader>.value(getLogReader()),
    portForwarder: portForwarder,
    devicePort: filterDevicePort,
    hostPort: expectedHostPort,
    ipv6: ipv6,
    logger: logger,
  );
}

class FakeIOSDevice extends Fake implements IOSDevice {
  FakeIOSDevice({
    DevicePortForwarder? portForwarder,
    this.onGetLogReader,
    this.connectionInterface = DeviceConnectionInterface.attached,
    this.majorSdkVersion = 0,
  }) : _portForwarder = portForwarder;

  final DevicePortForwarder? _portForwarder;
  @override
  int majorSdkVersion;

  @override
  final DeviceConnectionInterface connectionInterface;

  @override
  bool get isWirelesslyConnected => connectionInterface == DeviceConnectionInterface.wireless;

  @override
  DevicePortForwarder get portForwarder => _portForwarder!;

  @override
  DartDevelopmentService get dds => throw UnimplementedError('getter dds not implemented');

  final DeviceLogReader Function()? onGetLogReader;

  @override
  DeviceLogReader getLogReader({
    IOSApp? app,
    bool includePastLogs = false,
    bool usingCISystem = false,
  }) {
    if (onGetLogReader == null) {
      throw UnimplementedError(
        'Called getLogReader but no onGetLogReader callback was supplied in the constructor to FakeIOSDevice',
      );
    }
    return onGetLogReader!();
  }

  @override
  final String name = 'name';

  @override
  Future<TargetPlatform> get targetPlatform async => TargetPlatform.ios;

  @override
  final PlatformType platformType = PlatformType.ios;

  @override
  bool isSupported() => true;

  @override
  bool isSupportedForProject(FlutterProject project) => true;

  @override
  bool get isConnected => true;

  @override
  bool get ephemeral => true;

  @override
  VMServiceDiscoveryForAttach getVMServiceDiscoveryForAttach({
    String? appId,
    String? fuchsiaModule,
    int? filterDevicePort,
    int? expectedHostPort,
    required bool ipv6,
    required Logger logger,
  }) {
    final bool compatibleWithProtocolDiscovery =
        majorSdkVersion < IOSDeviceLogReader.minimumUniversalLoggingSdkVersion &&
        !isWirelesslyConnected;
    final MdnsVMServiceDiscoveryForAttach mdnsVMServiceDiscoveryForAttach =
        MdnsVMServiceDiscoveryForAttach(
          device: this,
          appId: appId,
          deviceVmservicePort: filterDevicePort,
          hostVmservicePort: expectedHostPort,
          usesIpv6: ipv6,
          useDeviceIPAsHost: isWirelesslyConnected,
        );

    if (compatibleWithProtocolDiscovery) {
      return DelegateVMServiceDiscoveryForAttach(<VMServiceDiscoveryForAttach>[
        mdnsVMServiceDiscoveryForAttach,
        LogScanningVMServiceDiscoveryForAttach(
          Future<DeviceLogReader>.value(getLogReader()),
          portForwarder: portForwarder,
          devicePort: filterDevicePort,
          hostPort: expectedHostPort,
          ipv6: ipv6,
          logger: logger,
        ),
      ]);
    } else {
      return mdnsVMServiceDiscoveryForAttach;
    }
  }
}

class FakeMDnsClient extends Fake implements MDnsClient {
  FakeMDnsClient(
    this.ptrRecords,
    this.srvResponse, {
    this.txtResponse = const <String, List<TxtResourceRecord>>{},
    this.ipResponse = const <String, List<IPAddressResourceRecord>>{},
    this.osErrorOnStart = false,
  });

  final List<PtrResourceRecord> ptrRecords;
  final Map<String, List<SrvResourceRecord>> srvResponse;
  final Map<String, List<TxtResourceRecord>> txtResponse;
  final Map<String, List<IPAddressResourceRecord>> ipResponse;
  final bool osErrorOnStart;

  @override
  Future<void> start({
    InternetAddress? listenAddress,
    NetworkInterfacesFactory? interfacesFactory,
    int mDnsPort = 5353,
    InternetAddress? mDnsAddress,
  }) async {
    if (osErrorOnStart) {
      throw const OSError('Operation not supported on socket', 102);
    }
  }

  @override
  Stream<T> lookup<T extends ResourceRecord>(
    ResourceRecordQuery query, {
    Duration timeout = const Duration(seconds: 5),
  }) {
    if (T == PtrResourceRecord &&
        query.fullyQualifiedName == MDnsVmServiceDiscovery.dartVmServiceName) {
      return Stream<PtrResourceRecord>.fromIterable(ptrRecords) as Stream<T>;
    }
    if (T == SrvResourceRecord) {
      final String key = query.fullyQualifiedName;
      return Stream<SrvResourceRecord>.fromIterable(srvResponse[key] ?? <SrvResourceRecord>[])
          as Stream<T>;
    }
    if (T == TxtResourceRecord) {
      final String key = query.fullyQualifiedName;
      return Stream<TxtResourceRecord>.fromIterable(txtResponse[key] ?? <TxtResourceRecord>[])
          as Stream<T>;
    }
    if (T == IPAddressResourceRecord) {
      final String key = query.fullyQualifiedName;
      return Stream<IPAddressResourceRecord>.fromIterable(
            ipResponse[key] ?? <IPAddressResourceRecord>[],
          )
          as Stream<T>;
    }
    throw UnsupportedError('Unsupported query type $T');
  }

  @override
  void stop() {}
}

class TestDeviceManager extends DeviceManager {
  TestDeviceManager({required super.logger});
  List<Device> devices = <Device>[];

  @override
  List<DeviceDiscovery> get deviceDiscoverers {
    final FakePollingDeviceDiscovery discoverer = FakePollingDeviceDiscovery();
    devices.forEach(discoverer.addDevice);
    return <DeviceDiscovery>[discoverer];
  }
}

class FakeTerminal extends Fake implements AnsiTerminal {
  FakeTerminal({this.stdinHasTerminal = true});

  @override
  final bool stdinHasTerminal;

  @override
  bool usesTerminalUi = false;

  @override
  bool singleCharMode = false;

  @override
  Stream<String> get keystrokes => StreamController<String>().stream;
}
