// Copyright 2014 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

import 'dart:async';

import 'package:flutter_tools/src/artifacts.dart';
import 'package:flutter_tools/src/base/async_guard.dart';
import 'package:flutter_tools/src/base/logger.dart';
import 'package:flutter_tools/src/cache.dart';
import 'package:flutter_tools/src/convert.dart';
import 'package:flutter_tools/src/device.dart';
import 'package:flutter_tools/src/ios/devices.dart';
import 'package:flutter_tools/src/ios/ios_deploy.dart';
import 'package:flutter_tools/src/ios/mac.dart';
import 'package:flutter_tools/src/vmservice.dart';
import 'package:test/fake.dart';
import 'package:vm_service/vm_service.dart';

import '../../src/common.dart';
import '../../src/fake_process_manager.dart';
import '../../src/fake_vm_services.dart';

void main() {
  late FakeProcessManager processManager;
  late Artifacts artifacts;
  late Cache fakeCache;
  late BufferLogger logger;
  late String ideviceSyslogPath;

  setUp(() {
    processManager = FakeProcessManager.empty();
    fakeCache = Cache.test(processManager: FakeProcessManager.any());
    artifacts = Artifacts.test();
    logger = BufferLogger.test();
    ideviceSyslogPath = artifacts.getHostArtifact(HostArtifact.idevicesyslog).path;
  });

  group('syslog stream', () {
    testWithoutContext('decodeSyslog decodes a syslog-encoded line', () {
      final String decoded = decodeSyslog(
        r'I \M-b\M^]\M-$\M-o\M-8\M^O syslog '
        r'\M-B\M-/\134_(\M-c\M^C\M^D)_/\M-B\M-/ \M-l\M^F\240!',
      );

      expect(decoded, r'I ❤️ syslog ¯\_(ツ)_/¯ 솠!');
    });

    testWithoutContext('decodeSyslog passes through un-decodeable lines as-is', () {
      final String decoded = decodeSyslog(r'I \M-b\M^O syslog!');

      expect(decoded, r'I \M-b\M^O syslog!');
    });

    testWithoutContext(
      'IOSDeviceLogReader suppresses non-Flutter lines from output with syslog',
      () async {
        processManager.addCommand(
          FakeCommand(
            command: <String>[ideviceSyslogPath, '-u', '1234'],
            stdout: '''
Runner(Flutter)[297] <Notice>: A is for ari
Runner(libsystem_asl.dylib)[297] <Notice>: libMobileGestalt MobileGestaltSupport.m:153: pid 123 (Runner) does not have sandbox access for frZQaeyWLUvLjeuEK43hmg and IS NOT appropriately entitled
Runner(libsystem_asl.dylib)[297] <Notice>: libMobileGestalt MobileGestalt.c:550: no access to InverseDeviceID (see <rdar://problem/11744455>)
Runner(Flutter)[297] <Notice>: I is for ichigo
Runner(UIKit)[297] <Notice>: E is for enpitsu"
''',
          ),
        );
        final DeviceLogReader logReader = IOSDeviceLogReader.test(
          iMobileDevice: IMobileDevice(
            artifacts: artifacts,
            processManager: processManager,
            cache: fakeCache,
            logger: logger,
          ),
        );
        final List<String> lines = await logReader.logLines.toList();

        expect(lines, <String>['A is for ari', 'I is for ichigo']);
      },
    );

    testWithoutContext(
      'IOSDeviceLogReader includes multi-line Flutter logs in the output with syslog',
      () async {
        processManager.addCommand(
          FakeCommand(
            command: <String>[ideviceSyslogPath, '-u', '1234'],
            stdout: '''
Runner(Flutter)[297] <Notice>: This is a multi-line message,
  with another Flutter message following it.
Runner(Flutter)[297] <Notice>: This is a multi-line message,
  with a non-Flutter log message following it.
Runner(libsystem_asl.dylib)[297] <Notice>: libMobileGestalt
''',
          ),
        );
        final DeviceLogReader logReader = IOSDeviceLogReader.test(
          iMobileDevice: IMobileDevice(
            artifacts: artifacts,
            processManager: processManager,
            cache: fakeCache,
            logger: logger,
          ),
        );
        final List<String> lines = await logReader.logLines.toList();

        expect(lines, <String>[
          'This is a multi-line message,',
          '  with another Flutter message following it.',
          'This is a multi-line message,',
          '  with a non-Flutter log message following it.',
        ]);
      },
    );

    testWithoutContext('includes multi-line Flutter logs in the output', () async {
      processManager.addCommand(
        FakeCommand(
          command: <String>[ideviceSyslogPath, '-u', '1234'],
          stdout: '''
Runner(Flutter)[297] <Notice>: This is a multi-line message,
  with another Flutter message following it.
Runner(Flutter)[297] <Notice>: This is a multi-line message,
  with a non-Flutter log message following it.
Runner(libsystem_asl.dylib)[297] <Notice>: libMobileGestalt
''',
        ),
      );

      final DeviceLogReader logReader = IOSDeviceLogReader.test(
        iMobileDevice: IMobileDevice(
          artifacts: artifacts,
          processManager: processManager,
          cache: fakeCache,
          logger: logger,
        ),
      );
      final List<String> lines = await logReader.logLines.toList();

      expect(lines, <String>[
        'This is a multi-line message,',
        '  with another Flutter message following it.',
        'This is a multi-line message,',
        '  with a non-Flutter log message following it.',
      ]);
    });
  });

  group('VM service', () {
    testWithoutContext('IOSDeviceLogReader can listen to VM Service logs', () async {
      final Event stdoutEvent = Event(
        kind: 'Stdout',
        timestamp: 0,
        bytes: base64.encode(utf8.encode('  This is a message ')),
      );
      final Event stderrEvent = Event(
        kind: 'Stderr',
        timestamp: 0,
        bytes: base64.encode(utf8.encode('  And this is an error ')),
      );
      final FlutterVmService vmService =
          FakeVmServiceHost(
            requests: <VmServiceExpectation>[
              const FakeVmServiceRequest(
                method: 'streamListen',
                args: <String, Object>{'streamId': 'Debug'},
              ),
              const FakeVmServiceRequest(
                method: 'streamListen',
                args: <String, Object>{'streamId': 'Stdout'},
              ),
              const FakeVmServiceRequest(
                method: 'streamListen',
                args: <String, Object>{'streamId': 'Stderr'},
              ),
              FakeVmServiceStreamResponse(event: stdoutEvent, streamId: 'Stdout'),
              FakeVmServiceStreamResponse(event: stderrEvent, streamId: 'Stderr'),
            ],
          ).vmService;
      final DeviceLogReader logReader = IOSDeviceLogReader.test(
        useSyslog: false,
        iMobileDevice: IMobileDevice(
          artifacts: artifacts,
          processManager: processManager,
          cache: fakeCache,
          logger: logger,
        ),
      );
      await logReader.provideVmService(vmService);

      // Wait for stream listeners to fire.
      await expectLater(
        logReader.logLines,
        emitsInAnyOrder(<Matcher>[
          equals('  This is a message '),
          equals('  And this is an error '),
        ]),
      );
    });

    testWithoutContext(
      'IOSDeviceLogReader ignores VM Service logs when attached to and received flutter logs from debugger',
      () async {
        final Event stdoutEvent = Event(
          kind: 'Stdout',
          timestamp: 0,
          bytes: base64.encode(utf8.encode('  This is a message ')),
        );
        final Event stderrEvent = Event(
          kind: 'Stderr',
          timestamp: 0,
          bytes: base64.encode(utf8.encode('  And this is an error ')),
        );
        final FlutterVmService vmService =
            FakeVmServiceHost(
              requests: <VmServiceExpectation>[
                const FakeVmServiceRequest(
                  method: 'streamListen',
                  args: <String, Object>{'streamId': 'Debug'},
                ),
                const FakeVmServiceRequest(
                  method: 'streamListen',
                  args: <String, Object>{'streamId': 'Stdout'},
                ),
                const FakeVmServiceRequest(
                  method: 'streamListen',
                  args: <String, Object>{'streamId': 'Stderr'},
                ),
                FakeVmServiceStreamResponse(event: stdoutEvent, streamId: 'Stdout'),
                FakeVmServiceStreamResponse(event: stderrEvent, streamId: 'Stderr'),
              ],
            ).vmService;
        final IOSDeviceLogReader logReader = IOSDeviceLogReader.test(
          useSyslog: false,
          iMobileDevice: IMobileDevice(
            artifacts: artifacts,
            processManager: processManager,
            cache: fakeCache,
            logger: logger,
          ),
        );
        await logReader.provideVmService(vmService);

        final FakeIOSDeployDebugger iosDeployDebugger = FakeIOSDeployDebugger();
        iosDeployDebugger.debuggerAttached = true;

        final Stream<String> debuggingLogs = Stream<String>.fromIterable(<String>[
          'flutter: Message from debugger',
        ]);
        iosDeployDebugger.logLines = debuggingLogs;
        logReader.debuggerStream = iosDeployDebugger;

        // Wait for stream listeners to fire.
        await expectLater(
          logReader.logLines,
          emitsInAnyOrder(<Matcher>[equals('flutter: Message from debugger')]),
        );
      },
    );
  });

  group('debugger stream', () {
    testWithoutContext('IOSDeviceLogReader removes metadata prefix from lldb output', () async {
      final Stream<String> debuggingLogs = Stream<String>.fromIterable(<String>[
        '2020-09-15 19:15:10.931434-0700 Runner[541:226276] Did finish launching.',
        '2020-09-15 19:15:10.931434-0700 Runner[541:226276] [Category] Did finish launching from logging category.',
        'stderr from dart',
        '',
      ]);

      final IOSDeviceLogReader logReader = IOSDeviceLogReader.test(
        iMobileDevice: IMobileDevice(
          artifacts: artifacts,
          processManager: processManager,
          cache: fakeCache,
          logger: logger,
        ),
        useSyslog: false,
      );
      final FakeIOSDeployDebugger iosDeployDebugger = FakeIOSDeployDebugger();
      iosDeployDebugger.logLines = debuggingLogs;
      logReader.debuggerStream = iosDeployDebugger;
      final Future<List<String>> logLines = logReader.logLines.toList();

      expect(await logLines, <String>[
        'Did finish launching.',
        '[Category] Did finish launching from logging category.',
        'stderr from dart',
        '',
      ]);
    });

    testWithoutContext('errors on debugger stream closes log stream', () async {
      final Stream<String> debuggingLogs = Stream<String>.error('ios-deploy error');
      final IOSDeviceLogReader logReader = IOSDeviceLogReader.test(
        iMobileDevice: IMobileDevice(
          artifacts: artifacts,
          processManager: processManager,
          cache: fakeCache,
          logger: logger,
        ),
        useSyslog: false,
      );
      final Completer<void> streamComplete = Completer<void>();
      final FakeIOSDeployDebugger iosDeployDebugger = FakeIOSDeployDebugger();
      iosDeployDebugger.logLines = debuggingLogs;
      logReader.logLines.listen(null, onError: (Object error) => streamComplete.complete());
      logReader.debuggerStream = iosDeployDebugger;

      await streamComplete.future;
    });

    testWithoutContext('detaches debugger', () async {
      final IOSDeviceLogReader logReader = IOSDeviceLogReader.test(
        iMobileDevice: IMobileDevice(
          artifacts: artifacts,
          processManager: processManager,
          cache: fakeCache,
          logger: logger,
        ),
        useSyslog: false,
      );
      final FakeIOSDeployDebugger iosDeployDebugger = FakeIOSDeployDebugger();
      logReader.debuggerStream = iosDeployDebugger;

      logReader.dispose();
      expect(iosDeployDebugger.detached, true);
    });

    testWithoutContext('Does not throw if debuggerStream set after logReader closed', () async {
      final Stream<String> debuggingLogs = Stream<String>.fromIterable(<String>[
        '2020-09-15 19:15:10.931434-0700 Runner[541:226276] Did finish launching.',
        '2020-09-15 19:15:10.931434-0700 Runner[541:226276] [Category] Did finish launching from logging category.',
        'stderr from dart',
        '',
      ]);

      final IOSDeviceLogReader logReader = IOSDeviceLogReader.test(
        iMobileDevice: IMobileDevice(
          artifacts: artifacts,
          processManager: processManager,
          cache: fakeCache,
          logger: logger,
        ),
        useSyslog: false,
      );
      Object? exception;
      StackTrace? trace;
      await asyncGuard(
        () async {
          await logReader.linesController.close();
          final FakeIOSDeployDebugger iosDeployDebugger = FakeIOSDeployDebugger();
          iosDeployDebugger.logLines = debuggingLogs;
          logReader.debuggerStream = iosDeployDebugger;
          await logReader.logLines.drain<void>();
        },
        onError: (Object err, StackTrace stackTrace) {
          exception = err;
          trace = stackTrace;
        },
      );
      expect(exception, isNull, reason: trace.toString());
    });
  });

  group('Determine which loggers to use', () {
    testWithoutContext('for physically attached CoreDevice', () {
      final IOSDeviceLogReader logReader = IOSDeviceLogReader.test(
        iMobileDevice: IMobileDevice(
          artifacts: artifacts,
          processManager: processManager,
          cache: fakeCache,
          logger: logger,
        ),
        majorSdkVersion: 17,
        isCoreDevice: true,
      );

      expect(logReader.useSyslogLogging, isTrue);
      expect(logReader.useUnifiedLogging, isTrue);
      expect(logReader.useIOSDeployLogging, isFalse);
      expect(logReader.logSources.primarySource, IOSDeviceLogSource.idevicesyslog);
      expect(logReader.logSources.fallbackSource, IOSDeviceLogSource.unifiedLogging);
    });

    testWithoutContext('for wirelessly attached CoreDevice', () {
      final IOSDeviceLogReader logReader = IOSDeviceLogReader.test(
        iMobileDevice: IMobileDevice(
          artifacts: artifacts,
          processManager: processManager,
          cache: fakeCache,
          logger: logger,
        ),
        majorSdkVersion: 17,
        isCoreDevice: true,
        isWirelesslyConnected: true,
      );

      expect(logReader.useSyslogLogging, isFalse);
      expect(logReader.useUnifiedLogging, isTrue);
      expect(logReader.useIOSDeployLogging, isFalse);
      expect(logReader.logSources.primarySource, IOSDeviceLogSource.unifiedLogging);
      expect(logReader.logSources.fallbackSource, isNull);
    });

    testWithoutContext('for iOS 12 or less device', () {
      final IOSDeviceLogReader logReader = IOSDeviceLogReader.test(
        iMobileDevice: IMobileDevice(
          artifacts: artifacts,
          processManager: processManager,
          cache: fakeCache,
          logger: logger,
        ),
        majorSdkVersion: 12,
      );

      expect(logReader.useSyslogLogging, isTrue);
      expect(logReader.useUnifiedLogging, isFalse);
      expect(logReader.useIOSDeployLogging, isFalse);
      expect(logReader.logSources.primarySource, IOSDeviceLogSource.idevicesyslog);
      expect(logReader.logSources.fallbackSource, isNull);
    });

    testWithoutContext(
      'for iOS 13 or greater non-CoreDevice and _iosDeployDebugger not attached',
      () {
        final IOSDeviceLogReader logReader = IOSDeviceLogReader.test(
          iMobileDevice: IMobileDevice(
            artifacts: artifacts,
            processManager: processManager,
            cache: fakeCache,
            logger: logger,
          ),
          majorSdkVersion: 13,
        );

        expect(logReader.useSyslogLogging, isFalse);
        expect(logReader.useUnifiedLogging, isTrue);
        expect(logReader.useIOSDeployLogging, isTrue);
        expect(logReader.logSources.primarySource, IOSDeviceLogSource.iosDeploy);
        expect(logReader.logSources.fallbackSource, IOSDeviceLogSource.unifiedLogging);
      },
    );

    testWithoutContext(
      'for iOS 13 or greater non-CoreDevice, _iosDeployDebugger not attached, and VM is connected',
      () async {
        final IOSDeviceLogReader logReader = IOSDeviceLogReader.test(
          iMobileDevice: IMobileDevice(
            artifacts: artifacts,
            processManager: processManager,
            cache: fakeCache,
            logger: logger,
          ),
          majorSdkVersion: 13,
        );

        final FlutterVmService vmService =
            FakeVmServiceHost(
              requests: <VmServiceExpectation>[
                const FakeVmServiceRequest(
                  method: 'streamListen',
                  args: <String, Object>{'streamId': 'Debug'},
                ),
                const FakeVmServiceRequest(
                  method: 'streamListen',
                  args: <String, Object>{'streamId': 'Stdout'},
                ),
                const FakeVmServiceRequest(
                  method: 'streamListen',
                  args: <String, Object>{'streamId': 'Stderr'},
                ),
              ],
            ).vmService;

        await logReader.provideVmService(vmService);

        expect(logReader.useSyslogLogging, isFalse);
        expect(logReader.useUnifiedLogging, isTrue);
        expect(logReader.useIOSDeployLogging, isTrue);
        expect(logReader.logSources.primarySource, IOSDeviceLogSource.unifiedLogging);
        expect(logReader.logSources.fallbackSource, IOSDeviceLogSource.iosDeploy);
      },
    );

    testWithoutContext(
      'for iOS 13 or greater non-CoreDevice and _iosDeployDebugger is attached',
      () async {
        final IOSDeviceLogReader logReader = IOSDeviceLogReader.test(
          iMobileDevice: IMobileDevice(
            artifacts: artifacts,
            processManager: processManager,
            cache: fakeCache,
            logger: logger,
          ),
          majorSdkVersion: 13,
        );

        final FakeIOSDeployDebugger iosDeployDebugger = FakeIOSDeployDebugger();
        iosDeployDebugger.debuggerAttached = true;
        logReader.debuggerStream = iosDeployDebugger;

        final FlutterVmService vmService =
            FakeVmServiceHost(
              requests: <VmServiceExpectation>[
                const FakeVmServiceRequest(
                  method: 'streamListen',
                  args: <String, Object>{'streamId': 'Debug'},
                ),
                const FakeVmServiceRequest(
                  method: 'streamListen',
                  args: <String, Object>{'streamId': 'Stdout'},
                ),
                const FakeVmServiceRequest(
                  method: 'streamListen',
                  args: <String, Object>{'streamId': 'Stderr'},
                ),
              ],
            ).vmService;

        await logReader.provideVmService(vmService);

        expect(logReader.useSyslogLogging, isFalse);
        expect(logReader.useUnifiedLogging, isTrue);
        expect(logReader.useIOSDeployLogging, isTrue);
        expect(logReader.logSources.primarySource, IOSDeviceLogSource.iosDeploy);
        expect(logReader.logSources.fallbackSource, IOSDeviceLogSource.unifiedLogging);
      },
    );

    testWithoutContext('for iOS 16 or greater non-CoreDevice', () {
      final IOSDeviceLogReader logReader = IOSDeviceLogReader.test(
        iMobileDevice: IMobileDevice(
          artifacts: artifacts,
          processManager: processManager,
          cache: fakeCache,
          logger: logger,
        ),
        majorSdkVersion: 16,
      );

      final FakeIOSDeployDebugger iosDeployDebugger = FakeIOSDeployDebugger();
      iosDeployDebugger.debuggerAttached = true;
      logReader.debuggerStream = iosDeployDebugger;

      expect(logReader.useSyslogLogging, isFalse);
      expect(logReader.useUnifiedLogging, isTrue);
      expect(logReader.useIOSDeployLogging, isTrue);
      expect(logReader.logSources.primarySource, IOSDeviceLogSource.iosDeploy);
      expect(logReader.logSources.fallbackSource, IOSDeviceLogSource.unifiedLogging);
    });

    testWithoutContext('for iOS 16 or greater non-CoreDevice in CI', () {
      final IOSDeviceLogReader logReader = IOSDeviceLogReader.test(
        iMobileDevice: IMobileDevice(
          artifacts: artifacts,
          processManager: processManager,
          cache: fakeCache,
          logger: logger,
        ),
        usingCISystem: true,
        majorSdkVersion: 16,
      );

      expect(logReader.useSyslogLogging, isTrue);
      expect(logReader.useUnifiedLogging, isFalse);
      expect(logReader.useIOSDeployLogging, isTrue);
      expect(logReader.logSources.primarySource, IOSDeviceLogSource.iosDeploy);
      expect(logReader.logSources.fallbackSource, IOSDeviceLogSource.idevicesyslog);
    });

    group('when useSyslogLogging', () {
      testWithoutContext('is true syslog sends flutter messages to stream', () async {
        processManager.addCommand(
          FakeCommand(
            command: <String>[ideviceSyslogPath, '-u', '1234'],
            stdout: '''
  Runner(Flutter)[297] <Notice>: A is for ari
  Runner(Flutter)[297] <Notice>: I is for ichigo
  May 30 13:56:28 Runner(Flutter)[2037] <Notice>: flutter: The Dart VM service is listening on http://127.0.0.1:63098/35ZezGIQLnw=/
  May 30 13:56:28 Runner(Flutter)[2037] <Notice>: flutter: This is a test
  May 30 13:56:28 Runner(Flutter)[2037] <Notice>: [VERBOSE-2:FlutterDarwinContextMetalImpeller.mm(39)] Using the Impeller rendering backend.
  ''',
          ),
        );
        final IOSDeviceLogReader logReader = IOSDeviceLogReader.test(
          iMobileDevice: IMobileDevice(
            artifacts: artifacts,
            processManager: processManager,
            cache: fakeCache,
            logger: logger,
          ),
          usingCISystem: true,
          majorSdkVersion: 16,
        );
        final List<String> lines = await logReader.logLines.toList();

        expect(logReader.useSyslogLogging, isTrue);
        expect(processManager, hasNoRemainingExpectations);
        expect(lines, <String>[
          'flutter: The Dart VM service is listening on http://127.0.0.1:63098/35ZezGIQLnw=/',
          'flutter: This is a test',
        ]);
      });

      testWithoutContext('is false syslog does not send flutter messages to stream', () async {
        final IOSDeviceLogReader logReader = IOSDeviceLogReader.test(
          iMobileDevice: IMobileDevice(
            artifacts: artifacts,
            processManager: processManager,
            cache: fakeCache,
            logger: logger,
          ),
          majorSdkVersion: 16,
        );

        final FakeIOSDeployDebugger iosDeployDebugger = FakeIOSDeployDebugger();
        iosDeployDebugger.logLines = Stream<String>.fromIterable(<String>[]);
        logReader.debuggerStream = iosDeployDebugger;

        final List<String> lines = await logReader.logLines.toList();

        expect(logReader.useSyslogLogging, isFalse);
        expect(processManager, hasNoRemainingExpectations);
        expect(lines, isEmpty);
      });
    });

    group('when useIOSDeployLogging', () {
      testWithoutContext('is true ios-deploy sends flutter messages to stream', () async {
        final IOSDeviceLogReader logReader = IOSDeviceLogReader.test(
          iMobileDevice: IMobileDevice(
            artifacts: artifacts,
            processManager: processManager,
            cache: fakeCache,
            logger: logger,
          ),
          majorSdkVersion: 16,
        );

        final FakeIOSDeployDebugger iosDeployDebugger = FakeIOSDeployDebugger();
        final Stream<String> debuggingLogs = Stream<String>.fromIterable(<String>[
          'flutter: Message from debugger',
        ]);
        iosDeployDebugger.logLines = debuggingLogs;
        logReader.debuggerStream = iosDeployDebugger;

        final List<String> lines = await logReader.logLines.toList();

        expect(logReader.useIOSDeployLogging, isTrue);
        expect(processManager, hasNoRemainingExpectations);
        expect(lines, <String>['flutter: Message from debugger']);
      });

      testWithoutContext('is false ios-deploy does not send flutter messages to stream', () async {
        final IOSDeviceLogReader logReader = IOSDeviceLogReader.test(
          iMobileDevice: IMobileDevice(
            artifacts: artifacts,
            processManager: FakeProcessManager.any(),
            cache: fakeCache,
            logger: logger,
          ),
          majorSdkVersion: 12,
        );

        final FakeIOSDeployDebugger iosDeployDebugger = FakeIOSDeployDebugger();
        final Stream<String> debuggingLogs = Stream<String>.fromIterable(<String>[
          'flutter: Message from debugger',
        ]);
        iosDeployDebugger.logLines = debuggingLogs;
        logReader.debuggerStream = iosDeployDebugger;

        final List<String> lines = await logReader.logLines.toList();

        expect(logReader.useIOSDeployLogging, isFalse);
        expect(processManager, hasNoRemainingExpectations);
        expect(lines, isEmpty);
      });
    });

    group('when useUnifiedLogging', () {
      testWithoutContext('is true Dart VM sends flutter messages to stream', () async {
        final Event stdoutEvent = Event(
          kind: 'Stdout',
          timestamp: 0,
          bytes: base64.encode(utf8.encode('flutter: A flutter message')),
        );
        final Event stderrEvent = Event(
          kind: 'Stderr',
          timestamp: 0,
          bytes: base64.encode(utf8.encode('flutter: A second flutter message')),
        );
        final FlutterVmService vmService =
            FakeVmServiceHost(
              requests: <VmServiceExpectation>[
                const FakeVmServiceRequest(
                  method: 'streamListen',
                  args: <String, Object>{'streamId': 'Debug'},
                ),
                const FakeVmServiceRequest(
                  method: 'streamListen',
                  args: <String, Object>{'streamId': 'Stdout'},
                ),
                const FakeVmServiceRequest(
                  method: 'streamListen',
                  args: <String, Object>{'streamId': 'Stderr'},
                ),
                FakeVmServiceStreamResponse(event: stdoutEvent, streamId: 'Stdout'),
                FakeVmServiceStreamResponse(event: stderrEvent, streamId: 'Stderr'),
              ],
            ).vmService;
        final IOSDeviceLogReader logReader = IOSDeviceLogReader.test(
          useSyslog: false,
          iMobileDevice: IMobileDevice(
            artifacts: artifacts,
            processManager: processManager,
            cache: fakeCache,
            logger: logger,
          ),
        );
        await logReader.provideVmService(vmService);

        // Wait for stream listeners to fire.
        expect(logReader.useUnifiedLogging, isTrue);
        expect(processManager, hasNoRemainingExpectations);
        await expectLater(
          logReader.logLines,
          emitsInAnyOrder(<Matcher>[
            equals('flutter: A flutter message'),
            equals('flutter: A second flutter message'),
          ]),
        );
      });

      testWithoutContext('is false Dart VM does not send flutter messages to stream', () async {
        final Event stdoutEvent = Event(
          kind: 'Stdout',
          timestamp: 0,
          bytes: base64.encode(utf8.encode('flutter: A flutter message')),
        );
        final Event stderrEvent = Event(
          kind: 'Stderr',
          timestamp: 0,
          bytes: base64.encode(utf8.encode('flutter: A second flutter message')),
        );
        final FlutterVmService vmService =
            FakeVmServiceHost(
              requests: <VmServiceExpectation>[
                const FakeVmServiceRequest(
                  method: 'streamListen',
                  args: <String, Object>{'streamId': 'Debug'},
                ),
                const FakeVmServiceRequest(
                  method: 'streamListen',
                  args: <String, Object>{'streamId': 'Stdout'},
                ),
                const FakeVmServiceRequest(
                  method: 'streamListen',
                  args: <String, Object>{'streamId': 'Stderr'},
                ),
                FakeVmServiceStreamResponse(event: stdoutEvent, streamId: 'Stdout'),
                FakeVmServiceStreamResponse(event: stderrEvent, streamId: 'Stderr'),
              ],
            ).vmService;
        final IOSDeviceLogReader logReader = IOSDeviceLogReader.test(
          iMobileDevice: IMobileDevice(
            artifacts: artifacts,
            processManager: FakeProcessManager.any(),
            cache: fakeCache,
            logger: logger,
          ),
          majorSdkVersion: 12,
        );
        await logReader.provideVmService(vmService);

        final List<String> lines = await logReader.logLines.toList();

        // Wait for stream listeners to fire.
        expect(logReader.useUnifiedLogging, isFalse);
        expect(processManager, hasNoRemainingExpectations);
        expect(lines, isEmpty);
      });
    });

    group('and when to exclude logs:', () {
      testWithoutContext(
        'all primary messages are included except if fallback sent flutter message first',
        () async {
          final IOSDeviceLogReader logReader = IOSDeviceLogReader.test(
            iMobileDevice: IMobileDevice(
              artifacts: artifacts,
              processManager: FakeProcessManager.any(),
              cache: fakeCache,
              logger: logger,
            ),
            usingCISystem: true,
            majorSdkVersion: 16,
          );

          expect(logReader.useSyslogLogging, isTrue);
          expect(logReader.useIOSDeployLogging, isTrue);
          expect(logReader.logSources.primarySource, IOSDeviceLogSource.iosDeploy);
          expect(logReader.logSources.fallbackSource, IOSDeviceLogSource.idevicesyslog);

          final Future<List<String>> logLines = logReader.logLines.toList();

          logReader.addToLinesController(
            'flutter: The Dart VM service is listening on http://127.0.0.1:63098/35ZezGIQLnw=/',
            IOSDeviceLogSource.idevicesyslog,
          );
          // Will be excluded because was already added by fallback.
          logReader.addToLinesController(
            'flutter: The Dart VM service is listening on http://127.0.0.1:63098/35ZezGIQLnw=/',
            IOSDeviceLogSource.iosDeploy,
          );
          logReader.addToLinesController(
            'A second non-flutter message',
            IOSDeviceLogSource.iosDeploy,
          );
          logReader.addToLinesController(
            'flutter: Another flutter message',
            IOSDeviceLogSource.iosDeploy,
          );
          final List<String> lines = await logLines;

          expect(
            lines,
            containsAllInOrder(<String>[
              'flutter: The Dart VM service is listening on http://127.0.0.1:63098/35ZezGIQLnw=/', // from idevicesyslog
              'A second non-flutter message', // from iosDeploy
              'flutter: Another flutter message', // from iosDeploy
            ]),
          );
        },
      );

      testWithoutContext('all primary messages are included when there is no fallback', () async {
        final IOSDeviceLogReader logReader = IOSDeviceLogReader.test(
          iMobileDevice: IMobileDevice(
            artifacts: artifacts,
            processManager: FakeProcessManager.any(),
            cache: fakeCache,
            logger: logger,
          ),
          majorSdkVersion: 12,
        );

        expect(logReader.useSyslogLogging, isTrue);
        expect(logReader.logSources.primarySource, IOSDeviceLogSource.idevicesyslog);
        expect(logReader.logSources.fallbackSource, isNull);

        final Future<List<String>> logLines = logReader.logLines.toList();

        logReader.addToLinesController(
          'flutter: The Dart VM service is listening on http://127.0.0.1:63098/35ZezGIQLnw=/',
          IOSDeviceLogSource.idevicesyslog,
        );
        logReader.addToLinesController('A non-flutter message', IOSDeviceLogSource.idevicesyslog);
        logReader.addToLinesController('A non-flutter message', IOSDeviceLogSource.idevicesyslog);
        logReader.addToLinesController(
          'flutter: A flutter message',
          IOSDeviceLogSource.idevicesyslog,
        );
        logReader.addToLinesController(
          'flutter: A flutter message',
          IOSDeviceLogSource.idevicesyslog,
        );
        final List<String> lines = await logLines;

        expect(
          lines,
          containsAllInOrder(<String>[
            'flutter: The Dart VM service is listening on http://127.0.0.1:63098/35ZezGIQLnw=/',
            'A non-flutter message',
            'A non-flutter message',
            'flutter: A flutter message',
            'flutter: A flutter message',
          ]),
        );
      });

      testWithoutContext(
        'primary messages are not added if fallback already added them, otherwise duplicates are allowed',
        () async {
          final IOSDeviceLogReader logReader = IOSDeviceLogReader.test(
            iMobileDevice: IMobileDevice(
              artifacts: artifacts,
              processManager: FakeProcessManager.any(),
              cache: fakeCache,
              logger: logger,
            ),
            usingCISystem: true,
            majorSdkVersion: 16,
          );

          expect(logReader.useSyslogLogging, isTrue);
          expect(logReader.useIOSDeployLogging, isTrue);
          expect(logReader.logSources.primarySource, IOSDeviceLogSource.iosDeploy);
          expect(logReader.logSources.fallbackSource, IOSDeviceLogSource.idevicesyslog);

          final Future<List<String>> logLines = logReader.logLines.toList();

          logReader.addToLinesController(
            'flutter: A flutter message',
            IOSDeviceLogSource.idevicesyslog,
          );
          logReader.addToLinesController(
            'flutter: A flutter message',
            IOSDeviceLogSource.idevicesyslog,
          );
          logReader.addToLinesController('A non-flutter message', IOSDeviceLogSource.iosDeploy);
          logReader.addToLinesController('A non-flutter message', IOSDeviceLogSource.iosDeploy);
          // Will be excluded because was already added by fallback.
          logReader.addToLinesController(
            'flutter: A flutter message',
            IOSDeviceLogSource.iosDeploy,
          );
          // Will be excluded because was already added by fallback.
          logReader.addToLinesController(
            'flutter: A flutter message',
            IOSDeviceLogSource.iosDeploy,
          );
          // Will be included because, although the message is the same, the
          // fallback only added it twice so this third one is considered new.
          logReader.addToLinesController(
            'flutter: A flutter message',
            IOSDeviceLogSource.iosDeploy,
          );

          final List<String> lines = await logLines;

          expect(
            lines,
            containsAllInOrder(<String>[
              'flutter: A flutter message', // from idevicesyslog
              'flutter: A flutter message', // from idevicesyslog
              'A non-flutter message', // from iosDeploy
              'A non-flutter message', // from iosDeploy
              'flutter: A flutter message', // from iosDeploy
            ]),
          );
        },
      );

      testWithoutContext(
        'flutter fallback messages are included until a primary flutter message is received',
        () async {
          final IOSDeviceLogReader logReader = IOSDeviceLogReader.test(
            iMobileDevice: IMobileDevice(
              artifacts: artifacts,
              processManager: FakeProcessManager.any(),
              cache: fakeCache,
              logger: logger,
            ),
            usingCISystem: true,
            majorSdkVersion: 16,
          );

          expect(logReader.useSyslogLogging, isTrue);
          expect(logReader.useIOSDeployLogging, isTrue);
          expect(logReader.logSources.primarySource, IOSDeviceLogSource.iosDeploy);
          expect(logReader.logSources.fallbackSource, IOSDeviceLogSource.idevicesyslog);

          final Future<List<String>> logLines = logReader.logLines.toList();

          logReader.addToLinesController(
            'flutter: The Dart VM service is listening on http://127.0.0.1:63098/35ZezGIQLnw=/',
            IOSDeviceLogSource.idevicesyslog,
          );
          logReader.addToLinesController(
            'A second non-flutter message',
            IOSDeviceLogSource.iosDeploy,
          );
          // Will be included because the first log from primary source wasn't a
          // flutter log.
          logReader.addToLinesController(
            'flutter: A flutter message',
            IOSDeviceLogSource.idevicesyslog,
          );
          // Will be excluded because was already added by fallback, however, it
          // will be used to determine a flutter log was received by the primary source.
          logReader.addToLinesController(
            'flutter: The Dart VM service is listening on http://127.0.0.1:63098/35ZezGIQLnw=/',
            IOSDeviceLogSource.iosDeploy,
          );
          // Will be excluded because flutter log from primary was received.
          logReader.addToLinesController(
            'flutter: A third flutter message',
            IOSDeviceLogSource.idevicesyslog,
          );

          final List<String> lines = await logLines;

          expect(
            lines,
            containsAllInOrder(<String>[
              'flutter: The Dart VM service is listening on http://127.0.0.1:63098/35ZezGIQLnw=/', // from idevicesyslog
              'A second non-flutter message', // from iosDeploy
              'flutter: A flutter message', // from idevicesyslog
            ]),
          );
        },
      );

      testWithoutContext('non-flutter fallback messages are not included', () async {
        final IOSDeviceLogReader logReader = IOSDeviceLogReader.test(
          iMobileDevice: IMobileDevice(
            artifacts: artifacts,
            processManager: FakeProcessManager.any(),
            cache: fakeCache,
            logger: logger,
          ),
          usingCISystem: true,
          majorSdkVersion: 16,
        );

        expect(logReader.useSyslogLogging, isTrue);
        expect(logReader.useIOSDeployLogging, isTrue);
        expect(logReader.logSources.primarySource, IOSDeviceLogSource.iosDeploy);
        expect(logReader.logSources.fallbackSource, IOSDeviceLogSource.idevicesyslog);

        final Future<List<String>> logLines = logReader.logLines.toList();

        logReader.addToLinesController(
          'flutter: A flutter message',
          IOSDeviceLogSource.idevicesyslog,
        );
        // Will be excluded because it's from fallback and not a flutter message.
        logReader.addToLinesController('A non-flutter message', IOSDeviceLogSource.idevicesyslog);

        final List<String> lines = await logLines;

        expect(lines, containsAllInOrder(<String>['flutter: A flutter message']));
      });
    });
  });
}

class FakeIOSDeployDebugger extends Fake implements IOSDeployDebugger {
  bool detached = false;

  @override
  bool debuggerAttached = false;

  @override
  Stream<String> logLines = const Stream<String>.empty();

  @override
  Future<void> detach() async {
    detached = true;
  }
}
