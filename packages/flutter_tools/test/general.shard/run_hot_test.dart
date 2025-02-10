// Copyright 2014 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

import 'package:file/memory.dart';
import 'package:flutter_tools/src/base/file_system.dart';
import 'package:flutter_tools/src/build_info.dart';
import 'package:flutter_tools/src/compile.dart';
import 'package:flutter_tools/src/devfs.dart';
import 'package:flutter_tools/src/device.dart';
import 'package:flutter_tools/src/resident_devtools_handler.dart';
import 'package:flutter_tools/src/resident_runner.dart';
import 'package:flutter_tools/src/run_hot.dart';
import 'package:flutter_tools/src/vmservice.dart';
import 'package:test/fake.dart';
import 'package:unified_analytics/unified_analytics.dart';
import 'package:vm_service/vm_service.dart' as vm_service;

import '../src/common.dart';
import '../src/context.dart';
import 'hot_shared.dart';

void main() {
  testWithoutContext('defaultReloadSourcesHelper() handles empty DeviceReloadReports)', () {
    defaultReloadSourcesHelper(
      _FakeHotRunner(),
      <FlutterDevice?>[_FakeFlutterDevice()],
      false,
      const <String, dynamic>{},
      'android',
      'flutter-sdk',
      false,
      'test-reason',
      const NoOpAnalytics(),
    );
  });

  group('signal handling', () {
    late _FakeHotCompatibleFlutterDevice flutterDevice;
    late MemoryFileSystem fileSystem;

    setUp(() {
      flutterDevice = _FakeHotCompatibleFlutterDevice(FakeDevice());
      fileSystem = MemoryFileSystem.test();
    });

    testUsingContext(
      'kills the test device',
      () async {
        final HotRunner runner = HotRunner(
          <FlutterDevice>[flutterDevice],
          target: 'main.dart',
          debuggingOptions: DebuggingOptions.disabled(BuildInfo.debug),
          analytics: _FakeAnalytics(),
          devtoolsHandler: createNoOpHandler,
        );

        await runner.run();
        await runner.cleanupAfterSignal();
        expect(flutterDevice.wasExited, true);
        expect((flutterDevice.device.dds as FakeDartDevelopmentService).wasShutdown, true);
        expect((runner.residentDevtoolsHandler! as NoOpDevtoolsHandler).wasShutdown, true);
      },
      overrides: <Type, Generator>{
        FileSystem: () => fileSystem,
        ProcessManager: FakeProcessManager.empty,
      },
    );

    testUsingContext(
      'kill with a detach keeps the test device running',
      () async {
        final HotRunner runner = HotRunner(
          <FlutterDevice>[flutterDevice],
          target: 'main.dart',
          debuggingOptions: DebuggingOptions.disabled(BuildInfo.debug),
          analytics: _FakeAnalytics(),
        );

        await runner.run();
        await runner.detach();
        await runner.cleanupAfterSignal();
        expect(flutterDevice.wasExited, false);
      },
      overrides: <Type, Generator>{
        FileSystem: () => fileSystem,
        ProcessManager: FakeProcessManager.empty,
      },
    );

    testUsingContext(
      'kill on an attached device keeps the test device running',
      () async {
        final HotRunner runner = HotRunner(
          <FlutterDevice>[flutterDevice],
          target: 'main.dart',
          debuggingOptions: DebuggingOptions.disabled(BuildInfo.debug),
          analytics: _FakeAnalytics(),
        );

        await runner.attach();
        await runner.cleanupAfterSignal();
        expect(flutterDevice.wasExited, false);
      },
      overrides: <Type, Generator>{
        FileSystem: () => fileSystem,
        ProcessManager: FakeProcessManager.empty,
      },
    );
  });
}

class _FakeAnalytics extends Fake implements Analytics {
  @override
  void send(Event event) {}
}

class _FakeHotRunner extends Fake implements HotRunner {}

class _FakeDevFS extends Fake implements DevFS {
  @override
  final Uri? baseUri = Uri();

  @override
  Future<void> destroy() async {}

  @override
  void resetLastCompiled() {}
}

class _FakeFlutterDevice extends Fake implements FlutterDevice {
  @override
  final DevFS? devFS = _FakeDevFS();

  @override
  final FlutterVmService? vmService = _FakeFlutterVmService();
}

class _FakeHotCompatibleFlutterDevice extends Fake implements FlutterDevice {
  _FakeHotCompatibleFlutterDevice(this.device);

  @override
  final Device device;

  @override
  DevFS? devFS = _FakeDevFS();

  @override
  ResidentCompiler? get generator => null;

  @override
  Future<int> runHot({required HotRunner hotRunner, String? route}) async {
    return 0;
  }

  @override
  Future<void> stopEchoingDeviceLog() async {}

  @override
  Future<void> exitApps({Duration timeoutDelay = const Duration(seconds: 10)}) async {
    wasExited = true;
  }

  bool wasExited = false;
}

class _FakeFlutterVmService extends Fake implements FlutterVmService {
  @override
  final vm_service.VmService service = _FakeVmService();
}

class _FakeVmService extends Fake implements vm_service.VmService {
  @override
  Future<_FakeVm> getVM() async => _FakeVm();
}

class _FakeVm extends Fake implements vm_service.VM {
  final List<vm_service.IsolateRef> _isolates = <vm_service.IsolateRef>[];

  @override
  List<vm_service.IsolateRef>? get isolates => _isolates;
}
