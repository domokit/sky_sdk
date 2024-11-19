// Copyright 2014 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

import 'package:flutter_tools/src/build_info.dart';
import 'package:flutter_tools/src/devfs.dart';
import 'package:flutter_tools/src/device.dart';
import 'package:flutter_tools/src/reporting/reporting.dart';
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
      TestUsage(),
      const NoOpAnalytics(),
    );
  });

  group('signal handling', () {
    late final FlutterDevice flutterDevice;

    setUpAll(() {
      flutterDevice = FakeFlutterDevice(FakeDevice());
    });

    late HotRunner runner;

    setUp(() {
      runner = HotRunner(
        <FlutterDevice>[
          flutterDevice,
        ],
        target: 'main.dart',
        debuggingOptions: DebuggingOptions.disabled(BuildInfo.debug),
        analytics: _FakeAnalytics(),
      );
    });

    testUsingContext('a signal kill without a detach kills the test device', () async {

    });

    testUsingContext('a signal kill with a detach keeps the test device running', () async {

    });
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
  void resetLastCompiled() {}
}

class _FakeFlutterDevice extends Fake implements FlutterDevice {
  @override
  final DevFS? devFS = _FakeDevFS();

  @override
  final FlutterVmService? vmService = _FakeFlutterVmService();
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
