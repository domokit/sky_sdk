// Copyright 2014 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

import 'package:file/memory.dart';
import 'package:flutter_tools/src/base/file_system.dart';
import 'package:flutter_tools/src/base/io.dart';
import 'package:flutter_tools/src/base/platform.dart';
import 'package:flutter_tools/src/build_info.dart';
import 'package:flutter_tools/src/build_system/tools/shader_compiler.dart';
import 'package:flutter_tools/src/compile.dart';
import 'package:flutter_tools/src/devfs.dart';
import 'package:flutter_tools/src/device.dart';
import 'package:flutter_tools/src/project.dart';
import 'package:flutter_tools/src/resident_runner.dart';
import 'package:flutter_tools/src/run_cold.dart';
import 'package:flutter_tools/src/tracing.dart';
import 'package:flutter_tools/src/vmservice.dart';
import 'package:test/fake.dart';
import 'package:vm_service/vm_service.dart';

import '../src/common.dart';
import '../src/context.dart';

void main() {
  testUsingContext('Exits with code 2 when HttpException is thrown '
      'during VM service connection', () async {
    final FakeResidentCompiler residentCompiler = FakeResidentCompiler();
    final FakeDevice device =
        FakeDevice()
          ..supportsHotReload = true
          ..supportsHotRestart = false;

    final List<FlutterDevice> devices = <FlutterDevice>[
      TestFlutterDevice(
        device: device,
        generator: residentCompiler,
        exception: const HttpException(
          'Connection closed before full header was received, '
          'uri = http://127.0.0.1:63394/5ZmLv8A59xY=/ws',
        ),
      ),
    ];

    final int exitCode =
        await ColdRunner(
          devices,
          debuggingOptions: DebuggingOptions.enabled(BuildInfo.debug),
          target: 'main.dart',
        ).attach();
    expect(exitCode, 2);
  });

  group('cleanupAtFinish()', () {
    testUsingContext('disposes each device', () async {
      final FakeDevice device1 = FakeDevice();
      final FakeDevice device2 = FakeDevice();
      final FakeFlutterDevice flutterDevice1 = FakeFlutterDevice(device1);
      final FakeFlutterDevice flutterDevice2 = FakeFlutterDevice(device2);

      final List<FlutterDevice> devices = <FlutterDevice>[flutterDevice1, flutterDevice2];

      await ColdRunner(
        devices,
        debuggingOptions: DebuggingOptions.enabled(BuildInfo.debug),
        target: 'main.dart',
      ).cleanupAtFinish();

      expect(flutterDevice1.stopEchoingDeviceLogCount, 1);
      expect(flutterDevice2.stopEchoingDeviceLogCount, 1);
      expect(device2.wasDisposed, true);
      expect(device1.wasDisposed, true);
    });
  });

  group('cold run', () {
    late MemoryFileSystem memoryFileSystem;
    late FakePlatform fakePlatform;

    setUp(() {
      memoryFileSystem = MemoryFileSystem();
      fakePlatform = FakePlatform(environment: <String, String>{});
    });

    testUsingContext('calls runCold on attached device', () async {
      final FakeDevice device = FakeDevice();
      final FakeFlutterDevice flutterDevice = FakeFlutterDevice(device)..runColdCode = 1;
      final List<FlutterDevice> devices = <FlutterDevice>[flutterDevice];
      final File applicationBinary = MemoryFileSystem.test().file('binary');
      final int result =
          await ColdRunner(
            devices,
            applicationBinary: applicationBinary,
            debuggingOptions: DebuggingOptions.enabled(BuildInfo.debug),
            target: 'main.dart',
          ).run();

      expect(result, 1);
    });

    testUsingContext(
      'with traceStartup, no env variable',
      () async {
        final FakeDevice device = FakeDevice();
        final FakeFlutterDevice flutterDevice = FakeFlutterDevice(device);
        final List<FlutterDevice> devices = <FlutterDevice>[flutterDevice];
        final File applicationBinary = MemoryFileSystem.test().file('binary');
        final int result =
            await ColdRunner(
              devices,
              applicationBinary: applicationBinary,
              debuggingOptions: DebuggingOptions.disabled(BuildInfo.debug),
              target: 'main.dart',
              traceStartup: true,
            ).run();

        expect(result, 0);
        expect(
          memoryFileSystem
              .directory(getBuildDirectory())
              .childFile('start_up_info.json')
              .existsSync(),
          true,
        );
      },
      overrides: <Type, Generator>{
        FileSystem: () => memoryFileSystem,
        ProcessManager: () => FakeProcessManager.any(),
        Platform: () => fakePlatform,
      },
    );

    testUsingContext(
      'with traceStartup, env variable',
      () async {
        fakePlatform.environment[kFlutterTestOutputsDirEnvName] = 'test_output_dir';

        final FakeDevice device = FakeDevice();
        final FakeFlutterDevice flutterDevice = FakeFlutterDevice(device);
        final List<FlutterDevice> devices = <FlutterDevice>[flutterDevice];
        final File applicationBinary = MemoryFileSystem.test().file('binary');
        final int result =
            await ColdRunner(
              devices,
              applicationBinary: applicationBinary,
              debuggingOptions: DebuggingOptions.disabled(BuildInfo.debug),
              target: 'main.dart',
              traceStartup: true,
            ).run();

        expect(result, 0);
        expect(
          memoryFileSystem
              .directory('test_output_dir')
              .childFile('start_up_info.json')
              .existsSync(),
          true,
        );
      },
      overrides: <Type, Generator>{
        FileSystem: () => memoryFileSystem,
        ProcessManager: () => FakeProcessManager.any(),
        Platform: () => fakePlatform,
      },
    );
  });
}

class FakeFlutterDevice extends Fake implements FlutterDevice {
  FakeFlutterDevice(this.device);

  @override
  Stream<Uri> get vmServiceUris => const Stream<Uri>.empty();

  @override
  final Device device;

  int stopEchoingDeviceLogCount = 0;

  @override
  Future<void> stopEchoingDeviceLog() async {
    stopEchoingDeviceLogCount += 1;
  }

  @override
  FlutterVmService get vmService => FakeFlutterVmService();

  int runColdCode = 0;

  @override
  Future<int> runCold({ColdRunner? coldRunner, String? route}) async {
    return runColdCode;
  }
}

class FakeDevice extends Fake implements Device {
  @override
  bool isSupported() => true;

  @override
  bool supportsHotReload = false;

  @override
  bool supportsHotRestart = false;

  @override
  Future<String> get sdkNameAndVersion async => 'Android 10';

  @override
  String get name => 'test';

  @override
  String get displayName => name;

  @override
  Future<TargetPlatform> get targetPlatform async => TargetPlatform.tester;

  bool wasDisposed = false;

  @override
  Future<void> dispose() async {
    wasDisposed = true;
  }
}

class TestFlutterDevice extends FlutterDevice {
  TestFlutterDevice({
    required Device device,
    required this.exception,
    required ResidentCompiler generator,
  }) : super(
         device,
         buildInfo: BuildInfo.debug,
         generator: generator,
         developmentShaderCompiler: const FakeShaderCompiler(),
       );

  /// The exception to throw when the connect method is called.
  final Exception exception;

  @override
  Future<void> connect({
    ReloadSources? reloadSources,
    Restart? restart,
    CompileExpression? compileExpression,
    FlutterProject? flutterProject,
    PrintStructuredErrorLogMethod? printStructuredErrorLogMethod,
    required DebuggingOptions debuggingOptions,
    int? hostVmServicePort,
    required bool allowExistingDdsInstance,
  }) async {
    throw exception;
  }
}

class FakeResidentCompiler extends Fake implements ResidentCompiler {}

class FakeFlutterVmService extends Fake implements FlutterVmService {
  @override
  VmService get service => FakeVmService();

  @override
  Future<List<FlutterView>> getFlutterViews({
    bool returnEarly = false,
    Duration delay = const Duration(milliseconds: 50),
  }) async {
    return <FlutterView>[];
  }

  @override
  Future<bool> flutterAlreadyPaintedFirstUsefulFrame({String? isolateId}) async => true;

  @override
  Future<Response?> getTimeline() async {
    return Response.parse(<String, dynamic>{
      'traceEvents': <dynamic>[
        <String, dynamic>{'name': kFlutterEngineMainEnterEventName, 'ts': 123},
        <String, dynamic>{'name': kFirstFrameBuiltEventName, 'ts': 124},
        <String, dynamic>{'name': kFirstFrameRasterizedEventName, 'ts': 124},
      ],
    });
  }

  @override
  Future<void> setTimelineFlags(List<String> recordedStreams) async {}
}

class FakeVmService extends Fake implements VmService {
  @override
  Future<Success> streamListen(String streamId) async => Success();

  @override
  Stream<Event> get onExtensionEvent {
    return Stream<Event>.fromIterable(<Event>[
      Event(kind: 'Extension', extensionKind: 'Flutter.FirstFrame', timestamp: 1),
    ]);
  }
}

class FakeShaderCompiler implements DevelopmentShaderCompiler {
  const FakeShaderCompiler();

  @override
  void configureCompiler(TargetPlatform? platform) {}

  @override
  Future<DevFSContent> recompileShader(DevFSContent inputShader) {
    throw UnimplementedError();
  }
}
