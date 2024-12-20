// Copyright 2014 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

import 'package:file/file.dart';
import 'package:flutter_tools/src/application_package.dart';
import 'package:flutter_tools/src/base/dds.dart';
import 'package:flutter_tools/src/base/io.dart' as io;
import 'package:flutter_tools/src/base/logger.dart';
import 'package:flutter_tools/src/build_info.dart';
import 'package:flutter_tools/src/device.dart';
import 'package:flutter_tools/src/project.dart';
import 'package:flutter_tools/src/test/integration_test_device.dart';
import 'package:flutter_tools/src/test/test_device.dart';
import 'package:flutter_tools/src/vmservice.dart';
import 'package:stream_channel/stream_channel.dart';
import 'package:test/fake.dart';
import 'package:vm_service/vm_service.dart' as vm_service;

import '../src/context.dart';
import '../src/fake_devices.dart';
import '../src/fake_vm_services.dart';
import '../src/fakes.dart';

final vm_service.Isolate isolate = vm_service.Isolate(
  id: '1',
  pauseEvent: vm_service.Event(kind: vm_service.EventKind.kResume, timestamp: 0),
  breakpoints: <vm_service.Breakpoint>[],
  libraries: <vm_service.LibraryRef>[
    vm_service.LibraryRef(id: '1', uri: 'file:///hello_world/main.dart', name: ''),
  ],
  livePorts: 0,
  name: 'test',
  number: '1',
  pauseOnExit: false,
  runnable: true,
  startTime: 0,
  isSystemIsolate: false,
  isolateFlags: <vm_service.IsolateFlag>[],
  extensionRPCs: <String>[kIntegrationTestMethod],
);

final FlutterView fakeFlutterView = FlutterView(id: 'a', uiIsolate: isolate);

final FakeVmServiceRequest listViewsRequest = FakeVmServiceRequest(
  method: kListViewsMethod,
  jsonResponse: <String, Object>{
    'views': <Object>[fakeFlutterView.toJson()],
  },
);

final Uri vmServiceUri = Uri.parse('http://localhost:1234');

void main() {
  late FakeVmServiceHost fakeVmServiceHost;
  late TestDevice testDevice;
  late DDSLauncherCallback originalDdsLauncher;

  setUp(() {
    testDevice = IntegrationTestTestDevice(
      id: 1,
      device: FakeDevice(
        'ephemeral',
        'ephemeral',
        type: PlatformType.android,
        launchResult: LaunchResult.succeeded(vmServiceUri: vmServiceUri),
      ),
      debuggingOptions: DebuggingOptions.enabled(BuildInfo.debug),
      userIdentifier: '',
      compileExpression: null,
    );

    fakeVmServiceHost = FakeVmServiceHost(
      requests: <VmServiceExpectation>[
        const FakeVmServiceRequest(
          method: 'streamListen',
          args: <String, Object>{'streamId': 'Isolate'},
        ),
        listViewsRequest,
        FakeVmServiceRequest(
          method: 'getIsolate',
          jsonResponse: isolate.toJson(),
          args: <String, Object>{'isolateId': '1'},
        ),
        const FakeVmServiceRequest(
          method: 'streamListen',
          args: <String, Object>{'streamId': 'Extension'},
        ),
      ],
    );

    originalDdsLauncher = ddsLauncherCallback;
    ddsLauncherCallback = ({
      required Uri remoteVmServiceUri,
      Uri? serviceUri,
      bool enableAuthCodes = true,
      bool serveDevTools = false,
      Uri? devToolsServerAddress,
      bool enableServicePortFallback = false,
      List<String> cachedUserTags = const <String>[],
      String? dartExecutable,
      String? google3WorkspaceRoot,
    }) async {
      return FakeDartDevelopmentServiceLauncher(uri: Uri.parse('http://localhost:1234'));
    };
  });

  tearDown(() {
    ddsLauncherCallback = originalDdsLauncher;
  });

  testUsingContext('will not start when package missing', () async {
    await expectLater(
      testDevice.start('entrypointPath'),
      throwsA(
        isA<TestDeviceException>().having(
          (Exception e) => e.toString(),
          'description',
          contains('No application found for TargetPlatform.android_arm'),
        ),
      ),
    );
  });

  testUsingContext(
    'Can start the entrypoint',
    () async {
      await testDevice.start('entrypointPath');

      expect(await testDevice.vmServiceUri, vmServiceUri);
      expect(testDevice.finished, doesNotComplete);
    },
    overrides: <Type, Generator>{
      ApplicationPackageFactory: () => FakeApplicationPackageFactory(),
      VMServiceConnector:
          () =>
              (
                Uri httpUri, {
                ReloadSources? reloadSources,
                Restart? restart,
                CompileExpression? compileExpression,
                GetSkSLMethod? getSkSLMethod,
                FlutterProject? flutterProject,
                PrintStructuredErrorLogMethod? printStructuredErrorLogMethod,
                io.CompressionOptions? compression,
                Device? device,
                Logger? logger,
              }) async => fakeVmServiceHost.vmService,
    },
  );

  testUsingContext(
    'Can kill the started device',
    () async {
      await testDevice.start('entrypointPath');
      await testDevice.kill();

      expect(testDevice.finished, completes);
    },
    overrides: <Type, Generator>{
      ApplicationPackageFactory: () => FakeApplicationPackageFactory(),
      VMServiceConnector:
          () =>
              (
                Uri httpUri, {
                ReloadSources? reloadSources,
                Restart? restart,
                CompileExpression? compileExpression,
                GetSkSLMethod? getSkSLMethod,
                FlutterProject? flutterProject,
                PrintStructuredErrorLogMethod? printStructuredErrorLogMethod,
                io.CompressionOptions? compression,
                Device? device,
                Logger? logger,
              }) async => fakeVmServiceHost.vmService,
    },
  );

  testUsingContext(
    'when the device starts without providing an vmService URI',
    () async {
      final TestDevice testDevice = IntegrationTestTestDevice(
        id: 1,
        device: FakeDevice(
          'ephemeral',
          'ephemeral',
          type: PlatformType.android,
          launchResult: LaunchResult.succeeded(),
        ),
        debuggingOptions: DebuggingOptions.enabled(BuildInfo.debug),
        userIdentifier: '',
        compileExpression: null,
      );

      expect(() => testDevice.start('entrypointPath'), throwsA(isA<TestDeviceException>()));
    },
    overrides: <Type, Generator>{
      VMServiceConnector:
          () =>
              (
                Uri httpUri, {
                ReloadSources? reloadSources,
                Restart? restart,
                CompileExpression? compileExpression,
                GetSkSLMethod? getSkSLMethod,
                FlutterProject? flutterProject,
                PrintStructuredErrorLogMethod? printStructuredErrorLogMethod,
                io.CompressionOptions? compression,
                Device? device,
              }) async => fakeVmServiceHost.vmService,
    },
  );

  testUsingContext(
    'when the device fails to start',
    () async {
      final TestDevice testDevice = IntegrationTestTestDevice(
        id: 1,
        device: FakeDevice(
          'ephemeral',
          'ephemeral',
          type: PlatformType.android,
          launchResult: LaunchResult.failed(),
        ),
        debuggingOptions: DebuggingOptions.enabled(BuildInfo.debug),
        userIdentifier: '',
        compileExpression: null,
      );

      expect(() => testDevice.start('entrypointPath'), throwsA(isA<TestDeviceException>()));
    },
    overrides: <Type, Generator>{
      VMServiceConnector:
          () =>
              (
                Uri httpUri, {
                ReloadSources? reloadSources,
                Restart? restart,
                CompileExpression? compileExpression,
                GetSkSLMethod? getSkSLMethod,
                FlutterProject? flutterProject,
                PrintStructuredErrorLogMethod? printStructuredErrorLogMethod,
                io.CompressionOptions? compression,
                Device? device,
              }) async => fakeVmServiceHost.vmService,
    },
  );

  testUsingContext(
    'Can handle closing of the VM service',
    () async {
      final StreamChannel<String> channel = await testDevice.start('entrypointPath');
      await fakeVmServiceHost.vmService.dispose();
      expect(await channel.stream.isEmpty, true);
    },
    overrides: <Type, Generator>{
      ApplicationPackageFactory: () => FakeApplicationPackageFactory(),
      VMServiceConnector:
          () =>
              (
                Uri httpUri, {
                ReloadSources? reloadSources,
                Restart? restart,
                CompileExpression? compileExpression,
                GetSkSLMethod? getSkSLMethod,
                FlutterProject? flutterProject,
                PrintStructuredErrorLogMethod? printStructuredErrorLogMethod,
                io.CompressionOptions? compression,
                Device? device,
                Logger? logger,
              }) async => fakeVmServiceHost.vmService,
    },
  );
}

class FakeApplicationPackageFactory extends Fake implements ApplicationPackageFactory {
  @override
  Future<ApplicationPackage> getPackageForPlatform(
    TargetPlatform platform, {
    BuildInfo? buildInfo,
    File? applicationBinary,
  }) async => FakeApplicationPackage();
}

class FakeApplicationPackage extends Fake implements ApplicationPackage {}
