// Copyright 2014 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

@Tags(<String>['flutter-test-driver'])
library;

import 'package:file/file.dart';
import 'package:flutter_tools/src/base/file_system.dart';
import 'package:vm_service/vm_service.dart';
import 'package:vm_service/vm_service_io.dart';

import '../src/common.dart';
import 'test_data/basic_project.dart';
import 'test_driver.dart';
import 'test_utils.dart';

void main() {
  group('Flutter Tool VMService method', () {
    late Directory tempDir;
    late FlutterRunTestDriver flutter;
    late VmService vmService;

    setUp(() async {
      tempDir = createResolvedTempDirectorySync('vmservice_integration_test.');

      final BasicProject project = BasicProject();
      await project.setUpIn(tempDir);

      flutter = FlutterRunTestDriver(tempDir);
      await flutter.run(withDebugger: true, verbose: true);
      final int? port = flutter.vmServicePort;
      expect(port != null, true);
      vmService = await vmServiceConnectUri('ws://localhost:$port/ws');
    });

    tearDown(() async {
      await flutter.stop();
      tryToDelete(tempDir);
    });

    testWithoutContext('getSupportedProtocols includes DDS', () async {
      final ProtocolList protocolList = await vmService.getSupportedProtocols();
      expect(protocolList.protocols, hasLength(2));
      for (final Protocol protocol in protocolList.protocols!) {
        expect(protocol.protocolName, anyOf('VM Service', 'DDS'));
      }
    });

    testWithoutContext('flutterVersion can be called', () async {
      final Response response = await vmService.callServiceExtension('s0.flutterVersion');
      expect(response.type, 'Success');
      expect(response.json, containsPair('frameworkRevisionShort', isNotNull));
      expect(response.json, containsPair('engineRevisionShort', isNotNull));
    });

    testWithoutContext('flutterMemoryInfo can be called', () async {
      final Response response = await vmService.callServiceExtension('s0.flutterMemoryInfo');
      expect(response.type, 'Success');
    });

    testWithoutContext('reloadSources can be called', () async {
      final VM vm = await vmService.getVM();
      final IsolateRef? isolateRef = vm.isolates?.first;
      expect(isolateRef != null, true);
      final Response response = await vmService.callMethod(
        's0.reloadSources',
        isolateId: isolateRef!.id,
      );
      expect(response.type, 'Success');
    });

    testWithoutContext('reloadSources fails on bad params', () async {
      final Future<Response> response = vmService.callMethod('s0.reloadSources', isolateId: '');
      expect(response, throwsA(const TypeMatcher<RPCError>()));
    });

    testWithoutContext('hotRestart can be called', () async {
      final VM vm = await vmService.getVM();
      final IsolateRef? isolateRef = vm.isolates?.first;
      expect(isolateRef != null, true);
      final Response response = await vmService.callMethod(
        's0.hotRestart',
        isolateId: isolateRef!.id,
      );
      expect(response.type, 'Success');
    });

    testWithoutContext('hotRestart fails on bad params', () async {
      final Future<Response> response = vmService.callMethod(
        's0.hotRestart',
        args: <String, dynamic>{'pause': 'not_a_bool'},
      );
      expect(response, throwsA(const TypeMatcher<RPCError>()));
    });

    testWithoutContext('ext.flutter.brightnessOverride can toggle window brightness', () async {
      final Isolate isolate = await waitForExtension(vmService, 'ext.flutter.brightnessOverride');
      final Response response = await vmService.callServiceExtension(
        'ext.flutter.brightnessOverride',
        isolateId: isolate.id,
      );
      expect(response.json?['value'], 'Brightness.light');

      final Response updateResponse = await vmService.callServiceExtension(
        'ext.flutter.brightnessOverride',
        isolateId: isolate.id,
        args: <String, String>{'value': 'Brightness.dark'},
      );
      expect(updateResponse.json?['value'], 'Brightness.dark');

      // Change the brightness back to light
      final Response verifyResponse = await vmService.callServiceExtension(
        'ext.flutter.brightnessOverride',
        isolateId: isolate.id,
        args: <String, String>{'value': 'Brightness.light'},
      );
      expect(verifyResponse.json?['value'], 'Brightness.light');

      // Change with a bogus value
      final Response bogusResponse = await vmService.callServiceExtension(
        'ext.flutter.brightnessOverride',
        isolateId: isolate.id,
        args: <String, String>{
          'value': 'dark', // Intentionally invalid value.
        },
      );
      expect(bogusResponse.json?['value'], 'Brightness.light');
    });

    testWithoutContext('ext.flutter.debugPaint can toggle debug painting', () async {
      final Isolate isolate = await waitForExtension(vmService, 'ext.flutter.debugPaint');
      final Response response = await vmService.callServiceExtension(
        'ext.flutter.debugPaint',
        isolateId: isolate.id,
      );
      expect(response.json?['enabled'], 'false');

      final Response updateResponse = await vmService.callServiceExtension(
        'ext.flutter.debugPaint',
        isolateId: isolate.id,
        args: <String, String>{'enabled': 'true'},
      );
      expect(updateResponse.json?['enabled'], 'true');
    });
  });
}
