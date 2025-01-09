// Copyright 2014 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

@Tags(<String>['flutter-test-driver'])
library;

import 'dart:convert';

import 'package:file/file.dart';
import 'package:flutter_tools/src/base/io.dart';
import 'package:vm_service/vm_service.dart';

import '../src/common.dart';
import 'test_data/basic_project.dart';
import 'test_driver.dart';
import 'test_utils.dart';

Future<int> getFreePort() async {
  int port = 0;
  final ServerSocket serverSocket = await ServerSocket.bind(InternetAddress.loopbackIPv4, 0);
  port = serverSocket.port;
  await serverSocket.close();
  return port;
}

void main() {
  final BasicProject project = BasicProject();
  late Directory tempDir;

  setUp(() async {
    tempDir = createResolvedTempDirectorySync('attach_test.');
    await project.setUpIn(tempDir);
  });

  tearDown(() {
    tryToDelete(tempDir);
  });

  group('DDS in flutter run', () {
    late FlutterRunTestDriver flutterRun, flutterAttach;
    setUp(() {
      flutterRun = FlutterRunTestDriver(tempDir, logPrefix: '   RUN  ');
      flutterAttach = FlutterRunTestDriver(
        tempDir,
        logPrefix: 'ATTACH  ',
        // Only one DDS instance can be connected to the VM service at a time.
        // DDS can also only initialize if the VM service doesn't have any existing
        // clients, so we'll just let _flutterRun be responsible for spawning DDS.
        spawnDdsInstance: false,
      );
    });

    tearDown(() async {
      await flutterAttach.detach();
      await flutterRun.stop();
    });

    testWithoutContext('can hot reload', () async {
      await flutterRun.run(withDebugger: true);
      await flutterAttach.attach(flutterRun.vmServicePort!);
      await flutterAttach.hotReload();
    });

    testWithoutContext('can detach, reattach, hot reload', () async {
      await flutterRun.run(withDebugger: true);
      await flutterAttach.attach(flutterRun.vmServicePort!);
      await flutterAttach.detach();
      await flutterAttach.attach(flutterRun.vmServicePort!);
      await flutterAttach.hotReload();
    });

    testWithoutContext('killing process behaves the same as detach ', () async {
      await flutterRun.run(withDebugger: true);
      await flutterAttach.attach(flutterRun.vmServicePort!);
      await flutterAttach.quit();
      flutterAttach = FlutterRunTestDriver(tempDir, logPrefix: 'ATTACH-2', spawnDdsInstance: false);
      await flutterAttach.attach(flutterRun.vmServicePort!);
      await flutterAttach.hotReload();
    });

    testWithoutContext('sets activeDevToolsServerAddress extension', () async {
      await flutterRun.run(
        startPaused: true,
        withDebugger: true,
        additionalCommandArgs: <String>['--devtools-server-address', 'http://127.0.0.1:9105'],
      );
      await flutterRun.resume();
      await pollForServiceExtensionValue<String>(
        testDriver: flutterRun,
        extension: 'ext.flutter.activeDevToolsServerAddress',
        continuePollingValue: '',
        matches: equals('http://127.0.0.1:9105'),
      );
      await pollForServiceExtensionValue<String>(
        testDriver: flutterRun,
        extension: 'ext.flutter.connectedVmServiceUri',
        continuePollingValue: '',
        matches: isNotEmpty,
      );

      final Response response = await flutterRun.callServiceExtension(
        'ext.flutter.connectedVmServiceUri',
      );
      final String vmServiceUri = response.json!['value'] as String;

      // Attach with a different DevTools server address.
      await flutterAttach.attach(
        flutterRun.vmServicePort!,
        additionalCommandArgs: <String>['--devtools-server-address', 'http://127.0.0.1:9110'],
      );
      await pollForServiceExtensionValue<String>(
        testDriver: flutterAttach,
        extension: 'ext.flutter.activeDevToolsServerAddress',
        continuePollingValue: '',
        matches: equals('http://127.0.0.1:9110'),
      );
      await pollForServiceExtensionValue<String>(
        testDriver: flutterRun,
        extension: 'ext.flutter.connectedVmServiceUri',
        continuePollingValue: '',
        matches: equals(vmServiceUri),
      );
    });
  });

  group('DDS in flutter attach', () {
    late FlutterRunTestDriver flutterRun, flutterAttach;
    setUp(() {
      flutterRun = FlutterRunTestDriver(tempDir, logPrefix: '   RUN  ', spawnDdsInstance: false);
      flutterAttach = FlutterRunTestDriver(tempDir, logPrefix: 'ATTACH  ');
    });

    tearDown(() async {
      await flutterAttach.detach();
      await flutterRun.stop();
    });

    testWithoutContext('uses the designated dds port', () async {
      final int ddsPort = await getFreePort();

      await flutterRun.run(withDebugger: true);
      await flutterAttach.attach(
        flutterRun.vmServicePort!,
        additionalCommandArgs: <String>['--dds-port=$ddsPort'],
      );

      final Response response = await flutterAttach.callServiceExtension(
        'ext.flutter.connectedVmServiceUri',
      );
      final String vmServiceUriString = response.json!['value'] as String;
      final Uri vmServiceUri = Uri.parse(vmServiceUriString);
      expect(vmServiceUri.port, equals(ddsPort));
    });
  });

  group('--serve-observatory', () {
    late FlutterRunTestDriver flutterRun, flutterAttach;

    setUp(() async {
      flutterRun = FlutterRunTestDriver(tempDir, logPrefix: '   RUN  ');
      flutterAttach = FlutterRunTestDriver(
        tempDir,
        logPrefix: 'ATTACH  ',
        // Only one DDS instance can be connected to the VM service at a time.
        // DDS can also only initialize if the VM service doesn't have any existing
        // clients, so we'll just let _flutterRun be responsible for spawning DDS.
        spawnDdsInstance: false,
      );
    });

    tearDown(() async {
      await flutterAttach.detach();
      await flutterRun.stop();
    });

    Future<bool> isObservatoryAvailable() async {
      final HttpClient client = HttpClient();
      final Uri vmServiceUri = Uri(
        scheme: 'http',
        host: flutterRun.vmServiceWsUri!.host,
        port: flutterRun.vmServicePort,
      );

      final HttpClientRequest request = await client.getUrl(vmServiceUri);
      final HttpClientResponse response = await request.close();
      final String content = await response.transform(utf8.decoder).join();
      return content.contains('Dart VM Observatory');
    }

    testWithoutContext('enables Observatory on run', () async {
      await flutterRun.run(withDebugger: true, serveObservatory: true);
      expect(await isObservatoryAvailable(), true);
    });

    testWithoutContext('enables Observatory on attach', () async {
      await flutterRun.run(withDebugger: true);
      expect(await isObservatoryAvailable(), false);
      await flutterAttach.attach(flutterRun.vmServicePort!, serveObservatory: true);
      expect(await isObservatoryAvailable(), true);
    });
  });
}
