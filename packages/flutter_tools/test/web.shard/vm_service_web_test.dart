// Copyright 2014 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

@Tags(<String>['flutter-test-driver'])
library;

import 'dart:async';

import 'package:file/file.dart';
import 'package:flutter_tools/src/base/file_system.dart';
import 'package:flutter_tools/src/vmservice.dart';
import 'package:vm_service/vm_service.dart';
import 'package:vm_service/vm_service_io.dart';

import '../integration.shard/test_data/basic_project.dart';
import '../integration.shard/test_driver.dart';
import '../integration.shard/test_utils.dart';
import '../src/common.dart';

void main() {
  late Directory tempDir;
  final BasicProjectWithUnaryMain project = BasicProjectWithUnaryMain();
  late FlutterRunTestDriver flutter;

  group('Clients of flutter run on web with DDS enabled', () {
    setUp(() async {
      tempDir = createResolvedTempDirectorySync('run_test.');
      await project.setUpIn(tempDir);
      flutter = FlutterRunTestDriver(tempDir);
    });

    tearDown(() async {
      await flutter.stop();
      tryToDelete(tempDir);
    });

    testWithoutContext('can validate flutter version', () async {
      await flutter.run(
        withDebugger: true,
        chrome: true,
        additionalCommandArgs: <String>['--verbose'],
      );

      expect(flutter.vmServiceWsUri, isNotNull);

      final VmService client = await vmServiceConnectUri('${flutter.vmServiceWsUri}');
      await validateFlutterVersion(client);
    });

    testWithoutContext('can validate flutter version in parallel', () async {
      await flutter.run(
        withDebugger: true,
        chrome: true,
        additionalCommandArgs: <String>['--verbose'],
      );

      expect(flutter.vmServiceWsUri, isNotNull);

      final VmService client1 = await vmServiceConnectUri('${flutter.vmServiceWsUri}');

      final VmService client2 = await vmServiceConnectUri('${flutter.vmServiceWsUri}');

      await Future.wait(<Future<void>>[
        validateFlutterVersion(client1),
        validateFlutterVersion(client2),
      ]);
    }, skip: true); // https://github.com/flutter/flutter/issues/99003
  });

  group('Clients of flutter run on web with DDS disabled', () {
    setUp(() async {
      tempDir = createResolvedTempDirectorySync('run_test.');
      await project.setUpIn(tempDir);
      flutter = FlutterRunTestDriver(tempDir, spawnDdsInstance: false);
    });

    tearDown(() async {
      await flutter.stop();
      tryToDelete(tempDir);
    });

    testWithoutContext('can validate flutter version', () async {
      await flutter.run(
        withDebugger: true,
        chrome: true,
        additionalCommandArgs: <String>['--verbose'],
      );

      expect(flutter.vmServiceWsUri, isNotNull);

      final VmService client = await vmServiceConnectUri('${flutter.vmServiceWsUri}');
      await validateFlutterVersion(client);
    });
  });
}

Future<void> validateFlutterVersion(VmService client) async {
  String? method;

  final Future<dynamic> registration = expectLater(
    client.onEvent('Service'),
    emitsThrough(
      predicate((Event e) {
        if (e.kind == EventKind.kServiceRegistered && e.service == kFlutterVersionServiceName) {
          method = e.method;
          return true;
        }
        return false;
      }),
    ),
  );

  await client.streamListen('Service');
  await registration;
  await client.streamCancel('Service');

  final dynamic version1 = await client.callServiceExtension(method!);
  expect(
    version1,
    const TypeMatcher<Success>()
        .having((Success r) => r.type, 'type', 'Success')
        .having((Success r) => r.json!['frameworkVersion'], 'frameworkVersion', isNotNull),
  );

  await client.dispose();
}
