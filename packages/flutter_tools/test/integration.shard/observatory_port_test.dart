// Copyright 2014 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

import 'dart:async';

import 'package:flutter_tools/src/base/file_system.dart';
import 'package:flutter_tools/src/base/io.dart';
import 'package:flutter_tools/src/convert.dart';

import '../src/common.dart';
import 'test_data/basic_project.dart';
import 'test_utils.dart';

Future<int> getFreePort() async {
  int port = 0;
  final ServerSocket serverSocket = await ServerSocket.bind(InternetAddress.loopbackIPv4, 0);
  port = serverSocket.port;
  await serverSocket.close();
  return port;
}

Future<void> waitForVmServiceMessage(Process process, int port) async {
  final Completer<void> completer = Completer<void>();
  process.stdout.transform(utf8.decoder).listen((String line) {
    printOnFailure(line);
    if (line.contains(
      'The Flutter DevTools debugger and profiler on Flutter test device is available at',
    )) {
      if (line.contains('http://127.0.0.1:$port')) {
        completer.complete();
      } else {
        completer.completeError(
          Exception('Did not forward to provided port $port, instead found $line'),
        );
      }
    }
  });
  process.stderr.transform(utf8.decoder).listen(printOnFailure);
  return completer.future;
}

void main() {
  late Directory tempDir;
  final BasicProject project = BasicProject();

  setUp(() async {
    tempDir = createResolvedTempDirectorySync('run_test.');
    await project.setUpIn(tempDir);
  });

  tearDown(() async {
    tryToDelete(tempDir);
  });

  testWithoutContext('flutter run --vm-service-port', () async {
    final int port = await getFreePort();
    // If only --vm-service-port is provided, --vm-service-port will be used by DDS
    // and the VM service will bind to a random port.
    final Process process = await processManager.start(<String>[
      flutterBin,
      'run',
      '--show-test-device',
      '--vm-service-port=$port',
      '-d',
      'flutter-tester',
    ], workingDirectory: tempDir.path);
    await waitForVmServiceMessage(process, port);
    // Send a quit command to flutter_tools to cleanly shut down the tool
    // and its child processes.
    process.stdin.writeln('q');
    await process.exitCode;
  });

  testWithoutContext('flutter run --dds-port --vm-service-port', () async {
    final int vmServicePort = await getFreePort();
    int ddsPort = await getFreePort();
    while (ddsPort == vmServicePort) {
      ddsPort = await getFreePort();
    }
    // If both --dds-port and --vm-service-port are provided, --dds-port will be used by
    // DDS and --vm-service-port will be used by the VM service.
    final Process process = await processManager.start(<String>[
      flutterBin,
      'run',
      '--show-test-device',
      '--vm-service-port=$vmServicePort',
      '--dds-port=$ddsPort',
      '-d',
      'flutter-tester',
    ], workingDirectory: tempDir.path);
    await waitForVmServiceMessage(process, ddsPort);
    process.stdin.writeln('q');
    await process.exitCode;
  });

  testWithoutContext('flutter run --dds-port', () async {
    final int ddsPort = await getFreePort();
    // If only --dds-port is provided, --dds-port will be used by DDS and the VM service
    // will bind to a random port.
    final Process process = await processManager.start(<String>[
      flutterBin,
      'run',
      '--show-test-device',
      '--dds-port=$ddsPort',
      '-d',
      'flutter-tester',
    ], workingDirectory: tempDir.path);
    await waitForVmServiceMessage(process, ddsPort);
    process.stdin.writeln('q');
    await process.exitCode;
  });
}
