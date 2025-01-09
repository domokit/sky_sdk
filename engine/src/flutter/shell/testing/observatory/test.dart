// Copyright 2013 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

// This is a minimal dependency heart beat test for the Dart VM Service.

// ignore_for_file: avoid_print

import 'dart:async';
import 'dart:convert';
import 'dart:io';

import 'launcher.dart';
import 'service_client.dart';

class Expect {
  static void equals(Object? actual, Object? expected) {
    if (actual != expected) {
      throw AssertionError('Expected $actual == $expected');
    }
  }
}

Future<String> readResponse(HttpClientResponse response) {
  final Completer<String> completer = Completer<String>();
  final StringBuffer contents = StringBuffer();
  response.transform(utf8.decoder).listen((String data) {
    contents.write(data);
  }, onDone: () => completer.complete(contents.toString()));
  return completer.future;
}

// Test accessing the service protocol over http.
Future<void> testHttpProtocolRequest(Uri uri) async {
  uri = uri.replace(path: 'getVM');
  final HttpClient client = HttpClient();
  final HttpClientRequest request = await client.getUrl(uri);
  final HttpClientResponse response = await request.close();
  Expect.equals(response.statusCode, 200);
  final responseAsMap = json.decode(await readResponse(response)) as Map<String, Object?>;
  Expect.equals(responseAsMap['jsonrpc'], '2.0');
  client.close();
}

// Test accessing the service protocol over ws.
Future<void> testWebSocketProtocolRequest(Uri uri) async {
  uri = uri.replace(scheme: 'ws', path: 'ws');
  final WebSocket webSocketClient = await WebSocket.connect(uri.toString());
  final ServiceClient serviceClient = ServiceClient(webSocketClient);
  final Map<String, dynamic> response = await serviceClient.invokeRPC('getVM');
  Expect.equals(response['type'], 'VM');
  try {
    await serviceClient.invokeRPC('BART_SIMPSON');
    throw AssertionError('Unreachable');
  } catch (e) {
    // Method not found.
    Expect.equals((e as Map<String, dynamic>)['code'], -32601);
  }
}

// Test accessing an Observatory UI asset.
Future<void> testHttpAssetRequest(Uri uri) async {
  uri = uri.replace(path: 'third_party/trace_viewer_full.html');
  final HttpClient client = HttpClient();
  final HttpClientRequest request = await client.getUrl(uri);
  final HttpClientResponse response = await request.close();
  Expect.equals(response.statusCode, 200);
  await response.drain<void>();
  client.close();
}

typedef TestFunction = Future<void> Function(Uri uri);

final List<TestFunction> basicTests = <TestFunction>[
  testHttpProtocolRequest,
  testWebSocketProtocolRequest,
  testHttpAssetRequest,
];

Future<bool> runTests(ShellLauncher launcher, List<TestFunction> tests) async {
  final ShellProcess? process = await launcher.launch();
  if (process == null) {
    return false;
  }
  final Uri uri = await process.waitForVMService();
  try {
    for (int i = 0; i < tests.length; i++) {
      print('Executing test ${i + 1}/${tests.length}');
      await tests[i](uri);
    }
  } catch (e, st) {
    print('Dart VM Service test failure: $e\n$st');
    exitCode = -1;
  }
  await process.kill();
  return exitCode == 0;
}

Future<void> main(List<String> args) async {
  if (args.length < 2) {
    print(
      'Usage: dart ${Platform.script} '
      '<sky_shell_executable> <main_dart> ...',
    );
    return;
  }
  final String shellExecutablePath = args[0];
  final String mainDartPath = args[1];
  final List<String> extraArgs = args.length <= 2 ? <String>[] : args.sublist(2);

  final ShellLauncher launcher = ShellLauncher(shellExecutablePath, mainDartPath, false, extraArgs);

  await runTests(launcher, basicTests);
}
