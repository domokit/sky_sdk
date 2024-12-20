// Copyright 2014 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

import 'dart:async';

import 'package:flutter_tools/src/base/io.dart';
import 'package:flutter_tools/src/base/logger.dart';
import 'package:flutter_tools/src/commands/daemon.dart';
import 'package:test/fake.dart';

import '../../src/common.dart';

void main() {
  testWithoutContext('binds on ipv4 normally', () async {
    final FakeServerSocket socket = FakeServerSocket();
    final BufferLogger logger = BufferLogger.test();

    int bindCalledTimes = 0;
    final List<Object?> bindAddresses = <Object?>[];
    final List<int> bindPorts = <int>[];

    final DaemonServer server = DaemonServer(
      port: 123,
      logger: logger,
      bind: (Object? address, int port) async {
        bindCalledTimes++;
        bindAddresses.add(address);
        bindPorts.add(port);
        return socket;
      },
    );
    await server.run();
    expect(bindCalledTimes, 1);
    expect(bindAddresses, <Object?>[InternetAddress.loopbackIPv4]);
    expect(bindPorts, <int>[123]);
  });

  testWithoutContext('binds on ipv6 if ipv4 failed normally', () async {
    final FakeServerSocket socket = FakeServerSocket();
    final BufferLogger logger = BufferLogger.test();

    int bindCalledTimes = 0;
    final List<Object?> bindAddresses = <Object?>[];
    final List<int> bindPorts = <int>[];

    final DaemonServer server = DaemonServer(
      port: 123,
      logger: logger,
      bind: (Object? address, int port) async {
        bindCalledTimes++;
        bindAddresses.add(address);
        bindPorts.add(port);
        if (address == InternetAddress.loopbackIPv4) {
          throw const SocketException('fail');
        }
        return socket;
      },
    );
    await server.run();
    expect(bindCalledTimes, 2);
    expect(bindAddresses, <Object?>[InternetAddress.loopbackIPv4, InternetAddress.loopbackIPv6]);
    expect(bindPorts, <int>[123, 123]);
  });
}

class FakeServerSocket extends Fake implements ServerSocket {
  FakeServerSocket();

  @override
  int get port => 1;

  bool closeCalled = false;
  final StreamController<Socket> controller = StreamController<Socket>();

  @override
  StreamSubscription<Socket> listen(
    void Function(Socket event)? onData, {
    Function? onError,
    void Function()? onDone,
    bool? cancelOnError,
  }) {
    // Close the controller immediately for testing purpose.
    scheduleMicrotask(() {
      controller.close();
    });
    return controller.stream.listen(
      onData,
      onError: onError,
      onDone: onDone,
      cancelOnError: cancelOnError,
    );
  }

  @override
  Future<ServerSocket> close() async {
    closeCalled = true;
    return this;
  }
}
