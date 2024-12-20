// Copyright 2014 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

import 'dart:async';

import 'base/io.dart';

class ForwardedPort {
  ForwardedPort(this.hostPort, this.devicePort) : context = null;
  ForwardedPort.withContext(this.hostPort, this.devicePort, this.context);

  final int hostPort;
  final int devicePort;
  final Process? context;

  @override
  String toString() => 'ForwardedPort HOST:$hostPort to DEVICE:$devicePort';

  /// Kill subprocess (if present) used in forwarding.
  void dispose() {
    if (context != null) {
      context!.kill();
    }
  }
}

/// Forward ports from the host machine to the device.
abstract class DevicePortForwarder {
  /// Returns a Future that completes with the current list of forwarded
  /// ports for this device.
  List<ForwardedPort> get forwardedPorts;

  /// Forward [hostPort] on the host to [devicePort] on the device.
  /// If [hostPort] is null or zero, will auto select a host port.
  /// Returns a Future that completes with the host port.
  Future<int> forward(int devicePort, {int? hostPort});

  /// Stops forwarding [forwardedPort].
  Future<void> unforward(ForwardedPort forwardedPort);

  /// Cleanup allocated resources, like [forwardedPorts].
  Future<void> dispose();
}

// A port forwarder which does not support forwarding ports.
class NoOpDevicePortForwarder implements DevicePortForwarder {
  const NoOpDevicePortForwarder();

  @override
  Future<int> forward(int devicePort, {int? hostPort}) async => devicePort;

  @override
  List<ForwardedPort> get forwardedPorts => <ForwardedPort>[];

  @override
  Future<void> unforward(ForwardedPort forwardedPort) async {}

  @override
  Future<void> dispose() async {}
}
