// Copyright 2014 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

import 'dart:io' as io;
import 'package:flutter_tools/src/base/io.dart';
import 'package:flutter_tools/src/device_port_forwarder.dart';
import 'package:test/fake.dart';

import '../src/common.dart';

void main() {
  testWithoutContext('dispose does not throw exception if no process is present', () {
    final ForwardedPort forwardedPort = ForwardedPort(123, 456);
    expect(forwardedPort.context, isNull);
    forwardedPort.dispose();
  });

  testWithoutContext('dispose kills process if process was available', () {
    final FakeProcess process = FakeProcess();
    final ForwardedPort forwardedPort = ForwardedPort.withContext(123, 456, process);
    forwardedPort.dispose();

    expect(forwardedPort.context, isNotNull);
    expect(process.killed, true);
  });
}

class FakeProcess extends Fake implements Process {
  bool killed = false;

  @override
  bool kill([io.ProcessSignal signal = io.ProcessSignal.sigterm]) {
    return killed = true;
  }
}
