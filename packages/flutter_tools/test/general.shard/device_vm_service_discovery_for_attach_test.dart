// Copyright 2014 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

import 'package:flutter_tools/src/base/logger.dart';
import 'package:flutter_tools/src/device_port_forwarder.dart';
import 'package:flutter_tools/src/device_vm_service_discovery_for_attach.dart';
import 'package:test/fake.dart';

import '../src/common.dart';
import '../src/fake_devices.dart';

void main() {
  group('LogScanningVMServiceDiscoveryForAttach', () {
    testWithoutContext('can discover the port', () async {
      final FakeDeviceLogReader logReader = FakeDeviceLogReader();
      final LogScanningVMServiceDiscoveryForAttach discovery =
          LogScanningVMServiceDiscoveryForAttach(
            Future<FakeDeviceLogReader>.value(logReader),
            ipv6: false,
            logger: BufferLogger.test(),
          );

      logReader.addLine('The Dart VM service is listening on http://127.0.0.1:9999');

      expect(await discovery.uris.first, Uri.parse('http://127.0.0.1:9999'));
    });

    testWithoutContext('ignores the port that does not match devicePort', () async {
      final FakeDeviceLogReader logReader = FakeDeviceLogReader();
      final LogScanningVMServiceDiscoveryForAttach discovery =
          LogScanningVMServiceDiscoveryForAttach(
            Future<FakeDeviceLogReader>.value(logReader),
            devicePort: 9998,
            ipv6: false,
            logger: BufferLogger.test(),
          );

      logReader.addLine('The Dart VM service is listening on http://127.0.0.1:9999');
      logReader.addLine('The Dart VM service is listening on http://127.0.0.1:9998');

      expect(await discovery.uris.first, Uri.parse('http://127.0.0.1:9998'));
    });

    testWithoutContext('forwards the port if given a port forwarder', () async {
      final FakeDeviceLogReader logReader = FakeDeviceLogReader();
      final FakePortForwarder portForwarder = FakePortForwarder(9900);
      final LogScanningVMServiceDiscoveryForAttach discovery =
          LogScanningVMServiceDiscoveryForAttach(
            Future<FakeDeviceLogReader>.value(logReader),
            portForwarder: portForwarder,
            ipv6: false,
            logger: BufferLogger.test(),
          );

      logReader.addLine('The Dart VM service is listening on http://127.0.0.1:9999');

      expect(await discovery.uris.first, Uri.parse('http://127.0.0.1:9900'));
      expect(portForwarder.forwardDevicePort, 9999);
      expect(portForwarder.forwardHostPort, null);
    });

    testWithoutContext('uses the host port if given', () async {
      final FakeDeviceLogReader logReader = FakeDeviceLogReader();
      final FakePortForwarder portForwarder = FakePortForwarder(9900);
      final LogScanningVMServiceDiscoveryForAttach discovery =
          LogScanningVMServiceDiscoveryForAttach(
            Future<FakeDeviceLogReader>.value(logReader),
            portForwarder: portForwarder,
            hostPort: 9901,
            ipv6: false,
            logger: BufferLogger.test(),
          );

      logReader.addLine('The Dart VM service is listening on http://127.0.0.1:9999');

      expect(await discovery.uris.first, Uri.parse('http://127.0.0.1:9900'));
      expect(portForwarder.forwardDevicePort, 9999);
      expect(portForwarder.forwardHostPort, 9901);
    });
  });

  group('DelegateVMServiceDiscoveryForAttach', () {
    late List<Uri> uris1;
    late List<Uri> uris2;
    late FakeVmServiceDiscoveryForAttach fakeDiscovery1;
    late FakeVmServiceDiscoveryForAttach fakeDiscovery2;
    late DelegateVMServiceDiscoveryForAttach delegateDiscovery;

    setUp(() {
      uris1 = <Uri>[];
      uris2 = <Uri>[];
      fakeDiscovery1 = FakeVmServiceDiscoveryForAttach(uris1);
      fakeDiscovery2 = FakeVmServiceDiscoveryForAttach(uris2);
      delegateDiscovery = DelegateVMServiceDiscoveryForAttach(<VMServiceDiscoveryForAttach>[
        fakeDiscovery1,
        fakeDiscovery2,
      ]);
    });

    testWithoutContext('uris returns from both delegates', () async {
      uris1.add(Uri.parse('http://127.0.0.1:1'));
      uris1.add(Uri.parse('http://127.0.0.2:2'));
      uris2.add(Uri.parse('http://127.0.0.3:3'));
      uris2.add(Uri.parse('http://127.0.0.4:4'));

      expect(
        await delegateDiscovery.uris.toList(),
        unorderedEquals(<Uri>[
          Uri.parse('http://127.0.0.1:1'),
          Uri.parse('http://127.0.0.2:2'),
          Uri.parse('http://127.0.0.3:3'),
          Uri.parse('http://127.0.0.4:4'),
        ]),
      );
    });
  });
}

class FakePortForwarder extends Fake implements DevicePortForwarder {
  FakePortForwarder(this.forwardReturnValue);

  int? forwardDevicePort;
  int? forwardHostPort;
  final int forwardReturnValue;

  @override
  Future<int> forward(int devicePort, {int? hostPort}) async {
    forwardDevicePort = devicePort;
    forwardHostPort = hostPort;
    return forwardReturnValue;
  }
}

class FakeVmServiceDiscoveryForAttach extends Fake implements VMServiceDiscoveryForAttach {
  FakeVmServiceDiscoveryForAttach(this._uris);

  final List<Uri> _uris;

  @override
  Stream<Uri> get uris => Stream<Uri>.fromIterable(_uris);
}
