// Copyright 2014 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

import 'dart:async';

import 'package:async/async.dart';

import 'base/logger.dart';
import 'device.dart';
import 'device_port_forwarder.dart';
import 'mdns_discovery.dart';
import 'protocol_discovery.dart';

/// Discovers the VM service uri on a device, and forwards the port to the host.
///
/// This is mainly used during a `flutter attach`.
abstract class VMServiceDiscoveryForAttach {
  VMServiceDiscoveryForAttach();

  /// The discovered VM service URis.
  ///
  /// Port forwarding is only attempted when this is invoked, for each VM
  /// Service URI in the stream.
  Stream<Uri> get uris;
}

/// An implementation of [VMServiceDiscoveryForAttach] that uses log scanning
/// for the discovery.
class LogScanningVMServiceDiscoveryForAttach extends VMServiceDiscoveryForAttach {
  LogScanningVMServiceDiscoveryForAttach(
    Future<DeviceLogReader> logReader, {
    DevicePortForwarder? portForwarder,
    int? hostPort,
    int? devicePort,
    required bool ipv6,
    required Logger logger,
  }) {
    _protocolDiscovery =
        (() async => ProtocolDiscovery.vmService(
          await logReader,
          portForwarder: portForwarder,
          ipv6: ipv6,
          devicePort: devicePort,
          hostPort: hostPort,
          logger: logger,
        ))();
  }

  late final Future<ProtocolDiscovery> _protocolDiscovery;

  @override
  Stream<Uri> get uris {
    final StreamController<Uri> controller = StreamController<Uri>();
    _protocolDiscovery.then((ProtocolDiscovery protocolDiscovery) async {
      await controller.addStream(protocolDiscovery.uris);
      await controller.close();
    }, onError: (Object error) => controller.addError(error));
    return controller.stream;
  }
}

/// An implementation of [VMServiceDiscoveryForAttach] that uses mdns for the
/// discovery.
class MdnsVMServiceDiscoveryForAttach extends VMServiceDiscoveryForAttach {
  MdnsVMServiceDiscoveryForAttach({
    required this.device,
    this.appId,
    required this.usesIpv6,
    required this.useDeviceIPAsHost,
    this.deviceVmservicePort,
    this.hostVmservicePort,
  });

  final Device device;
  final String? appId;
  final bool usesIpv6;
  final bool useDeviceIPAsHost;
  final int? deviceVmservicePort;
  final int? hostVmservicePort;

  @override
  Stream<Uri> get uris {
    final Future<Uri?> mDNSDiscoveryFuture = MDnsVmServiceDiscovery.instance!
        .getVMServiceUriForAttach(
          appId,
          device,
          usesIpv6: usesIpv6,
          useDeviceIPAsHost: useDeviceIPAsHost,
          deviceVmservicePort: deviceVmservicePort,
          hostVmservicePort: hostVmservicePort,
        );

    return Stream<Uri?>.fromFuture(
      mDNSDiscoveryFuture,
    ).where((Uri? uri) => uri != null).cast<Uri>().asBroadcastStream();
  }
}

/// An implementation of [VMServiceDiscoveryForAttach] that delegates to other
/// [VMServiceDiscoveryForAttach] instances for discovery.
class DelegateVMServiceDiscoveryForAttach extends VMServiceDiscoveryForAttach {
  DelegateVMServiceDiscoveryForAttach(this.delegates);

  final List<VMServiceDiscoveryForAttach> delegates;

  @override
  Stream<Uri> get uris => StreamGroup.merge<Uri>(
    delegates.map((VMServiceDiscoveryForAttach delegate) => delegate.uris),
  );
}
