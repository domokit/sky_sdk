// Copyright 2014 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

import 'dart:io' as io;
import 'dart:typed_data';

import 'package:flutter_tools/src/base/io.dart';
import 'package:flutter_tools/src/base/platform.dart';
import 'package:flutter_tools/src/doctor_validator.dart';
import 'package:flutter_tools/src/proxy_validator.dart';

import '../../src/common.dart';

void main() {
  setUp(() {
    setNetworkInterfaceLister(({
      bool includeLoopback = true,
      bool includeLinkLocal = true,
      InternetAddressType type = InternetAddressType.any,
    }) async {
      final List<FakeNetworkInterface> interfaces = <FakeNetworkInterface>[
        FakeNetworkInterface(<FakeInternetAddress>[const FakeInternetAddress('127.0.0.1')]),
        FakeNetworkInterface(<FakeInternetAddress>[const FakeInternetAddress('::1')]),
      ];

      return Future<List<NetworkInterface>>.value(interfaces);
    });
  });

  tearDown(() {
    resetNetworkInterfaceLister();
  });

  testWithoutContext('ProxyValidator does not show if HTTP_PROXY is not set', () {
    final Platform platform = FakePlatform(environment: <String, String>{});

    expect(ProxyValidator(platform: platform).shouldShow, isFalse);
  });

  testWithoutContext('ProxyValidator does not show if HTTP_PROXY is only whitespace', () {
    final Platform platform = FakePlatform(environment: <String, String>{'HTTP_PROXY': ' '});

    expect(ProxyValidator(platform: platform).shouldShow, isFalse);
  });

  testWithoutContext('ProxyValidator shows when HTTP_PROXY is set', () {
    final Platform platform = FakePlatform(
      environment: <String, String>{'HTTP_PROXY': 'fakeproxy.local'},
    );

    expect(ProxyValidator(platform: platform).shouldShow, isTrue);
  });

  testWithoutContext('ProxyValidator shows when http_proxy is set', () {
    final Platform platform = FakePlatform(
      environment: <String, String>{'http_proxy': 'fakeproxy.local'},
    );

    expect(ProxyValidator(platform: platform).shouldShow, isTrue);
  });

  testWithoutContext(
    'ProxyValidator reports success when NO_PROXY is configured correctly',
    () async {
      final Platform platform = FakePlatform(
        environment: <String, String>{
          'HTTP_PROXY': 'fakeproxy.local',
          'NO_PROXY': 'localhost,127.0.0.1,::1',
        },
      );
      final ValidationResult results = await ProxyValidator(platform: platform).validate();

      expect(results.messages, const <ValidationMessage>[
        ValidationMessage('HTTP_PROXY is set'),
        ValidationMessage('NO_PROXY is localhost,127.0.0.1,::1'),
        ValidationMessage('NO_PROXY contains localhost'),
        ValidationMessage('NO_PROXY contains 127.0.0.1'),
        ValidationMessage('NO_PROXY contains ::1'),
      ]);
    },
  );

  testWithoutContext(
    'ProxyValidator reports success when no_proxy is configured correctly',
    () async {
      final Platform platform = FakePlatform(
        environment: <String, String>{
          'http_proxy': 'fakeproxy.local',
          'no_proxy': 'localhost,127.0.0.1,::1',
        },
      );
      final ValidationResult results = await ProxyValidator(platform: platform).validate();

      expect(results.messages, const <ValidationMessage>[
        ValidationMessage('HTTP_PROXY is set'),
        ValidationMessage('NO_PROXY is localhost,127.0.0.1,::1'),
        ValidationMessage('NO_PROXY contains localhost'),
        ValidationMessage('NO_PROXY contains 127.0.0.1'),
        ValidationMessage('NO_PROXY contains ::1'),
      ]);
    },
  );

  testWithoutContext('ProxyValidator reports issues when NO_PROXY is missing localhost', () async {
    final Platform platform = FakePlatform(
      environment: <String, String>{'HTTP_PROXY': 'fakeproxy.local', 'NO_PROXY': '127.0.0.1,::1'},
    );
    final ValidationResult results = await ProxyValidator(platform: platform).validate();

    expect(results.messages, const <ValidationMessage>[
      ValidationMessage('HTTP_PROXY is set'),
      ValidationMessage('NO_PROXY is 127.0.0.1,::1'),
      ValidationMessage.hint('NO_PROXY does not contain localhost'),
      ValidationMessage('NO_PROXY contains 127.0.0.1'),
      ValidationMessage('NO_PROXY contains ::1'),
    ]);
  });

  testWithoutContext('ProxyValidator reports issues when NO_PROXY is missing 127.0.0.1', () async {
    final Platform platform = FakePlatform(
      environment: <String, String>{'HTTP_PROXY': 'fakeproxy.local', 'NO_PROXY': 'localhost,::1'},
    );
    final ValidationResult results = await ProxyValidator(platform: platform).validate();

    expect(results.messages, const <ValidationMessage>[
      ValidationMessage('HTTP_PROXY is set'),
      ValidationMessage('NO_PROXY is localhost,::1'),
      ValidationMessage('NO_PROXY contains localhost'),
      ValidationMessage.hint('NO_PROXY does not contain 127.0.0.1'),
      ValidationMessage('NO_PROXY contains ::1'),
    ]);
  });

  testWithoutContext('ProxyValidator reports issues when NO_PROXY is missing ::1', () async {
    final Platform platform = FakePlatform(
      environment: <String, String>{
        'HTTP_PROXY': 'fakeproxy.local',
        'NO_PROXY': 'localhost,127.0.0.1',
      },
    );
    final ValidationResult results = await ProxyValidator(platform: platform).validate();

    expect(results.messages, const <ValidationMessage>[
      ValidationMessage('HTTP_PROXY is set'),
      ValidationMessage('NO_PROXY is localhost,127.0.0.1'),
      ValidationMessage('NO_PROXY contains localhost'),
      ValidationMessage('NO_PROXY contains 127.0.0.1'),
      ValidationMessage.hint('NO_PROXY does not contain ::1'),
    ]);
  });

  testWithoutContext(
    'ProxyValidator reports issues when NO_PROXY is missing localhost, 127.0.0.1',
    () async {
      final Platform platform = FakePlatform(
        environment: <String, String>{'HTTP_PROXY': 'fakeproxy.local', 'NO_PROXY': '::1'},
      );
      final ValidationResult results = await ProxyValidator(platform: platform).validate();

      expect(results.messages, const <ValidationMessage>[
        ValidationMessage('HTTP_PROXY is set'),
        ValidationMessage('NO_PROXY is ::1'),
        ValidationMessage.hint('NO_PROXY does not contain localhost'),
        ValidationMessage.hint('NO_PROXY does not contain 127.0.0.1'),
        ValidationMessage('NO_PROXY contains ::1'),
      ]);
    },
  );

  testWithoutContext(
    'ProxyValidator reports issues when NO_PROXY is missing localhost, ::1',
    () async {
      final Platform platform = FakePlatform(
        environment: <String, String>{'HTTP_PROXY': 'fakeproxy.local', 'NO_PROXY': '127.0.0.1'},
      );
      final ValidationResult results = await ProxyValidator(platform: platform).validate();

      expect(results.messages, const <ValidationMessage>[
        ValidationMessage('HTTP_PROXY is set'),
        ValidationMessage('NO_PROXY is 127.0.0.1'),
        ValidationMessage.hint('NO_PROXY does not contain localhost'),
        ValidationMessage('NO_PROXY contains 127.0.0.1'),
        ValidationMessage.hint('NO_PROXY does not contain ::1'),
      ]);
    },
  );

  testWithoutContext(
    'ProxyValidator reports issues when NO_PROXY is missing 127.0.0.1, ::1',
    () async {
      final Platform platform = FakePlatform(
        environment: <String, String>{'HTTP_PROXY': 'fakeproxy.local', 'NO_PROXY': 'localhost'},
      );
      final ValidationResult results = await ProxyValidator(platform: platform).validate();

      expect(results.messages, const <ValidationMessage>[
        ValidationMessage('HTTP_PROXY is set'),
        ValidationMessage('NO_PROXY is localhost'),
        ValidationMessage('NO_PROXY contains localhost'),
        ValidationMessage.hint('NO_PROXY does not contain 127.0.0.1'),
        ValidationMessage.hint('NO_PROXY does not contain ::1'),
      ]);
    },
  );
}

class FakeNetworkInterface extends NetworkInterface {
  FakeNetworkInterface(List<FakeInternetAddress> addresses)
    : super(FakeNetworkInterfaceDelegate(addresses));

  @override
  String get name => 'FakeNetworkInterface$index';
}

class FakeNetworkInterfaceDelegate implements io.NetworkInterface {
  FakeNetworkInterfaceDelegate(this._fakeAddresses);

  final List<FakeInternetAddress> _fakeAddresses;

  @override
  List<io.InternetAddress> get addresses => _fakeAddresses;

  @override
  int get index => addresses.length;

  @override
  String get name => 'FakeNetworkInterfaceDelegate$index';
}

class FakeInternetAddress implements io.InternetAddress {
  const FakeInternetAddress(this._fakeAddress);

  final String _fakeAddress;

  @override
  String get address => _fakeAddress;

  @override
  String get host => throw UnimplementedError();

  @override
  bool get isLinkLocal => throw UnimplementedError();

  @override
  bool get isLoopback => true;

  @override
  bool get isMulticast => throw UnimplementedError();

  @override
  Uint8List get rawAddress => throw UnimplementedError();

  @override
  Future<io.InternetAddress> reverse() => throw UnimplementedError();

  @override
  io.InternetAddressType get type => throw UnimplementedError();
}
