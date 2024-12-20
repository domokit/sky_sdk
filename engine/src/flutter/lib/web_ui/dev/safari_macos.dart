// Copyright 2013 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

import 'dart:async';
import 'dart:convert';
import 'dart:io';

import 'package:test_api/backend.dart';

import 'webdriver_browser.dart';

/// Provides an environment for the desktop variant of Safari running on macOS.
class SafariMacOsEnvironment extends WebDriverBrowserEnvironment {
  @override
  final String name = 'Safari macOS';

  @override
  Runtime get packageTestRuntime => Runtime.safari;

  @override
  String get packageTestConfigurationYamlFile => 'dart_test_safari.yaml';

  @override
  Uri get driverUri => Uri(scheme: 'http', host: 'localhost', port: portNumber);

  late Process _driverProcess;
  int _retryCount = 0;
  static const int _waitBetweenRetryInSeconds = 1;
  static const int _maxRetryCount = 10;

  @override
  Future<Process> spawnDriverProcess() =>
      Process.start('safaridriver', <String>['-p', portNumber.toString()]);

  @override
  Future<void> prepare() async {
    await _startDriverProcess();
  }

  /// Pick an unused port and start `safaridriver` using that port.
  ///
  /// On macOS 13, starting `safaridriver` can be flaky so if it returns an
  /// "Operation not permitted" error, kill the `safaridriver` process and try
  /// again with a different port. Wait [_waitBetweenRetryInSeconds] seconds
  /// between retries. Try up to [_maxRetryCount] times.
  Future<void> _startDriverProcess() async {
    _retryCount += 1;
    if (_retryCount > 1) {
      await Future<void>.delayed(const Duration(seconds: _waitBetweenRetryInSeconds));
    }
    portNumber = await pickUnusedPort();

    print('Attempt $_retryCount to start safaridriver on port $portNumber');

    _driverProcess = await spawnDriverProcess();

    _driverProcess.stderr.transform(utf8.decoder).transform(const LineSplitter()).listen((
      String error,
    ) {
      print('[Webdriver][Error] $error');
      if (_retryCount > _maxRetryCount) {
        print('[Webdriver][Error] Failed to start after $_maxRetryCount tries.');
      } else if (error.contains('Operation not permitted')) {
        _driverProcess.kill();
        _startDriverProcess();
      }
    });
    _driverProcess.stdout.transform(utf8.decoder).transform(const LineSplitter()).listen((
      String log,
    ) {
      print('[Webdriver] $log');
    });
  }

  @override
  Future<void> cleanup() async {
    _driverProcess.kill();
  }
}
