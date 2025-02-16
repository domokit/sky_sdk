// Copyright 2013 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

import 'dart:async';
import 'dart:convert';
import 'dart:io';

import 'package:test_api/backend.dart';
import 'package:webdriver/async_io.dart' show WebDriver, createDriver;

import 'browser.dart';
import 'webdriver_browser.dart';

/// Provides an environment for the desktop variant of Safari running on macOS.
class SafariMacOsEnvironment extends BrowserEnvironment {
  static const Duration _waitBetweenRetries = Duration(seconds: 1);
  static const int _maxRetryCount = 5;

  late int _portNumber;
  late Process _driverProcess;
  Uri get _driverUri => Uri(scheme: 'http', host: 'localhost', port: _portNumber);
  WebDriver? webDriver;

  @override
  final String name = 'Safari macOS';

  @override
  Runtime get packageTestRuntime => Runtime.safari;

  @override
  String get packageTestConfigurationYamlFile => 'dart_test_safari.yaml';

  @override
  Future<void> prepare() async {
    int retryCount = 0;

    while (true) {
      try {
        if (retryCount > 0) {
          print('Retry #$retryCount');
        }
        retryCount += 1;
        await _startDriverProcess();
        return;
      } catch (error, stackTrace) {
        if (retryCount < _maxRetryCount) {
          print('''
Failed to start safaridriver:

Error: $error
$stackTrace
''');
          print('Will try again.');
          await Future<void>.delayed(_waitBetweenRetries);
        } else {
          print('Too many retries. Giving up.');
          rethrow;
        }
      }
    }
  }

  /// Pick an unused port and start `safaridriver` using that port.
  ///
  /// On macOS 13, starting `safaridriver` can be flaky so if it returns an
  /// "Operation not permitted" error, kill the `safaridriver` process and try
  /// again with a different port. Wait [_waitBetweenRetryInSeconds] seconds
  /// between retries. Try up to [_maxRetryCount] times.
  Future<void> _startDriverProcess() async {
    _portNumber = await pickUnusedPort();
    print('Starting safaridriver on port $_portNumber');

    try {
      _driverProcess = await Process.start('safaridriver', <String>['-p', _portNumber.toString()]);

      _driverProcess.stdout.transform(utf8.decoder).transform(const LineSplitter()).listen((
        String log,
      ) {
        print('[safaridriver] $log');
      });

      _driverProcess.stderr.transform(utf8.decoder).transform(const LineSplitter()).listen((
        String error,
      ) {
        print('[safaridriver][error] $error');
      });

      await _waitForSafariDriverServerReady();

      // Smoke-test the web driver process by connecting to it and asking for a
      // list of windows. It doesn't matter how many windows there are.
      webDriver = await createDriver(
        uri: _driverUri,
        desired: <String, dynamic>{'browserName': packageTestRuntime.identifier},
      );

      await webDriver!.windows.toList();
    } catch (_) {
      print('safaridriver failed to start.');

      final badDriver = webDriver;
      webDriver = null; // let's not keep faulty driver around

      if (badDriver != null) {
        // This means the launch process got to a point where a WebDriver
        // instance was created, but it failed the smoke test. To make sure no
        // stray driver sessions are left hanging, try to close the session.
        try {
          // The method is called "quit" but all it does is close the session.
          //
          // See: https://www.w3.org/TR/webdriver2/#delete-session
          await badDriver.quit();
        } catch (error, stackTrace) {
          // Just print. Do not rethrow. The attempt to close the session is
          // only a best-effort thing.
          print('''
Failed to close driver session. Will try to kill the safaridriver process.

Error: $error
$stackTrace
''');
        }
      }

      // Try to kill gracefully using SIGTERM first.
      _driverProcess.kill();
      await _driverProcess.exitCode.timeout(
        const Duration(seconds: 2),
        onTimeout: () async {
          // If the process fails to exit gracefully in a reasonable amount of
          // time, kill it forcefully.
          print('safaridriver failed to exit normally. Killing with SIGKILL.');
          _driverProcess.kill(ProcessSignal.sigkill);
          return 0;
        },
      );

      // Rethrow the error to allow the caller to retry, if need be.
      rethrow;
    }
  }

  /// The Safari Driver process cannot instantly spawn a server, so this function
  /// attempts to connect to the server in a loop until it succeeds.
  ///
  /// A healthy driver process is expected to respond to a `GET /status` HTTP
  /// request with `{value: {ready: true}}` JSON response.
  ///
  /// See also: https://www.w3.org/TR/webdriver2/#status
  Future<void> _waitForSafariDriverServerReady() async {
    // Wait just a tiny bit before connecting for the very first time because
    // frequently safaridriver isn't quick enough to bring up the server.
    //
    // 100ms seems enough in most cases, but feel free to revisit this.
    await Future<void>.delayed(const Duration(milliseconds: 100));

    int retryCount = 0;
    while (true) {
      retryCount += 1;
      final httpClient = HttpClient();
      try {
        final request = await httpClient.get('localhost', _portNumber, '/status');
        final response = await request.close();
        final stringData = await response.transform(utf8.decoder).join();
        final jsonResponse = json.decode(stringData) as Map<String, Object?>;
        final value = jsonResponse['value']! as Map<String, Object?>;
        final ready = value['ready']! as bool;
        if (ready) {
          break;
        }
      } catch (_) {
        if (retryCount < 10) {
          print('safaridriver not ready yet. Waiting...');
          await Future<void>.delayed(const Duration(milliseconds: 100));
        } else {
          print(
            'safaridriver failed to reach ready state in a reasonable amount of time. Giving up.',
          );
          rethrow;
        }
      }
    }
  }

  @override
  Future<Browser> launchBrowserInstance(Uri url, {bool debug = false}) async {
    return WebDriverBrowser(webDriver!, url);
  }

  @override
  Future<void> cleanup() async {
    // WebDriver.quit() is not called here, because that's done in
    // WebDriverBrowser.close().
    _driverProcess.kill();
  }
}
