// Copyright 2013 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

import 'dart:async';
import 'dart:io';

import 'package:path/path.dart' as path;
import 'package:test_api/backend.dart';
// TODO(ditman): Fix ignore when https://github.com/flutter/flutter/issues/143599 is resolved.
import 'package:test_core/src/util/io.dart'; // ignore: implementation_imports

import 'browser.dart';
import 'browser_process.dart';
import 'common.dart';
import 'environment.dart';
import 'firefox_installer.dart';
import 'package_lock.dart';

/// Provides an environment for the desktop Firefox.
class FirefoxEnvironment implements BrowserEnvironment {
  late final BrowserInstallation _installation;

  @override
  Future<Browser> launchBrowserInstance(Uri url, {bool debug = false}) async {
    return Firefox(url, _installation, debug: debug);
  }

  @override
  Runtime get packageTestRuntime => Runtime.firefox;

  @override
  Future<void> prepare() async {
    _installation = await getOrInstallFirefox(
      packageLock.firefoxLock.version,
      infoLog: isCi ? stdout : DevNull(),
    );
  }

  @override
  Future<void> cleanup() async {}

  @override
  final String name = 'Firefox';

  @override
  String get packageTestConfigurationYamlFile => 'dart_test_firefox.yaml';
}

/// Runs desktop Firefox.
///
/// Most of the communication with the browser is expected to happen via HTTP,
/// so this exposes a bare-bones API. The browser starts as soon as the class is
/// constructed, and is killed when [close] is called.
///
/// Any errors starting or running the process are reported through [onExit].
class Firefox extends Browser {
  /// Starts a new instance of Firefox open to the given [url], which may be a
  /// [Uri] or a [String].
  factory Firefox(Uri url, BrowserInstallation installation, {bool debug = false}) {
    final Completer<Uri> remoteDebuggerCompleter = Completer<Uri>.sync();
    return Firefox._(
      BrowserProcess(() async {
        // Using a profile on opening will prevent popups related to profiles.
        const String profile = '''
user_pref("browser.shell.checkDefaultBrowser", false);
user_pref("dom.disable_open_during_load", false);
user_pref("dom.max_script_run_time", 0);
''';

        final Directory temporaryProfileDirectory = Directory(
          path.join(environment.webUiDartToolDir.path, 'firefox_profile'),
        );

        // A good source of various Firefox Command Line options:
        // https://developer.mozilla.org/en-US/docs/Mozilla/Command_Line_Options#Browser
        //
        if (temporaryProfileDirectory.existsSync()) {
          temporaryProfileDirectory.deleteSync(recursive: true);
        }
        temporaryProfileDirectory.createSync(recursive: true);
        File(path.join(temporaryProfileDirectory.path, 'prefs.js')).writeAsStringSync(profile);

        final bool isMac = Platform.isMacOS;
        final List<String> args = <String>[
          url.toString(),
          '--profile',
          temporaryProfileDirectory.path,
          if (!debug) '--headless',
          '-width $kMaxScreenshotWidth',
          '-height $kMaxScreenshotHeight',
          // On Mac Firefox uses the -- option prefix, while elsewhere it uses the - prefix.
          '${isMac ? '-' : ''}-new-window',
          '${isMac ? '-' : ''}-new-instance',
          '--start-debugger-server $kDevtoolsPort',
        ];

        final Process process = await Process.start(installation.executable, args);

        remoteDebuggerCompleter.complete(
          getRemoteDebuggerUrl(Uri.parse('http://localhost:$kDevtoolsPort')),
        );

        unawaited(
          process.exitCode.then((_) {
            temporaryProfileDirectory.deleteSync(recursive: true);
          }),
        );

        return process;
      }),
      remoteDebuggerCompleter.future,
    );
  }

  Firefox._(this._process, this.remoteDebuggerUrl);

  final BrowserProcess _process;

  @override
  final Future<Uri> remoteDebuggerUrl;

  @override
  Future<void> get onExit => _process.onExit;

  @override
  Future<void> close() => _process.close();
}
