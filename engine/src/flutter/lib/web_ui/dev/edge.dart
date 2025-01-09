// Copyright 2013 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

import 'dart:async';
import 'dart:io';

import 'package:test_api/backend.dart';

import 'browser.dart';
import 'browser_process.dart';
import 'common.dart';
import 'edge_installation.dart';
import 'package_lock.dart';

/// Provides an environment for the desktop Microsoft Edge (Chromium-based).
class EdgeEnvironment implements BrowserEnvironment {
  @override
  final String name = 'Edge';

  @override
  Future<Browser> launchBrowserInstance(Uri url, {bool debug = false}) async {
    return Edge(url);
  }

  @override
  Runtime get packageTestRuntime => Runtime.edge;

  @override
  Future<void> prepare() async {
    // Edge doesn't need any special prep.
  }

  @override
  Future<void> cleanup() async {}

  @override
  String get packageTestConfigurationYamlFile => 'dart_test_edge.yaml';
}

/// Runs desktop Edge.
///
/// Most of the communication with the browser is expected to happen via HTTP,
/// so this exposes a bare-bones API. The browser starts as soon as the class is
/// constructed, and is killed when [close] is called.
///
/// Any errors starting or running the process are reported through [onExit].
class Edge extends Browser {
  /// Starts a new instance of Safari open to the given [url], which may be a
  /// [Uri] or a [String].
  factory Edge(Uri url) {
    return Edge._(
      BrowserProcess(() async {
        final BrowserInstallation installation = await getEdgeInstallation(
          packageLock.edgeLock.launcherVersion,
          infoLog: DevNull(),
        );

        // Debug is not a valid option for Edge. Remove it.
        String pathToOpen = url.toString();
        if (pathToOpen.contains('debug')) {
          final int index = pathToOpen.indexOf('debug');
          pathToOpen = pathToOpen.substring(0, index - 1);
        }

        final Process process = await Process.start(installation.executable, <String>[
          pathToOpen,
          '-k',
        ]);

        return process;
      }),
    );
  }

  Edge._(this._process);

  final BrowserProcess _process;

  @override
  Future<void> get onExit => _process.onExit;

  @override
  Future<void> close() => _process.close();
}
