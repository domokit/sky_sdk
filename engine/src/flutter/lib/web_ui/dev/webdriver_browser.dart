// Copyright 2013 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

import 'dart:async';
import 'dart:io';
import 'dart:math';

import 'package:image/image.dart';
import 'package:webdriver/async_io.dart' show WebDriver;

import 'browser.dart';

/// Finds and returns an unused port on the test host in the local port range.
Future<int> pickUnusedPort() async {
  // Use bind to allocate an unused port, then unbind from that port to
  // make it available for use.
  final ServerSocket socket = await ServerSocket.bind('localhost', 0);
  final int port = socket.port;
  await socket.close();
  return port;
}

/// A browser managed through a WebDriver connection.
class WebDriverBrowser extends Browser {
  WebDriverBrowser(this._driver, this._url) {
    _driver.get(_url);
    _activateLoopFuture = () async {
      // Some browsers (i.e. Safari) stop actually executing our unit tests if
      // their window is occluded or non-visible. This hacky solution of
      // re-activating the window every two seconds prevents our unit tests from
      // stalling out if the window becomes obscured by some other thing that
      // may appear on the system.
      while (!_shouldStopActivating) {
        await (await _driver.window).setAsActive();
        await Future<void>.delayed(const Duration(seconds: 2));
      }
    }();
  }

  final WebDriver _driver;
  final Uri _url;
  final Completer<void> _onExitCompleter = Completer<void>();
  bool _shouldStopActivating = false;
  late final Future<void> _activateLoopFuture;

  @override
  Future<void> close() async {
    _shouldStopActivating = true;
    await _activateLoopFuture;

    await _driver.quit();
    if (!_onExitCompleter.isCompleted) {
      _onExitCompleter.complete();
    }
  }

  @override
  Future<void> get onExit => _onExitCompleter.future;

  @override
  bool get supportsScreenshots => true;

  @override
  Future<Image> captureScreenshot(Rectangle<num> region) async {
    final Image image = decodePng(await _driver.captureScreenshotAsList())!;
    return copyCrop(
      image,
      region.left.round(),
      region.top.round(),
      region.width.round(),
      region.height.round(),
    );
  }
}
