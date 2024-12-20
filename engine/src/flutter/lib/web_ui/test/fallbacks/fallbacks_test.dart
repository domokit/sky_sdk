// Copyright 2013 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

import 'dart:js_interop';

import 'package:test/bootstrap/browser.dart';
import 'package:test/test.dart';
import 'package:ui/src/engine.dart';
import 'package:ui/ui_web/src/ui_web.dart' as ui_web;

import '../common/test_initialization.dart';
import '../ui/utils.dart';

void main() {
  internalBootstrapBrowserTest(() => testMain);
}

@JS()
external JSBoolean get crossOriginIsolated;

Future<void> testMain() async {
  setUpUnitTests(setUpTestViewDimensions: false);

  test('bootstrapper selects correct builds', () {
    if (ui_web.browser.browserEngine == ui_web.BrowserEngine.blink) {
      expect(isWasm, isTrue);
      expect(isSkwasm, isTrue);
      final bool shouldBeMultiThreaded =
          crossOriginIsolated.toDart && !configuration.forceSingleThreadedSkwasm;
      expect(isMultiThreaded, shouldBeMultiThreaded);
    } else {
      expect(isWasm, isFalse);
      expect(isCanvasKit, isTrue);
    }
  });
}
