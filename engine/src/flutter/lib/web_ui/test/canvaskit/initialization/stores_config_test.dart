// Copyright 2013 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

import 'package:js/js_util.dart' as js_util;
import 'package:test/bootstrap/browser.dart';
import 'package:test/test.dart';
import 'package:ui/src/engine.dart';

void main() {
  internalBootstrapBrowserTest(() => testMain);
}

void testMain() {
  group('initializeEngineServices', () {
    test('stores user configuration', () async {
      final JsFlutterConfiguration config = JsFlutterConfiguration();
      // `canvasKitBaseUrl` is required for the test to actually run.
      js_util.setProperty(config, 'canvasKitBaseUrl', '/canvaskit/');
      // A property under test, that we'll try to read later.
      js_util.setProperty(config, 'nonce', 'some_nonce');
      // A non-existing property to verify our js-interop doesn't crash.
      js_util.setProperty(config, 'nonexistentProperty', 32.0);

      // Remove window.flutterConfiguration (if it's there)
      js_util.setProperty(domWindow, 'flutterConfiguration', null);

      // TODO(web): Replace the above nullification by the following assertion
      // when wasm and JS tests initialize their config the same way:
      // assert(js_util.getProperty<Object?>(domWindow, 'flutterConfiguration') == null);

      await initializeEngineServices(jsConfiguration: config);

      expect(configuration.nonce, 'some_nonce');
    });
  });
}
