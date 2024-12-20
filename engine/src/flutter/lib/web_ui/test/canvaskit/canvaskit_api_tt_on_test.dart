// Copyright 2013 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

import 'package:test/bootstrap/browser.dart';
import 'package:test/test.dart';
import 'package:ui/src/engine.dart';
import 'package:ui/ui_web/src/ui_web.dart' as ui_web;

import '../common/matchers.dart';
import 'canvaskit_api_test.dart';

final bool isBlink = ui_web.browser.browserEngine == ui_web.BrowserEngine.blink;

const String goodUrl = 'https://www.unpkg.com/blah-blah/33.x/canvaskit.js';
const String badUrl = 'https://www.unpkg.com/soemthing/not-canvaskit.js';

// These tests need to happen in a separate file, because a Content Security
// Policy cannot be relaxed once set, only made more strict.
void main() {
  internalBootstrapBrowserTest(() => testMainWithTTOn);
}

// Enables Trusted Types, runs all `canvaskit_api_test.dart`, then tests the
// createTrustedScriptUrl function.
void testMainWithTTOn() {
  enableTrustedTypes();

  // Run all standard canvaskit tests, with TT on...
  testMain();

  group('TrustedTypes API supported', () {
    test('createTrustedScriptUrl - returns TrustedScriptURL object', () async {
      final Object trusted = createTrustedScriptUrl(goodUrl);

      expect(trusted, isA<DomTrustedScriptURL>());
      expect((trusted as DomTrustedScriptURL).url, goodUrl);
    });

    test('createTrustedScriptUrl - rejects bad canvaskit.js URL', () async {
      expect(() {
        createTrustedScriptUrl(badUrl);
      }, throwsAssertionError);
    });
  }, skip: !isBlink);

  group('Trusted Types API NOT supported', () {
    test('createTrustedScriptUrl - returns unmodified url', () async {
      expect(createTrustedScriptUrl(badUrl), badUrl);
    });
  }, skip: isBlink);
}

/// Enables Trusted Types by setting the appropriate meta tag in the DOM:
/// <meta http-equiv="Content-Security-Policy" content="require-trusted-types-for 'script'">
void enableTrustedTypes() {
  print('Enabling TrustedTypes in browser window...');
  final DomHTMLMetaElement enableTTMeta =
      createDomHTMLMetaElement()
        ..setAttribute('http-equiv', 'Content-Security-Policy')
        ..content = "require-trusted-types-for 'script'";
  domDocument.head!.append(enableTTMeta);
}
