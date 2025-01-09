// Copyright 2013 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

import 'package:test/bootstrap/browser.dart';
import 'package:test/test.dart';
import 'package:ui/ui_web/src/ui_web.dart' as ui_web;

void main() {
  internalBootstrapBrowserTest(() => testMain);
}

void testMain() {
  group('detectBrowserEngineByVendorAgent', () {
    test('Should detect Blink', () {
      // Chrome Version 89.0.4389.90 (Official Build) (x86_64) / MacOS
      final ui_web.BrowserEngine browserEngine = ui_web.browser.detectBrowserEngineByVendorAgent(
        'Google Inc.',
        'mozilla/5.0 (macintosh; intel mac os x 11_2_3) applewebkit/537.36 '
            '(khtml, like gecko) chrome/89.0.4389.90 safari/537.36',
      );
      expect(browserEngine, ui_web.BrowserEngine.blink);
    });

    test('Should detect Firefox', () {
      // 85.0.2 (64-bit) / MacOS
      final ui_web.BrowserEngine browserEngine = ui_web.browser.detectBrowserEngineByVendorAgent(
        '',
        'mozilla/5.0 (macintosh; intel mac os x 10.16; rv:85.0) '
            'gecko/20100101 firefox/85.0',
      );
      expect(browserEngine, ui_web.BrowserEngine.firefox);
    });

    test('Should detect Safari', () {
      final ui_web.BrowserEngine browserEngine = ui_web.browser.detectBrowserEngineByVendorAgent(
        'Apple Computer, Inc.',
        'mozilla/5.0 (macintosh; intel mac os x 10_15_6) applewebkit/605.1.15 '
            '(khtml, like gecko) version/14.0.3 safari/605.1.15',
      );
      expect(browserEngine, ui_web.BrowserEngine.webkit);
    });
  });

  group('detectOperatingSystem', () {
    void expectOs(
      ui_web.OperatingSystem expectedOs, {
      String platform = 'any',
      String ua = 'any',
      int touchPoints = 0,
    }) {
      try {
        ui_web.browser.debugUserAgentOverride = ua;
        expect(
          ui_web.browser.detectOperatingSystem(
            overridePlatform: platform,
            overrideMaxTouchPoints: touchPoints,
          ),
          expectedOs,
        );
      } finally {
        ui_web.browser.debugUserAgentOverride = null;
      }
    }

    test('Determine unknown for weird values of platform/ua', () {
      expectOs(ui_web.OperatingSystem.unknown);
    });

    test('Determine MacOS if platform starts by Mac', () {
      expectOs(ui_web.OperatingSystem.macOs, platform: 'MacIntel');
      expectOs(ui_web.OperatingSystem.macOs, platform: 'MacAnythingElse');
    });

    test('Determine iOS if platform contains iPhone/iPad/iPod', () {
      expectOs(ui_web.OperatingSystem.iOs, platform: 'iPhone');
      expectOs(ui_web.OperatingSystem.iOs, platform: 'iPhone Simulator');
      expectOs(ui_web.OperatingSystem.iOs, platform: 'iPad');
      expectOs(ui_web.OperatingSystem.iOs, platform: 'iPad Simulator');
      expectOs(ui_web.OperatingSystem.iOs, platform: 'iPod');
      expectOs(ui_web.OperatingSystem.iOs, platform: 'iPod Simulator');
    });

    // See https://github.com/flutter/flutter/issues/81918
    test('Tell apart MacOS from iOS requesting a desktop site.', () {
      expectOs(ui_web.OperatingSystem.macOs, platform: 'MacARM');

      expectOs(ui_web.OperatingSystem.iOs, platform: 'MacARM', touchPoints: 5);
    });

    test('Determine Android if user agent contains Android', () {
      expectOs(
        ui_web.OperatingSystem.android,
        ua:
            'Mozilla/5.0 (Linux; U; Android 2.2) AppleWebKit/533.1 (KHTML, like Gecko) Version/4.0 Mobile Safari/533.1',
      );
    });

    test('Determine Linux if the platform begins with Linux', () {
      expectOs(ui_web.OperatingSystem.linux, platform: 'Linux');
      expectOs(ui_web.OperatingSystem.linux, platform: 'Linux armv8l');
      expectOs(ui_web.OperatingSystem.linux, platform: 'Linux x86_64');
    });

    test('Determine Windows if the platform begins with Win', () {
      expectOs(ui_web.OperatingSystem.windows, platform: 'Windows');
      expectOs(ui_web.OperatingSystem.windows, platform: 'Win32');
      expectOs(ui_web.OperatingSystem.windows, platform: 'Win16');
      expectOs(ui_web.OperatingSystem.windows, platform: 'WinCE');
    });
  });
}
