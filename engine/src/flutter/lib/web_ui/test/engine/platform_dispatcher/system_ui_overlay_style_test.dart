// Copyright 2013 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

import 'package:test/bootstrap/browser.dart';
import 'package:test/test.dart';
import 'package:ui/src/engine.dart';
import 'package:ui/ui.dart' as ui;
import 'package:ui/ui_web/src/ui_web.dart' as ui_web;

void main() {
  internalBootstrapBrowserTest(() => testMain);
}

void testMain() {
  const MethodCodec codec = JSONMethodCodec();

  void sendSetSystemUIOverlayStyle({ui.Color? statusBarColor}) {
    ui.PlatformDispatcher.instance.sendPlatformMessage(
      'flutter/platform',
      codec.encodeMethodCall(
        MethodCall('SystemChrome.setSystemUIOverlayStyle', <String, dynamic>{
          'statusBarColor': statusBarColor?.value,
        }),
      ),
      null,
    );
  }

  String? getCssThemeColor() {
    final DomHTMLMetaElement? theme =
        domDocument.querySelector('#flutterweb-theme') as DomHTMLMetaElement?;
    return theme?.content;
  }

  group('SystemUIOverlayStyle', () {
    test('theme color is set / removed by platform message', () {
      // Run the unit test without emulating Flutter tester environment.
      ui_web.debugEmulateFlutterTesterEnvironment = false;

      expect(getCssThemeColor(), null);

      const ui.Color statusBarColor = ui.Color(0xFFF44336);
      sendSetSystemUIOverlayStyle(statusBarColor: statusBarColor);
      expect(getCssThemeColor(), statusBarColor.toCssString());

      sendSetSystemUIOverlayStyle();
      expect(getCssThemeColor(), null);
    });
  });
}
