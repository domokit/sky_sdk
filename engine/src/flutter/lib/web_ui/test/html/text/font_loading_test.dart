// Copyright 2013 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

import 'dart:async';
import 'dart:convert';
import 'dart:typed_data';

import 'package:test/bootstrap/browser.dart';
import 'package:test/test.dart';
import 'package:ui/src/engine.dart';
import 'package:ui/ui.dart' as ui;
import 'package:ui/ui_web/src/ui_web.dart' as ui_web;

import '../../common/test_initialization.dart';

void main() {
  internalBootstrapBrowserTest(() => testMain);
}

Future<void> testMain() async {
  setUpUnitTests(withImplicitView: true);
  group('loadFontFromList', () {
    const String testFontUrl = '/assets/fonts/ahem.ttf';

    tearDown(() {
      domDocument.fonts!.clear();
    });

    test(
      'returns normally from invalid font buffer',
      () async {
        await expectLater(
          () async => ui.loadFontFromList(Uint8List(0), fontFamily: 'test-font'),
          returnsNormally,
        );
      },
      // TODO(hterkelsen): https://github.com/flutter/flutter/issues/56702
      skip: ui_web.browser.browserEngine == ui_web.BrowserEngine.webkit,
    );

    test(
      'loads Blehm font from buffer',
      () async {
        expect(_containsFontFamily('Blehm'), isFalse);

        final ByteBuffer response = await httpFetchByteBuffer(testFontUrl);
        await ui.loadFontFromList(response.asUint8List(), fontFamily: 'Blehm');

        expect(_containsFontFamily('Blehm'), isTrue);
      },
      // TODO(hterkelsen): https://github.com/flutter/flutter/issues/56702
      skip: ui_web.browser.browserEngine == ui_web.BrowserEngine.webkit,
    );

    test(
      'loading font should clear measurement caches',
      () async {
        final EngineParagraphStyle style = EngineParagraphStyle();
        const ui.ParagraphConstraints constraints = ui.ParagraphConstraints(width: 30.0);

        final CanvasParagraphBuilder canvasBuilder = CanvasParagraphBuilder(style);
        canvasBuilder.addText('test');
        // Triggers the measuring and verifies the ruler cache has been populated.
        canvasBuilder.build().layout(constraints);
        expect(Spanometer.rulers.length, 1);

        // Now, loads a new font using loadFontFromList. This should clear the
        // cache
        final ByteBuffer response = await httpFetchByteBuffer(testFontUrl);
        await ui.loadFontFromList(response.asUint8List(), fontFamily: 'Blehm');

        // Verifies the font is loaded, and the cache is cleaned.
        expect(_containsFontFamily('Blehm'), isTrue);
        expect(Spanometer.rulers.length, 0);
      },
      // TODO(hterkelsen): https://github.com/flutter/flutter/issues/56702
      skip: ui_web.browser.browserEngine == ui_web.BrowserEngine.webkit,
    );

    test(
      'loading font should send font change message',
      () async {
        final ui.PlatformMessageCallback? oldHandler =
            ui.PlatformDispatcher.instance.onPlatformMessage;
        String? actualName;
        String? message;
        ui.PlatformDispatcher.instance.onPlatformMessage = (
          String name,
          ByteData? data,
          ui.PlatformMessageResponseCallback? callback,
        ) {
          actualName = name;
          final ByteBuffer buffer = data!.buffer;
          final Uint8List list = buffer.asUint8List(data.offsetInBytes, data.lengthInBytes);
          message = utf8.decode(list);
        };
        final ByteBuffer response = await httpFetchByteBuffer(testFontUrl);
        await ui.loadFontFromList(response.asUint8List(), fontFamily: 'Blehm');
        final Completer<void> completer = Completer<void>();
        domWindow.requestAnimationFrame((_) {
          completer.complete();
        });
        await completer.future;
        ui.PlatformDispatcher.instance.onPlatformMessage = oldHandler;
        expect(actualName, 'flutter/system');
        expect(message, '{"type":"fontsChange"}');
      },
      // TODO(hterkelsen): https://github.com/flutter/flutter/issues/56702
      skip: ui_web.browser.browserEngine == ui_web.BrowserEngine.webkit,
    );
  });
}

bool _containsFontFamily(String family) {
  bool found = false;
  domDocument.fonts!.forEach((
    DomFontFace fontFace,
    DomFontFace fontFaceAgain,
    DomFontFaceSet fontFaceSet,
  ) {
    if (fontFace.family == family) {
      found = true;
    }
  });
  return found;
}
