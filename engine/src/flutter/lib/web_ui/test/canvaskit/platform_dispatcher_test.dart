// Copyright 2013 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

import 'dart:async';
import 'dart:typed_data';

import 'package:test/bootstrap/browser.dart';
import 'package:test/test.dart';

import 'package:ui/src/engine.dart';
import 'package:ui/ui.dart' as ui;

import 'common.dart';

void main() {
  internalBootstrapBrowserTest(() => testMain);
}

void testMain() {
  group('PlatformDispatcher', () {
    setUpCanvasKitTest();

    test('responds to flutter/skia Skia.setResourceCacheMaxBytes', () async {
      const MethodCodec codec = JSONMethodCodec();
      final Completer<ByteData?> completer = Completer<ByteData?>();
      ui.PlatformDispatcher.instance.sendPlatformMessage(
        'flutter/skia',
        codec.encodeMethodCall(
          const MethodCall('Skia.setResourceCacheMaxBytes', 512 * 1000 * 1000),
        ),
        completer.complete,
      );

      final ByteData? response = await completer.future;
      expect(response, isNotNull);
      expect(codec.decodeEnvelope(response!), <bool>[true]);
    });
  });
}
