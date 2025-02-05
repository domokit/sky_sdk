// Copyright 2013 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

import 'package:test/bootstrap/browser.dart';
import 'package:test/test.dart';
import 'package:ui/src/engine.dart';
import 'package:ui/ui.dart' as ui;

import '../common/test_initialization.dart';

void main() {
  internalBootstrapBrowserTest(() => testMain);
}

Future<void> testMain() async {
  setUpUnitTests(emulateTesterEnvironment: false, setUpTestViewDimensions: false);

  test('$ResizingCodec gives correct repetition count for GIFs', () async {
    final ui.Codec codec = await renderer.instantiateImageCodecFromUrl(
      Uri(path: '/test_images/required.gif'),
    );
    final ui.Codec resizingCodec = ResizingCodec(codec);
    expect(resizingCodec.repetitionCount, 0);
  });
}
