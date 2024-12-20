// Copyright 2013 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

import 'package:test/bootstrap/browser.dart';
import 'package:test/test.dart';
import 'package:ui/src/engine.dart';

void main() {
  internalBootstrapBrowserTest(() => testMain);
}

void testMain() {
  test('CanvasKit reuses the instance already set on `window`', () async {
    // First initialization should make CanvasKit available through `window`.
    await renderer.initialize();
    expect(windowFlutterCanvasKit, isNotNull);

    // Remember the initial instance.
    final CanvasKit firstCanvasKitInstance = windowFlutterCanvasKit!;

    // Try to load CanvasKit again.
    await renderer.initialize();

    // Should find the existing instance and reuse it.
    expect(firstCanvasKitInstance, windowFlutterCanvasKit);
  });
}
