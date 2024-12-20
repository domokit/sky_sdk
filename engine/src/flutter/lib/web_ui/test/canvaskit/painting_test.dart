// Copyright 2013 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

import 'package:test/bootstrap/browser.dart';
import 'package:test/test.dart';

import 'package:ui/src/engine.dart';

import 'common.dart';

void main() {
  internalBootstrapBrowserTest(() => testMain);
}

void testMain() {
  group('CkPaint', () {
    setUpCanvasKitTest();

    test('toSkPaint', () {
      final paint = CkPaint();
      final skPaint = paint.toSkPaint();
      expect(skPaint, isNotNull);
      skPaint.delete();
    });
  });
}
