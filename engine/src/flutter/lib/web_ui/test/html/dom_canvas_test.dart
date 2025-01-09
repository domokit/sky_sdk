// Copyright 2013 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

import 'package:test/bootstrap/browser.dart';
import 'package:test/test.dart';
import 'package:ui/src/engine.dart';
import 'package:ui/ui.dart';

void main() {
  internalBootstrapBrowserTest(() => testMain);
}

Future<void> testMain() async {
  group('$adjustRectForDom', () {
    test('does not change rect when not necessary', () async {
      const Rect rect = Rect.fromLTWH(10, 20, 140, 160);
      expect(adjustRectForDom(rect, SurfacePaintData()..style = PaintingStyle.fill), rect);
      expect(
        adjustRectForDom(
          rect,
          SurfacePaintData()
            ..style = PaintingStyle.stroke
            ..strokeWidth = 0,
        ),
        rect,
      );
    });

    test('takes stroke width into consideration', () async {
      const Rect rect = Rect.fromLTWH(10, 20, 140, 160);
      expect(
        adjustRectForDom(
          rect,
          SurfacePaintData()
            ..style = PaintingStyle.stroke
            ..strokeWidth = 1,
        ),
        const Rect.fromLTWH(9.5, 19.5, 139, 159),
      );
      expect(
        adjustRectForDom(
          rect,
          SurfacePaintData()
            ..style = PaintingStyle.stroke
            ..strokeWidth = 10,
        ),
        const Rect.fromLTWH(5, 15, 130, 150),
      );
      expect(
        adjustRectForDom(
          rect,
          SurfacePaintData()
            ..style = PaintingStyle.stroke
            ..strokeWidth = 15,
        ),
        const Rect.fromLTWH(2.5, 12.5, 125, 145),
      );
    });

    test('flips rect when necessary', () {
      Rect rect = const Rect.fromLTWH(100, 200, -40, -60);
      expect(
        adjustRectForDom(rect, SurfacePaintData()..style = PaintingStyle.fill),
        const Rect.fromLTWH(60, 140, 40, 60),
      );

      rect = const Rect.fromLTWH(100, 200, 40, -60);
      expect(
        adjustRectForDom(rect, SurfacePaintData()..style = PaintingStyle.fill),
        const Rect.fromLTWH(100, 140, 40, 60),
      );

      rect = const Rect.fromLTWH(100, 200, -40, 60);
      expect(
        adjustRectForDom(rect, SurfacePaintData()..style = PaintingStyle.fill),
        const Rect.fromLTWH(60, 200, 40, 60),
      );
    });

    test('handles stroke width greater than width or height', () {
      const Rect rect = Rect.fromLTWH(100, 200, 20, 70);
      expect(
        adjustRectForDom(
          rect,
          SurfacePaintData()
            ..style = PaintingStyle.stroke
            ..strokeWidth = 50,
        ),
        const Rect.fromLTWH(75, 175, 0, 20),
      );
      expect(
        adjustRectForDom(
          rect,
          SurfacePaintData()
            ..style = PaintingStyle.stroke
            ..strokeWidth = 80,
        ),
        const Rect.fromLTWH(60, 160, 0, 0),
      );
    });
  });
}
