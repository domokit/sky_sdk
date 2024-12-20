// Copyright 2013 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

import 'package:test/bootstrap/browser.dart';
import 'package:test/test.dart';
import 'package:ui/src/engine/html/shaders/normalized_gradient.dart';
import 'package:ui/ui.dart' as ui hide window;

void main() {
  internalBootstrapBrowserTest(() => testMain);
}

void testMain() {
  group('Shader Normalized Gradient', () {
    test('3 stop at start', () {
      final NormalizedGradient gradient = NormalizedGradient(
        const <ui.Color>[ui.Color(0xFF000000), ui.Color(0xFFFF7f3f)],
        stops: <double>[0.0, 0.5],
      );
      int res = _computeColorAt(gradient, 0.0);
      assert(res == 0xFF000000);
      res = _computeColorAt(gradient, 0.25);
      assert(res == 0xFF7f3f1f);
      res = _computeColorAt(gradient, 0.5);
      assert(res == 0xFFFF7f3f);
      res = _computeColorAt(gradient, 0.7);
      assert(res == 0xFFFF7f3f);
      res = _computeColorAt(gradient, 1.0);
      assert(res == 0xFFFF7f3f);
    });

    test('3 stop at end', () {
      final NormalizedGradient gradient = NormalizedGradient(
        const <ui.Color>[ui.Color(0xFF000000), ui.Color(0xFFFF7f3f)],
        stops: <double>[0.5, 1.0],
      );
      int res = _computeColorAt(gradient, 0.0);
      assert(res == 0xFF000000);
      res = _computeColorAt(gradient, 0.25);
      assert(res == 0xFF000000);
      res = _computeColorAt(gradient, 0.5);
      assert(res == 0xFF000000);
      res = _computeColorAt(gradient, 0.75);
      assert(res == 0xFF7f3f1f);
      res = _computeColorAt(gradient, 1.0);
      assert(res == 0xFFFF7f3f);
    });

    test('4 stop', () {
      final NormalizedGradient gradient = NormalizedGradient(
        const <ui.Color>[ui.Color(0xFF000000), ui.Color(0xFFFF7f3f)],
        stops: <double>[0.25, 0.5],
      );
      int res = _computeColorAt(gradient, 0.0);
      assert(res == 0xFF000000);
      res = _computeColorAt(gradient, 0.25);
      assert(res == 0xFF000000);
      res = _computeColorAt(gradient, 0.4);
      assert(res == 0xFF994c25);
      res = _computeColorAt(gradient, 0.5);
      assert(res == 0xFFFF7f3f);
      res = _computeColorAt(gradient, 0.75);
      assert(res == 0xFFFF7f3f);
      res = _computeColorAt(gradient, 1.0);
      assert(res == 0xFFFF7f3f);
    });

    test('5 stop', () {
      final NormalizedGradient gradient = NormalizedGradient(
        const <ui.Color>[
          ui.Color(0x10000000),
          ui.Color(0x20FF0000),
          ui.Color(0x4000FF00),
          ui.Color(0x800000FF),
          ui.Color(0xFFFFFFFF),
        ],
        stops: <double>[0.0, 0.1, 0.2, 0.5, 1.0],
      );
      int res = _computeColorAt(gradient, 0.0);
      assert(res == 0x10000000);
      res = _computeColorAt(gradient, 0.05);
      assert(res == 0x187f0000);
      res = _computeColorAt(gradient, 0.1);
      assert(res == 0x20ff0000);
      res = _computeColorAt(gradient, 0.15);
      assert(res == 0x307f7f00);
      res = _computeColorAt(gradient, 0.2);
      assert(res == 0x4000ff00);
      res = _computeColorAt(gradient, 0.4);
      assert(res == 0x6a0054a9);
      res = _computeColorAt(gradient, 0.5);
      assert(res == 0x800000fe);
      res = _computeColorAt(gradient, 0.9);
      assert(res == 0xe5ccccff);
      res = _computeColorAt(gradient, 1.0);
      assert(res == 0xffffffff);
    });

    test('2 stops at ends', () {
      final NormalizedGradient gradient = NormalizedGradient(const <ui.Color>[
        ui.Color(0x00000000),
        ui.Color(0xFFFFFFFF),
      ]);
      int res = _computeColorAt(gradient, 0.0);
      assert(res == 0);
      res = _computeColorAt(gradient, 1.0);
      assert(res == 0xFFFFFFFF);
      res = _computeColorAt(gradient, 0.5);
      assert(res == 0x7f7f7f7f);
    });
  });
}

int _computeColorAt(NormalizedGradient gradient, double t) {
  int i = 0;
  while (t > gradient.thresholdAt(i + 1)) {
    ++i;
  }
  final double r = t * gradient.scaleAt(i * 4) + gradient.biasAt(i * 4);
  final double g = t * gradient.scaleAt(i * 4 + 1) + gradient.biasAt(i * 4 + 1);
  final double b = t * gradient.scaleAt(i * 4 + 2) + gradient.biasAt(i * 4 + 2);
  final double a = t * gradient.scaleAt(i * 4 + 3) + gradient.biasAt(i * 4 + 3);
  int val = 0;
  val |= (a * 0xFF).toInt() & 0xFF;
  val <<= 8;
  val |= (r * 0xFF).toInt() & 0xFF;
  val <<= 8;
  val |= (g * 0xFF).toInt() & 0xFF;
  val <<= 8;
  val |= (b * 0xFF).toInt() & 0xFF;
  return val;
}
