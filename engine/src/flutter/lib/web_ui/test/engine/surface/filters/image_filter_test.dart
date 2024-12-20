// Copyright 2013 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

import 'dart:typed_data';

import 'package:test/bootstrap/browser.dart';
import 'package:test/test.dart';
import 'package:ui/src/engine.dart';
import 'package:ui/ui.dart';

void main() {
  internalBootstrapBrowserTest(() => testMain);
}

void testMain() {
  group('ImageFilter constructors', () {
    test('matrix is copied', () {
      final Matrix4 matrix = Matrix4.identity();
      final Float64List storage = matrix.toFloat64();
      final ImageFilter filter1 = ImageFilter.matrix(storage);
      storage[0] = 2.0;
      final ImageFilter filter2 = ImageFilter.matrix(storage);
      expect(filter1, filter1);
      expect(filter2, filter2);
      expect(filter1, isNot(equals(filter2)));
      expect(filter2, isNot(equals(filter1)));
    });

    test('matrix tests all values on ==', () {
      final Matrix4 matrix = Matrix4.identity();
      final Float64List storage = matrix.toFloat64();
      final ImageFilter filter1a = ImageFilter.matrix(storage, filterQuality: FilterQuality.none);
      final ImageFilter filter1b = ImageFilter.matrix(storage, filterQuality: FilterQuality.high);

      storage[0] = 2.0;
      final ImageFilter filter2a = ImageFilter.matrix(storage, filterQuality: FilterQuality.none);
      final ImageFilter filter2b = ImageFilter.matrix(storage, filterQuality: FilterQuality.high);

      expect(filter1a, filter1a);
      expect(filter1a, isNot(equals(filter1b)));
      expect(filter1a, isNot(equals(filter2a)));
      expect(filter1a, isNot(equals(filter2b)));

      expect(filter1b, isNot(equals(filter1a)));
      expect(filter1b, filter1b);
      expect(filter1b, isNot(equals(filter2a)));
      expect(filter1b, isNot(equals(filter2b)));

      expect(filter2a, isNot(equals(filter1a)));
      expect(filter2a, isNot(equals(filter1b)));
      expect(filter2a, filter2a);
      expect(filter2a, isNot(equals(filter2b)));

      expect(filter2b, isNot(equals(filter1a)));
      expect(filter2b, isNot(equals(filter1b)));
      expect(filter2b, isNot(equals(filter2a)));
      expect(filter2b, filter2b);
    });

    test('blur tests all values on ==', () {
      final ImageFilter filter1 = ImageFilter.blur(
        sigmaX: 2.0,
        sigmaY: 2.0,
        tileMode: TileMode.decal,
      );
      final ImageFilter filter2 = ImageFilter.blur(
        sigmaX: 2.0,
        sigmaY: 3.0,
        tileMode: TileMode.decal,
      );
      final ImageFilter filter3 = ImageFilter.blur(
        sigmaX: 2.0,
        sigmaY: 2.0,
        tileMode: TileMode.mirror,
      );

      expect(filter1, filter1);
      expect(filter1, isNot(equals(filter2)));
      expect(filter1, isNot(equals(filter3)));

      expect(filter2, isNot(equals(filter1)));
      expect(filter2, filter2);
      expect(filter2, isNot(equals(filter3)));

      expect(filter3, isNot(equals(filter1)));
      expect(filter3, isNot(equals(filter2)));
      expect(filter3, filter3);
    });
  });
}
