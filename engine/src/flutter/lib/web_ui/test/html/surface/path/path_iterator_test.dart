// Copyright 2013 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

import 'dart:typed_data';

import 'package:test/bootstrap/browser.dart';
import 'package:test/test.dart';
import 'package:ui/src/engine.dart';

void main() {
  internalBootstrapBrowserTest(() => testMain);
}

void testMain() {
  final Float32List points = Float32List(PathIterator.kMaxBufferSize);

  group('PathIterator', () {
    test('Should return done verb for empty path', () {
      final SurfacePath path = SurfacePath();
      final PathIterator iter = PathIterator(path.pathRef, false);
      expect(iter.peek(), SPath.kDoneVerb);
      expect(iter.next(points), SPath.kDoneVerb);
    });

    test('Should return done when moveTo is last instruction', () {
      final SurfacePath path = SurfacePath();
      path.moveTo(10, 10);
      final PathIterator iter = PathIterator(path.pathRef, false);
      expect(iter.peek(), SPath.kMoveVerb);
      expect(iter.next(points), SPath.kDoneVerb);
    });

    test('Should return lineTo', () {
      final SurfacePath path = SurfacePath();
      path.moveTo(10, 10);
      path.lineTo(20, 20);
      final PathIterator iter = PathIterator(path.pathRef, false);
      expect(iter.peek(), SPath.kMoveVerb);
      expect(iter.next(points), SPath.kMoveVerb);
      expect(points[0], 10);
      expect(iter.next(points), SPath.kLineVerb);
      expect(points[2], 20);
      expect(iter.next(points), SPath.kDoneVerb);
    });

    test('Should return extra lineTo if iteration is closed', () {
      final SurfacePath path = SurfacePath();
      path.moveTo(10, 10);
      path.lineTo(20, 20);
      final PathIterator iter = PathIterator(path.pathRef, true);
      expect(iter.peek(), SPath.kMoveVerb);
      expect(iter.next(points), SPath.kMoveVerb);
      expect(points[0], 10);
      expect(iter.next(points), SPath.kLineVerb);
      expect(points[2], 20);
      expect(iter.next(points), SPath.kLineVerb);
      expect(points[2], 10);
      expect(iter.next(points), SPath.kCloseVerb);
      expect(iter.next(points), SPath.kDoneVerb);
    });

    test('Should not return extra lineTo if last point is starting point', () {
      final SurfacePath path = SurfacePath();
      path.moveTo(10, 10);
      path.lineTo(20, 20);
      path.lineTo(10, 10);
      final PathIterator iter = PathIterator(path.pathRef, true);
      expect(iter.peek(), SPath.kMoveVerb);
      expect(iter.next(points), SPath.kMoveVerb);
      expect(points[0], 10);
      expect(iter.next(points), SPath.kLineVerb);
      expect(points[2], 20);
      expect(iter.next(points), SPath.kLineVerb);
      expect(points[2], 10);
      expect(iter.next(points), SPath.kCloseVerb);
      expect(iter.next(points), SPath.kDoneVerb);
    });

    test('peek should return lineTo if iteration is closed', () {
      final SurfacePath path = SurfacePath();
      path.moveTo(10, 10);
      path.lineTo(20, 20);
      final PathIterator iter = PathIterator(path.pathRef, true);
      expect(iter.next(points), SPath.kMoveVerb);
      expect(iter.next(points), SPath.kLineVerb);
      expect(iter.peek(), SPath.kLineVerb);
    });
  });
}
