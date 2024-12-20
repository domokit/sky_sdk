// Copyright 2013 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

import 'package:test/bootstrap/browser.dart';
import 'package:test/test.dart';

import 'package:ui/src/engine.dart';
import 'package:ui/ui.dart' as ui;

import 'common.dart';

void main() {
  internalBootstrapBrowserTest(() => testMain);
}

void testMain() {
  group('CkPath', () {
    setUpCanvasKitTest();

    test('Using CanvasKit', () {
      expect(renderer is CanvasKitRenderer, isTrue);
    });

    test(CkPathMetrics, () {
      final ui.Path path = ui.Path();
      expect(path, isA<CkPath>());
      expect(path.computeMetrics().length, 0);

      path.addRect(const ui.Rect.fromLTRB(0, 0, 10, 10));
      final ui.PathMetric metric = path.computeMetrics().single;
      expect(metric.contourIndex, 0);
      expect(metric.extractPath(0, 0.5).computeMetrics().length, 1);

      final ui.Tangent tangent1 = metric.getTangentForOffset(5)!;
      expect(tangent1.position, const ui.Offset(5, 0));
      expect(tangent1.vector, const ui.Offset(1, 0));

      final ui.Tangent tangent2 = metric.getTangentForOffset(15)!;
      expect(tangent2.position, const ui.Offset(10, 5));
      expect(tangent2.vector, const ui.Offset(0, 1));

      expect(metric.isClosed, isTrue);

      path.addOval(const ui.Rect.fromLTRB(10, 10, 100, 100));
      expect(path.computeMetrics().length, 2);

      // Can simultaneously iterate over multiple metrics from the same path.
      final ui.PathMetrics metrics1 = path.computeMetrics();
      final ui.PathMetrics metrics2 = path.computeMetrics();
      final Iterator<ui.PathMetric> iter1 = metrics1.iterator;
      final Iterator<ui.PathMetric> iter2 = metrics2.iterator;
      expect(iter1.moveNext(), isTrue);
      expect(iter2.moveNext(), isTrue);
      expect(iter1.current, isNotNull);
      expect(iter2.current, isNotNull);
      expect(iter1.moveNext(), isTrue);
      expect(iter2.moveNext(), isTrue);
      expect(iter1.current, isNotNull);
      expect(iter2.current, isNotNull);
      expect(iter1.moveNext(), isFalse);
      expect(iter2.moveNext(), isFalse);
      expect(() => iter1.current, throwsRangeError);
      expect(() => iter2.current, throwsRangeError);
    });

    test('CkPath.reset', () {
      final ui.Path path = ui.Path();
      expect(path, isA<CkPath>());
      path.addRect(const ui.Rect.fromLTRB(0, 0, 10, 10));
      expect(path.contains(const ui.Offset(5, 5)), isTrue);

      expect(path.fillType, ui.PathFillType.nonZero);
      path.fillType = ui.PathFillType.evenOdd;
      expect(path.fillType, ui.PathFillType.evenOdd);

      path.reset();
      expect(path.fillType, ui.PathFillType.nonZero);
      expect(path.contains(const ui.Offset(5, 5)), isFalse);
    });

    test('CkPath.shift creates a shifted copy of the path', () {
      const ui.Rect testRect = ui.Rect.fromLTRB(0, 0, 10, 10);
      final CkPath path = CkPath();
      path.addRect(testRect);
      expect(path.getBounds(), testRect);

      expect(
        path.shift(const ui.Offset(20, 20)).getBounds(),
        testRect.shift(const ui.Offset(20, 20)),
      );

      // Make sure the original path wasn't mutated.
      expect(path.getBounds(), testRect);
    });

    test('CkContourMeasure iteration', () {
      final ui.Path path = ui.Path();
      expect(path, isA<CkPath>());
      path.addRect(const ui.Rect.fromLTRB(0, 0, 10, 10));
      path.addRect(const ui.Rect.fromLTRB(20, 20, 30, 30));
      path.addRect(const ui.Rect.fromLTRB(40, 40, 50, 50));

      final ui.PathMetrics metrics = path.computeMetrics();
      final CkContourMeasureIter iterator = metrics.iterator as CkContourMeasureIter;

      expect(iterator.moveNext(), isTrue);
      expect(iterator.current.contourIndex, 0);
      expect(iterator.moveNext(), isTrue);
      expect(iterator.current.contourIndex, 1);
    });

    test('CkContourMeasure index', () {
      final ui.Path path = ui.Path();
      expect(path, isA<CkPath>());
      path.addRect(const ui.Rect.fromLTRB(0, 0, 10, 10));
      path.addRect(const ui.Rect.fromLTRB(20, 20, 30, 30));
      path.addRect(const ui.Rect.fromLTRB(40, 40, 50, 50));

      final ui.PathMetrics metrics = path.computeMetrics();
      final CkContourMeasureIter iterator = metrics.iterator as CkContourMeasureIter;

      expect(iterator.moveNext(), isTrue);
      final CkContourMeasure measure0 = iterator.current as CkContourMeasure;
      expect(measure0.contourIndex, 0);
      expect(measure0.extractPath(0, 15).getBounds(), const ui.Rect.fromLTRB(0, 0, 10, 5));

      expect(iterator.moveNext(), isTrue);
      final CkContourMeasure measure1 = iterator.current as CkContourMeasure;
      expect(measure1.contourIndex, 1);
      expect(measure1.extractPath(0, 15).getBounds(), const ui.Rect.fromLTRB(20, 20, 30, 25));
    });

    test('Path.from', () {
      const ui.Rect rect1 = ui.Rect.fromLTRB(0, 0, 10, 10);
      const ui.Rect rect2 = ui.Rect.fromLTRB(10, 10, 20, 20);

      final ui.Path original = ui.Path();
      original.addRect(rect1);
      expect(original, isA<CkPath>());
      expect(original.getBounds(), rect1);

      final ui.Path copy = ui.Path.from(original);
      expect(copy, isA<CkPath>());
      expect(copy.getBounds(), rect1);

      // Test that when copy is mutated, the original is not affected
      copy.addRect(rect2);
      expect(original.getBounds(), rect1);
      expect(copy.getBounds(), rect1.expandToInclude(rect2));
    });
  });
}
