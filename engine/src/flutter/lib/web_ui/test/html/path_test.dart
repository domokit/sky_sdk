// Copyright 2013 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

import 'dart:js_util' as js_util;
import 'dart:typed_data';
import 'package:test/bootstrap/browser.dart';
import 'package:test/test.dart';
import 'package:ui/src/engine.dart';
import 'package:ui/ui.dart' hide window;
import 'package:ui/ui_web/src/ui_web.dart' as ui_web;

import '../common/matchers.dart';

void main() {
  internalBootstrapBrowserTest(() => testMain);
}

void testMain() {
  group('Path', () {
    test('Should have no subpaths when created', () {
      final SurfacePath path = SurfacePath();
      expect(path.isEmpty, isTrue);
    });

    test('LineTo should add command', () {
      final SurfacePath path = SurfacePath();
      path.moveTo(5.0, 10.0);
      path.lineTo(20.0, 40.0);
      path.lineTo(30.0, 50.0);
      expect(path.pathRef.countPoints(), 3);
      expect(path.pathRef.atPoint(2).dx, 30.0);
      expect(path.pathRef.atPoint(2).dy, 50.0);
    });

    test('LineTo should add moveTo 0,0 when first call to Path API', () {
      final SurfacePath path = SurfacePath();
      path.lineTo(20.0, 40.0);
      expect(path.pathRef.countPoints(), 2);
      expect(path.pathRef.atPoint(0).dx, 0);
      expect(path.pathRef.atPoint(0).dy, 0);
      expect(path.pathRef.atPoint(1).dx, 20.0);
      expect(path.pathRef.atPoint(1).dy, 40.0);
    });

    test('relativeLineTo should increments currentX', () {
      final SurfacePath path = SurfacePath();
      path.moveTo(5.0, 10.0);
      path.lineTo(20.0, 40.0);
      path.relativeLineTo(5.0, 5.0);
      expect(path.pathRef.countPoints(), 3);
      expect(path.pathRef.atPoint(2).dx, 25.0);
      expect(path.pathRef.atPoint(2).dy, 45.0);
    });

    test('Should allow calling relativeLineTo before moveTo', () {
      final SurfacePath path = SurfacePath();
      path.relativeLineTo(5.0, 5.0);
      path.moveTo(5.0, 10.0);
      expect(path.pathRef.countPoints(), 3);
      expect(path.pathRef.atPoint(1).dx, 5.0);
      expect(path.pathRef.atPoint(1).dy, 5.0);
      expect(path.pathRef.atPoint(2).dx, 5.0);
      expect(path.pathRef.atPoint(2).dy, 10.0);
    });

    test('Should allow relativeLineTo after reset', () {
      final SurfacePath path = SurfacePath();
      final Path subPath = Path();
      subPath.moveTo(50.0, 60.0);
      subPath.lineTo(200.0, 200.0);
      path.extendWithPath(subPath, Offset.zero);
      path.reset();
      path.relativeLineTo(5.0, 5.0);
      expect(path.pathRef.countPoints(), 2);
      expect(path.pathRef.atPoint(0).dx, 0);
      expect(path.pathRef.atPoint(0).dy, 0);
      expect(path.pathRef.atPoint(1).dx, 5.0);
    });

    test('Should detect rectangular path', () {
      final SurfacePath path = SurfacePath();
      path.addRect(const Rect.fromLTWH(1.0, 2.0, 3.0, 4.0));
      expect(path.toRect(), const Rect.fromLTWH(1.0, 2.0, 3.0, 4.0));
    });

    test('Should detect horizontal line path', () {
      SurfacePath path = SurfacePath();
      path.moveTo(10, 20);
      path.lineTo(100, 0);
      expect(path.toStraightLine(), null);
      path = SurfacePath();
      path.moveTo(10, 20);
      path.lineTo(200, 20);
      final Rect r = path.toStraightLine()!;
      expect(r, equals(const Rect.fromLTRB(10, 20, 200, 20)));
    });

    test('Should detect vertical line path', () {
      final SurfacePath path = SurfacePath();
      path.moveTo(10, 20);
      path.lineTo(10, 200);
      final Rect r = path.toStraightLine()!;
      expect(r, equals(const Rect.fromLTRB(10, 20, 10, 200)));
    });

    test('Should detect non rectangular path if empty', () {
      final SurfacePath path = SurfacePath();
      expect(path.toRect(), null);
    });

    test('Should detect non rectangular path if there are multiple subpaths', () {
      final SurfacePath path = SurfacePath();
      path.addRect(const Rect.fromLTWH(1.0, 2.0, 3.0, 4.0));
      path.addRect(const Rect.fromLTWH(5.0, 6.0, 7.0, 8.0));
      expect(path.toRect(), null);
    });

    test('Should detect rounded rectangular path', () {
      final SurfacePath path = SurfacePath();
      path.addRRect(
        RRect.fromRectAndRadius(
          const Rect.fromLTRB(1.0, 2.0, 30.0, 40.0),
          const Radius.circular(2.0),
        ),
      );
      expect(
        path.toRoundedRect(),
        RRect.fromRectAndRadius(
          const Rect.fromLTRB(1.0, 2.0, 30.0, 40.0),
          const Radius.circular(2.0),
        ),
      );
    });

    test('Should detect non rounded rectangular path if empty', () {
      final SurfacePath path = SurfacePath();
      expect(path.toRoundedRect(), null);
    });

    test('Should detect rectangular path is not round', () {
      final SurfacePath path = SurfacePath();
      path.addRect(const Rect.fromLTWH(1.0, 2.0, 3.0, 4.0));
      expect(path.toRoundedRect(), null);
    });

    test('Should detect non rounded  rectangular path if there are '
        'multiple subpaths', () {
      final SurfacePath path = SurfacePath();
      path.addRRect(
        RRect.fromRectAndRadius(
          const Rect.fromLTWH(1.0, 2.0, 3.0, 4.0),
          const Radius.circular(2.0),
        ),
      );
      path.addRRect(
        RRect.fromRectAndRadius(
          const Rect.fromLTWH(1.0, 2.0, 3.0, 4.0),
          const Radius.circular(2.0),
        ),
      );
      expect(path.toRoundedRect(), null);
    });

    test('Should compute bounds as empty for empty and moveTo only path', () {
      final Path emptyPath = Path();
      expect(emptyPath.getBounds(), Rect.zero);

      final SurfacePath path = SurfacePath();
      path.moveTo(50, 60);
      expect(path.getBounds(), const Rect.fromLTRB(50, 60, 50, 60));
    });

    test('Should compute bounds for multiple addRect calls', () {
      final Path emptyPath = Path();
      expect(emptyPath.getBounds(), Rect.zero);

      final SurfacePath path = SurfacePath();
      path.addRect(const Rect.fromLTWH(0, 0, 270, 45));
      path.addRect(const Rect.fromLTWH(134.5, 0, 1, 45));
      expect(path.getBounds(), const Rect.fromLTRB(0, 0, 270, 45));
    });

    test('Should compute bounds for addRRect', () {
      SurfacePath path = SurfacePath();
      const Rect bounds = Rect.fromLTRB(30, 40, 400, 300);
      RRect rrect = RRect.fromRectAndCorners(
        bounds,
        topLeft: const Radius.elliptical(1, 2),
        topRight: const Radius.elliptical(3, 4),
        bottomLeft: const Radius.elliptical(5, 6),
        bottomRight: const Radius.elliptical(7, 8),
      );
      path.addRRect(rrect);
      expect(path.getBounds(), bounds);
      expect(path.toRoundedRect(), rrect);
      path = SurfacePath();
      rrect = RRect.fromRectAndCorners(
        bounds,
        topLeft: const Radius.elliptical(0, 2),
        topRight: const Radius.elliptical(3, 4),
        bottomLeft: const Radius.elliptical(5, 6),
        bottomRight: const Radius.elliptical(7, 8),
      );
      path.addRRect(rrect);
      expect(path.getBounds(), bounds);
      expect(path.toRoundedRect(), rrect);
      path = SurfacePath();
      rrect = RRect.fromRectAndCorners(
        bounds,
        topRight: const Radius.elliptical(3, 4),
        bottomLeft: const Radius.elliptical(5, 6),
        bottomRight: const Radius.elliptical(7, 8),
      );
      path.addRRect(rrect);
      expect(path.getBounds(), bounds);
      expect(path.toRoundedRect(), rrect);
      path = SurfacePath();
      rrect = RRect.fromRectAndCorners(
        bounds,
        topLeft: const Radius.elliptical(1, 2),
        bottomLeft: const Radius.elliptical(5, 6),
        bottomRight: const Radius.elliptical(7, 8),
      );
      path.addRRect(rrect);
      expect(path.getBounds(), bounds);
      expect(path.toRoundedRect(), rrect);
      path = SurfacePath();
      rrect = RRect.fromRectAndCorners(
        bounds,
        topLeft: const Radius.elliptical(1, 2),
        topRight: const Radius.elliptical(3, 4),
        bottomRight: const Radius.elliptical(7, 8),
      );
      path.addRRect(rrect);
      expect(path.getBounds(), bounds);
      expect(path.toRoundedRect(), rrect);
      path = SurfacePath();
      rrect = RRect.fromRectAndCorners(
        bounds,
        topLeft: const Radius.elliptical(1, 2),
        topRight: const Radius.elliptical(3, 4),
        bottomLeft: const Radius.elliptical(5, 6),
      );
      path.addRRect(rrect);
      expect(path.getBounds(), bounds);
      expect(path.toRoundedRect(), rrect);
    });

    test('Should compute bounds for lines', () {
      final SurfacePath path = SurfacePath();
      path.moveTo(25, 30);
      path.lineTo(100, 200);
      expect(path.getBounds(), const Rect.fromLTRB(25, 30, 100, 200));

      final SurfacePath path2 = SurfacePath();
      path2.moveTo(250, 300);
      path2.lineTo(50, 60);
      expect(path2.getBounds(), const Rect.fromLTRB(50, 60, 250, 300));
    });

    test('Should compute bounds for polygon', () {
      final SurfacePath path = SurfacePath();
      path.addPolygon(const <Offset>[
        Offset(50, 100),
        Offset(250, 100),
        Offset(152, 180),
        Offset(159, 200),
        Offset(151, 190),
      ], true);
      expect(path.getBounds(), const Rect.fromLTRB(50, 100, 250, 200));
    });

    test('Should compute bounds for quadraticBezierTo', () {
      final SurfacePath path1 = SurfacePath();
      path1.moveTo(285.2, 682.1);
      path1.quadraticBezierTo(432.0, 431.4, 594.9, 681.2);
      expect(
        path1.getBounds(),
        within<Rect>(distance: 0.1, from: const Rect.fromLTRB(285.2, 556.5, 594.9, 682.1)),
      );

      // Control point below start , end.
      final SurfacePath path2 = SurfacePath();
      path2.moveTo(285.2, 682.1);
      path2.quadraticBezierTo(447.4, 946.8, 594.9, 681.2);
      expect(
        path2.getBounds(),
        within<Rect>(distance: 0.1, from: const Rect.fromLTRB(285.2, 681.2, 594.9, 814.2)),
      );

      // Control point to the right of end point.
      final SurfacePath path3 = SurfacePath();
      path3.moveTo(468.3, 685.6);
      path3.quadraticBezierTo(644.7, 555.2, 594.9, 681.2);
      expect(
        path3.getBounds(),
        within<Rect>(distance: 0.1, from: const Rect.fromLTRB(468.3, 619.3, 605.9, 685.6)),
      );
    });

    test('Should compute bounds for cubicTo', () {
      final SurfacePath path1 = SurfacePath();
      path1.moveTo(220, 300);
      path1.cubicTo(230, 120, 400, 125, 410, 280);
      expect(
        path1.getBounds(),
        within<Rect>(distance: 0.1, from: const Rect.fromLTRB(220.0, 164.3, 410.0, 300.0)),
      );

      // control point 1 to the right of control point 2
      final SurfacePath path2 = SurfacePath();
      path2.moveTo(220, 300);
      path2.cubicTo(564.2, 13.7, 400.0, 125.0, 410.0, 280.0);
      expect(
        path2.getBounds(),
        within<Rect>(distance: 0.1, from: const Rect.fromLTRB(220.0, 122.8, 440.5, 300.0)),
      );

      // control point 1 to the right of control point 2 inflection
      final SurfacePath path3 = SurfacePath();
      path3.moveTo(220, 300);
      path3.cubicTo(839.8, 67.9, 400.0, 125.0, 410.0, 280.0);
      expect(
        path3.getBounds(),
        within<Rect>(distance: 0.1, from: const Rect.fromLTRB(220.0, 144.5, 552.1, 300.0)),
      );

      // control point 1 below and between start and end points
      final SurfacePath path4 = SurfacePath();
      path4.moveTo(220.0, 300.0);
      path4.cubicTo(354.8, 388.3, 400.0, 125.0, 410.0, 280.0);
      expect(
        path4.getBounds(),
        within<Rect>(distance: 0.1, from: const Rect.fromLTRB(220.0, 230.0, 410.0, 318.6)),
      );

      // control points inverted below
      final SurfacePath path5 = SurfacePath();
      path5.moveTo(220.0, 300.0);
      path5.cubicTo(366.5, 487.3, 256.4, 489.9, 410.0, 280.0);
      expect(
        path5.getBounds(),
        within<Rect>(distance: 0.1, from: const Rect.fromLTRB(220.0, 280.0, 410.0, 439.0)),
      );

      // control points inverted below wide
      final SurfacePath path6 = SurfacePath();
      path6.moveTo(220.0, 300.0);
      path6.cubicTo(496.1, 485.5, 121.4, 491.6, 410.0, 280.0);
      expect(
        path6.getBounds(),
        within<Rect>(distance: 0.1, from: const Rect.fromLTRB(220.0, 280.0, 410.0, 439.0)),
      );

      // control point 2 and end point swapped
      final SurfacePath path7 = SurfacePath();
      path7.moveTo(220.0, 300.0);
      path7.cubicTo(230.0, 120.0, 394.5, 296.1, 382.3, 124.1);
      expect(
        path7.getBounds(),
        within<Rect>(distance: 0.1, from: const Rect.fromLTRB(220.0, 124.1, 382.9, 300.0)),
      );
    });

    // Regression test for https://github.com/flutter/flutter/issues/46813.
    test('Should deep copy path', () {
      final SurfacePath path = SurfacePath();
      path.moveTo(25, 30);
      path.lineTo(100, 200);
      expect(path.getBounds(), const Rect.fromLTRB(25, 30, 100, 200));

      final SurfacePath path2 = SurfacePath.from(path);
      path2.lineTo(250, 300);
      expect(path2.getBounds(), const Rect.fromLTRB(25, 30, 250, 300));
      // Expect original path to stay the same.
      expect(path.getBounds(), const Rect.fromLTRB(25, 30, 100, 200));
    });

    test('Should handle contains inclusive right,bottom coordinates', () {
      final Path path = Path();
      path.moveTo(50, 60);
      path.lineTo(110, 60);
      path.lineTo(110, 190);
      path.lineTo(50, 190);
      path.close();
      expect(path.contains(const Offset(80, 190)), isTrue);
      expect(path.contains(const Offset(110, 80)), isTrue);
      expect(path.contains(const Offset(110, 190)), isTrue);
      expect(path.contains(const Offset(110, 191)), isFalse);
    });

    test('Should not contain top-left of beveled border', () {
      final Path path = Path();
      path.moveTo(10, 25);
      path.lineTo(15, 20);
      path.lineTo(25, 20);
      path.lineTo(30, 25);
      path.lineTo(30, 35);
      path.lineTo(25, 40);
      path.lineTo(15, 40);
      path.lineTo(10, 35);
      path.close();
      expect(path.contains(const Offset(10, 20)), isFalse);
    });

    test('Computes contains for cubic curves', () {
      final Path path = Path();
      path.moveTo(10, 25);
      path.cubicTo(10, 20, 10, 20, 20, 15);
      path.lineTo(25, 20);
      path.cubicTo(30, 20, 30, 20, 30, 25);
      path.lineTo(30, 35);
      path.cubicTo(30, 40, 30, 40, 25, 40);
      path.lineTo(15, 40);
      path.cubicTo(10, 40, 10, 40, 10, 35);
      path.close();
      expect(path.contains(const Offset(10, 20)), isFalse);
      expect(path.contains(const Offset(30, 40)), isFalse);
    });

    // Regression test for https://github.com/flutter/flutter/issues/44470
    test(
      'Should handle contains for devicepixelratio != 1.0',
      () {
        js_util.setProperty(domWindow, 'devicePixelRatio', 4.0);
        EngineFlutterDisplay.instance.debugOverrideDevicePixelRatio(4.0);
        final Path path =
            Path()
              ..moveTo(50, 0)
              ..lineTo(100, 100)
              ..lineTo(0, 100)
              ..lineTo(50, 0)
              ..close();
        expect(path.contains(const Offset(50, 50)), isTrue);
        js_util.setProperty(domWindow, 'devicePixelRatio', 1.0);
        EngineFlutterDisplay.instance.debugOverrideDevicePixelRatio(1.0);
        // TODO(ferhat): Investigate failure on CI. Locally this passes.
        // [Exception... "Failure"  nsresult: "0x80004005 (NS_ERROR_FAILURE)"
      },
      skip: ui_web.browser.browserEngine == ui_web.BrowserEngine.firefox,
    );

    // Path contains should handle case where invalid RRect with large
    // corner radius is used for hit test. Use case is a RenderPhysicalShape
    // with a clipper that contains RRect of width/height 50 but corner radius
    // of 100.
    //
    // Regression test for https://github.com/flutter/flutter/issues/48887
    test('Should hit test correctly for malformed rrect', () {
      // Correctly formed rrect.
      final Path path1 =
          Path()..addRRect(RRect.fromLTRBR(50, 50, 100, 100, const Radius.circular(20)));
      expect(path1.contains(const Offset(75, 75)), isTrue);
      expect(path1.contains(const Offset(52, 75)), isTrue);
      expect(path1.contains(const Offset(50, 50)), isFalse);
      expect(path1.contains(const Offset(100, 50)), isFalse);
      expect(path1.contains(const Offset(100, 100)), isFalse);
      expect(path1.contains(const Offset(50, 100)), isFalse);

      final Path path2 =
          Path()..addRRect(RRect.fromLTRBR(50, 50, 100, 100, const Radius.circular(100)));
      expect(path2.contains(const Offset(75, 75)), isTrue);
      expect(path2.contains(const Offset(52, 75)), isTrue);
      expect(path2.contains(const Offset(50, 50)), isFalse);
      expect(path2.contains(const Offset(100, 50)), isFalse);
      expect(path2.contains(const Offset(100, 100)), isFalse);
      expect(path2.contains(const Offset(50, 100)), isFalse);
    });

    test('Should set segment masks', () {
      final SurfacePath path = SurfacePath();
      path.pathRef.computeSegmentMask();
      expect(path.pathRef.segmentMasks, 0);
      path.moveTo(20, 40);
      path.pathRef.computeSegmentMask();
      expect(path.pathRef.segmentMasks, 0);
      path.lineTo(200, 40);
      path.pathRef.computeSegmentMask();
      expect(path.pathRef.segmentMasks, SPathSegmentMask.kLine_SkPathSegmentMask);
    });

    test('Should convert conic to quad when approximation error is small', () {
      final Conic conic = Conic(
        120.0,
        20.0,
        160.99470420829266,
        20.0,
        190.19301120261332,
        34.38770865870253,
        0.9252691032413082,
      );
      expect(conic.toQuads().length, 3);
    });

    test('Should be able to construct from empty path', () {
      final SurfacePath path = SurfacePath();
      expect(path.isEmpty, isTrue);
      final SurfacePath path2 = SurfacePath.from(path);
      expect(path2.isEmpty, isTrue);
    });
  });

  group('PathRef', () {
    test('Should return empty when created', () {
      final PathRef pathRef = PathRef();
      expect(pathRef.isEmpty, isTrue);
    });

    test('Should return non-empty when mutated', () {
      final PathRef pathRef = PathRef();
      pathRef.growForVerb(SPath.kMoveVerb, 0);
      expect(pathRef.isEmpty, isFalse);
    });
  });
  group('PathRefIterator', () {
    test('Should iterate through empty path', () {
      final Float32List points = Float32List(20);
      final PathRef pathRef = PathRef();
      final PathRefIterator iter = PathRefIterator(pathRef);
      expect(iter.next(points), SPath.kDoneVerb);
    });

    test('Should iterate through verbs', () {
      final Float32List points = Float32List(20);
      final PathRef pathRef = PathRef();
      pathRef.growForVerb(SPath.kMoveVerb, 0);
      pathRef.growForVerb(SPath.kLineVerb, 0);
      pathRef.growForVerb(SPath.kQuadVerb, 0);
      pathRef.growForVerb(SPath.kCubicVerb, 0);
      pathRef.growForVerb(SPath.kConicVerb, 0.8);
      pathRef.growForVerb(SPath.kLineVerb, 0.8);
      final PathRefIterator iter = PathRefIterator(pathRef);
      expect(iter.next(points), SPath.kMoveVerb);
      expect(iter.next(points), SPath.kLineVerb);
      expect(iter.next(points), SPath.kQuadVerb);
      expect(iter.next(points), SPath.kCubicVerb);
      expect(iter.next(points), SPath.kConicVerb);
      expect(iter.next(points), SPath.kLineVerb);
      expect(iter.next(points), SPath.kDoneVerb);
    });

    test('Should iterate by index through empty path', () {
      final PathRef pathRef = PathRef();
      final PathRefIterator iter = PathRefIterator(pathRef);
      expect(iter.nextIndex(), SPath.kDoneVerb);
    });

    test('Should iterate through contours', () {
      final PathRef pathRef = PathRef();
      pathRef.growForVerb(SPath.kMoveVerb, 0);
      pathRef.growForVerb(SPath.kLineVerb, 0);
      pathRef.growForVerb(SPath.kQuadVerb, 0);
      pathRef.growForVerb(SPath.kCubicVerb, 0);
      pathRef.growForVerb(SPath.kMoveVerb, 0);
      pathRef.growForVerb(SPath.kConicVerb, 0.8);
      pathRef.growForVerb(SPath.kLineVerb, 0.8);
      pathRef.growForVerb(SPath.kCloseVerb, 0.8);
      pathRef.growForVerb(SPath.kMoveVerb, 0);
      pathRef.growForVerb(SPath.kLineVerb, 0);
      pathRef.growForVerb(SPath.kLineVerb, 0);
      final PathRefIterator iter = PathRefIterator(pathRef);
      int start = iter.pointIndex;
      int end = iter.skipToNextContour();
      expect(end - start, 7);

      start = end;
      end = iter.skipToNextContour();
      expect(end - start, 4);

      start = end;
      end = iter.skipToNextContour();
      expect(end - start, 3);

      start = end;
      end = iter.skipToNextContour();
      expect(start, end);
    });

    /// Regression test for https://github.com/flutter/flutter/issues/68702.
    test('Path should return correct bounds after transform', () {
      final Path path1 =
          Path()
            ..moveTo(100, 100)
            ..lineTo(200, 100)
            ..lineTo(150, 200)
            ..close();
      final SurfacePath path2 = Path.from(path1) as SurfacePath;
      final Rect bounds = path2.pathRef.getBounds();
      final SurfacePath transformedPath = path2.transform(
        Matrix4.identity().scaled(0.5, 0.5).toFloat64(),
      );
      expect(transformedPath.pathRef.getBounds(), isNot(bounds));
    });
  });
}
