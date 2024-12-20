// Copyright 2014 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

import 'package:flutter/painting.dart';
import 'package:flutter_test/flutter_test.dart';

void main() {
  test('BeveledRectangleBorder defaults', () {
    const BeveledRectangleBorder border = BeveledRectangleBorder();
    expect(border.side, BorderSide.none);
    expect(border.borderRadius, BorderRadius.zero);
  });

  test('BeveledRectangleBorder copyWith, ==, hashCode', () {
    expect(const BeveledRectangleBorder(), const BeveledRectangleBorder().copyWith());
    expect(
      const BeveledRectangleBorder().hashCode,
      const BeveledRectangleBorder().copyWith().hashCode,
    );
    const BorderSide side = BorderSide(width: 10.0, color: Color(0xff123456));
    const BorderRadius radius = BorderRadius.all(Radius.circular(16.0));
    const BorderRadiusDirectional directionalRadius = BorderRadiusDirectional.all(
      Radius.circular(16.0),
    );
    expect(
      const BeveledRectangleBorder().copyWith(side: side, borderRadius: radius),
      const BeveledRectangleBorder(side: side, borderRadius: radius),
    );

    expect(
      const BeveledRectangleBorder().copyWith(side: side, borderRadius: directionalRadius),
      const BeveledRectangleBorder(side: side, borderRadius: directionalRadius),
    );
  });

  test('BeveledRectangleBorder scale and lerp', () {
    const BeveledRectangleBorder c10 = BeveledRectangleBorder(
      side: BorderSide(width: 10.0),
      borderRadius: BorderRadius.all(Radius.circular(100.0)),
    );
    const BeveledRectangleBorder c15 = BeveledRectangleBorder(
      side: BorderSide(width: 15.0),
      borderRadius: BorderRadius.all(Radius.circular(150.0)),
    );
    const BeveledRectangleBorder c20 = BeveledRectangleBorder(
      side: BorderSide(width: 20.0),
      borderRadius: BorderRadius.all(Radius.circular(200.0)),
    );
    expect(c10.dimensions, const EdgeInsets.all(10.0));
    expect(c10.scale(2.0), c20);
    expect(c20.scale(0.5), c10);
    expect(ShapeBorder.lerp(c10, c20, 0.0), c10);
    expect(ShapeBorder.lerp(c10, c20, 0.5), c15);
    expect(ShapeBorder.lerp(c10, c20, 1.0), c20);
  });

  test('BeveledRectangleBorder BorderRadius.zero', () {
    const Rect rect1 = Rect.fromLTRB(10.0, 20.0, 30.0, 40.0);
    final Matcher looksLikeRect1 = isPathThat(
      includes: const <Offset>[Offset(10.0, 20.0), Offset(20.0, 30.0)],
      excludes: const <Offset>[Offset(9.0, 19.0), Offset(31.0, 41.0)],
    );

    // Default border radius and border side are zero, i.e. just a rectangle.
    expect(const BeveledRectangleBorder().getOuterPath(rect1), looksLikeRect1);
    expect(const BeveledRectangleBorder().getInnerPath(rect1), looksLikeRect1);

    // Represents the inner path when borderSide.width = 4, which is just rect1
    // inset by 4 on all sides.
    final Matcher looksLikeInnerPath = isPathThat(
      includes: const <Offset>[Offset(14.0, 24.0), Offset(16.0, 26.0)],
      excludes: const <Offset>[Offset(9.0, 23.0), Offset(27.0, 37.0)],
    );

    const BorderSide side = BorderSide(width: 4.0);
    expect(const BeveledRectangleBorder(side: side).getOuterPath(rect1), looksLikeRect1);
    expect(const BeveledRectangleBorder(side: side).getInnerPath(rect1), looksLikeInnerPath);
  });

  test('BeveledRectangleBorder non-zero BorderRadius', () {
    const Rect rect = Rect.fromLTRB(10.0, 20.0, 30.0, 40.0);
    final Matcher looksLikeRect = isPathThat(
      includes: const <Offset>[Offset(15.0, 25.0), Offset(20.0, 30.0)],
      excludes: const <Offset>[Offset(10.0, 20.0), Offset(30.0, 40.0)],
    );
    const BeveledRectangleBorder border = BeveledRectangleBorder(
      borderRadius: BorderRadius.all(Radius.circular(5.0)),
    );
    expect(border.getOuterPath(rect), looksLikeRect);
    expect(border.getInnerPath(rect), looksLikeRect);
  });

  test('BeveledRectangleBorder non-zero BorderRadiusDirectional', () {
    const Rect rect = Rect.fromLTRB(10.0, 20.0, 30.0, 40.0);
    final Matcher looksLikeRectLtr = isPathThat(
      includes: const <Offset>[Offset(15.0, 25.0), Offset(20.0, 30.0)],
      excludes: const <Offset>[Offset(10.0, 20.0), Offset(10.0, 40.0)],
    );
    const BeveledRectangleBorder border = BeveledRectangleBorder(
      borderRadius: BorderRadiusDirectional.only(
        topStart: Radius.circular(5.0),
        bottomStart: Radius.circular(5.0),
      ),
    );

    // Test ltr situation
    expect(border.getOuterPath(rect, textDirection: TextDirection.ltr), looksLikeRectLtr);
    expect(border.getInnerPath(rect, textDirection: TextDirection.ltr), looksLikeRectLtr);

    final Matcher looksLikeRectRtl = isPathThat(
      includes: const <Offset>[Offset(25.0, 35.0), Offset(25.0, 25.0)],
      excludes: const <Offset>[Offset(30.0, 20.0), Offset(30.0, 40.0)],
    );

    // Test Rtl situation
    expect(border.getOuterPath(rect, textDirection: TextDirection.rtl), looksLikeRectRtl);
    expect(border.getInnerPath(rect, textDirection: TextDirection.rtl), looksLikeRectRtl);
  });

  test('BeveledRectangleBorder with StrokeAlign', () {
    const BorderRadius borderRadius = BorderRadius.all(Radius.circular(10));
    const BeveledRectangleBorder inside = BeveledRectangleBorder(
      side: BorderSide(width: 10.0),
      borderRadius: borderRadius,
    );
    const BeveledRectangleBorder center = BeveledRectangleBorder(
      side: BorderSide(width: 10.0, strokeAlign: BorderSide.strokeAlignCenter),
      borderRadius: borderRadius,
    );
    const BeveledRectangleBorder outside = BeveledRectangleBorder(
      side: BorderSide(width: 10.0, strokeAlign: BorderSide.strokeAlignOutside),
      borderRadius: borderRadius,
    );
    expect(inside.dimensions, const EdgeInsets.all(10.0));
    expect(center.dimensions, const EdgeInsets.all(5.0));
    expect(outside.dimensions, EdgeInsets.zero);

    const Rect rect = Rect.fromLTWH(0.0, 0.0, 120.0, 40.0);

    expect(
      inside.getInnerPath(rect),
      isPathThat(
        includes: const <Offset>[Offset(10, 20), Offset(100, 10), Offset(50, 30), Offset(50, 20)],
        excludes: const <Offset>[Offset(9, 9), Offset(100, 0), Offset(110, 31), Offset(9, 31)],
      ),
    );
    expect(
      center.getInnerPath(rect),
      isPathThat(
        includes: const <Offset>[Offset(9, 9), Offset(100, 10), Offset(110, 31), Offset(9, 31)],
        excludes: const <Offset>[Offset(4, 4), Offset(100, 0), Offset(116, 31), Offset(4, 31)],
      ),
    );
    expect(
      outside.getInnerPath(rect),
      isPathThat(
        includes: const <Offset>[Offset(5, 5), Offset(110, 0), Offset(116, 31), Offset(4, 31)],
        excludes: const <Offset>[Offset.zero, Offset(120, 0), Offset(120, 31), Offset(0, 31)],
      ),
    );
  });
}
