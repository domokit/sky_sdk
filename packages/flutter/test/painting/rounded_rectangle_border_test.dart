// Copyright 2014 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

import 'package:flutter/painting.dart';
import 'package:flutter_test/flutter_test.dart';

import 'common_matchers.dart';

void main() {
  test('RoundedRectangleBorder defaults', () {
    const RoundedRectangleBorder border = RoundedRectangleBorder();
    expect(border.side, BorderSide.none);
    expect(border.borderRadius, BorderRadius.zero);
  });

  test('RoundedRectangleBorder copyWith, ==, hashCode', () {
    expect(const RoundedRectangleBorder(), const RoundedRectangleBorder().copyWith());
    expect(
      const RoundedRectangleBorder().hashCode,
      const RoundedRectangleBorder().copyWith().hashCode,
    );
    const BorderSide side = BorderSide(width: 10.0, color: Color(0xff123456));
    const BorderRadius radius = BorderRadius.all(Radius.circular(16.0));
    const BorderRadiusDirectional directionalRadius = BorderRadiusDirectional.all(
      Radius.circular(16.0),
    );

    expect(
      const RoundedRectangleBorder().copyWith(side: side, borderRadius: radius),
      const RoundedRectangleBorder(side: side, borderRadius: radius),
    );

    expect(
      const RoundedRectangleBorder().copyWith(side: side, borderRadius: directionalRadius),
      const RoundedRectangleBorder(side: side, borderRadius: directionalRadius),
    );
  });

  test('RoundedRectangleBorder', () {
    const RoundedRectangleBorder c10 = RoundedRectangleBorder(
      side: BorderSide(width: 10.0),
      borderRadius: BorderRadius.all(Radius.circular(100.0)),
    );
    const RoundedRectangleBorder c15 = RoundedRectangleBorder(
      side: BorderSide(width: 15.0),
      borderRadius: BorderRadius.all(Radius.circular(150.0)),
    );
    const RoundedRectangleBorder c20 = RoundedRectangleBorder(
      side: BorderSide(width: 20.0),
      borderRadius: BorderRadius.all(Radius.circular(200.0)),
    );
    expect(c10.dimensions, const EdgeInsets.all(10.0));
    expect(c10.scale(2.0), c20);
    expect(c20.scale(0.5), c10);
    expect(ShapeBorder.lerp(c10, c20, 0.0), c10);
    expect(ShapeBorder.lerp(c10, c20, 0.5), c15);
    expect(ShapeBorder.lerp(c10, c20, 1.0), c20);

    const RoundedRectangleBorder c1 = RoundedRectangleBorder(
      side: BorderSide(),
      borderRadius: BorderRadius.all(Radius.circular(1.0)),
    );
    const RoundedRectangleBorder c2 = RoundedRectangleBorder(
      side: BorderSide(),
      borderRadius: BorderRadius.all(Radius.circular(2.0)),
    );
    expect(c2.getInnerPath(Rect.fromCircle(center: Offset.zero, radius: 2.0)), isUnitCircle);
    expect(c1.getOuterPath(Rect.fromCircle(center: Offset.zero, radius: 1.0)), isUnitCircle);
    const Rect rect = Rect.fromLTRB(10.0, 20.0, 80.0, 190.0);
    expect(
      (Canvas canvas) => c10.paint(canvas, rect),
      paints..drrect(
        outer: RRect.fromRectAndRadius(rect, const Radius.circular(100.0)),
        inner: RRect.fromRectAndRadius(rect.deflate(10.0), const Radius.circular(90.0)),
        strokeWidth: 0.0,
      ),
    );

    const RoundedRectangleBorder directional = RoundedRectangleBorder(
      borderRadius: BorderRadiusDirectional.only(topStart: Radius.circular(20)),
    );
    expect(ShapeBorder.lerp(directional, c10, 1.0), ShapeBorder.lerp(c10, directional, 0.0));
  });

  test('RoundedRectangleBorder and CircleBorder', () {
    const RoundedRectangleBorder r = RoundedRectangleBorder(
      borderRadius: BorderRadius.all(Radius.circular(10.0)),
    );
    const CircleBorder c = CircleBorder();
    const Rect rect = Rect.fromLTWH(0.0, 0.0, 100.0, 20.0); // center is x=40..60 y=10
    final Matcher looksLikeR = isPathThat(
      includes: const <Offset>[Offset(30.0, 10.0), Offset(50.0, 10.0)],
      excludes: const <Offset>[Offset(1.0, 1.0), Offset(99.0, 19.0)],
    );
    final Matcher looksLikeC = isPathThat(
      includes: const <Offset>[Offset(50.0, 10.0)],
      excludes: const <Offset>[Offset(1.0, 1.0), Offset(30.0, 10.0), Offset(99.0, 19.0)],
    );
    expect(r.getOuterPath(rect), looksLikeR);
    expect(c.getOuterPath(rect), looksLikeC);
    expect(ShapeBorder.lerp(r, c, 0.1)!.getOuterPath(rect), looksLikeR);
    expect(ShapeBorder.lerp(r, c, 0.9)!.getOuterPath(rect), looksLikeC);
    expect(ShapeBorder.lerp(ShapeBorder.lerp(r, c, 0.9), r, 0.1)!.getOuterPath(rect), looksLikeC);
    expect(ShapeBorder.lerp(ShapeBorder.lerp(r, c, 0.9), r, 0.9)!.getOuterPath(rect), looksLikeR);
    expect(ShapeBorder.lerp(ShapeBorder.lerp(r, c, 0.1), c, 0.1)!.getOuterPath(rect), looksLikeR);
    expect(ShapeBorder.lerp(ShapeBorder.lerp(r, c, 0.1), c, 0.9)!.getOuterPath(rect), looksLikeC);
    expect(
      ShapeBorder.lerp(
        ShapeBorder.lerp(r, c, 0.1),
        ShapeBorder.lerp(r, c, 0.9),
        0.1,
      )!.getOuterPath(rect),
      looksLikeR,
    );
    expect(
      ShapeBorder.lerp(
        ShapeBorder.lerp(r, c, 0.1),
        ShapeBorder.lerp(r, c, 0.9),
        0.9,
      )!.getOuterPath(rect),
      looksLikeC,
    );
    expect(ShapeBorder.lerp(r, ShapeBorder.lerp(r, c, 0.9), 0.1)!.getOuterPath(rect), looksLikeR);
    expect(ShapeBorder.lerp(r, ShapeBorder.lerp(r, c, 0.9), 0.9)!.getOuterPath(rect), looksLikeC);
    expect(ShapeBorder.lerp(c, ShapeBorder.lerp(r, c, 0.1), 0.1)!.getOuterPath(rect), looksLikeC);
    expect(ShapeBorder.lerp(c, ShapeBorder.lerp(r, c, 0.1), 0.9)!.getOuterPath(rect), looksLikeR);

    expect(
      ShapeBorder.lerp(r, c, 0.1).toString(),
      'RoundedRectangleBorder(BorderSide(width: 0.0, style: none), BorderRadius.circular(10.0), 10.0% of the way to being a CircleBorder)',
    );
    expect(
      ShapeBorder.lerp(r, c, 0.2).toString(),
      'RoundedRectangleBorder(BorderSide(width: 0.0, style: none), BorderRadius.circular(10.0), 20.0% of the way to being a CircleBorder)',
    );
    expect(
      ShapeBorder.lerp(ShapeBorder.lerp(r, c, 0.1), ShapeBorder.lerp(r, c, 0.9), 0.9).toString(),
      'RoundedRectangleBorder(BorderSide(width: 0.0, style: none), BorderRadius.circular(10.0), 82.0% of the way to being a CircleBorder)',
    );

    expect(
      ShapeBorder.lerp(c, r, 0.9).toString(),
      'RoundedRectangleBorder(BorderSide(width: 0.0, style: none), BorderRadius.circular(10.0), 10.0% of the way to being a CircleBorder)',
    );
    expect(
      ShapeBorder.lerp(c, r, 0.8).toString(),
      'RoundedRectangleBorder(BorderSide(width: 0.0, style: none), BorderRadius.circular(10.0), 20.0% of the way to being a CircleBorder)',
    );
    expect(
      ShapeBorder.lerp(ShapeBorder.lerp(r, c, 0.9), ShapeBorder.lerp(r, c, 0.1), 0.1).toString(),
      'RoundedRectangleBorder(BorderSide(width: 0.0, style: none), BorderRadius.circular(10.0), 82.0% of the way to being a CircleBorder)',
    );

    expect(ShapeBorder.lerp(r, c, 0.1), ShapeBorder.lerp(r, c, 0.1));
    expect(ShapeBorder.lerp(r, c, 0.1).hashCode, ShapeBorder.lerp(r, c, 0.1).hashCode);

    final ShapeBorder direct50 = ShapeBorder.lerp(r, c, 0.5)!;
    final ShapeBorder indirect50 =
        ShapeBorder.lerp(ShapeBorder.lerp(c, r, 0.1), ShapeBorder.lerp(c, r, 0.9), 0.5)!;
    expect(direct50, indirect50);
    expect(direct50.hashCode, indirect50.hashCode);
    expect(direct50.toString(), indirect50.toString());
  });

  test('RoundedRectangleBorder.dimensions and CircleBorder.dimensions', () {
    const RoundedRectangleBorder insideRoundedRectangleBorder = RoundedRectangleBorder(
      side: BorderSide(width: 10),
    );
    expect(insideRoundedRectangleBorder.dimensions, const EdgeInsets.all(10));

    const RoundedRectangleBorder centerRoundedRectangleBorder = RoundedRectangleBorder(
      side: BorderSide(width: 10, strokeAlign: BorderSide.strokeAlignCenter),
    );
    expect(centerRoundedRectangleBorder.dimensions, const EdgeInsets.all(5));

    const RoundedRectangleBorder outsideRoundedRectangleBorder = RoundedRectangleBorder(
      side: BorderSide(width: 10, strokeAlign: BorderSide.strokeAlignOutside),
    );
    expect(outsideRoundedRectangleBorder.dimensions, EdgeInsets.zero);

    const CircleBorder insideCircleBorder = CircleBorder(side: BorderSide(width: 10));
    expect(insideCircleBorder.dimensions, const EdgeInsets.all(10));

    const CircleBorder centerCircleBorder = CircleBorder(
      side: BorderSide(width: 10, strokeAlign: BorderSide.strokeAlignCenter),
    );
    expect(centerCircleBorder.dimensions, const EdgeInsets.all(5));

    const CircleBorder outsideCircleBorder = CircleBorder(
      side: BorderSide(width: 10, strokeAlign: BorderSide.strokeAlignOutside),
    );
    expect(outsideCircleBorder.dimensions, EdgeInsets.zero);
  });

  test('RoundedRectangleBorder.lerp with different StrokeAlign', () {
    const RoundedRectangleBorder rInside = RoundedRectangleBorder(side: BorderSide(width: 10.0));
    const RoundedRectangleBorder rOutside = RoundedRectangleBorder(
      side: BorderSide(width: 20.0, strokeAlign: BorderSide.strokeAlignOutside),
    );
    const RoundedRectangleBorder rCenter = RoundedRectangleBorder(
      side: BorderSide(width: 15.0, strokeAlign: BorderSide.strokeAlignCenter),
    );
    expect(ShapeBorder.lerp(rInside, rOutside, 0.5), rCenter);
  });
}
