// Copyright 2014 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

import 'package:flutter/painting.dart';
import 'package:flutter_test/flutter_test.dart';

import 'common_matchers.dart';

void main() {
  test('StadiumBorder defaults', () {
    const StadiumBorder border = StadiumBorder();
    expect(border.side, BorderSide.none);
  });

  test('StadiumBorder copyWith, ==, hashCode', () {
    expect(const StadiumBorder(), const StadiumBorder().copyWith());
    expect(const StadiumBorder().hashCode, const StadiumBorder().copyWith().hashCode);
    const BorderSide side = BorderSide(width: 10.0, color: Color(0xff123456));
    expect(const StadiumBorder().copyWith(side: side), const StadiumBorder(side: side));
  });

  test('StadiumBorder', () {
    const StadiumBorder c10 = StadiumBorder(side: BorderSide(width: 10.0));
    const StadiumBorder c15 = StadiumBorder(side: BorderSide(width: 15.0));
    const StadiumBorder c20 = StadiumBorder(side: BorderSide(width: 20.0));
    expect(c10.dimensions, const EdgeInsets.all(10.0));
    expect(c10.scale(2.0), c20);
    expect(c20.scale(0.5), c10);
    expect(ShapeBorder.lerp(c10, c20, 0.0), c10);
    expect(ShapeBorder.lerp(c10, c20, 0.5), c15);
    expect(ShapeBorder.lerp(c10, c20, 1.0), c20);

    const StadiumBorder c1 = StadiumBorder(side: BorderSide());
    expect(c1.getOuterPath(Rect.fromCircle(center: Offset.zero, radius: 1.0)), isUnitCircle);
    const StadiumBorder c2 = StadiumBorder(side: BorderSide());
    expect(c2.getInnerPath(Rect.fromCircle(center: Offset.zero, radius: 2.0)), isUnitCircle);
    const Rect rect = Rect.fromLTRB(10.0, 20.0, 100.0, 200.0);
    expect(
      (Canvas canvas) => c10.paint(canvas, rect),
      paints..rrect(
        rrect: RRect.fromRectAndRadius(
          rect.deflate(5.0),
          Radius.circular(rect.shortestSide / 2.0 - 5.0),
        ),
        strokeWidth: 10.0,
      ),
    );
  });

  test('StadiumBorder with StrokeAlign', () {
    const StadiumBorder center = StadiumBorder(
      side: BorderSide(width: 10.0, strokeAlign: BorderSide.strokeAlignCenter),
    );
    const StadiumBorder outside = StadiumBorder(
      side: BorderSide(width: 10.0, strokeAlign: BorderSide.strokeAlignOutside),
    );
    expect(center.dimensions, const EdgeInsets.all(5.0));
    expect(outside.dimensions, EdgeInsets.zero);

    const Rect rect = Rect.fromLTRB(10.0, 20.0, 100.0, 200.0);

    expect(
      (Canvas canvas) => center.paint(canvas, rect),
      paints..rrect(
        rrect: RRect.fromRectAndRadius(rect, Radius.circular(rect.shortestSide / 2.0)),
        strokeWidth: 10.0,
      ),
    );

    expect(
      (Canvas canvas) => outside.paint(canvas, rect),
      paints..rrect(
        rrect: RRect.fromRectAndRadius(rect, Radius.circular(rect.shortestSide / 2.0)).inflate(5.0),
        strokeWidth: 10.0,
      ),
    );
  });

  test('StadiumBorder and CircleBorder', () {
    const StadiumBorder stadium = StadiumBorder();
    const CircleBorder circle = CircleBorder();
    const Rect rect = Rect.fromLTWH(0.0, 0.0, 100.0, 20.0);
    final Matcher looksLikeS = isPathThat(
      includes: const <Offset>[Offset(30.0, 10.0), Offset(50.0, 10.0)],
      excludes: const <Offset>[Offset(1.0, 1.0), Offset(99.0, 19.0)],
    );
    final Matcher looksLikeC = isPathThat(
      includes: const <Offset>[Offset(50.0, 10.0)],
      excludes: const <Offset>[Offset(1.0, 1.0), Offset(30.0, 10.0), Offset(99.0, 19.0)],
    );
    expect(stadium.getOuterPath(rect), looksLikeS);
    expect(circle.getOuterPath(rect), looksLikeC);
    expect(ShapeBorder.lerp(stadium, circle, 0.1)!.getOuterPath(rect), looksLikeS);
    expect(ShapeBorder.lerp(stadium, circle, 0.9)!.getOuterPath(rect), looksLikeC);
    expect(
      ShapeBorder.lerp(ShapeBorder.lerp(stadium, circle, 0.9), stadium, 0.1)!.getOuterPath(rect),
      looksLikeC,
    );
    expect(
      ShapeBorder.lerp(ShapeBorder.lerp(stadium, circle, 0.9), stadium, 0.9)!.getOuterPath(rect),
      looksLikeS,
    );
    expect(
      ShapeBorder.lerp(ShapeBorder.lerp(stadium, circle, 0.1), circle, 0.1)!.getOuterPath(rect),
      looksLikeS,
    );
    expect(
      ShapeBorder.lerp(ShapeBorder.lerp(stadium, circle, 0.1), circle, 0.9)!.getOuterPath(rect),
      looksLikeC,
    );
    expect(
      ShapeBorder.lerp(
        ShapeBorder.lerp(stadium, circle, 0.1),
        ShapeBorder.lerp(stadium, circle, 0.9),
        0.1,
      )!.getOuterPath(rect),
      looksLikeS,
    );
    expect(
      ShapeBorder.lerp(
        ShapeBorder.lerp(stadium, circle, 0.1),
        ShapeBorder.lerp(stadium, circle, 0.9),
        0.9,
      )!.getOuterPath(rect),
      looksLikeC,
    );
    expect(
      ShapeBorder.lerp(stadium, ShapeBorder.lerp(stadium, circle, 0.9), 0.1)!.getOuterPath(rect),
      looksLikeS,
    );
    expect(
      ShapeBorder.lerp(stadium, ShapeBorder.lerp(stadium, circle, 0.9), 0.9)!.getOuterPath(rect),
      looksLikeC,
    );
    expect(
      ShapeBorder.lerp(circle, ShapeBorder.lerp(stadium, circle, 0.1), 0.1)!.getOuterPath(rect),
      looksLikeC,
    );
    expect(
      ShapeBorder.lerp(circle, ShapeBorder.lerp(stadium, circle, 0.1), 0.9)!.getOuterPath(rect),
      looksLikeS,
    );

    expect(
      ShapeBorder.lerp(stadium, circle, 0.1).toString(),
      'StadiumBorder(BorderSide(width: 0.0, style: none), 10.0% of the way to being a CircleBorder)',
    );
    expect(
      ShapeBorder.lerp(stadium, circle, 0.2).toString(),
      'StadiumBorder(BorderSide(width: 0.0, style: none), 20.0% of the way to being a CircleBorder)',
    );
    expect(
      ShapeBorder.lerp(
        ShapeBorder.lerp(stadium, circle, 0.1),
        ShapeBorder.lerp(stadium, circle, 0.9),
        0.9,
      ).toString(),
      'StadiumBorder(BorderSide(width: 0.0, style: none), 82.0% of the way to being a CircleBorder)',
    );

    expect(
      ShapeBorder.lerp(circle, stadium, 0.9).toString(),
      'StadiumBorder(BorderSide(width: 0.0, style: none), 10.0% of the way to being a CircleBorder)',
    );
    expect(
      ShapeBorder.lerp(circle, stadium, 0.8).toString(),
      'StadiumBorder(BorderSide(width: 0.0, style: none), 20.0% of the way to being a CircleBorder)',
    );
    expect(
      ShapeBorder.lerp(
        ShapeBorder.lerp(stadium, circle, 0.9),
        ShapeBorder.lerp(stadium, circle, 0.1),
        0.1,
      ).toString(),
      'StadiumBorder(BorderSide(width: 0.0, style: none), 82.0% of the way to being a CircleBorder)',
    );

    expect(ShapeBorder.lerp(stadium, circle, 0.1), ShapeBorder.lerp(stadium, circle, 0.1));
    expect(
      ShapeBorder.lerp(stadium, circle, 0.1).hashCode,
      ShapeBorder.lerp(stadium, circle, 0.1).hashCode,
    );

    final ShapeBorder direct50 = ShapeBorder.lerp(stadium, circle, 0.5)!;
    final ShapeBorder indirect50 =
        ShapeBorder.lerp(
          ShapeBorder.lerp(circle, stadium, 0.1),
          ShapeBorder.lerp(circle, stadium, 0.9),
          0.5,
        )!;
    expect(direct50, indirect50);
    expect(direct50.hashCode, indirect50.hashCode);
    expect(direct50.toString(), indirect50.toString());
  });

  test('StadiumBorder and RoundedRectBorder with BorderRadius.zero', () {
    const StadiumBorder stadium = StadiumBorder();
    const RoundedRectangleBorder rrect = RoundedRectangleBorder();
    const Rect rect = Rect.fromLTWH(0.0, 0.0, 100.0, 50.0);
    final Matcher looksLikeS = isPathThat(
      includes: const <Offset>[Offset(25.0, 25.0), Offset(50.0, 25.0), Offset(7.33, 7.33)],
      excludes: const <Offset>[
        Offset(0.001, 0.001),
        Offset(99.999, 0.001),
        Offset(99.999, 49.999),
        Offset(0.001, 49.999),
      ],
    );
    final Matcher looksLikeR = isPathThat(
      includes: const <Offset>[
        Offset(25.0, 25.0),
        Offset(50.0, 25.0),
        Offset(7.33, 7.33),
        Offset(4.0, 4.0),
        Offset(96.0, 4.0),
        Offset(96.0, 46.0),
        Offset(4.0, 46.0),
      ],
    );
    expect(stadium.getOuterPath(rect), looksLikeS);
    expect(rrect.getOuterPath(rect), looksLikeR);
    expect(ShapeBorder.lerp(stadium, rrect, 0.1)!.getOuterPath(rect), looksLikeS);
    expect(ShapeBorder.lerp(stadium, rrect, 0.9)!.getOuterPath(rect), looksLikeR);
    expect(
      ShapeBorder.lerp(ShapeBorder.lerp(stadium, rrect, 0.9), stadium, 0.1)!.getOuterPath(rect),
      looksLikeR,
    );
    expect(
      ShapeBorder.lerp(ShapeBorder.lerp(stadium, rrect, 0.9), stadium, 0.9)!.getOuterPath(rect),
      looksLikeS,
    );
    expect(
      ShapeBorder.lerp(ShapeBorder.lerp(stadium, rrect, 0.1), rrect, 0.1)!.getOuterPath(rect),
      looksLikeS,
    );
    expect(
      ShapeBorder.lerp(ShapeBorder.lerp(stadium, rrect, 0.1), rrect, 0.9)!.getOuterPath(rect),
      looksLikeS,
    );
    expect(
      ShapeBorder.lerp(
        ShapeBorder.lerp(stadium, rrect, 0.1),
        ShapeBorder.lerp(stadium, rrect, 0.9),
        0.1,
      )!.getOuterPath(rect),
      looksLikeS,
    );
    expect(
      ShapeBorder.lerp(
        ShapeBorder.lerp(stadium, rrect, 0.1),
        ShapeBorder.lerp(stadium, rrect, 0.9),
        0.9,
      )!.getOuterPath(rect),
      looksLikeR,
    );
    expect(
      ShapeBorder.lerp(stadium, ShapeBorder.lerp(stadium, rrect, 0.9), 0.1)!.getOuterPath(rect),
      looksLikeS,
    );
    expect(
      ShapeBorder.lerp(stadium, ShapeBorder.lerp(stadium, rrect, 0.9), 0.9)!.getOuterPath(rect),
      looksLikeR,
    );
    expect(
      ShapeBorder.lerp(rrect, ShapeBorder.lerp(stadium, rrect, 0.1), 0.1)!.getOuterPath(rect),
      looksLikeS,
    );
    expect(
      ShapeBorder.lerp(rrect, ShapeBorder.lerp(stadium, rrect, 0.1), 0.9)!.getOuterPath(rect),
      looksLikeS,
    );

    expect(
      ShapeBorder.lerp(stadium, rrect, 0.1).toString(),
      'StadiumBorder(BorderSide(width: 0.0, style: none), '
      'BorderRadius.zero, 10.0% of the way to being a RoundedRectangleBorder)',
    );
    expect(
      ShapeBorder.lerp(stadium, rrect, 0.2).toString(),
      'StadiumBorder(BorderSide(width: 0.0, style: none), '
      'BorderRadius.zero, 20.0% of the way to being a RoundedRectangleBorder)',
    );
    expect(
      ShapeBorder.lerp(
        ShapeBorder.lerp(stadium, rrect, 0.1),
        ShapeBorder.lerp(stadium, rrect, 0.9),
        0.9,
      ).toString(),
      'StadiumBorder(BorderSide(width: 0.0, style: none), '
      'BorderRadius.zero, 82.0% of the way to being a RoundedRectangleBorder)',
    );

    expect(
      ShapeBorder.lerp(rrect, stadium, 0.9).toString(),
      'StadiumBorder(BorderSide(width: 0.0, style: none), '
      'BorderRadius.zero, 10.0% of the way to being a RoundedRectangleBorder)',
    );
    expect(
      ShapeBorder.lerp(rrect, stadium, 0.8).toString(),
      'StadiumBorder(BorderSide(width: 0.0, style: none), '
      'BorderRadius.zero, 20.0% of the way to being a RoundedRectangleBorder)',
    );
    expect(
      ShapeBorder.lerp(
        ShapeBorder.lerp(stadium, rrect, 0.9),
        ShapeBorder.lerp(stadium, rrect, 0.1),
        0.1,
      ).toString(),
      'StadiumBorder(BorderSide(width: 0.0, style: none), '
      'BorderRadius.zero, 82.0% of the way to being a RoundedRectangleBorder)',
    );

    expect(ShapeBorder.lerp(stadium, rrect, 0.1), ShapeBorder.lerp(stadium, rrect, 0.1));
    expect(
      ShapeBorder.lerp(stadium, rrect, 0.1).hashCode,
      ShapeBorder.lerp(stadium, rrect, 0.1).hashCode,
    );

    final ShapeBorder direct50 = ShapeBorder.lerp(stadium, rrect, 0.5)!;
    final ShapeBorder indirect50 =
        ShapeBorder.lerp(
          ShapeBorder.lerp(rrect, stadium, 0.1),
          ShapeBorder.lerp(rrect, stadium, 0.9),
          0.5,
        )!;
    expect(direct50, indirect50);
    expect(direct50.hashCode, indirect50.hashCode);
    expect(direct50.toString(), indirect50.toString());
  });

  test('StadiumBorder and RoundedRectBorder with circular BorderRadius', () {
    const StadiumBorder stadium = StadiumBorder();
    const RoundedRectangleBorder rrect = RoundedRectangleBorder(
      borderRadius: BorderRadius.all(Radius.circular(10)),
    );
    const Rect rect = Rect.fromLTWH(0.0, 0.0, 100.0, 50.0);
    final Matcher looksLikeS = isPathThat(
      includes: const <Offset>[Offset(25.0, 25.0), Offset(50.0, 25.0), Offset(7.33, 7.33)],
      excludes: const <Offset>[
        Offset(0.001, 0.001),
        Offset(99.999, 0.001),
        Offset(99.999, 49.999),
        Offset(0.001, 49.999),
      ],
    );
    final Matcher looksLikeR = isPathThat(
      includes: const <Offset>[
        Offset(25.0, 25.0),
        Offset(50.0, 25.0),
        Offset(7.33, 7.33),
        Offset(4.0, 4.0),
        Offset(96.0, 4.0),
        Offset(96.0, 46.0),
        Offset(4.0, 46.0),
      ],
    );
    expect(stadium.getOuterPath(rect), looksLikeS);
    expect(rrect.getOuterPath(rect), looksLikeR);
    expect(ShapeBorder.lerp(stadium, rrect, 0.1)!.getOuterPath(rect), looksLikeS);
    expect(ShapeBorder.lerp(stadium, rrect, 0.9)!.getOuterPath(rect), looksLikeR);
    expect(
      ShapeBorder.lerp(ShapeBorder.lerp(stadium, rrect, 0.9), stadium, 0.1)!.getOuterPath(rect),
      looksLikeR,
    );
    expect(
      ShapeBorder.lerp(ShapeBorder.lerp(stadium, rrect, 0.9), stadium, 0.9)!.getOuterPath(rect),
      looksLikeS,
    );
    expect(
      ShapeBorder.lerp(ShapeBorder.lerp(stadium, rrect, 0.1), rrect, 0.1)!.getOuterPath(rect),
      looksLikeS,
    );
    expect(
      ShapeBorder.lerp(ShapeBorder.lerp(stadium, rrect, 0.1), rrect, 0.9)!.getOuterPath(rect),
      looksLikeS,
    );
    expect(
      ShapeBorder.lerp(
        ShapeBorder.lerp(stadium, rrect, 0.1),
        ShapeBorder.lerp(stadium, rrect, 0.9),
        0.1,
      )!.getOuterPath(rect),
      looksLikeS,
    );
    expect(
      ShapeBorder.lerp(
        ShapeBorder.lerp(stadium, rrect, 0.1),
        ShapeBorder.lerp(stadium, rrect, 0.9),
        0.9,
      )!.getOuterPath(rect),
      looksLikeR,
    );
    expect(
      ShapeBorder.lerp(stadium, ShapeBorder.lerp(stadium, rrect, 0.9), 0.1)!.getOuterPath(rect),
      looksLikeS,
    );
    expect(
      ShapeBorder.lerp(stadium, ShapeBorder.lerp(stadium, rrect, 0.9), 0.9)!.getOuterPath(rect),
      looksLikeR,
    );
    expect(
      ShapeBorder.lerp(rrect, ShapeBorder.lerp(stadium, rrect, 0.1), 0.1)!.getOuterPath(rect),
      looksLikeS,
    );
    expect(
      ShapeBorder.lerp(rrect, ShapeBorder.lerp(stadium, rrect, 0.1), 0.9)!.getOuterPath(rect),
      looksLikeS,
    );

    expect(
      ShapeBorder.lerp(stadium, rrect, 0.1).toString(),
      'StadiumBorder(BorderSide(width: 0.0, style: none), '
      'BorderRadius.circular(10.0), 10.0% of the way to being a RoundedRectangleBorder)',
    );
    expect(
      ShapeBorder.lerp(stadium, rrect, 0.2).toString(),
      'StadiumBorder(BorderSide(width: 0.0, style: none), '
      'BorderRadius.circular(10.0), 20.0% of the way to being a RoundedRectangleBorder)',
    );
    expect(
      ShapeBorder.lerp(
        ShapeBorder.lerp(stadium, rrect, 0.1),
        ShapeBorder.lerp(stadium, rrect, 0.9),
        0.9,
      ).toString(),
      'StadiumBorder(BorderSide(width: 0.0, style: none), '
      'BorderRadius.circular(10.0), 82.0% of the way to being a RoundedRectangleBorder)',
    );

    expect(
      ShapeBorder.lerp(rrect, stadium, 0.9).toString(),
      'StadiumBorder(BorderSide(width: 0.0, style: none), '
      'BorderRadius.circular(10.0), 10.0% of the way to being a RoundedRectangleBorder)',
    );
    expect(
      ShapeBorder.lerp(rrect, stadium, 0.8).toString(),
      'StadiumBorder(BorderSide(width: 0.0, style: none), '
      'BorderRadius.circular(10.0), 20.0% of the way to being a RoundedRectangleBorder)',
    );
    expect(
      ShapeBorder.lerp(
        ShapeBorder.lerp(stadium, rrect, 0.9),
        ShapeBorder.lerp(stadium, rrect, 0.1),
        0.1,
      ).toString(),
      'StadiumBorder(BorderSide(width: 0.0, style: none), '
      'BorderRadius.circular(10.0), 82.0% of the way to being a RoundedRectangleBorder)',
    );

    expect(ShapeBorder.lerp(stadium, rrect, 0.1), ShapeBorder.lerp(stadium, rrect, 0.1));
    expect(
      ShapeBorder.lerp(stadium, rrect, 0.1).hashCode,
      ShapeBorder.lerp(stadium, rrect, 0.1).hashCode,
    );

    final ShapeBorder direct50 = ShapeBorder.lerp(stadium, rrect, 0.5)!;
    final ShapeBorder indirect50 =
        ShapeBorder.lerp(
          ShapeBorder.lerp(rrect, stadium, 0.1),
          ShapeBorder.lerp(rrect, stadium, 0.9),
          0.5,
        )!;
    expect(direct50, indirect50);
    expect(direct50.hashCode, indirect50.hashCode);
    expect(direct50.toString(), indirect50.toString());
  });

  test('StadiumBorder and RoundedRectBorder with BorderRadiusDirectional', () {
    const StadiumBorder stadium = StadiumBorder();
    const RoundedRectangleBorder rrect = RoundedRectangleBorder(
      borderRadius: BorderRadiusDirectional.only(
        topStart: Radius.circular(10),
        bottomEnd: Radius.circular(10),
      ),
    );
    const Rect rect = Rect.fromLTWH(0.0, 0.0, 100.0, 50.0);
    final Matcher looksLikeS = isPathThat(
      includes: const <Offset>[Offset(25.0, 25.0), Offset(50.0, 25.0), Offset(7.33, 7.33)],
      excludes: const <Offset>[
        Offset(0.001, 0.001),
        Offset(99.999, 0.001),
        Offset(99.999, 49.999),
        Offset(0.001, 49.999),
      ],
    );
    final Matcher looksLikeR = isPathThat(
      includes: const <Offset>[
        Offset(25.0, 25.0),
        Offset(50.0, 25.0),
        Offset(7.33, 7.33),
        Offset(4.0, 4.0),
        Offset(96.0, 4.0),
        Offset(96.0, 46.0),
        Offset(4.0, 46.0),
      ],
    );
    expect(stadium.getOuterPath(rect, textDirection: TextDirection.rtl), looksLikeS);
    expect(rrect.getOuterPath(rect, textDirection: TextDirection.rtl), looksLikeR);
    expect(
      ShapeBorder.lerp(stadium, rrect, 0.1)!.getOuterPath(rect, textDirection: TextDirection.rtl),
      looksLikeS,
    );
    expect(
      ShapeBorder.lerp(stadium, rrect, 0.9)!.getOuterPath(rect, textDirection: TextDirection.rtl),
      looksLikeR,
    );
    expect(
      ShapeBorder.lerp(
        ShapeBorder.lerp(stadium, rrect, 0.9),
        stadium,
        0.1,
      )!.getOuterPath(rect, textDirection: TextDirection.rtl),
      looksLikeR,
    );
    expect(
      ShapeBorder.lerp(
        ShapeBorder.lerp(stadium, rrect, 0.9),
        stadium,
        0.9,
      )!.getOuterPath(rect, textDirection: TextDirection.rtl),
      looksLikeS,
    );
    expect(
      ShapeBorder.lerp(
        ShapeBorder.lerp(stadium, rrect, 0.1),
        rrect,
        0.1,
      )!.getOuterPath(rect, textDirection: TextDirection.rtl),
      looksLikeS,
    );
    expect(
      ShapeBorder.lerp(
        ShapeBorder.lerp(stadium, rrect, 0.1),
        rrect,
        0.9,
      )!.getOuterPath(rect, textDirection: TextDirection.rtl),
      looksLikeS,
    );
    expect(
      ShapeBorder.lerp(
        ShapeBorder.lerp(stadium, rrect, 0.1),
        ShapeBorder.lerp(stadium, rrect, 0.9),
        0.1,
      )!.getOuterPath(rect, textDirection: TextDirection.rtl),
      looksLikeS,
    );
    expect(
      ShapeBorder.lerp(
        ShapeBorder.lerp(stadium, rrect, 0.1),
        ShapeBorder.lerp(stadium, rrect, 0.9),
        0.9,
      )!.getOuterPath(rect, textDirection: TextDirection.rtl),
      looksLikeR,
    );
    expect(
      ShapeBorder.lerp(
        stadium,
        ShapeBorder.lerp(stadium, rrect, 0.9),
        0.1,
      )!.getOuterPath(rect, textDirection: TextDirection.rtl),
      looksLikeS,
    );
    expect(
      ShapeBorder.lerp(
        stadium,
        ShapeBorder.lerp(stadium, rrect, 0.9),
        0.9,
      )!.getOuterPath(rect, textDirection: TextDirection.rtl),
      looksLikeR,
    );
    expect(
      ShapeBorder.lerp(
        rrect,
        ShapeBorder.lerp(stadium, rrect, 0.1),
        0.1,
      )!.getOuterPath(rect, textDirection: TextDirection.rtl),
      looksLikeS,
    );
    expect(
      ShapeBorder.lerp(
        rrect,
        ShapeBorder.lerp(stadium, rrect, 0.1),
        0.9,
      )!.getOuterPath(rect, textDirection: TextDirection.rtl),
      looksLikeS,
    );

    expect(
      ShapeBorder.lerp(stadium, rrect, 0.1).toString(),
      'StadiumBorder(BorderSide(width: 0.0, style: none), '
      'BorderRadiusDirectional.only(topStart: Radius.circular(10.0), '
      'bottomEnd: Radius.circular(10.0)), 10.0% of the way to being a RoundedRectangleBorder)',
    );
    expect(
      ShapeBorder.lerp(stadium, rrect, 0.2).toString(),
      'StadiumBorder(BorderSide(width: 0.0, style: none), '
      'BorderRadiusDirectional.only(topStart: Radius.circular(10.0), '
      'bottomEnd: Radius.circular(10.0)), 20.0% of the way to being a RoundedRectangleBorder)',
    );
    expect(
      ShapeBorder.lerp(
        ShapeBorder.lerp(stadium, rrect, 0.1),
        ShapeBorder.lerp(stadium, rrect, 0.9),
        0.9,
      ).toString(),
      'StadiumBorder(BorderSide(width: 0.0, style: none), '
      'BorderRadiusDirectional.only(topStart: Radius.circular(10.0), '
      'bottomEnd: Radius.circular(10.0)), 82.0% of the way to being a RoundedRectangleBorder)',
    );
    expect(
      ShapeBorder.lerp(rrect, stadium, 0.9).toString(),
      'StadiumBorder(BorderSide(width: 0.0, style: none), '
      'BorderRadiusDirectional.only(topStart: Radius.circular(10.0), '
      'bottomEnd: Radius.circular(10.0)), 10.0% of the way to being a RoundedRectangleBorder)',
    );
    expect(
      ShapeBorder.lerp(rrect, stadium, 0.8).toString(),
      'StadiumBorder(BorderSide(width: 0.0, style: none), '
      'BorderRadiusDirectional.only(topStart: Radius.circular(10.0), '
      'bottomEnd: Radius.circular(10.0)), 20.0% of the way to being a RoundedRectangleBorder)',
    );
    expect(
      ShapeBorder.lerp(
        ShapeBorder.lerp(stadium, rrect, 0.9),
        ShapeBorder.lerp(stadium, rrect, 0.1),
        0.1,
      ).toString(),
      'StadiumBorder(BorderSide(width: 0.0, style: none), '
      'BorderRadiusDirectional.only(topStart: Radius.circular(10.0), '
      'bottomEnd: Radius.circular(10.0)), 82.0% of the way to being a RoundedRectangleBorder)',
    );

    expect(ShapeBorder.lerp(stadium, rrect, 0.1), ShapeBorder.lerp(stadium, rrect, 0.1));
    expect(
      ShapeBorder.lerp(stadium, rrect, 0.1).hashCode,
      ShapeBorder.lerp(stadium, rrect, 0.1).hashCode,
    );

    final ShapeBorder direct50 = ShapeBorder.lerp(stadium, rrect, 0.5)!;
    final ShapeBorder indirect50 =
        ShapeBorder.lerp(
          ShapeBorder.lerp(rrect, stadium, 0.1),
          ShapeBorder.lerp(rrect, stadium, 0.9),
          0.5,
        )!;
    expect(direct50, indirect50);
    expect(direct50.hashCode, indirect50.hashCode);
    expect(direct50.toString(), indirect50.toString());
  });
}
