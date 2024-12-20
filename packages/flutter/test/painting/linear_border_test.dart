// Copyright 2014 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

import 'package:flutter/painting.dart';
import 'package:flutter_test/flutter_test.dart';

const Rect canvasRect = Rect.fromLTWH(0, 0, 100, 100);
const BorderSide borderSide = BorderSide(width: 4, color: Color(0x0f00ff00));

// Test points for rectangular filled paths based on a BorderSide with width 4 and
// a 100x100 bounding rectangle (canvasRect).
List<Offset> rectIncludes(Rect r) {
  return <Offset>[r.topLeft, r.topRight, r.bottomLeft, r.bottomRight, r.center];
}

final List<Offset> leftRectIncludes = rectIncludes(const Rect.fromLTWH(0, 0, 4, 100));
final List<Offset> rightRectIncludes = rectIncludes(const Rect.fromLTWH(96, 0, 4, 100));
final List<Offset> topRectIncludes = rectIncludes(const Rect.fromLTWH(0, 0, 100, 4));
final List<Offset> bottomRectIncludes = rectIncludes(const Rect.fromLTWH(0, 96, 100, 4));

void main() {
  test('LinearBorderEdge defaults', () {
    expect(const LinearBorderEdge().size, 1);
    expect(const LinearBorderEdge().alignment, 0);
  });

  test('LinearBorder defaults', () {
    void expectEmptyBorder(LinearBorder border) {
      expect(border.side, BorderSide.none);
      expect(border.dimensions, EdgeInsets.zero);
      expect(border.preferPaintInterior, false);
      expect(border.start, null);
      expect(border.end, null);
      expect(border.top, null);
      expect(border.bottom, null);
    }

    expectEmptyBorder(LinearBorder.none);

    expect(LinearBorder.start().side, BorderSide.none);
    expect(LinearBorder.start().start, const LinearBorderEdge());
    expect(LinearBorder.start().end, null);
    expect(LinearBorder.start().top, null);
    expect(LinearBorder.start().bottom, null);

    expect(LinearBorder.end().side, BorderSide.none);
    expect(LinearBorder.end().start, null);
    expect(LinearBorder.end().end, const LinearBorderEdge());
    expect(LinearBorder.end().top, null);
    expect(LinearBorder.end().bottom, null);

    expect(LinearBorder.top().side, BorderSide.none);
    expect(LinearBorder.top().start, null);
    expect(LinearBorder.top().end, null);
    expect(LinearBorder.top().top, const LinearBorderEdge());
    expect(LinearBorder.top().bottom, null);

    expect(LinearBorder.bottom().side, BorderSide.none);
    expect(LinearBorder.bottom().start, null);
    expect(LinearBorder.bottom().end, null);
    expect(LinearBorder.bottom().top, null);
    expect(LinearBorder.bottom().bottom, const LinearBorderEdge());
  });

  test('LinearBorder copyWith, ==, hashCode', () {
    expect(LinearBorder.none, LinearBorder.none.copyWith());
    expect(LinearBorder.none.hashCode, LinearBorder.none.copyWith().hashCode);
    const BorderSide side = BorderSide(width: 10.0, color: Color(0xff123456));
    expect(LinearBorder.none.copyWith(side: side), const LinearBorder(side: side));
  });

  test('LinearBorder lerp identical a,b', () {
    expect(OutlinedBorder.lerp(null, null, 0), null);
    const LinearBorder border = LinearBorder.none;
    expect(identical(OutlinedBorder.lerp(border, border, 0.5), border), true);
  });

  test('LinearBorderEdge.lerp identical a,b', () {
    expect(LinearBorderEdge.lerp(null, null, 0), null);
    const LinearBorderEdge edge = LinearBorderEdge();
    expect(identical(LinearBorderEdge.lerp(edge, edge, 0.5), edge), true);
  });

  test(
    'LinearBorderEdge, LinearBorder toString()',
    () {
      expect(
        const LinearBorderEdge(size: 0.5, alignment: -0.5).toString(),
        'LinearBorderEdge(size: 0.5, alignment: -0.5)',
      );
      expect(LinearBorder.none.toString(), 'LinearBorder.none');
      const BorderSide side = BorderSide(width: 10.0, color: Color(0xff123456));
      expect(
        const LinearBorder(side: side).toString(),
        'LinearBorder(side: BorderSide(color: ${const Color(0xff123456)}, width: 10.0))',
      );
      expect(
        const LinearBorder(
          side: side,
          start: LinearBorderEdge(size: 0, alignment: -0.75),
          end: LinearBorderEdge(size: 0.25, alignment: -0.5),
          top: LinearBorderEdge(size: 0.5, alignment: 0.5),
          bottom: LinearBorderEdge(size: 0.75, alignment: 0.75),
        ).toString(),
        'LinearBorder('
        'side: BorderSide(color: ${const Color(0xff123456)}, width: 10.0), '
        'start: LinearBorderEdge(size: 0.0, alignment: -0.75), '
        'end: LinearBorderEdge(size: 0.25, alignment: -0.5), '
        'top: LinearBorderEdge(size: 0.5, alignment: 0.5), '
        'bottom: LinearBorderEdge(size: 0.75, alignment: 0.75))',
      );
    },
    skip: isBrowser, // [intended] see https://github.com/flutter/flutter/issues/118207
  );

  test('LinearBorder.start()', () {
    final LinearBorder border = LinearBorder.start(side: borderSide);
    expect(
      (Canvas canvas) => border.paint(canvas, canvasRect, textDirection: TextDirection.ltr),
      paints
        ..path(includes: leftRectIncludes, excludes: rightRectIncludes, color: borderSide.color),
    );
    expect(
      (Canvas canvas) => border.paint(canvas, canvasRect, textDirection: TextDirection.rtl),
      paints
        ..path(includes: rightRectIncludes, excludes: leftRectIncludes, color: borderSide.color),
    );
  });

  test('LinearBorder.end()', () {
    final LinearBorder border = LinearBorder.end(side: borderSide);
    expect(
      (Canvas canvas) => border.paint(canvas, canvasRect, textDirection: TextDirection.ltr),
      paints
        ..path(includes: rightRectIncludes, excludes: leftRectIncludes, color: borderSide.color),
    );
    expect(
      (Canvas canvas) => border.paint(canvas, canvasRect, textDirection: TextDirection.rtl),
      paints
        ..path(includes: leftRectIncludes, excludes: rightRectIncludes, color: borderSide.color),
    );
  });

  test('LinearBorder.top()', () {
    final LinearBorder border = LinearBorder.top(side: borderSide);
    expect(
      (Canvas canvas) => border.paint(canvas, canvasRect, textDirection: TextDirection.ltr),
      paints
        ..path(includes: topRectIncludes, excludes: bottomRectIncludes, color: borderSide.color),
    );
  });

  test('LinearBorder.bottom()', () {
    final LinearBorder border = LinearBorder.bottom(side: borderSide);
    expect(
      (Canvas canvas) => border.paint(canvas, canvasRect, textDirection: TextDirection.ltr),
      paints
        ..path(includes: bottomRectIncludes, excludes: topRectIncludes, color: borderSide.color),
    );
  });
}
