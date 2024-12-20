// Copyright 2014 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

import 'package:flutter/painting.dart';
import 'package:flutter_test/flutter_test.dart';

void main() {
  test('BoxDecoration.lerp identical a,b', () {
    expect(BoxDecoration.lerp(null, null, 0), null);
    const BoxDecoration decoration = BoxDecoration();
    expect(identical(BoxDecoration.lerp(decoration, decoration, 0.5), decoration), true);
  });

  test('BoxDecoration with BorderRadiusDirectional', () {
    const BoxDecoration decoration = BoxDecoration(
      color: Color(0xFF000000),
      borderRadius: BorderRadiusDirectional.only(topStart: Radius.circular(100.0)),
    );
    final BoxPainter painter = decoration.createBoxPainter();
    const Size size = Size(1000.0, 1000.0);
    expect(
      (Canvas canvas) {
        painter.paint(
          canvas,
          Offset.zero,
          const ImageConfiguration(size: size, textDirection: TextDirection.rtl),
        );
      },
      paints..rrect(
        rrect: RRect.fromRectAndCorners(Offset.zero & size, topRight: const Radius.circular(100.0)),
      ),
    );
    expect(
      decoration.hitTest(size, const Offset(10.0, 10.0), textDirection: TextDirection.rtl),
      isTrue,
    );
    expect(
      decoration.hitTest(size, const Offset(990.0, 10.0), textDirection: TextDirection.rtl),
      isFalse,
    );
    expect(
      (Canvas canvas) {
        painter.paint(
          canvas,
          Offset.zero,
          const ImageConfiguration(size: size, textDirection: TextDirection.ltr),
        );
      },
      paints..rrect(
        rrect: RRect.fromRectAndCorners(Offset.zero & size, topLeft: const Radius.circular(100.0)),
      ),
    );
    expect(
      decoration.hitTest(size, const Offset(10.0, 10.0), textDirection: TextDirection.ltr),
      isFalse,
    );
    expect(
      decoration.hitTest(size, const Offset(990.0, 10.0), textDirection: TextDirection.ltr),
      isTrue,
    );
  });

  test('BoxDecoration with LinearGradient using AlignmentDirectional', () {
    const BoxDecoration decoration = BoxDecoration(
      color: Color(0xFF000000),
      gradient: LinearGradient(
        begin: AlignmentDirectional.centerStart,
        end: AlignmentDirectional.bottomEnd,
        colors: <Color>[Color(0xFF000000), Color(0xFFFFFFFF)],
      ),
    );
    final BoxPainter painter = decoration.createBoxPainter();
    const Size size = Size(1000.0, 1000.0);
    expect((Canvas canvas) {
      painter.paint(
        canvas,
        Offset.zero,
        const ImageConfiguration(size: size, textDirection: TextDirection.rtl),
      );
    }, paints..rect(rect: Offset.zero & size));
  });

  test('BoxDecoration.getClipPath with borderRadius', () {
    const double radius = 10;
    final BoxDecoration decoration = BoxDecoration(borderRadius: BorderRadius.circular(radius));
    const Rect rect = Rect.fromLTWH(0.0, 0.0, 100.0, 20.0);
    final Path clipPath = decoration.getClipPath(rect, TextDirection.ltr);
    final Matcher isLookLikeExpectedPath = isPathThat(
      includes: const <Offset>[Offset(30.0, 10.0), Offset(50.0, 10.0)],
      excludes: const <Offset>[Offset(1.0, 1.0), Offset(99.0, 19.0)],
    );
    expect(clipPath, isLookLikeExpectedPath);
  });

  test('BoxDecoration.getClipPath with shape BoxShape.circle', () {
    const BoxDecoration decoration = BoxDecoration(shape: BoxShape.circle);
    const Rect rect = Rect.fromLTWH(0.0, 0.0, 100.0, 20.0);
    final Path clipPath = decoration.getClipPath(rect, TextDirection.ltr);
    final Matcher isLookLikeExpectedPath = isPathThat(
      includes: const <Offset>[Offset(50.0, 0.0), Offset(40.0, 10.0)],
      excludes: const <Offset>[Offset(40.0, 0.0), Offset(10.0, 10.0)],
    );
    expect(clipPath, isLookLikeExpectedPath);
  });

  test('BoxDecorations with different blendModes are not equal', () {
    // Regression test for https://github.com/flutter/flutter/issues/100754.
    const BoxDecoration one = BoxDecoration(
      color: Color(0x00000000),
      backgroundBlendMode: BlendMode.color,
    );
    const BoxDecoration two = BoxDecoration(
      color: Color(0x00000000),
      backgroundBlendMode: BlendMode.difference,
    );
    expect(one == two, isFalse);
  });
}
