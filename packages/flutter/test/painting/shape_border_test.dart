// Copyright 2014 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

import 'package:flutter/painting.dart';
import 'package:flutter_test/flutter_test.dart';

void main() {
  test('Border.lerp identical a,b', () {
    expect(Border.lerp(null, null, 0), null);
    const Border border = Border();
    expect(identical(Border.lerp(border, border, 0.5), border), true);
  });

  test('BoxBorder.lerp identical a,b', () {
    expect(BoxBorder.lerp(null, null, 0), null);
    const BoxBorder border = Border();
    expect(identical(BoxBorder.lerp(border, border, 0.5), border), true);
  });

  test('BorderDirectional.lep identical a,b', () {
    expect(BorderDirectional.lerp(null, null, 0), null);
    const BorderDirectional border = BorderDirectional();
    expect(identical(ShapeBorder.lerp(border, border, 0.5), border), true);
  });

  test('OutlinedBorder.lep identical a,b', () {
    expect(OutlinedBorder.lerp(null, null, 0), null);
    const OutlinedBorder border = RoundedRectangleBorder();
    expect(identical(OutlinedBorder.lerp(border, border, 0.5), border), true);
  });

  test('ShapeBorder.lep identical a,b', () {
    expect(ShapeBorder.lerp(null, null, 0), null);
    const ShapeBorder border = CircleBorder();
    expect(identical(ShapeBorder.lerp(border, border, 0.5), border), true);
  });

  test('Compound borders', () {
    final Border b1 = Border.all(color: const Color(0xFF00FF00));
    final Border b2 = Border.all(color: const Color(0xFF0000FF));
    expect(
      (b1 + b2).toString(),
      'Border.all(BorderSide(color: ${const Color(0xff00ff00)})) + '
      'Border.all(BorderSide(color: ${const Color(0xff0000ff)}))',
    );
    expect(
      (b1 + (b2 + b2)).toString(),
      'Border.all(BorderSide(color: ${const Color(0xff00ff00)})) + '
      'Border.all(BorderSide(color: ${const Color(0xff0000ff)}, width: 2.0))',
    );
    expect(
      ((b1 + b2) + b2).toString(),
      'Border.all(BorderSide(color: ${const Color(0xff00ff00)})) + '
      'Border.all(BorderSide(color: ${const Color(0xff0000ff)}, width: 2.0))',
    );
    expect((b1 + b2) + b2, b1 + (b2 + b2));
    expect(
      (b1 + b2).scale(3.0).toString(),
      'Border.all(BorderSide(color: ${const Color(0xff00ff00)}, width: 3.0)) + '
      'Border.all(BorderSide(color: ${const Color(0xff0000ff)}, width: 3.0))',
    );
    expect(
      (b1 + b2).scale(0.0).toString(),
      'Border.all(BorderSide(color: ${const Color(0xff00ff00)}, width: 0.0, style: none)) + '
      'Border.all(BorderSide(color: ${const Color(0xff0000ff)}, width: 0.0, style: none))',
    );
    expect(
      ShapeBorder.lerp(b2 + b1, b1 + b2, 0.0).toString(),
      'Border.all(BorderSide(color: ${const Color(0xff0000ff)})) + '
      'Border.all(BorderSide(color: ${const Color(0xff00ff00)}))',
    );
    expect(
      ShapeBorder.lerp(b2 + b1, b1 + b2, 0.20).toString(),
      'Border.all(BorderSide(color: ${const Color(0xff0033cc)})) + '
      'Border.all(BorderSide(color: ${const Color(0xff00cc33)}))',
    );
    expect(
      ShapeBorder.lerp(b2 + b1, b1 + b2, 1 / 3).toString(),
      'Border.all(BorderSide(color: ${const Color(0xff0055aa)})) + '
      'Border.all(BorderSide(color: ${const Color(0xff00aa55)}))',
    );
    expect(
      ShapeBorder.lerp(b2 + b1, b1 + b2, 1.0).toString(),
      'Border.all(BorderSide(color: ${const Color(0xff00ff00)})) + '
      'Border.all(BorderSide(color: ${const Color(0xff0000ff)}))',
    );
    expect((b1 + b2).dimensions, const EdgeInsets.all(2.0));
    const Rect rect = Rect.fromLTRB(11.0, 15.0, 299.0, 175.0);
    expect(
      (Canvas canvas) => (b1 + b2).paint(canvas, rect),
      paints
        ..rect(rect: rect.deflate(0.5), color: b2.top.color)
        ..rect(rect: rect.deflate(1.5), color: b1.top.color),
    );
    expect((b1 + b2 + b1).dimensions, const EdgeInsets.all(3.0));
    expect(
      (Canvas canvas) => (b1 + b2 + b1).paint(canvas, rect),
      paints
        ..rect(rect: rect.deflate(0.5), color: b1.top.color)
        ..rect(rect: rect.deflate(1.5), color: b2.top.color)
        ..rect(rect: rect.deflate(2.5), color: b1.top.color),
    );
  });

  test('Compound borders', () {
    const BorderSide side1 = BorderSide(color: Color(0xFF00FF00));
    const BorderSide side2 = BorderSide(color: Color(0xFF0000FF));
    const BorderDirectional b1 = BorderDirectional(
      top: side1,
      start: side1,
      end: side1,
      bottom: side1,
    );
    const BorderDirectional b2 = BorderDirectional(
      top: side2,
      start: side2,
      end: side2,
      bottom: side2,
    );
    expect(
      (b1 + b2).toString(),
      'BorderDirectional(top: BorderSide(color: ${const Color(0xff00ff00)}), start: BorderSide(color: ${const Color(0xff00ff00)}), end: BorderSide(color: ${const Color(0xff00ff00)}), bottom: BorderSide(color: ${const Color(0xff00ff00)})) + '
      'BorderDirectional(top: BorderSide(color: ${const Color(0xff0000ff)}), start: BorderSide(color: ${const Color(0xff0000ff)}), end: BorderSide(color: ${const Color(0xff0000ff)}), bottom: BorderSide(color: ${const Color(0xff0000ff)}))',
    );
    expect(
      (b1 + (b2 + b2)).toString(),
      'BorderDirectional(top: BorderSide(color: ${const Color(0xff00ff00)}), start: BorderSide(color: ${const Color(0xff00ff00)}), end: BorderSide(color: ${const Color(0xff00ff00)}), bottom: BorderSide(color: ${const Color(0xff00ff00)})) + '
      'BorderDirectional(top: BorderSide(color: ${const Color(0xff0000ff)}, width: 2.0), start: BorderSide(color: ${const Color(0xff0000ff)}, width: 2.0), end: BorderSide(color: ${const Color(0xff0000ff)}, width: 2.0), bottom: BorderSide(color: ${const Color(0xff0000ff)}, width: 2.0))',
    );
    expect(
      ((b1 + b2) + b2).toString(),
      'BorderDirectional(top: BorderSide(color: ${const Color(0xff00ff00)}), start: BorderSide(color: ${const Color(0xff00ff00)}), end: BorderSide(color: ${const Color(0xff00ff00)}), bottom: BorderSide(color: ${const Color(0xff00ff00)})) + '
      'BorderDirectional(top: BorderSide(color: ${const Color(0xff0000ff)}, width: 2.0), start: BorderSide(color: ${const Color(0xff0000ff)}, width: 2.0), end: BorderSide(color: ${const Color(0xff0000ff)}, width: 2.0), bottom: BorderSide(color: ${const Color(0xff0000ff)}, width: 2.0))',
    );
    expect((b1 + b2) + b2, b1 + (b2 + b2));
    expect(
      (b1 + b2).scale(3.0).toString(),
      'BorderDirectional(top: BorderSide(color: ${const Color(0xff00ff00)}, width: 3.0), start: BorderSide(color: ${const Color(0xff00ff00)}, width: 3.0), end: BorderSide(color: ${const Color(0xff00ff00)}, width: 3.0), bottom: BorderSide(color: ${const Color(0xff00ff00)}, width: 3.0)) + '
      'BorderDirectional(top: BorderSide(color: ${const Color(0xff0000ff)}, width: 3.0), start: BorderSide(color: ${const Color(0xff0000ff)}, width: 3.0), end: BorderSide(color: ${const Color(0xff0000ff)}, width: 3.0), bottom: BorderSide(color: ${const Color(0xff0000ff)}, width: 3.0))',
    );
    expect(
      (b1 + b2).scale(0.0).toString(),
      'BorderDirectional(top: BorderSide(color: ${const Color(0xff00ff00)}, width: 0.0, style: none), start: BorderSide(color: ${const Color(0xff00ff00)}, width: 0.0, style: none), end: BorderSide(color: ${const Color(0xff00ff00)}, width: 0.0, style: none), bottom: BorderSide(color: ${const Color(0xff00ff00)}, width: 0.0, style: none)) + '
      'BorderDirectional(top: BorderSide(color: ${const Color(0xff0000ff)}, width: 0.0, style: none), start: BorderSide(color: ${const Color(0xff0000ff)}, width: 0.0, style: none), end: BorderSide(color: ${const Color(0xff0000ff)}, width: 0.0, style: none), bottom: BorderSide(color: ${const Color(0xff0000ff)}, width: 0.0, style: none))',
    );
    expect(
      ShapeBorder.lerp(b2 + b1, b1 + b2, 0.0).toString(),
      'BorderDirectional(top: BorderSide(color: ${const Color(0xff0000ff)}), start: BorderSide(color: ${const Color(0xff0000ff)}), end: BorderSide(color: ${const Color(0xff0000ff)}), bottom: BorderSide(color: ${const Color(0xff0000ff)})) + '
      'BorderDirectional(top: BorderSide(color: ${const Color(0xff00ff00)}), start: BorderSide(color: ${const Color(0xff00ff00)}), end: BorderSide(color: ${const Color(0xff00ff00)}), bottom: BorderSide(color: ${const Color(0xff00ff00)}))',
    );
    expect(
      ShapeBorder.lerp(b2 + b1, b1 + b2, 0.20).toString(),
      'BorderDirectional(top: BorderSide(color: ${const Color(0xff0033cc)}), start: BorderSide(color: ${const Color(0xff0033cc)}), end: BorderSide(color: ${const Color(0xff0033cc)}), bottom: BorderSide(color: ${const Color(0xff0033cc)})) + '
      'BorderDirectional(top: BorderSide(color: ${const Color(0xff00cc33)}), start: BorderSide(color: ${const Color(0xff00cc33)}), end: BorderSide(color: ${const Color(0xff00cc33)}), bottom: BorderSide(color: ${const Color(0xff00cc33)}))',
    );
    expect(
      ShapeBorder.lerp(b2 + b1, b1 + b2, 1 / 3).toString(),
      'BorderDirectional(top: BorderSide(color: ${const Color(0xff0055aa)}), start: BorderSide(color: ${const Color(0xff0055aa)}), end: BorderSide(color: ${const Color(0xff0055aa)}), bottom: BorderSide(color: ${const Color(0xff0055aa)})) + '
      'BorderDirectional(top: BorderSide(color: ${const Color(0xff00aa55)}), start: BorderSide(color: ${const Color(0xff00aa55)}), end: BorderSide(color: ${const Color(0xff00aa55)}), bottom: BorderSide(color: ${const Color(0xff00aa55)}))',
    );
    expect(
      ShapeBorder.lerp(b2 + b1, b1 + b2, 1.0).toString(),
      'BorderDirectional(top: BorderSide(color: ${const Color(0xff00ff00)}), start: BorderSide(color: ${const Color(0xff00ff00)}), end: BorderSide(color: ${const Color(0xff00ff00)}), bottom: BorderSide(color: ${const Color(0xff00ff00)})) + '
      'BorderDirectional(top: BorderSide(color: ${const Color(0xff0000ff)}), start: BorderSide(color: ${const Color(0xff0000ff)}), end: BorderSide(color: ${const Color(0xff0000ff)}), bottom: BorderSide(color: ${const Color(0xff0000ff)}))',
    );
    expect((b1 + b2).dimensions, const EdgeInsetsDirectional.fromSTEB(2.0, 2.0, 2.0, 2.0));
    const Rect rect = Rect.fromLTRB(11.0, 15.0, 299.0, 175.0);
    expect(
      (Canvas canvas) => (b1 + b2).paint(canvas, rect, textDirection: TextDirection.rtl),
      paints
        ..rect(rect: rect.deflate(0.5), color: b2.top.color)
        ..rect(rect: rect.deflate(1.5), color: b1.top.color),
    );
    expect((b1 + b2 + b1).dimensions, const EdgeInsetsDirectional.fromSTEB(3.0, 3.0, 3.0, 3.0));
    expect(
      (Canvas canvas) => (b1 + b2 + b1).paint(canvas, rect, textDirection: TextDirection.rtl),
      paints
        ..rect(rect: rect.deflate(0.5), color: b1.top.color)
        ..rect(rect: rect.deflate(1.5), color: b2.top.color)
        ..rect(rect: rect.deflate(2.5), color: b1.top.color),
    );
  });
}
