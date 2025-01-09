// Copyright 2014 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

import 'package:flutter/rendering.dart';
import 'package:flutter_test/flutter_test.dart';

void main() {
  test('RelativeRect.==', () {
    const RelativeRect r = RelativeRect.fromLTRB(10.0, 20.0, 30.0, 40.0);
    expect(
      r,
      RelativeRect.fromSize(const Rect.fromLTWH(10.0, 20.0, 0.0, 0.0), const Size(40.0, 60.0)),
    );
  });
  test('RelativeRect.fromDirectional', () {
    final RelativeRect r1 = RelativeRect.fromDirectional(
      textDirection: TextDirection.ltr,
      start: 10.0,
      top: 20.0,
      end: 30.0,
      bottom: 40.0,
    );
    final RelativeRect r2 = RelativeRect.fromDirectional(
      textDirection: TextDirection.rtl,
      start: 10.0,
      top: 20.0,
      end: 30.0,
      bottom: 40.0,
    );
    expect(r1, const RelativeRect.fromLTRB(10.0, 20.0, 30.0, 40.0));
    expect(r2, const RelativeRect.fromLTRB(30.0, 20.0, 10.0, 40.0));
  });
  test('RelativeRect.shift', () {
    const RelativeRect r1 = RelativeRect.fromLTRB(10.0, 20.0, 30.0, 40.0);
    final RelativeRect r2 = r1.shift(const Offset(5.0, 50.0));
    expect(r2, const RelativeRect.fromLTRB(15.0, 70.0, 25.0, -10.0));
  });
  test('RelativeRect.inflate', () {
    const RelativeRect r1 = RelativeRect.fromLTRB(10.0, 20.0, 30.0, 40.0);
    final RelativeRect r2 = r1.inflate(5.0);
    expect(r2, const RelativeRect.fromLTRB(5.0, 15.0, 25.0, 35.0));
  });
  test('RelativeRect.deflate', () {
    const RelativeRect r1 = RelativeRect.fromLTRB(10.0, 20.0, 30.0, 40.0);
    final RelativeRect r2 = r1.deflate(5.0);
    expect(r2, const RelativeRect.fromLTRB(15.0, 25.0, 35.0, 45.0));
  });
  test('RelativeRect.intersect', () {
    const RelativeRect r1 = RelativeRect.fromLTRB(10.0, 20.0, 30.0, 40.0);
    const RelativeRect r2 = RelativeRect.fromLTRB(0.0, 30.0, 60.0, 0.0);
    final RelativeRect r3 = r1.intersect(r2);
    final RelativeRect r4 = r2.intersect(r1);
    expect(r3, r4);
    expect(r3, const RelativeRect.fromLTRB(10.0, 30.0, 60.0, 40.0));
  });
  test('RelativeRect.toRect', () {
    const RelativeRect r1 = RelativeRect.fromLTRB(10.0, 20.0, 30.0, 40.0);
    final Rect r2 = r1.toRect(const Rect.fromLTRB(10.0, 20.0, 90.0, 180.0));
    expect(r2, const Rect.fromLTRB(10.0, 20.0, 50.0, 120.0));
  });
  test('RelativeRect.toSize', () {
    const RelativeRect r1 = RelativeRect.fromLTRB(10.0, 20.0, 30.0, 40.0);
    final Size r2 = r1.toSize(const Size(80.0, 160.0));
    expect(r2, const Size(40.0, 100.0));
  });
  test('RelativeRect.lerp', () {
    const RelativeRect r1 = RelativeRect.fill;
    const RelativeRect r2 = RelativeRect.fromLTRB(10.0, 20.0, 30.0, 40.0);
    final RelativeRect r3 = RelativeRect.lerp(r1, r2, 0.5)!;
    expect(r3, const RelativeRect.fromLTRB(5.0, 10.0, 15.0, 20.0));
  });
  test('RelativeRect.lerp identical a,b', () {
    expect(RelativeRect.lerp(null, null, 0), null);
    const RelativeRect rect = RelativeRect.fill;
    expect(identical(RelativeRect.lerp(rect, rect, 0.5), rect), true);
  });
}
