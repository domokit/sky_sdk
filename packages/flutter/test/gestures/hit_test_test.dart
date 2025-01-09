// Copyright 2014 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

import 'package:flutter/gestures.dart';
import 'package:flutter_test/flutter_test.dart';

void main() {
  test('wrapped HitTestResult gets HitTestEntry added to wrapping HitTestResult', () async {
    final HitTestEntry entry1 = HitTestEntry(_DummyHitTestTarget());
    final HitTestEntry entry2 = HitTestEntry(_DummyHitTestTarget());
    final HitTestEntry entry3 = HitTestEntry(_DummyHitTestTarget());
    final Matrix4 transform = Matrix4.translationValues(40.0, 150.0, 0.0);

    final HitTestResult wrapped = MyHitTestResult()..publicPushTransform(transform);
    wrapped.add(entry1);
    expect(wrapped.path, equals(<HitTestEntry>[entry1]));
    expect(entry1.transform, transform);

    final HitTestResult wrapping = HitTestResult.wrap(wrapped);
    expect(wrapping.path, equals(<HitTestEntry>[entry1]));
    expect(wrapping.path, same(wrapped.path));

    wrapping.add(entry2);
    expect(wrapping.path, equals(<HitTestEntry>[entry1, entry2]));
    expect(wrapped.path, equals(<HitTestEntry>[entry1, entry2]));
    expect(entry2.transform, transform);

    wrapped.add(entry3);
    expect(wrapping.path, equals(<HitTestEntry>[entry1, entry2, entry3]));
    expect(wrapped.path, equals(<HitTestEntry>[entry1, entry2, entry3]));
    expect(entry3.transform, transform);
  });

  test('HitTestResult should correctly push and pop transforms', () {
    Matrix4? currentTransform(HitTestResult targetResult) {
      final HitTestEntry entry = HitTestEntry(_DummyHitTestTarget());
      targetResult.add(entry);
      return entry.transform;
    }

    final MyHitTestResult result = MyHitTestResult();

    final Matrix4 m1 = Matrix4.translationValues(10, 20, 0);
    final Matrix4 m2 = Matrix4.rotationZ(1);
    final Matrix4 m3 = Matrix4.diagonal3Values(1.1, 1.2, 1.0);

    result.publicPushTransform(m1);
    expect(currentTransform(result), equals(m1));

    result.publicPushTransform(m2);
    expect(currentTransform(result), equals(m2 * m1));
    expect(currentTransform(result), equals(m2 * m1)); // Test repeated add

    // The `wrapped` is wrapped at [m1, m2]
    final MyHitTestResult wrapped = MyHitTestResult.wrap(result);
    expect(currentTransform(wrapped), equals(m2 * m1));

    result.publicPushTransform(m3);
    // ignore: avoid_dynamic_calls
    expect(currentTransform(result), equals(m3 * m2 * m1));
    // ignore: avoid_dynamic_calls
    expect(currentTransform(wrapped), equals(m3 * m2 * m1));

    result.publicPopTransform();
    result.publicPopTransform();
    expect(currentTransform(result), equals(m1));

    result.publicPopTransform();
    result.publicPushTransform(m3);
    expect(currentTransform(result), equals(m3));

    result.publicPushTransform(m2);
    expect(currentTransform(result), equals(m2 * m3));
  });

  test('HitTestResult should correctly push and pop offsets', () {
    Matrix4? currentTransform(HitTestResult targetResult) {
      final HitTestEntry entry = HitTestEntry(_DummyHitTestTarget());
      targetResult.add(entry);
      return entry.transform;
    }

    final MyHitTestResult result = MyHitTestResult();

    final Matrix4 m1 = Matrix4.rotationZ(1);
    final Matrix4 m2 = Matrix4.diagonal3Values(1.1, 1.2, 1.0);
    const Offset o3 = Offset(10, 20);
    final Matrix4 m3 = Matrix4.translationValues(o3.dx, o3.dy, 0.0);

    // Test pushing offset as the first element
    result.publicPushOffset(o3);
    expect(currentTransform(result), equals(m3));
    result.publicPopTransform();

    result.publicPushOffset(o3);
    result.publicPushTransform(m1);
    expect(currentTransform(result), equals(m1 * m3));
    expect(currentTransform(result), equals(m1 * m3)); // Test repeated add

    // The `wrapped` is wrapped at [m1, m2]
    final MyHitTestResult wrapped = MyHitTestResult.wrap(result);
    expect(currentTransform(wrapped), equals(m1 * m3));

    result.publicPushTransform(m2);
    // ignore: avoid_dynamic_calls
    expect(currentTransform(result), equals(m2 * m1 * m3));
    // ignore: avoid_dynamic_calls
    expect(currentTransform(wrapped), equals(m2 * m1 * m3));

    result.publicPopTransform();
    result.publicPopTransform();
    result.publicPopTransform();
    expect(currentTransform(result), equals(Matrix4.identity()));

    result.publicPushTransform(m2);
    result.publicPushOffset(o3);
    result.publicPushTransform(m1);

    // ignore: avoid_dynamic_calls
    expect(currentTransform(result), equals(m1 * m3 * m2));

    result.publicPopTransform();

    expect(currentTransform(result), equals(m3 * m2));
  });
}

class _DummyHitTestTarget implements HitTestTarget {
  @override
  void handleEvent(PointerEvent event, HitTestEntry entry) {
    // Nothing to do.
  }
}

class MyHitTestResult extends HitTestResult {
  MyHitTestResult();
  MyHitTestResult.wrap(super.result) : super.wrap();

  void publicPushTransform(Matrix4 transform) => pushTransform(transform);
  void publicPushOffset(Offset offset) => pushOffset(offset);
  void publicPopTransform() => popTransform();
}
