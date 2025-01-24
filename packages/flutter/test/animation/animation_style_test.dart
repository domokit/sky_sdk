// Copyright 2014 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

import 'package:flutter/animation.dart';
import 'package:flutter/rendering.dart';
import 'package:flutter_test/flutter_test.dart';

void main() {
  test('copyWith, ==, hashCode basics', () {
    expect(const AnimationStyle(), const AnimationStyle().copyWith());
    expect(const AnimationStyle().hashCode, const AnimationStyle().copyWith().hashCode);
  });

  testWidgets('AnimationStyle.copyWith() overrides all properties', (WidgetTester tester) async {
    const AnimationStyle original = AnimationStyle(
      curve: Curves.ease,
      duration: Duration(seconds: 1),
      reverseCurve: Curves.ease,
      reverseDuration: Duration(seconds: 1),
    );
    final AnimationStyle copy = original.copyWith(
      curve: Curves.linear,
      duration: const Duration(seconds: 2),
      reverseCurve: Curves.linear,
      reverseDuration: const Duration(seconds: 2),
    );
    expect(copy.curve, Curves.linear);
    expect(copy.duration, const Duration(seconds: 2));
    expect(copy.reverseCurve, Curves.linear);
    expect(copy.reverseDuration, const Duration(seconds: 2));
  });

  test('AnimationStyle.lerp identical a,b', () {
    expect(AnimationStyle.lerp(null, null, 0), null);
    const AnimationStyle data = AnimationStyle();
    expect(identical(AnimationStyle.lerp(data, data, 0.5), data), true);
  });

  testWidgets('default AnimationStyle debugFillProperties', (WidgetTester tester) async {
    const AnimationStyle a = AnimationStyle(
      curve: Curves.ease,
      duration: Duration(seconds: 1),
      reverseCurve: Curves.ease,
      reverseDuration: Duration(seconds: 1),
    );
    const AnimationStyle b = AnimationStyle(
      curve: Curves.linear,
      duration: Duration(seconds: 2),
      reverseCurve: Curves.linear,
      reverseDuration: Duration(seconds: 2),
    );

    expect(AnimationStyle.lerp(a, b, 0), a);
    expect(AnimationStyle.lerp(a, b, 0.5), b);
    expect(AnimationStyle.lerp(a, b, 1.0), b);
  });

  testWidgets('default AnimationStyle debugFillProperties', (WidgetTester tester) async {
    final DiagnosticPropertiesBuilder builder = DiagnosticPropertiesBuilder();

    const AnimationStyle().debugFillProperties(builder);

    final List<String> description =
        builder.properties
            .where((DiagnosticsNode node) => !node.isFiltered(DiagnosticLevel.info))
            .map((DiagnosticsNode node) => node.toString())
            .toList();

    expect(description, <String>[]);
  });

  testWidgets('AnimationStyle implements debugFillProperties', (WidgetTester tester) async {
    final DiagnosticPropertiesBuilder builder = DiagnosticPropertiesBuilder();

    const AnimationStyle(
      curve: Curves.easeInOut,
      duration: Duration(seconds: 1),
      reverseCurve: Curves.bounceInOut,
      reverseDuration: Duration(seconds: 2),
    ).debugFillProperties(builder);

    final List<String> description =
        builder.properties
            .where((DiagnosticsNode node) => !node.isFiltered(DiagnosticLevel.info))
            .map((DiagnosticsNode node) => node.toString())
            .toList();

    expect(description, <String>[
      'curve: Cubic(0.42, 0.00, 0.58, 1.00)',
      'duration: 0:00:01.000000',
      'reverseCurve: _BounceInOutCurve',
      'reverseDuration: 0:00:02.000000',
    ]);
  });
}
