// Copyright 2014 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

import 'dart:ui' as ui show lerpDouble;

import 'package:flutter/animation.dart';
import 'package:flutter/rendering.dart';
import 'package:flutter_test/flutter_test.dart';

void main() {
  test('copyWith, ==, hashCode basics', () {
    expect(AnimationStyle(), AnimationStyle().copyWith());
    expect(AnimationStyle().hashCode, AnimationStyle().copyWith().hashCode);
  });

  testWidgets('AnimationStyle.copyWith() overrides all properties', (WidgetTester tester) async {
    final AnimationStyle original = AnimationStyle(
      curve: Curves.ease,
      duration: const Duration(seconds: 1),
      reverseCurve: Curves.ease,
      reverseDuration: const Duration(seconds: 1),
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

  testWidgets('AnimationStyle.merge() fills in null properties', (WidgetTester tester) async {
    final AnimationStyle original = AnimationStyle(
      curve: Curves.ease,
      duration: const Duration(seconds: 1),
      reverseCurve: Curves.ease,
      reverseDuration: const Duration(seconds: 1),
    );
    final AnimationStyle other = AnimationStyle(
      curve: Curves.linear,
      duration: const Duration(seconds: 2),
    );
    final AnimationStyle merged = original.merge(other);
    expect(merged.curve, Curves.linear);
    expect(merged.duration, const Duration(seconds: 2));
    expect(merged.reverseCurve, Curves.ease);
    expect(merged.reverseDuration, const Duration(seconds: 1));
  });

  test('AnimationStyle.lerp identical a,b', () {
    expect(AnimationStyle.lerp(null, null, 0), null);
    final AnimationStyle data = AnimationStyle();
    expect(identical(AnimationStyle.lerp(data, data, 0.5), data), true);
  });

  testWidgets('AnimationStyle.lerp smoothly transitions all values', (WidgetTester tester) async {
    final AnimationStyle a = AnimationStyle(
      curve: Curves.ease,
      duration: const Duration(seconds: 1),
      reverseCurve: Curves.ease,
      reverseDuration: const Duration(seconds: 1),
    );
    final AnimationStyle b = AnimationStyle(
      curve: Curves.linear,
      duration: const Duration(seconds: 2),
      reverseCurve: Curves.linear,
      reverseDuration: const Duration(seconds: 2),
    );
    final int aForward = a.duration!.inMicroseconds;
    final int aReverse = a.reverseDuration!.inMicroseconds;
    final int bForward = b.duration!.inMicroseconds;
    final int bReverse = b.reverseDuration!.inMicroseconds;

    expect(AnimationStyle.lerp(a, b, 0), a);
    expect(AnimationStyle.lerp(a, b, 1.0), b);

    const int styleSteps = 5;
    const int curveSteps = 5;
    for (int styleStep = 0; styleStep < styleSteps; styleStep += 1) {
      final double styleTransition = (styleStep + 1) / (styleSteps + 1);
      final AnimationStyle? lerpedStyle = AnimationStyle.lerp(a, b, styleTransition);

      expect(
        lerpedStyle?.duration?.inMicroseconds,
        ui.lerpDouble(aForward, bForward, styleTransition)?.round(),
      );
      expect(
        lerpedStyle?.reverseDuration?.inMicroseconds,
        ui.lerpDouble(aReverse, bReverse, styleTransition)?.round(),
      );

      for (int curveStep = 0; curveStep < curveSteps; curveStep += 1) {
        final double t = (curveStep + 1) / (curveSteps + 1);
        final double aResult = Curves.ease.transform(t);
        final double bResult = Curves.linear.transform(t);
        final double? lerpResult = lerpedStyle?.curve?.transform(t);

        expect(lerpResult, ui.lerpDouble(aResult, bResult, styleTransition));
      }
    }
  });

  testWidgets('default AnimationStyle debugFillProperties', (WidgetTester tester) async {
    final DiagnosticPropertiesBuilder builder = DiagnosticPropertiesBuilder();

    AnimationStyle().debugFillProperties(builder);

    final List<String> description =
        builder.properties
            .where((DiagnosticsNode node) => !node.isFiltered(DiagnosticLevel.info))
            .map((DiagnosticsNode node) => node.toString())
            .toList();

    expect(description, <String>[]);
  });

  testWidgets('AnimationStyle implements debugFillProperties', (WidgetTester tester) async {
    final DiagnosticPropertiesBuilder builder = DiagnosticPropertiesBuilder();

    AnimationStyle(
      curve: Curves.easeInOut,
      duration: const Duration(seconds: 1),
      reverseCurve: Curves.bounceInOut,
      reverseDuration: const Duration(seconds: 2),
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
