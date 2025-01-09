// Copyright 2014 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

import 'dart:math' as math;

import 'package:flutter/material.dart';
import 'package:flutter_api_samples/widgets/inherited_notifier/inherited_notifier.0.dart'
    as example;
import 'package:flutter_test/flutter_test.dart';

void main() {
  testWidgets('It rotates the spinners', (WidgetTester tester) async {
    await tester.pumpWidget(const example.InheritedNotifierExampleApp());

    final Iterable<Transform> widgets = tester.widgetList<Transform>(
      find.ancestor(of: find.text('Whee!'), matching: find.byType(Transform)),
    );

    Matcher transformMatcher(Matrix4 matrix) {
      return everyElement(
        isA<Transform>().having((Transform widget) => widget.transform, 'transform', matrix),
      );
    }

    expect(widgets, transformMatcher(Matrix4.identity()));

    await tester.pump(const Duration(seconds: 1));
    expect(widgets, transformMatcher(Matrix4.rotationZ(0.2 * math.pi)));

    await tester.pump(const Duration(seconds: 1));
    expect(widgets, transformMatcher(Matrix4.rotationZ(0.4 * math.pi)));

    await tester.pump(const Duration(seconds: 1));
    expect(widgets, transformMatcher(Matrix4.rotationZ(0.6 * math.pi)));

    await tester.pump(const Duration(seconds: 1));
    expect(widgets, transformMatcher(Matrix4.rotationZ(0.8 * math.pi)));

    await tester.pump(const Duration(seconds: 1));
    expect(
      widgets,
      transformMatcher(
        Matrix4.identity()
          ..storage[0] = -1
          ..storage[5] = -1,
      ),
    );

    await tester.pump(const Duration(seconds: 1));
    expect(widgets, transformMatcher(Matrix4.rotationZ(1.2 * math.pi)));

    await tester.pump(const Duration(seconds: 1));
    expect(widgets, transformMatcher(Matrix4.rotationZ(1.4 * math.pi)));

    await tester.pump(const Duration(seconds: 1));
    expect(widgets, transformMatcher(Matrix4.rotationZ(1.6 * math.pi)));

    await tester.pump(const Duration(seconds: 1));
    expect(widgets, transformMatcher(Matrix4.rotationZ(1.8 * math.pi)));

    await tester.pump(const Duration(seconds: 1));
    expect(widgets, transformMatcher(Matrix4.identity()));
  });
}
