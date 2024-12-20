// Copyright 2014 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

import 'package:flutter/material.dart';
import 'package:flutter_api_samples/widgets/implicit_animations/animated_slide.0.dart' as example;
import 'package:flutter_test/flutter_test.dart';

void main() {
  testWidgets('Translate FlutterLogo using AnimatedSlide', (WidgetTester tester) async {
    await tester.pumpWidget(const example.AnimatedSlideApp());

    Offset logoOffset = tester.getCenter(find.byType(FlutterLogo));
    expect(logoOffset.dx, 376.0);
    expect(logoOffset.dy, 304.0);

    // Test Y axis slider.
    final Offset y = tester.getCenter(find.text('Y'));
    await tester.tapAt(Offset(y.dx, y.dy + 100));
    await tester.pumpAndSettle();

    logoOffset = tester.getCenter(find.byType(FlutterLogo));
    expect(logoOffset.dx.roundToDouble(), 376.0);
    expect(logoOffset.dy.roundToDouble(), 137.0);

    // Test X axis slider.
    final Offset x = tester.getCenter(find.text('X'));
    await tester.tapAt(Offset(x.dx + 100, x.dy));
    await tester.pumpAndSettle();

    logoOffset = tester.getCenter(find.byType(FlutterLogo));
    expect(logoOffset.dx.roundToDouble(), 178.0);
    expect(logoOffset.dy.roundToDouble(), 137.0);
  });
}
