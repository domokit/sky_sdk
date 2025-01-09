// Copyright 2014 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

import 'package:flutter/material.dart';
import 'package:flutter_test/flutter_test.dart';

import '../widgets/gestures.dart';

void main() {
  testWidgets('Tap on center change color', (WidgetTester tester) async {
    await tester.pumpWidget(
      const Directionality(textDirection: TextDirection.ltr, child: GestureDemo()),
    );
    final Finder finder = find.byType(GestureDemo);

    MaterialColor getSwatch() => tester.state<GestureDemoState>(finder).swatch;
    Future<void> tap() async {
      final Offset topLeft = tester.getTopLeft(finder);
      await tester.tapAt(tester.getSize(finder).center(topLeft));
      await tester.pump(const Duration(seconds: 1));
    }

    // initial swatch
    expect(getSwatch(), GestureDemoState.kSwatches[0]);

    // every tap change swatch
    for (int i = 1; i < GestureDemoState.kSwatches.length; i++) {
      await tap();
      expect(getSwatch(), GestureDemoState.kSwatches[i]);
    }

    // tap on last swatch display first swatch
    await tap();
    expect(getSwatch(), GestureDemoState.kSwatches[0]);
  });
}
