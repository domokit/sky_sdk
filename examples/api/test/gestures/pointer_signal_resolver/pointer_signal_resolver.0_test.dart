// Copyright 2014 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

import 'package:flutter/gestures.dart';
import 'package:flutter/material.dart';
import 'package:flutter_api_samples/gestures/pointer_signal_resolver/pointer_signal_resolver.0.dart'
    as example;
import 'package:flutter_test/flutter_test.dart';

void main() {
  ({Color outer, Color inner}) getColors(WidgetTester tester) {
    final DecoratedBox outerBox = tester.widget(find.byType(DecoratedBox).first);
    final DecoratedBox innerBox = tester.widget(find.byType(DecoratedBox).last);
    return (
      outer: (outerBox.decoration as BoxDecoration).color!,
      inner: (innerBox.decoration as BoxDecoration).color!,
    );
  }

  testWidgets('Scrolling on the boxes changes their color', (WidgetTester tester) async {
    await tester.pumpWidget(const example.PointerSignalResolverExampleApp());

    expect(getColors(tester), (outer: const Color(0x3300ff00), inner: const Color(0xffffff00)));

    // Scroll on the outer box.
    final TestPointer pointer = TestPointer(1, PointerDeviceKind.mouse);
    pointer.hover(const Offset(100, 300));
    await tester.sendEventToBinding(pointer.scroll(const Offset(0.0, 100.0)));
    await tester.pump();

    // The outer box changes color.
    ({Color inner, Color outer}) colors = getColors(tester);
    expect(colors.outer, const Color(0x3300ff0d));
    expect(colors.inner, const Color(0xffffff00));

    // Scroll on the inner box.
    pointer.hover(const Offset(400, 300));
    await tester.sendEventToBinding(pointer.scroll(const Offset(0.0, 100.0)));
    await tester.pump();

    // Both boxes change color.
    colors = getColors(tester);
    expect(colors.outer, const Color(0x3300ff1a));
    expect(colors.inner, const Color(0xfff2ff00));

    // Use PointerSignalResolver.
    await tester.tap(find.byType(Switch));
    await tester.pump();

    pointer.hover(const Offset(100, 300));
    await tester.sendEventToBinding(pointer.scroll(const Offset(0.0, 100.0)));
    await tester.pump();

    // The outer box changes color.
    colors = getColors(tester);
    expect(colors.outer, const Color(0x3300ff26));
    expect(colors.inner, const Color(0xfff2ff00));

    // Scroll on the inner box.
    pointer.hover(const Offset(400, 300));
    await tester.sendEventToBinding(pointer.scroll(const Offset(0.0, 100.0)));
    await tester.pump();

    // The inner box changes color.
    colors = getColors(tester);
    expect(colors.outer, const Color(0x3300ff26));
    expect(colors.inner, const Color(0xffe5ff00));
  });
}
