// Copyright 2014 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

import 'package:flutter/material.dart';
import 'package:flutter_test/flutter_test.dart';

void main() {
  test('Can dispose ScrollPosition when hasPixels is false', () {
    final ScrollPosition position = ScrollPositionWithSingleContext(
      initialPixels: null,
      keepScrollOffset: false,
      physics: const AlwaysScrollableScrollPhysics(),
      context: ScrollableState(),
    );

    expect(position.hasPixels, false);
    position.dispose(); // Should not throw/assert.
  });

  testWidgets('scrollable in hidden overlay does not crash when unhidden', (
    WidgetTester tester,
  ) async {
    // Regression test for https://github.com/flutter/flutter/issues/44269.
    final TabController controller = TabController(vsync: const TestVSync(), length: 1);
    addTearDown(controller.dispose);

    final OverlayEntry entry1 = OverlayEntry(
      maintainState: true,
      opaque: true,
      builder: (BuildContext context) {
        return TabBar(
          isScrollable: true,
          controller: controller,
          tabs: const <Tab>[Tab(text: 'Main')],
        );
      },
    );
    addTearDown(() {
      entry1.remove();
      entry1.dispose();
    });

    final OverlayEntry entry2 = OverlayEntry(
      maintainState: true,
      opaque: true,
      builder: (BuildContext context) {
        return const Text('number2');
      },
    );
    addTearDown(() {
      entry2.dispose();
    });

    await tester.pumpWidget(
      MaterialApp(home: Material(child: Overlay(initialEntries: <OverlayEntry>[entry1, entry2]))),
    );

    entry2.remove();
    await tester.pump();
    expect(tester.takeException(), isNull);
  });
}
