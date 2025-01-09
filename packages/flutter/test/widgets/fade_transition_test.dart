// Copyright 2014 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

import 'package:flutter/foundation.dart';
import 'package:flutter/widgets.dart';
import 'package:flutter_test/flutter_test.dart';

void main() {
  testWidgets('FadeTransition', (WidgetTester tester) async {
    final DebugPrintCallback oldPrint = debugPrint;
    final List<String> log = <String>[];
    debugPrint = (String? message, {int? wrapWidth}) {
      log.add(message!);
    };
    debugPrintBuildScope = true;
    final AnimationController controller = AnimationController(
      vsync: const TestVSync(),
      duration: const Duration(seconds: 2),
    );
    addTearDown(controller.dispose);
    await tester.pumpWidget(FadeTransition(opacity: controller, child: const Placeholder()));
    expect(log, hasLength(2));
    expect(log.last, 'buildScope finished');
    await tester.pump();
    expect(log, hasLength(2));
    controller.forward();
    await tester.pumpAndSettle();
    expect(log, hasLength(2));
    debugPrint = oldPrint;
    debugPrintBuildScope = false;
  });

  // Regression test for https://github.com/flutter/flutter/issues/157312
  testWidgets('No exception when calling markNeedsPaint during opacity changes', (
    WidgetTester tester,
  ) async {
    final GlobalKey key = GlobalKey();
    final AnimationController controller = AnimationController(
      vsync: const TestVSync(),
      value: 1,
      duration: const Duration(seconds: 2),
    );
    addTearDown(controller.dispose);
    await tester.pumpWidget(FadeTransition(opacity: controller, child: Placeholder(key: key)));
    controller.value = 0.5;
    key.currentContext?.findRenderObject()?.markNeedsPaint();
    controller.value = 0;
    await tester.pump();
    expect(tester.takeException(), isNull);
  });
}
