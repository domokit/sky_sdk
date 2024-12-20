// Copyright 2014 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

import 'package:flutter/gestures.dart';
import 'package:flutter/material.dart';
import 'package:flutter/rendering.dart';
import 'package:flutter_api_samples/widgets/basic/absorb_pointer.0.dart' as example;
import 'package:flutter_test/flutter_test.dart';

void main() {
  testWidgets('AbsorbPointer prevents hit testing on its child', (WidgetTester tester) async {
    await tester.pumpWidget(const example.AbsorbPointerApp());

    // Get the center of the stack.
    final Offset center = tester.getCenter(find.byType(Stack).first);

    final TestGesture gesture = await tester.createGesture(
      kind: PointerDeviceKind.mouse,
      pointer: 1,
    );
    // Add the point to the center of the stack where the AbsorbPointer is.
    await gesture.addPointer(location: center);
    expect(
      RendererBinding.instance.mouseTracker.debugDeviceActiveCursor(1),
      SystemMouseCursors.basic,
    );

    // Move the pointer to the left of the stack where the AbsorbPointer is not.
    await gesture.moveTo(center + const Offset(-100, 0));
    expect(
      RendererBinding.instance.mouseTracker.debugDeviceActiveCursor(1),
      SystemMouseCursors.click,
    );
  });
}
