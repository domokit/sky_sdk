// Copyright 2014 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

import 'package:flutter/gestures.dart';
import 'package:flutter/rendering.dart';
import 'package:flutter_api_samples/widgets/basic/ignore_pointer.0.dart' as example;
import 'package:flutter_test/flutter_test.dart';

void main() {
  testWidgets('IgnorePointer ignores pointer on the ElevatedButton', (WidgetTester tester) async {
    const String clickButtonText = 'Click me!';

    await tester.pumpWidget(const example.IgnorePointerApp());

    // The ElevatedButton is clickable.
    expect(find.text('Ignoring: false'), findsOneWidget);

    final TestGesture gesture = await tester.createGesture(
      kind: PointerDeviceKind.mouse,
      pointer: 1,
    );
    await gesture.addPointer(location: tester.getCenter(find.text(clickButtonText)));
    // On hovering the ElevatedButton, the cursor should be SystemMouseCursors.click.
    expect(
      RendererBinding.instance.mouseTracker.debugDeviceActiveCursor(1),
      SystemMouseCursors.click,
    );

    // Tap to set ignoring pointer to true.
    await tester.tap(find.text('Set ignoring to true'));
    await tester.pump();

    // The ElevatedButton is not clickable so the cursor should be SystemMouseCursors.basic.
    expect(
      RendererBinding.instance.mouseTracker.debugDeviceActiveCursor(1),
      SystemMouseCursors.basic,
    );
  });
}
