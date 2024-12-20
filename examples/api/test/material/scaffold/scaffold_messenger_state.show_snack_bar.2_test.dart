// Copyright 2014 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

import 'package:flutter/material.dart';
import 'package:flutter_api_samples/material/scaffold/scaffold_messenger_state.show_snack_bar.2.dart'
    as example;
import 'package:flutter_test/flutter_test.dart';

void main() {
  testWidgets('ScaffoldMessenger showSnackBar animation can be customized using AnimationStyle', (
    WidgetTester tester,
  ) async {
    await tester.pumpWidget(const example.SnackBarApp());

    // Tap the button to show the SnackBar with default animation style.
    await tester.tap(find.byType(ElevatedButton));
    await tester.pump();
    await tester.pump(const Duration(milliseconds: 125)); // Advance the animation by 125ms.

    expect(tester.getTopLeft(find.text('I am a snack bar.')).dy, closeTo(576.7, 0.1));

    await tester.pump(const Duration(milliseconds: 125)); // Advance the animation by 125ms.

    expect(tester.getTopLeft(find.text('I am a snack bar.')).dy, closeTo(566, 0.1));

    // Tap the close button to dismiss the SnackBar.
    await tester.tap(find.byType(IconButton));
    await tester.pump();
    await tester.pump(const Duration(milliseconds: 250)); // Advance the animation by 250ms.

    expect(tester.getTopLeft(find.text('I am a snack bar.')).dy, closeTo(614, 0.1));

    // Select custom animation style.
    await tester.tap(find.text('Custom'));
    await tester.pumpAndSettle();

    // Tap the button to show the SnackBar with custom animation style.
    await tester.tap(find.byType(ElevatedButton));
    await tester.pump();
    await tester.pump(const Duration(milliseconds: 1500)); // Advance the animation by 125ms.

    expect(tester.getTopLeft(find.text('I am a snack bar.')).dy, closeTo(576.7, 0.1));

    await tester.pump(const Duration(milliseconds: 1500)); // Advance the animation by 125ms.

    expect(tester.getTopLeft(find.text('I am a snack bar.')).dy, closeTo(566, 0.1));

    // Tap the close button to dismiss the SnackBar.
    await tester.tap(find.byType(IconButton));
    await tester.pump();
    await tester.pump(const Duration(seconds: 1)); // Advance the animation by 1sec.

    expect(tester.getTopLeft(find.text('I am a snack bar.')).dy, closeTo(614, 0.1));

    // Select no animation style.
    await tester.tap(find.text('None'));
    await tester.pumpAndSettle();

    // Tap the button to show the SnackBar with no animation style.
    await tester.tap(find.byType(ElevatedButton));
    await tester.pump();

    expect(tester.getTopLeft(find.text('I am a snack bar.')).dy, closeTo(566, 0.1));

    // Tap the close button to dismiss the SnackBar.
    await tester.tap(find.byType(IconButton));
    await tester.pump();

    expect(find.text('I am a snack bar.'), findsNothing);
  });
}
