// Copyright 2014 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

import 'package:flutter/foundation.dart';
import 'package:flutter/gestures.dart';
import 'package:flutter/material.dart';
import 'package:flutter/services.dart';
import 'package:flutter_api_samples/material/context_menu/context_menu_controller.0.dart'
    as example;
import 'package:flutter_test/flutter_test.dart';

void main() {
  testWidgets('showing and hiding the custom context menu in the whole app', (
    WidgetTester tester,
  ) async {
    await tester.pumpWidget(const example.ContextMenuControllerExampleApp());

    expect(BrowserContextMenu.enabled, !kIsWeb);

    expect(find.byType(AdaptiveTextSelectionToolbar), findsNothing);

    // Right clicking the middle of the app shows the custom context menu.
    final Offset center = tester.getCenter(find.byType(Scaffold));
    final TestGesture gesture = await tester.startGesture(
      center,
      kind: PointerDeviceKind.mouse,
      buttons: kSecondaryMouseButton,
    );
    await tester.pump();
    await gesture.up();
    await tester.pumpAndSettle();

    expect(find.byType(AdaptiveTextSelectionToolbar), findsOneWidget);
    expect(find.text('Print'), findsOneWidget);

    // Tap to dismiss.
    await tester.tapAt(center);
    await tester.pumpAndSettle();

    expect(find.byType(AdaptiveTextSelectionToolbar), findsNothing);

    // Long pressing also shows the custom context menu.
    await tester.longPressAt(center);

    expect(find.byType(AdaptiveTextSelectionToolbar), findsOneWidget);
    expect(find.text('Print'), findsOneWidget);
  });
}
