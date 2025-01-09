// Copyright 2014 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

import 'package:flutter/foundation.dart';
import 'package:flutter/gestures.dart';
import 'package:flutter/material.dart';
import 'package:flutter/services.dart';
import 'package:flutter_api_samples/material/context_menu/selectable_region_toolbar_builder.0.dart'
    as example;
import 'package:flutter_test/flutter_test.dart';

void main() {
  testWidgets('showing and hiding the custom context menu on SelectionArea', (
    WidgetTester tester,
  ) async {
    await tester.pumpWidget(const example.SelectableRegionToolbarBuilderExampleApp());

    expect(BrowserContextMenu.enabled, !kIsWeb);

    // Allow the selection overlay geometry to be created.
    await tester.pumpAndSettle();

    expect(find.byType(AdaptiveTextSelectionToolbar), findsNothing);

    // Right clicking the Text in the SelectionArea shows the custom context
    // menu.
    final TestGesture primaryMouseButtonGesture = await tester.createGesture(
      kind: PointerDeviceKind.mouse,
    );
    final TestGesture gesture = await tester.startGesture(
      tester.getCenter(find.text(example.text)),
      kind: PointerDeviceKind.mouse,
      buttons: kSecondaryMouseButton,
    );
    await tester.pump();
    await gesture.up();
    await tester.pumpAndSettle();

    expect(find.byType(AdaptiveTextSelectionToolbar), findsOneWidget);
    expect(find.text('Print'), findsOneWidget);

    // Tap to dismiss.
    await primaryMouseButtonGesture.down(tester.getCenter(find.byType(Scaffold)));
    await tester.pump();
    await primaryMouseButtonGesture.up();
    await tester.pumpAndSettle();

    expect(find.byType(AdaptiveTextSelectionToolbar), findsNothing);
  });
}
