// Copyright 2014 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

import 'package:flutter/foundation.dart';
import 'package:flutter/gestures.dart';
import 'package:flutter/material.dart';
import 'package:flutter/services.dart';
import 'package:flutter_api_samples/material/context_menu/editable_text_toolbar_builder.1.dart'
    as example;
import 'package:flutter_test/flutter_test.dart';

void main() {
  testWidgets('showing and hiding the custom context menu in TextField with a specific selection', (
    WidgetTester tester,
  ) async {
    await tester.pumpWidget(const example.EditableTextToolbarBuilderExampleApp());

    expect(BrowserContextMenu.enabled, !kIsWeb);

    expect(find.byType(AdaptiveTextSelectionToolbar), findsNothing);

    // Right clicking the Text in the TextField shows the custom context menu,
    // but no email button since no email address is selected.
    TestGesture gesture = await tester.startGesture(
      tester.getTopLeft(find.text(example.text)),
      kind: PointerDeviceKind.mouse,
      buttons: kSecondaryMouseButton,
    );
    await tester.pump();
    await gesture.up();
    await tester.pumpAndSettle();

    expect(find.byType(AdaptiveTextSelectionToolbar), findsOneWidget);
    expect(find.text('Send email'), findsNothing);

    // Tap to dismiss.
    await tester.tapAt(tester.getTopLeft(find.byType(EditableText)));
    await tester.pumpAndSettle();

    expect(find.byType(AdaptiveTextSelectionToolbar), findsNothing);

    // Select the email address.
    final EditableTextState state = tester.state<EditableTextState>(find.byType(EditableText));
    state.updateEditingValue(
      state.textEditingValue.copyWith(
        selection: TextSelection(
          baseOffset: example.text.indexOf(example.emailAddress),
          extentOffset: example.text.length,
        ),
      ),
    );
    await tester.pump();

    // Right clicking the Text in the TextField shows the custom context menu
    // with the email button.
    gesture = await tester.startGesture(
      tester.getCenter(find.text(example.text)),
      kind: PointerDeviceKind.mouse,
      buttons: kSecondaryMouseButton,
    );
    await tester.pump();
    await gesture.up();
    await tester.pumpAndSettle();

    expect(find.byType(AdaptiveTextSelectionToolbar), findsOneWidget);
    expect(find.text('Send email'), findsOneWidget);

    // Tap to dismiss.
    await tester.tapAt(tester.getTopLeft(find.byType(EditableText)));
    await tester.pumpAndSettle();

    expect(find.byType(AdaptiveTextSelectionToolbar), findsNothing);
  });
}
