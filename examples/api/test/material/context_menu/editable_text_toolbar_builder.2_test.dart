// Copyright 2014 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

import 'package:flutter/cupertino.dart';
import 'package:flutter/foundation.dart';
import 'package:flutter/material.dart';
import 'package:flutter/services.dart';
import 'package:flutter_api_samples/material/context_menu/editable_text_toolbar_builder.2.dart'
    as example;
import 'package:flutter_test/flutter_test.dart';

void main() {
  testWidgets('showing and hiding the context menu in TextField with a custom toolbar', (
    WidgetTester tester,
  ) async {
    await tester.pumpWidget(const example.EditableTextToolbarBuilderExampleApp());

    expect(BrowserContextMenu.enabled, !kIsWeb);

    await tester.tap(find.byType(EditableText));
    await tester.pump();

    expect(find.byType(AdaptiveTextSelectionToolbar), findsNothing);

    // Long pressing the field shows the custom context menu.
    await tester.longPress(find.byType(EditableText));
    await tester.pumpAndSettle();

    expect(find.byType(AdaptiveTextSelectionToolbar), findsNothing);

    // The buttons use the default widgets but with custom labels.
    switch (defaultTargetPlatform) {
      case TargetPlatform.iOS:
        expect(find.byType(CupertinoTextSelectionToolbarButton), findsAtLeastNWidgets(1));
      case TargetPlatform.android:
      case TargetPlatform.fuchsia:
        expect(find.byType(TextSelectionToolbarTextButton), findsAtLeastNWidgets(1));
      case TargetPlatform.linux:
      case TargetPlatform.windows:
        expect(find.byType(DesktopTextSelectionToolbarButton), findsAtLeastNWidgets(1));
      case TargetPlatform.macOS:
        expect(find.byType(CupertinoDesktopTextSelectionToolbarButton), findsAtLeastNWidgets(1));
    }
    expect(find.text('Copy'), findsNothing);
    expect(find.text('Cut'), findsNothing);
    expect(find.text('Paste'), findsNothing);
    expect(find.text('Select all'), findsNothing);

    // Tap to dismiss.
    await tester.tapAt(tester.getTopLeft(find.byType(EditableText)));
    await tester.pumpAndSettle();

    expect(find.byType(AdaptiveTextSelectionToolbar), findsNothing);
    expect(find.byType(CupertinoTextSelectionToolbarButton), findsNothing);
    expect(find.byType(TextSelectionToolbarTextButton), findsNothing);
    expect(find.byType(DesktopTextSelectionToolbarButton), findsNothing);
    expect(find.byType(CupertinoDesktopTextSelectionToolbarButton), findsNothing);
    expect(find.text('Copy'), findsNothing);
    expect(find.text('Cut'), findsNothing);
    expect(find.text('Paste'), findsNothing);
    expect(find.text('Select all'), findsNothing);
  });
}
