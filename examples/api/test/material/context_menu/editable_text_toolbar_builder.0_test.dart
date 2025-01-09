// Copyright 2014 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

import 'package:flutter/cupertino.dart';
import 'package:flutter/foundation.dart';
import 'package:flutter/material.dart';
import 'package:flutter/services.dart';
import 'package:flutter_api_samples/material/context_menu/editable_text_toolbar_builder.0.dart'
    as example;
import 'package:flutter_test/flutter_test.dart';

void main() {
  testWidgets('showing and hiding the context menu in TextField with custom buttons', (
    WidgetTester tester,
  ) async {
    await tester.pumpWidget(const example.EditableTextToolbarBuilderExampleApp());

    expect(BrowserContextMenu.enabled, !kIsWeb);

    await tester.tap(find.byType(EditableText));
    await tester.pump();

    expect(find.byType(AdaptiveTextSelectionToolbar), findsNothing);

    // Long pressing the field shows the default context menu but with custom
    // buttons.
    await tester.longPress(find.byType(EditableText));
    await tester.pumpAndSettle();

    expect(find.byType(AdaptiveTextSelectionToolbar), findsOneWidget);
    expect(find.byType(CupertinoButton), findsAtLeastNWidgets(1));

    // Tap to dismiss.
    await tester.tapAt(tester.getTopLeft(find.byType(EditableText)));
    await tester.pumpAndSettle();

    expect(find.byType(AdaptiveTextSelectionToolbar), findsNothing);
    expect(find.byType(CupertinoButton), findsNothing);
  });
}
