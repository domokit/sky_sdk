// Copyright 2014 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

import 'package:flutter/foundation.dart';
import 'package:flutter/material.dart';
import 'package:flutter_api_samples/widgets/system_context_menu/system_context_menu.0.dart'
    as example;
import 'package:flutter_test/flutter_test.dart';

void main() {
  testWidgets(
    'only shows the system context menu on iOS when MediaQuery says it is supported',
    (WidgetTester tester) async {
      await tester.pumpWidget(
        Builder(
          builder: (BuildContext context) {
            final MediaQueryData mediaQueryData = MediaQuery.of(context);
            return MediaQuery(
              data: mediaQueryData.copyWith(
                // Faking this value, which is usually set to true only on
                // devices running iOS 16+.
                supportsShowingSystemContextMenu: defaultTargetPlatform == TargetPlatform.iOS,
              ),
              child: const example.SystemContextMenuExampleApp(),
            );
          },
        ),
      );

      expect(find.byType(SystemContextMenu), findsNothing);

      // Show the context menu.
      final Finder textFinder = find.byType(EditableText);
      await tester.longPress(textFinder);
      tester.state<EditableTextState>(textFinder).showToolbar();
      await tester.pumpAndSettle();

      switch (defaultTargetPlatform) {
        case TargetPlatform.iOS:
          expect(find.byType(SystemContextMenu), findsOneWidget);
          expect(find.byType(AdaptiveTextSelectionToolbar), findsNothing);
        case TargetPlatform.android:
        case TargetPlatform.fuchsia:
        case TargetPlatform.linux:
        case TargetPlatform.macOS:
        case TargetPlatform.windows:
          expect(find.byType(AdaptiveTextSelectionToolbar), findsOneWidget);
          expect(find.byType(SystemContextMenu), findsNothing);
      }
    },
    variant: TargetPlatformVariant.all(),
    skip: kIsWeb, // [intended]
  );

  testWidgets(
    'does not show the system context menu when not supported',
    (WidgetTester tester) async {
      await tester.pumpWidget(
        // By default, MediaQueryData.supportsShowingSystemContextMenu is false.
        const example.SystemContextMenuExampleApp(),
      );

      expect(find.byType(SystemContextMenu), findsNothing);

      // Show the context menu.
      final Finder textFinder = find.byType(EditableText);
      await tester.longPress(textFinder);
      tester.state<EditableTextState>(textFinder).showToolbar();
      await tester.pumpAndSettle();

      expect(find.byType(AdaptiveTextSelectionToolbar), findsOneWidget);
      expect(find.byType(SystemContextMenu), findsNothing);
    },
    variant: TargetPlatformVariant.all(),
    skip: kIsWeb, // [intended]
  );
}
