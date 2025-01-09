// Copyright 2014 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

import 'package:flutter/material.dart';
import 'package:flutter_api_samples/material/about/about_list_tile.0.dart' as example;
import 'package:flutter_test/flutter_test.dart';

void main() {
  testWidgets('It should show the about dialog after clicking on the button', (
    WidgetTester tester,
  ) async {
    await tester.pumpWidget(const example.AboutListTileExampleApp());

    expect(find.widgetWithText(AppBar, 'Show About Example'), findsOne);

    await tester.tap(find.widgetWithText(ElevatedButton, 'Show About Example'));
    await tester.pumpAndSettle();

    expect(find.byType(AboutDialog), findsOne);
    expect(find.widgetWithText(AboutDialog, 'Show About Example'), findsOne);
    expect(find.text('August 2019'), findsOne);
    expect(find.byType(FlutterLogo), findsOne);
    expect(find.text('\u{a9} 2014 The Flutter Authors'), findsOne);
    expect(
      find.text(
        "Flutter is Google's UI toolkit for building beautiful, "
        'natively compiled applications for mobile, web, and desktop '
        'from a single codebase. Learn more about Flutter at '
        'https://flutter.dev.',
        findRichText: true,
      ),
      findsOne,
    );
  });

  testWidgets('It should show the about dialog after clicking on about list tile in the drawer', (
    WidgetTester tester,
  ) async {
    await tester.pumpWidget(const example.AboutListTileExampleApp());

    expect(find.widgetWithText(AppBar, 'Show About Example'), findsOne);

    await tester.tap(find.byType(DrawerButton));
    await tester.pumpAndSettle();

    expect(find.byType(Drawer), findsOne);
    expect(find.widgetWithText(AboutListTile, 'About Show About Example'), findsOne);
    expect(find.widgetWithIcon(AboutListTile, Icons.info), findsOne);

    await tester.tap(find.widgetWithIcon(AboutListTile, Icons.info));
    await tester.pumpAndSettle();

    expect(find.byType(AboutDialog), findsOne);
    expect(find.widgetWithText(AboutDialog, 'Show About Example'), findsOne);
    expect(find.text('August 2019'), findsOne);
    expect(find.byType(FlutterLogo), findsOne);
    expect(find.text('\u{a9} 2014 The Flutter Authors'), findsOne);
    expect(
      find.text(
        "Flutter is Google's UI toolkit for building beautiful, "
        'natively compiled applications for mobile, web, and desktop '
        'from a single codebase. Learn more about Flutter at '
        'https://flutter.dev.',
        findRichText: true,
      ),
      findsOne,
    );
  });
}
