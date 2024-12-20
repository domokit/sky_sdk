// Copyright 2014 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

import 'package:flutter/widgets.dart' as widgets show Container, Size, runApp;
import 'package:flutter_test/flutter_test.dart';
import 'package:integration_test/integration_test.dart';
import 'package:integration_ui/resize.dart' as app;

void main() {
  IntegrationTestWidgetsFlutterBinding.ensureInitialized();

  group('end-to-end test', () {
    testWidgets('Use button to resize window', timeout: const Timeout(Duration(seconds: 5)), (
      WidgetTester tester,
    ) async {
      const app.ResizeApp resizeApp = app.ResizeApp();

      widgets.runApp(resizeApp);
      await tester.pumpAndSettle();

      final Finder fab = find.byKey(app.ResizeApp.extendedFab);
      expect(fab, findsOneWidget);

      final Finder root = find.byWidget(resizeApp);
      final widgets.Size sizeBefore = tester.getSize(root);

      await tester.tap(fab);
      await tester.pumpAndSettle();

      final widgets.Size sizeAfter = tester.getSize(root);
      expect(sizeAfter.width, equals(sizeBefore.width + app.ResizeApp.resizeBy));
      expect(sizeAfter.height, equals(sizeBefore.height + app.ResizeApp.resizeBy));

      final Finder widthLabel = find.byKey(app.ResizeApp.widthLabel);
      expect(widthLabel, findsOneWidget);
      expect(find.text('width: ${sizeAfter.width}'), findsOneWidget);

      final Finder heightLabel = find.byKey(app.ResizeApp.heightLabel);
      expect(heightLabel, findsOneWidget);
      expect(find.text('height: ${sizeAfter.height}'), findsOneWidget);
    });
  });

  testWidgets(
    'resize window after calling runApp twice, the second with no content',
    timeout: const Timeout(Duration(seconds: 5)),
    (WidgetTester tester) async {
      const app.ResizeApp root = app.ResizeApp();
      widgets.runApp(root);
      widgets.runApp(widgets.Container());

      await tester.pumpAndSettle();

      const widgets.Size expectedSize = widgets.Size(100, 100);
      await app.ResizeApp.resize(expectedSize);
    },
  );
}
