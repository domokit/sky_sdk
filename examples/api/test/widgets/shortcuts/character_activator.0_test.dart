// Copyright 2014 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

import 'package:flutter/material.dart';
import 'package:flutter/services.dart';
import 'package:flutter_api_samples/widgets/shortcuts/character_activator.0.dart' as example;
import 'package:flutter_test/flutter_test.dart';

void main() {
  group('CharacterActivatorExampleApp', () {
    testWidgets('displays correct labels', (WidgetTester tester) async {
      await tester.pumpWidget(const example.CharacterActivatorExampleApp());

      expect(find.text('CharacterActivator Sample'), findsOneWidget);
      expect(find.text('Press question mark for help'), findsOneWidget);
    });

    testWidgets('shows snack bar on question key pressed', (WidgetTester tester) async {
      await tester.pumpWidget(const example.CharacterActivatorExampleApp());

      final Finder snackBarFinder = find.ancestor(
        of: find.text('Keep calm and carry on!'),
        matching: find.byType(SnackBar),
      );

      expect(snackBarFinder, findsNothing);

      await tester.sendKeyEvent(LogicalKeyboardKey.slash, character: '?');

      // Advance the SnackBar entrance animation to the end.
      await tester.pumpAndSettle();

      expect(snackBarFinder, findsOneWidget);

      // Advance time by default SnackBar display duration.
      await tester.pump(const Duration(milliseconds: 4000));

      // Advance the SnackBar exit animation to the end.
      await tester.pumpAndSettle();

      expect(snackBarFinder, findsNothing);
    });
  });
}
