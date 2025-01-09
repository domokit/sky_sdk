// Copyright 2014 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

import 'package:flutter_api_samples/material/dialog/dialog.0.dart' as example;
import 'package:flutter_test/flutter_test.dart';

void main() {
  testWidgets('Show Dialog', (WidgetTester tester) async {
    const String dialogText = 'This is a typical dialog.';

    await tester.pumpWidget(const example.DialogExampleApp());

    expect(find.text(dialogText), findsNothing);

    await tester.tap(find.text('Show Dialog'));
    await tester.pumpAndSettle();
    expect(find.text(dialogText), findsOneWidget);

    await tester.tap(find.text('Close'));
    await tester.pumpAndSettle();
    expect(find.text(dialogText), findsNothing);
  });

  testWidgets('Show Dialog.fullscreen', (WidgetTester tester) async {
    const String dialogText = 'This is a fullscreen dialog.';

    await tester.pumpWidget(const example.DialogExampleApp());

    expect(find.text(dialogText), findsNothing);

    await tester.tap(find.text('Show Fullscreen Dialog'));
    await tester.pumpAndSettle();
    expect(find.text(dialogText), findsOneWidget);

    await tester.tap(find.text('Close'));
    await tester.pumpAndSettle();
    expect(find.text(dialogText), findsNothing);
  });
}
