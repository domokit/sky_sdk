// Copyright 2014 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

import 'package:flutter/material.dart';
import 'package:flutter_api_samples/widgets/framework/error_widget.0.dart' as example;
import 'package:flutter_test/flutter_test.dart';

void main() {
  testWidgets('ErrorWidget is displayed in debug mode', (WidgetTester tester) async {
    final ErrorWidgetBuilder oldBuilder = ErrorWidget.builder;
    ErrorWidget.builder = (FlutterErrorDetails details) {
      return ErrorWidget(details.exception);
    };
    await tester.pumpWidget(const example.ErrorWidgetExampleApp());

    expect(find.widgetWithText(AppBar, 'ErrorWidget Sample'), findsOne);

    await tester.tap(find.widgetWithText(TextButton, 'Error Prone'));
    await tester.pump();

    expectLater(tester.takeException(), isInstanceOf<Exception>());

    final Finder errorWidget = find.byType(ErrorWidget);
    expect(errorWidget, findsOneWidget);
    final ErrorWidget error = tester.firstWidget(errorWidget);
    expect(error.message, 'Exception: oh no, an error');

    // Restore the ErrorWidget to conclude the test.
    ErrorWidget.builder = oldBuilder;
  });

  testWidgets('ErrorWidget is displayed in release mode', (WidgetTester tester) async {
    final ErrorWidgetBuilder oldBuilder = ErrorWidget.builder;
    ErrorWidget.builder = (FlutterErrorDetails details) {
      return example.ReleaseModeErrorWidget(details: details);
    };
    await tester.pumpWidget(const example.ErrorWidgetExampleApp());

    expect(find.widgetWithText(AppBar, 'ErrorWidget Sample'), findsOne);

    await tester.tap(find.widgetWithText(TextButton, 'Error Prone'));
    await tester.pump();

    expectLater(tester.takeException(), isInstanceOf<Exception>());

    final Finder errorTextFinder = find.textContaining('Error!\nException: oh no, an error');
    expect(errorTextFinder, findsOneWidget);
    final Text errorText = tester.firstWidget(errorTextFinder);
    expect(errorText.style?.color, Colors.yellow);

    // Restore the ErrorWidget to conclude the test.
    ErrorWidget.builder = oldBuilder;
  });
}
