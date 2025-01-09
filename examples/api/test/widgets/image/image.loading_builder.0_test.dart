// Copyright 2014 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

import 'dart:io';

import 'package:flutter/material.dart';
import 'package:flutter_api_samples/widgets/image/image.loading_builder.0.dart' as example;
import 'package:flutter_test/flutter_test.dart';

void main() {
  // The app being tested loads images via HTTP which the test
  // framework defeats by default.
  setUpAll(() {
    HttpOverrides.global = null;
  });

  testWidgets('The loading builder returns the child when there is no loading progress', (
    WidgetTester tester,
  ) async {
    await tester.pumpWidget(const example.LoadingBuilderExampleApp());
    await tester.pumpAndSettle();

    final Image image = tester.widget<Image>(find.byType(Image));
    final ImageLoadingBuilder loadingBuilder = image.loadingBuilder!;
    final BuildContext context = tester.element(find.byType(Image));

    const SizedBox child = SizedBox(key: Key('child'));

    await tester.pumpWidget(loadingBuilder(context, child, null));

    expect(find.byWidget(child), findsOne);
    expect(find.byType(CircularProgressIndicator), findsNothing);
  });

  testWidgets('The loading builder returns a circular progress indicator when loading', (
    WidgetTester tester,
  ) async {
    await tester.pumpWidget(const example.LoadingBuilderExampleApp());
    await tester.pumpAndSettle();

    final Image image = tester.widget<Image>(find.byType(Image));
    final ImageLoadingBuilder loadingBuilder = image.loadingBuilder!;
    final BuildContext context = tester.element(find.byType(Image));

    const SizedBox child = SizedBox(key: Key('child'));

    await tester.pumpWidget(
      MaterialApp(
        home: loadingBuilder(
          context,
          child,
          const ImageChunkEvent(cumulativeBytesLoaded: 1, expectedTotalBytes: 10),
        ),
      ),
    );

    expect(find.byWidget(child), findsNothing);
    expect(find.byType(CircularProgressIndicator), findsOne);
  });
}
