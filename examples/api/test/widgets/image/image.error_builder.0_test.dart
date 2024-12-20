// Copyright 2014 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

import 'dart:io';

import 'package:flutter/material.dart';
import 'package:flutter_api_samples/widgets/image/image.error_builder.0.dart' as example;
import 'package:flutter_test/flutter_test.dart';

void main() {
  // The app being tested loads images via HTTP which the test
  // framework defeats by default.
  setUpAll(() {
    HttpOverrides.global = null;
  });

  testWidgets('Has nonexistent url', (WidgetTester tester) async {
    await tester.pumpWidget(const example.ErrorBuilderExampleApp());
    await tester.pumpAndSettle();

    final Image image = tester.widget<Image>(find.byType(Image));
    final NetworkImage imageProvider = image.image as NetworkImage;

    expect(imageProvider.url, equals('https://example.does.not.exist/image.jpg'));
  });

  testWidgets('errorBuilder returns text with emoji', (WidgetTester tester) async {
    await tester.pumpWidget(const example.ErrorBuilderExampleApp());
    await tester.pumpAndSettle();

    final Image image = tester.widget<Image>(find.byType(Image));
    final ImageErrorWidgetBuilder errorBuilder = image.errorBuilder!;
    final BuildContext context = tester.element(find.byType(Image));

    expect(
      errorBuilder(context, const HttpException('oops'), StackTrace.empty),
      isA<Text>().having((Text text) => text.data, 'data', equals('😢')),
    );
  });
}
