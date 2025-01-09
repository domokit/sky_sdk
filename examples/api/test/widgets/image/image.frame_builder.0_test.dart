// Copyright 2014 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

import 'dart:io';

import 'package:flutter/material.dart';
import 'package:flutter_api_samples/widgets/image/image.frame_builder.0.dart' as example;
import 'package:flutter_test/flutter_test.dart';

void main() {
  // The app being tested loads images via HTTP which the test
  // framework defeats by default.
  setUpAll(() {
    HttpOverrides.global = null;
  });

  testWidgets('The frame builder returns an AnimatedOpacity when not synchronously loaded', (
    WidgetTester tester,
  ) async {
    await tester.pumpWidget(const example.FrameBuilderExampleApp());
    await tester.pumpAndSettle();

    final Image image = tester.widget<Image>(find.byType(Image));
    final ImageFrameBuilder frameBuilder = image.frameBuilder!;
    final BuildContext context = tester.element(find.byType(Image));

    const Key key = Key('child');

    expect(
      frameBuilder(context, const SizedBox(key: key), null, false),
      isA<AnimatedOpacity>().having((AnimatedOpacity opacity) => opacity.child!.key, 'key', key),
    );
  });

  testWidgets('The frame builder returns the child when synchronously loaded', (
    WidgetTester tester,
  ) async {
    await tester.pumpWidget(const example.FrameBuilderExampleApp());
    await tester.pumpAndSettle();

    final Image image = tester.widget<Image>(find.byType(Image));
    final ImageFrameBuilder frameBuilder = image.frameBuilder!;
    final BuildContext context = tester.element(find.byType(Image));

    const Key key = Key('child');

    expect(
      frameBuilder(context, const SizedBox(key: key), null, true),
      isA<SizedBox>().having((SizedBox widget) => widget.key, 'key', key),
    );
  });
}
