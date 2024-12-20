// Copyright 2014 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

import 'dart:io';

import 'package:flutter/material.dart';
import 'package:flutter_api_samples/widgets/basic/fitted_box.0.dart' as example;
import 'package:flutter_test/flutter_test.dart';

void main() {
  setUpAll(() {
    HttpOverrides.global = null;
  });

  testWidgets('FittedBox scales the image to fill the parent container', (
    WidgetTester tester,
  ) async {
    await tester.pumpWidget(const example.FittedBoxApp());

    final Size containerSize = tester.getSize(find.byType(Container));
    expect(containerSize, const Size(300, 400));

    // FittedBox should scale the image to fill the parent container.
    final FittedBox fittedBox = tester.widget(find.byType(FittedBox));
    expect(fittedBox.fit, BoxFit.fill);
  });
}
