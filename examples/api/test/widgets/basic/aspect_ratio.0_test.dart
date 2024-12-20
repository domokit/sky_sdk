// Copyright 2014 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

import 'package:flutter/material.dart';
import 'package:flutter_api_samples/widgets/basic/aspect_ratio.0.dart' as example;
import 'package:flutter_test/flutter_test.dart';

void main() {
  testWidgets('AspectRatio applies 16 / 9 aspect ratio on its child', (WidgetTester tester) async {
    const double height = 100.0;

    await tester.pumpWidget(const example.AspectRatioApp());

    final Size parentContainer = tester.getSize(find.byType(Container).first);
    expect(parentContainer.width, 800.0);
    expect(parentContainer.height, height);

    final Size childContainer = tester.getSize(find.byType(Container).last);
    expect(childContainer.width, height * 16 / 9);
    expect(childContainer.height, height);
  });
}
