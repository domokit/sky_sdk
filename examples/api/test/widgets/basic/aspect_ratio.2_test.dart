// Copyright 2014 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

import 'package:flutter/material.dart';
import 'package:flutter_api_samples/widgets/basic/aspect_ratio.2.dart' as example;
import 'package:flutter_test/flutter_test.dart';

void main() {
  testWidgets('AspectRatio applies 0.5 aspect ratio on its child', (WidgetTester tester) async {
    const Size containerSize = Size(100, 100);

    await tester.pumpWidget(const example.AspectRatioApp());

    final Size parentContainer = tester.getSize(find.byType(Container).first);
    expect(parentContainer, containerSize);

    final Size childContainer = tester.getSize(find.byType(Container).last);
    expect(childContainer, Size(containerSize.height * 0.5, containerSize.height));
  });
}
