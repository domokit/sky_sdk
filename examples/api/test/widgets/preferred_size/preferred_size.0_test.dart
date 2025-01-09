// Copyright 2014 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

import 'package:flutter/material.dart';
import 'package:flutter_api_samples/widgets/preferred_size/preferred_size.0.dart' as example;
import 'package:flutter_test/flutter_test.dart';

void main() {
  testWidgets('PreferredSize determines the height of AppBarContent', (WidgetTester tester) async {
    await tester.pumpWidget(const example.PreferredSizeExampleApp());

    final PreferredSize preferredSize = tester.widget(
      find.ancestor(of: find.byType(example.AppBarContent), matching: find.byType(PreferredSize)),
    );

    final RenderBox appBarContent =
        tester.renderObject(find.byType(example.AppBarContent)) as RenderBox;

    expect(preferredSize.preferredSize.height, equals(appBarContent.size.height));
  });
}
