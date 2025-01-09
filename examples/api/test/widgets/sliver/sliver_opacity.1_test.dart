// Copyright 2014 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

import 'package:flutter/material.dart';
import 'package:flutter_api_samples/widgets/sliver/sliver_opacity.1.dart' as example;
import 'package:flutter_test/flutter_test.dart';

void main() {
  testWidgets('SliverOpacity example', (WidgetTester tester) async {
    await tester.pumpWidget(const example.SliverOpacityExampleApp());

    final Finder button = find.byType(FloatingActionButton);
    final Finder opacity = find.byType(SliverOpacity);
    expect((tester.widget(opacity) as SliverOpacity).opacity, 1.0);
    await tester.tap(button);
    await tester.pump();
    expect((tester.widget(opacity) as SliverOpacity).opacity, 0.0);
  });
}
