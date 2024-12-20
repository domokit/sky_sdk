// Copyright 2014 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

import 'package:flutter/material.dart';
import 'package:flutter_api_samples/widgets/sliver/sliver_resizing_header.0.dart' as example;
import 'package:flutter_test/flutter_test.dart';

void main() {
  testWidgets('SliverResizingHeader example', (WidgetTester tester) async {
    await tester.pumpWidget(const example.SliverResizingHeaderApp());

    final Finder headerMaterial = find.text(
      'SliverResizingHeader\nWith Two Optional\nLines of Text',
    );
    final double initialHeight = tester.getSize(headerMaterial).height;

    await tester.drag(find.byType(CustomScrollView), const Offset(0, -200));
    await tester.pumpAndSettle();
    expect(tester.getSize(headerMaterial).height, lessThan(initialHeight / 2));

    await tester.drag(find.byType(CustomScrollView), const Offset(0, 200));
    await tester.pumpAndSettle();
    expect(tester.getSize(headerMaterial).height, initialHeight);
  });
}
