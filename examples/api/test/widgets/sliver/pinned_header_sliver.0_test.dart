// Copyright 2014 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

import 'package:flutter/material.dart';
import 'package:flutter_api_samples/widgets/sliver/pinned_header_sliver.0.dart' as example;
import 'package:flutter_test/flutter_test.dart';

void main() {
  testWidgets('PinnedHeaderSliver example', (WidgetTester tester) async {
    await tester.pumpWidget(const example.PinnedHeaderSliverApp());

    expect(find.text('PinnedHeaderSliver'), findsOneWidget);

    await tester.tap(find.byType(FloatingActionButton));
    await tester.pumpAndSettle();
    expect(find.text('Alternative Title\nWith Two Lines'), findsOneWidget);
  });
}
