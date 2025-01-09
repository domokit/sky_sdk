// Copyright 2014 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

import 'package:flutter_api_samples/widgets/sliver/sliver_tree.1.dart' as example;
import 'package:flutter_test/flutter_test.dart';

void main() {
  testWidgets('Can toggle nodes in TreeSliver', (WidgetTester tester) async {
    await tester.pumpWidget(const example.TreeSliverExampleApp());
    expect(find.text('lib'), findsOneWidget);
    expect(find.text('src'), findsNothing);
    // Toggle tree node.
    await tester.tap(find.text('lib'));
    await tester.pumpAndSettle();
    expect(find.text('src'), findsOneWidget);
  });
}
