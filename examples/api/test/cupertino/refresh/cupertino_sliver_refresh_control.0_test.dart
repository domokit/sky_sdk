// Copyright 2014 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

import 'package:flutter/cupertino.dart';
import 'package:flutter_api_samples/cupertino/refresh/cupertino_sliver_refresh_control.0.dart'
    as example;
import 'package:flutter_test/flutter_test.dart';

void main() {
  testWidgets('Can pull down to reveal CupertinoSliverRefreshControl', (WidgetTester tester) async {
    await tester.pumpWidget(const example.RefreshControlApp());

    expect(find.byType(CupertinoSliverRefreshControl), findsNothing);
    expect(find.byType(Container), findsNWidgets(3));

    final Finder firstItem = find.byType(Container).first;
    await tester.drag(firstItem, const Offset(0.0, 150.0), touchSlopY: 0);
    await tester.pump();
    expect(find.byType(CupertinoSliverRefreshControl), findsOneWidget);
    await tester.pumpAndSettle();
    expect(find.byType(CupertinoSliverRefreshControl), findsNothing);
    expect(find.byType(Container), findsNWidgets(4));
  });
}
