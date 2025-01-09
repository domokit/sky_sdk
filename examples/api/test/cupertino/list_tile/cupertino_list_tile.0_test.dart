// Copyright 2014 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

import 'package:flutter/cupertino.dart';
import 'package:flutter/material.dart';
import 'package:flutter_api_samples/cupertino/list_tile/cupertino_list_tile.0.dart' as example;
import 'package:flutter_test/flutter_test.dart';

void main() {
  testWidgets('CupertinoListTile respects properties', (WidgetTester tester) async {
    await tester.pumpWidget(const example.CupertinoListTileApp());

    expect(find.text('CupertinoListTile Sample'), findsOne);
    expect(find.byType(CupertinoListTile), findsNWidgets(6));

    // Verify if the CupertinoListTile contains the expected widgets.
    expect(find.byType(FlutterLogo), findsNWidgets(4));
    expect(find.text('One-line with leading widget'), findsOne);
    expect(find.text('One-line with trailing widget'), findsOne);
    expect(find.text('One-line CupertinoListTile'), findsOne);
    expect(find.text('One-line with both widgets'), findsOne);
    expect(find.text('Two-line CupertinoListTile'), findsOne);
    expect(find.text('Here is a subtitle'), findsOne);
    expect(find.text('CupertinoListTile with background color'), findsOne);
    expect(find.byIcon(Icons.more_vert), findsNWidgets(3));
    expect(find.byIcon(Icons.info), findsOne);

    final Finder tileWithBackgroundFinder = find.byKey(
      const Key('CupertinoListTile with background color'),
    );
    expect(
      tester.firstWidget<CupertinoListTile>(tileWithBackgroundFinder).backgroundColor,
      Colors.lightBlue,
    );
  });
}
