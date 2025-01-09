// Copyright 2014 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

import 'package:flutter/material.dart';
import 'package:flutter_test/flutter_test.dart';

void main() {
  testWidgets('GridTile control test', (WidgetTester tester) async {
    final Key headerKey = UniqueKey();
    final Key footerKey = UniqueKey();

    await tester.pumpWidget(
      MaterialApp(
        home: GridTile(
          header: GridTileBar(
            key: headerKey,
            leading: const Icon(Icons.thumb_up),
            title: const Text('Header'),
            subtitle: const Text('Subtitle'),
            trailing: const Icon(Icons.thumb_up),
          ),
          footer: GridTileBar(
            key: footerKey,
            title: const Text('Footer'),
            backgroundColor: Colors.black38,
          ),
          child: DecoratedBox(decoration: BoxDecoration(color: Colors.green[500])),
        ),
      ),
    );

    expect(find.text('Header'), findsOneWidget);
    expect(find.text('Footer'), findsOneWidget);

    expect(
      tester.getBottomLeft(find.byKey(headerKey)).dy,
      lessThan(tester.getTopLeft(find.byKey(footerKey)).dy),
    );

    await tester.pumpWidget(
      const Directionality(
        textDirection: TextDirection.ltr,
        child: GridTile(child: Text('Simple')),
      ),
    );

    expect(find.text('Simple'), findsOneWidget);
  });
}
