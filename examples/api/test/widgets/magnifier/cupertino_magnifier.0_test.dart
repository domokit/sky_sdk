// Copyright 2014 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

import 'package:flutter/cupertino.dart';
import 'package:flutter_api_samples/widgets/magnifier/cupertino_magnifier.0.dart' as example;
import 'package:flutter_test/flutter_test.dart';

void main() {
  testWidgets('CupertinoMagnifier must be visible', (WidgetTester tester) async {
    await tester.pumpWidget(const example.CupertinoMagnifierApp());

    final Finder cupertinoMagnifierWidget = find.byType(CupertinoMagnifier);
    expect(cupertinoMagnifierWidget, findsOneWidget);
  });

  testWidgets('CupertinoMagnifier is not using the default value', (WidgetTester tester) async {
    await tester.pumpWidget(const example.CupertinoMagnifierApp());
    expect(
      tester.widget(find.byType(CupertinoMagnifier)),
      isA<CupertinoMagnifier>().having(
        (CupertinoMagnifier t) => t.magnificationScale,
        'magnificationScale',
        1.5,
      ),
    );
  });

  testWidgets('should update CupertinoMagnifier position on drag', (WidgetTester tester) async {
    await tester.pumpWidget(const example.CupertinoMagnifierApp());

    Matcher isPositionedAt(Offset at) {
      return isA<Positioned>().having(
        (Positioned positioned) => Offset(positioned.left!, positioned.top!),
        'magnifier position',
        at,
      );
    }

    // Make sure magnifier is present.
    final Finder positionedWidget = find.byType(Positioned);
    final Widget positionedWidgetInTree = tester.widget(positionedWidget.first);
    final Positioned oldConcretePositioned = positionedWidgetInTree as Positioned;
    final Offset centerOfPositioned = tester.getCenter(positionedWidget.first);

    // Drag the magnifier and confirm its new position is expected.
    const Offset dragDistance = Offset(10, 10);
    final Offset updatedPositioned = Offset(
      oldConcretePositioned.left ?? 0.0 + 10.0,
      oldConcretePositioned.top ?? 0.0 + 10.0,
    );

    await tester.dragFrom(centerOfPositioned, dragDistance);
    await tester.pump();
    expect(positionedWidgetInTree, isPositionedAt(updatedPositioned));
  });
}
