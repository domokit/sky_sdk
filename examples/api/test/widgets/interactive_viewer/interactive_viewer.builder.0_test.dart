// Copyright 2014 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

import 'package:flutter/widgets.dart';
import 'package:flutter_api_samples/widgets/interactive_viewer/interactive_viewer.builder.0.dart'
    as example;
import 'package:flutter_test/flutter_test.dart';

void main() {
  testWidgets('has correct items on screen', (WidgetTester tester) async {
    await tester.pumpWidget(const example.IVBuilderExampleApp());

    final Finder positionedFinder = find.byType(Positioned);
    final Finder zeroFinder = find.text('0 x 0');
    final Finder nineFinder = find.text('0 x 9');

    expect(positionedFinder, findsNWidgets(35));
    expect(zeroFinder, findsOneWidget);
    expect(nineFinder, findsNothing);

    const Offset firstLocation = Offset(750.0, 100.0);
    final TestGesture gesture = await tester.startGesture(firstLocation);

    const Offset secondLocation = Offset(50.0, 100.0);
    await gesture.moveTo(secondLocation);
    await tester.pump();

    expect(positionedFinder, findsNWidgets(42));
    expect(nineFinder, findsOneWidget);
    expect(zeroFinder, findsNothing);
  });
}
