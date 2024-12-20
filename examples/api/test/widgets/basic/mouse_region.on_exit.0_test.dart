// Copyright 2014 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

import 'package:flutter/gestures.dart';
import 'package:flutter/material.dart';
import 'package:flutter_api_samples/widgets/basic/mouse_region.on_exit.0.dart' as example;
import 'package:flutter_test/flutter_test.dart';

void main() {
  testWidgets('MouseRegion detects mouse hover', (WidgetTester tester) async {
    await tester.pumpWidget(const example.MouseRegionApp());

    Container container = tester.widget<Container>(find.byType(Container));
    expect(container.decoration, const BoxDecoration(color: Colors.blue));

    final TestGesture gesture = await tester.createGesture(kind: PointerDeviceKind.mouse);
    await gesture.addPointer();
    await gesture.moveTo(tester.getCenter(find.byType(Container)));
    await tester.pump();

    container = tester.widget<Container>(find.byType(Container));
    expect(container.decoration, const BoxDecoration(color: Colors.yellow));
  });
}
