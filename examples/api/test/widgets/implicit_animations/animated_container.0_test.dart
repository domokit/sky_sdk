// Copyright 2014 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

import 'package:flutter/material.dart';
import 'package:flutter_api_samples/widgets/implicit_animations/animated_container.0.dart'
    as example;
import 'package:flutter_test/flutter_test.dart';

void main() {
  testWidgets('AnimatedContainer updates on tap', (WidgetTester tester) async {
    await tester.pumpWidget(const example.AnimatedContainerExampleApp());

    Container container = tester.widget(
      find.ancestor(of: find.byType(AnimatedContainer), matching: find.byType(Container)),
    );
    expect((container.decoration! as BoxDecoration).color, equals(Colors.blue));
    expect(container.constraints, equals(const BoxConstraints.tightFor(width: 100, height: 200)));
    expect(container.alignment, equals(Alignment.topCenter));

    await tester.tap(find.byType(FlutterLogo));
    await tester.pump();

    container = tester.widget(
      find.ancestor(of: find.byType(AnimatedContainer), matching: find.byType(Container)),
    );
    expect((container.decoration! as BoxDecoration).color, equals(Colors.blue));
    expect(container.constraints, equals(const BoxConstraints.tightFor(width: 100, height: 200)));
    expect(container.alignment, equals(Alignment.topCenter));

    // Advance animation to the end by the 2-second duration specified in
    // the example app.
    await tester.pump(const Duration(seconds: 2));

    container = tester.widget(
      find.ancestor(of: find.byType(AnimatedContainer), matching: find.byType(Container)),
    );
    expect((container.decoration! as BoxDecoration).color, equals(Colors.red));
    expect(container.constraints, equals(const BoxConstraints.tightFor(width: 200, height: 100)));
    expect(container.alignment, equals(Alignment.center));
  });
}
