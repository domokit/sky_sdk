// Copyright 2014 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

import 'package:flutter/material.dart';
import 'package:flutter_api_samples/widgets/animated_size/animated_size.0.dart' as example;
import 'package:flutter_test/flutter_test.dart';

void main() {
  testWidgets('AnimatedSize animates on tap', (WidgetTester tester) async {
    await tester.pumpWidget(const example.AnimatedSizeExampleApp());

    const Size beginSize = Size.square(100.0);
    const Size endSize = Size.square(250.0);

    RenderBox box = tester.renderObject(find.byType(AnimatedSize));
    expect(box.size, equals(beginSize));

    // Tap on the AnimatedSize to start the forward animation.
    await tester.tap(find.byType(AnimatedSize));
    await tester.pump();

    box = tester.renderObject(find.byType(AnimatedSize));
    expect(box.size, equals(beginSize));

    // Advance animation to the middle.
    await tester.pump(example.AnimatedSizeExampleApp.duration ~/ 2);

    final double t = example.AnimatedSizeExampleApp.curve.transform(0.5);

    box = tester.renderObject(find.byType(AnimatedSize));
    expect(box.size, equals(Size.lerp(beginSize, endSize, t)));

    // Advance animation to the end.
    await tester.pump(example.AnimatedSizeExampleApp.duration ~/ 2);

    box = tester.renderObject(find.byType(AnimatedSize));
    expect(box.size, equals(endSize));

    // Tap on the AnimatedSize again to start the reverse animation.
    await tester.tap(find.byType(AnimatedSize));
    await tester.pump();

    box = tester.renderObject(find.byType(AnimatedSize));
    expect(box.size, equals(endSize));

    // Advance animation to the middle.
    await tester.pump(example.AnimatedSizeExampleApp.duration ~/ 2);

    box = tester.renderObject(find.byType(AnimatedSize));
    expect(box.size, equals(Size.lerp(endSize, beginSize, t)));

    // Advance animation to the end.
    await tester.pump(example.AnimatedSizeExampleApp.duration ~/ 2);

    box = tester.renderObject(find.byType(AnimatedSize));
    expect(box.size, equals(beginSize));
  });
}
