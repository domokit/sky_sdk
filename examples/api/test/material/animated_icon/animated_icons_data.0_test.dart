// Copyright 2014 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

import 'package:flutter/material.dart';
import 'package:flutter_api_samples/material/animated_icon/animated_icons_data.0.dart' as example;
import 'package:flutter_test/flutter_test.dart';

void main() {
  testWidgets('Show all the animated icons', (WidgetTester tester) async {
    await tester.pumpWidget(const example.AnimatedIconApp());

    // Check if the total number of AnimatedIcons matches the icons list.
    expect(find.byType(AnimatedIcon, skipOffstage: false), findsNWidgets(example.iconsList.length));

    // Test the AnimatedIcon size.
    final Size iconSize = tester.getSize(find.byType(AnimatedIcon).first);
    expect(iconSize.width, 72.0);
    expect(iconSize.height, 72.0);

    // Check if AnimatedIcon is animating.
    await tester.pump(const Duration(milliseconds: 500));
    AnimatedIcon animatedIcon = tester.widget(find.byType(AnimatedIcon).first);
    expect(animatedIcon.progress.value, 0.25);

    // Check if animation is completed.
    await tester.pump(const Duration(milliseconds: 1500));
    animatedIcon = tester.widget(find.byType(AnimatedIcon).first);
    expect(animatedIcon.progress.value, 1.0);
  });
}
