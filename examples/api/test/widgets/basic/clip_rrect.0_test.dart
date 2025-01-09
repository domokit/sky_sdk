// Copyright 2014 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

import 'package:flutter/material.dart';
import 'package:flutter_api_samples/widgets/basic/clip_rrect.0.dart' as example;
import 'package:flutter_test/flutter_test.dart';

void main() {
  testWidgets('ClipRRect adds rounded corners to containers', (WidgetTester tester) async {
    await tester.pumpWidget(const example.ClipRRectApp());

    final Finder clipRRectFinder = find.byType(ClipRRect);
    final Finder containerFinder = find.byType(Container);
    expect(clipRRectFinder, findsNWidgets(2));
    expect(containerFinder, findsNWidgets(3));

    final Rect firstClipRect = tester.getRect(clipRRectFinder.first);
    final Rect secondContainerRect = tester.getRect(containerFinder.at(1));
    expect(firstClipRect, equals(secondContainerRect));

    final Rect secondClipRect = tester.getRect(clipRRectFinder.at(1));
    final Rect thirdContainerRect = tester.getRect(containerFinder.at(2));
    expect(secondClipRect, equals(thirdContainerRect));
  });
}
