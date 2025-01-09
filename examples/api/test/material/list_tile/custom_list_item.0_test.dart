// Copyright 2014 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

import 'package:flutter/material.dart';
import 'package:flutter_api_samples/material/list_tile/custom_list_item.0.dart' as example;
import 'package:flutter_test/flutter_test.dart';

void main() {
  testWidgets('Custom list item uses Expanded widgets for the layout', (WidgetTester tester) async {
    await tester.pumpWidget(const example.CustomListItemApp());

    // The Expanded widget is used to control the size of the thumbnail.
    Expanded thumbnailExpanded = tester.widget(
      find.ancestor(of: find.byType(Container).first, matching: find.byType(Expanded)),
    );
    expect(thumbnailExpanded.flex, 2);

    // The Expanded widget is used to control the size of the text.
    Expanded textExpanded = tester.widget(
      find.ancestor(of: find.text('The Flutter YouTube Channel'), matching: find.byType(Expanded)),
    );
    expect(textExpanded.flex, 3);

    // The Expanded widget is used to control the size of the thumbnail.
    thumbnailExpanded = tester.widget(
      find.ancestor(of: find.byType(Container).last, matching: find.byType(Expanded)),
    );
    expect(thumbnailExpanded.flex, 2);

    // The Expanded widget is used to control the size of the text.
    textExpanded = tester.widget(
      find.ancestor(of: find.text('Announcing Flutter 1.0'), matching: find.byType(Expanded)),
    );
    expect(textExpanded.flex, 3);
  });
}
