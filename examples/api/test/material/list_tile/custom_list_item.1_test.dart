// Copyright 2014 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

import 'package:flutter/material.dart';
import 'package:flutter_api_samples/material/list_tile/custom_list_item.1.dart' as example;
import 'package:flutter_test/flutter_test.dart';

void main() {
  testWidgets('Custom list item uses AspectRatio and Expanded widgets for the layout', (
    WidgetTester tester,
  ) async {
    await tester.pumpWidget(const example.CustomListItemApp());

    // The AspectRatio widget is used to constrain the size of the thumbnail.
    AspectRatio thumbnailAspectRatio = tester.widget(
      find.ancestor(of: find.byType(Container).first, matching: find.byType(AspectRatio)),
    );
    expect(thumbnailAspectRatio.aspectRatio, 1.0);

    // The Expanded widget is used to control the size of the text.
    final Expanded textExpanded = tester.widget(
      find.ancestor(of: find.text('Flutter 1.0 Launch'), matching: find.byType(Expanded).at(0)),
    );
    expect(textExpanded.flex, 1);

    // The AspectRatio widget is used to constrain the size of the thumbnail.
    thumbnailAspectRatio = tester.widget(
      find.ancestor(of: find.byType(Container).last, matching: find.byType(AspectRatio)),
    );
    expect(thumbnailAspectRatio.aspectRatio, 1.0);
  });
}
