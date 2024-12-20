// Copyright 2014 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

import 'package:flutter/material.dart';
import 'package:flutter_test/flutter_test.dart';

void main() {
  testWidgets('positions itself at anchorAbove if it fits and shifts up when not', (
    WidgetTester tester,
  ) async {
    late StateSetter setState;
    const double toolbarOverlap = 100;
    const double height = 500;
    double anchorY = 200.0;

    await tester.pumpWidget(
      MediaQuery(
        data: const MediaQueryData(),
        child: Directionality(
          textDirection: TextDirection.ltr,
          child: StatefulBuilder(
            builder: (BuildContext context, StateSetter setter) {
              setState = setter;
              return CustomSingleChildLayout(
                delegate: SpellCheckSuggestionsToolbarLayoutDelegate(anchor: Offset(50.0, anchorY)),
                child: Container(width: 200.0, height: height, color: const Color(0xffff0000)),
              );
            },
          ),
        ),
      ),
    );

    // When the toolbar doesn't fit below anchor, it positions itself such that
    // it can just fit.
    double toolbarY = tester.getTopLeft(find.byType(Container)).dy;
    // Total height available is 600.
    expect(toolbarY, equals(toolbarOverlap));

    // When it does fit below anchor, it positions itself there.
    setState(() {
      anchorY = anchorY - toolbarOverlap;
    });
    await tester.pump();
    toolbarY = tester.getTopLeft(find.byType(Container)).dy;
    expect(toolbarY, equals(anchorY));
  });
}
