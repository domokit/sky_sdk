// Copyright 2014 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

import 'package:flutter/widgets.dart';
import 'package:flutter_test/flutter_test.dart';

void main() {
  testWidgets('positions itself at anchorAbove if it fits', (WidgetTester tester) async {
    late StateSetter setState;
    const double height = 43.0;
    const double anchorBelowY = 500.0;
    double anchorAboveY = 0.0;

    await tester.pumpWidget(
      MediaQuery(
        data: const MediaQueryData(),
        child: Directionality(
          textDirection: TextDirection.ltr,
          child: StatefulBuilder(
            builder: (BuildContext context, StateSetter setter) {
              setState = setter;
              return CustomSingleChildLayout(
                delegate: TextSelectionToolbarLayoutDelegate(
                  anchorAbove: Offset(50.0, anchorAboveY),
                  anchorBelow: const Offset(50.0, anchorBelowY),
                ),
                child: Container(width: 200.0, height: height, color: const Color(0xffff0000)),
              );
            },
          ),
        ),
      ),
    );

    // When the toolbar doesn't fit above aboveAnchor, it positions itself below
    // belowAnchor.
    double toolbarY = tester.getTopLeft(find.byType(Container)).dy;
    expect(toolbarY, equals(anchorBelowY));

    // Even when it barely doesn't fit.
    setState(() {
      anchorAboveY = height - 1.0;
    });
    await tester.pump();
    toolbarY = tester.getTopLeft(find.byType(Container)).dy;
    expect(toolbarY, equals(anchorBelowY));

    // When it does fit above aboveAnchor, it positions itself there.
    setState(() {
      anchorAboveY = height;
    });
    await tester.pump();
    toolbarY = tester.getTopLeft(find.byType(Container)).dy;
    expect(toolbarY, equals(anchorAboveY - height));
  });
}
