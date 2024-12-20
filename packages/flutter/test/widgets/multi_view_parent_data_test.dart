// Copyright 2014 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

import 'package:flutter/cupertino.dart';
import 'package:flutter/material.dart';
import 'package:flutter_test/flutter_test.dart';

import 'multi_view_testing.dart';

void main() {
  testWidgets(
    'Hot reload does not crash if ViewAnchor is used between ParentDataWidget and the render object it is applied to',
    (WidgetTester tester) async {
      // Regression test for https://github.com/flutter/flutter/issues/142480.
      Widget buildTest(String string) {
        return Directionality(
          textDirection: TextDirection.ltr,
          child: Stack(
            children: <Widget>[
              Positioned(
                // ParentDataWidget
                right: 0,
                bottom: 0,
                child: ViewAnchor(
                  view: View(view: FakeView(tester.view), child: const Text('Side-view')),
                  child: Text(
                    string,
                  ), // Text's RenderObject is the target for the ParentDataWidget above.
                ),
              ),
            ],
          ),
        );
      }

      await tester.pumpWidget(buildTest('bottom-right'));
      expect(tester.getBottomRight(find.text('bottom-right')), const Offset(800, 600));
      // Rebuild with a slightly different string to simulate a hot reload.
      await tester.pumpWidget(buildTest('bottom-right-again'));
      expect(find.text('bottom-right'), findsNothing);
      expect(tester.getBottomRight(find.text('bottom-right-again')), const Offset(800, 600));
    },
  );
}
