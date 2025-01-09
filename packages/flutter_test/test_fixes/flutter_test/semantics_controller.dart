// Copyright 2014 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

import 'package:flutter_test/flutter_test.dart';

void main() {
  // Generic reference variables.
  finders.FinderBase<Element> theStart;
  finders.FinderBase<Element> theEnd;

  testWidgets('simulatedAccessibilityTraversal', (WidgetTester tester) async {
    // Changes made in https://github.com/flutter/flutter/pull/143386
    tester.semantics.simulatedAccessibilityTraversal();
    tester.semantics.simulatedAccessibilityTraversal(start: theStart);
    tester.semantics.simulatedAccessibilityTraversal(end: theEnd);
    tester.semantics.simulatedAccessibilityTraversal(
      start: theStart,
      end: theEnd,
    );
  });
}
