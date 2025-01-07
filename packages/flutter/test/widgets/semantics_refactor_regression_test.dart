// Copyright 2014 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

import 'package:flutter/material.dart';
import 'package:flutter/widgets.dart';
import 'package:flutter_test/flutter_test.dart';

void main() {
  testWidgets('finder does not return dirty semantics nodes', (WidgetTester tester) async {
    final UniqueKey key1 = UniqueKey();
    final UniqueKey key2 = UniqueKey();
    const String label = 'label';
    // not merged
    await tester.pumpWidget(
      Directionality(
        textDirection: TextDirection.ltr,
        child: Semantics(
          key: key1,
          label: label,
          container: true,
          child: Semantics(
            key: key2,
            label: label,
            container: true,
            child: const SizedBox(width: 100, height: 100),
          ),
        ),
      ),
    );

    expect(find.bySemanticsLabel(label), findsExactly(2));

    await tester.pumpWidget(
      Directionality(
        textDirection: TextDirection.ltr,
        // key2 widget should merge up to key1, its dirty cached semantics node
        // should not show up in the finder.
        child: Semantics(
          key: key1,
          container: true,
          child: Semantics(key: key2, label: label, child: const SizedBox(width: 100, height: 100)),
        ),
      ),
    );
    expect(find.bySemanticsLabel(label), findsOneWidget);
  });
}
