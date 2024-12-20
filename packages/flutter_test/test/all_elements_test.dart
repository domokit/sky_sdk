// Copyright 2014 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

import 'package:flutter/material.dart';
import 'package:flutter_test/flutter_test.dart';

void main() {
  testWidgets('collectAllElements goes in LTR DFS', (WidgetTester tester) async {
    final GlobalKey key = GlobalKey();
    await tester.pumpWidget(
      Directionality(
        key: key,
        textDirection: TextDirection.ltr,
        child: Row(
          children: <Widget>[
            RichText(text: const TextSpan(text: 'a')),
            RichText(text: const TextSpan(text: 'b')),
          ],
        ),
      ),
    );

    final List<Element> elements =
        collectAllElementsFrom(key.currentContext! as Element, skipOffstage: false).toList();

    expect(elements.length, 3);
    expect(elements[0].widget, isA<Row>());
    expect(elements[1].widget, isA<RichText>());
    expect(((elements[1].widget as RichText).text as TextSpan).text, 'a');
    expect(elements[2].widget, isA<RichText>());
    expect(((elements[2].widget as RichText).text as TextSpan).text, 'b');
  });
}
