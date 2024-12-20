// Copyright 2014 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

import 'package:flutter/widgets.dart';
import 'package:flutter_test/flutter_test.dart';

void main() {
  testWidgets('Padding RTL', (WidgetTester tester) async {
    const Widget child = Padding(
      padding: EdgeInsetsDirectional.only(start: 10.0),
      child: Placeholder(),
    );
    await tester.pumpWidget(const Directionality(textDirection: TextDirection.ltr, child: child));
    expect(tester.getTopLeft(find.byType(Placeholder)), const Offset(10.0, 0.0));
    await tester.pumpWidget(const Directionality(textDirection: TextDirection.rtl, child: child));
    expect(tester.getTopLeft(find.byType(Placeholder)), Offset.zero);

    await tester.pumpWidget(
      const Padding(
        key: GlobalObjectKey<State<StatefulWidget>>(Object()),
        padding: EdgeInsets.only(left: 1.0),
      ),
    );
    await tester.pumpWidget(
      const Directionality(
        textDirection: TextDirection.rtl,
        child: Padding(
          key: GlobalObjectKey<State<StatefulWidget>>(Object()),
          padding: EdgeInsetsDirectional.only(start: 1.0),
        ),
      ),
    );
    await tester.pumpWidget(
      const Padding(
        key: GlobalObjectKey<State<StatefulWidget>>(Object()),
        padding: EdgeInsets.only(left: 1.0),
      ),
    );
  });

  testWidgets('Container padding/margin RTL', (WidgetTester tester) async {
    final Widget child = Container(
      padding: const EdgeInsetsDirectional.only(start: 6.0),
      margin: const EdgeInsetsDirectional.only(end: 20.0, start: 4.0),
      child: const Placeholder(),
    );
    await tester.pumpWidget(Directionality(textDirection: TextDirection.ltr, child: child));
    expect(tester.getTopLeft(find.byType(Placeholder)), const Offset(10.0, 0.0));
    expect(tester.getTopRight(find.byType(Placeholder)), const Offset(780.0, 0.0));
    await tester.pumpWidget(Directionality(textDirection: TextDirection.rtl, child: child));
    expect(tester.getTopLeft(find.byType(Placeholder)), const Offset(20.0, 0.0));
    expect(tester.getTopRight(find.byType(Placeholder)), const Offset(790.0, 0.0));
  });

  testWidgets('Container padding/margin mixed RTL/absolute', (WidgetTester tester) async {
    final Widget child = Container(
      padding: const EdgeInsets.only(left: 6.0),
      margin: const EdgeInsetsDirectional.only(end: 20.0, start: 4.0),
      child: const Placeholder(),
    );
    await tester.pumpWidget(Directionality(textDirection: TextDirection.ltr, child: child));
    expect(tester.getTopLeft(find.byType(Placeholder)), const Offset(10.0, 0.0));
    expect(tester.getTopRight(find.byType(Placeholder)), const Offset(780.0, 0.0));
    await tester.pumpWidget(Directionality(textDirection: TextDirection.rtl, child: child));
    expect(tester.getTopLeft(find.byType(Placeholder)), const Offset(26.0, 0.0));
    expect(tester.getTopRight(find.byType(Placeholder)), const Offset(796.0, 0.0));
  });

  testWidgets('EdgeInsetsDirectional without Directionality', (WidgetTester tester) async {
    await tester.pumpWidget(const Padding(padding: EdgeInsetsDirectional.zero));
    expect(tester.takeException(), isAssertionError);
  });
}
