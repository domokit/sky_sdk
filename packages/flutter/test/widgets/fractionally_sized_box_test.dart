// Copyright 2014 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

import 'package:flutter/widgets.dart';
import 'package:flutter_test/flutter_test.dart';

void main() {
  testWidgets('FractionallySizedBox', (WidgetTester tester) async {
    final GlobalKey inner = GlobalKey();
    await tester.pumpWidget(
      OverflowBox(
        minWidth: 0.0,
        maxWidth: 100.0,
        minHeight: 0.0,
        maxHeight: 100.0,
        alignment: Alignment.topLeft,
        child: Center(
          child: FractionallySizedBox(
            widthFactor: 0.5,
            heightFactor: 0.25,
            child: Container(key: inner),
          ),
        ),
      ),
    );
    final RenderBox box = inner.currentContext!.findRenderObject()! as RenderBox;
    expect(box.size, equals(const Size(50.0, 25.0)));
    expect(box.localToGlobal(Offset.zero), equals(const Offset(25.0, 37.5)));
  });

  testWidgets('FractionallySizedBox alignment', (WidgetTester tester) async {
    final GlobalKey inner = GlobalKey();
    await tester.pumpWidget(
      Directionality(
        textDirection: TextDirection.rtl,
        child: FractionallySizedBox(
          widthFactor: 0.5,
          heightFactor: 0.5,
          alignment: Alignment.topRight,
          child: Placeholder(key: inner),
        ),
      ),
    );
    final RenderBox box = inner.currentContext!.findRenderObject()! as RenderBox;
    expect(box.size, equals(const Size(400.0, 300.0)));
    expect(
      box.localToGlobal(box.size.center(Offset.zero)),
      equals(const Offset(800.0 - 400.0 / 2.0, 0.0 + 300.0 / 2.0)),
    );
  });

  testWidgets('FractionallySizedBox alignment (direction-sensitive)', (WidgetTester tester) async {
    final GlobalKey inner = GlobalKey();
    await tester.pumpWidget(
      Directionality(
        textDirection: TextDirection.rtl,
        child: FractionallySizedBox(
          widthFactor: 0.5,
          heightFactor: 0.5,
          alignment: AlignmentDirectional.topEnd,
          child: Placeholder(key: inner),
        ),
      ),
    );
    final RenderBox box = inner.currentContext!.findRenderObject()! as RenderBox;
    expect(box.size, equals(const Size(400.0, 300.0)));
    expect(
      box.localToGlobal(box.size.center(Offset.zero)),
      equals(const Offset(0.0 + 400.0 / 2.0, 0.0 + 300.0 / 2.0)),
    );
  });

  testWidgets('OverflowBox alignment with FractionallySizedBox', (WidgetTester tester) async {
    final GlobalKey inner = GlobalKey();
    await tester.pumpWidget(
      Directionality(
        textDirection: TextDirection.rtl,
        child: OverflowBox(
          minWidth: 0.0,
          maxWidth: 100.0,
          minHeight: 0.0,
          maxHeight: 100.0,
          alignment: AlignmentDirectional.topEnd,
          child: Center(
            child: FractionallySizedBox(
              widthFactor: 0.5,
              heightFactor: 0.25,
              child: Container(key: inner),
            ),
          ),
        ),
      ),
    );
    final RenderBox box = inner.currentContext!.findRenderObject()! as RenderBox;
    expect(box.size, equals(const Size(50.0, 25.0)));
    expect(box.localToGlobal(Offset.zero), equals(const Offset(25.0, 37.5)));
  });
}
