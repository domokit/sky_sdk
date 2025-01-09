// Copyright 2014 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

import 'package:flutter/foundation.dart';
import 'package:flutter/rendering.dart';
import 'package:flutter/widgets.dart';
import 'package:flutter_test/flutter_test.dart';

class OrderPainter extends CustomPainter {
  const OrderPainter(this.index);

  final int index;

  static List<int> log = <int>[];

  @override
  void paint(Canvas canvas, Size size) {
    log.add(index);
  }

  @override
  bool shouldRepaint(OrderPainter old) => false;
}

Widget log(int index) => CustomPaint(painter: OrderPainter(index));

void main() {
  // NO DIRECTION

  testWidgets('Row with one Flexible child - no textDirection', (WidgetTester tester) async {
    OrderPainter.log.clear();
    const Key rowKey = Key('row');
    const Key child0Key = Key('child0');
    const Key child1Key = Key('child1');
    const Key child2Key = Key('child2');

    final FlutterExceptionHandler? oldHandler = FlutterError.onError;
    dynamic exception;
    FlutterError.onError = (FlutterErrorDetails details) {
      exception ??= details.exception;
    };

    // Default is MainAxisAlignment.start so this should fail, asking for a direction.
    await tester.pumpWidget(
      Center(
        child: Row(
          key: rowKey,
          children: <Widget>[
            SizedBox(key: child0Key, width: 100.0, height: 100.0, child: log(1)),
            Expanded(child: SizedBox(key: child1Key, width: 100.0, height: 100.0, child: log(2))),
            SizedBox(key: child2Key, width: 100.0, height: 100.0, child: log(3)),
          ],
        ),
      ),
    );

    FlutterError.onError = oldHandler;
    expect(exception, isAssertionError);
    expect(exception.toString(), contains('textDirection'));
    expect(OrderPainter.log, <int>[]);
  });

  testWidgets('Row with default main axis parameters - no textDirection', (
    WidgetTester tester,
  ) async {
    OrderPainter.log.clear();
    const Key rowKey = Key('row');
    const Key child0Key = Key('child0');
    const Key child1Key = Key('child1');
    const Key child2Key = Key('child2');

    final FlutterExceptionHandler? oldHandler = FlutterError.onError;
    dynamic exception;
    FlutterError.onError = (FlutterErrorDetails details) {
      exception ??= details.exception;
    };

    // Default is MainAxisAlignment.start so this should fail too.
    await tester.pumpWidget(
      Center(
        child: Row(
          key: rowKey,
          children: <Widget>[
            SizedBox(key: child0Key, width: 100.0, height: 100.0, child: log(1)),
            SizedBox(key: child1Key, width: 100.0, height: 100.0, child: log(2)),
            SizedBox(key: child2Key, width: 100.0, height: 100.0, child: log(3)),
          ],
        ),
      ),
    );

    FlutterError.onError = oldHandler;
    expect(exception, isAssertionError);
    expect(exception.toString(), contains('textDirection'));
    expect(OrderPainter.log, <int>[]);
  });

  testWidgets('Row with MainAxisAlignment.center - no textDirection', (WidgetTester tester) async {
    OrderPainter.log.clear();
    const Key rowKey = Key('row');
    const Key child0Key = Key('child0');
    const Key child1Key = Key('child1');

    final FlutterExceptionHandler? oldHandler = FlutterError.onError;
    dynamic exception;
    FlutterError.onError = (FlutterErrorDetails details) {
      exception ??= details.exception;
    };

    // More than one child, so it's not clear what direction to lay out in: should fail.
    await tester.pumpWidget(
      Center(
        child: Row(
          key: rowKey,
          mainAxisAlignment: MainAxisAlignment.center,
          children: <Widget>[
            SizedBox(key: child0Key, width: 100.0, height: 100.0, child: log(1)),
            SizedBox(key: child1Key, width: 100.0, height: 100.0, child: log(2)),
          ],
        ),
      ),
    );

    FlutterError.onError = oldHandler;
    expect(exception, isAssertionError);
    expect(exception.toString(), contains('textDirection'));
    expect(OrderPainter.log, <int>[]);
  });

  testWidgets('Row with MainAxisAlignment.end - no textDirection', (WidgetTester tester) async {
    OrderPainter.log.clear();
    const Key rowKey = Key('row');
    const Key child0Key = Key('child0');
    const Key child1Key = Key('child1');
    const Key child2Key = Key('child2');

    final FlutterExceptionHandler? oldHandler = FlutterError.onError;
    dynamic exception;
    FlutterError.onError = (FlutterErrorDetails details) {
      exception ??= details.exception;
    };

    // No direction so this should fail, asking for a direction.
    await tester.pumpWidget(
      Center(
        child: Row(
          key: rowKey,
          mainAxisAlignment: MainAxisAlignment.end,
          children: <Widget>[
            SizedBox(key: child0Key, width: 100.0, height: 100.0, child: log(1)),
            SizedBox(key: child1Key, width: 100.0, height: 100.0, child: log(2)),
            SizedBox(key: child2Key, width: 100.0, height: 100.0, child: log(3)),
          ],
        ),
      ),
    );

    FlutterError.onError = oldHandler;
    expect(exception, isAssertionError);
    expect(exception.toString(), contains('textDirection'));
    expect(OrderPainter.log, <int>[]);
  });

  testWidgets('Row with MainAxisAlignment.spaceBetween - no textDirection', (
    WidgetTester tester,
  ) async {
    OrderPainter.log.clear();
    const Key rowKey = Key('row');
    const Key child0Key = Key('child0');
    const Key child1Key = Key('child1');
    const Key child2Key = Key('child2');

    final FlutterExceptionHandler? oldHandler = FlutterError.onError;
    dynamic exception;
    FlutterError.onError = (FlutterErrorDetails details) {
      exception ??= details.exception;
    };

    // More than one child, so it's not clear what direction to lay out in: should fail.
    await tester.pumpWidget(
      Center(
        child: Row(
          key: rowKey,
          mainAxisAlignment: MainAxisAlignment.spaceBetween,
          children: <Widget>[
            SizedBox(key: child0Key, width: 100.0, height: 100.0, child: log(1)),
            SizedBox(key: child1Key, width: 100.0, height: 100.0, child: log(2)),
            SizedBox(key: child2Key, width: 100.0, height: 100.0, child: log(3)),
          ],
        ),
      ),
    );

    FlutterError.onError = oldHandler;
    expect(exception, isAssertionError);
    expect(exception.toString(), contains('textDirection'));
    expect(OrderPainter.log, <int>[]);
  });

  testWidgets('Row with MainAxisAlignment.spaceAround - no textDirection', (
    WidgetTester tester,
  ) async {
    OrderPainter.log.clear();
    const Key rowKey = Key('row');
    const Key child0Key = Key('child0');
    const Key child1Key = Key('child1');
    const Key child2Key = Key('child2');
    const Key child3Key = Key('child3');

    final FlutterExceptionHandler? oldHandler = FlutterError.onError;
    dynamic exception;
    FlutterError.onError = (FlutterErrorDetails details) {
      exception ??= details.exception;
    };

    // More than one child, so it's not clear what direction to lay out in: should fail.
    await tester.pumpWidget(
      Center(
        child: Row(
          key: rowKey,
          mainAxisAlignment: MainAxisAlignment.spaceAround,
          children: <Widget>[
            SizedBox(key: child0Key, width: 100.0, height: 100.0, child: log(1)),
            SizedBox(key: child1Key, width: 100.0, height: 100.0, child: log(2)),
            SizedBox(key: child2Key, width: 100.0, height: 100.0, child: log(3)),
            SizedBox(key: child3Key, width: 100.0, height: 100.0, child: log(4)),
          ],
        ),
      ),
    );

    FlutterError.onError = oldHandler;
    expect(exception, isAssertionError);
    expect(exception.toString(), contains('textDirection'));
    expect(OrderPainter.log, <int>[]);
  });

  testWidgets('Row with MainAxisAlignment.spaceEvenly - no textDirection', (
    WidgetTester tester,
  ) async {
    OrderPainter.log.clear();
    const Key rowKey = Key('row');
    const Key child0Key = Key('child0');
    const Key child1Key = Key('child1');
    const Key child2Key = Key('child2');

    final FlutterExceptionHandler? oldHandler = FlutterError.onError;
    dynamic exception;
    FlutterError.onError = (FlutterErrorDetails details) {
      exception ??= details.exception;
    };

    // More than one child, so it's not clear what direction to lay out in: should fail.
    await tester.pumpWidget(
      Center(
        child: Row(
          key: rowKey,
          mainAxisAlignment: MainAxisAlignment.spaceEvenly,
          children: <Widget>[
            SizedBox(key: child0Key, width: 200.0, height: 100.0, child: log(1)),
            SizedBox(key: child1Key, width: 200.0, height: 100.0, child: log(2)),
            SizedBox(key: child2Key, width: 200.0, height: 100.0, child: log(3)),
          ],
        ),
      ),
    );

    FlutterError.onError = oldHandler;
    expect(exception, isAssertionError);
    expect(exception.toString(), contains('textDirection'));
    expect(OrderPainter.log, <int>[]);
  });

  testWidgets('Row and MainAxisSize.min - no textDirection', (WidgetTester tester) async {
    OrderPainter.log.clear();
    const Key rowKey = Key('rowKey');
    const Key child0Key = Key('child0');
    const Key child1Key = Key('child1');

    final FlutterExceptionHandler? oldHandler = FlutterError.onError;
    dynamic exception;
    FlutterError.onError = (FlutterErrorDetails details) {
      exception ??= details.exception;
    };

    // Default is MainAxisAlignment.start so this should fail, asking for a direction.
    await tester.pumpWidget(
      Center(
        child: Row(
          key: rowKey,
          mainAxisSize: MainAxisSize.min,
          children: <Widget>[
            SizedBox(key: child0Key, width: 100.0, height: 100.0, child: log(1)),
            SizedBox(key: child1Key, width: 150.0, height: 100.0, child: log(2)),
          ],
        ),
      ),
    );

    FlutterError.onError = oldHandler;
    expect(exception, isAssertionError);
    expect(exception.toString(), contains('textDirection'));
    expect(OrderPainter.log, <int>[]);
  });

  testWidgets('Row MainAxisSize.min layout at zero size - no textDirection', (
    WidgetTester tester,
  ) async {
    OrderPainter.log.clear();
    const Key childKey = Key('childKey');

    await tester.pumpWidget(
      const Center(
        child: SizedBox.shrink(
          child: Row(
            mainAxisSize: MainAxisSize.min,
            mainAxisAlignment: MainAxisAlignment.center,
            children: <Widget>[SizedBox(key: childKey, width: 100.0, height: 100.0)],
          ),
        ),
      ),
    );

    final RenderBox renderBox = tester.renderObject(find.byKey(childKey));
    expect(renderBox.size.width, equals(100.0));
    expect(renderBox.size.height, equals(0.0));
  });

  // LTR

  testWidgets('Row with one Flexible child - LTR', (WidgetTester tester) async {
    OrderPainter.log.clear();
    const Key rowKey = Key('row');
    const Key child0Key = Key('child0');
    const Key child1Key = Key('child1');
    const Key child2Key = Key('child2');

    // Default is MainAxisSize.max so the Row should be as wide as the test: 800.
    // Default is MainAxisAlignment.start so children so the children's
    // left edges should be at 0, 100, 700, child2's width should be 600
    await tester.pumpWidget(
      Center(
        child: Row(
          key: rowKey,
          textDirection: TextDirection.ltr,
          children: <Widget>[
            SizedBox(key: child0Key, width: 100.0, height: 100.0, child: log(1)),
            Expanded(child: SizedBox(key: child1Key, width: 100.0, height: 100.0, child: log(2))),
            SizedBox(key: child2Key, width: 100.0, height: 100.0, child: log(3)),
          ],
        ),
      ),
    );

    RenderBox renderBox;
    BoxParentData boxParentData;

    renderBox = tester.renderObject(find.byKey(rowKey));
    expect(renderBox.size.width, equals(800.0));
    expect(renderBox.size.height, equals(100.0));

    renderBox = tester.renderObject(find.byKey(child0Key));
    expect(renderBox.size.width, equals(100.0));
    expect(renderBox.size.height, equals(100.0));
    boxParentData = renderBox.parentData! as BoxParentData;
    expect(boxParentData.offset.dx, equals(0.0));

    renderBox = tester.renderObject(find.byKey(child1Key));
    expect(renderBox.size.width, equals(600.0));
    expect(renderBox.size.height, equals(100.0));
    boxParentData = renderBox.parentData! as BoxParentData;
    expect(boxParentData.offset.dx, equals(100.0));

    renderBox = tester.renderObject(find.byKey(child2Key));
    expect(renderBox.size.width, equals(100.0));
    expect(renderBox.size.height, equals(100.0));
    boxParentData = renderBox.parentData! as BoxParentData;
    expect(boxParentData.offset.dx, equals(700.0));

    expect(OrderPainter.log, <int>[1, 2, 3]);
  });

  testWidgets('Row with default main axis parameters - LTR', (WidgetTester tester) async {
    OrderPainter.log.clear();
    const Key rowKey = Key('row');
    const Key child0Key = Key('child0');
    const Key child1Key = Key('child1');
    const Key child2Key = Key('child2');

    // Default is MainAxisSize.max so the Row should be as wide as the test: 800.
    // Default is MainAxisAlignment.start so children so the children's
    // left edges should be at 0, 100, 200
    await tester.pumpWidget(
      Center(
        child: Row(
          key: rowKey,
          textDirection: TextDirection.ltr,
          children: <Widget>[
            SizedBox(key: child0Key, width: 100.0, height: 100.0, child: log(1)),
            SizedBox(key: child1Key, width: 100.0, height: 100.0, child: log(2)),
            SizedBox(key: child2Key, width: 100.0, height: 100.0, child: log(3)),
          ],
        ),
      ),
    );

    RenderBox renderBox;
    BoxParentData boxParentData;

    renderBox = tester.renderObject(find.byKey(rowKey));
    expect(renderBox.size.width, equals(800.0));
    expect(renderBox.size.height, equals(100.0));

    renderBox = tester.renderObject(find.byKey(child0Key));
    expect(renderBox.size.width, equals(100.0));
    expect(renderBox.size.height, equals(100.0));
    boxParentData = renderBox.parentData! as BoxParentData;
    expect(boxParentData.offset.dx, equals(0.0));

    renderBox = tester.renderObject(find.byKey(child1Key));
    expect(renderBox.size.width, equals(100.0));
    expect(renderBox.size.height, equals(100.0));
    boxParentData = renderBox.parentData! as BoxParentData;
    expect(boxParentData.offset.dx, equals(100.0));

    renderBox = tester.renderObject(find.byKey(child2Key));
    expect(renderBox.size.width, equals(100.0));
    expect(renderBox.size.height, equals(100.0));
    boxParentData = renderBox.parentData! as BoxParentData;
    expect(boxParentData.offset.dx, equals(200.0));

    expect(OrderPainter.log, <int>[1, 2, 3]);
  });

  testWidgets('Row with MainAxisAlignment.center - LTR', (WidgetTester tester) async {
    OrderPainter.log.clear();
    const Key rowKey = Key('row');
    const Key child0Key = Key('child0');
    const Key child1Key = Key('child1');

    // Default is MainAxisSize.max so the Row should be as wide as the test: 800.
    // The 100x100 children's left edges should be at 300, 400
    await tester.pumpWidget(
      Center(
        child: Row(
          key: rowKey,
          mainAxisAlignment: MainAxisAlignment.center,
          textDirection: TextDirection.ltr,
          children: <Widget>[
            SizedBox(key: child0Key, width: 100.0, height: 100.0, child: log(1)),
            SizedBox(key: child1Key, width: 100.0, height: 100.0, child: log(2)),
          ],
        ),
      ),
    );

    RenderBox renderBox;
    BoxParentData boxParentData;

    renderBox = tester.renderObject(find.byKey(rowKey));
    expect(renderBox.size.width, equals(800.0));
    expect(renderBox.size.height, equals(100.0));

    renderBox = tester.renderObject(find.byKey(child0Key));
    expect(renderBox.size.width, equals(100.0));
    expect(renderBox.size.height, equals(100.0));
    boxParentData = renderBox.parentData! as BoxParentData;
    expect(boxParentData.offset.dx, equals(300.0));

    renderBox = tester.renderObject(find.byKey(child1Key));
    expect(renderBox.size.width, equals(100.0));
    expect(renderBox.size.height, equals(100.0));
    boxParentData = renderBox.parentData! as BoxParentData;
    expect(boxParentData.offset.dx, equals(400.0));

    expect(OrderPainter.log, <int>[1, 2]);
  });

  testWidgets('Row with MainAxisAlignment.end - LTR', (WidgetTester tester) async {
    OrderPainter.log.clear();
    const Key rowKey = Key('row');
    const Key child0Key = Key('child0');
    const Key child1Key = Key('child1');
    const Key child2Key = Key('child2');

    // Default is MainAxisSize.max so the Row should be as wide as the test: 800.
    // The 100x100 children's left edges should be at 500, 600, 700.
    await tester.pumpWidget(
      Center(
        child: Row(
          key: rowKey,
          textDirection: TextDirection.ltr,
          mainAxisAlignment: MainAxisAlignment.end,
          children: <Widget>[
            SizedBox(key: child0Key, width: 100.0, height: 100.0, child: log(1)),
            SizedBox(key: child1Key, width: 100.0, height: 100.0, child: log(2)),
            SizedBox(key: child2Key, width: 100.0, height: 100.0, child: log(3)),
          ],
        ),
      ),
    );

    RenderBox renderBox;
    BoxParentData boxParentData;

    renderBox = tester.renderObject(find.byKey(rowKey));
    expect(renderBox.size.width, equals(800.0));
    expect(renderBox.size.height, equals(100.0));

    renderBox = tester.renderObject(find.byKey(child0Key));
    expect(renderBox.size.width, equals(100.0));
    expect(renderBox.size.height, equals(100.0));
    boxParentData = renderBox.parentData! as BoxParentData;
    expect(boxParentData.offset.dx, equals(500.0));

    renderBox = tester.renderObject(find.byKey(child1Key));
    expect(renderBox.size.width, equals(100.0));
    expect(renderBox.size.height, equals(100.0));
    boxParentData = renderBox.parentData! as BoxParentData;
    expect(boxParentData.offset.dx, equals(600.0));

    renderBox = tester.renderObject(find.byKey(child2Key));
    expect(renderBox.size.width, equals(100.0));
    expect(renderBox.size.height, equals(100.0));
    boxParentData = renderBox.parentData! as BoxParentData;
    expect(boxParentData.offset.dx, equals(700.0));

    expect(OrderPainter.log, <int>[1, 2, 3]);
  });

  testWidgets('Row with MainAxisAlignment.spaceBetween - LTR', (WidgetTester tester) async {
    OrderPainter.log.clear();
    const Key rowKey = Key('row');
    const Key child0Key = Key('child0');
    const Key child1Key = Key('child1');
    const Key child2Key = Key('child2');

    // Default is MainAxisSize.max so the Row should be as wide as the test: 800.
    // The 100x100 children's left edges should be at 0, 350, 700
    await tester.pumpWidget(
      Center(
        child: Row(
          key: rowKey,
          mainAxisAlignment: MainAxisAlignment.spaceBetween,
          textDirection: TextDirection.ltr,
          children: <Widget>[
            SizedBox(key: child0Key, width: 100.0, height: 100.0, child: log(1)),
            SizedBox(key: child1Key, width: 100.0, height: 100.0, child: log(2)),
            SizedBox(key: child2Key, width: 100.0, height: 100.0, child: log(3)),
          ],
        ),
      ),
    );

    RenderBox renderBox;
    BoxParentData boxParentData;

    renderBox = tester.renderObject(find.byKey(rowKey));
    expect(renderBox.size.width, equals(800.0));
    expect(renderBox.size.height, equals(100.0));

    renderBox = tester.renderObject(find.byKey(child0Key));
    expect(renderBox.size.width, equals(100.0));
    expect(renderBox.size.height, equals(100.0));
    boxParentData = renderBox.parentData! as BoxParentData;
    expect(boxParentData.offset.dx, equals(0.0));

    renderBox = tester.renderObject(find.byKey(child1Key));
    expect(renderBox.size.width, equals(100.0));
    expect(renderBox.size.height, equals(100.0));
    boxParentData = renderBox.parentData! as BoxParentData;
    expect(boxParentData.offset.dx, equals(350.0));

    renderBox = tester.renderObject(find.byKey(child2Key));
    expect(renderBox.size.width, equals(100.0));
    expect(renderBox.size.height, equals(100.0));
    boxParentData = renderBox.parentData! as BoxParentData;
    expect(boxParentData.offset.dx, equals(700.0));

    expect(OrderPainter.log, <int>[1, 2, 3]);
  });

  testWidgets('Row with MainAxisAlignment.spaceAround - LTR', (WidgetTester tester) async {
    OrderPainter.log.clear();
    const Key rowKey = Key('row');
    const Key child0Key = Key('child0');
    const Key child1Key = Key('child1');
    const Key child2Key = Key('child2');
    const Key child3Key = Key('child3');

    // Default is MainAxisSize.max so the Row should be as wide as the test: 800.
    // The 100x100 children's left edges should be at 50, 250, 450, 650
    await tester.pumpWidget(
      Center(
        child: Row(
          key: rowKey,
          mainAxisAlignment: MainAxisAlignment.spaceAround,
          textDirection: TextDirection.ltr,
          children: <Widget>[
            SizedBox(key: child0Key, width: 100.0, height: 100.0, child: log(1)),
            SizedBox(key: child1Key, width: 100.0, height: 100.0, child: log(2)),
            SizedBox(key: child2Key, width: 100.0, height: 100.0, child: log(3)),
            SizedBox(key: child3Key, width: 100.0, height: 100.0, child: log(4)),
          ],
        ),
      ),
    );

    RenderBox renderBox;
    BoxParentData boxParentData;

    renderBox = tester.renderObject(find.byKey(rowKey));
    expect(renderBox.size.width, equals(800.0));
    expect(renderBox.size.height, equals(100.0));

    renderBox = tester.renderObject(find.byKey(child0Key));
    expect(renderBox.size.width, equals(100.0));
    expect(renderBox.size.height, equals(100.0));
    boxParentData = renderBox.parentData! as BoxParentData;
    expect(boxParentData.offset.dx, equals(50.0));

    renderBox = tester.renderObject(find.byKey(child1Key));
    expect(renderBox.size.width, equals(100.0));
    expect(renderBox.size.height, equals(100.0));
    boxParentData = renderBox.parentData! as BoxParentData;
    expect(boxParentData.offset.dx, equals(250.0));

    renderBox = tester.renderObject(find.byKey(child2Key));
    expect(renderBox.size.width, equals(100.0));
    expect(renderBox.size.height, equals(100.0));
    boxParentData = renderBox.parentData! as BoxParentData;
    expect(boxParentData.offset.dx, equals(450.0));

    renderBox = tester.renderObject(find.byKey(child3Key));
    expect(renderBox.size.width, equals(100.0));
    expect(renderBox.size.height, equals(100.0));
    boxParentData = renderBox.parentData! as BoxParentData;
    expect(boxParentData.offset.dx, equals(650.0));

    expect(OrderPainter.log, <int>[1, 2, 3, 4]);
  });

  testWidgets('Row with MainAxisAlignment.spaceEvenly - LTR', (WidgetTester tester) async {
    OrderPainter.log.clear();
    const Key rowKey = Key('row');
    const Key child0Key = Key('child0');
    const Key child1Key = Key('child1');
    const Key child2Key = Key('child2');

    // Default is MainAxisSize.max so the Row should be as wide as the test: 800.
    // The 200x100 children's left edges should be at 50, 300, 550
    await tester.pumpWidget(
      Center(
        child: Row(
          key: rowKey,
          mainAxisAlignment: MainAxisAlignment.spaceEvenly,
          textDirection: TextDirection.ltr,
          children: <Widget>[
            SizedBox(key: child0Key, width: 200.0, height: 100.0, child: log(1)),
            SizedBox(key: child1Key, width: 200.0, height: 100.0, child: log(2)),
            SizedBox(key: child2Key, width: 200.0, height: 100.0, child: log(3)),
          ],
        ),
      ),
    );

    RenderBox renderBox;
    BoxParentData boxParentData;

    renderBox = tester.renderObject(find.byKey(rowKey));
    expect(renderBox.size.width, equals(800.0));
    expect(renderBox.size.height, equals(100.0));

    renderBox = tester.renderObject(find.byKey(child0Key));
    expect(renderBox.size.width, equals(200.0));
    expect(renderBox.size.height, equals(100.0));
    boxParentData = renderBox.parentData! as BoxParentData;
    expect(boxParentData.offset.dx, equals(50.0));

    renderBox = tester.renderObject(find.byKey(child1Key));
    expect(renderBox.size.width, equals(200.0));
    expect(renderBox.size.height, equals(100.0));
    boxParentData = renderBox.parentData! as BoxParentData;
    expect(boxParentData.offset.dx, equals(300.0));

    renderBox = tester.renderObject(find.byKey(child2Key));
    expect(renderBox.size.width, equals(200.0));
    expect(renderBox.size.height, equals(100.0));
    boxParentData = renderBox.parentData! as BoxParentData;
    expect(boxParentData.offset.dx, equals(550.0));

    expect(OrderPainter.log, <int>[1, 2, 3]);
  });

  testWidgets('Row and MainAxisSize.min - LTR', (WidgetTester tester) async {
    OrderPainter.log.clear();
    const Key rowKey = Key('rowKey');
    const Key child0Key = Key('child0');
    const Key child1Key = Key('child1');

    // Row with MainAxisSize.min without flexible children shrink wraps.
    // Row's width should be 250, children should be at 0, 100.
    await tester.pumpWidget(
      Center(
        child: Row(
          key: rowKey,
          mainAxisSize: MainAxisSize.min,
          textDirection: TextDirection.ltr,
          children: <Widget>[
            SizedBox(key: child0Key, width: 100.0, height: 100.0, child: log(1)),
            SizedBox(key: child1Key, width: 150.0, height: 100.0, child: log(2)),
          ],
        ),
      ),
    );

    RenderBox renderBox;
    BoxParentData boxParentData;

    renderBox = tester.renderObject(find.byKey(rowKey));
    expect(renderBox.size.width, equals(250.0));
    expect(renderBox.size.height, equals(100.0));

    renderBox = tester.renderObject(find.byKey(child0Key));
    expect(renderBox.size.width, equals(100.0));
    expect(renderBox.size.height, equals(100.0));
    boxParentData = renderBox.parentData! as BoxParentData;
    expect(boxParentData.offset.dx, equals(0.0));

    renderBox = tester.renderObject(find.byKey(child1Key));
    expect(renderBox.size.width, equals(150.0));
    expect(renderBox.size.height, equals(100.0));
    boxParentData = renderBox.parentData! as BoxParentData;
    expect(boxParentData.offset.dx, equals(100.0));

    expect(OrderPainter.log, <int>[1, 2]);
  });

  testWidgets('Row MainAxisSize.min layout at zero size - LTR', (WidgetTester tester) async {
    OrderPainter.log.clear();
    const Key childKey = Key('childKey');

    await tester.pumpWidget(
      const Center(
        child: SizedBox.shrink(
          child: Row(
            textDirection: TextDirection.ltr,
            mainAxisSize: MainAxisSize.min,
            mainAxisAlignment: MainAxisAlignment.center,
            children: <Widget>[SizedBox(key: childKey, width: 100.0, height: 100.0)],
          ),
        ),
      ),
    );

    final RenderBox renderBox = tester.renderObject(find.byKey(childKey));
    expect(renderBox.size.width, equals(100.0));
    expect(renderBox.size.height, equals(0.0));
  });

  // RTL

  testWidgets('Row with one Flexible child - RTL', (WidgetTester tester) async {
    OrderPainter.log.clear();
    const Key rowKey = Key('row');
    const Key child0Key = Key('child0');
    const Key child1Key = Key('child1');
    const Key child2Key = Key('child2');

    // Default is MainAxisSize.max so the Row should be as wide as the test: 800.
    // Default is MainAxisAlignment.start so children so the children's
    // right edges should be at 0, 100, 700 from the right, child2's width should be 600
    await tester.pumpWidget(
      Center(
        child: Row(
          key: rowKey,
          textDirection: TextDirection.rtl,
          children: <Widget>[
            SizedBox(key: child0Key, width: 100.0, height: 100.0, child: log(1)),
            Expanded(child: SizedBox(key: child1Key, width: 100.0, height: 100.0, child: log(2))),
            SizedBox(key: child2Key, width: 100.0, height: 100.0, child: log(3)),
          ],
        ),
      ),
    );

    RenderBox renderBox;
    BoxParentData boxParentData;

    renderBox = tester.renderObject(find.byKey(rowKey));
    expect(renderBox.size.width, equals(800.0));
    expect(renderBox.size.height, equals(100.0));

    renderBox = tester.renderObject(find.byKey(child0Key));
    expect(renderBox.size.width, equals(100.0));
    expect(renderBox.size.height, equals(100.0));
    boxParentData = renderBox.parentData! as BoxParentData;
    expect(boxParentData.offset.dx, equals(700.0));

    renderBox = tester.renderObject(find.byKey(child1Key));
    expect(renderBox.size.width, equals(600.0));
    expect(renderBox.size.height, equals(100.0));
    boxParentData = renderBox.parentData! as BoxParentData;
    expect(boxParentData.offset.dx, equals(100.0));

    renderBox = tester.renderObject(find.byKey(child2Key));
    expect(renderBox.size.width, equals(100.0));
    expect(renderBox.size.height, equals(100.0));
    boxParentData = renderBox.parentData! as BoxParentData;
    expect(boxParentData.offset.dx, equals(0.0));

    expect(OrderPainter.log, <int>[1, 2, 3]);
  });

  testWidgets('Row with default main axis parameters - RTL', (WidgetTester tester) async {
    OrderPainter.log.clear();
    const Key rowKey = Key('row');
    const Key child0Key = Key('child0');
    const Key child1Key = Key('child1');
    const Key child2Key = Key('child2');

    // Default is MainAxisSize.max so the Row should be as wide as the test: 800.
    // Default is MainAxisAlignment.start so children so the children's
    // right edges should be at 0, 100, 200 from the right
    await tester.pumpWidget(
      Center(
        child: Row(
          key: rowKey,
          textDirection: TextDirection.rtl,
          children: <Widget>[
            SizedBox(key: child0Key, width: 100.0, height: 100.0, child: log(1)),
            SizedBox(key: child1Key, width: 100.0, height: 100.0, child: log(2)),
            SizedBox(key: child2Key, width: 100.0, height: 100.0, child: log(3)),
          ],
        ),
      ),
    );

    RenderBox renderBox;
    BoxParentData boxParentData;

    renderBox = tester.renderObject(find.byKey(rowKey));
    expect(renderBox.size.width, equals(800.0));
    expect(renderBox.size.height, equals(100.0));

    renderBox = tester.renderObject(find.byKey(child0Key));
    expect(renderBox.size.width, equals(100.0));
    expect(renderBox.size.height, equals(100.0));
    boxParentData = renderBox.parentData! as BoxParentData;
    expect(boxParentData.offset.dx, equals(700.0));

    renderBox = tester.renderObject(find.byKey(child1Key));
    expect(renderBox.size.width, equals(100.0));
    expect(renderBox.size.height, equals(100.0));
    boxParentData = renderBox.parentData! as BoxParentData;
    expect(boxParentData.offset.dx, equals(600.0));

    renderBox = tester.renderObject(find.byKey(child2Key));
    expect(renderBox.size.width, equals(100.0));
    expect(renderBox.size.height, equals(100.0));
    boxParentData = renderBox.parentData! as BoxParentData;
    expect(boxParentData.offset.dx, equals(500.0));

    expect(OrderPainter.log, <int>[1, 2, 3]);
  });

  testWidgets('Row with MainAxisAlignment.center - RTL', (WidgetTester tester) async {
    OrderPainter.log.clear();
    const Key rowKey = Key('row');
    const Key child0Key = Key('child0');
    const Key child1Key = Key('child1');

    // Default is MainAxisSize.max so the Row should be as wide as the test: 800.
    // The 100x100 children's right edges should be at 300, 400 from the right
    await tester.pumpWidget(
      Center(
        child: Row(
          key: rowKey,
          mainAxisAlignment: MainAxisAlignment.center,
          textDirection: TextDirection.rtl,
          children: <Widget>[
            SizedBox(key: child0Key, width: 100.0, height: 100.0, child: log(1)),
            SizedBox(key: child1Key, width: 100.0, height: 100.0, child: log(2)),
          ],
        ),
      ),
    );

    RenderBox renderBox;
    BoxParentData boxParentData;

    renderBox = tester.renderObject(find.byKey(rowKey));
    expect(renderBox.size.width, equals(800.0));
    expect(renderBox.size.height, equals(100.0));

    renderBox = tester.renderObject(find.byKey(child0Key));
    expect(renderBox.size.width, equals(100.0));
    expect(renderBox.size.height, equals(100.0));
    boxParentData = renderBox.parentData! as BoxParentData;
    expect(boxParentData.offset.dx, equals(400.0));

    renderBox = tester.renderObject(find.byKey(child1Key));
    expect(renderBox.size.width, equals(100.0));
    expect(renderBox.size.height, equals(100.0));
    boxParentData = renderBox.parentData! as BoxParentData;
    expect(boxParentData.offset.dx, equals(300.0));

    expect(OrderPainter.log, <int>[1, 2]);
  });

  testWidgets('Row with MainAxisAlignment.end - RTL', (WidgetTester tester) async {
    OrderPainter.log.clear();
    const Key rowKey = Key('row');
    const Key child0Key = Key('child0');
    const Key child1Key = Key('child1');
    const Key child2Key = Key('child2');

    // Default is MainAxisSize.max so the Row should be as wide as the test: 800.
    // The 100x100 children's right edges should be at 500, 600, 700 from the right.
    await tester.pumpWidget(
      Center(
        child: Row(
          key: rowKey,
          textDirection: TextDirection.rtl,
          mainAxisAlignment: MainAxisAlignment.end,
          children: <Widget>[
            SizedBox(key: child0Key, width: 100.0, height: 100.0, child: log(1)),
            SizedBox(key: child1Key, width: 100.0, height: 100.0, child: log(2)),
            SizedBox(key: child2Key, width: 100.0, height: 100.0, child: log(3)),
          ],
        ),
      ),
    );

    RenderBox renderBox;
    BoxParentData boxParentData;

    renderBox = tester.renderObject(find.byKey(rowKey));
    expect(renderBox.size.width, equals(800.0));
    expect(renderBox.size.height, equals(100.0));

    renderBox = tester.renderObject(find.byKey(child0Key));
    expect(renderBox.size.width, equals(100.0));
    expect(renderBox.size.height, equals(100.0));
    boxParentData = renderBox.parentData! as BoxParentData;
    expect(boxParentData.offset.dx, equals(200.0));

    renderBox = tester.renderObject(find.byKey(child1Key));
    expect(renderBox.size.width, equals(100.0));
    expect(renderBox.size.height, equals(100.0));
    boxParentData = renderBox.parentData! as BoxParentData;
    expect(boxParentData.offset.dx, equals(100.0));

    renderBox = tester.renderObject(find.byKey(child2Key));
    expect(renderBox.size.width, equals(100.0));
    expect(renderBox.size.height, equals(100.0));
    boxParentData = renderBox.parentData! as BoxParentData;
    expect(boxParentData.offset.dx, equals(0.0));

    expect(OrderPainter.log, <int>[1, 2, 3]);
  });

  testWidgets('Row with MainAxisAlignment.spaceBetween - RTL', (WidgetTester tester) async {
    OrderPainter.log.clear();
    const Key rowKey = Key('row');
    const Key child0Key = Key('child0');
    const Key child1Key = Key('child1');
    const Key child2Key = Key('child2');

    // Default is MainAxisSize.max so the Row should be as wide as the test: 800.
    // The 100x100 children's right edges should be at 0, 350, 700 from the right
    await tester.pumpWidget(
      Center(
        child: Row(
          key: rowKey,
          mainAxisAlignment: MainAxisAlignment.spaceBetween,
          textDirection: TextDirection.rtl,
          children: <Widget>[
            SizedBox(key: child0Key, width: 100.0, height: 100.0, child: log(1)),
            SizedBox(key: child1Key, width: 100.0, height: 100.0, child: log(2)),
            SizedBox(key: child2Key, width: 100.0, height: 100.0, child: log(3)),
          ],
        ),
      ),
    );

    RenderBox renderBox;
    BoxParentData boxParentData;

    renderBox = tester.renderObject(find.byKey(rowKey));
    expect(renderBox.size.width, equals(800.0));
    expect(renderBox.size.height, equals(100.0));

    renderBox = tester.renderObject(find.byKey(child0Key));
    expect(renderBox.size.width, equals(100.0));
    expect(renderBox.size.height, equals(100.0));
    boxParentData = renderBox.parentData! as BoxParentData;
    expect(boxParentData.offset.dx, equals(700.0));

    renderBox = tester.renderObject(find.byKey(child1Key));
    expect(renderBox.size.width, equals(100.0));
    expect(renderBox.size.height, equals(100.0));
    boxParentData = renderBox.parentData! as BoxParentData;
    expect(boxParentData.offset.dx, equals(350.0));

    renderBox = tester.renderObject(find.byKey(child2Key));
    expect(renderBox.size.width, equals(100.0));
    expect(renderBox.size.height, equals(100.0));
    boxParentData = renderBox.parentData! as BoxParentData;
    expect(boxParentData.offset.dx, equals(0.0));

    expect(OrderPainter.log, <int>[1, 2, 3]);
  });

  testWidgets('Row with MainAxisAlignment.spaceAround - RTL', (WidgetTester tester) async {
    OrderPainter.log.clear();
    const Key rowKey = Key('row');
    const Key child0Key = Key('child0');
    const Key child1Key = Key('child1');
    const Key child2Key = Key('child2');
    const Key child3Key = Key('child3');

    // Default is MainAxisSize.max so the Row should be as wide as the test: 800.
    // The 100x100 children's right edges should be at 50, 250, 450, 650 from the right
    await tester.pumpWidget(
      Center(
        child: Row(
          key: rowKey,
          mainAxisAlignment: MainAxisAlignment.spaceAround,
          textDirection: TextDirection.rtl,
          children: <Widget>[
            SizedBox(key: child0Key, width: 100.0, height: 100.0, child: log(1)),
            SizedBox(key: child1Key, width: 100.0, height: 100.0, child: log(2)),
            SizedBox(key: child2Key, width: 100.0, height: 100.0, child: log(3)),
            SizedBox(key: child3Key, width: 100.0, height: 100.0, child: log(4)),
          ],
        ),
      ),
    );

    RenderBox renderBox;
    BoxParentData boxParentData;

    renderBox = tester.renderObject(find.byKey(rowKey));
    expect(renderBox.size.width, equals(800.0));
    expect(renderBox.size.height, equals(100.0));

    renderBox = tester.renderObject(find.byKey(child0Key));
    expect(renderBox.size.width, equals(100.0));
    expect(renderBox.size.height, equals(100.0));
    boxParentData = renderBox.parentData! as BoxParentData;
    expect(boxParentData.offset.dx, equals(650.0));

    renderBox = tester.renderObject(find.byKey(child1Key));
    expect(renderBox.size.width, equals(100.0));
    expect(renderBox.size.height, equals(100.0));
    boxParentData = renderBox.parentData! as BoxParentData;
    expect(boxParentData.offset.dx, equals(450.0));

    renderBox = tester.renderObject(find.byKey(child2Key));
    expect(renderBox.size.width, equals(100.0));
    expect(renderBox.size.height, equals(100.0));
    boxParentData = renderBox.parentData! as BoxParentData;
    expect(boxParentData.offset.dx, equals(250.0));

    renderBox = tester.renderObject(find.byKey(child3Key));
    expect(renderBox.size.width, equals(100.0));
    expect(renderBox.size.height, equals(100.0));
    boxParentData = renderBox.parentData! as BoxParentData;
    expect(boxParentData.offset.dx, equals(50.0));

    expect(OrderPainter.log, <int>[1, 2, 3, 4]);
  });

  testWidgets('Row with MainAxisAlignment.spaceEvenly - RTL', (WidgetTester tester) async {
    OrderPainter.log.clear();
    const Key rowKey = Key('row');
    const Key child0Key = Key('child0');
    const Key child1Key = Key('child1');
    const Key child2Key = Key('child2');

    // Default is MainAxisSize.max so the Row should be as wide as the test: 800.
    // The 200x100 children's right edges should be at 50, 300, 550 from the right
    await tester.pumpWidget(
      Center(
        child: Row(
          key: rowKey,
          mainAxisAlignment: MainAxisAlignment.spaceEvenly,
          textDirection: TextDirection.rtl,
          children: <Widget>[
            SizedBox(key: child0Key, width: 200.0, height: 100.0, child: log(1)),
            SizedBox(key: child1Key, width: 200.0, height: 100.0, child: log(2)),
            SizedBox(key: child2Key, width: 200.0, height: 100.0, child: log(3)),
          ],
        ),
      ),
    );

    RenderBox renderBox;
    BoxParentData boxParentData;

    renderBox = tester.renderObject(find.byKey(rowKey));
    expect(renderBox.size.width, equals(800.0));
    expect(renderBox.size.height, equals(100.0));

    renderBox = tester.renderObject(find.byKey(child0Key));
    expect(renderBox.size.width, equals(200.0));
    expect(renderBox.size.height, equals(100.0));
    boxParentData = renderBox.parentData! as BoxParentData;
    expect(boxParentData.offset.dx, equals(550.0));

    renderBox = tester.renderObject(find.byKey(child1Key));
    expect(renderBox.size.width, equals(200.0));
    expect(renderBox.size.height, equals(100.0));
    boxParentData = renderBox.parentData! as BoxParentData;
    expect(boxParentData.offset.dx, equals(300.0));

    renderBox = tester.renderObject(find.byKey(child2Key));
    expect(renderBox.size.width, equals(200.0));
    expect(renderBox.size.height, equals(100.0));
    boxParentData = renderBox.parentData! as BoxParentData;
    expect(boxParentData.offset.dx, equals(50.0));

    expect(OrderPainter.log, <int>[1, 2, 3]);
  });

  testWidgets('Row and MainAxisSize.min - RTL', (WidgetTester tester) async {
    OrderPainter.log.clear();
    const Key rowKey = Key('rowKey');
    const Key child0Key = Key('child0');
    const Key child1Key = Key('child1');

    // Row with MainAxisSize.min without flexible children shrink wraps.
    // Row's width should be 250, children should be at 0, 100 from right.
    await tester.pumpWidget(
      Center(
        child: Row(
          key: rowKey,
          mainAxisSize: MainAxisSize.min,
          textDirection: TextDirection.rtl,
          children: <Widget>[
            SizedBox(key: child0Key, width: 100.0, height: 100.0, child: log(1)),
            SizedBox(key: child1Key, width: 150.0, height: 100.0, child: log(2)),
          ],
        ),
      ),
    );

    RenderBox renderBox;
    BoxParentData boxParentData;

    renderBox = tester.renderObject(find.byKey(rowKey));
    expect(renderBox.size.width, equals(250.0));
    expect(renderBox.size.height, equals(100.0));

    renderBox = tester.renderObject(find.byKey(child0Key));
    expect(renderBox.size.width, equals(100.0));
    expect(renderBox.size.height, equals(100.0));
    boxParentData = renderBox.parentData! as BoxParentData;
    expect(boxParentData.offset.dx, equals(150.0));

    renderBox = tester.renderObject(find.byKey(child1Key));
    expect(renderBox.size.width, equals(150.0));
    expect(renderBox.size.height, equals(100.0));
    boxParentData = renderBox.parentData! as BoxParentData;
    expect(boxParentData.offset.dx, equals(0.0));

    expect(OrderPainter.log, <int>[1, 2]);
  });

  testWidgets('Row MainAxisSize.min layout at zero size - RTL', (WidgetTester tester) async {
    OrderPainter.log.clear();
    const Key childKey = Key('childKey');

    await tester.pumpWidget(
      const Center(
        child: SizedBox.shrink(
          child: Row(
            textDirection: TextDirection.rtl,
            mainAxisSize: MainAxisSize.min,
            mainAxisAlignment: MainAxisAlignment.center,
            children: <Widget>[SizedBox(key: childKey, width: 100.0, height: 100.0)],
          ),
        ),
      ),
    );

    final RenderBox renderBox = tester.renderObject(find.byKey(childKey));
    expect(renderBox.size.width, equals(100.0));
    expect(renderBox.size.height, equals(0.0));
  });

  testWidgets('Default Row.spacing value', (WidgetTester tester) async {
    await tester.pumpWidget(const Directionality(textDirection: TextDirection.ltr, child: Row()));

    expect(tester.widget<Row>(find.byType(Row)).spacing, 0.0);
  });

  testWidgets('Can update Row.spacing value', (WidgetTester tester) async {
    Widget buildRow({required double spacing}) {
      return Center(
        child: Directionality(
          textDirection: TextDirection.ltr,
          child: Row(
            spacing: spacing,
            mainAxisSize: MainAxisSize.min,
            children: <Widget>[
              Container(height: 100.0, width: 100.0, color: const Color(0xFFFF0000)),
              Container(height: 100.0, width: 100.0, color: const Color(0xFF0000FF)),
              Container(height: 100.0, width: 100.0, color: const Color(0xff00FF00)),
            ],
          ),
        ),
      );
    }

    await tester.pumpWidget(buildRow(spacing: 8.0));

    expect(tester.widget<Row>(find.byType(Row)).spacing, equals(8.0));
    expect(tester.getSize(find.byType(Row)).width, equals(316.0));
    expect(tester.getSize(find.byType(Row)).height, equals(100.0));

    await tester.pumpWidget(buildRow(spacing: 18.0));

    expect(tester.widget<Row>(find.byType(Row)).spacing, equals(18.0));
    expect(tester.getSize(find.byType(Row)).width, equals(336.0));
    expect(tester.getSize(find.byType(Row)).height, equals(100.0));
  });
}
