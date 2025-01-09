// Copyright 2014 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

import 'package:flutter/rendering.dart';
import 'package:flutter/widgets.dart';
import 'package:flutter_test/flutter_test.dart';

class TestCustomPainter extends CustomPainter {
  TestCustomPainter({required this.log, this.name});

  final List<String?> log;
  final String? name;

  @override
  void paint(Canvas canvas, Size size) {
    log.add(name);
  }

  @override
  bool shouldRepaint(TestCustomPainter oldPainter) => true;
}

class MockCanvas extends Fake implements Canvas {
  int saveCount = 0;
  int saveCountDelta = 1;

  @override
  int getSaveCount() {
    return saveCount += saveCountDelta;
  }

  @override
  void save() {}
}

class MockPaintingContext extends Fake implements PaintingContext {
  @override
  final MockCanvas canvas = MockCanvas();
}

void main() {
  testWidgets('Control test for custom painting', (WidgetTester tester) async {
    final List<String?> log = <String?>[];
    await tester.pumpWidget(
      CustomPaint(
        painter: TestCustomPainter(log: log, name: 'background'),
        foregroundPainter: TestCustomPainter(log: log, name: 'foreground'),
        child: CustomPaint(painter: TestCustomPainter(log: log, name: 'child')),
      ),
    );

    expect(log, equals(<String>['background', 'child', 'foreground']));
  });

  testWidgets('Throws FlutterError on custom painter incorrect restore/save calls', (
    WidgetTester tester,
  ) async {
    final GlobalKey target = GlobalKey();
    final List<String?> log = <String?>[];
    await tester.pumpWidget(
      CustomPaint(key: target, isComplex: true, painter: TestCustomPainter(log: log)),
    );
    final RenderCustomPaint renderCustom =
        target.currentContext!.findRenderObject()! as RenderCustomPaint;
    final MockPaintingContext paintingContext = MockPaintingContext();
    final MockCanvas canvas = paintingContext.canvas;

    FlutterError getError() {
      late FlutterError error;
      try {
        renderCustom.paint(paintingContext, Offset.zero);
      } on FlutterError catch (e) {
        error = e;
      }
      return error;
    }

    FlutterError error = getError();
    expect(
      error.toStringDeep(),
      equalsIgnoringHashCodes(
        'FlutterError\n'
        '   The TestCustomPainter#00000() custom painter called canvas.save()\n'
        '   or canvas.saveLayer() at least 1 more time than it called\n'
        '   canvas.restore().\n'
        '   This leaves the canvas in an inconsistent state and will probably\n'
        '   result in a broken display.\n'
        '   You must pair each call to save()/saveLayer() with a later\n'
        '   matching call to restore().\n',
      ),
    );

    canvas.saveCountDelta = -1;
    error = getError();
    expect(
      error.toStringDeep(),
      equalsIgnoringHashCodes(
        'FlutterError\n'
        '   The TestCustomPainter#00000() custom painter called\n'
        '   canvas.restore() 1 more time than it called canvas.save() or\n'
        '   canvas.saveLayer().\n'
        '   This leaves the canvas in an inconsistent state and will result\n'
        '   in a broken display.\n'
        '   You should only call restore() if you first called save() or\n'
        '   saveLayer().\n',
      ),
    );

    canvas.saveCountDelta = 2;
    error = getError();
    expect(error.toStringDeep(), contains('2 more times'));

    canvas.saveCountDelta = -2;
    error = getError();
    expect(error.toStringDeep(), contains('2 more times'));
  });

  testWidgets('CustomPaint sizing', (WidgetTester tester) async {
    final GlobalKey target = GlobalKey();

    await tester.pumpWidget(Center(child: CustomPaint(key: target)));
    expect(target.currentContext!.size, Size.zero);

    await tester.pumpWidget(Center(child: CustomPaint(key: target, child: Container())));
    expect(target.currentContext!.size, const Size(800.0, 600.0));

    await tester.pumpWidget(Center(child: CustomPaint(key: target, size: const Size(20.0, 20.0))));
    expect(target.currentContext!.size, const Size(20.0, 20.0));

    await tester.pumpWidget(
      Center(child: CustomPaint(key: target, size: const Size(2000.0, 100.0))),
    );
    expect(target.currentContext!.size, const Size(800.0, 100.0));

    await tester.pumpWidget(Center(child: CustomPaint(key: target, child: Container())));
    expect(target.currentContext!.size, const Size(800.0, 600.0));

    await tester.pumpWidget(
      Center(child: CustomPaint(key: target, child: const SizedBox.shrink())),
    );
    expect(target.currentContext!.size, Size.zero);
  });

  testWidgets('Raster cache hints', (WidgetTester tester) async {
    final GlobalKey target = GlobalKey();

    final List<String?> log = <String?>[];
    await tester.pumpWidget(
      CustomPaint(key: target, isComplex: true, painter: TestCustomPainter(log: log)),
    );
    RenderCustomPaint renderCustom =
        target.currentContext!.findRenderObject()! as RenderCustomPaint;
    expect(renderCustom.isComplex, true);
    expect(renderCustom.willChange, false);

    await tester.pumpWidget(
      CustomPaint(key: target, willChange: true, foregroundPainter: TestCustomPainter(log: log)),
    );
    renderCustom = target.currentContext!.findRenderObject()! as RenderCustomPaint;
    expect(renderCustom.isComplex, false);
    expect(renderCustom.willChange, true);
  });

  test('Raster cache hints cannot be set with null painters', () {
    expect(() => CustomPaint(isComplex: true), throwsAssertionError);
    expect(() => CustomPaint(willChange: true), throwsAssertionError);
  });

  test('RenderCustomPaint consults preferred size for intrinsics when it has no child', () {
    final RenderCustomPaint inner = RenderCustomPaint(preferredSize: const Size(20, 30));
    expect(inner.getMinIntrinsicWidth(double.infinity), 20);
    expect(inner.getMaxIntrinsicWidth(double.infinity), 20);
    expect(inner.getMinIntrinsicHeight(double.infinity), 30);
    expect(inner.getMaxIntrinsicHeight(double.infinity), 30);
  });

  test('RenderCustomPaint does not return infinity for its intrinsics', () {
    final RenderCustomPaint inner = RenderCustomPaint(preferredSize: Size.infinite);
    expect(inner.getMinIntrinsicWidth(double.infinity), 0);
    expect(inner.getMaxIntrinsicWidth(double.infinity), 0);
    expect(inner.getMinIntrinsicHeight(double.infinity), 0);
    expect(inner.getMaxIntrinsicHeight(double.infinity), 0);
  });
}
