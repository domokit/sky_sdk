// Copyright 2014 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

import 'package:flutter/rendering.dart';
import 'package:flutter/widgets.dart';
import 'package:flutter_test/flutter_test.dart';

class TestFlowDelegate extends FlowDelegate {
  TestFlowDelegate({required this.startOffset}) : super(repaint: startOffset);

  final Animation<double> startOffset;

  @override
  BoxConstraints getConstraintsForChild(int i, BoxConstraints constraints) {
    return constraints.loosen();
  }

  @override
  void paintChildren(FlowPaintingContext context) {
    double dy = startOffset.value;
    for (int i = 0; i < context.childCount; ++i) {
      context.paintChild(i, transform: Matrix4.translationValues(0.0, dy, 0.0));
      dy += 0.75 * context.getChildSize(i)!.height;
    }
  }

  @override
  bool shouldRepaint(TestFlowDelegate oldDelegate) => startOffset == oldDelegate.startOffset;
}

class OpacityFlowDelegate extends FlowDelegate {
  OpacityFlowDelegate(this.opacity);

  double opacity;

  @override
  void paintChildren(FlowPaintingContext context) {
    for (int i = 0; i < context.childCount; ++i) {
      context.paintChild(i, opacity: opacity);
    }
  }

  @override
  bool shouldRepaint(OpacityFlowDelegate oldDelegate) => opacity != oldDelegate.opacity;
}

// OpacityFlowDelegate that paints one of its children twice
class DuplicatePainterOpacityFlowDelegate extends OpacityFlowDelegate {
  DuplicatePainterOpacityFlowDelegate(super.opacity);

  @override
  void paintChildren(FlowPaintingContext context) {
    for (int i = 0; i < context.childCount; ++i) {
      context.paintChild(i, opacity: opacity);
    }
    if (context.childCount > 0) {
      context.paintChild(0, opacity: opacity);
    }
  }
}

void main() {
  testWidgets('Flow control test', (WidgetTester tester) async {
    final AnimationController startOffset = AnimationController.unbounded(vsync: tester);
    addTearDown(startOffset.dispose);
    final List<int> log = <int>[];

    Widget buildBox(int i) {
      return GestureDetector(
        onTap: () {
          log.add(i);
        },
        child: Container(
          width: 100.0,
          height: 100.0,
          color: const Color(0xFF0000FF),
          child: Text('$i', textDirection: TextDirection.ltr),
        ),
      );
    }

    await tester.pumpWidget(
      Flow(
        delegate: TestFlowDelegate(startOffset: startOffset),
        children: <Widget>[
          buildBox(0),
          buildBox(1),
          buildBox(2),
          buildBox(3),
          buildBox(4),
          buildBox(5),
          buildBox(6),
        ],
      ),
    );

    await tester.tap(find.text('0'));
    expect(log, equals(<int>[0]));
    await tester.tap(find.text('1'));
    expect(log, equals(<int>[0, 1]));
    await tester.tap(find.text('2'));
    expect(log, equals(<int>[0, 1, 2]));

    log.clear();
    await tester.tapAt(const Offset(20.0, 90.0));
    expect(log, equals(<int>[1]));

    startOffset.value = 50.0;
    await tester.pump();

    log.clear();
    await tester.tapAt(const Offset(20.0, 90.0));
    expect(log, equals(<int>[0]));
  });

  testWidgets('paintChild gets called twice', (WidgetTester tester) async {
    await tester.pumpWidget(
      Flow(
        delegate: DuplicatePainterOpacityFlowDelegate(1.0),
        children: const <Widget>[
          SizedBox(width: 100.0, height: 100.0),
          SizedBox(width: 100.0, height: 100.0),
        ],
      ),
    );
    final dynamic exception = tester.takeException();
    expect(exception, isFlutterError);
    final FlutterError error = exception as FlutterError;
    expect(
      error.toStringDeep(),
      equalsIgnoringHashCodes(
        'FlutterError\n'
        '   Cannot call paintChild twice for the same child.\n'
        '   The flow delegate of type DuplicatePainterOpacityFlowDelegate\n'
        '   attempted to paint child 0 multiple times, which is not\n'
        '   permitted.\n',
      ),
    );
  });

  testWidgets('Flow opacity layer', (WidgetTester tester) async {
    const double opacity = 0.2;
    await tester.pumpWidget(
      Flow(
        delegate: OpacityFlowDelegate(opacity),
        children: const <Widget>[SizedBox(width: 100.0, height: 100.0)],
      ),
    );
    ContainerLayer? layer = RendererBinding.instance.renderView.debugLayer;
    while (layer != null && layer is! OpacityLayer) {
      layer = layer.firstChild as ContainerLayer?;
    }
    expect(layer, isA<OpacityLayer>());
    final OpacityLayer? opacityLayer = layer as OpacityLayer?;
    expect(opacityLayer!.alpha, equals(opacity * 255));
    expect(layer!.firstChild, isA<TransformLayer>());
  });

  testWidgets('Flow can set and update clipBehavior', (WidgetTester tester) async {
    const double opacity = 0.2;
    await tester.pumpWidget(
      Flow(
        delegate: OpacityFlowDelegate(opacity),
        children: const <Widget>[SizedBox(width: 100.0, height: 100.0)],
      ),
    );

    // By default, clipBehavior should be Clip.hardEdge
    final RenderFlow renderObject = tester.renderObject(find.byType(Flow));
    expect(renderObject.clipBehavior, equals(Clip.hardEdge));

    for (final Clip clip in Clip.values) {
      await tester.pumpWidget(
        Flow(
          delegate: OpacityFlowDelegate(opacity),
          clipBehavior: clip,
          children: const <Widget>[SizedBox(width: 100.0, height: 100.0)],
        ),
      );
      expect(renderObject.clipBehavior, clip);
    }
  });

  testWidgets('Flow.unwrapped can set and update clipBehavior', (WidgetTester tester) async {
    const double opacity = 0.2;
    await tester.pumpWidget(
      Flow.unwrapped(
        delegate: OpacityFlowDelegate(opacity),
        children: const <Widget>[SizedBox(width: 100.0, height: 100.0)],
      ),
    );

    // By default, clipBehavior should be Clip.hardEdge
    final RenderFlow renderObject = tester.renderObject(find.byType(Flow));
    expect(renderObject.clipBehavior, equals(Clip.hardEdge));

    for (final Clip clip in Clip.values) {
      await tester.pumpWidget(
        Flow.unwrapped(
          delegate: OpacityFlowDelegate(opacity),
          clipBehavior: clip,
          children: const <Widget>[SizedBox(width: 100.0, height: 100.0)],
        ),
      );
      expect(renderObject.clipBehavior, clip);
    }
  });
}
