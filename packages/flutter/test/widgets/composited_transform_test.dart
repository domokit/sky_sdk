// Copyright 2014 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

import 'dart:ui' as ui;

import 'package:flutter/rendering.dart';
import 'package:flutter/widgets.dart';
import 'package:flutter_test/flutter_test.dart';

void main() {
  final LayerLink link = LayerLink();

  testWidgets('Change link during layout', (WidgetTester tester) async {
    final GlobalKey key = GlobalKey();
    Widget build({LayerLink? linkToUse}) {
      return Directionality(
        textDirection: TextDirection.ltr,
        // The LayoutBuilder forces the CompositedTransformTarget widget to
        // access its own size when [RenderObject.debugActiveLayout] is
        // non-null.
        child: LayoutBuilder(
          builder: (BuildContext context, BoxConstraints constraints) {
            return Stack(
              children: <Widget>[
                Positioned(
                  left: 123.0,
                  top: 456.0,
                  child: CompositedTransformTarget(
                    link: linkToUse ?? link,
                    child: const SizedBox(height: 10.0, width: 10.0),
                  ),
                ),
                Positioned(
                  left: 787.0,
                  top: 343.0,
                  child: CompositedTransformFollower(
                    link: linkToUse ?? link,
                    targetAnchor: Alignment.center,
                    followerAnchor: Alignment.center,
                    child: SizedBox(key: key, height: 20.0, width: 20.0),
                  ),
                ),
              ],
            );
          },
        ),
      );
    }

    await tester.pumpWidget(build());
    final RenderBox box = key.currentContext!.findRenderObject()! as RenderBox;
    expect(box.localToGlobal(Offset.zero), const Offset(118.0, 451.0));

    await tester.pumpWidget(build(linkToUse: LayerLink()));
    expect(box.localToGlobal(Offset.zero), const Offset(118.0, 451.0));
  });

  testWidgets('LeaderLayer should not cause error', (WidgetTester tester) async {
    final LayerLink link = LayerLink();

    Widget buildWidget({
      required double paddingLeft,
      Color siblingColor = const Color(0xff000000),
    }) {
      return Directionality(
        textDirection: TextDirection.ltr,
        child: Stack(
          children: <Widget>[
            Padding(
              padding: EdgeInsets.only(left: paddingLeft),
              child: CompositedTransformTarget(
                link: link,
                child: RepaintBoundary(
                  child: ClipRect(child: Container(color: const Color(0x00ff0000))),
                ),
              ),
            ),
            Positioned.fill(child: RepaintBoundary(child: ColoredBox(color: siblingColor))),
          ],
        ),
      );
    }

    await tester.pumpWidget(buildWidget(paddingLeft: 10));
    await tester.pumpWidget(buildWidget(paddingLeft: 0));
    await tester.pumpWidget(buildWidget(paddingLeft: 0, siblingColor: const Color(0x0000ff00)));
  });

  group('Composited transforms - only offsets', () {
    final GlobalKey key = GlobalKey();

    Widget build({required Alignment targetAlignment, required Alignment followerAlignment}) {
      return Directionality(
        textDirection: TextDirection.ltr,
        child: Stack(
          children: <Widget>[
            Positioned(
              left: 123.0,
              top: 456.0,
              child: CompositedTransformTarget(
                link: link,
                child: const SizedBox(height: 10.0, width: 10.0),
              ),
            ),
            Positioned(
              left: 787.0,
              top: 343.0,
              child: CompositedTransformFollower(
                link: link,
                targetAnchor: targetAlignment,
                followerAnchor: followerAlignment,
                child: SizedBox(key: key, height: 20.0, width: 20.0),
              ),
            ),
          ],
        ),
      );
    }

    testWidgets('topLeft', (WidgetTester tester) async {
      await tester.pumpWidget(
        build(targetAlignment: Alignment.topLeft, followerAlignment: Alignment.topLeft),
      );
      final RenderBox box = key.currentContext!.findRenderObject()! as RenderBox;
      expect(box.localToGlobal(Offset.zero), const Offset(123.0, 456.0));
    });

    testWidgets('center', (WidgetTester tester) async {
      await tester.pumpWidget(
        build(targetAlignment: Alignment.center, followerAlignment: Alignment.center),
      );
      final RenderBox box = key.currentContext!.findRenderObject()! as RenderBox;
      expect(box.localToGlobal(Offset.zero), const Offset(118.0, 451.0));
    });

    testWidgets('bottomRight - topRight', (WidgetTester tester) async {
      await tester.pumpWidget(
        build(targetAlignment: Alignment.bottomRight, followerAlignment: Alignment.topRight),
      );
      final RenderBox box = key.currentContext!.findRenderObject()! as RenderBox;
      expect(box.localToGlobal(Offset.zero), const Offset(113.0, 466.0));
    });
  });

  group('Composited transforms - with rotations', () {
    final GlobalKey key1 = GlobalKey();
    final GlobalKey key2 = GlobalKey();

    Widget build({required Alignment targetAlignment, required Alignment followerAlignment}) {
      return Directionality(
        textDirection: TextDirection.ltr,
        child: Stack(
          children: <Widget>[
            Positioned(
              top: 123.0,
              left: 456.0,
              child: Transform.rotate(
                angle: 1.0, // radians
                child: CompositedTransformTarget(
                  link: link,
                  child: SizedBox(key: key1, width: 80.0, height: 10.0),
                ),
              ),
            ),
            Positioned(
              top: 787.0,
              left: 343.0,
              child: Transform.rotate(
                angle: -0.3, // radians
                child: CompositedTransformFollower(
                  link: link,
                  targetAnchor: targetAlignment,
                  followerAnchor: followerAlignment,
                  child: SizedBox(key: key2, width: 40.0, height: 20.0),
                ),
              ),
            ),
          ],
        ),
      );
    }

    testWidgets('topLeft', (WidgetTester tester) async {
      await tester.pumpWidget(
        build(targetAlignment: Alignment.topLeft, followerAlignment: Alignment.topLeft),
      );
      final RenderBox box1 = key1.currentContext!.findRenderObject()! as RenderBox;
      final RenderBox box2 = key2.currentContext!.findRenderObject()! as RenderBox;
      final Offset position1 = box1.localToGlobal(Offset.zero);
      final Offset position2 = box2.localToGlobal(Offset.zero);
      expect(position1, offsetMoreOrLessEquals(position2));
    });

    testWidgets('center', (WidgetTester tester) async {
      await tester.pumpWidget(
        build(targetAlignment: Alignment.center, followerAlignment: Alignment.center),
      );
      final RenderBox box1 = key1.currentContext!.findRenderObject()! as RenderBox;
      final RenderBox box2 = key2.currentContext!.findRenderObject()! as RenderBox;
      final Offset position1 = box1.localToGlobal(const Offset(40, 5));
      final Offset position2 = box2.localToGlobal(const Offset(20, 10));
      expect(position1, offsetMoreOrLessEquals(position2));
    });

    testWidgets('bottomRight - topRight', (WidgetTester tester) async {
      await tester.pumpWidget(
        build(targetAlignment: Alignment.bottomRight, followerAlignment: Alignment.topRight),
      );
      final RenderBox box1 = key1.currentContext!.findRenderObject()! as RenderBox;
      final RenderBox box2 = key2.currentContext!.findRenderObject()! as RenderBox;
      final Offset position1 = box1.localToGlobal(const Offset(80, 10));
      final Offset position2 = box2.localToGlobal(const Offset(40, 0));
      expect(position1, offsetMoreOrLessEquals(position2));
    });
  });

  group('Composited transforms - nested', () {
    final GlobalKey key1 = GlobalKey();
    final GlobalKey key2 = GlobalKey();

    Widget build({required Alignment targetAlignment, required Alignment followerAlignment}) {
      return Directionality(
        textDirection: TextDirection.ltr,
        child: Stack(
          children: <Widget>[
            Positioned(
              top: 123.0,
              left: 456.0,
              child: Transform.rotate(
                angle: 1.0, // radians
                child: CompositedTransformTarget(
                  link: link,
                  child: SizedBox(key: key1, width: 80.0, height: 10.0),
                ),
              ),
            ),
            Positioned(
              top: 787.0,
              left: 343.0,
              child: Transform.rotate(
                angle: -0.3, // radians
                child: Padding(
                  padding: const EdgeInsets.all(20.0),
                  child: CompositedTransformFollower(
                    link: LayerLink(),
                    child: Transform(
                      transform: Matrix4.skew(0.9, 1.1),
                      child: Padding(
                        padding: const EdgeInsets.all(20.0),
                        child: CompositedTransformFollower(
                          link: link,
                          targetAnchor: targetAlignment,
                          followerAnchor: followerAlignment,
                          child: SizedBox(key: key2, width: 40.0, height: 20.0),
                        ),
                      ),
                    ),
                  ),
                ),
              ),
            ),
          ],
        ),
      );
    }

    testWidgets('topLeft', (WidgetTester tester) async {
      await tester.pumpWidget(
        build(targetAlignment: Alignment.topLeft, followerAlignment: Alignment.topLeft),
      );
      final RenderBox box1 = key1.currentContext!.findRenderObject()! as RenderBox;
      final RenderBox box2 = key2.currentContext!.findRenderObject()! as RenderBox;
      final Offset position1 = box1.localToGlobal(Offset.zero);
      final Offset position2 = box2.localToGlobal(Offset.zero);
      expect(position1, offsetMoreOrLessEquals(position2));
    });

    testWidgets('center', (WidgetTester tester) async {
      await tester.pumpWidget(
        build(targetAlignment: Alignment.center, followerAlignment: Alignment.center),
      );
      final RenderBox box1 = key1.currentContext!.findRenderObject()! as RenderBox;
      final RenderBox box2 = key2.currentContext!.findRenderObject()! as RenderBox;
      final Offset position1 = box1.localToGlobal(Alignment.center.alongSize(const Size(80, 10)));
      final Offset position2 = box2.localToGlobal(Alignment.center.alongSize(const Size(40, 20)));
      expect(position1, offsetMoreOrLessEquals(position2));
    });

    testWidgets('bottomRight - topRight', (WidgetTester tester) async {
      await tester.pumpWidget(
        build(targetAlignment: Alignment.bottomRight, followerAlignment: Alignment.topRight),
      );
      final RenderBox box1 = key1.currentContext!.findRenderObject()! as RenderBox;
      final RenderBox box2 = key2.currentContext!.findRenderObject()! as RenderBox;
      final Offset position1 = box1.localToGlobal(
        Alignment.bottomRight.alongSize(const Size(80, 10)),
      );
      final Offset position2 = box2.localToGlobal(Alignment.topRight.alongSize(const Size(40, 20)));
      expect(position1, offsetMoreOrLessEquals(position2));
    });
  });

  group('Composited transforms - hit testing', () {
    final GlobalKey key1 = GlobalKey();
    final GlobalKey key2 = GlobalKey();
    final GlobalKey key3 = GlobalKey();

    bool tapped = false;

    Widget build({required Alignment targetAlignment, required Alignment followerAlignment}) {
      return Directionality(
        textDirection: TextDirection.ltr,
        child: Stack(
          children: <Widget>[
            Positioned(
              left: 123.0,
              top: 456.0,
              child: CompositedTransformTarget(
                link: link,
                child: SizedBox(key: key1, height: 10.0, width: 10.0),
              ),
            ),
            CompositedTransformFollower(
              link: link,
              child: GestureDetector(
                key: key2,
                behavior: HitTestBehavior.opaque,
                onTap: () {
                  tapped = true;
                },
                child: SizedBox(key: key3, height: 2.0, width: 2.0),
              ),
            ),
          ],
        ),
      );
    }

    const List<Alignment> alignments = <Alignment>[
      Alignment.topLeft,
      Alignment.topRight,
      Alignment.center,
      Alignment.bottomLeft,
      Alignment.bottomRight,
    ];

    setUp(() {
      tapped = false;
    });

    for (final Alignment targetAlignment in alignments) {
      for (final Alignment followerAlignment in alignments) {
        testWidgets('$targetAlignment - $followerAlignment', (WidgetTester tester) async {
          await tester.pumpWidget(
            build(targetAlignment: targetAlignment, followerAlignment: followerAlignment),
          );
          final RenderBox box2 = key2.currentContext!.findRenderObject()! as RenderBox;
          expect(box2.size, const Size(2.0, 2.0));
          expect(tapped, isFalse);
          await tester.tap(
            find.byKey(key3),
            warnIfMissed: false,
          ); // the container itself is transparent to hits
          expect(tapped, isTrue);
        });
      }
    }
  });

  testWidgets('Leader after Follower asserts', (WidgetTester tester) async {
    final LayerLink link = LayerLink();
    await tester.pumpWidget(
      CompositedTransformFollower(
        link: link,
        child: CompositedTransformTarget(link: link, child: const SizedBox(height: 20, width: 20)),
      ),
    );

    expect(
      (tester.takeException() as AssertionError).message,
      contains('LeaderLayer anchor must come before FollowerLayer in paint order'),
    );
  });

  testWidgets(
    '`FollowerLayer` (`CompositedTransformFollower`) has null pointer error when using with some kinds of `Layer`s',
    (WidgetTester tester) async {
      final LayerLink link = LayerLink();
      await tester.pumpWidget(
        CompositedTransformTarget(
          link: link,
          child: CompositedTransformFollower(link: link, child: const _CustomWidget()),
        ),
      );
    },
  );
}

class _CustomWidget extends SingleChildRenderObjectWidget {
  const _CustomWidget();

  @override
  _CustomRenderObject createRenderObject(BuildContext context) => _CustomRenderObject();

  @override
  void updateRenderObject(BuildContext context, _CustomRenderObject renderObject) {}
}

class _CustomRenderObject extends RenderProxyBox {
  _CustomRenderObject({RenderBox? child}) : super(child);

  @override
  void paint(PaintingContext context, Offset offset) {
    if (layer == null) {
      layer = _CustomLayer(computeSomething: _computeSomething);
    } else {
      (layer as _CustomLayer?)?.computeSomething = _computeSomething;
    }

    context.pushLayer(layer!, super.paint, Offset.zero);
  }

  void _computeSomething() {
    // indeed, use `globalToLocal` to compute some useful data
    globalToLocal(Offset.zero);
  }
}

class _CustomLayer extends ContainerLayer {
  _CustomLayer({required this.computeSomething});

  VoidCallback computeSomething;

  @override
  void addToScene(ui.SceneBuilder builder) {
    computeSomething(); // indeed, need to use result of this function
    super.addToScene(builder);
  }
}
