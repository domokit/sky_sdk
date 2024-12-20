// Copyright 2014 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

import 'package:flutter/rendering.dart';
import 'package:flutter/widgets.dart';
import 'package:flutter_test/flutter_test.dart';

void main() {
  testWidgets('Can size according to aspect ratio', (WidgetTester tester) async {
    final Key outside = UniqueKey();
    final Key inside = UniqueKey();

    await tester.pumpWidget(
      Center(
        child: SizedBox(
          width: 200.0,
          child: FittedBox(key: outside, child: SizedBox(key: inside, width: 100.0, height: 50.0)),
        ),
      ),
    );

    final RenderBox outsideBox = tester.firstRenderObject(find.byKey(outside));
    expect(outsideBox.size.width, 200.0);
    expect(outsideBox.size.height, 100.0);

    final RenderBox insideBox = tester.firstRenderObject(find.byKey(inside));
    expect(insideBox.size.width, 100.0);
    expect(insideBox.size.height, 50.0);

    final Offset insidePoint = insideBox.localToGlobal(const Offset(100.0, 50.0));
    final Offset outsidePoint = outsideBox.localToGlobal(const Offset(200.0, 100.0));

    expect(outsidePoint, equals(const Offset(500.0, 350.0)));
    expect(insidePoint, equals(outsidePoint));
  });

  testWidgets('Can contain child', (WidgetTester tester) async {
    final Key outside = UniqueKey();
    final Key inside = UniqueKey();

    await tester.pumpWidget(
      Center(
        child: SizedBox(
          width: 200.0,
          height: 200.0,
          child: FittedBox(key: outside, child: SizedBox(key: inside, width: 100.0, height: 50.0)),
        ),
      ),
    );

    final RenderBox outsideBox = tester.firstRenderObject(find.byKey(outside));
    expect(outsideBox.size.width, 200.0);
    expect(outsideBox.size.height, 200.0);

    final RenderBox insideBox = tester.firstRenderObject(find.byKey(inside));
    expect(insideBox.size.width, 100.0);
    expect(insideBox.size.height, 50.0);

    final Offset insidePoint = insideBox.localToGlobal(const Offset(100.0, 0.0));
    final Offset outsidePoint = outsideBox.localToGlobal(const Offset(200.0, 50.0));

    expect(insidePoint, equals(outsidePoint));
  });

  testWidgets('Child can cover', (WidgetTester tester) async {
    final Key outside = UniqueKey();
    final Key inside = UniqueKey();

    await tester.pumpWidget(
      Center(
        child: SizedBox(
          width: 200.0,
          height: 200.0,
          child: FittedBox(
            key: outside,
            fit: BoxFit.cover,
            child: SizedBox(key: inside, width: 100.0, height: 50.0),
          ),
        ),
      ),
    );

    final RenderBox outsideBox = tester.firstRenderObject(find.byKey(outside));
    expect(outsideBox.size.width, 200.0);
    expect(outsideBox.size.height, 200.0);

    final RenderBox insideBox = tester.firstRenderObject(find.byKey(inside));
    expect(insideBox.size.width, 100.0);
    expect(insideBox.size.height, 50.0);

    final Offset insidePoint = insideBox.localToGlobal(const Offset(50.0, 25.0));
    final Offset outsidePoint = outsideBox.localToGlobal(const Offset(100.0, 100.0));

    expect(insidePoint, equals(outsidePoint));
  });

  testWidgets('FittedBox with no child', (WidgetTester tester) async {
    final Key key = UniqueKey();
    await tester.pumpWidget(Center(child: FittedBox(key: key, fit: BoxFit.cover)));

    final RenderBox box = tester.firstRenderObject(find.byKey(key));
    expect(box.size.width, 0.0);
    expect(box.size.height, 0.0);
  });

  testWidgets('Child can be aligned multiple ways in a row', (WidgetTester tester) async {
    final Key outside = UniqueKey();
    final Key inside = UniqueKey();

    {
      // align RTL

      await tester.pumpWidget(
        Directionality(
          textDirection: TextDirection.rtl,
          child: Center(
            child: SizedBox(
              width: 100.0,
              height: 100.0,
              child: FittedBox(
                key: outside,
                fit: BoxFit.scaleDown,
                alignment: AlignmentDirectional.bottomEnd,
                child: SizedBox(key: inside, width: 10.0, height: 10.0),
              ),
            ),
          ),
        ),
      );

      final RenderBox outsideBox = tester.firstRenderObject(find.byKey(outside));
      expect(outsideBox.size.width, 100.0);
      expect(outsideBox.size.height, 100.0);

      final RenderBox insideBox = tester.firstRenderObject(find.byKey(inside));
      expect(insideBox.size.width, 10.0);
      expect(insideBox.size.height, 10.0);

      final Offset insideTopLeft = insideBox.localToGlobal(Offset.zero);
      final Offset outsideTopLeft = outsideBox.localToGlobal(const Offset(0.0, 90.0));
      final Offset insideBottomRight = insideBox.localToGlobal(const Offset(10.0, 10.0));
      final Offset outsideBottomRight = outsideBox.localToGlobal(const Offset(10.0, 100.0));

      expect(insideTopLeft, equals(outsideTopLeft));
      expect(insideBottomRight, equals(outsideBottomRight));
    }

    {
      // change direction

      await tester.pumpWidget(
        Directionality(
          textDirection: TextDirection.ltr,
          child: Center(
            child: SizedBox(
              width: 100.0,
              height: 100.0,
              child: FittedBox(
                key: outside,
                fit: BoxFit.scaleDown,
                alignment: AlignmentDirectional.bottomEnd,
                child: SizedBox(key: inside, width: 10.0, height: 10.0),
              ),
            ),
          ),
        ),
      );

      final RenderBox outsideBox = tester.firstRenderObject(find.byKey(outside));
      expect(outsideBox.size.width, 100.0);
      expect(outsideBox.size.height, 100.0);

      final RenderBox insideBox = tester.firstRenderObject(find.byKey(inside));
      expect(insideBox.size.width, 10.0);
      expect(insideBox.size.height, 10.0);

      final Offset insideTopLeft = insideBox.localToGlobal(Offset.zero);
      final Offset outsideTopLeft = outsideBox.localToGlobal(const Offset(90.0, 90.0));
      final Offset insideBottomRight = insideBox.localToGlobal(const Offset(10.0, 10.0));
      final Offset outsideBottomRight = outsideBox.localToGlobal(const Offset(100.0, 100.0));

      expect(insideTopLeft, equals(outsideTopLeft));
      expect(insideBottomRight, equals(outsideBottomRight));
    }

    {
      // change alignment

      await tester.pumpWidget(
        Directionality(
          textDirection: TextDirection.ltr,
          child: Center(
            child: SizedBox(
              width: 100.0,
              height: 100.0,
              child: FittedBox(
                key: outside,
                fit: BoxFit.scaleDown,
                alignment: AlignmentDirectional.center,
                child: SizedBox(key: inside, width: 10.0, height: 10.0),
              ),
            ),
          ),
        ),
      );

      final RenderBox outsideBox = tester.firstRenderObject(find.byKey(outside));
      expect(outsideBox.size.width, 100.0);
      expect(outsideBox.size.height, 100.0);

      final RenderBox insideBox = tester.firstRenderObject(find.byKey(inside));
      expect(insideBox.size.width, 10.0);
      expect(insideBox.size.height, 10.0);

      final Offset insideTopLeft = insideBox.localToGlobal(Offset.zero);
      final Offset outsideTopLeft = outsideBox.localToGlobal(const Offset(45.0, 45.0));
      final Offset insideBottomRight = insideBox.localToGlobal(const Offset(10.0, 10.0));
      final Offset outsideBottomRight = outsideBox.localToGlobal(const Offset(55.0, 55.0));

      expect(insideTopLeft, equals(outsideTopLeft));
      expect(insideBottomRight, equals(outsideBottomRight));
    }

    {
      // change size

      await tester.pumpWidget(
        Directionality(
          textDirection: TextDirection.ltr,
          child: Center(
            child: SizedBox(
              width: 100.0,
              height: 100.0,
              child: FittedBox(
                key: outside,
                fit: BoxFit.scaleDown,
                alignment: AlignmentDirectional.center,
                child: SizedBox(key: inside, width: 30.0, height: 10.0),
              ),
            ),
          ),
        ),
      );

      final RenderBox outsideBox = tester.firstRenderObject(find.byKey(outside));
      expect(outsideBox.size.width, 100.0);
      expect(outsideBox.size.height, 100.0);

      final RenderBox insideBox = tester.firstRenderObject(find.byKey(inside));
      expect(insideBox.size.width, 30.0);
      expect(insideBox.size.height, 10.0);

      final Offset insideTopLeft = insideBox.localToGlobal(Offset.zero);
      final Offset outsideTopLeft = outsideBox.localToGlobal(const Offset(35.0, 45.0));
      final Offset insideBottomRight = insideBox.localToGlobal(const Offset(30.0, 10.0));
      final Offset outsideBottomRight = outsideBox.localToGlobal(const Offset(65.0, 55.0));

      expect(insideTopLeft, equals(outsideTopLeft));
      expect(insideBottomRight, equals(outsideBottomRight));
    }

    {
      // change fit

      await tester.pumpWidget(
        Directionality(
          textDirection: TextDirection.ltr,
          child: Center(
            child: SizedBox(
              width: 100.0,
              height: 100.0,
              child: FittedBox(
                key: outside,
                fit: BoxFit.fill,
                alignment: AlignmentDirectional.center,
                child: SizedBox(key: inside, width: 30.0, height: 10.0),
              ),
            ),
          ),
        ),
      );

      final RenderBox outsideBox = tester.firstRenderObject(find.byKey(outside));
      expect(outsideBox.size.width, 100.0);
      expect(outsideBox.size.height, 100.0);

      final RenderBox insideBox = tester.firstRenderObject(find.byKey(inside));
      expect(insideBox.size.width, 30.0);
      expect(insideBox.size.height, 10.0);

      final Offset insideTopLeft = insideBox.localToGlobal(Offset.zero);
      final Offset outsideTopLeft = outsideBox.localToGlobal(Offset.zero);
      final Offset insideBottomRight = insideBox.localToGlobal(const Offset(30.0, 10.0));
      final Offset outsideBottomRight = outsideBox.localToGlobal(const Offset(100.0, 100.0));

      expect(insideTopLeft, equals(outsideTopLeft));
      expect(insideBottomRight, equals(outsideBottomRight));
    }
  });

  testWidgets('FittedBox layers - contain', (WidgetTester tester) async {
    await tester.pumpWidget(
      const Center(
        child: SizedBox(
          width: 100.0,
          height: 10.0,
          child: FittedBox(
            child: SizedBox(
              width: 50.0,
              height: 50.0,
              child: RepaintBoundary(child: Placeholder()),
            ),
          ),
        ),
      ),
    );
    expect(getLayers(), <Type>[TransformLayer, TransformLayer, OffsetLayer]);
  });

  testWidgets('FittedBox layers - cover - horizontal', (WidgetTester tester) async {
    await tester.pumpWidget(
      const Center(
        child: SizedBox(
          width: 100.0,
          height: 10.0,
          child: FittedBox(
            fit: BoxFit.cover,
            clipBehavior: Clip.hardEdge,
            child: SizedBox(
              width: 10.0,
              height: 50.0,
              child: RepaintBoundary(child: Placeholder()),
            ),
          ),
        ),
      ),
    );
    expect(getLayers(), <Type>[TransformLayer, ClipRectLayer, TransformLayer, OffsetLayer]);
  });

  testWidgets('FittedBox layers - cover - vertical', (WidgetTester tester) async {
    await tester.pumpWidget(
      const Center(
        child: SizedBox(
          width: 10.0,
          height: 100.0,
          child: FittedBox(
            fit: BoxFit.cover,
            clipBehavior: Clip.hardEdge,
            child: SizedBox(
              width: 50.0,
              height: 10.0,
              child: RepaintBoundary(child: Placeholder()),
            ),
          ),
        ),
      ),
    );
    expect(getLayers(), <Type>[TransformLayer, ClipRectLayer, TransformLayer, OffsetLayer]);
  });

  testWidgets('FittedBox layers - none - clip', (WidgetTester tester) async {
    final List<double> values = <double>[10.0, 50.0, 100.0];
    for (final double a in values) {
      for (final double b in values) {
        for (final double c in values) {
          for (final double d in values) {
            await tester.pumpWidget(
              Center(
                child: SizedBox(
                  width: a,
                  height: b,
                  child: FittedBox(
                    fit: BoxFit.none,
                    clipBehavior: Clip.hardEdge,
                    child: SizedBox(
                      width: c,
                      height: d,
                      child: const RepaintBoundary(child: Placeholder()),
                    ),
                  ),
                ),
              ),
            );
            if (a < c || b < d) {
              expect(getLayers(), <Type>[TransformLayer, ClipRectLayer, OffsetLayer]);
            } else {
              expect(getLayers(), <Type>[TransformLayer, OffsetLayer]);
            }
          }
        }
      }
    }
  });

  testWidgets('Big child into small fitted box - hit testing', (WidgetTester tester) async {
    final GlobalKey key1 = GlobalKey();
    bool pointerDown = false;
    await tester.pumpWidget(
      Center(
        child: SizedBox(
          width: 100.0,
          height: 100.0,
          child: FittedBox(
            alignment: FractionalOffset.center,
            child: SizedBox(
              width: 1000.0,
              height: 1000.0,
              child: Listener(
                onPointerDown: (PointerDownEvent event) {
                  pointerDown = true;
                },
                child: Container(key: key1, color: const Color(0xFF000000)),
              ),
            ),
          ),
        ),
      ),
    );
    expect(pointerDown, isFalse);
    await tester.tap(find.byKey(key1));
    expect(pointerDown, isTrue);
  });

  testWidgets('Can set and update clipBehavior', (WidgetTester tester) async {
    await tester.pumpWidget(FittedBox(fit: BoxFit.none, child: Container()));
    final RenderFittedBox renderObject = tester.allRenderObjects.whereType<RenderFittedBox>().first;
    expect(renderObject.clipBehavior, equals(Clip.none));

    await tester.pumpWidget(
      FittedBox(fit: BoxFit.none, clipBehavior: Clip.antiAlias, child: Container()),
    );
    expect(renderObject.clipBehavior, equals(Clip.antiAlias));
  });

  testWidgets('BoxFit.scaleDown matches size of child', (WidgetTester tester) async {
    final Key outside = UniqueKey();
    final Key inside = UniqueKey();

    // Does not scale up when child is smaller than constraints

    await tester.pumpWidget(
      Center(
        child: SizedBox(
          width: 200.0,
          child: FittedBox(
            key: outside,
            fit: BoxFit.scaleDown,
            child: SizedBox(key: inside, width: 100.0, height: 50.0),
          ),
        ),
      ),
    );

    final RenderBox outsideBox = tester.firstRenderObject(find.byKey(outside));
    final RenderBox insideBox = tester.firstRenderObject(find.byKey(inside));

    expect(outsideBox.size.width, 200.0);
    expect(outsideBox.size.height, 50.0);

    Offset outsidePoint = outsideBox.localToGlobal(Offset.zero);
    Offset insidePoint = insideBox.localToGlobal(Offset.zero);
    expect(insidePoint - outsidePoint, equals(const Offset(50.0, 0.0)));

    // Scales down when child is bigger than constraints

    await tester.pumpWidget(
      Center(
        child: SizedBox(
          width: 200.0,
          child: FittedBox(
            key: outside,
            fit: BoxFit.scaleDown,
            child: SizedBox(key: inside, width: 400.0, height: 200.0),
          ),
        ),
      ),
    );

    expect(outsideBox.size.width, 200.0);
    expect(outsideBox.size.height, 100.0);

    outsidePoint = outsideBox.localToGlobal(Offset.zero);
    insidePoint = insideBox.localToGlobal(Offset.zero);

    expect(insidePoint - outsidePoint, equals(Offset.zero));
  });

  testWidgets('Switching to and from BoxFit.scaleDown causes relayout', (
    WidgetTester tester,
  ) async {
    final Key outside = UniqueKey();

    final Widget scaleDownWidget = Center(
      child: SizedBox(
        width: 200.0,
        child: FittedBox(
          key: outside,
          fit: BoxFit.scaleDown,
          child: const SizedBox(width: 100.0, height: 50.0),
        ),
      ),
    );

    final Widget coverWidget = Center(
      child: SizedBox(
        width: 200.0,
        child: FittedBox(key: outside, child: const SizedBox(width: 100.0, height: 50.0)),
      ),
    );

    await tester.pumpWidget(scaleDownWidget);

    final RenderBox outsideBox = tester.firstRenderObject(find.byKey(outside));
    expect(outsideBox.size.height, 50.0);

    await tester.pumpWidget(coverWidget);

    expect(outsideBox.size.height, 100.0);

    await tester.pumpWidget(scaleDownWidget);

    expect(outsideBox.size.height, 50.0);
  });

  testWidgets('FittedBox without child does not throw', (WidgetTester tester) async {
    await tester.pumpWidget(
      const Center(child: SizedBox(width: 200.0, height: 200.0, child: FittedBox())),
    );

    expect(find.byType(FittedBox), findsOneWidget);

    // Tapping it also should not throw.
    await tester.tap(find.byType(FittedBox), warnIfMissed: false);
    expect(tester.takeException(), isNull);
  });

  // Regression test for https://github.com/flutter/flutter/issues/135082
  testWidgets('FittedBox with zero size child does not throw', (WidgetTester tester) async {
    await tester.pumpWidget(
      const Center(
        child: SizedBox(
          height: 200.0,
          width: 200.0,
          child: FittedBox(fit: BoxFit.scaleDown, child: SizedBox.shrink()),
        ),
      ),
    );
    expect(tester.takeException(), isNull);

    await tester.pumpWidget(
      Center(
        child: ConstrainedBox(
          constraints: const BoxConstraints(maxWidth: 200.0, maxHeight: 200.0),
          child: const FittedBox(child: SizedBox.shrink()),
        ),
      ),
    );
    expect(tester.takeException(), isNull);
  });
}

List<Type> getLayers() {
  final List<Type> layers = <Type>[];
  Layer? container = RendererBinding.instance.renderView.debugLayer;
  while (container is ContainerLayer) {
    layers.add(container.runtimeType);
    expect(container.firstChild, same(container.lastChild));
    container = container.firstChild;
  }
  return layers;
}
