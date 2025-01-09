// Copyright 2014 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

import 'package:flutter/rendering.dart';
import 'package:flutter/widgets.dart';
import 'package:flutter_test/flutter_test.dart';

final BoxDecoration kBoxDecorationA = BoxDecoration(border: nonconst(null));
final BoxDecoration kBoxDecorationB = BoxDecoration(border: nonconst(null));
final BoxDecoration kBoxDecorationC = BoxDecoration(border: nonconst(null));

class TestWidget extends StatelessWidget {
  const TestWidget({super.key, required this.child});

  final Widget child;

  @override
  Widget build(BuildContext context) => child;
}

class TestOrientedBox extends SingleChildRenderObjectWidget {
  const TestOrientedBox({super.key, super.child});

  Decoration _getDecoration(BuildContext context) {
    return switch (MediaQuery.orientationOf(context)) {
      Orientation.landscape => const BoxDecoration(color: Color(0xFF00FF00)),
      Orientation.portrait => const BoxDecoration(color: Color(0xFF0000FF)),
    };
  }

  @override
  RenderDecoratedBox createRenderObject(BuildContext context) =>
      RenderDecoratedBox(decoration: _getDecoration(context));

  @override
  void updateRenderObject(BuildContext context, RenderDecoratedBox renderObject) {
    renderObject.decoration = _getDecoration(context);
  }
}

class TestNonVisitingWidget extends SingleChildRenderObjectWidget {
  const TestNonVisitingWidget({super.key, required Widget super.child});

  @override
  RenderObject createRenderObject(BuildContext context) => TestNonVisitingRenderObject();
}

class TestNonVisitingRenderObject extends RenderBox with RenderObjectWithChildMixin<RenderBox> {
  @override
  Size computeDryLayout(BoxConstraints constraints) {
    return child!.getDryLayout(constraints);
  }

  @override
  void performLayout() {
    child!.layout(constraints, parentUsesSize: true);
    size = child!.size;
  }

  @override
  void paint(PaintingContext context, Offset offset) {
    context.paintChild(child!, offset);
  }

  @override
  void visitChildren(RenderObjectVisitor visitor) {
    // oops!
  }
}

void main() {
  testWidgets('RenderObjectWidget smoke test', (WidgetTester tester) async {
    await tester.pumpWidget(DecoratedBox(decoration: kBoxDecorationA));
    SingleChildRenderObjectElement element = tester.element(
      find.byElementType(SingleChildRenderObjectElement),
    );
    expect(element, isNotNull);
    expect(element.renderObject, isA<RenderDecoratedBox>());
    RenderDecoratedBox renderObject = element.renderObject as RenderDecoratedBox;
    expect(renderObject.decoration, equals(kBoxDecorationA));
    expect(renderObject.position, equals(DecorationPosition.background));

    await tester.pumpWidget(DecoratedBox(decoration: kBoxDecorationB));
    element = tester.element(find.byElementType(SingleChildRenderObjectElement));
    expect(element, isNotNull);
    expect(element.renderObject, isA<RenderDecoratedBox>());
    renderObject = element.renderObject as RenderDecoratedBox;
    expect(renderObject.decoration, equals(kBoxDecorationB));
    expect(renderObject.position, equals(DecorationPosition.background));
  });

  testWidgets('RenderObjectWidget can add and remove children', (WidgetTester tester) async {
    void checkFullTree() {
      final SingleChildRenderObjectElement element = tester.firstElement(
        find.byElementType(SingleChildRenderObjectElement),
      );
      expect(element, isNotNull);
      expect(element.renderObject, isA<RenderDecoratedBox>());
      final RenderDecoratedBox renderObject = element.renderObject as RenderDecoratedBox;
      expect(renderObject.decoration, equals(kBoxDecorationA));
      expect(renderObject.position, equals(DecorationPosition.background));
      expect(renderObject.child, isNotNull);
      expect(renderObject.child, isA<RenderDecoratedBox>());
      final RenderDecoratedBox child = renderObject.child! as RenderDecoratedBox;
      expect(child.decoration, equals(kBoxDecorationB));
      expect(child.position, equals(DecorationPosition.background));
      expect(child.child, isNull);
    }

    void childBareTree() {
      final SingleChildRenderObjectElement element = tester.element(
        find.byElementType(SingleChildRenderObjectElement),
      );
      expect(element, isNotNull);
      expect(element.renderObject, isA<RenderDecoratedBox>());
      final RenderDecoratedBox renderObject = element.renderObject as RenderDecoratedBox;
      expect(renderObject.decoration, equals(kBoxDecorationA));
      expect(renderObject.position, equals(DecorationPosition.background));
      expect(renderObject.child, isNull);
    }

    await tester.pumpWidget(
      DecoratedBox(decoration: kBoxDecorationA, child: DecoratedBox(decoration: kBoxDecorationB)),
    );

    checkFullTree();

    await tester.pumpWidget(
      DecoratedBox(
        decoration: kBoxDecorationA,
        child: TestWidget(child: DecoratedBox(decoration: kBoxDecorationB)),
      ),
    );

    checkFullTree();

    await tester.pumpWidget(
      DecoratedBox(decoration: kBoxDecorationA, child: DecoratedBox(decoration: kBoxDecorationB)),
    );

    checkFullTree();

    await tester.pumpWidget(DecoratedBox(decoration: kBoxDecorationA));

    childBareTree();

    await tester.pumpWidget(
      DecoratedBox(
        decoration: kBoxDecorationA,
        child: TestWidget(child: TestWidget(child: DecoratedBox(decoration: kBoxDecorationB))),
      ),
    );

    checkFullTree();

    await tester.pumpWidget(DecoratedBox(decoration: kBoxDecorationA));

    childBareTree();
  });

  testWidgets('Detached render tree is intact', (WidgetTester tester) async {
    await tester.pumpWidget(
      DecoratedBox(
        decoration: kBoxDecorationA,
        child: DecoratedBox(
          decoration: kBoxDecorationB,
          child: DecoratedBox(decoration: kBoxDecorationC),
        ),
      ),
    );

    SingleChildRenderObjectElement element = tester.firstElement(
      find.byElementType(SingleChildRenderObjectElement),
    );
    expect(element.renderObject, isA<RenderDecoratedBox>());
    final RenderDecoratedBox parent = element.renderObject as RenderDecoratedBox;
    expect(parent.child, isA<RenderDecoratedBox>());
    final RenderDecoratedBox child = parent.child! as RenderDecoratedBox;
    expect(child.decoration, equals(kBoxDecorationB));
    expect(child.child, isA<RenderDecoratedBox>());
    final RenderDecoratedBox grandChild = child.child! as RenderDecoratedBox;
    expect(grandChild.decoration, equals(kBoxDecorationC));
    expect(grandChild.child, isNull);

    await tester.pumpWidget(DecoratedBox(decoration: kBoxDecorationA));

    element = tester.element(find.byElementType(SingleChildRenderObjectElement));
    expect(element.renderObject, isA<RenderDecoratedBox>());
    expect(element.renderObject, equals(parent));
    expect(parent.child, isNull);

    expect(child.parent, isNull);
    expect(child.decoration, equals(kBoxDecorationB));
    expect(child.child, equals(grandChild));
    expect(grandChild.parent, equals(child));
    expect(grandChild.decoration, equals(kBoxDecorationC));
    expect(grandChild.child, isNull);
  });

  testWidgets('Can watch inherited widgets', (WidgetTester tester) async {
    final Key boxKey = UniqueKey();
    final TestOrientedBox box = TestOrientedBox(key: boxKey);

    await tester.pumpWidget(
      MediaQuery(data: const MediaQueryData(size: Size(400.0, 300.0)), child: box),
    );

    final RenderDecoratedBox renderBox = tester.renderObject(find.byKey(boxKey));
    BoxDecoration decoration = renderBox.decoration as BoxDecoration;
    expect(decoration.color, equals(const Color(0xFF00FF00)));

    await tester.pumpWidget(
      MediaQuery(data: const MediaQueryData(size: Size(300.0, 400.0)), child: box),
    );

    decoration = renderBox.decoration as BoxDecoration;
    expect(decoration.color, equals(const Color(0xFF0000FF)));
  });

  testWidgets('RenderObject not visiting children provides helpful error message', (
    WidgetTester tester,
  ) async {
    await tester.pumpWidget(
      TestNonVisitingWidget(child: Container(color: const Color(0xFFED1D7F))),
    );

    final RenderObject renderObject = tester.renderObject(find.byType(TestNonVisitingWidget));
    final Canvas testCanvas = TestRecordingCanvas();
    final PaintingContext testContext = TestRecordingPaintingContext(testCanvas);

    // When a parent fails to visit a child in visitChildren, the child's compositing
    // bits won't be cleared properly, leading to an exception during paint.
    renderObject.paint(testContext, Offset.zero);

    final dynamic error = tester.takeException();
    expect(error, isNotNull, reason: 'RenderObject did not throw when painting');
    expect(error, isFlutterError);
    expect(
      error.toString(),
      contains("A RenderObject was not visited by the parent's visitChildren"),
    );
  });
}
