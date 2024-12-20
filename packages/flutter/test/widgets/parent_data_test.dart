// Copyright 2014 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

import 'package:flutter/material.dart';
import 'package:flutter/rendering.dart';
import 'package:flutter/widgets.dart';
import 'package:flutter_test/flutter_test.dart';

import 'test_widgets.dart';

class TestParentData {
  TestParentData({this.top, this.right, this.bottom, this.left});

  final double? top;
  final double? right;
  final double? bottom;
  final double? left;
}

void checkTree(WidgetTester tester, List<TestParentData> expectedParentData) {
  final MultiChildRenderObjectElement element = tester.element(
    find.byElementPredicate((Element element) => element is MultiChildRenderObjectElement),
  );
  expect(element, isNotNull);
  expect(element.renderObject, isA<RenderStack>());
  final RenderStack renderObject = element.renderObject as RenderStack;
  try {
    RenderObject? child = renderObject.firstChild;
    for (final TestParentData expected in expectedParentData) {
      expect(child, isA<RenderDecoratedBox>());
      final RenderDecoratedBox decoratedBox = child! as RenderDecoratedBox;
      expect(decoratedBox.parentData, isA<StackParentData>());
      final StackParentData parentData = decoratedBox.parentData! as StackParentData;
      expect(parentData.top, equals(expected.top));
      expect(parentData.right, equals(expected.right));
      expect(parentData.bottom, equals(expected.bottom));
      expect(parentData.left, equals(expected.left));
      final StackParentData? decoratedBoxParentData = decoratedBox.parentData as StackParentData?;
      child = decoratedBoxParentData?.nextSibling;
    }
    expect(child, isNull);
  } catch (e) {
    debugPrint(renderObject.toStringDeep());
    rethrow;
  }
}

final TestParentData kNonPositioned = TestParentData();

void main() {
  testWidgets('ParentDataWidget control test', (WidgetTester tester) async {
    await tester.pumpWidget(
      const Stack(
        textDirection: TextDirection.ltr,
        children: <Widget>[
          DecoratedBox(decoration: kBoxDecorationA),
          Positioned(top: 10.0, left: 10.0, child: DecoratedBox(decoration: kBoxDecorationB)),
          DecoratedBox(decoration: kBoxDecorationC),
        ],
      ),
    );

    checkTree(tester, <TestParentData>[
      kNonPositioned,
      TestParentData(top: 10.0, left: 10.0),
      kNonPositioned,
    ]);

    await tester.pumpWidget(
      const Stack(
        textDirection: TextDirection.ltr,
        children: <Widget>[
          Positioned(bottom: 5.0, right: 7.0, child: DecoratedBox(decoration: kBoxDecorationA)),
          Positioned(top: 10.0, left: 10.0, child: DecoratedBox(decoration: kBoxDecorationB)),
          DecoratedBox(decoration: kBoxDecorationC),
        ],
      ),
    );

    checkTree(tester, <TestParentData>[
      TestParentData(bottom: 5.0, right: 7.0),
      TestParentData(top: 10.0, left: 10.0),
      kNonPositioned,
    ]);

    const DecoratedBox kDecoratedBoxA = DecoratedBox(decoration: kBoxDecorationA);
    const DecoratedBox kDecoratedBoxB = DecoratedBox(decoration: kBoxDecorationB);
    const DecoratedBox kDecoratedBoxC = DecoratedBox(decoration: kBoxDecorationC);

    await tester.pumpWidget(
      const Stack(
        textDirection: TextDirection.ltr,
        children: <Widget>[
          Positioned(bottom: 5.0, right: 7.0, child: kDecoratedBoxA),
          Positioned(top: 10.0, left: 10.0, child: kDecoratedBoxB),
          kDecoratedBoxC,
        ],
      ),
    );

    checkTree(tester, <TestParentData>[
      TestParentData(bottom: 5.0, right: 7.0),
      TestParentData(top: 10.0, left: 10.0),
      kNonPositioned,
    ]);

    await tester.pumpWidget(
      const Stack(
        textDirection: TextDirection.ltr,
        children: <Widget>[
          Positioned(bottom: 6.0, right: 8.0, child: kDecoratedBoxA),
          Positioned(left: 10.0, right: 10.0, child: kDecoratedBoxB),
          kDecoratedBoxC,
        ],
      ),
    );

    checkTree(tester, <TestParentData>[
      TestParentData(bottom: 6.0, right: 8.0),
      TestParentData(left: 10.0, right: 10.0),
      kNonPositioned,
    ]);

    await tester.pumpWidget(
      Stack(
        textDirection: TextDirection.ltr,
        children: <Widget>[
          kDecoratedBoxA,
          Positioned(left: 11.0, right: 12.0, child: Container(child: kDecoratedBoxB)),
          kDecoratedBoxC,
        ],
      ),
    );

    checkTree(tester, <TestParentData>[
      kNonPositioned,
      TestParentData(left: 11.0, right: 12.0),
      kNonPositioned,
    ]);

    await tester.pumpWidget(
      Stack(
        textDirection: TextDirection.ltr,
        children: <Widget>[
          kDecoratedBoxA,
          Positioned(right: 10.0, child: Container(child: kDecoratedBoxB)),
          const DummyWidget(child: Positioned(top: 8.0, child: kDecoratedBoxC)),
        ],
      ),
    );

    checkTree(tester, <TestParentData>[
      kNonPositioned,
      TestParentData(right: 10.0),
      TestParentData(top: 8.0),
    ]);

    await tester.pumpWidget(
      const Stack(
        textDirection: TextDirection.ltr,
        children: <Widget>[
          Positioned(right: 10.0, child: FlipWidget(left: kDecoratedBoxA, right: kDecoratedBoxB)),
        ],
      ),
    );

    checkTree(tester, <TestParentData>[TestParentData(right: 10.0)]);

    flipStatefulWidget(tester);
    await tester.pump();

    checkTree(tester, <TestParentData>[TestParentData(right: 10.0)]);

    await tester.pumpWidget(
      const Stack(
        textDirection: TextDirection.ltr,
        children: <Widget>[
          Positioned(top: 7.0, child: FlipWidget(left: kDecoratedBoxA, right: kDecoratedBoxB)),
        ],
      ),
    );

    checkTree(tester, <TestParentData>[TestParentData(top: 7.0)]);

    flipStatefulWidget(tester);
    await tester.pump();

    checkTree(tester, <TestParentData>[TestParentData(top: 7.0)]);

    await tester.pumpWidget(const Stack(textDirection: TextDirection.ltr));

    checkTree(tester, <TestParentData>[]);
  });

  testWidgets('ParentData overwrite with custom ParentDataWidget subclasses', (
    WidgetTester tester,
  ) async {
    await tester.pumpWidget(
      const Directionality(
        textDirection: TextDirection.ltr,
        child: Stack(
          children: <Widget>[
            CustomPositionedWidget(
              bottom: 8.0,
              child: Positioned(
                top: 6.0,
                left: 7.0,
                child: DecoratedBox(decoration: kBoxDecorationB),
              ),
            ),
          ],
        ),
      ),
    );

    dynamic exception = tester.takeException();
    expect(exception, isFlutterError);
    expect(
      exception.toString(),
      startsWith(
        'Incorrect use of ParentDataWidget.\n'
        'Competing ParentDataWidgets are providing parent data to the same RenderObject:\n'
        '- Positioned(left: 7.0, top: 6.0), which writes ParentData of type '
        'StackParentData, (typically placed directly inside a Stack widget)\n'
        '- CustomPositionedWidget, which writes ParentData of type '
        'StackParentData, (typically placed directly inside a Stack widget)\n'
        'A RenderObject can receive parent data from multiple '
        'ParentDataWidgets, but the Type of ParentData must be unique to '
        'prevent one overwriting another.\n'
        'Usually, this indicates that one or more of the offending ParentDataWidgets listed '
        "above isn't placed inside a dedicated compatible ancestor widget that it isn't "
        'sharing with another ParentDataWidget of the same type.\n'
        'Otherwise, separating aspects of ParentData to prevent conflicts can '
        'be done using mixins, mixing them all in on the full ParentData '
        'Object, such as KeepAlive does with KeepAliveParentDataMixin.\n'
        'The ownership chain for the RenderObject that received the parent data was:\n'
        '  DecoratedBox ← Positioned ← CustomPositionedWidget ← Stack ← Directionality ← ', // End of chain omitted, not relevant for test.
      ),
    );

    await tester.pumpWidget(
      const Directionality(
        textDirection: TextDirection.ltr,
        child: Stack(
          children: <Widget>[
            SubclassPositioned(
              bottom: 8.0,
              child: Positioned(
                top: 6.0,
                left: 7.0,
                child: DecoratedBox(decoration: kBoxDecorationB),
              ),
            ),
          ],
        ),
      ),
    );

    exception = tester.takeException();
    expect(exception, isFlutterError);
    expect(
      exception.toString(),
      startsWith(
        'Incorrect use of ParentDataWidget.\n'
        'Competing ParentDataWidgets are providing parent data to the same RenderObject:\n'
        '- Positioned(left: 7.0, top: 6.0), which writes ParentData of type '
        'StackParentData, (typically placed directly inside a Stack widget)\n'
        '- SubclassPositioned(bottom: 8.0), which writes ParentData of type '
        'StackParentData, (typically placed directly inside a Stack widget)\n'
        'A RenderObject can receive parent data from multiple '
        'ParentDataWidgets, but the Type of ParentData must be unique to '
        'prevent one overwriting another.\n'
        'Usually, this indicates that one or more of the offending ParentDataWidgets listed '
        "above isn't placed inside a dedicated compatible ancestor widget that it isn't "
        'sharing with another ParentDataWidget of the same type.\n'
        'Otherwise, separating aspects of ParentData to prevent conflicts can '
        'be done using mixins, mixing them all in on the full ParentData '
        'Object, such as KeepAlive does with KeepAliveParentDataMixin.\n'
        'The ownership chain for the RenderObject that received the parent data was:\n'
        '  DecoratedBox ← Positioned ← SubclassPositioned ← Stack ← Directionality ← ', // End of chain omitted, not relevant for test.
      ),
    );
  });

  testWidgets('ParentDataWidget conflicting data', (WidgetTester tester) async {
    await tester.pumpWidget(
      const Directionality(
        textDirection: TextDirection.ltr,
        child: Stack(
          children: <Widget>[
            Positioned(
              top: 5.0,
              bottom: 8.0,
              child: Positioned(
                top: 6.0,
                left: 7.0,
                child: DecoratedBox(decoration: kBoxDecorationB),
              ),
            ),
          ],
        ),
      ),
    );

    dynamic exception = tester.takeException();
    expect(exception, isFlutterError);
    expect(
      exception.toString(),
      startsWith(
        'Incorrect use of ParentDataWidget.\n'
        'Competing ParentDataWidgets are providing parent data to the same RenderObject:\n'
        '- Positioned(left: 7.0, top: 6.0), which writes ParentData of type '
        'StackParentData, (typically placed directly inside a Stack widget)\n'
        '- Positioned(top: 5.0, bottom: 8.0), which writes ParentData of type '
        'StackParentData, (typically placed directly inside a Stack widget)\n'
        'A RenderObject can receive parent data from multiple '
        'ParentDataWidgets, but the Type of ParentData must be unique to '
        'prevent one overwriting another.\n'
        'Usually, this indicates that one or more of the offending ParentDataWidgets listed '
        "above isn't placed inside a dedicated compatible ancestor widget that it isn't "
        'sharing with another ParentDataWidget of the same type.\n'
        'Otherwise, separating aspects of ParentData to prevent conflicts can '
        'be done using mixins, mixing them all in on the full ParentData '
        'Object, such as KeepAlive does with KeepAliveParentDataMixin.\n'
        'The ownership chain for the RenderObject that received the parent data was:\n'
        '  DecoratedBox ← Positioned ← Positioned ← Stack ← Directionality ← ', // End of chain omitted, not relevant for test.
      ),
    );

    await tester.pumpWidget(const Stack(textDirection: TextDirection.ltr));
    checkTree(tester, <TestParentData>[]);

    await tester.pumpWidget(
      const Directionality(
        textDirection: TextDirection.ltr,
        child: DummyWidget(
          child: Row(
            children: <Widget>[
              Positioned(top: 6.0, left: 7.0, child: DecoratedBox(decoration: kBoxDecorationB)),
            ],
          ),
        ),
      ),
    );

    exception = tester.takeException();
    expect(exception, isFlutterError);
    expect(
      exception.toString(),
      startsWith(
        'Incorrect use of ParentDataWidget.\n'
        'The ParentDataWidget Positioned(left: 7.0, top: 6.0) wants to apply ParentData of type '
        'StackParentData to a RenderObject, which has been set up to accept ParentData of '
        'incompatible type FlexParentData.\n'
        'Usually, this means that the Positioned widget has the wrong ancestor RenderObjectWidget. '
        'Typically, Positioned widgets are placed directly inside Stack widgets.\n'
        'The offending Positioned is currently placed inside a Row widget.\n'
        'The ownership chain for the RenderObject that received the incompatible parent data was:\n'
        '  DecoratedBox ← Positioned ← Row ← DummyWidget ← Directionality ← ', // End of chain omitted, not relevant for test.
      ),
    );

    await tester.pumpWidget(const Stack(textDirection: TextDirection.ltr));
    checkTree(tester, <TestParentData>[]);
  });

  testWidgets('ParentDataWidget interacts with global keys', (WidgetTester tester) async {
    final GlobalKey key = GlobalKey();

    await tester.pumpWidget(
      Stack(
        textDirection: TextDirection.ltr,
        children: <Widget>[
          Positioned(
            top: 10.0,
            left: 10.0,
            child: DecoratedBox(key: key, decoration: kBoxDecorationA),
          ),
        ],
      ),
    );

    checkTree(tester, <TestParentData>[TestParentData(top: 10.0, left: 10.0)]);

    await tester.pumpWidget(
      Stack(
        textDirection: TextDirection.ltr,
        children: <Widget>[
          Positioned(
            top: 10.0,
            left: 10.0,
            child: DecoratedBox(
              decoration: kBoxDecorationB,
              child: DecoratedBox(key: key, decoration: kBoxDecorationA),
            ),
          ),
        ],
      ),
    );

    checkTree(tester, <TestParentData>[TestParentData(top: 10.0, left: 10.0)]);

    await tester.pumpWidget(
      Stack(
        textDirection: TextDirection.ltr,
        children: <Widget>[
          Positioned(
            top: 10.0,
            left: 10.0,
            child: DecoratedBox(key: key, decoration: kBoxDecorationA),
          ),
        ],
      ),
    );

    checkTree(tester, <TestParentData>[TestParentData(top: 10.0, left: 10.0)]);
  });

  testWidgets('Parent data invalid ancestor', (WidgetTester tester) async {
    await tester.pumpWidget(
      Directionality(
        textDirection: TextDirection.ltr,
        child: Row(
          children: <Widget>[
            Stack(
              textDirection: TextDirection.ltr,
              children: <Widget>[Expanded(child: Container())],
            ),
          ],
        ),
      ),
    );

    final dynamic exception = tester.takeException();
    expect(exception, isFlutterError);
    expect(
      exception.toString(),
      startsWith(
        'Incorrect use of ParentDataWidget.\n'
        'The ParentDataWidget Expanded(flex: 1) wants to apply ParentData of type '
        'FlexParentData to a RenderObject, which has been set up to accept ParentData of '
        'incompatible type StackParentData.\n'
        'Usually, this means that the Expanded widget has the wrong ancestor RenderObjectWidget. '
        'Typically, Expanded widgets are placed directly inside Flex widgets.\n'
        'The offending Expanded is currently placed inside a Stack widget.\n'
        'The ownership chain for the RenderObject that received the incompatible parent data was:\n'
        '  LimitedBox ← Container ← Expanded ← Stack ← Row ← Directionality ← ', // Omitted end of debugCreator chain because it's irrelevant for test.
      ),
    );
  });

  testWidgets('ParentDataWidget can be used with different ancestor RenderObjectWidgets', (
    WidgetTester tester,
  ) async {
    await tester.pumpWidget(OneAncestorWidget(child: Container()));
    DummyParentData parentData =
        tester.renderObject(find.byType(Container)).parentData! as DummyParentData;
    expect(parentData.string, isNull);

    await tester.pumpWidget(
      OneAncestorWidget(child: TestParentDataWidget(string: 'Foo', child: Container())),
    );
    parentData = tester.renderObject(find.byType(Container)).parentData! as DummyParentData;
    expect(parentData.string, 'Foo');

    await tester.pumpWidget(
      AnotherAncestorWidget(child: TestParentDataWidget(string: 'Bar', child: Container())),
    );
    parentData = tester.renderObject(find.byType(Container)).parentData! as DummyParentData;
    expect(parentData.string, 'Bar');
  });
}

class SubclassPositioned extends Positioned {
  const SubclassPositioned({
    super.key,
    super.left,
    super.top,
    super.right,
    super.bottom,
    super.width,
    super.height,
    required super.child,
  });

  @override
  void applyParentData(RenderObject renderObject) {
    assert(renderObject.parentData is StackParentData);
    final StackParentData parentData = renderObject.parentData! as StackParentData;
    parentData.bottom = bottom;
  }
}

class CustomPositionedWidget extends ParentDataWidget<StackParentData> {
  const CustomPositionedWidget({super.key, required this.bottom, required super.child});

  final double bottom;

  @override
  void applyParentData(RenderObject renderObject) {
    assert(renderObject.parentData is StackParentData);
    final StackParentData parentData = renderObject.parentData! as StackParentData;
    parentData.bottom = bottom;
  }

  @override
  Type get debugTypicalAncestorWidgetClass => Stack;
}

class TestParentDataWidget extends ParentDataWidget<DummyParentData> {
  const TestParentDataWidget({super.key, required this.string, required super.child});

  final String string;

  @override
  void applyParentData(RenderObject renderObject) {
    assert(renderObject.parentData is DummyParentData);
    final DummyParentData parentData = renderObject.parentData! as DummyParentData;
    parentData.string = string;
  }

  @override
  Type get debugTypicalAncestorWidgetClass => OneAncestorWidget;
}

class DummyParentData extends ParentData {
  String? string;
}

class OneAncestorWidget extends SingleChildRenderObjectWidget {
  const OneAncestorWidget({super.key, required Widget super.child});

  @override
  RenderOne createRenderObject(BuildContext context) => RenderOne();
}

class AnotherAncestorWidget extends SingleChildRenderObjectWidget {
  const AnotherAncestorWidget({super.key, required Widget super.child});

  @override
  RenderAnother createRenderObject(BuildContext context) => RenderAnother();
}

class RenderOne extends RenderProxyBox {
  @override
  void setupParentData(RenderBox child) {
    if (child.parentData is! DummyParentData) {
      child.parentData = DummyParentData();
    }
  }
}

class RenderAnother extends RenderProxyBox {
  @override
  void setupParentData(RenderBox child) {
    if (child.parentData is! DummyParentData) {
      child.parentData = DummyParentData();
    }
  }
}

class DummyWidget extends StatelessWidget {
  const DummyWidget({super.key, required this.child});

  final Widget child;

  @override
  Widget build(BuildContext context) => child;
}
