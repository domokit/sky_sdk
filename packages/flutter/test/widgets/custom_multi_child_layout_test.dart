// Copyright 2014 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

import 'package:flutter/foundation.dart';
import 'package:flutter/rendering.dart';
import 'package:flutter/widgets.dart';
import 'package:flutter_test/flutter_test.dart';

class TestMultiChildLayoutDelegate extends MultiChildLayoutDelegate {
  late BoxConstraints getSizeConstraints;

  @override
  Size getSize(BoxConstraints constraints) {
    if (!RenderObject.debugCheckingIntrinsics) {
      getSizeConstraints = constraints;
    }
    return const Size(200.0, 300.0);
  }

  Size? performLayoutSize;
  late Size performLayoutSize0;
  late Size performLayoutSize1;
  late bool performLayoutIsChild;

  @override
  void performLayout(Size size) {
    assert(!RenderObject.debugCheckingIntrinsics);
    expect(() {
      performLayoutSize = size;
      final BoxConstraints constraints = BoxConstraints.loose(size);
      performLayoutSize0 = layoutChild(0, constraints);
      performLayoutSize1 = layoutChild(1, constraints);
      performLayoutIsChild = hasChild('fred');
    }, returnsNormally);
  }

  bool shouldRelayoutCalled = false;
  bool shouldRelayoutValue = false;

  @override
  bool shouldRelayout(_) {
    assert(!RenderObject.debugCheckingIntrinsics);
    shouldRelayoutCalled = true;
    return shouldRelayoutValue;
  }
}

Widget buildFrame(MultiChildLayoutDelegate delegate) {
  return Center(
    child: CustomMultiChildLayout(
      delegate: delegate,
      children: <Widget>[
        LayoutId(id: 0, child: const SizedBox(width: 150.0, height: 100.0)),
        LayoutId(id: 1, child: const SizedBox(width: 100.0, height: 200.0)),
      ],
    ),
  );
}

class PreferredSizeDelegate extends MultiChildLayoutDelegate {
  PreferredSizeDelegate({required this.preferredSize});

  final Size preferredSize;

  @override
  Size getSize(BoxConstraints constraints) => preferredSize;

  @override
  void performLayout(Size size) {}

  @override
  bool shouldRelayout(PreferredSizeDelegate oldDelegate) {
    return preferredSize != oldDelegate.preferredSize;
  }
}

class NotifierLayoutDelegate extends MultiChildLayoutDelegate {
  NotifierLayoutDelegate(this.size) : super(relayout: size);

  final ValueNotifier<Size> size;

  @override
  Size getSize(BoxConstraints constraints) => size.value;

  @override
  void performLayout(Size size) {}

  @override
  bool shouldRelayout(NotifierLayoutDelegate oldDelegate) {
    return size != oldDelegate.size;
  }
}

// LayoutDelegate that lays out child with id 0 and 1
// Used in the 'performLayout error control test' test case to trigger:
//  - error when laying out a non existent child and a child that has not been laid out
class ZeroAndOneIdLayoutDelegate extends MultiChildLayoutDelegate {
  @override
  void performLayout(Size size) {
    final BoxConstraints constraints = BoxConstraints.loose(size);
    layoutChild(0, constraints);
    layoutChild(1, constraints);
  }

  @override
  bool shouldRelayout(MultiChildLayoutDelegate oldDelegate) => true;
}

// Used in the 'performLayout error control test' test case
//  to trigger an error when laying out child more than once
class DuplicateLayoutDelegate extends MultiChildLayoutDelegate {
  @override
  void performLayout(Size size) {
    final BoxConstraints constraints = BoxConstraints.loose(size);
    layoutChild(0, constraints);
    layoutChild(0, constraints);
  }

  @override
  bool shouldRelayout(MultiChildLayoutDelegate oldDelegate) => true;
}

// Used in the 'performLayout error control test' test case
//  to trigger an error when positioning non existent child
class NonExistentPositionDelegate extends MultiChildLayoutDelegate {
  @override
  void performLayout(Size size) {
    positionChild(0, Offset.zero);
    positionChild(1, Offset.zero);
  }

  @override
  bool shouldRelayout(MultiChildLayoutDelegate oldDelegate) => true;
}

// Used in the 'performLayout error control test' test case for triggering
//  to layout child more than once
class InvalidConstraintsChildLayoutDelegate extends MultiChildLayoutDelegate {
  @override
  void performLayout(Size size) {
    final BoxConstraints constraints = BoxConstraints.loose(
      // Invalid because width and height must be greater than or equal to 0
      const Size(-1, -1),
    );
    layoutChild(0, constraints);
  }

  @override
  bool shouldRelayout(MultiChildLayoutDelegate oldDelegate) => true;
}

class LayoutWithMissingId extends ParentDataWidget<MultiChildLayoutParentData> {
  const LayoutWithMissingId({super.key, required super.child});

  @override
  void applyParentData(RenderObject renderObject) {}

  @override
  Type get debugTypicalAncestorWidgetClass => CustomMultiChildLayout;
}

void main() {
  testWidgets('Control test for CustomMultiChildLayout', (WidgetTester tester) async {
    final TestMultiChildLayoutDelegate delegate = TestMultiChildLayoutDelegate();
    await tester.pumpWidget(buildFrame(delegate));

    expect(delegate.getSizeConstraints.minWidth, 0.0);
    expect(delegate.getSizeConstraints.maxWidth, 800.0);
    expect(delegate.getSizeConstraints.minHeight, 0.0);
    expect(delegate.getSizeConstraints.maxHeight, 600.0);

    expect(delegate.performLayoutSize!.width, 200.0);
    expect(delegate.performLayoutSize!.height, 300.0);
    expect(delegate.performLayoutSize0.width, 150.0);
    expect(delegate.performLayoutSize0.height, 100.0);
    expect(delegate.performLayoutSize1.width, 100.0);
    expect(delegate.performLayoutSize1.height, 200.0);
    expect(delegate.performLayoutIsChild, false);
  });

  testWidgets('Test MultiChildDelegate shouldRelayout method', (WidgetTester tester) async {
    TestMultiChildLayoutDelegate delegate = TestMultiChildLayoutDelegate();
    await tester.pumpWidget(buildFrame(delegate));

    // Layout happened because the delegate was set.
    expect(delegate.performLayoutSize, isNotNull); // i.e. layout happened
    expect(delegate.shouldRelayoutCalled, isFalse);

    // Layout did not happen because shouldRelayout() returned false.
    delegate = TestMultiChildLayoutDelegate();
    delegate.shouldRelayoutValue = false;
    await tester.pumpWidget(buildFrame(delegate));
    expect(delegate.shouldRelayoutCalled, isTrue);
    expect(delegate.performLayoutSize, isNull);

    // Layout happened because shouldRelayout() returned true.
    delegate = TestMultiChildLayoutDelegate();
    delegate.shouldRelayoutValue = true;
    await tester.pumpWidget(buildFrame(delegate));
    expect(delegate.shouldRelayoutCalled, isTrue);
    expect(delegate.performLayoutSize, isNotNull);
  });

  testWidgets('Nested CustomMultiChildLayouts', (WidgetTester tester) async {
    final TestMultiChildLayoutDelegate delegate = TestMultiChildLayoutDelegate();
    await tester.pumpWidget(
      Center(
        child: CustomMultiChildLayout(
          delegate: delegate,
          children: <Widget>[
            LayoutId(
              id: 0,
              child: CustomMultiChildLayout(
                delegate: delegate,
                children: <Widget>[
                  LayoutId(id: 0, child: const SizedBox(width: 150.0, height: 100.0)),
                  LayoutId(id: 1, child: const SizedBox(width: 100.0, height: 200.0)),
                ],
              ),
            ),
            LayoutId(id: 1, child: const SizedBox(width: 100.0, height: 200.0)),
          ],
        ),
      ),
    );
  });

  testWidgets('Loose constraints', (WidgetTester tester) async {
    final Key key = UniqueKey();
    await tester.pumpWidget(
      Center(
        child: CustomMultiChildLayout(
          key: key,
          delegate: PreferredSizeDelegate(preferredSize: const Size(300.0, 200.0)),
        ),
      ),
    );

    final RenderBox box = tester.renderObject(find.byKey(key));
    expect(box.size.width, equals(300.0));
    expect(box.size.height, equals(200.0));

    await tester.pumpWidget(
      Center(
        child: CustomMultiChildLayout(
          key: key,
          delegate: PreferredSizeDelegate(preferredSize: const Size(350.0, 250.0)),
        ),
      ),
    );

    expect(box.size.width, equals(350.0));
    expect(box.size.height, equals(250.0));
  });

  testWidgets('Can use listener for relayout', (WidgetTester tester) async {
    final ValueNotifier<Size> size = ValueNotifier<Size>(const Size(100.0, 200.0));
    addTearDown(size.dispose);

    await tester.pumpWidget(
      Center(child: CustomMultiChildLayout(delegate: NotifierLayoutDelegate(size))),
    );

    RenderBox box = tester.renderObject(find.byType(CustomMultiChildLayout));
    expect(box.size, equals(const Size(100.0, 200.0)));

    size.value = const Size(150.0, 240.0);
    await tester.pump();

    box = tester.renderObject(find.byType(CustomMultiChildLayout));
    expect(box.size, equals(const Size(150.0, 240.0)));
  });

  group('performLayout error control test', () {
    Widget buildSingleChildFrame(MultiChildLayoutDelegate delegate) {
      return Center(
        child: CustomMultiChildLayout(
          delegate: delegate,
          children: <Widget>[LayoutId(id: 0, child: const SizedBox())],
        ),
      );
    }

    Future<void> expectFlutterErrorMessage({
      Widget? widget,
      MultiChildLayoutDelegate? delegate,
      required WidgetTester tester,
      required String message,
    }) async {
      final FlutterExceptionHandler? oldHandler = FlutterError.onError;
      final List<FlutterErrorDetails> errors = <FlutterErrorDetails>[];
      FlutterError.onError = (FlutterErrorDetails error) => errors.add(error);
      try {
        await tester.pumpWidget(widget ?? buildSingleChildFrame(delegate!));
      } finally {
        FlutterError.onError = oldHandler;
      }
      expect(errors.length, isNonZero);
      expect(errors.first, isNotNull);
      expect(errors.first.exception, isFlutterError);
      expect(
        (errors.first.exception as FlutterError).toStringDeep(),
        equalsIgnoringHashCodes(message),
      );
    }

    testWidgets('layoutChild on non existent child', (WidgetTester tester) async {
      await expectFlutterErrorMessage(
        tester: tester,
        delegate: ZeroAndOneIdLayoutDelegate(),
        message:
            'FlutterError\n'
            '   The ZeroAndOneIdLayoutDelegate custom multichild layout delegate\n'
            '   tried to lay out a non-existent child.\n'
            '   There is no child with the id "1".\n',
      );
    });

    testWidgets('layoutChild more than once', (WidgetTester tester) async {
      await expectFlutterErrorMessage(
        tester: tester,
        delegate: DuplicateLayoutDelegate(),
        message:
            'FlutterError\n'
            '   The DuplicateLayoutDelegate custom multichild layout delegate\n'
            '   tried to lay out the child with id "0" more than once.\n'
            '   Each child must be laid out exactly once.\n',
      );
    });

    testWidgets('layoutChild on invalid size constraint', (WidgetTester tester) async {
      await expectFlutterErrorMessage(
        tester: tester,
        delegate: InvalidConstraintsChildLayoutDelegate(),
        message:
            'FlutterError\n'
            '   The InvalidConstraintsChildLayoutDelegate custom multichild\n'
            '   layout delegate provided invalid box constraints for the child\n'
            '   with id "0".\n'
            '   FlutterError\n'
            '   The minimum width and height must be greater than or equal to\n'
            '   zero.\n'
            '   The maximum width must be greater than or equal to the minimum\n'
            '   width.\n'
            '   The maximum height must be greater than or equal to the minimum\n'
            '   height.\n',
      );
    });

    testWidgets('positionChild on non existent child', (WidgetTester tester) async {
      await expectFlutterErrorMessage(
        tester: tester,
        delegate: NonExistentPositionDelegate(),
        message:
            'FlutterError\n'
            '   The NonExistentPositionDelegate custom multichild layout delegate\n'
            '   tried to position out a non-existent child:\n'
            '   There is no child with the id "1".\n',
      );
    });

    testWidgets("_callPerformLayout on child that doesn't have id", (WidgetTester tester) async {
      await expectFlutterErrorMessage(
        widget: Center(
          child: CustomMultiChildLayout(
            delegate: PreferredSizeDelegate(preferredSize: const Size(10, 10)),
            children: <Widget>[LayoutWithMissingId(child: Container(width: 100))],
          ),
        ),
        tester: tester,
        message:
            'FlutterError\n'
            '   Every child of a RenderCustomMultiChildLayoutBox must have an ID\n'
            '   in its parent data.\n'
            '   The following child has no ID: RenderConstrainedBox#00000 NEEDS-LAYOUT NEEDS-PAINT:\n'
            '     creator: ConstrainedBox ← Container ← LayoutWithMissingId ←\n'
            '       CustomMultiChildLayout ← Center ← _FocusInheritedScope ←\n'
            '       _FocusScopeWithExternalFocusNode ← _FocusInheritedScope ← Focus\n'
            '       ← FocusTraversalGroup ← MediaQuery ← _MediaQueryFromView ← ⋯\n'
            '     parentData: offset=Offset(0.0, 0.0); id=null\n'
            '     constraints: MISSING\n'
            '     size: MISSING\n'
            '     additionalConstraints: BoxConstraints(w=100.0, 0.0<=h<=Infinity)\n',
      );
    });

    testWidgets('performLayout did not layout a child', (WidgetTester tester) async {
      await expectFlutterErrorMessage(
        widget: Center(
          child: CustomMultiChildLayout(
            delegate: ZeroAndOneIdLayoutDelegate(),
            children: <Widget>[
              LayoutId(id: 0, child: Container(width: 100)),
              LayoutId(id: 1, child: Container(width: 100)),
              LayoutId(id: 2, child: Container(width: 100)),
            ],
          ),
        ),
        tester: tester,
        message:
            'FlutterError\n'
            '   Each child must be laid out exactly once.\n'
            '   The ZeroAndOneIdLayoutDelegate custom multichild layout delegate'
            ' forgot to lay out the following child:\n'
            '     2: RenderConstrainedBox#62a34 NEEDS-LAYOUT NEEDS-PAINT\n',
      );
    });

    testWidgets('performLayout did not layout multiple child', (WidgetTester tester) async {
      await expectFlutterErrorMessage(
        widget: Center(
          child: CustomMultiChildLayout(
            delegate: ZeroAndOneIdLayoutDelegate(),
            children: <Widget>[
              LayoutId(id: 0, child: Container(width: 100)),
              LayoutId(id: 1, child: Container(width: 100)),
              LayoutId(id: 2, child: Container(width: 100)),
              LayoutId(id: 3, child: Container(width: 100)),
            ],
          ),
        ),
        tester: tester,
        message:
            'FlutterError\n'
            '   Each child must be laid out exactly once.\n'
            '   The ZeroAndOneIdLayoutDelegate custom multichild layout delegate'
            ' forgot to lay out the following children:\n'
            '     2: RenderConstrainedBox#62a34 NEEDS-LAYOUT NEEDS-PAINT\n'
            '     3: RenderConstrainedBox#62a34 NEEDS-LAYOUT NEEDS-PAINT\n',
      );
    });
  });
}
