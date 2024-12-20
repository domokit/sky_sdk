// Copyright 2014 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

import 'package:flutter/foundation.dart';
import 'package:flutter/material.dart';
import 'package:flutter/rendering.dart';
import 'package:flutter/widgets.dart';
import 'package:flutter_test/flutter_test.dart';
import 'package:leak_tracker_flutter_testing/leak_tracker_flutter_testing.dart';

const Color green = Color(0xFF00FF00);
const Color yellow = Color(0xFFFFFF00);

void main() {
  testWidgets('SlottedRenderObjectWidget test', (WidgetTester tester) async {
    await tester.pumpWidget(
      buildWidget(
        topLeft: Container(height: 100, width: 80, color: yellow, child: const Text('topLeft')),
        bottomRight: Container(
          height: 120,
          width: 110,
          color: green,
          child: const Text('bottomRight'),
        ),
      ),
    );

    expect(find.text('topLeft'), findsOneWidget);
    expect(find.text('bottomRight'), findsOneWidget);
    expect(tester.getSize(find.byType(_Diagonal)), const Size(80 + 110, 100 + 120));
    expect(
      find.byType(_Diagonal),
      paints
        ..rect(rect: const Rect.fromLTWH(0, 0, 80, 100), color: yellow)
        ..rect(rect: const Rect.fromLTWH(80, 100, 110, 120), color: green),
    );

    await tester.pumpWidget(
      buildWidget(
        topLeft: Container(height: 200, width: 100, color: yellow, child: const Text('topLeft')),
        bottomRight: Container(
          height: 220,
          width: 210,
          color: green,
          child: const Text('bottomRight'),
        ),
      ),
    );

    expect(find.text('topLeft'), findsOneWidget);
    expect(find.text('bottomRight'), findsOneWidget);
    expect(tester.getSize(find.byType(_Diagonal)), const Size(100 + 210, 200 + 220));
    expect(
      find.byType(_Diagonal),
      paints
        ..rect(rect: const Rect.fromLTWH(0, 0, 100, 200), color: yellow)
        ..rect(rect: const Rect.fromLTWH(100, 200, 210, 220), color: green),
    );

    await tester.pumpWidget(
      buildWidget(
        topLeft: Container(height: 200, width: 100, color: yellow, child: const Text('topLeft')),
        bottomRight: Container(
          key: UniqueKey(),
          height: 230,
          width: 220,
          color: green,
          child: const Text('bottomRight'),
        ),
      ),
    );

    expect(find.text('topLeft'), findsOneWidget);
    expect(find.text('bottomRight'), findsOneWidget);
    expect(tester.getSize(find.byType(_Diagonal)), const Size(100 + 220, 200 + 230));
    expect(
      find.byType(_Diagonal),
      paints
        ..rect(rect: const Rect.fromLTWH(0, 0, 100, 200), color: yellow)
        ..rect(rect: const Rect.fromLTWH(100, 200, 220, 230), color: green),
    );

    await tester.pumpWidget(
      buildWidget(
        topLeft: Container(height: 200, width: 100, color: yellow, child: const Text('topLeft')),
      ),
    );

    expect(find.text('topLeft'), findsOneWidget);
    expect(find.text('bottomRight'), findsNothing);
    expect(tester.getSize(find.byType(_Diagonal)), const Size(100, 200));
    expect(
      find.byType(_Diagonal),
      paints..rect(rect: const Rect.fromLTWH(0, 0, 100, 200), color: yellow),
    );

    await tester.pumpWidget(buildWidget());
    expect(find.text('topLeft'), findsNothing);
    expect(find.text('bottomRight'), findsNothing);
    expect(tester.getSize(find.byType(_Diagonal)), Size.zero);
    expect(find.byType(_Diagonal), paintsNothing);

    await tester.pumpWidget(Container());
    expect(find.byType(_Diagonal), findsNothing);
  });

  test('nameForSlot', () {
    expect(_RenderDiagonal().publicNameForSlot(_DiagonalSlot.bottomRight), 'bottomRight');
    expect(_RenderDiagonal().publicNameForSlot(_DiagonalSlot.topLeft), 'topLeft');
    final _Slot slot = _Slot();
    expect(_RenderTest().publicNameForSlot(slot), slot.toString());
  });

  testWidgets('key reparenting', (WidgetTester tester) async {
    const Widget widget1 = SizedBox(key: ValueKey<String>('smol'), height: 10, width: 10);
    const Widget widget2 = SizedBox(key: ValueKey<String>('big'), height: 100, width: 100);
    const Widget nullWidget = SizedBox(key: ValueKey<String>('null'), height: 50, width: 50);

    await tester.pumpWidget(
      buildWidget(topLeft: widget1, bottomRight: widget2, nullSlot: nullWidget),
    );
    final _RenderDiagonal renderObject = tester.renderObject(find.byType(_Diagonal));
    expect(renderObject._topLeft!.size, const Size(10, 10));
    expect(renderObject._bottomRight!.size, const Size(100, 100));
    expect(renderObject._nullSlot!.size, const Size(50, 50));

    final Element widget1Element = tester.element(find.byWidget(widget1));
    final Element widget2Element = tester.element(find.byWidget(widget2));
    final Element nullWidgetElement = tester.element(find.byWidget(nullWidget));

    // Swapping 1 and 2.
    await tester.pumpWidget(
      buildWidget(topLeft: widget2, bottomRight: widget1, nullSlot: nullWidget),
    );
    expect(renderObject._topLeft!.size, const Size(100, 100));
    expect(renderObject._bottomRight!.size, const Size(10, 10));
    expect(renderObject._nullSlot!.size, const Size(50, 50));
    expect(widget1Element, same(tester.element(find.byWidget(widget1))));
    expect(widget2Element, same(tester.element(find.byWidget(widget2))));
    expect(nullWidgetElement, same(tester.element(find.byWidget(nullWidget))));

    // Shifting slots
    await tester.pumpWidget(
      buildWidget(topLeft: nullWidget, bottomRight: widget2, nullSlot: widget1),
    );
    expect(renderObject._topLeft!.size, const Size(50, 50));
    expect(renderObject._bottomRight!.size, const Size(100, 100));
    expect(renderObject._nullSlot!.size, const Size(10, 10));
    expect(widget1Element, same(tester.element(find.byWidget(widget1))));
    expect(widget2Element, same(tester.element(find.byWidget(widget2))));
    expect(nullWidgetElement, same(tester.element(find.byWidget(nullWidget))));

    // Moving + Deleting.
    await tester.pumpWidget(buildWidget(bottomRight: widget2));
    expect(renderObject._topLeft, null);
    expect(renderObject._bottomRight!.size, const Size(100, 100));
    expect(renderObject._nullSlot, null);
    expect(widget1Element.debugIsDefunct, isTrue);
    expect(nullWidgetElement.debugIsDefunct, isTrue);
    expect(widget2Element, same(tester.element(find.byWidget(widget2))));

    // Moving.
    await tester.pumpWidget(buildWidget(topLeft: widget2));
    expect(renderObject._topLeft!.size, const Size(100, 100));
    expect(renderObject._bottomRight, null);
    expect(widget2Element, same(tester.element(find.byWidget(widget2))));
  });

  testWidgets(
    'duplicated key error message',
    experimentalLeakTesting:
        LeakTesting.settings.withIgnoredAll(), // leaking by design because of exception
    (WidgetTester tester) async {
      const Widget widget1 = SizedBox(key: ValueKey<String>('widget 1'), height: 10, width: 10);
      const Widget widget2 = SizedBox(key: ValueKey<String>('widget 1'), height: 100, width: 100);
      const Widget widget3 = SizedBox(key: ValueKey<String>('widget 1'), height: 50, width: 50);

      await tester.pumpWidget(
        buildWidget(topLeft: widget1, bottomRight: widget2, nullSlot: widget3),
      );

      expect(
        (tester.takeException() as FlutterError).toString(),
        equalsIgnoringHashCodes(
          'Multiple widgets used the same key in _Diagonal.\n'
          "The key [<'widget 1'>] was used by multiple widgets. The offending widgets were:\n"
          "  - SizedBox-[<'widget 1'>](width: 50.0, height: 50.0, renderObject: RenderConstrainedBox#00000 NEEDS-LAYOUT NEEDS-PAINT)\n"
          "  - SizedBox-[<'widget 1'>](width: 10.0, height: 10.0, renderObject: RenderConstrainedBox#00000 NEEDS-LAYOUT NEEDS-PAINT)\n"
          "  - SizedBox-[<'widget 1'>](width: 100.0, height: 100.0, renderObject: RenderConstrainedBox#a4685 NEEDS-LAYOUT NEEDS-PAINT)\n"
          'A key can only be specified on one widget at a time in the same parent widget.',
        ),
      );
    },
  );

  testWidgets('debugDescribeChildren', (WidgetTester tester) async {
    await tester.pumpWidget(
      buildWidget(
        topLeft: const SizedBox(height: 100, width: 80),
        bottomRight: const SizedBox(height: 120, width: 110),
      ),
    );

    expect(
      tester.renderObject(find.byType(_Diagonal)).toStringDeep(),
      equalsIgnoringHashCodes(
        '_RenderDiagonal#00000 relayoutBoundary=up1\n'
        ' │ creator: _Diagonal ← Align ← Directionality ←\n'
        ' │   _FocusInheritedScope ← _FocusScopeWithExternalFocusNode ←\n'
        ' │   _FocusInheritedScope ← Focus ← FocusTraversalGroup ← MediaQuery\n'
        ' │   ← _MediaQueryFromView ← _PipelineOwnerScope ← _ViewScope ← ⋯\n'
        ' │ parentData: offset=Offset(0.0, 0.0) (can use size)\n'
        ' │ constraints: BoxConstraints(0.0<=w<=800.0, 0.0<=h<=600.0)\n'
        ' │ size: Size(190.0, 220.0)\n'
        ' │\n'
        ' ├─topLeft: RenderConstrainedBox#00000 relayoutBoundary=up2\n'
        ' │   creator: SizedBox ← _Diagonal ← Align ← Directionality ←\n'
        ' │     _FocusInheritedScope ← _FocusScopeWithExternalFocusNode ←\n'
        ' │     _FocusInheritedScope ← Focus ← FocusTraversalGroup ← MediaQuery\n'
        ' │     ← _MediaQueryFromView ← _PipelineOwnerScope ← ⋯\n'
        ' │   parentData: offset=Offset(0.0, 0.0) (can use size)\n'
        ' │   constraints: BoxConstraints(unconstrained)\n'
        ' │   size: Size(80.0, 100.0)\n'
        ' │   additionalConstraints: BoxConstraints(w=80.0, h=100.0)\n'
        ' │\n'
        ' └─bottomRight: RenderConstrainedBox#00000 relayoutBoundary=up2\n'
        '     creator: SizedBox ← _Diagonal ← Align ← Directionality ←\n'
        '       _FocusInheritedScope ← _FocusScopeWithExternalFocusNode ←\n'
        '       _FocusInheritedScope ← Focus ← FocusTraversalGroup ← MediaQuery\n'
        '       ← _MediaQueryFromView ← _PipelineOwnerScope ← ⋯\n'
        '     parentData: offset=Offset(80.0, 100.0) (can use size)\n'
        '     constraints: BoxConstraints(unconstrained)\n'
        '     size: Size(110.0, 120.0)\n'
        '     additionalConstraints: BoxConstraints(w=110.0, h=120.0)\n',
      ),
    );
  });
}

Widget buildWidget({Widget? topLeft, Widget? bottomRight, Widget? nullSlot}) {
  return Directionality(
    textDirection: TextDirection.ltr,
    child: Align(
      alignment: Alignment.topLeft,
      child: _Diagonal(topLeft: topLeft, bottomRight: bottomRight, nullSlot: nullSlot),
    ),
  );
}

enum _DiagonalSlot { topLeft, bottomRight }

class _Diagonal extends RenderObjectWidget
    with SlottedMultiChildRenderObjectWidgetMixin<_DiagonalSlot?, RenderBox> {
  const _Diagonal({this.topLeft, this.bottomRight, this.nullSlot});

  final Widget? topLeft;
  final Widget? bottomRight;
  final Widget? nullSlot;

  @override
  Iterable<_DiagonalSlot?> get slots => <_DiagonalSlot?>[null, ..._DiagonalSlot.values];

  @override
  Widget? childForSlot(_DiagonalSlot? slot) {
    return switch (slot) {
      null => nullSlot,
      _DiagonalSlot.topLeft => topLeft,
      _DiagonalSlot.bottomRight => bottomRight,
    };
  }

  @override
  SlottedContainerRenderObjectMixin<_DiagonalSlot?, RenderBox> createRenderObject(
    BuildContext context,
  ) {
    return _RenderDiagonal();
  }
}

class _RenderDiagonal extends RenderBox
    with SlottedContainerRenderObjectMixin<_DiagonalSlot?, RenderBox> {
  RenderBox? get _topLeft => childForSlot(_DiagonalSlot.topLeft);
  RenderBox? get _bottomRight => childForSlot(_DiagonalSlot.bottomRight);
  RenderBox? get _nullSlot => childForSlot(null);

  @override
  void performLayout() {
    const BoxConstraints childConstraints = BoxConstraints();

    Size topLeftSize = Size.zero;
    if (_topLeft != null) {
      _topLeft!.layout(childConstraints, parentUsesSize: true);
      _positionChild(_topLeft!, Offset.zero);
      topLeftSize = _topLeft!.size;
    }

    Size bottomRightSize = Size.zero;
    if (_bottomRight != null) {
      _bottomRight!.layout(childConstraints, parentUsesSize: true);
      _positionChild(_bottomRight!, Offset(topLeftSize.width, topLeftSize.height));
      bottomRightSize = _bottomRight!.size;
    }

    Size nullSlotSize = Size.zero;
    final RenderBox? nullSlot = _nullSlot;
    if (nullSlot != null) {
      nullSlot.layout(childConstraints, parentUsesSize: true);
      _positionChild(nullSlot, Offset.zero);
      nullSlotSize = nullSlot.size;
    }

    size = constraints.constrain(
      Size(
        topLeftSize.width + bottomRightSize.width + nullSlotSize.width,
        topLeftSize.height + bottomRightSize.height + nullSlotSize.height,
      ),
    );
  }

  void _positionChild(RenderBox child, Offset offset) {
    (child.parentData! as BoxParentData).offset = offset;
  }

  @override
  void paint(PaintingContext context, Offset offset) {
    if (_topLeft != null) {
      _paintChild(_topLeft!, context, offset);
    }
    if (_bottomRight != null) {
      _paintChild(_bottomRight!, context, offset);
    }
  }

  void _paintChild(RenderBox child, PaintingContext context, Offset offset) {
    final BoxParentData childParentData = child.parentData! as BoxParentData;
    context.paintChild(child, childParentData.offset + offset);
  }

  String publicNameForSlot(_DiagonalSlot slot) => debugNameForSlot(slot);
}

class _Slot {
  @override
  String toString() => describeIdentity(this);
}

class _RenderTest extends RenderBox with SlottedContainerRenderObjectMixin<_Slot, RenderBox> {
  String publicNameForSlot(_Slot slot) => debugNameForSlot(slot);
}
