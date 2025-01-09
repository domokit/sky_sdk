// Copyright 2014 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

import 'package:flutter/rendering.dart';
import 'package:flutter_test/flutter_test.dart';

import 'rendering_tester.dart';

void main() {
  TestRenderingFlutterBinding.ensureInitialized();

  test('StackParentData basic test', () {
    final StackParentData parentData = StackParentData();
    const Size stackSize = Size(800.0, 600.0);
    expect(parentData.isPositioned, isFalse);

    parentData.width = -100.0;
    expect(parentData.isPositioned, isTrue);
    expect(
      parentData.positionedChildConstraints(stackSize),
      const BoxConstraints.tightFor(width: 0.0),
    );

    parentData.width = 100.0;
    expect(
      parentData.positionedChildConstraints(stackSize),
      const BoxConstraints.tightFor(width: 100.0),
    );

    parentData.left = 0.0;
    parentData.right = 0.0;
    expect(
      parentData.positionedChildConstraints(stackSize),
      const BoxConstraints.tightFor(width: 800.0),
    );

    parentData.height = -100.0;
    expect(
      parentData.positionedChildConstraints(stackSize),
      const BoxConstraints.tightFor(width: 800.0, height: 0.0),
    );

    parentData.height = 100.0;
    expect(
      parentData.positionedChildConstraints(stackSize),
      const BoxConstraints.tightFor(width: 800.0, height: 100.0),
    );

    parentData.top = 0.0;
    parentData.bottom = 0.0;
    expect(
      parentData.positionedChildConstraints(stackSize),
      const BoxConstraints.tightFor(width: 800.0, height: 600.0),
    );

    parentData.bottom = 1000.0;
    expect(
      parentData.positionedChildConstraints(stackSize),
      const BoxConstraints.tightFor(width: 800.0, height: 0.0),
    );
  });

  test('Stack can layout with top, right, bottom, left 0.0', () {
    final RenderBox size = RenderConstrainedBox(
      additionalConstraints: BoxConstraints.tight(const Size(100.0, 100.0)),
    );

    final RenderBox red = RenderDecoratedBox(
      decoration: const BoxDecoration(color: Color(0xFFFF0000)),
      child: size,
    );

    final RenderBox green = RenderDecoratedBox(
      decoration: const BoxDecoration(color: Color(0xFFFF0000)),
    );

    final RenderBox stack = RenderStack(
      textDirection: TextDirection.ltr,
      children: <RenderBox>[red, green],
    );
    final StackParentData greenParentData = green.parentData! as StackParentData;
    greenParentData
      ..top = 0.0
      ..right = 0.0
      ..bottom = 0.0
      ..left = 0.0;

    layout(stack, constraints: const BoxConstraints());

    expect(stack.size.width, equals(100.0));
    expect(stack.size.height, equals(100.0));

    expect(red.size.width, equals(100.0));
    expect(red.size.height, equals(100.0));

    expect(green.size.width, equals(100.0));
    expect(green.size.height, equals(100.0));
  });

  test('Stack can layout with no children', () {
    final RenderBox stack = RenderStack(textDirection: TextDirection.ltr, children: <RenderBox>[]);

    layout(stack, constraints: BoxConstraints.tight(const Size(100.0, 100.0)));

    expect(stack.size.width, equals(100.0));
    expect(stack.size.height, equals(100.0));
  });

  test('Stack has correct clipBehavior', () {
    const BoxConstraints viewport = BoxConstraints(maxHeight: 100.0, maxWidth: 100.0);

    for (final Clip? clip in <Clip?>[null, ...Clip.values]) {
      final TestClipPaintingContext context = TestClipPaintingContext();
      final RenderBox child = box200x200;
      final RenderStack stack;
      switch (clip) {
        case Clip.none:
        case Clip.hardEdge:
        case Clip.antiAlias:
        case Clip.antiAliasWithSaveLayer:
          stack = RenderStack(
            textDirection: TextDirection.ltr,
            children: <RenderBox>[child],
            clipBehavior: clip!,
          );
        case null:
          stack = RenderStack(textDirection: TextDirection.ltr, children: <RenderBox>[child]);
      }
      {
        // Make sure that the child is positioned so the stack will consider it as overflowed.
        final StackParentData parentData = child.parentData! as StackParentData;
        parentData.left = parentData.right = 0;
      }
      layout(
        stack,
        constraints: viewport,
        phase: EnginePhase.composite,
        onErrors: expectNoFlutterErrors,
      );
      context.paintChild(stack, Offset.zero);
      // By default, clipBehavior should be Clip.hardEdge
      expect(context.clipBehavior, equals(clip ?? Clip.hardEdge), reason: 'for $clip');
    }
  });

  group('RenderIndexedStack', () {
    test('visitChildrenForSemantics only visits displayed child', () {
      final RenderBox child1 = RenderConstrainedBox(
        additionalConstraints: BoxConstraints.tight(const Size(100.0, 100.0)),
      );
      final RenderBox child2 = RenderConstrainedBox(
        additionalConstraints: BoxConstraints.tight(const Size(100.0, 100.0)),
      );
      final RenderBox child3 = RenderConstrainedBox(
        additionalConstraints: BoxConstraints.tight(const Size(100.0, 100.0)),
      );
      final RenderBox stack = RenderIndexedStack(
        index: 1,
        textDirection: TextDirection.ltr,
        children: <RenderBox>[child1, child2, child3],
      );

      final List<RenderObject> visitedChildren = <RenderObject>[];
      void visitor(RenderObject child) {
        visitedChildren.add(child);
      }

      layout(stack);
      stack.visitChildrenForSemantics(visitor);

      expect(visitedChildren, hasLength(1));
      expect(visitedChildren.first, child2);
    });

    test('debugDescribeChildren marks invisible children as offstage', () {
      final RenderBox child1 = RenderConstrainedBox(
        additionalConstraints: BoxConstraints.tight(const Size(100.0, 100.0)),
      );
      final RenderBox child2 = RenderConstrainedBox(
        additionalConstraints: BoxConstraints.tight(const Size(100.0, 100.0)),
      );
      final RenderBox child3 = RenderConstrainedBox(
        additionalConstraints: BoxConstraints.tight(const Size(100.0, 100.0)),
      );

      final RenderBox stack = RenderIndexedStack(
        index: 2,
        children: <RenderBox>[child1, child2, child3],
      );

      final List<DiagnosticsNode> diagnosticNodes = stack.debugDescribeChildren();

      expect(diagnosticNodes[0].name, 'child 1');
      expect(diagnosticNodes[0].style, DiagnosticsTreeStyle.offstage);

      expect(diagnosticNodes[1].name, 'child 2');
      expect(diagnosticNodes[1].style, DiagnosticsTreeStyle.offstage);

      expect(diagnosticNodes[2].name, 'child 3');
      expect(diagnosticNodes[2].style, DiagnosticsTreeStyle.sparse);
    });

    test('debugDescribeChildren handles a null index', () {
      final RenderBox child1 = RenderConstrainedBox(
        additionalConstraints: BoxConstraints.tight(const Size(100.0, 100.0)),
      );
      final RenderBox child2 = RenderConstrainedBox(
        additionalConstraints: BoxConstraints.tight(const Size(100.0, 100.0)),
      );
      final RenderBox child3 = RenderConstrainedBox(
        additionalConstraints: BoxConstraints.tight(const Size(100.0, 100.0)),
      );

      final RenderBox stack = RenderIndexedStack(
        index: null,
        children: <RenderBox>[child1, child2, child3],
      );

      final List<DiagnosticsNode> diagnosticNodes = stack.debugDescribeChildren();

      expect(diagnosticNodes[0].name, 'child 1');
      expect(diagnosticNodes[0].style, DiagnosticsTreeStyle.offstage);

      expect(diagnosticNodes[1].name, 'child 2');
      expect(diagnosticNodes[1].style, DiagnosticsTreeStyle.offstage);

      expect(diagnosticNodes[2].name, 'child 3');
      expect(diagnosticNodes[2].style, DiagnosticsTreeStyle.offstage);
    });
  });

  test('Stack in Flex can layout with no children', () {
    // Render an empty Stack in a Flex
    final RenderFlex flex = RenderFlex(
      textDirection: TextDirection.ltr,
      mainAxisAlignment: MainAxisAlignment.spaceEvenly,
      children: <RenderBox>[RenderStack(textDirection: TextDirection.ltr, children: <RenderBox>[])],
    );

    bool stackFlutterErrorThrown = false;
    layout(
      flex,
      constraints: BoxConstraints.tight(const Size(100.0, 100.0)),
      onErrors: () {
        stackFlutterErrorThrown = true;
      },
    );

    expect(stackFlutterErrorThrown, false);
  });

  // More tests in ../widgets/stack_test.dart
}
