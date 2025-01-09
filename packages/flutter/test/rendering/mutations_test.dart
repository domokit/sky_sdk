// Copyright 2014 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

import 'package:flutter/rendering.dart';
import 'package:flutter_test/flutter_test.dart';

import 'rendering_tester.dart';

class RenderLayoutTestBox extends RenderProxyBox {
  RenderLayoutTestBox(this.onLayout, {this.onPerformLayout});

  final VoidCallback onLayout;
  final VoidCallback? onPerformLayout;

  @override
  void layout(Constraints constraints, {bool parentUsesSize = false}) {
    // Doing this in tests is ok, but if you're writing your own
    // render object, you want to override performLayout(), not
    // layout(). Overriding layout() would remove many critical
    // performance optimizations of the rendering system, as well as
    // many bypassing many checked-mode integrity checks.
    super.layout(constraints, parentUsesSize: parentUsesSize);
    onLayout();
  }

  @override
  bool get sizedByParent => true;

  @override
  void performLayout() {
    child?.layout(constraints, parentUsesSize: true);
    onPerformLayout?.call();
  }
}

void main() {
  TestRenderingFlutterBinding.ensureInitialized();

  test('moving children', () {
    RenderBox child1, child2;
    bool movedChild1 = false;
    bool movedChild2 = false;
    final RenderFlex block = RenderFlex(textDirection: TextDirection.ltr);
    block.add(
      child1 = RenderLayoutTestBox(() {
        movedChild1 = true;
      }),
    );
    block.add(
      child2 = RenderLayoutTestBox(() {
        movedChild2 = true;
      }),
    );

    expect(movedChild1, isFalse);
    expect(movedChild2, isFalse);
    layout(block);
    expect(movedChild1, isTrue);
    expect(movedChild2, isTrue);

    movedChild1 = false;
    movedChild2 = false;

    expect(movedChild1, isFalse);
    expect(movedChild2, isFalse);
    pumpFrame();
    expect(movedChild1, isFalse);
    expect(movedChild2, isFalse);

    block.move(child1, after: child2);
    expect(movedChild1, isFalse);
    expect(movedChild2, isFalse);
    pumpFrame();
    expect(movedChild1, isTrue);
    expect(movedChild2, isTrue);

    movedChild1 = false;
    movedChild2 = false;

    expect(movedChild1, isFalse);
    expect(movedChild2, isFalse);
    pumpFrame();
    expect(movedChild1, isFalse);
    expect(movedChild2, isFalse);
  });

  group('Throws when illegal mutations are attempted: ', () {
    FlutterError catchLayoutError(RenderBox box) {
      Object? error;
      layout(
        box,
        onErrors: () {
          error = TestRenderingFlutterBinding.instance.takeFlutterErrorDetails()!.exception;
        },
      );
      expect(error, isFlutterError);
      return error! as FlutterError;
    }

    test('on disposed render objects', () {
      final RenderBox box = RenderLayoutTestBox(() {});
      box.dispose();

      Object? error;
      try {
        box.markNeedsLayout();
      } catch (e) {
        error = e;
      }

      expect(error, isFlutterError);
      expect(
        (error! as FlutterError).message,
        equalsIgnoringWhitespace(
          'A disposed RenderObject was mutated.\n'
          'The disposed RenderObject was:\n'
          '${box.toStringShort()}',
        ),
      );
    });

    test('marking itself dirty in performLayout', () {
      late RenderBox child1;
      final RenderFlex block = RenderFlex(textDirection: TextDirection.ltr);
      block.add(
        child1 = RenderLayoutTestBox(
          () {},
          onPerformLayout: () {
            child1.markNeedsLayout();
          },
        ),
      );

      expect(
        catchLayoutError(block).message,
        equalsIgnoringWhitespace(
          'A RenderLayoutTestBox was mutated in its own performLayout implementation.\n'
          'A RenderObject must not re-dirty itself while still being laid out.\n'
          'The RenderObject being mutated was:\n'
          '${child1.toStringShort()}\n'
          'Consider using the LayoutBuilder widget to dynamically change a subtree during layout.',
        ),
      );
    });

    test('marking a sibling dirty in performLayout', () {
      late RenderBox child1, child2;
      final RenderFlex block = RenderFlex(textDirection: TextDirection.ltr);
      block.add(child1 = RenderLayoutTestBox(() {}));
      block.add(
        child2 = RenderLayoutTestBox(
          () {},
          onPerformLayout: () {
            child1.markNeedsLayout();
          },
        ),
      );

      expect(
        catchLayoutError(block).message,
        equalsIgnoringWhitespace(
          'A RenderLayoutTestBox was mutated in RenderLayoutTestBox.performLayout.\n'
          'A RenderObject must not mutate another RenderObject from a different render subtree in its performLayout method.\n'
          'The RenderObject being mutated was:\n'
          '${child1.toStringShort()}\n'
          'The RenderObject that was mutating the said RenderLayoutTestBox was:\n'
          '${child2.toStringShort()}\n'
          'Their common ancestor was:\n'
          '${block.toStringShort()}\n'
          'Mutating the layout of another RenderObject may cause some RenderObjects in its subtree to be laid out more than once. Consider using the LayoutBuilder widget to dynamically mutate a subtree during layout.',
        ),
      );
    });

    test('marking a descendant dirty in performLayout', () {
      late RenderBox child1;
      final RenderFlex block = RenderFlex(textDirection: TextDirection.ltr);
      block.add(child1 = RenderLayoutTestBox(() {}));
      block.add(RenderLayoutTestBox(child1.markNeedsLayout));

      expect(
        catchLayoutError(block).message,
        equalsIgnoringWhitespace(
          'A RenderLayoutTestBox was mutated in RenderFlex.performLayout.\n'
          'A RenderObject must not mutate its descendants in its performLayout method.\n'
          'The RenderObject being mutated was:\n'
          '${child1.toStringShort()}\n'
          'The ancestor RenderObject that was mutating the said RenderLayoutTestBox was:\n'
          '${block.toStringShort()}\n'
          'Mutating the layout of another RenderObject may cause some RenderObjects in its subtree to be laid out more than once. Consider using the LayoutBuilder widget to dynamically mutate a subtree during layout.',
        ),
      );
    });

    test('marking an out-of-band mutation in performLayout', () {
      late RenderProxyBox child1, child11, child2, child21;
      final RenderFlex block = RenderFlex(textDirection: TextDirection.ltr);
      block.add(child1 = RenderLayoutTestBox(() {}));
      block.add(child2 = RenderLayoutTestBox(() {}));
      child1.child = child11 = RenderLayoutTestBox(() {});
      layout(block);

      expect(block.debugNeedsLayout, false);
      expect(child1.debugNeedsLayout, false);
      expect(child11.debugNeedsLayout, false);
      expect(child2.debugNeedsLayout, false);

      // Add a new child to child2 which is a relayout boundary.
      child2.child = child21 = RenderLayoutTestBox(() {}, onPerformLayout: child11.markNeedsLayout);

      FlutterError? error;
      pumpFrame(
        onErrors: () {
          error =
              TestRenderingFlutterBinding.instance.takeFlutterErrorDetails()!.exception
                  as FlutterError;
        },
      );

      expect(
        error?.message,
        equalsIgnoringWhitespace(
          'A RenderLayoutTestBox was mutated in RenderLayoutTestBox.performLayout.\n'
          'The RenderObject was mutated when none of its ancestors is actively performing layout.\n'
          'The RenderObject being mutated was:\n'
          '${child11.toStringShort()}\n'
          'The RenderObject that was mutating the said RenderLayoutTestBox was:\n'
          '${child21.toStringShort()}',
        ),
      );
    });
  });
}
