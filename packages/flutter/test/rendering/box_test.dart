// Copyright 2014 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

import 'package:flutter/foundation.dart';
import 'package:flutter/gestures.dart';
import 'package:flutter/material.dart';
import 'package:flutter/rendering.dart';
import 'package:flutter_test/flutter_test.dart';

import 'rendering_tester.dart';

class MissingPerformLayoutRenderBox extends RenderBox {
  void triggerExceptionSettingSizeOutsideOfLayout() {
    size = const Size(200, 200);
  }

  // performLayout is left unimplemented to test the error reported if it is
  // missing.
}

class FakeMissingSizeRenderBox extends RenderBox {
  @override
  void performLayout() {
    size = constraints.biggest;
  }

  @override
  bool get hasSize => !fakeMissingSize && super.hasSize;

  bool fakeMissingSize = false;
}

class MissingSetSizeRenderBox extends RenderBox {
  @override
  void performLayout() {}
}

class BadBaselineRenderBox extends RenderBox {
  @override
  void performLayout() {
    size = constraints.biggest;
  }

  @override
  double? computeDistanceToActualBaseline(TextBaseline baseline) {
    throw Exception();
  }
}

void main() {
  TestRenderingFlutterBinding.ensureInitialized();

  test('should size to render view', () {
    final RenderBox root = RenderDecoratedBox(
      decoration: BoxDecoration(
        color: const Color(0xFF00FF00),
        gradient: RadialGradient(
          center: Alignment.topLeft,
          radius: 1.8,
          colors: <Color>[Colors.yellow[500]!, Colors.blue[500]!],
        ),
        boxShadow: kElevationToShadow[3],
      ),
    );
    layout(root);
    expect(root.size.width, equals(800.0));
    expect(root.size.height, equals(600.0));
  });

  test('performLayout error message', () {
    late FlutterError result;
    try {
      MissingPerformLayoutRenderBox().performLayout();
    } on FlutterError catch (e) {
      result = e;
    }
    expect(result, isNotNull);
    expect(
      result.toStringDeep(),
      equalsIgnoringHashCodes(
        'FlutterError\n'
        '   MissingPerformLayoutRenderBox did not implement performLayout().\n'
        '   RenderBox subclasses need to either override performLayout() to\n'
        '   set a size and lay out any children, or, set sizedByParent to\n'
        '   true so that performResize() sizes the render object.\n',
      ),
    );
    expect(
      result.diagnostics
          .singleWhere((DiagnosticsNode node) => node.level == DiagnosticLevel.hint)
          .toString(),
      'RenderBox subclasses need to either override performLayout() to set a '
      'size and lay out any children, or, set sizedByParent to true so that '
      'performResize() sizes the render object.',
    );
  });

  test('applyPaintTransform error message', () {
    final RenderBox paddingBox = RenderPadding(padding: const EdgeInsets.all(10.0));
    final RenderBox root = RenderPadding(padding: const EdgeInsets.all(10.0), child: paddingBox);
    layout(root);
    // Trigger the error by overriding the parentData with data that isn't a
    // BoxParentData.
    paddingBox.parentData = ParentData();

    late FlutterError result;
    try {
      root.applyPaintTransform(paddingBox, Matrix4.identity());
    } on FlutterError catch (e) {
      result = e;
    }
    expect(result, isNotNull);
    expect(
      result.toStringDeep(),
      equalsIgnoringHashCodes(
        'FlutterError\n'
        '   RenderPadding does not implement applyPaintTransform.\n'
        '   The following RenderPadding object: RenderPadding#00000 NEEDS-PAINT NEEDS-COMPOSITING-BITS-UPDATE:\n'
        '     parentData: <none>\n'
        '     constraints: BoxConstraints(w=800.0, h=600.0)\n'
        '     size: Size(800.0, 600.0)\n'
        '     padding: EdgeInsets.all(10.0)\n'
        '   ...did not use a BoxParentData class for the parentData field of the following child:\n'
        '     RenderPadding#00000 NEEDS-PAINT:\n'
        '     parentData: <none> (can use size)\n'
        '     constraints: BoxConstraints(w=780.0, h=580.0)\n'
        '     size: Size(780.0, 580.0)\n'
        '     padding: EdgeInsets.all(10.0)\n'
        '   The RenderPadding class inherits from RenderBox.\n'
        '   The default applyPaintTransform implementation provided by\n'
        '   RenderBox assumes that the children all use BoxParentData objects\n'
        '   for their parentData field. Since RenderPadding does not in fact\n'
        '   use that ParentData class for its children, it must provide an\n'
        '   implementation of applyPaintTransform that supports the specific\n'
        '   ParentData subclass used by its children (which apparently is\n'
        '   ParentData).\n',
      ),
    );

    expect(
      result.diagnostics
          .singleWhere((DiagnosticsNode node) => node.level == DiagnosticLevel.hint)
          .toString(),
      'The default applyPaintTransform implementation provided by RenderBox '
      'assumes that the children all use BoxParentData objects for their '
      'parentData field. Since RenderPadding does not in fact use that '
      'ParentData class for its children, it must provide an implementation '
      'of applyPaintTransform that supports the specific ParentData subclass '
      'used by its children (which apparently is ParentData).',
    );
  });

  test('Set size error messages', () {
    final RenderBox root = RenderDecoratedBox(
      decoration: const BoxDecoration(color: Color(0xFF00FF00)),
    );
    layout(root);

    final MissingPerformLayoutRenderBox testBox = MissingPerformLayoutRenderBox();
    {
      late FlutterError result;
      try {
        testBox.triggerExceptionSettingSizeOutsideOfLayout();
      } on FlutterError catch (e) {
        result = e;
      }
      expect(result, isNotNull);
      expect(
        result.toStringDeep(),
        equalsIgnoringHashCodes(
          'FlutterError\n'
          '   RenderBox size setter called incorrectly.\n'
          '   The size setter was called from outside layout (neither\n'
          '   performResize() nor performLayout() were being run for this\n'
          '   object).\n'
          '   Because this RenderBox has sizedByParent set to false, it must\n'
          '   set its size in performLayout().\n',
        ),
      );
      expect(
        result.diagnostics.where((DiagnosticsNode node) => node.level == DiagnosticLevel.hint),
        isEmpty,
      );
    }
    {
      late FlutterError result;
      try {
        testBox.debugAdoptSize(root.size);
      } on FlutterError catch (e) {
        result = e;
      }
      expect(result, isNotNull);
      expect(
        result.toStringDeep(wrapWidth: 640),
        equalsIgnoringHashCodes(
          'FlutterError\n'
          '   The size property was assigned a size inappropriately.\n'
          '   The following render object: MissingPerformLayoutRenderBox#00000 NEEDS-LAYOUT NEEDS-PAINT DETACHED:\n'
          '     parentData: MISSING\n'
          '     constraints: MISSING\n'
          '     size: MISSING\n'
          '   ...was assigned a size obtained from: RenderDecoratedBox#00000 NEEDS-PAINT:\n'
          '     parentData: <none>\n'
          '     constraints: BoxConstraints(w=800.0, h=600.0)\n'
          '     size: Size(800.0, 600.0)\n'
          '     decoration: BoxDecoration:\n'
          '       color: ${const Color(0xff00ff00)}\n'
          '     configuration: ImageConfiguration()\n'
          '   However, this second render object is not, or is no longer, a '
          'child of the first, and it is therefore a violation of the '
          'RenderBox layout protocol to use that size in the layout of the '
          'first render object.\n'
          '   If the size was obtained at a time where it was valid to read '
          'the size (because the second render object above was a child of '
          'the first at the time), then it should be adopted using '
          'debugAdoptSize at that time.\n'
          '   If the size comes from a grandchild or a render object from an '
          'entirely different part of the render tree, then there is no way '
          'to be notified when the size changes and therefore attempts to '
          'read that size are almost certainly a source of bugs. A different '
          'approach should be used.\n',
        ),
      );
      expect(
        result.diagnostics
            .where((DiagnosticsNode node) => node.level == DiagnosticLevel.hint)
            .length,
        2,
      );
    }
  });

  test('Flex and padding', () {
    final RenderBox size = RenderConstrainedBox(
      additionalConstraints: const BoxConstraints().tighten(height: 100.0),
    );
    final RenderBox inner = RenderDecoratedBox(
      decoration: const BoxDecoration(color: Color(0xFF00FF00)),
      child: size,
    );
    final RenderBox padding = RenderPadding(padding: const EdgeInsets.all(50.0), child: inner);
    final RenderBox flex = RenderFlex(
      children: <RenderBox>[padding],
      direction: Axis.vertical,
      crossAxisAlignment: CrossAxisAlignment.stretch,
    );
    final RenderBox outer = RenderDecoratedBox(
      decoration: const BoxDecoration(color: Color(0xFF0000FF)),
      child: flex,
    );

    layout(outer);

    expect(size.size.width, equals(700.0));
    expect(size.size.height, equals(100.0));
    expect(inner.size.width, equals(700.0));
    expect(inner.size.height, equals(100.0));
    expect(padding.size.width, equals(800.0));
    expect(padding.size.height, equals(200.0));
    expect(flex.size.width, equals(800.0));
    expect(flex.size.height, equals(600.0));
    expect(outer.size.width, equals(800.0));
    expect(outer.size.height, equals(600.0));
  });

  test('should not have a 0 sized colored Box', () {
    final RenderBox coloredBox = RenderDecoratedBox(decoration: const BoxDecoration());

    expect(coloredBox, hasAGoodToStringDeep);
    expect(
      coloredBox.toStringDeep(minLevel: DiagnosticLevel.info),
      equalsIgnoringHashCodes(
        'RenderDecoratedBox#00000 NEEDS-LAYOUT NEEDS-PAINT DETACHED\n'
        '   parentData: MISSING\n'
        '   constraints: MISSING\n'
        '   size: MISSING\n'
        '   decoration: BoxDecoration:\n'
        '     <no decorations specified>\n'
        '   configuration: ImageConfiguration()\n',
      ),
    );

    final RenderBox paddingBox = RenderPadding(
      padding: const EdgeInsets.all(10.0),
      child: coloredBox,
    );
    final RenderBox root = RenderDecoratedBox(decoration: const BoxDecoration(), child: paddingBox);
    layout(root);
    expect(coloredBox.size.width, equals(780.0));
    expect(coloredBox.size.height, equals(580.0));

    expect(coloredBox, hasAGoodToStringDeep);
    expect(
      coloredBox.toStringDeep(minLevel: DiagnosticLevel.info),
      equalsIgnoringHashCodes(
        'RenderDecoratedBox#00000 NEEDS-PAINT\n'
        '   parentData: offset=Offset(10.0, 10.0) (can use size)\n'
        '   constraints: BoxConstraints(w=780.0, h=580.0)\n'
        '   size: Size(780.0, 580.0)\n'
        '   decoration: BoxDecoration:\n'
        '     <no decorations specified>\n'
        '   configuration: ImageConfiguration()\n',
      ),
    );
  });

  test('reparenting should clear position', () {
    final RenderDecoratedBox coloredBox = RenderDecoratedBox(decoration: const BoxDecoration());

    final RenderPadding paddedBox = RenderPadding(
      child: coloredBox,
      padding: const EdgeInsets.all(10.0),
    );
    layout(paddedBox);
    final BoxParentData parentData = coloredBox.parentData! as BoxParentData;
    expect(parentData.offset.dx, isNot(equals(0.0)));
    paddedBox.child = null;

    final RenderConstrainedBox constrainedBox = RenderConstrainedBox(
      child: coloredBox,
      additionalConstraints: const BoxConstraints(),
    );
    layout(constrainedBox);
    expect(coloredBox.parentData?.runtimeType, ParentData);
  });

  test('UnconstrainedBox expands to fit children', () {
    final RenderConstraintsTransformBox unconstrained = RenderConstraintsTransformBox(
      constraintsTransform: ConstraintsTransformBox.widthUnconstrained,
      textDirection: TextDirection.ltr,
      child: RenderConstrainedBox(
        additionalConstraints: const BoxConstraints.tightFor(width: 200.0, height: 200.0),
      ),
      alignment: Alignment.center,
    );
    layout(
      unconstrained,
      constraints: const BoxConstraints(
        minWidth: 200.0,
        maxWidth: 200.0,
        minHeight: 200.0,
        maxHeight: 200.0,
      ),
    );
    // Check that we can update the constrained axis to null.
    unconstrained.constraintsTransform = ConstraintsTransformBox.unconstrained;
    TestRenderingFlutterBinding.instance.reassembleApplication();

    expect(unconstrained.size.width, equals(200.0), reason: 'unconstrained width');
    expect(unconstrained.size.height, equals(200.0), reason: 'unconstrained height');
  });

  test('UnconstrainedBox handles vertical overflow', () {
    final RenderConstraintsTransformBox unconstrained = RenderConstraintsTransformBox(
      constraintsTransform: ConstraintsTransformBox.unconstrained,
      textDirection: TextDirection.ltr,
      child: RenderConstrainedBox(
        additionalConstraints: const BoxConstraints.tightFor(height: 200.0),
      ),
      alignment: Alignment.center,
    );
    const BoxConstraints viewport = BoxConstraints(maxHeight: 100.0, maxWidth: 100.0);
    layout(unconstrained, constraints: viewport);
    expect(unconstrained.getMinIntrinsicHeight(100.0), equals(200.0));
    expect(unconstrained.getMaxIntrinsicHeight(100.0), equals(200.0));
    expect(unconstrained.getMinIntrinsicWidth(100.0), equals(0.0));
    expect(unconstrained.getMaxIntrinsicWidth(100.0), equals(0.0));
  });

  test('UnconstrainedBox handles horizontal overflow', () {
    final RenderConstraintsTransformBox unconstrained = RenderConstraintsTransformBox(
      constraintsTransform: ConstraintsTransformBox.unconstrained,
      textDirection: TextDirection.ltr,
      child: RenderConstrainedBox(
        additionalConstraints: const BoxConstraints.tightFor(width: 200.0),
      ),
      alignment: Alignment.center,
    );
    const BoxConstraints viewport = BoxConstraints(maxHeight: 100.0, maxWidth: 100.0);
    layout(unconstrained, constraints: viewport);
    expect(unconstrained.getMinIntrinsicHeight(100.0), equals(0.0));
    expect(unconstrained.getMaxIntrinsicHeight(100.0), equals(0.0));
    expect(unconstrained.getMinIntrinsicWidth(100.0), equals(200.0));
    expect(unconstrained.getMaxIntrinsicWidth(100.0), equals(200.0));
  });

  group('ConstraintsTransformBox', () {
    FlutterErrorDetails? firstErrorDetails;
    void exhaustErrors() {
      FlutterErrorDetails? next;
      do {
        next = TestRenderingFlutterBinding.instance.takeFlutterErrorDetails();
        firstErrorDetails ??= next;
      } while (next != null);
    }

    tearDown(() {
      firstErrorDetails = null;
      RenderObject.debugCheckingIntrinsics = false;
    });

    test('throws if the resulting constraints are not normalized', () {
      final RenderConstrainedBox child = RenderConstrainedBox(
        additionalConstraints: const BoxConstraints.tightFor(height: 0),
      );
      final RenderConstraintsTransformBox box = RenderConstraintsTransformBox(
        alignment: Alignment.center,
        textDirection: TextDirection.ltr,
        constraintsTransform:
            (BoxConstraints constraints) => const BoxConstraints(maxHeight: -1, minHeight: 200),
        child: child,
      );

      layout(box, constraints: const BoxConstraints(), onErrors: exhaustErrors);

      expect(firstErrorDetails?.toString(), contains('is not normalized'));
    });

    test('overflow is reported when insufficient size is given and clipBehavior is Clip.none', () {
      bool hadErrors = false;
      void expectOverflowedErrors() {
        absorbOverflowedErrors();
        hadErrors = true;
      }

      final TestClipPaintingContext context = TestClipPaintingContext();
      for (final Clip? clip in <Clip?>[null, ...Clip.values]) {
        final RenderConstraintsTransformBox box;
        switch (clip) {
          case Clip.none:
          case Clip.hardEdge:
          case Clip.antiAlias:
          case Clip.antiAliasWithSaveLayer:
            box = RenderConstraintsTransformBox(
              alignment: Alignment.center,
              textDirection: TextDirection.ltr,
              constraintsTransform:
                  (BoxConstraints constraints) => constraints.copyWith(maxWidth: double.infinity),
              clipBehavior: clip!,
              child: RenderConstrainedBox(
                additionalConstraints: const BoxConstraints.tightFor(
                  width: double.maxFinite,
                  height: double.maxFinite,
                ),
              ),
            );
          case null:
            box = RenderConstraintsTransformBox(
              alignment: Alignment.center,
              textDirection: TextDirection.ltr,
              constraintsTransform:
                  (BoxConstraints constraints) => constraints.copyWith(maxWidth: double.infinity),
              child: RenderConstrainedBox(
                additionalConstraints: const BoxConstraints.tightFor(
                  width: double.maxFinite,
                  height: double.maxFinite,
                ),
              ),
            );
        }
        layout(
          box,
          constraints: const BoxConstraints(),
          phase: EnginePhase.composite,
          onErrors: expectOverflowedErrors,
        );
        context.paintChild(box, Offset.zero);
        // By default, clipBehavior should be Clip.none
        expect(context.clipBehavior, equals(clip ?? Clip.none));
        switch (clip) {
          case null:
          case Clip.none:
            expect(hadErrors, isTrue, reason: 'Should have had overflow errors for $clip');
          case Clip.hardEdge:
          case Clip.antiAlias:
          case Clip.antiAliasWithSaveLayer:
            expect(hadErrors, isFalse, reason: 'Should not have had overflow errors for $clip');
        }
        hadErrors = false;
      }
    });

    test('handles flow layout', () {
      final RenderParagraph child = RenderParagraph(
        TextSpan(text: 'a' * 100),
        textDirection: TextDirection.ltr,
      );
      final RenderConstraintsTransformBox box = RenderConstraintsTransformBox(
        alignment: Alignment.center,
        textDirection: TextDirection.ltr,
        constraintsTransform:
            (BoxConstraints constraints) => constraints.copyWith(maxWidth: double.infinity),
        child: child,
      );

      // With a width of 30, the RenderParagraph would have wrapped, but the
      // RenderConstraintsTransformBox allows the paragraph to expand regardless
      // of the width constraint:
      // unconstrainedHeight * numberOfLines = constrainedHeight.
      final double constrainedHeight = child.getMinIntrinsicHeight(30);
      final double unconstrainedHeight = box.getMinIntrinsicHeight(30);

      // At least 2 lines.
      expect(constrainedHeight, greaterThanOrEqualTo(2 * unconstrainedHeight));
    });

    test('paints even when its size is empty', () {
      // Regression test for https://github.com/flutter/flutter/issues/146840.
      final RenderParagraph child = RenderParagraph(
        const TextSpan(text: ''),
        textDirection: TextDirection.ltr,
      );
      final RenderConstraintsTransformBox box = RenderConstraintsTransformBox(
        alignment: Alignment.center,
        textDirection: TextDirection.ltr,
        constraintsTransform:
            (BoxConstraints constraints) => constraints.copyWith(maxWidth: double.infinity),
        child: child,
      );

      layout(box, constraints: BoxConstraints.tight(Size.zero), phase: EnginePhase.paint);
      expect(box, paints..paragraph());
    });
  });

  test('getMinIntrinsicWidth error handling', () {
    final RenderConstraintsTransformBox unconstrained = RenderConstraintsTransformBox(
      constraintsTransform: ConstraintsTransformBox.unconstrained,
      textDirection: TextDirection.ltr,
      child: RenderConstrainedBox(
        additionalConstraints: const BoxConstraints.tightFor(width: 200.0),
      ),
      alignment: Alignment.center,
    );
    const BoxConstraints viewport = BoxConstraints(maxHeight: 100.0, maxWidth: 100.0);
    layout(unconstrained, constraints: viewport);

    {
      late FlutterError result;
      try {
        unconstrained.getMinIntrinsicWidth(-1);
      } on FlutterError catch (e) {
        result = e;
      }
      expect(result, isNotNull);
      expect(
        result.toStringDeep(),
        equalsIgnoringHashCodes(
          'FlutterError\n'
          '   The height argument to getMinIntrinsicWidth was negative.\n'
          '   The argument to getMinIntrinsicWidth must not be negative or\n'
          '   null.\n'
          '   If you perform computations on another height before passing it\n'
          '   to getMinIntrinsicWidth, consider using math.max() or\n'
          '   double.clamp() to force the value into the valid range.\n',
        ),
      );
      expect(
        result.diagnostics
            .singleWhere((DiagnosticsNode node) => node.level == DiagnosticLevel.hint)
            .toString(),
        'If you perform computations on another height before passing it to '
        'getMinIntrinsicWidth, consider using math.max() or double.clamp() '
        'to force the value into the valid range.',
      );
    }

    {
      late FlutterError result;
      try {
        unconstrained.getMinIntrinsicHeight(-1);
      } on FlutterError catch (e) {
        result = e;
      }
      expect(result, isNotNull);
      expect(
        result.toStringDeep(),
        equalsIgnoringHashCodes(
          'FlutterError\n'
          '   The width argument to getMinIntrinsicHeight was negative.\n'
          '   The argument to getMinIntrinsicHeight must not be negative or\n'
          '   null.\n'
          '   If you perform computations on another width before passing it to\n'
          '   getMinIntrinsicHeight, consider using math.max() or\n'
          '   double.clamp() to force the value into the valid range.\n',
        ),
      );
      expect(
        result.diagnostics
            .singleWhere((DiagnosticsNode node) => node.level == DiagnosticLevel.hint)
            .toString(),
        'If you perform computations on another width before passing it to '
        'getMinIntrinsicHeight, consider using math.max() or double.clamp() '
        'to force the value into the valid range.',
      );
    }

    {
      late FlutterError result;
      try {
        unconstrained.getMaxIntrinsicWidth(-1);
      } on FlutterError catch (e) {
        result = e;
      }
      expect(result, isNotNull);
      expect(
        result.toStringDeep(),
        equalsIgnoringHashCodes(
          'FlutterError\n'
          '   The height argument to getMaxIntrinsicWidth was negative.\n'
          '   The argument to getMaxIntrinsicWidth must not be negative or\n'
          '   null.\n'
          '   If you perform computations on another height before passing it\n'
          '   to getMaxIntrinsicWidth, consider using math.max() or\n'
          '   double.clamp() to force the value into the valid range.\n',
        ),
      );
      expect(
        result.diagnostics
            .singleWhere((DiagnosticsNode node) => node.level == DiagnosticLevel.hint)
            .toString(),
        'If you perform computations on another height before passing it to '
        'getMaxIntrinsicWidth, consider using math.max() or double.clamp() '
        'to force the value into the valid range.',
      );
    }

    {
      late FlutterError result;
      try {
        unconstrained.getMaxIntrinsicHeight(-1);
      } on FlutterError catch (e) {
        result = e;
      }
      expect(result, isNotNull);
      expect(
        result.toStringDeep(),
        equalsIgnoringHashCodes(
          'FlutterError\n'
          '   The width argument to getMaxIntrinsicHeight was negative.\n'
          '   The argument to getMaxIntrinsicHeight must not be negative or\n'
          '   null.\n'
          '   If you perform computations on another width before passing it to\n'
          '   getMaxIntrinsicHeight, consider using math.max() or\n'
          '   double.clamp() to force the value into the valid range.\n',
        ),
      );
      expect(
        result.diagnostics
            .singleWhere((DiagnosticsNode node) => node.level == DiagnosticLevel.hint)
            .toString(),
        'If you perform computations on another width before passing it to '
        'getMaxIntrinsicHeight, consider using math.max() or double.clamp() '
        'to force the value into the valid range.',
      );
    }
  });

  test('UnconstrainedBox.toStringDeep returns useful information', () {
    final RenderConstraintsTransformBox unconstrained = RenderConstraintsTransformBox(
      constraintsTransform: ConstraintsTransformBox.unconstrained,
      textDirection: TextDirection.ltr,
      alignment: Alignment.center,
    );
    expect(unconstrained.alignment, Alignment.center);
    expect(unconstrained.textDirection, TextDirection.ltr);
    expect(unconstrained, hasAGoodToStringDeep);
    expect(
      unconstrained.toStringDeep(minLevel: DiagnosticLevel.info),
      equalsIgnoringHashCodes(
        'RenderConstraintsTransformBox#00000 NEEDS-LAYOUT NEEDS-PAINT DETACHED\n'
        '   parentData: MISSING\n'
        '   constraints: MISSING\n'
        '   size: MISSING\n'
        '   alignment: Alignment.center\n'
        '   textDirection: ltr\n',
      ),
    );
  });

  test('UnconstrainedBox honors constrainedAxis=Axis.horizontal', () {
    final RenderConstrainedBox flexible = RenderConstrainedBox(
      additionalConstraints: const BoxConstraints.expand(height: 200.0),
    );
    final RenderConstraintsTransformBox unconstrained = RenderConstraintsTransformBox(
      constraintsTransform: ConstraintsTransformBox.heightUnconstrained,
      textDirection: TextDirection.ltr,
      child: RenderFlex(textDirection: TextDirection.ltr, children: <RenderBox>[flexible]),
      alignment: Alignment.center,
    );
    final FlexParentData flexParentData = flexible.parentData! as FlexParentData;
    flexParentData.flex = 1;
    flexParentData.fit = FlexFit.tight;

    const BoxConstraints viewport = BoxConstraints(maxWidth: 100.0);
    layout(unconstrained, constraints: viewport);

    expect(unconstrained.size.width, equals(100.0), reason: 'constrained width');
    expect(unconstrained.size.height, equals(200.0), reason: 'unconstrained height');
  });

  test('UnconstrainedBox honors constrainedAxis=Axis.vertical', () {
    final RenderConstrainedBox flexible = RenderConstrainedBox(
      additionalConstraints: const BoxConstraints.expand(width: 200.0),
    );
    final RenderConstraintsTransformBox unconstrained = RenderConstraintsTransformBox(
      constraintsTransform: ConstraintsTransformBox.widthUnconstrained,
      textDirection: TextDirection.ltr,
      child: RenderFlex(
        direction: Axis.vertical,
        textDirection: TextDirection.ltr,
        children: <RenderBox>[flexible],
      ),
      alignment: Alignment.center,
    );
    final FlexParentData flexParentData = flexible.parentData! as FlexParentData;
    flexParentData.flex = 1;
    flexParentData.fit = FlexFit.tight;

    const BoxConstraints viewport = BoxConstraints(maxHeight: 100.0);
    layout(unconstrained, constraints: viewport);

    expect(unconstrained.size.width, equals(200.0), reason: 'unconstrained width');
    expect(unconstrained.size.height, equals(100.0), reason: 'constrained height');
  });

  test('clipBehavior is respected', () {
    const BoxConstraints viewport = BoxConstraints(maxHeight: 100.0, maxWidth: 100.0);
    final TestClipPaintingContext context = TestClipPaintingContext();

    bool hadErrors = false;
    void expectOverflowedErrors() {
      absorbOverflowedErrors();
      hadErrors = true;
    }

    for (final Clip? clip in <Clip?>[null, ...Clip.values]) {
      final RenderConstraintsTransformBox box;
      switch (clip) {
        case Clip.none:
        case Clip.hardEdge:
        case Clip.antiAlias:
        case Clip.antiAliasWithSaveLayer:
          box = RenderConstraintsTransformBox(
            constraintsTransform: ConstraintsTransformBox.unconstrained,
            alignment: Alignment.center,
            textDirection: TextDirection.ltr,
            child: box200x200,
            clipBehavior: clip!,
          );
        case null:
          box = RenderConstraintsTransformBox(
            constraintsTransform: ConstraintsTransformBox.unconstrained,
            alignment: Alignment.center,
            textDirection: TextDirection.ltr,
            child: box200x200,
          );
      }
      layout(
        box,
        constraints: viewport,
        phase: EnginePhase.composite,
        onErrors: expectOverflowedErrors,
      );
      switch (clip) {
        case null:
        case Clip.none:
          expect(hadErrors, isTrue, reason: 'Should have had overflow errors for $clip');
        case Clip.hardEdge:
        case Clip.antiAlias:
        case Clip.antiAliasWithSaveLayer:
          expect(hadErrors, isFalse, reason: 'Should not have had overflow errors for $clip');
      }
      hadErrors = false;
      context.paintChild(box, Offset.zero);
      // By default, clipBehavior should be Clip.none
      expect(context.clipBehavior, equals(clip ?? Clip.none), reason: 'for $clip');
    }
  });

  group('hit testing', () {
    test('BoxHitTestResult wrapping HitTestResult', () {
      final HitTestEntry entry1 = HitTestEntry(_DummyHitTestTarget());
      final HitTestEntry entry2 = HitTestEntry(_DummyHitTestTarget());
      final HitTestEntry entry3 = HitTestEntry(_DummyHitTestTarget());
      final Matrix4 transform = Matrix4.translationValues(40.0, 150.0, 0.0);

      final HitTestResult wrapped = MyHitTestResult()..publicPushTransform(transform);
      wrapped.add(entry1);
      expect(wrapped.path, equals(<HitTestEntry>[entry1]));
      expect(entry1.transform, transform);

      final BoxHitTestResult wrapping = BoxHitTestResult.wrap(wrapped);
      expect(wrapping.path, equals(<HitTestEntry>[entry1]));
      expect(wrapping.path, same(wrapped.path));

      wrapping.add(entry2);
      expect(wrapping.path, equals(<HitTestEntry>[entry1, entry2]));
      expect(wrapped.path, equals(<HitTestEntry>[entry1, entry2]));
      expect(entry2.transform, transform);

      wrapped.add(entry3);
      expect(wrapping.path, equals(<HitTestEntry>[entry1, entry2, entry3]));
      expect(wrapped.path, equals(<HitTestEntry>[entry1, entry2, entry3]));
      expect(entry3.transform, transform);
    });

    test('addWithPaintTransform', () {
      final BoxHitTestResult result = BoxHitTestResult();
      final List<Offset> positions = <Offset>[];

      bool isHit = result.addWithPaintTransform(
        transform: null,
        position: Offset.zero,
        hitTest: (BoxHitTestResult result, Offset position) {
          expect(result, isNotNull);
          positions.add(position);
          return true;
        },
      );
      expect(isHit, isTrue);
      expect(positions.single, Offset.zero);
      positions.clear();

      isHit = result.addWithPaintTransform(
        transform: Matrix4.translationValues(20, 30, 0),
        position: Offset.zero,
        hitTest: (BoxHitTestResult result, Offset position) {
          expect(result, isNotNull);
          positions.add(position);
          return true;
        },
      );
      expect(isHit, isTrue);
      expect(positions.single, const Offset(-20.0, -30.0));
      positions.clear();

      const Offset position = Offset(3, 4);
      isHit = result.addWithPaintTransform(
        transform: null,
        position: position,
        hitTest: (BoxHitTestResult result, Offset position) {
          expect(result, isNotNull);
          positions.add(position);
          return false;
        },
      );
      expect(isHit, isFalse);
      expect(positions.single, position);
      positions.clear();

      isHit = result.addWithPaintTransform(
        transform: Matrix4.identity(),
        position: position,
        hitTest: (BoxHitTestResult result, Offset position) {
          expect(result, isNotNull);
          positions.add(position);
          return true;
        },
      );
      expect(isHit, isTrue);
      expect(positions.single, position);
      positions.clear();

      isHit = result.addWithPaintTransform(
        transform: Matrix4.translationValues(20, 30, 0),
        position: position,
        hitTest: (BoxHitTestResult result, Offset position) {
          expect(result, isNotNull);
          positions.add(position);
          return true;
        },
      );
      expect(isHit, isTrue);
      expect(positions.single, position - const Offset(20, 30));
      positions.clear();

      isHit = result.addWithPaintTransform(
        transform: MatrixUtils.forceToPoint(position), // cannot be inverted
        position: position,
        hitTest: (BoxHitTestResult result, Offset position) {
          expect(result, isNotNull);
          positions.add(position);
          return true;
        },
      );
      expect(isHit, isFalse);
      expect(positions, isEmpty);
      positions.clear();
    });

    test('addWithPaintOffset', () {
      final BoxHitTestResult result = BoxHitTestResult();
      final List<Offset> positions = <Offset>[];

      bool isHit = result.addWithPaintOffset(
        offset: null,
        position: Offset.zero,
        hitTest: (BoxHitTestResult result, Offset position) {
          expect(result, isNotNull);
          positions.add(position);
          return true;
        },
      );
      expect(isHit, isTrue);
      expect(positions.single, Offset.zero);
      positions.clear();

      isHit = result.addWithPaintOffset(
        offset: const Offset(55, 32),
        position: Offset.zero,
        hitTest: (BoxHitTestResult result, Offset position) {
          expect(result, isNotNull);
          positions.add(position);
          return true;
        },
      );
      expect(isHit, isTrue);
      expect(positions.single, const Offset(-55.0, -32.0));
      positions.clear();

      const Offset position = Offset(3, 4);
      isHit = result.addWithPaintOffset(
        offset: null,
        position: position,
        hitTest: (BoxHitTestResult result, Offset position) {
          expect(result, isNotNull);
          positions.add(position);
          return false;
        },
      );
      expect(isHit, isFalse);
      expect(positions.single, position);
      positions.clear();

      isHit = result.addWithPaintOffset(
        offset: Offset.zero,
        position: position,
        hitTest: (BoxHitTestResult result, Offset position) {
          expect(result, isNotNull);
          positions.add(position);
          return true;
        },
      );
      expect(isHit, isTrue);
      expect(positions.single, position);
      positions.clear();

      isHit = result.addWithPaintOffset(
        offset: const Offset(20, 30),
        position: position,
        hitTest: (BoxHitTestResult result, Offset position) {
          expect(result, isNotNull);
          positions.add(position);
          return true;
        },
      );
      expect(isHit, isTrue);
      expect(positions.single, position - const Offset(20, 30));
      positions.clear();
    });

    test('addWithRawTransform', () {
      final BoxHitTestResult result = BoxHitTestResult();
      final List<Offset> positions = <Offset>[];

      bool isHit = result.addWithRawTransform(
        transform: null,
        position: Offset.zero,
        hitTest: (BoxHitTestResult result, Offset position) {
          expect(result, isNotNull);
          positions.add(position);
          return true;
        },
      );
      expect(isHit, isTrue);
      expect(positions.single, Offset.zero);
      positions.clear();

      isHit = result.addWithRawTransform(
        transform: Matrix4.translationValues(20, 30, 0),
        position: Offset.zero,
        hitTest: (BoxHitTestResult result, Offset position) {
          expect(result, isNotNull);
          positions.add(position);
          return true;
        },
      );
      expect(isHit, isTrue);
      expect(positions.single, const Offset(20.0, 30.0));
      positions.clear();

      const Offset position = Offset(3, 4);
      isHit = result.addWithRawTransform(
        transform: null,
        position: position,
        hitTest: (BoxHitTestResult result, Offset position) {
          expect(result, isNotNull);
          positions.add(position);
          return false;
        },
      );
      expect(isHit, isFalse);
      expect(positions.single, position);
      positions.clear();

      isHit = result.addWithRawTransform(
        transform: Matrix4.identity(),
        position: position,
        hitTest: (BoxHitTestResult result, Offset position) {
          expect(result, isNotNull);
          positions.add(position);
          return true;
        },
      );
      expect(isHit, isTrue);
      expect(positions.single, position);
      positions.clear();

      isHit = result.addWithRawTransform(
        transform: Matrix4.translationValues(20, 30, 0),
        position: position,
        hitTest: (BoxHitTestResult result, Offset position) {
          expect(result, isNotNull);
          positions.add(position);
          return true;
        },
      );
      expect(isHit, isTrue);
      expect(positions.single, position + const Offset(20, 30));
      positions.clear();
    });

    test('addWithOutOfBandPosition', () {
      final BoxHitTestResult result = BoxHitTestResult();
      bool ran = false;

      bool isHit = result.addWithOutOfBandPosition(
        paintOffset: const Offset(20, 30),
        hitTest: (BoxHitTestResult result) {
          expect(result, isNotNull);
          ran = true;
          return true;
        },
      );
      expect(isHit, isTrue);
      expect(ran, isTrue);
      ran = false;

      isHit = result.addWithOutOfBandPosition(
        paintTransform: Matrix4.translationValues(20, 30, 0),
        hitTest: (BoxHitTestResult result) {
          expect(result, isNotNull);
          ran = true;
          return true;
        },
      );
      expect(isHit, isTrue);
      expect(ran, isTrue);
      ran = false;

      isHit = result.addWithOutOfBandPosition(
        rawTransform: Matrix4.translationValues(20, 30, 0),
        hitTest: (BoxHitTestResult result) {
          expect(result, isNotNull);
          ran = true;
          return true;
        },
      );
      expect(isHit, isTrue);
      expect(ran, isTrue);
      ran = false;

      isHit = result.addWithOutOfBandPosition(
        rawTransform: MatrixUtils.forceToPoint(Offset.zero), // cannot be inverted
        hitTest: (BoxHitTestResult result) {
          expect(result, isNotNull);
          ran = true;
          return true;
        },
      );
      expect(isHit, isTrue);
      expect(ran, isTrue);
      isHit = false;
      ran = false;

      expect(
        () {
          isHit = result.addWithOutOfBandPosition(
            paintTransform: MatrixUtils.forceToPoint(Offset.zero), // cannot be inverted
            hitTest: (BoxHitTestResult result) {
              fail('non-invertible transform should be caught');
            },
          );
        },
        throwsA(
          isAssertionError.having(
            (AssertionError error) => error.message,
            'message',
            'paintTransform must be invertible.',
          ),
        ),
      );
      expect(isHit, isFalse);

      expect(
        () {
          isHit = result.addWithOutOfBandPosition(
            hitTest: (BoxHitTestResult result) {
              fail('addWithOutOfBandPosition should need some transformation of some sort');
            },
          );
        },
        throwsA(
          isAssertionError.having(
            (AssertionError error) => error.message,
            'message',
            'Exactly one transform or offset argument must be provided.',
          ),
        ),
      );
      expect(isHit, isFalse);
    });

    test('error message', () {
      {
        final RenderBox renderObject = RenderConstrainedBox(
          additionalConstraints: const BoxConstraints().tighten(height: 100.0),
        );
        late FlutterError result;
        try {
          final BoxHitTestResult result = BoxHitTestResult();
          renderObject.hitTest(result, position: Offset.zero);
        } on FlutterError catch (e) {
          result = e;
        }
        expect(result, isNotNull);
        expect(
          result.toStringDeep(),
          equalsIgnoringHashCodes(
            'FlutterError\n'
            '   Cannot hit test a render box that has never been laid out.\n'
            '   The hitTest() method was called on this RenderBox: RenderConstrainedBox#00000 NEEDS-LAYOUT NEEDS-PAINT DETACHED:\n'
            '     parentData: MISSING\n'
            '     constraints: MISSING\n'
            '     size: MISSING\n'
            '     additionalConstraints: BoxConstraints(0.0<=w<=Infinity, h=100.0)\n'
            "   Unfortunately, this object's geometry is not known at this time,\n"
            '   probably because it has never been laid out. This means it cannot\n'
            '   be accurately hit-tested.\n'
            '   If you are trying to perform a hit test during the layout phase\n'
            '   itself, make sure you only hit test nodes that have completed\n'
            "   layout (e.g. the node's children, after their layout() method has\n"
            '   been called).\n',
          ),
        );
        expect(
          result.diagnostics
              .singleWhere((DiagnosticsNode node) => node.level == DiagnosticLevel.hint)
              .toString(),
          'If you are trying to perform a hit test during the layout phase '
          'itself, make sure you only hit test nodes that have completed '
          "layout (e.g. the node's children, after their layout() method has "
          'been called).',
        );
      }

      {
        late FlutterError result;
        final FakeMissingSizeRenderBox renderObject = FakeMissingSizeRenderBox();
        layout(renderObject);
        renderObject.fakeMissingSize = true;
        try {
          final BoxHitTestResult result = BoxHitTestResult();
          renderObject.hitTest(result, position: Offset.zero);
        } on FlutterError catch (e) {
          result = e;
        }
        expect(result, isNotNull);
        expect(
          result.toStringDeep(),
          equalsIgnoringHashCodes(
            'FlutterError\n'
            '   Cannot hit test a render box with no size.\n'
            '   The hitTest() method was called on this RenderBox: FakeMissingSizeRenderBox#00000 NEEDS-PAINT:\n'
            '     parentData: <none>\n'
            '     constraints: BoxConstraints(w=800.0, h=600.0)\n'
            '     size: Size(800.0, 600.0)\n'
            '   Although this node is not marked as needing layout, its size is\n'
            '   not set.\n'
            '   A RenderBox object must have an explicit size before it can be\n'
            '   hit-tested. Make sure that the RenderBox in question sets its\n'
            '   size during layout.\n',
          ),
        );
        expect(
          result.diagnostics
              .singleWhere((DiagnosticsNode node) => node.level == DiagnosticLevel.hint)
              .toString(),
          'A RenderBox object must have an explicit size before it can be '
          'hit-tested. Make sure that the RenderBox in question sets its '
          'size during layout.',
        );
      }
    });

    test('localToGlobal with ancestor', () {
      final RenderConstrainedBox innerConstrained = RenderConstrainedBox(
        additionalConstraints: const BoxConstraints.tightFor(width: 50, height: 50),
      );
      final RenderPositionedBox innerCenter = RenderPositionedBox(child: innerConstrained);
      final RenderConstrainedBox outerConstrained = RenderConstrainedBox(
        additionalConstraints: const BoxConstraints.tightFor(width: 100, height: 100),
        child: innerCenter,
      );
      final RenderPositionedBox outerCentered = RenderPositionedBox(child: outerConstrained);

      layout(outerCentered);

      expect(innerConstrained.localToGlobal(Offset.zero, ancestor: outerConstrained).dy, 25.0);
    });
  });

  test(
    'Error message when size has not been set in RenderBox performLayout should be well versed',
    () {
      late FlutterErrorDetails errorDetails;
      final FlutterExceptionHandler? oldHandler = FlutterError.onError;
      FlutterError.onError = (FlutterErrorDetails details) {
        errorDetails = details;
      };
      try {
        MissingSetSizeRenderBox().layout(const BoxConstraints());
      } finally {
        FlutterError.onError = oldHandler;
      }

      expect(errorDetails, isNotNull);

      // Check the ErrorDetails without the stack trace.
      final List<String> lines = errorDetails.toString().split('\n');
      expect(
        lines.take(5).join('\n'),
        equalsIgnoringHashCodes(
          '══╡ EXCEPTION CAUGHT BY RENDERING LIBRARY ╞══════════════════════\n'
          'The following assertion was thrown during performLayout():\n'
          'RenderBox did not set its size during layout.\n'
          'Because this RenderBox has sizedByParent set to false, it must\n'
          'set its size in performLayout().',
        ),
      );
    },
  );

  test('debugDoingBaseline flag is cleared after exception', () {
    final BadBaselineRenderBox badChild = BadBaselineRenderBox();
    final RenderBox badRoot = RenderBaseline(
      child: badChild,
      baseline: 0.0,
      baselineType: TextBaseline.alphabetic,
    );
    final List<dynamic> exceptions = <dynamic>[];
    layout(
      badRoot,
      onErrors: () {
        exceptions.addAll(TestRenderingFlutterBinding.instance.takeAllFlutterExceptions());
      },
    );
    expect(exceptions, isNotEmpty);

    final RenderBox goodRoot = RenderBaseline(
      child: RenderDecoratedBox(decoration: const BoxDecoration()),
      baseline: 0.0,
      baselineType: TextBaseline.alphabetic,
    );
    layout(
      goodRoot,
      onErrors: () {
        assert(false);
      },
    );
  });

  group('BaselineOffset', () {
    test('minOf', () {
      expect(BaselineOffset.noBaseline.minOf(BaselineOffset.noBaseline), BaselineOffset.noBaseline);

      expect(BaselineOffset.noBaseline.minOf(const BaselineOffset(1)), const BaselineOffset(1));
      expect(const BaselineOffset(1).minOf(BaselineOffset.noBaseline), const BaselineOffset(1));

      expect(const BaselineOffset(2).minOf(const BaselineOffset(1)), const BaselineOffset(1));
      expect(const BaselineOffset(1).minOf(const BaselineOffset(2)), const BaselineOffset(1));
    });

    test('+', () {
      expect(BaselineOffset.noBaseline + 2, BaselineOffset.noBaseline);
      expect(const BaselineOffset(1) + 2, const BaselineOffset(3));
    });
  });
}

class _DummyHitTestTarget implements HitTestTarget {
  @override
  void handleEvent(PointerEvent event, HitTestEntry entry) {
    // Nothing to do.
  }
}

class MyHitTestResult extends HitTestResult {
  void publicPushTransform(Matrix4 transform) => pushTransform(transform);
}
