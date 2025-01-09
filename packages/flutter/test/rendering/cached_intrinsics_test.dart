// Copyright 2014 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

import 'package:flutter/rendering.dart';
import 'package:flutter_test/flutter_test.dart';

import 'rendering_tester.dart';

class RenderTestBox extends RenderBox {
  late Size boxSize;
  int calls = 0;
  double value = 0.0;
  double next() {
    value += 1.0;
    return value;
  }

  @override
  double computeMinIntrinsicWidth(double height) => next();
  @override
  double computeMaxIntrinsicWidth(double height) => next();
  @override
  double computeMinIntrinsicHeight(double width) => next();
  @override
  double computeMaxIntrinsicHeight(double width) => next();

  @override
  void performLayout() {
    size = constraints.biggest;
    boxSize = size;
  }

  @override
  double? computeDistanceToActualBaseline(TextBaseline baseline) {
    if (!RenderObject.debugCheckingIntrinsics) {
      calls += 1;
    }
    return boxSize.height / 2.0;
  }
}

class RenderDryBaselineTestBox extends RenderTestBox {
  double? baselineOverride;

  @override
  double? computeDryBaseline(covariant BoxConstraints constraints, TextBaseline baseline) {
    if (!RenderObject.debugCheckingIntrinsics) {
      calls += 1;
    }
    return baselineOverride ?? constraints.biggest.height / 2.0;
  }
}

class RenderBadDryBaselineTestBox extends RenderTestBox {
  @override
  double? computeDryBaseline(covariant BoxConstraints constraints, TextBaseline baseline) {
    return size.height / 2.0;
  }
}

class RenderCannotComputeDryBaselineTestBox extends RenderTestBox {
  bool shouldAssert = true;
  @override
  double? computeDryBaseline(covariant BoxConstraints constraints, TextBaseline baseline) {
    if (shouldAssert) {
      assert(debugCannotComputeDryLayout(reason: 'no dry baseline for you'));
    }
    return null;
  }
}

void main() {
  TestRenderingFlutterBinding.ensureInitialized();

  test('Intrinsics cache', () {
    final RenderBox test = RenderTestBox();

    expect(test.getMinIntrinsicWidth(0.0), equals(1.0));
    expect(test.getMinIntrinsicWidth(100.0), equals(2.0));
    expect(test.getMinIntrinsicWidth(200.0), equals(3.0));
    expect(test.getMinIntrinsicWidth(0.0), equals(1.0));
    expect(test.getMinIntrinsicWidth(100.0), equals(2.0));
    expect(test.getMinIntrinsicWidth(200.0), equals(3.0));

    expect(test.getMaxIntrinsicWidth(0.0), equals(4.0));
    expect(test.getMaxIntrinsicWidth(100.0), equals(5.0));
    expect(test.getMaxIntrinsicWidth(200.0), equals(6.0));
    expect(test.getMaxIntrinsicWidth(0.0), equals(4.0));
    expect(test.getMaxIntrinsicWidth(100.0), equals(5.0));
    expect(test.getMaxIntrinsicWidth(200.0), equals(6.0));

    expect(test.getMinIntrinsicHeight(0.0), equals(7.0));
    expect(test.getMinIntrinsicHeight(100.0), equals(8.0));
    expect(test.getMinIntrinsicHeight(200.0), equals(9.0));
    expect(test.getMinIntrinsicHeight(0.0), equals(7.0));
    expect(test.getMinIntrinsicHeight(100.0), equals(8.0));
    expect(test.getMinIntrinsicHeight(200.0), equals(9.0));

    expect(test.getMaxIntrinsicHeight(0.0), equals(10.0));
    expect(test.getMaxIntrinsicHeight(100.0), equals(11.0));
    expect(test.getMaxIntrinsicHeight(200.0), equals(12.0));
    expect(test.getMaxIntrinsicHeight(0.0), equals(10.0));
    expect(test.getMaxIntrinsicHeight(100.0), equals(11.0));
    expect(test.getMaxIntrinsicHeight(200.0), equals(12.0));

    // now read them all again backwards
    expect(test.getMaxIntrinsicHeight(200.0), equals(12.0));
    expect(test.getMaxIntrinsicHeight(100.0), equals(11.0));
    expect(test.getMaxIntrinsicHeight(0.0), equals(10.0));
    expect(test.getMinIntrinsicHeight(200.0), equals(9.0));
    expect(test.getMinIntrinsicHeight(100.0), equals(8.0));
    expect(test.getMinIntrinsicHeight(0.0), equals(7.0));
    expect(test.getMaxIntrinsicWidth(200.0), equals(6.0));
    expect(test.getMaxIntrinsicWidth(100.0), equals(5.0));
    expect(test.getMaxIntrinsicWidth(0.0), equals(4.0));
    expect(test.getMinIntrinsicWidth(200.0), equals(3.0));
    expect(test.getMinIntrinsicWidth(100.0), equals(2.0));
    expect(test.getMinIntrinsicWidth(0.0), equals(1.0));
  });

  // Regression test for https://github.com/flutter/flutter/issues/101179
  test('Cached baselines should be cleared if its parent re-layout', () {
    double viewHeight = 200.0;
    final RenderTestBox test = RenderTestBox();
    final RenderBox baseline = RenderBaseline(
      baseline: 0.0,
      baselineType: TextBaseline.alphabetic,
      child: test,
    );
    final RenderConstrainedBox root = RenderConstrainedBox(
      additionalConstraints: BoxConstraints.tightFor(width: 200.0, height: viewHeight),
      child: baseline,
    );

    layout(RenderPositionedBox(child: root));

    BoxParentData? parentData = test.parentData as BoxParentData?;
    expect(parentData!.offset.dy, -(viewHeight / 2.0));
    expect(test.calls, 1);

    // Trigger the root render re-layout.
    viewHeight = 300.0;
    root.additionalConstraints = BoxConstraints.tightFor(width: 200.0, height: viewHeight);
    pumpFrame();

    parentData = test.parentData as BoxParentData?;
    expect(parentData!.offset.dy, -(viewHeight / 2.0));
    expect(test.calls, 2); // The layout constraints change will clear the cached data.

    final RenderObject parent = test.parent!;
    expect(parent.debugNeedsLayout, false);

    // Do not forget notify parent dirty after the cached data be cleared by `layout()`
    test.markNeedsLayout();
    expect(parent.debugNeedsLayout, true);

    pumpFrame();
    expect(parent.debugNeedsLayout, false);
    expect(test.calls, 3); // Self dirty will clear the cached data.

    parent.markNeedsLayout();
    pumpFrame();

    expect(test.calls, 3); // Use the cached data if the layout constraints do not change.
  });

  group('Dry baseline', () {
    test(
      'computeDryBaseline results are cached and shared with computeDistanceToActualBaseline',
      () {
        const double viewHeight = 200.0;
        const BoxConstraints constraints = BoxConstraints.tightFor(
          width: 200.0,
          height: viewHeight,
        );
        final RenderDryBaselineTestBox test = RenderDryBaselineTestBox();
        final RenderBox baseline = RenderBaseline(
          baseline: 0.0,
          baselineType: TextBaseline.alphabetic,
          child: test,
        );

        final RenderConstrainedBox root = RenderConstrainedBox(
          additionalConstraints: constraints,
          child: baseline,
        );

        layout(RenderPositionedBox(child: root));
        expect(test.calls, 1);

        // The baseline widget loosens the input constraints when passing on to child.
        expect(
          test.getDryBaseline(constraints.loosen(), TextBaseline.alphabetic),
          test.boxSize.height / 2,
        );
        // There's cache for the constraints so this should be 1, but we always evaluate
        // computeDryBaseline in debug mode in case it asserts even if the baseline
        // cache hits.
        expect(test.calls, 2);

        const BoxConstraints newConstraints = BoxConstraints.tightFor(width: 10.0, height: 10.0);
        expect(test.getDryBaseline(newConstraints.loosen(), TextBaseline.alphabetic), 5.0);
        // Should be 3 but there's an additional computeDryBaseline call in getDryBaseline,
        // in an assert.
        expect(test.calls, 4);

        root.additionalConstraints = newConstraints;
        pumpFrame();
        expect(test.calls, 4);
      },
    );

    test('Asserts when a RenderBox cannot compute dry baseline', () {
      final RenderCannotComputeDryBaselineTestBox test = RenderCannotComputeDryBaselineTestBox();
      layout(RenderBaseline(baseline: 0.0, baselineType: TextBaseline.alphabetic, child: test));

      final BoxConstraints incomingConstraints = test.constraints;
      assert(incomingConstraints != const BoxConstraints());
      expect(
        () => test.getDryBaseline(const BoxConstraints(), TextBaseline.alphabetic),
        throwsA(
          isA<AssertionError>().having(
            (AssertionError e) => e.message,
            'message',
            contains('no dry baseline for you'),
          ),
        ),
      );

      // Still throws when there is cache.
      expect(
        () => test.getDryBaseline(incomingConstraints, TextBaseline.alphabetic),
        throwsA(
          isA<AssertionError>().having(
            (AssertionError e) => e.message,
            'message',
            contains('no dry baseline for you'),
          ),
        ),
      );
    });

    test(
      'Catches inconsistencies between computeDryBaseline and computeDistanceToActualBaseline',
      () {
        final RenderDryBaselineTestBox test = RenderDryBaselineTestBox();
        layout(test, phase: EnginePhase.composite);

        FlutterErrorDetails? error;
        test.markNeedsLayout();
        test.baselineOverride = 123;
        pumpFrame(
          phase: EnginePhase.composite,
          onErrors: () {
            error = TestRenderingFlutterBinding.instance.takeFlutterErrorDetails();
          },
        );

        expect(
          error?.exceptionAsString(),
          contains('differs from the baseline location computed by computeDryBaseline'),
        );
      },
    );

    test('Accessing RenderBox.size in computeDryBaseline is not allowed', () {
      final RenderBadDryBaselineTestBox test = RenderBadDryBaselineTestBox();
      FlutterErrorDetails? error;
      layout(
        test,
        phase: EnginePhase.composite,
        onErrors: () {
          error = TestRenderingFlutterBinding.instance.takeFlutterErrorDetails();
        },
      );

      expect(
        error?.exceptionAsString(),
        contains('RenderBox.size accessed in RenderBadDryBaselineTestBox.computeDryBaseline.'),
      );
    });

    test('debug baseline checks do not freak out when debugCannotComputeDryLayout is called', () {
      FlutterErrorDetails? error;
      void onErrors() {
        error = TestRenderingFlutterBinding.instance.takeFlutterErrorDetails();
      }

      final RenderCannotComputeDryBaselineTestBox test = RenderCannotComputeDryBaselineTestBox();
      layout(test, phase: EnginePhase.composite, onErrors: onErrors);
      expect(error, isNull);

      test.shouldAssert = false;
      test.markNeedsLayout();
      pumpFrame(phase: EnginePhase.composite, onErrors: onErrors);
      expect(
        error?.exceptionAsString(),
        contains('differs from the baseline location computed by computeDryBaseline'),
      );
    });
  });
}
