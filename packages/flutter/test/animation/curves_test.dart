// Copyright 2014 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

import 'dart:math' as math;

import 'package:flutter/widgets.dart';
import 'package:flutter_test/flutter_test.dart';

void main() {
  test('toString control test', () {
    expect(Curves.linear, hasOneLineDescription);
    expect(const SawTooth(3), hasOneLineDescription);
    expect(const Interval(0.25, 0.75), hasOneLineDescription);
    expect(const Interval(0.25, 0.75, curve: Curves.ease), hasOneLineDescription);
    expect(const Split(0.25, beginCurve: Curves.ease), hasOneLineDescription);
  });

  test('Curve flipped control test', () {
    const Curve ease = Curves.ease;
    final Curve flippedEase = ease.flipped;
    expect(flippedEase.transform(0.0), lessThan(0.001));
    expect(flippedEase.transform(0.5), lessThan(ease.transform(0.5)));
    expect(flippedEase.transform(1.0), greaterThan(0.999));
    expect(flippedEase, hasOneLineDescription);
  });

  test('Threshold has a threshold', () {
    const Curve step = Threshold(0.25);
    expect(step.transform(0.0), 0.0);
    expect(step.transform(0.24), 0.0);
    expect(step.transform(0.25), 1.0);
    expect(step.transform(0.26), 1.0);
    expect(step.transform(1.0), 1.0);
  });

  void assertMaximumSlope(Curve curve, double maximumSlope) {
    const double delta = 0.005;
    for (double x = 0.0; x < 1.0 - delta; x += delta) {
      final double deltaY = curve.transform(x) - curve.transform(x + delta);
      assert(deltaY.abs() < delta * maximumSlope, '$curve discontinuous at $x');
    }
  }

  test('Curve is continuous', () {
    assertMaximumSlope(Curves.linear, 20.0);
    assertMaximumSlope(Curves.decelerate, 20.0);
    assertMaximumSlope(Curves.fastOutSlowIn, 20.0);
    assertMaximumSlope(Curves.slowMiddle, 20.0);
    assertMaximumSlope(Curves.bounceIn, 20.0);
    assertMaximumSlope(Curves.bounceOut, 20.0);
    assertMaximumSlope(Curves.bounceInOut, 20.0);
    assertMaximumSlope(Curves.elasticOut, 20.0);
    assertMaximumSlope(Curves.elasticInOut, 20.0);
    assertMaximumSlope(Curves.ease, 20.0);

    assertMaximumSlope(Curves.easeIn, 20.0);
    assertMaximumSlope(Curves.easeInSine, 20.0);
    assertMaximumSlope(Curves.easeInQuad, 20.0);
    assertMaximumSlope(Curves.easeInCubic, 20.0);
    assertMaximumSlope(Curves.easeInQuart, 20.0);
    assertMaximumSlope(Curves.easeInQuint, 20.0);
    assertMaximumSlope(Curves.easeInExpo, 20.0);
    assertMaximumSlope(Curves.easeInCirc, 20.0);

    assertMaximumSlope(Curves.easeOut, 20.0);
    assertMaximumSlope(Curves.easeOutSine, 20.0);
    assertMaximumSlope(Curves.easeOutQuad, 20.0);
    assertMaximumSlope(Curves.easeOutCubic, 20.0);
    assertMaximumSlope(Curves.easeInOutCubicEmphasized, 20.0);
    assertMaximumSlope(Curves.easeOutQuart, 20.0);
    assertMaximumSlope(Curves.easeOutQuint, 20.0);
    assertMaximumSlope(Curves.easeOutExpo, 20.0);
    assertMaximumSlope(Curves.easeOutCirc, 20.0);

    // Curves.easeInOutExpo is discontinuous at its midpoint, so not included
    // here

    assertMaximumSlope(Curves.easeInOut, 20.0);
    assertMaximumSlope(Curves.easeInOutSine, 20.0);
    assertMaximumSlope(Curves.easeInOutQuad, 20.0);
    assertMaximumSlope(Curves.easeInOutCubic, 20.0);
    assertMaximumSlope(Curves.easeInOutQuart, 20.0);
    assertMaximumSlope(Curves.easeInOutQuint, 20.0);
    assertMaximumSlope(Curves.easeInOutCirc, 20.0);
  });

  void expectStaysInBounds(Curve curve) {
    expect(curve.transform(0.0), inInclusiveRange(0.0, 1.0));
    expect(curve.transform(0.1), inInclusiveRange(0.0, 1.0));
    expect(curve.transform(0.2), inInclusiveRange(0.0, 1.0));
    expect(curve.transform(0.3), inInclusiveRange(0.0, 1.0));
    expect(curve.transform(0.4), inInclusiveRange(0.0, 1.0));
    expect(curve.transform(0.5), inInclusiveRange(0.0, 1.0));
    expect(curve.transform(0.6), inInclusiveRange(0.0, 1.0));
    expect(curve.transform(0.7), inInclusiveRange(0.0, 1.0));
    expect(curve.transform(0.8), inInclusiveRange(0.0, 1.0));
    expect(curve.transform(0.9), inInclusiveRange(0.0, 1.0));
    expect(curve.transform(1.0), inInclusiveRange(0.0, 1.0));
  }

  test('Bounce stays in bounds', () {
    expectStaysInBounds(Curves.bounceIn);
    expectStaysInBounds(Curves.bounceOut);
    expectStaysInBounds(Curves.bounceInOut);
  });

  List<double> estimateBounds(Curve curve) {
    final List<double> values = <double>[
      curve.transform(0.0),
      curve.transform(0.1),
      curve.transform(0.2),
      curve.transform(0.3),
      curve.transform(0.4),
      curve.transform(0.5),
      curve.transform(0.6),
      curve.transform(0.7),
      curve.transform(0.8),
      curve.transform(0.9),
      curve.transform(1.0),
    ];

    return <double>[values.reduce(math.min), values.reduce(math.max)];
  }

  test('Elastic overshoots its bounds', () {
    expect(Curves.elasticIn, hasOneLineDescription);
    expect(Curves.elasticOut, hasOneLineDescription);
    expect(Curves.elasticInOut, hasOneLineDescription);

    List<double> bounds;
    bounds = estimateBounds(Curves.elasticIn);
    expect(bounds[0], lessThan(0.0));
    expect(bounds[1], lessThanOrEqualTo(1.0));
    bounds = estimateBounds(Curves.elasticOut);
    expect(bounds[0], greaterThanOrEqualTo(0.0));
    expect(bounds[1], greaterThan(1.0));
    bounds = estimateBounds(Curves.elasticInOut);
    expect(bounds[0], lessThan(0.0));
    expect(bounds[1], greaterThan(1.0));
  });

  test('Back overshoots its bounds', () {
    expect(Curves.easeInBack, hasOneLineDescription);
    expect(Curves.easeOutBack, hasOneLineDescription);
    expect(Curves.easeInOutBack, hasOneLineDescription);

    List<double> bounds;
    bounds = estimateBounds(Curves.easeInBack);
    expect(bounds[0], lessThan(0.0));
    expect(bounds[1], lessThanOrEqualTo(1.0));
    bounds = estimateBounds(Curves.easeOutBack);
    expect(bounds[0], greaterThanOrEqualTo(0.0));
    expect(bounds[1], greaterThan(1.0));
    bounds = estimateBounds(Curves.easeInOutBack);
    expect(bounds[0], lessThan(0.0));
    expect(bounds[1], greaterThan(1.0));
  });

  test('Decelerate does so', () {
    expect(Curves.decelerate, hasOneLineDescription);

    final List<double> bounds = estimateBounds(Curves.decelerate);
    expect(bounds[0], greaterThanOrEqualTo(0.0));
    expect(bounds[1], lessThanOrEqualTo(1.0));

    final double d1 = Curves.decelerate.transform(0.2) - Curves.decelerate.transform(0.0);
    final double d2 = Curves.decelerate.transform(1.0) - Curves.decelerate.transform(0.8);
    expect(d2, lessThan(d1));
  });

  test('ThreePointCubic interpolates midpoint', () {
    const ThreePointCubic test = ThreePointCubic(
      Offset(0.05, 0),
      Offset(0.133333, 0.06),
      Offset(0.166666, 0.4),
      Offset(0.208333, 0.82),
      Offset(0.25, 1),
    );
    expect(test.transform(0.166666), equals(0.4));
  });

  test('Invalid transform parameter should assert', () {
    expect(() => const SawTooth(2).transform(-0.0001), throwsAssertionError);
    expect(() => const SawTooth(2).transform(1.0001), throwsAssertionError);

    expect(() => const Interval(0.0, 1.0).transform(-0.0001), throwsAssertionError);
    expect(() => const Interval(0.0, 1.0).transform(1.0001), throwsAssertionError);

    expect(() => const Split(0.0).transform(-0.0001), throwsAssertionError);
    expect(() => const Split(0.0).transform(1.0001), throwsAssertionError);

    expect(() => const Threshold(0.5).transform(-0.0001), throwsAssertionError);
    expect(() => const Threshold(0.5).transform(1.0001), throwsAssertionError);

    expect(() => const ElasticInCurve().transform(-0.0001), throwsAssertionError);
    expect(() => const ElasticInCurve().transform(1.0001), throwsAssertionError);

    expect(() => const ElasticOutCurve().transform(-0.0001), throwsAssertionError);
    expect(() => const ElasticOutCurve().transform(1.0001), throwsAssertionError);

    expect(() => const Cubic(0.42, 0.0, 0.58, 1.0).transform(-0.0001), throwsAssertionError);
    expect(() => const Cubic(0.42, 0.0, 0.58, 1.0).transform(1.0001), throwsAssertionError);

    expect(() => Curves.easeInOutCubicEmphasized.transform(-0.0001), throwsAssertionError);
    expect(() => Curves.easeInOutCubicEmphasized.transform(1.0001), throwsAssertionError);

    expect(() => Curves.decelerate.transform(-0.0001), throwsAssertionError);
    expect(() => Curves.decelerate.transform(1.0001), throwsAssertionError);

    expect(() => Curves.bounceIn.transform(-0.0001), throwsAssertionError);
    expect(() => Curves.bounceIn.transform(1.0001), throwsAssertionError);

    expect(() => Curves.bounceOut.transform(-0.0001), throwsAssertionError);
    expect(() => Curves.bounceOut.transform(1.0001), throwsAssertionError);

    expect(() => Curves.bounceInOut.transform(-0.0001), throwsAssertionError);
    expect(() => Curves.bounceInOut.transform(1.0001), throwsAssertionError);
  });

  test('Curve transform method should return 0.0 for t=0.0 and 1.0 for t=1.0', () {
    expect(const SawTooth(2).transform(0), 0);
    expect(const SawTooth(2).transform(1), 1);

    expect(const Interval(0, 1).transform(0), 0);
    expect(const Interval(0, 1).transform(1), 1);

    expect(const Split(0.5).transform(0), 0);
    expect(const Split(0.5).transform(1), 1);

    expect(const Threshold(0.5).transform(0), 0);
    expect(const Threshold(0.5).transform(1), 1);

    expect(const ElasticInCurve().transform(0), 0);
    expect(const ElasticInCurve().transform(1), 1);

    expect(const ElasticOutCurve().transform(0), 0);
    expect(const ElasticOutCurve().transform(1), 1);

    expect(const ElasticInOutCurve().transform(0), 0);
    expect(const ElasticInOutCurve().transform(1), 1);

    expect(Curves.linear.transform(0), 0);
    expect(Curves.linear.transform(1), 1);

    expect(Curves.easeInOutExpo.transform(0), 0);
    expect(Curves.easeInOutExpo.transform(1), 1);

    expect(Curves.easeInOutCubicEmphasized.transform(0), 0);
    expect(Curves.easeInOutCubicEmphasized.transform(1), 1);

    expect(const FlippedCurve(Curves.easeInOutExpo).transform(0), 0);
    expect(const FlippedCurve(Curves.easeInOutExpo).transform(1), 1);

    expect(Curves.decelerate.transform(0), 0);
    expect(Curves.decelerate.transform(1), 1);

    expect(Curves.bounceIn.transform(0), 0);
    expect(Curves.bounceIn.transform(1), 1);

    expect(Curves.bounceOut.transform(0), 0);
    expect(Curves.bounceOut.transform(1), 1);

    expect(Curves.bounceInOut.transform(0), 0);
    expect(Curves.bounceInOut.transform(1), 1);
  });

  test('Split interpolates values properly', () {
    const Split curve = Split(0.3);

    const double tolerance = 1e-6;
    expect(curve.transform(0.0), equals(0.0));
    expect(curve.transform(0.1), equals(0.1));
    expect(curve.transform(0.25), equals(0.25));
    expect(curve.transform(0.3), equals(0.3));
    expect(curve.transform(0.5), moreOrLessEquals(0.760461, epsilon: tolerance));
    expect(curve.transform(0.75), moreOrLessEquals(0.962055, epsilon: tolerance));
    expect(curve.transform(1.0), equals(1.0));
  });

  test('CatmullRomSpline interpolates values properly', () {
    final CatmullRomSpline curve = CatmullRomSpline(
      const <Offset>[
        Offset.zero,
        Offset(0.01, 0.25),
        Offset(0.2, 0.25),
        Offset(0.33, 0.25),
        Offset(0.5, 1.0),
        Offset(0.66, 0.75),
        Offset(1.0, 1.0),
      ],
      startHandle: const Offset(0.0, -0.3),
      endHandle: const Offset(1.3, 1.3),
    );
    const double tolerance = 1e-6;
    expect(curve.transform(0.0).dx, moreOrLessEquals(0.0, epsilon: tolerance));
    expect(curve.transform(0.0).dy, moreOrLessEquals(0.0, epsilon: tolerance));
    expect(curve.transform(0.25).dx, moreOrLessEquals(0.0966945, epsilon: tolerance));
    expect(curve.transform(0.25).dy, moreOrLessEquals(0.2626806, epsilon: tolerance));
    expect(curve.transform(0.5).dx, moreOrLessEquals(0.33, epsilon: tolerance));
    expect(curve.transform(0.5).dy, moreOrLessEquals(0.25, epsilon: tolerance));
    expect(curve.transform(0.75).dx, moreOrLessEquals(0.570260, epsilon: tolerance));
    expect(curve.transform(0.75).dy, moreOrLessEquals(0.883085, epsilon: tolerance));
    expect(curve.transform(1.0).dx, moreOrLessEquals(1.0, epsilon: tolerance));
    expect(curve.transform(1.0).dy, moreOrLessEquals(1.0, epsilon: tolerance));
  });

  test('CatmullRomSpline enforces contract', () {
    expect(() {
      CatmullRomSpline(const <Offset>[]);
    }, throwsAssertionError);
    expect(() {
      CatmullRomSpline(const <Offset>[Offset.zero]);
    }, throwsAssertionError);
    expect(() {
      CatmullRomSpline(const <Offset>[Offset.zero, Offset.zero]);
    }, throwsAssertionError);
    expect(() {
      CatmullRomSpline(const <Offset>[Offset.zero, Offset.zero, Offset.zero]);
    }, throwsAssertionError);
    expect(() {
      CatmullRomSpline(const <Offset>[
        Offset.zero,
        Offset.zero,
        Offset.zero,
        Offset.zero,
      ], tension: -1.0);
    }, throwsAssertionError);
    expect(() {
      CatmullRomSpline(const <Offset>[
        Offset.zero,
        Offset.zero,
        Offset.zero,
        Offset.zero,
      ], tension: 2.0);
    }, throwsAssertionError);
    expect(() {
      CatmullRomSpline(const <Offset>[
        Offset(double.infinity, 0.0),
        Offset.zero,
        Offset.zero,
        Offset.zero,
      ]).generateSamples();
    }, throwsAssertionError);
    expect(() {
      CatmullRomSpline(const <Offset>[
        Offset(0.0, double.infinity),
        Offset.zero,
        Offset.zero,
        Offset.zero,
      ]).generateSamples();
    }, throwsAssertionError);
    expect(() {
      CatmullRomSpline(startHandle: const Offset(0.0, double.infinity), const <Offset>[
        Offset.zero,
        Offset.zero,
        Offset.zero,
        Offset.zero,
      ]).generateSamples();
    }, throwsAssertionError);
    expect(() {
      CatmullRomSpline(endHandle: const Offset(0.0, double.infinity), const <Offset>[
        Offset.zero,
        Offset.zero,
        Offset.zero,
        Offset.zero,
      ]).generateSamples();
    }, throwsAssertionError);
  });

  test('CatmullRomSpline interpolates values properly when precomputed', () {
    final CatmullRomSpline curve = CatmullRomSpline.precompute(
      const <Offset>[
        Offset.zero,
        Offset(0.01, 0.25),
        Offset(0.2, 0.25),
        Offset(0.33, 0.25),
        Offset(0.5, 1.0),
        Offset(0.66, 0.75),
        Offset(1.0, 1.0),
      ],
      startHandle: const Offset(0.0, -0.3),
      endHandle: const Offset(1.3, 1.3),
    );
    const double tolerance = 1e-6;
    expect(curve.transform(0.0).dx, moreOrLessEquals(0.0, epsilon: tolerance));
    expect(curve.transform(0.0).dy, moreOrLessEquals(0.0, epsilon: tolerance));
    expect(curve.transform(0.25).dx, moreOrLessEquals(0.0966945, epsilon: tolerance));
    expect(curve.transform(0.25).dy, moreOrLessEquals(0.2626806, epsilon: tolerance));
    expect(curve.transform(0.5).dx, moreOrLessEquals(0.33, epsilon: tolerance));
    expect(curve.transform(0.5).dy, moreOrLessEquals(0.25, epsilon: tolerance));
    expect(curve.transform(0.75).dx, moreOrLessEquals(0.570260, epsilon: tolerance));
    expect(curve.transform(0.75).dy, moreOrLessEquals(0.883085, epsilon: tolerance));
    expect(curve.transform(1.0).dx, moreOrLessEquals(1.0, epsilon: tolerance));
    expect(curve.transform(1.0).dy, moreOrLessEquals(1.0, epsilon: tolerance));
  });

  test('CatmullRomSpline enforces contract when precomputed', () {
    expect(() {
      CatmullRomSpline.precompute(const <Offset>[]);
    }, throwsAssertionError);
    expect(() {
      CatmullRomSpline.precompute(const <Offset>[Offset.zero]);
    }, throwsAssertionError);
    expect(() {
      CatmullRomSpline.precompute(const <Offset>[Offset.zero, Offset.zero]);
    }, throwsAssertionError);
    expect(() {
      CatmullRomSpline.precompute(const <Offset>[Offset.zero, Offset.zero, Offset.zero]);
    }, throwsAssertionError);
    expect(() {
      CatmullRomSpline.precompute(const <Offset>[
        Offset.zero,
        Offset.zero,
        Offset.zero,
        Offset.zero,
      ], tension: -1.0);
    }, throwsAssertionError);
    expect(() {
      CatmullRomSpline.precompute(const <Offset>[
        Offset.zero,
        Offset.zero,
        Offset.zero,
        Offset.zero,
      ], tension: 2.0);
    }, throwsAssertionError);
    expect(() {
      CatmullRomSpline.precompute(const <Offset>[
        Offset(double.infinity, 0.0),
        Offset.zero,
        Offset.zero,
        Offset.zero,
      ]);
    }, throwsAssertionError);
    expect(() {
      CatmullRomSpline.precompute(const <Offset>[
        Offset(0.0, double.infinity),
        Offset.zero,
        Offset.zero,
        Offset.zero,
      ]);
    }, throwsAssertionError);
    expect(() {
      CatmullRomSpline.precompute(startHandle: const Offset(0.0, double.infinity), const <Offset>[
        Offset.zero,
        Offset.zero,
        Offset.zero,
        Offset.zero,
      ]);
    }, throwsAssertionError);
    expect(() {
      CatmullRomSpline.precompute(endHandle: const Offset(0.0, double.infinity), const <Offset>[
        Offset.zero,
        Offset.zero,
        Offset.zero,
        Offset.zero,
      ]);
    }, throwsAssertionError);
  });

  test('CatmullRomCurve interpolates given points correctly', () {
    final CatmullRomCurve curve = CatmullRomCurve(const <Offset>[
      Offset(0.2, 0.25),
      Offset(0.33, 0.25),
      Offset(0.5, 1.0),
      Offset(0.8, 0.75),
    ]);

    // These values are approximations.
    const double tolerance = 1e-6;
    expect(curve.transform(0.0), moreOrLessEquals(0.0, epsilon: tolerance));
    expect(curve.transform(0.01), moreOrLessEquals(0.012874734350170863, epsilon: tolerance));
    expect(curve.transform(0.2), moreOrLessEquals(0.24989646045277542, epsilon: tolerance));
    expect(curve.transform(0.33), moreOrLessEquals(0.250037698527661, epsilon: tolerance));
    expect(curve.transform(0.5), moreOrLessEquals(0.9999057323235939, epsilon: tolerance));
    expect(curve.transform(0.6), moreOrLessEquals(0.9357294964536621, epsilon: tolerance));
    expect(curve.transform(0.8), moreOrLessEquals(0.7500423402378034, epsilon: tolerance));
    expect(curve.transform(1.0), moreOrLessEquals(1.0, epsilon: tolerance));
  });

  test('CatmullRomCurve interpolates given points correctly when precomputed', () {
    final CatmullRomCurve curve = CatmullRomCurve.precompute(const <Offset>[
      Offset(0.2, 0.25),
      Offset(0.33, 0.25),
      Offset(0.5, 1.0),
      Offset(0.8, 0.75),
    ]);

    // These values are approximations.
    const double tolerance = 1e-6;
    expect(curve.transform(0.0), moreOrLessEquals(0.0, epsilon: tolerance));
    expect(curve.transform(0.01), moreOrLessEquals(0.012874734350170863, epsilon: tolerance));
    expect(curve.transform(0.2), moreOrLessEquals(0.24989646045277542, epsilon: tolerance));
    expect(curve.transform(0.33), moreOrLessEquals(0.250037698527661, epsilon: tolerance));
    expect(curve.transform(0.5), moreOrLessEquals(0.9999057323235939, epsilon: tolerance));
    expect(curve.transform(0.6), moreOrLessEquals(0.9357294964536621, epsilon: tolerance));
    expect(curve.transform(0.8), moreOrLessEquals(0.7500423402378034, epsilon: tolerance));
    expect(curve.transform(1.0), moreOrLessEquals(1.0, epsilon: tolerance));
  });

  test('CatmullRomCurve enforces contract', () {
    expect(() {
      CatmullRomCurve(const <Offset>[]);
    }, throwsAssertionError);
    expect(() {
      CatmullRomCurve(const <Offset>[Offset.zero]);
    }, throwsAssertionError);
    expect(() {
      CatmullRomCurve(const <Offset>[Offset.zero, Offset.zero]);
    }, throwsAssertionError);

    // Monotonically increasing in X.
    expect(
      CatmullRomCurve.validateControlPoints(const <Offset>[Offset(0.2, 0.25), Offset(0.01, 0.25)]),
      isFalse,
    );
    expect(() {
      CatmullRomCurve(const <Offset>[Offset(0.2, 0.25), Offset(0.01, 0.25)]);
    }, throwsAssertionError);

    // X within range (0.0, 1.0).
    expect(
      CatmullRomCurve.validateControlPoints(const <Offset>[Offset(0.2, 0.25), Offset(1.01, 0.25)]),
      isFalse,
    );
    expect(() {
      CatmullRomCurve(const <Offset>[Offset(0.2, 0.25), Offset(1.01, 0.25)]);
    }, throwsAssertionError);

    // Not multi-valued in Y at x=0.0.
    expect(
      CatmullRomCurve.validateControlPoints(const <Offset>[
        Offset(0.05, 0.50),
        Offset(0.50, 0.50),
        Offset(0.75, 0.75),
      ]),
      isFalse,
    );
    expect(() {
      CatmullRomCurve(const <Offset>[Offset(0.05, 0.50), Offset(0.50, 0.50), Offset(0.75, 0.75)]);
    }, throwsAssertionError);

    // Not multi-valued in Y at x=1.0.
    expect(
      CatmullRomCurve.validateControlPoints(const <Offset>[
        Offset(0.25, 0.25),
        Offset(0.50, 0.50),
        Offset(0.95, 0.51),
      ]),
      isFalse,
    );
    expect(() {
      CatmullRomCurve(const <Offset>[Offset(0.25, 0.25), Offset(0.50, 0.50), Offset(0.95, 0.51)]);
    }, throwsAssertionError);

    // Not multi-valued in Y in between x = 0.0 and x = 1.0.
    expect(
      CatmullRomCurve.validateControlPoints(const <Offset>[Offset(0.5, 0.05), Offset(0.5, 0.95)]),
      isFalse,
    );
    expect(() {
      CatmullRomCurve(const <Offset>[Offset(0.5, 0.05), Offset(0.5, 0.95)]);
    }, throwsAssertionError);
  });

  test('CatmullRomCurve enforces contract when precomputed', () {
    expect(() {
      CatmullRomCurve.precompute(const <Offset>[]);
    }, throwsAssertionError);
    expect(() {
      CatmullRomCurve.precompute(const <Offset>[Offset.zero]);
    }, throwsAssertionError);
    expect(() {
      CatmullRomCurve.precompute(const <Offset>[Offset.zero, Offset.zero]);
    }, throwsAssertionError);

    // Monotonically increasing in X.
    expect(() {
      CatmullRomCurve.precompute(const <Offset>[Offset(0.2, 0.25), Offset(0.01, 0.25)]);
    }, throwsAssertionError);

    // X within range (0.0, 1.0).
    expect(() {
      CatmullRomCurve.precompute(const <Offset>[Offset(0.2, 0.25), Offset(1.01, 0.25)]);
    }, throwsAssertionError);

    // Not multi-valued in Y at x=0.0.
    expect(() {
      CatmullRomCurve.precompute(const <Offset>[
        Offset(0.05, 0.50),
        Offset(0.50, 0.50),
        Offset(0.75, 0.75),
      ]);
    }, throwsAssertionError);

    // Not multi-valued in Y at x=1.0.
    expect(() {
      CatmullRomCurve.precompute(const <Offset>[
        Offset(0.25, 0.25),
        Offset(0.50, 0.50),
        Offset(0.95, 0.51),
      ]);
    }, throwsAssertionError);

    // Not multi-valued in Y in between x = 0.0 and x = 1.0.
    expect(() {
      CatmullRomCurve.precompute(const <Offset>[Offset(0.5, 0.05), Offset(0.5, 0.95)]);
    }, throwsAssertionError);
  });
}
