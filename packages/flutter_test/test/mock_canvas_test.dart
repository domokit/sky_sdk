// Copyright 2014 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

import 'dart:math' as math;

import 'package:flutter/material.dart';
import 'package:flutter_test/flutter_test.dart';

class MyPainter extends CustomPainter {
  const MyPainter({required this.color});

  final Color color;

  @override
  void paint(Canvas canvas, Size size) {
    canvas.drawColor(color, BlendMode.color);
  }

  @override
  bool shouldRepaint(MyPainter oldDelegate) {
    return true;
  }
}

@immutable
class MethodAndArguments {
  const MethodAndArguments(this.method, this.arguments);

  final Symbol method;
  final List<dynamic> arguments;

  @override
  bool operator ==(Object other) {
    if (!(other is MethodAndArguments && other.method == method)) {
      return false;
    }
    for (int i = 0; i < arguments.length; i++) {
      if (arguments[i] != other.arguments[i]) {
        return false;
      }
    }
    return true;
  }

  @override
  int get hashCode => method.hashCode;

  @override
  String toString() => '$method, $arguments';
}

void main() {
  group('something', () {
    testWidgets('matches when the predicate returns true', (WidgetTester tester) async {
      await tester.pumpWidget(
        const CustomPaint(
          painter: MyPainter(color: Colors.transparent),
          child: SizedBox(width: 50, height: 50),
        ),
      );

      final List<MethodAndArguments> methodsAndArguments = <MethodAndArguments>[];

      expect(
        tester.renderObject(find.byType(CustomPaint)),
        paints..something((Symbol method, List<dynamic> arguments) {
          methodsAndArguments.add(MethodAndArguments(method, arguments));
          return method == #drawColor;
        }),
      );

      expect(methodsAndArguments, <MethodAndArguments>[
        const MethodAndArguments(#save, <dynamic>[]),
        const MethodAndArguments(#drawColor, <dynamic>[Colors.transparent, BlendMode.color]),
        // The #restore call is never evaluated
      ]);
    });

    testWidgets('fails when the predicate always returns false', (WidgetTester tester) async {
      await tester.pumpWidget(
        const CustomPaint(
          painter: MyPainter(color: Colors.transparent),
          child: SizedBox(width: 50, height: 50),
        ),
      );

      final List<MethodAndArguments> methodsAndArguments = <MethodAndArguments>[];

      expect(
        tester.renderObject(find.byType(CustomPaint)),
        isNot(
          paints..something((Symbol method, List<dynamic> arguments) {
            methodsAndArguments.add(MethodAndArguments(method, arguments));
            return false;
          }),
        ),
      );

      expect(methodsAndArguments, <MethodAndArguments>[
        const MethodAndArguments(#save, <dynamic>[]),
        const MethodAndArguments(#drawColor, <dynamic>[Colors.transparent, BlendMode.color]),
        const MethodAndArguments(#restore, <dynamic>[]),
      ]);
    });

    testWidgets('fails when the predicate throws', (WidgetTester tester) async {
      await tester.pumpWidget(
        const CustomPaint(
          painter: MyPainter(color: Colors.transparent),
          child: SizedBox(width: 50, height: 50),
        ),
      );

      final List<MethodAndArguments> methodsAndArguments = <MethodAndArguments>[];

      expect(
        tester.renderObject(find.byType(CustomPaint)),
        isNot(
          paints..something((Symbol method, List<dynamic> arguments) {
            methodsAndArguments.add(MethodAndArguments(method, arguments));
            if (method == #save) {
              return false;
            }
            if (method == #drawColor) {
              fail('fail');
            }
            return true;
          }),
        ),
      );

      expect(methodsAndArguments, <MethodAndArguments>[
        const MethodAndArguments(#save, <dynamic>[]),
        const MethodAndArguments(#drawColor, <dynamic>[Colors.transparent, BlendMode.color]),
        // The #restore call is never evaluated
      ]);
    });
  });

  group('everything', () {
    testWidgets('matches when the predicate always returns true', (WidgetTester tester) async {
      await tester.pumpWidget(
        const CustomPaint(
          painter: MyPainter(color: Colors.transparent),
          child: SizedBox(width: 50, height: 50),
        ),
      );

      final List<MethodAndArguments> methodsAndArguments = <MethodAndArguments>[];

      expect(
        tester.renderObject(find.byType(CustomPaint)),
        paints..everything((Symbol method, List<dynamic> arguments) {
          methodsAndArguments.add(MethodAndArguments(method, arguments));
          return true;
        }),
      );

      expect(methodsAndArguments, <MethodAndArguments>[
        const MethodAndArguments(#save, <dynamic>[]),
        const MethodAndArguments(#drawColor, <dynamic>[Colors.transparent, BlendMode.color]),
        const MethodAndArguments(#restore, <dynamic>[]),
      ]);
    });

    testWidgets('fails when the predicate returns false', (WidgetTester tester) async {
      await tester.pumpWidget(
        const CustomPaint(
          painter: MyPainter(color: Colors.transparent),
          child: SizedBox(width: 50, height: 50),
        ),
      );

      final List<MethodAndArguments> methodsAndArguments = <MethodAndArguments>[];

      expect(
        tester.renderObject(find.byType(CustomPaint)),
        isNot(
          paints..everything((Symbol method, List<dynamic> arguments) {
            methodsAndArguments.add(MethodAndArguments(method, arguments));
            // returns false on #drawColor
            return method == #restore || method == #save;
          }),
        ),
      );

      expect(methodsAndArguments, <MethodAndArguments>[
        const MethodAndArguments(#save, <dynamic>[]),
        const MethodAndArguments(#drawColor, <dynamic>[Colors.transparent, BlendMode.color]),
        // The #restore call is never evaluated
      ]);
    });

    testWidgets('fails if the predicate ever throws', (WidgetTester tester) async {
      await tester.pumpWidget(
        const CustomPaint(
          painter: MyPainter(color: Colors.transparent),
          child: SizedBox(width: 50, height: 50),
        ),
      );

      final List<MethodAndArguments> methodsAndArguments = <MethodAndArguments>[];

      expect(
        tester.renderObject(find.byType(CustomPaint)),
        isNot(
          paints..everything((Symbol method, List<dynamic> arguments) {
            methodsAndArguments.add(MethodAndArguments(method, arguments));
            if (method == #drawColor) {
              fail('failed ');
            }
            return true;
          }),
        ),
      );

      expect(methodsAndArguments, <MethodAndArguments>[
        const MethodAndArguments(#save, <dynamic>[]),
        const MethodAndArguments(#drawColor, <dynamic>[Colors.transparent, BlendMode.color]),
        // The #restore call is never evaluated
      ]);
    });
  });

  group('arc', () {
    final Rect rect = Offset.zero & const Size.square(50);
    const double startAngle = math.pi / 4;
    const double sweepAngle = math.pi / 2;
    const bool useCenter = false;
    final Paint paint = Paint()..color = Colors.blue;

    Future<void> pumpPainter(WidgetTester tester) async {
      await tester.pumpWidget(
        Center(
          child: CustomPaint(
            painter: _ArcPainter(
              startAngle: startAngle,
              sweepAngle: sweepAngle,
              useCenter: useCenter,
              paint: paint,
            ),
            size: rect.size,
          ),
        ),
      );
    }

    testWidgets('matches when rect is correct', (WidgetTester tester) async {
      await pumpPainter(tester);
      expect(tester.renderObject(find.byType(CustomPaint)), paints..arc(rect: rect));
    });

    testWidgets('does not match when rect is incorrect', (WidgetTester tester) async {
      await pumpPainter(tester);

      expect(
        () => expect(
          tester.renderObject(find.byType(CustomPaint)),
          paints..arc(rect: rect.deflate(10)),
        ),
        throwsA(
          isA<TestFailure>().having(
            (TestFailure failure) => failure.message,
            'message',
            contains(
              'It called drawArc with a paint whose rect, '
              'Rect.fromLTRB(0.0, 0.0, 50.0, 50.0), was not exactly the '
              'expected rect (Rect.fromLTRB(10.0, 10.0, 40.0, 40.0)).',
            ),
          ),
        ),
      );
    });

    testWidgets('matches when startAngle is correct', (WidgetTester tester) async {
      await pumpPainter(tester);
      expect(tester.renderObject(find.byType(CustomPaint)), paints..arc(startAngle: startAngle));
    });

    testWidgets('does not match when startAngle is incorrect', (WidgetTester tester) async {
      await pumpPainter(tester);

      expect(
        () => expect(
          tester.renderObject(find.byType(CustomPaint)),
          paints..arc(startAngle: startAngle * 2),
        ),
        throwsA(
          isA<TestFailure>().having(
            (TestFailure failure) => failure.message,
            'message',
            contains(
              'It called drawArc with a start angle, 0.7853981633974483, which '
              'was not exactly the expected start angle (1.5707963267948966).',
            ),
          ),
        ),
      );
    });

    testWidgets('matches when sweepAngle is correct', (WidgetTester tester) async {
      await pumpPainter(tester);
      expect(tester.renderObject(find.byType(CustomPaint)), paints..arc(sweepAngle: sweepAngle));
    });

    testWidgets('does not match when sweepAngle is incorrect', (WidgetTester tester) async {
      await pumpPainter(tester);

      expect(
        () => expect(
          tester.renderObject(find.byType(CustomPaint)),
          paints..arc(sweepAngle: sweepAngle * 2),
        ),
        throwsA(
          isA<TestFailure>().having(
            (TestFailure failure) => failure.message,
            'message',
            contains(
              'It called drawArc with a sweep angle, 1.5707963267948966, which '
              'was not exactly the expected sweep angle (3.141592653589793).',
            ),
          ),
        ),
      );
    });

    testWidgets('matches when useCenter is correct', (WidgetTester tester) async {
      await pumpPainter(tester);
      expect(tester.renderObject(find.byType(CustomPaint)), paints..arc(useCenter: useCenter));
    });

    testWidgets('does not match when useCenter is incorrect', (WidgetTester tester) async {
      await pumpPainter(tester);

      expect(
        () => expect(
          tester.renderObject(find.byType(CustomPaint)),
          paints..arc(useCenter: !useCenter),
        ),
        throwsA(
          isA<TestFailure>().having(
            (TestFailure failure) => failure.message,
            'message',
            contains(
              'It called drawArc with a useCenter value, false, which was not '
              'exactly the expected value (true)',
            ),
          ),
        ),
      );
    });
  });
}

class _ArcPainter extends CustomPainter {
  const _ArcPainter({
    required this.startAngle,
    required this.sweepAngle,
    required this.useCenter,
    required Paint paint,
  }) : _paint = paint;

  final double startAngle;

  final double sweepAngle;

  final bool useCenter;

  final Paint _paint;

  @override
  void paint(Canvas canvas, Size size) {
    canvas.drawArc(Offset.zero & size, startAngle, sweepAngle, useCenter, _paint);
  }

  @override
  bool shouldRepaint(MyPainter oldDelegate) {
    return true;
  }
}
