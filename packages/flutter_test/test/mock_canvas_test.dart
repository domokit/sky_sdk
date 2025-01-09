// Copyright 2014 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

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
}
