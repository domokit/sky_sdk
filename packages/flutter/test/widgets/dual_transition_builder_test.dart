// Copyright 2014 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

import 'package:flutter/material.dart';
import 'package:flutter_test/flutter_test.dart';

void main() {
  testWidgets('runs animations', (WidgetTester tester) async {
    final AnimationController controller = AnimationController(
      vsync: const TestVSync(),
      duration: const Duration(milliseconds: 300),
    );
    addTearDown(controller.dispose);

    await tester.pumpWidget(
      Center(
        child: DualTransitionBuilder(
          animation: controller,
          forwardBuilder: (BuildContext context, Animation<double> animation, Widget? child) {
            return ScaleTransition(scale: animation, child: child);
          },
          reverseBuilder: (BuildContext context, Animation<double> animation, Widget? child) {
            return FadeTransition(
              opacity: Tween<double>(begin: 1.0, end: 0.0).animate(animation),
              child: child,
            );
          },
          child: Container(color: Colors.green, height: 100, width: 100),
        ),
      ),
    );
    expect(_getScale(tester), 0.0);
    expect(_getOpacity(tester), 1.0);

    controller.forward();
    await tester.pump();
    await tester.pump(const Duration(milliseconds: 150));
    expect(_getScale(tester), 0.5);
    expect(_getOpacity(tester), 1.0);

    await tester.pump(const Duration(milliseconds: 150));
    expect(_getScale(tester), 1.0);
    expect(_getOpacity(tester), 1.0);

    await tester.pumpAndSettle();
    expect(_getScale(tester), 1.0);
    expect(_getOpacity(tester), 1.0);

    controller.reverse();
    await tester.pump();
    await tester.pump(const Duration(milliseconds: 150));
    expect(_getScale(tester), 1.0);
    expect(_getOpacity(tester), 0.5);

    await tester.pump(const Duration(milliseconds: 150));
    expect(_getScale(tester), 1.0);
    expect(_getOpacity(tester), 0.0);

    await tester.pumpAndSettle();
    expect(_getScale(tester), 0.0);
    expect(_getOpacity(tester), 1.0);
  });

  testWidgets('keeps state', (WidgetTester tester) async {
    final AnimationController controller = AnimationController(
      vsync: const TestVSync(),
      duration: const Duration(milliseconds: 300),
    );
    addTearDown(controller.dispose);

    await tester.pumpWidget(
      Directionality(
        textDirection: TextDirection.ltr,
        child: Center(
          child: DualTransitionBuilder(
            animation: controller,
            forwardBuilder: (BuildContext context, Animation<double> animation, Widget? child) {
              return ScaleTransition(scale: animation, child: child);
            },
            reverseBuilder: (BuildContext context, Animation<double> animation, Widget? child) {
              return FadeTransition(
                opacity: Tween<double>(begin: 1.0, end: 0.0).animate(animation),
                child: child,
              );
            },
            child: const _StatefulTestWidget(name: 'Foo'),
          ),
        ),
      ),
    );
    final State<StatefulWidget> state = tester.state(find.byType(_StatefulTestWidget));
    expect(state, isNotNull);

    controller.forward();
    await tester.pump();
    expect(state, same(tester.state(find.byType(_StatefulTestWidget))));
    await tester.pump(const Duration(milliseconds: 150));
    expect(state, same(tester.state(find.byType(_StatefulTestWidget))));

    await tester.pump(const Duration(milliseconds: 150));
    expect(state, same(tester.state(find.byType(_StatefulTestWidget))));

    await tester.pumpAndSettle();
    expect(state, same(tester.state(find.byType(_StatefulTestWidget))));

    controller.reverse();
    await tester.pump();
    expect(state, same(tester.state(find.byType(_StatefulTestWidget))));
    await tester.pump(const Duration(milliseconds: 150));
    expect(state, same(tester.state(find.byType(_StatefulTestWidget))));

    await tester.pump(const Duration(milliseconds: 150));
    expect(state, same(tester.state(find.byType(_StatefulTestWidget))));

    await tester.pumpAndSettle();
    expect(state, same(tester.state(find.byType(_StatefulTestWidget))));
  });

  testWidgets('does not jump when interrupted - forward', (WidgetTester tester) async {
    final AnimationController controller = AnimationController(
      vsync: const TestVSync(),
      duration: const Duration(milliseconds: 300),
    );
    addTearDown(controller.dispose);

    await tester.pumpWidget(
      Center(
        child: DualTransitionBuilder(
          animation: controller,
          forwardBuilder: (BuildContext context, Animation<double> animation, Widget? child) {
            return ScaleTransition(scale: animation, child: child);
          },
          reverseBuilder: (BuildContext context, Animation<double> animation, Widget? child) {
            return FadeTransition(
              opacity: Tween<double>(begin: 1.0, end: 0.0).animate(animation),
              child: child,
            );
          },
          child: Container(color: Colors.green, height: 100, width: 100),
        ),
      ),
    );
    expect(_getScale(tester), 0.0);
    expect(_getOpacity(tester), 1.0);

    controller.forward();
    await tester.pump();
    await tester.pump(const Duration(milliseconds: 150));
    expect(_getScale(tester), 0.5);
    expect(_getOpacity(tester), 1.0);

    controller.reverse();
    expect(_getScale(tester), 0.5);
    expect(_getOpacity(tester), 1.0);
    await tester.pump();
    expect(_getScale(tester), 0.5);
    expect(_getOpacity(tester), 1.0);

    await tester.pump(const Duration(milliseconds: 75));
    expect(_getScale(tester), 0.25);
    expect(_getOpacity(tester), 1.0);

    await tester.pump(const Duration(milliseconds: 75));
    expect(_getScale(tester), 0.0);
    expect(_getOpacity(tester), 1.0);

    await tester.pumpAndSettle();
    expect(_getScale(tester), 0.0);
    expect(_getOpacity(tester), 1.0);
  });

  testWidgets('does not jump when interrupted - reverse', (WidgetTester tester) async {
    final AnimationController controller = AnimationController(
      value: 1.0,
      vsync: const TestVSync(),
      duration: const Duration(milliseconds: 300),
    );
    addTearDown(controller.dispose);

    await tester.pumpWidget(
      Center(
        child: DualTransitionBuilder(
          animation: controller,
          forwardBuilder: (BuildContext context, Animation<double> animation, Widget? child) {
            return ScaleTransition(scale: animation, child: child);
          },
          reverseBuilder: (BuildContext context, Animation<double> animation, Widget? child) {
            return FadeTransition(
              opacity: Tween<double>(begin: 1.0, end: 0.0).animate(animation),
              child: child,
            );
          },
          child: Container(color: Colors.green, height: 100, width: 100),
        ),
      ),
    );
    expect(_getScale(tester), 1.0);
    expect(_getOpacity(tester), 1.0);

    controller.reverse();
    await tester.pump();
    await tester.pump(const Duration(milliseconds: 150));
    expect(_getScale(tester), 1.0);
    expect(_getOpacity(tester), 0.5);

    controller.forward();
    expect(_getScale(tester), 1.0);
    expect(_getOpacity(tester), 0.5);
    await tester.pump();
    expect(_getScale(tester), 1.0);
    expect(_getOpacity(tester), 0.5);

    await tester.pump(const Duration(milliseconds: 75));
    expect(_getScale(tester), 1.0);
    expect(_getOpacity(tester), 0.75);

    await tester.pump(const Duration(milliseconds: 75));
    expect(_getScale(tester), 1.0);
    expect(_getOpacity(tester), 1.0);

    await tester.pumpAndSettle();
    expect(_getScale(tester), 1.0);
    expect(_getOpacity(tester), 1.0);
  });
}

double _getScale(WidgetTester tester) {
  final ScaleTransition scale = tester.widget(find.byType(ScaleTransition));
  return scale.scale.value;
}

double _getOpacity(WidgetTester tester) {
  final FadeTransition scale = tester.widget(find.byType(FadeTransition));
  return scale.opacity.value;
}

class _StatefulTestWidget extends StatefulWidget {
  const _StatefulTestWidget({required this.name});

  final String name;

  @override
  State<_StatefulTestWidget> createState() => _StatefulTestWidgetState();
}

class _StatefulTestWidgetState extends State<_StatefulTestWidget> {
  @override
  Widget build(BuildContext context) {
    return Text(widget.name);
  }
}
