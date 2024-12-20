// Copyright 2014 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

import 'package:flutter/material.dart';
import 'package:flutter_test/flutter_test.dart';

class TestTransition extends AnimatedWidget {
  const TestTransition({
    super.key,
    required this.childFirstHalf,
    required this.childSecondHalf,
    required Animation<double> animation,
  }) : super(listenable: animation);

  final Widget childFirstHalf;
  final Widget childSecondHalf;

  @override
  Widget build(BuildContext context) {
    final Animation<double> animation = listenable as Animation<double>;
    if (animation.value >= 0.5) {
      return childSecondHalf;
    }
    return childFirstHalf;
  }
}

class TestRoute<T> extends PageRoute<T> {
  TestRoute({required this.child, required RouteSettings settings, this.barrierColor})
    : super(settings: settings);

  final Widget child;

  @override
  Duration get transitionDuration => const Duration(milliseconds: 150);

  @override
  final Color? barrierColor;

  @override
  String? get barrierLabel => null;

  @override
  bool get maintainState => false;

  @override
  Widget buildPage(
    BuildContext context,
    Animation<double> animation,
    Animation<double> secondaryAnimation,
  ) {
    return child;
  }
}

void main() {
  const Duration kTwoTenthsOfTheTransitionDuration = Duration(milliseconds: 30);
  const Duration kFourTenthsOfTheTransitionDuration = Duration(milliseconds: 60);

  testWidgets('Check onstage/offstage handling around transitions', (WidgetTester tester) async {
    final GlobalKey insideKey = GlobalKey();

    String state({bool skipOffstage = true}) {
      String result = '';
      if (tester.any(find.text('A', skipOffstage: skipOffstage))) {
        result += 'A';
      }
      if (tester.any(find.text('B', skipOffstage: skipOffstage))) {
        result += 'B';
      }
      if (tester.any(find.text('C', skipOffstage: skipOffstage))) {
        result += 'C';
      }
      if (tester.any(find.text('D', skipOffstage: skipOffstage))) {
        result += 'D';
      }
      if (tester.any(find.text('E', skipOffstage: skipOffstage))) {
        result += 'E';
      }
      if (tester.any(find.text('F', skipOffstage: skipOffstage))) {
        result += 'F';
      }
      if (tester.any(find.text('G', skipOffstage: skipOffstage))) {
        result += 'G';
      }
      return result;
    }

    await tester.pumpWidget(
      MaterialApp(
        onGenerateRoute: (RouteSettings settings) {
          switch (settings.name) {
            case '/':
              return TestRoute<void>(
                settings: settings,
                child: Builder(
                  key: insideKey,
                  builder: (BuildContext context) {
                    final PageRoute<void> route = ModalRoute.of(context)! as PageRoute<void>;
                    return Column(
                      children: <Widget>[
                        TestTransition(
                          childFirstHalf: const Text('A'),
                          childSecondHalf: const Text('B'),
                          animation: route.animation!,
                        ),
                        TestTransition(
                          childFirstHalf: const Text('C'),
                          childSecondHalf: const Text('D'),
                          animation: route.secondaryAnimation!,
                        ),
                      ],
                    );
                  },
                ),
              );
            case '/2':
              return TestRoute<void>(settings: settings, child: const Text('E'));
            case '/3':
              return TestRoute<void>(settings: settings, child: const Text('F'));
            case '/4':
              return TestRoute<void>(settings: settings, child: const Text('G'));
          }
          return null;
        },
      ),
    );

    final NavigatorState navigator =
        insideKey.currentContext!.findAncestorStateOfType<NavigatorState>()!;

    expect(state(), equals('BC')); // transition ->1 is at 1.0

    navigator.pushNamed('/2');
    expect(state(), equals('BC')); // transition 1->2 is not yet built
    await tester.pump();
    expect(state(), equals('BC')); // transition 1->2 is at 0.0
    expect(state(skipOffstage: false), equals('BCE')); // E is offstage

    await tester.pump(kFourTenthsOfTheTransitionDuration);
    expect(state(), equals('BCE')); // transition 1->2 is at 0.4

    await tester.pump(kFourTenthsOfTheTransitionDuration);
    expect(state(), equals('BDE')); // transition 1->2 is at 0.8

    await tester.pump(kFourTenthsOfTheTransitionDuration);
    expect(state(), equals('E')); // transition 1->2 is at 1.0
    expect(
      state(skipOffstage: false),
      equals('E'),
    ); // B and C are gone, the route is inactive with maintainState=false

    navigator.pop();
    expect(state(), equals('E')); // transition 1<-2 is at 1.0, just reversed
    await tester.pump();
    await tester.pump();

    expect(state(), equals('BDE')); // transition 1<-2 is at 1.0

    await tester.pump(kFourTenthsOfTheTransitionDuration);
    expect(state(), equals('BDE')); // transition 1<-2 is at 0.6

    navigator.pushNamed('/3');
    expect(state(), equals('BDE')); // transition 1<-2 is at 0.6
    await tester.pump();
    expect(state(), equals('BDE')); // transition 1<-2 is at 0.6, 1->3 is at 0.0
    expect(state(skipOffstage: false), equals('BDEF')); // F is offstage since we're at 0.0

    await tester.pump(kFourTenthsOfTheTransitionDuration);
    expect(state(), equals('BCEF')); // transition 1<-2 is at 0.2, 1->3 is at 0.4
    expect(state(skipOffstage: false), equals('BCEF')); // nothing secret going on here

    await tester.pump(kFourTenthsOfTheTransitionDuration);
    expect(state(), equals('BDF')); // transition 1<-2 is done, 1->3 is at 0.8

    navigator.pop();
    expect(state(), equals('BDF')); // transition 1<-3 is at 0.8, just reversed
    await tester.pump();
    expect(state(), equals('BDF')); // transition 1<-3 is at 0.8

    await tester.pump(kTwoTenthsOfTheTransitionDuration); // notice that dT=0.2 here, not 0.4
    expect(state(), equals('BDF')); // transition 1<-3 is at 0.6

    await tester.pump(kFourTenthsOfTheTransitionDuration);
    expect(state(), equals('BCF')); // transition 1<-3 is at 0.2

    navigator.pushNamed('/4');
    expect(state(), equals('BCF')); // transition 1<-3 is at 0.2, 1->4 is not yet built
    await tester.pump();
    expect(state(), equals('BCF')); // transition 1<-3 is at 0.2, 1->4 is at 0.0
    expect(state(skipOffstage: false), equals('BCFG')); // G is offstage

    await tester.pump(kFourTenthsOfTheTransitionDuration);
    expect(state(), equals('BCG')); // transition 1<-3 is done, 1->4 is at 0.4

    await tester.pump(kFourTenthsOfTheTransitionDuration);
    expect(state(), equals('BDG')); // transition 1->4 is at 0.8

    await tester.pump(kFourTenthsOfTheTransitionDuration);
    expect(state(), equals('G')); // transition 1->4 is done
    expect(state(skipOffstage: false), equals('G')); // route 1 is not around any more
  });

  testWidgets('Check onstage/offstage handling of barriers around transitions', (
    WidgetTester tester,
  ) async {
    await tester.pumpWidget(
      MaterialApp(
        onGenerateRoute:
            (RouteSettings settings) => switch (settings.name) {
              '/' => TestRoute<void>(settings: settings, child: const Text('A')),
              '/1' => TestRoute<void>(
                settings: settings,
                barrierColor: const Color(0xFFFFFF00),
                child: const Text('B'),
              ),
              _ => null,
            },
      ),
    );
    expect(find.byType(ModalBarrier), findsOneWidget);

    tester.state<NavigatorState>(find.byType(Navigator)).pushNamed('/1');
    expect(find.byType(ModalBarrier), findsOneWidget);

    await tester.pump();
    expect(find.byType(ModalBarrier), findsNWidgets(2));
    expect(tester.widget<ModalBarrier>(find.byType(ModalBarrier).first).color, isNull);
    expect(tester.widget<ModalBarrier>(find.byType(ModalBarrier).last).color, isNull);

    await tester.pump(const Duration(seconds: 1));
    expect(find.byType(ModalBarrier), findsOneWidget);
    expect(tester.widget<ModalBarrier>(find.byType(ModalBarrier)).color, const Color(0xFFFFFF00));
  });
}
