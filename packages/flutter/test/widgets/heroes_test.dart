// Copyright 2014 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

import 'dart:ui' as ui;

import 'package:flutter/cupertino.dart';
import 'package:flutter/foundation.dart';
import 'package:flutter/material.dart';
import 'package:flutter/rendering.dart';
import 'package:flutter_test/flutter_test.dart';
import 'package:leak_tracker_flutter_testing/leak_tracker_flutter_testing.dart';

import '../painting/image_test_utils.dart' show TestImageProvider;

Future<ui.Image> createTestImage() {
  final ui.Paint paint =
      ui.Paint()
        ..style = ui.PaintingStyle.stroke
        ..strokeWidth = 1.0;
  final ui.PictureRecorder recorder = ui.PictureRecorder();
  final ui.Canvas pictureCanvas = ui.Canvas(recorder);
  pictureCanvas.drawCircle(Offset.zero, 20.0, paint);
  final ui.Picture picture = recorder.endRecording();
  return picture.toImage(300, 300);
}

Key firstKey = const Key('first');
Key secondKey = const Key('second');
Key thirdKey = const Key('third');
Key simpleKey = const Key('simple');

Key homeRouteKey = const Key('homeRoute');
Key routeTwoKey = const Key('routeTwo');
Key routeThreeKey = const Key('routeThree');

bool transitionFromUserGestures = false;

final Map<String, WidgetBuilder> routes = <String, WidgetBuilder>{
  '/':
      (BuildContext context) => Material(
        child: ListView(
          key: homeRouteKey,
          children: <Widget>[
            const SizedBox(height: 100.0, width: 100.0),
            Card(
              child: Hero(
                tag: 'a',
                transitionOnUserGestures: transitionFromUserGestures,
                child: SizedBox(height: 100.0, width: 100.0, key: firstKey),
              ),
            ),
            const SizedBox(height: 100.0, width: 100.0),
            TextButton(
              child: const Text('two'),
              onPressed: () {
                Navigator.pushNamed(context, '/two');
              },
            ),
            TextButton(
              child: const Text('twoInset'),
              onPressed: () {
                Navigator.pushNamed(context, '/twoInset');
              },
            ),
            TextButton(
              child: const Text('simple'),
              onPressed: () {
                Navigator.pushNamed(context, '/simple');
              },
            ),
          ],
        ),
      ),
  '/two':
      (BuildContext context) => Material(
        child: ListView(
          key: routeTwoKey,
          children: <Widget>[
            TextButton(
              child: const Text('pop'),
              onPressed: () {
                Navigator.pop(context);
              },
            ),
            const SizedBox(height: 150.0, width: 150.0),
            Card(
              child: Hero(
                tag: 'a',
                transitionOnUserGestures: transitionFromUserGestures,
                child: SizedBox(height: 150.0, width: 150.0, key: secondKey),
              ),
            ),
            const SizedBox(height: 150.0, width: 150.0),
            TextButton(
              child: const Text('three'),
              onPressed: () {
                Navigator.push(context, ThreeRoute());
              },
            ),
          ],
        ),
      ),
  // This route is the same as /two except that Hero 'a' is shifted to the right by
  // 50 pixels. When the hero's in-flight bounds between / and /twoInset are animated
  // using MaterialRectArcTween (the default) they'll follow a different path
  // then when the flight starts at /twoInset and returns to /.
  '/twoInset':
      (BuildContext context) => Material(
        child: ListView(
          key: routeTwoKey,
          children: <Widget>[
            TextButton(
              child: const Text('pop'),
              onPressed: () {
                Navigator.pop(context);
              },
            ),
            const SizedBox(height: 150.0, width: 150.0),
            Card(
              child: Padding(
                padding: const EdgeInsets.only(left: 50.0),
                child: Hero(
                  tag: 'a',
                  transitionOnUserGestures: transitionFromUserGestures,
                  child: SizedBox(height: 150.0, width: 150.0, key: secondKey),
                ),
              ),
            ),
            const SizedBox(height: 150.0, width: 150.0),
            TextButton(
              child: const Text('three'),
              onPressed: () {
                Navigator.push(context, ThreeRoute());
              },
            ),
          ],
        ),
      ),
  // This route is the same as /two except that Hero 'a' is shifted to the right by
  // 50 pixels. When the hero's in-flight bounds between / and /twoInset are animated
  // using MaterialRectArcTween (the default) they'll follow a different path
  // then when the flight starts at /twoInset and returns to /.
  '/simple':
      (BuildContext context) => CupertinoPageScaffold(
        child: Center(
          child: Hero(
            tag: 'a',
            transitionOnUserGestures: transitionFromUserGestures,
            child: SizedBox(height: 150.0, width: 150.0, key: simpleKey),
          ),
        ),
      ),
};

class ThreeRoute extends MaterialPageRoute<void> {
  ThreeRoute()
    : super(
        builder: (BuildContext context) {
          return Material(
            key: routeThreeKey,
            child: ListView(
              children: <Widget>[
                const SizedBox(height: 200.0, width: 200.0),
                Card(
                  child: Hero(
                    tag: 'a',
                    child: SizedBox(height: 200.0, width: 200.0, key: thirdKey),
                  ),
                ),
                const SizedBox(height: 200.0, width: 200.0),
              ],
            ),
          );
        },
      );
}

class MutatingRoute extends MaterialPageRoute<void> {
  MutatingRoute()
    : super(
        builder: (BuildContext context) {
          return Hero(tag: 'a', key: UniqueKey(), child: const Text('MutatingRoute'));
        },
      );

  void markNeedsBuild() {
    setState(() {
      // Trigger a rebuild
    });
  }
}

class _SimpleStatefulWidget extends StatefulWidget {
  const _SimpleStatefulWidget({super.key});
  @override
  _SimpleState createState() => _SimpleState();
}

class _SimpleState extends State<_SimpleStatefulWidget> {
  int state = 0;

  @override
  Widget build(BuildContext context) => Text(state.toString());
}

class MyStatefulWidget extends StatefulWidget {
  const MyStatefulWidget({super.key, this.value = '123'});
  final String value;
  @override
  MyStatefulWidgetState createState() => MyStatefulWidgetState();
}

class MyStatefulWidgetState extends State<MyStatefulWidget> {
  @override
  Widget build(BuildContext context) => Text(widget.value);
}

Future<void> main() async {
  final ui.Image testImage = await createTestImage();

  setUp(() {
    transitionFromUserGestures = false;
  });

  testWidgets('Heroes animate', (WidgetTester tester) async {
    await tester.pumpWidget(MaterialApp(routes: routes));

    // the initial setup.

    expect(find.byKey(firstKey), isOnstage);
    expect(find.byKey(firstKey), isInCard);
    expect(find.byKey(secondKey), findsNothing);

    await tester.tap(find.text('two'));
    await tester.pump(); // begin navigation

    // at this stage, the second route is offstage, so that we can form the
    // hero party.

    expect(find.byKey(firstKey), isOnstage);
    expect(find.byKey(firstKey), isInCard);
    expect(find.byKey(secondKey, skipOffstage: false), isOffstage);
    expect(find.byKey(secondKey, skipOffstage: false), isInCard);

    await tester.pump();

    // at this stage, the heroes have just gone on their journey, we are
    // seeing them at t=16ms. The original page no longer contains the hero.

    expect(find.byKey(firstKey), findsNothing);

    expect(find.byKey(secondKey), findsOneWidget);
    expect(find.byKey(secondKey), isNotInCard);
    expect(find.byKey(secondKey), isOnstage);

    await tester.pump();

    // t=32ms for the journey. Surely they are still at it.

    expect(find.byKey(firstKey), findsNothing);

    expect(find.byKey(secondKey), findsOneWidget);

    expect(find.byKey(secondKey), findsOneWidget);
    expect(find.byKey(secondKey), isNotInCard);
    expect(find.byKey(secondKey), isOnstage);

    await tester.pump(const Duration(seconds: 1));

    // t=1.032s for the journey. The journey has ended (it ends this frame, in
    // fact). The hero should now be in the new page, onstage. The original
    // widget will be back as well now (though not visible).

    expect(find.byKey(firstKey), findsNothing);
    expect(find.byKey(secondKey), isOnstage);
    expect(find.byKey(secondKey), isInCard);

    await tester.pump();

    // Should not change anything.

    expect(find.byKey(firstKey), findsNothing);
    expect(find.byKey(secondKey), isOnstage);
    expect(find.byKey(secondKey), isInCard);

    // Now move on to view 3

    await tester.tap(find.text('three'));
    await tester.pump(); // begin navigation

    // at this stage, the second route is offstage, so that we can form the
    // hero party.

    expect(find.byKey(secondKey), isOnstage);
    expect(find.byKey(secondKey), isInCard);
    expect(find.byKey(thirdKey, skipOffstage: false), isOffstage);
    expect(find.byKey(thirdKey, skipOffstage: false), isInCard);

    await tester.pump();

    // at this stage, the heroes have just gone on their journey, we are
    // seeing them at t=16ms. The original page no longer contains the hero.

    expect(find.byKey(secondKey), findsNothing);
    expect(find.byKey(thirdKey), isOnstage);
    expect(find.byKey(thirdKey), isNotInCard);

    await tester.pump();

    // t=32ms for the journey. Surely they are still at it.

    expect(find.byKey(secondKey), findsNothing);
    expect(find.byKey(thirdKey), isOnstage);
    expect(find.byKey(thirdKey), isNotInCard);

    await tester.pump(const Duration(seconds: 1));

    // t=1.032s for the journey. The journey has ended (it ends this frame, in
    // fact). The hero should now be in the new page, onstage.

    expect(find.byKey(secondKey), findsNothing);
    expect(find.byKey(thirdKey), isOnstage);
    expect(find.byKey(thirdKey), isInCard);

    await tester.pump();

    // Should not change anything.

    expect(find.byKey(secondKey), findsNothing);
    expect(find.byKey(thirdKey), isOnstage);
    expect(find.byKey(thirdKey), isInCard);
  });

  testWidgets('Heroes still animate after hero controller is swapped.', (
    WidgetTester tester,
  ) async {
    final GlobalKey<NavigatorState> key = GlobalKey<NavigatorState>();
    final UniqueKey heroKey = UniqueKey();
    final HeroController controller1 = HeroController();
    addTearDown(controller1.dispose);

    await tester.pumpWidget(
      HeroControllerScope(
        controller: controller1,
        child: TestDependencies(
          child: Navigator(
            key: key,
            initialRoute: 'navigator1',
            onGenerateRoute: (RouteSettings s) {
              return MaterialPageRoute<void>(
                builder: (BuildContext c) {
                  return Hero(
                    tag: 'hero',
                    child: Container(),
                    flightShuttleBuilder: (
                      BuildContext flightContext,
                      Animation<double> animation,
                      HeroFlightDirection flightDirection,
                      BuildContext fromHeroContext,
                      BuildContext toHeroContext,
                    ) {
                      return Container(key: heroKey);
                    },
                  );
                },
                settings: s,
              );
            },
          ),
        ),
      ),
    );
    key.currentState!.push(
      MaterialPageRoute<void>(
        builder: (BuildContext c) {
          return Hero(
            tag: 'hero',
            child: Container(),
            flightShuttleBuilder: (
              BuildContext flightContext,
              Animation<double> animation,
              HeroFlightDirection flightDirection,
              BuildContext fromHeroContext,
              BuildContext toHeroContext,
            ) {
              return Container(key: heroKey);
            },
          );
        },
      ),
    );

    expect(find.byKey(heroKey), findsNothing);
    // Begins the navigation
    await tester.pump();
    await tester.pump(const Duration(milliseconds: 30));
    expect(find.byKey(heroKey), isOnstage);
    final HeroController controller2 = HeroController();
    addTearDown(controller2.dispose);

    // Pumps a new hero controller.
    await tester.pumpWidget(
      HeroControllerScope(
        controller: controller2,
        child: TestDependencies(
          child: Navigator(
            key: key,
            initialRoute: 'navigator1',
            onGenerateRoute: (RouteSettings s) {
              return MaterialPageRoute<void>(
                builder: (BuildContext c) {
                  return Hero(
                    tag: 'hero',
                    child: Container(),
                    flightShuttleBuilder: (
                      BuildContext flightContext,
                      Animation<double> animation,
                      HeroFlightDirection flightDirection,
                      BuildContext fromHeroContext,
                      BuildContext toHeroContext,
                    ) {
                      return Container(key: heroKey);
                    },
                  );
                },
                settings: s,
              );
            },
          ),
        ),
      ),
    );

    // The original animation still flies.
    expect(find.byKey(heroKey), isOnstage);
    // Waits for the animation finishes.
    await tester.pumpAndSettle();
    expect(find.byKey(heroKey), findsNothing);
  });

  testWidgets('Heroes animate should hide original hero', (WidgetTester tester) async {
    await tester.pumpWidget(MaterialApp(routes: routes));
    // Checks initial state.
    expect(find.byKey(firstKey), isOnstage);
    expect(find.byKey(firstKey), isInCard);
    expect(find.byKey(secondKey), findsNothing);

    await tester.tap(find.text('two'));
    await tester.pumpAndSettle(); // Waits for transition finishes.

    expect(find.byKey(firstKey), findsNothing);
    final Offstage first = tester.widget(
      find
          .ancestor(
            of: find.byKey(firstKey, skipOffstage: false),
            matching: find.byType(Offstage, skipOffstage: false),
          )
          .first,
    );
    // Original hero should stay hidden.
    expect(first.offstage, isTrue);
    expect(find.byKey(secondKey), isOnstage);
    expect(find.byKey(secondKey), isInCard);
  });

  testWidgets('Destination hero is rebuilt midflight', (WidgetTester tester) async {
    final MutatingRoute route = MutatingRoute();

    await tester.pumpWidget(
      MaterialApp(
        home: Material(
          child: ListView(
            children: <Widget>[
              const Hero(tag: 'a', child: Text('foo')),
              Builder(
                builder: (BuildContext context) {
                  return TextButton(
                    child: const Text('two'),
                    onPressed: () => Navigator.push(context, route),
                  );
                },
              ),
            ],
          ),
        ),
      ),
    );

    await tester.tap(find.text('two'));
    await tester.pump(const Duration(milliseconds: 10));

    route.markNeedsBuild();

    await tester.pump(const Duration(milliseconds: 10));
    await tester.pump(const Duration(seconds: 1));
  });

  testWidgets('Heroes animation is fastOutSlowIn', (WidgetTester tester) async {
    await tester.pumpWidget(MaterialApp(routes: routes));
    await tester.tap(find.text('two'));
    await tester.pump(); // begin navigation

    // Expect the height of the secondKey Hero to vary from 100 to 150
    // over duration and according to curve.

    const Duration duration = Duration(milliseconds: 300);
    const Curve curve = Curves.fastOutSlowIn;
    final double initialHeight = tester.getSize(find.byKey(firstKey, skipOffstage: false)).height;
    final double finalHeight = tester.getSize(find.byKey(secondKey, skipOffstage: false)).height;
    final double deltaHeight = finalHeight - initialHeight;
    const double epsilon = 0.001;

    await tester.pump(duration * 0.25);
    expect(
      tester.getSize(find.byKey(secondKey)).height,
      moreOrLessEquals(curve.transform(0.25) * deltaHeight + initialHeight, epsilon: epsilon),
    );

    await tester.pump(duration * 0.25);
    expect(
      tester.getSize(find.byKey(secondKey)).height,
      moreOrLessEquals(curve.transform(0.50) * deltaHeight + initialHeight, epsilon: epsilon),
    );

    await tester.pump(duration * 0.25);
    expect(
      tester.getSize(find.byKey(secondKey)).height,
      moreOrLessEquals(curve.transform(0.75) * deltaHeight + initialHeight, epsilon: epsilon),
    );

    await tester.pump(duration * 0.25);
    expect(
      tester.getSize(find.byKey(secondKey)).height,
      moreOrLessEquals(curve.transform(1.0) * deltaHeight + initialHeight, epsilon: epsilon),
    );
  });

  testWidgets('Heroes are not interactive', (WidgetTester tester) async {
    final List<String> log = <String>[];

    await tester.pumpWidget(
      MaterialApp(
        home: Center(
          child: Hero(
            tag: 'foo',
            child: GestureDetector(
              onTap: () {
                log.add('foo');
              },
              child: const SizedBox(width: 100.0, height: 100.0, child: Text('foo')),
            ),
          ),
        ),
        routes: <String, WidgetBuilder>{
          '/next': (BuildContext context) {
            return Align(
              alignment: Alignment.topLeft,
              child: Hero(
                tag: 'foo',
                child: GestureDetector(
                  onTap: () {
                    log.add('bar');
                  },
                  child: const SizedBox(width: 100.0, height: 150.0, child: Text('bar')),
                ),
              ),
            );
          },
        },
      ),
    );

    expect(log, isEmpty);
    await tester.tap(find.text('foo'));
    expect(log, equals(<String>['foo']));
    log.clear();

    final NavigatorState navigator = tester.state(find.byType(Navigator));
    navigator.pushNamed('/next');

    expect(log, isEmpty);
    await tester.tap(find.text('foo', skipOffstage: false), warnIfMissed: false);
    expect(log, isEmpty);

    await tester.pump(const Duration(milliseconds: 10));
    await tester.tap(find.text('foo', skipOffstage: false), warnIfMissed: false);
    expect(log, isEmpty);
    await tester.tap(find.text('bar', skipOffstage: false), warnIfMissed: false);
    expect(log, isEmpty);

    await tester.pump(const Duration(milliseconds: 10));
    expect(find.text('foo'), findsNothing);
    await tester.tap(find.text('bar', skipOffstage: false), warnIfMissed: false);
    expect(log, isEmpty);

    await tester.pump(const Duration(seconds: 1));
    expect(find.text('foo'), findsNothing);
    await tester.tap(find.text('bar'));
    expect(log, equals(<String>['bar']));
  });

  testWidgets('Popping on first frame does not cause hero observer to crash', (
    WidgetTester tester,
  ) async {
    await tester.pumpWidget(
      MaterialApp(
        onGenerateRoute: (RouteSettings settings) {
          return MaterialPageRoute<void>(
            settings: settings,
            builder: (BuildContext context) => Hero(tag: 'test', child: Container()),
          );
        },
      ),
    );
    await tester.pump();

    final Finder heroes = find.byType(Hero);
    expect(heroes, findsOneWidget);

    Navigator.pushNamed(heroes.evaluate().first, 'test');
    await tester.pump(); // adds the new page to the tree...

    Navigator.pop(heroes.evaluate().first);
    await tester.pump(); // ...and removes it straight away (since it's already at 0.0)
  });

  testWidgets('Overlapping starting and ending a hero transition works ok', (
    WidgetTester tester,
  ) async {
    await tester.pumpWidget(
      MaterialApp(
        onGenerateRoute: (RouteSettings settings) {
          return MaterialPageRoute<void>(
            settings: settings,
            builder: (BuildContext context) => Hero(tag: 'test', child: Container()),
          );
        },
      ),
    );
    await tester.pump();

    final Finder heroes = find.byType(Hero);
    expect(heroes, findsOneWidget);

    Navigator.pushNamed(heroes.evaluate().first, 'test');
    await tester.pump();
    await tester.pump(const Duration(hours: 1));

    Navigator.pushNamed(heroes.evaluate().first, 'test');
    await tester.pump();
    await tester.pump(const Duration(hours: 1));

    Navigator.pop(heroes.evaluate().first);
    await tester.pump();
    Navigator.pop(heroes.evaluate().first);
    await tester.pump(
      const Duration(hours: 1),
    ); // so the first transition is finished, but the second hasn't started
    await tester.pump();
  });

  testWidgets('One route, two heroes, same tag, throws', (WidgetTester tester) async {
    await tester.pumpWidget(
      MaterialApp(
        home: Material(
          child: ListView(
            children: <Widget>[
              const Hero(tag: 'a', child: Text('a')),
              const Hero(tag: 'a', child: Text('a too')),
              Builder(
                builder: (BuildContext context) {
                  return TextButton(
                    child: const Text('push'),
                    onPressed: () {
                      Navigator.push(
                        context,
                        PageRouteBuilder<void>(
                          pageBuilder: (
                            BuildContext context,
                            Animation<double> _,
                            Animation<double> __,
                          ) {
                            return const Text('fail');
                          },
                        ),
                      );
                    },
                  );
                },
              ),
            ],
          ),
        ),
      ),
    );

    await tester.tap(find.text('push'));
    await tester.pump();
    final dynamic exception = tester.takeException();
    expect(exception, isFlutterError);
    final FlutterError error = exception as FlutterError;
    expect(error.diagnostics.length, 3);
    final DiagnosticsNode last = error.diagnostics.last;
    expect(last, isA<DiagnosticsProperty<StatefulElement>>());
    expect(
      last.toStringDeep(),
      equalsIgnoringHashCodes('# Here is the subtree for one of the offending heroes: Hero\n'),
    );
    expect(last.style, DiagnosticsTreeStyle.dense);
    expect(
      error.toStringDeep(),
      equalsIgnoringHashCodes(
        'FlutterError\n'
        '   There are multiple heroes that share the same tag within a\n'
        '   subtree.\n'
        '   Within each subtree for which heroes are to be animated (i.e. a\n'
        '   PageRoute subtree), each Hero must have a unique non-null tag.\n'
        '   In this case, multiple heroes had the following tag: a\n'
        '   ├# Here is the subtree for one of the offending heroes: Hero\n',
      ),
    );
  });

  testWidgets('Hero push transition interrupted by a pop', (WidgetTester tester) async {
    await tester.pumpWidget(MaterialApp(routes: routes));

    // Initially the firstKey Card on the '/' route is visible
    expect(find.byKey(firstKey), isOnstage);
    expect(find.byKey(firstKey), isInCard);
    expect(find.byKey(secondKey), findsNothing);

    // Pushes MaterialPageRoute '/two'.
    await tester.tap(find.text('two'));

    // Start the flight of Hero 'a' from route '/' to route '/two'. Route '/two'
    // is now offstage.
    await tester.pump();

    final double initialHeight = tester.getSize(find.byKey(firstKey)).height;
    final double finalHeight = tester.getSize(find.byKey(secondKey, skipOffstage: false)).height;
    expect(finalHeight, greaterThan(initialHeight)); // simplify the checks below

    // Build the first hero animation frame in the navigator's overlay.
    await tester.pump();

    // At this point the hero widgets have been replaced by placeholders
    // and the destination hero has been moved to the overlay.
    expect(
      find.descendant(of: find.byKey(homeRouteKey), matching: find.byKey(firstKey)),
      findsNothing,
    );
    expect(
      find.descendant(of: find.byKey(routeTwoKey), matching: find.byKey(secondKey)),
      findsNothing,
    );
    expect(find.byKey(firstKey), findsNothing);
    expect(find.byKey(secondKey), isOnstage);

    // The duration of a MaterialPageRoute's transition is 300ms.
    // At 150ms Hero 'a' is mid-flight.
    await tester.pump(const Duration(milliseconds: 150));
    final double height150ms = tester.getSize(find.byKey(secondKey)).height;
    expect(height150ms, greaterThan(initialHeight));
    expect(height150ms, lessThan(finalHeight));

    // Pop route '/two' before the push transition to '/two' has finished.
    await tester.tap(find.text('pop'));

    // Restart the flight of Hero 'a'. Now it's flying from route '/two' to
    // route '/'.
    await tester.pump();

    // After flying in the opposite direction for 50ms Hero 'a' will
    // be smaller than it was, but bigger than its initial size.
    await tester.pump(const Duration(milliseconds: 50));
    final double height100ms = tester.getSize(find.byKey(secondKey)).height;
    expect(height100ms, lessThan(height150ms));
    expect(finalHeight, greaterThan(height100ms));

    // Hero a's return flight at 149ms. The outgoing (push) flight took
    // 150ms so we should be just about back to where Hero 'a' started.
    const double epsilon = 0.001;
    await tester.pump(const Duration(milliseconds: 99));
    moreOrLessEquals(
      tester.getSize(find.byKey(secondKey)).height - initialHeight,
      epsilon: epsilon,
    );

    // The flight is finished. We're back to where we started.
    await tester.pump(const Duration(milliseconds: 300));
    expect(find.byKey(firstKey), isOnstage);
    expect(find.byKey(firstKey), isInCard);
    expect(find.byKey(secondKey), findsNothing);
  });

  testWidgets('Hero pop transition interrupted by a push', (WidgetTester tester) async {
    await tester.pumpWidget(
      MaterialApp(
        routes: routes,
        theme: ThemeData(
          pageTransitionsTheme: const PageTransitionsTheme(
            builders: <TargetPlatform, PageTransitionsBuilder>{
              TargetPlatform.android: FadeUpwardsPageTransitionsBuilder(),
            },
          ),
        ),
      ),
    );

    // Pushes MaterialPageRoute '/two'.
    await tester.tap(find.text('two'));
    await tester.pump();
    await tester.pump(const Duration(seconds: 1));

    // Now the secondKey Card on the '/2' route is visible
    expect(find.byKey(secondKey), isOnstage);
    expect(find.byKey(secondKey), isInCard);
    expect(find.byKey(firstKey), findsNothing);

    // Pop MaterialPageRoute '/two'.
    await tester.tap(find.text('pop'));

    // Start the flight of Hero 'a' from route '/two' to route '/'. Route '/two'
    // is now offstage.
    await tester.pump();

    final double initialHeight = tester.getSize(find.byKey(secondKey)).height;
    final double finalHeight = tester.getSize(find.byKey(firstKey, skipOffstage: false)).height;
    expect(finalHeight, lessThan(initialHeight)); // simplify the checks below

    // Build the first hero animation frame in the navigator's overlay.
    await tester.pump();

    // At this point the hero widgets have been replaced by placeholders
    // and the destination hero has been moved to the overlay.
    expect(
      find.descendant(of: find.byKey(homeRouteKey), matching: find.byKey(firstKey)),
      findsNothing,
    );
    expect(
      find.descendant(of: find.byKey(routeTwoKey), matching: find.byKey(secondKey)),
      findsNothing,
    );
    expect(find.byKey(firstKey), isOnstage);
    expect(find.byKey(secondKey), findsNothing);

    // The duration of a MaterialPageRoute's transition is 300ms.
    // At 150ms Hero 'a' is mid-flight.
    await tester.pump(const Duration(milliseconds: 150));
    final double height150ms = tester.getSize(find.byKey(firstKey)).height;
    expect(height150ms, lessThan(initialHeight));
    expect(height150ms, greaterThan(finalHeight));

    // Push route '/two' before the pop transition from '/two' has finished.
    await tester.tap(find.text('two'));

    // Restart the flight of Hero 'a'. Now it's flying from route '/' to
    // route '/two'.
    await tester.pump();

    // After flying in the opposite direction for 50ms Hero 'a' will
    // be smaller than it was, but bigger than its initial size.
    await tester.pump(const Duration(milliseconds: 50));
    final double height200ms = tester.getSize(find.byKey(firstKey)).height;
    expect(height200ms, greaterThan(height150ms));
    expect(finalHeight, lessThan(height200ms));

    // Hero a's return flight at 149ms. The outgoing (push) flight took
    // 150ms so we should be just about back to where Hero 'a' started.
    const double epsilon = 0.001;
    await tester.pump(const Duration(milliseconds: 99));
    moreOrLessEquals(tester.getSize(find.byKey(firstKey)).height - initialHeight, epsilon: epsilon);

    // The flight is finished. We're back to where we started.
    await tester.pump(const Duration(milliseconds: 300));
    expect(find.byKey(secondKey), isOnstage);
    expect(find.byKey(secondKey), isInCard);
    expect(find.byKey(firstKey), findsNothing);
  });

  testWidgets('Destination hero disappears mid-flight', (WidgetTester tester) async {
    const Key homeHeroKey = Key('home hero');
    const Key routeHeroKey = Key('route hero');
    bool routeIncludesHero = true;
    late StateSetter heroCardSetState;

    // Show a 200x200 Hero tagged 'H', with key routeHeroKey
    final MaterialPageRoute<void> route = MaterialPageRoute<void>(
      builder: (BuildContext context) {
        return Material(
          child: ListView(
            children: <Widget>[
              StatefulBuilder(
                builder: (BuildContext context, StateSetter setState) {
                  heroCardSetState = setState;
                  return Card(
                    child:
                        routeIncludesHero
                            ? const Hero(
                              tag: 'H',
                              child: SizedBox(key: routeHeroKey, height: 200.0, width: 200.0),
                            )
                            : const SizedBox(height: 200.0, width: 200.0),
                  );
                },
              ),
              TextButton(
                child: const Text('POP'),
                onPressed: () {
                  Navigator.pop(context);
                },
              ),
            ],
          ),
        );
      },
    );

    // Show a 100x100 Hero tagged 'H' with key homeHeroKey
    await tester.pumpWidget(
      MaterialApp(
        home: Scaffold(
          body: Builder(
            builder: (BuildContext context) {
              // Navigator.push() needs context
              return ListView(
                children: <Widget>[
                  const Card(
                    child: Hero(
                      tag: 'H',
                      child: SizedBox(key: homeHeroKey, height: 100.0, width: 100.0),
                    ),
                  ),
                  TextButton(
                    child: const Text('PUSH'),
                    onPressed: () {
                      Navigator.push(context, route);
                    },
                  ),
                ],
              );
            },
          ),
        ),
      ),
    );

    // Pushes route
    await tester.tap(find.text('PUSH'));
    await tester.pump();
    await tester.pump();
    final double initialHeight = tester.getSize(find.byKey(routeHeroKey)).height;

    await tester.pump(const Duration(milliseconds: 10));
    double midflightHeight = tester.getSize(find.byKey(routeHeroKey)).height;
    expect(midflightHeight, greaterThan(initialHeight));
    expect(midflightHeight, lessThan(200.0));

    await tester.pump(const Duration(milliseconds: 300));
    await tester.pump();
    double finalHeight = tester.getSize(find.byKey(routeHeroKey)).height;
    expect(finalHeight, 200.0);

    // Complete the flight
    await tester.pump(const Duration(milliseconds: 100));

    // Rebuild route with its Hero

    heroCardSetState(() {
      routeIncludesHero = true;
    });
    await tester.pump();

    // Pops route
    await tester.tap(find.text('POP'));
    await tester.pump();
    await tester.pump();

    await tester.pump(const Duration(milliseconds: 10));
    midflightHeight = tester.getSize(find.byKey(homeHeroKey)).height;
    expect(midflightHeight, lessThan(finalHeight));
    expect(midflightHeight, greaterThan(100.0));

    // Remove the destination hero midflight
    heroCardSetState(() {
      routeIncludesHero = false;
    });
    await tester.pump();

    await tester.pump(const Duration(milliseconds: 300));
    finalHeight = tester.getSize(find.byKey(homeHeroKey)).height;
    expect(finalHeight, 100.0);
  });

  testWidgets('Destination hero scrolls mid-flight', (WidgetTester tester) async {
    const Key homeHeroKey = Key('home hero');
    const Key routeHeroKey = Key('route hero');
    const Key routeContainerKey = Key('route hero container');

    // Show a 200x200 Hero tagged 'H', with key routeHeroKey
    final MaterialPageRoute<void> route = MaterialPageRoute<void>(
      builder: (BuildContext context) {
        return Material(
          child: ListView(
            children: <Widget>[
              const SizedBox(height: 100.0),
              // This container will appear at Y=100
              Container(
                key: routeContainerKey,
                child: const Hero(
                  tag: 'H',
                  child: SizedBox(key: routeHeroKey, height: 200.0, width: 200.0),
                ),
              ),
              TextButton(
                child: const Text('POP'),
                onPressed: () {
                  Navigator.pop(context);
                },
              ),
              const SizedBox(height: 600.0),
            ],
          ),
        );
      },
    );

    // Show a 100x100 Hero tagged 'H' with key homeHeroKey
    await tester.pumpWidget(
      MaterialApp(
        theme: ThemeData(
          pageTransitionsTheme: const PageTransitionsTheme(
            builders: <TargetPlatform, PageTransitionsBuilder>{
              TargetPlatform.android: FadeUpwardsPageTransitionsBuilder(),
            },
          ),
        ),
        home: Scaffold(
          body: Builder(
            builder: (BuildContext context) {
              // Navigator.push() needs context
              return ListView(
                children: <Widget>[
                  const SizedBox(height: 200.0),
                  // This container will appear at Y=200
                  const Hero(
                    tag: 'H',
                    child: SizedBox(key: homeHeroKey, height: 100.0, width: 100.0),
                  ),
                  TextButton(
                    child: const Text('PUSH'),
                    onPressed: () {
                      Navigator.push(context, route);
                    },
                  ),
                  const SizedBox(height: 600.0),
                ],
              );
            },
          ),
        ),
      ),
    );

    // Pushes route
    await tester.tap(find.text('PUSH'));
    await tester.pump();
    await tester.pump();

    final double initialY = tester.getTopLeft(find.byKey(routeHeroKey)).dy;
    expect(initialY, 200.0);

    await tester.pump(const Duration(milliseconds: 100));
    final double yAt100ms = tester.getTopLeft(find.byKey(routeHeroKey)).dy;
    expect(yAt100ms, lessThan(200.0));
    expect(yAt100ms, greaterThan(100.0));

    // Scroll the target upwards by 25 pixels. The Hero flight's Y coordinate
    // will be redirected from 100 to 75.
    await tester.drag(
      find.byKey(routeContainerKey),
      const Offset(0.0, -25.0),
      warnIfMissed: false,
    ); // the container itself wouldn't be hit
    await tester.pump();
    await tester.pump(const Duration(milliseconds: 10));
    final double yAt110ms = tester.getTopLeft(find.byKey(routeHeroKey)).dy;
    expect(yAt110ms, lessThan(yAt100ms));
    expect(yAt110ms, greaterThan(75.0));

    await tester.pump(const Duration(milliseconds: 300));
    await tester.pump();
    final double finalHeroY = tester.getTopLeft(find.byKey(routeHeroKey)).dy;
    expect(finalHeroY, 75.0); // 100 less 25 for the scroll
  });

  testWidgets('Destination hero scrolls out of view mid-flight', (WidgetTester tester) async {
    const Key homeHeroKey = Key('home hero');
    const Key routeHeroKey = Key('route hero');
    const Key routeContainerKey = Key('route hero container');

    // Show a 200x200 Hero tagged 'H', with key routeHeroKey
    final MaterialPageRoute<void> route = MaterialPageRoute<void>(
      builder: (BuildContext context) {
        return Material(
          child: ListView(
            cacheExtent: 0.0,
            children: <Widget>[
              const SizedBox(height: 100.0),
              // This container will appear at Y=100
              Container(
                key: routeContainerKey,
                child: const Hero(
                  tag: 'H',
                  child: SizedBox(key: routeHeroKey, height: 200.0, width: 200.0),
                ),
              ),
              const SizedBox(height: 800.0),
            ],
          ),
        );
      },
    );

    // Show a 100x100 Hero tagged 'H' with key homeHeroKey
    await tester.pumpWidget(
      MaterialApp(
        home: Scaffold(
          body: Builder(
            builder: (BuildContext context) {
              // Navigator.push() needs context
              return ListView(
                children: <Widget>[
                  const SizedBox(height: 200.0),
                  // This container will appear at Y=200
                  const Hero(
                    tag: 'H',
                    child: SizedBox(key: homeHeroKey, height: 100.0, width: 100.0),
                  ),
                  TextButton(
                    child: const Text('PUSH'),
                    onPressed: () {
                      Navigator.push(context, route);
                    },
                  ),
                ],
              );
            },
          ),
        ),
      ),
    );

    // Pushes route
    await tester.tap(find.text('PUSH'));
    await tester.pump();
    await tester.pump();

    final double initialY = tester.getTopLeft(find.byKey(routeHeroKey)).dy;
    expect(initialY, 200.0);

    await tester.pump(const Duration(milliseconds: 100));
    final double yAt100ms = tester.getTopLeft(find.byKey(routeHeroKey)).dy;
    expect(yAt100ms, lessThan(200.0));
    expect(yAt100ms, greaterThan(100.0));

    await tester.drag(
      find.byKey(routeContainerKey),
      const Offset(0.0, -400.0),
      warnIfMissed: false,
    ); // the container itself wouldn't be hit
    await tester.pump();
    await tester.pump(const Duration(milliseconds: 10));
    expect(find.byKey(routeContainerKey), findsNothing); // Scrolled off the top

    // Flight continues (the hero will fade out) even though the destination
    // no longer exists.
    final double yAt110ms = tester.getTopLeft(find.byKey(routeHeroKey)).dy;
    expect(yAt110ms, lessThan(yAt100ms));
    expect(yAt110ms, greaterThan(100.0));

    await tester.pump(const Duration(milliseconds: 300));
    await tester.pump();
    expect(find.byKey(routeHeroKey), findsNothing);
  });

  testWidgets('Aborted flight', (WidgetTester tester) async {
    // See https://github.com/flutter/flutter/issues/5798
    const Key heroABKey = Key('AB hero');
    const Key heroBCKey = Key('BC hero');

    // Show a 150x150 Hero tagged 'BC'
    final MaterialPageRoute<void> routeC = MaterialPageRoute<void>(
      builder: (BuildContext context) {
        return Material(
          child: ListView(
            children: const <Widget>[
              // This container will appear at Y=0
              Hero(tag: 'BC', child: SizedBox(key: heroBCKey, height: 150.0, child: Text('Hero'))),
              SizedBox(height: 800.0),
            ],
          ),
        );
      },
    );

    // Show a height=200 Hero tagged 'AB' and a height=50 Hero tagged 'BC'
    final MaterialPageRoute<void> routeB = MaterialPageRoute<void>(
      builder: (BuildContext context) {
        return Material(
          child: ListView(
            children: <Widget>[
              const SizedBox(height: 100.0),
              // This container will appear at Y=100
              const Hero(
                tag: 'AB',
                child: SizedBox(key: heroABKey, height: 200.0, child: Text('Hero')),
              ),
              TextButton(
                child: const Text('PUSH C'),
                onPressed: () {
                  Navigator.push(context, routeC);
                },
              ),
              const Hero(tag: 'BC', child: SizedBox(height: 150.0, child: Text('Hero'))),
              const SizedBox(height: 800.0),
            ],
          ),
        );
      },
    );

    // Show a 100x100 Hero tagged 'AB' with key heroABKey
    await tester.pumpWidget(
      MaterialApp(
        home: Scaffold(
          body: Builder(
            builder: (BuildContext context) {
              // Navigator.push() needs context
              return ListView(
                children: <Widget>[
                  const SizedBox(height: 200.0),
                  // This container will appear at Y=200
                  const Hero(
                    tag: 'AB',
                    child: SizedBox(height: 100.0, width: 100.0, child: Text('Hero')),
                  ),
                  TextButton(
                    child: const Text('PUSH B'),
                    onPressed: () {
                      Navigator.push(context, routeB);
                    },
                  ),
                ],
              );
            },
          ),
        ),
      ),
    );

    // Pushes routeB
    await tester.tap(find.text('PUSH B'));
    await tester.pump();
    await tester.pump();

    final double initialY = tester.getTopLeft(find.byKey(heroABKey)).dy;
    expect(initialY, 200.0);

    await tester.pump(const Duration(milliseconds: 200));
    final double yAt200ms = tester.getTopLeft(find.byKey(heroABKey)).dy;
    // Hero AB is mid flight.
    expect(yAt200ms, lessThan(200.0));
    expect(yAt200ms, greaterThan(100.0));

    // Pushes route C, causes hero AB's flight to abort, hero BC's flight to start
    await tester.tap(find.text('PUSH C'));
    await tester.pump();
    await tester.pump();

    // Hero AB's aborted flight finishes where it was expected although
    // it's been faded out.
    await tester.pump(const Duration(milliseconds: 100));
    expect(tester.getTopLeft(find.byKey(heroABKey)).dy, 100.0);

    bool isVisible(RenderObject node) {
      RenderObject? currentNode = node;
      while (currentNode != null) {
        if (currentNode is RenderAnimatedOpacity && currentNode.opacity.value == 0) {
          return false;
        }
        currentNode = currentNode.parent;
      }
      return true;
    }

    // Of all heroes only one should be visible now.
    final Iterable<RenderObject> renderObjects = find
        .text('Hero')
        .evaluate()
        .map((Element e) => e.renderObject!);
    expect(renderObjects.where(isVisible).length, 1);

    // Hero BC's flight finishes normally.
    await tester.pump(const Duration(milliseconds: 300));
    expect(tester.getTopLeft(find.byKey(heroBCKey)).dy, 0.0);
  });

  testWidgets('Stateful hero child state survives flight', (WidgetTester tester) async {
    final MaterialPageRoute<void> route = MaterialPageRoute<void>(
      builder: (BuildContext context) {
        return Material(
          child: ListView(
            children: <Widget>[
              const Card(
                child: Hero(
                  tag: 'H',
                  child: SizedBox(height: 200.0, child: MyStatefulWidget(value: '456')),
                ),
              ),
              TextButton(
                child: const Text('POP'),
                onPressed: () {
                  Navigator.pop(context);
                },
              ),
            ],
          ),
        );
      },
    );

    await tester.pumpWidget(
      MaterialApp(
        home: Scaffold(
          body: Builder(
            builder: (BuildContext context) {
              // Navigator.push() needs context
              return ListView(
                children: <Widget>[
                  const Card(
                    child: Hero(
                      tag: 'H',
                      child: SizedBox(height: 100.0, child: MyStatefulWidget(value: '456')),
                    ),
                  ),
                  TextButton(
                    child: const Text('PUSH'),
                    onPressed: () {
                      Navigator.push(context, route);
                    },
                  ),
                ],
              );
            },
          ),
        ),
      ),
    );

    expect(find.text('456'), findsOneWidget);

    // Push route.
    await tester.tap(find.text('PUSH'));
    await tester.pump();
    await tester.pump();

    // Push flight underway.
    await tester.pump(const Duration(milliseconds: 100));
    // Visible in the hero animation.
    expect(find.text('456'), findsOneWidget);

    // Push flight finished.
    await tester.pump(const Duration(milliseconds: 300));
    expect(find.text('456'), findsOneWidget);

    // Pop route.
    await tester.tap(find.text('POP'));
    await tester.pump();
    await tester.pump();

    // Pop flight underway.
    await tester.pump(const Duration(milliseconds: 100));
    expect(find.text('456'), findsOneWidget);

    // Pop flight finished
    await tester.pump(const Duration(milliseconds: 300));
    expect(find.text('456'), findsOneWidget);
  });

  testWidgets('Hero createRectTween', (WidgetTester tester) async {
    RectTween createRectTween(Rect? begin, Rect? end) {
      return MaterialRectCenterArcTween(begin: begin, end: end);
    }

    final Map<String, WidgetBuilder> createRectTweenHeroRoutes = <String, WidgetBuilder>{
      '/':
          (BuildContext context) => Material(
            child: Column(
              crossAxisAlignment: CrossAxisAlignment.start,
              children: <Widget>[
                Hero(
                  tag: 'a',
                  createRectTween: createRectTween,
                  child: SizedBox(height: 100.0, width: 100.0, key: firstKey),
                ),
                TextButton(
                  child: const Text('two'),
                  onPressed: () {
                    Navigator.pushNamed(context, '/two');
                  },
                ),
              ],
            ),
          ),
      '/two':
          (BuildContext context) => Material(
            child: Column(
              children: <Widget>[
                SizedBox(
                  height: 200.0,
                  child: TextButton(
                    child: const Text('pop'),
                    onPressed: () {
                      Navigator.pop(context);
                    },
                  ),
                ),
                Hero(
                  tag: 'a',
                  createRectTween: createRectTween,
                  child: SizedBox(height: 200.0, width: 100.0, key: secondKey),
                ),
              ],
            ),
          ),
    };

    await tester.pumpWidget(MaterialApp(routes: createRectTweenHeroRoutes));
    expect(tester.getCenter(find.byKey(firstKey)), const Offset(50.0, 50.0));

    const double epsilon = 0.001;
    const Duration duration = Duration(milliseconds: 300);
    const Curve curve = Curves.fastOutSlowIn;
    final MaterialPointArcTween pushCenterTween = MaterialPointArcTween(
      begin: const Offset(50.0, 50.0),
      end: const Offset(400.0, 300.0),
    );

    await tester.tap(find.text('two'));
    await tester.pump(); // begin navigation

    // Verify that the center of the secondKey Hero flies along the
    // pushCenterTween arc for the push /two flight.

    await tester.pump();
    expect(tester.getCenter(find.byKey(secondKey)), const Offset(50.0, 50.0));

    await tester.pump(duration * 0.25);
    Offset actualHeroCenter = tester.getCenter(find.byKey(secondKey));
    Offset predictedHeroCenter = pushCenterTween.lerp(curve.transform(0.25));
    expect(actualHeroCenter, within<Offset>(distance: epsilon, from: predictedHeroCenter));

    await tester.pump(duration * 0.25);
    actualHeroCenter = tester.getCenter(find.byKey(secondKey));
    predictedHeroCenter = pushCenterTween.lerp(curve.transform(0.5));
    expect(actualHeroCenter, within<Offset>(distance: epsilon, from: predictedHeroCenter));

    await tester.pump(duration * 0.25);
    actualHeroCenter = tester.getCenter(find.byKey(secondKey));
    predictedHeroCenter = pushCenterTween.lerp(curve.transform(0.75));
    expect(actualHeroCenter, within<Offset>(distance: epsilon, from: predictedHeroCenter));

    await tester.pumpAndSettle();
    expect(tester.getCenter(find.byKey(secondKey)), const Offset(400.0, 300.0));

    // Verify that the center of the firstKey Hero flies along the
    // pushCenterTween arc for the pop /two flight.

    await tester.tap(find.text('pop'));
    await tester.pump(); // begin navigation

    final MaterialPointArcTween popCenterTween = MaterialPointArcTween(
      begin: const Offset(400.0, 300.0),
      end: const Offset(50.0, 50.0),
    );
    await tester.pump();
    expect(tester.getCenter(find.byKey(firstKey)), const Offset(400.0, 300.0));

    await tester.pump(duration * 0.25);
    actualHeroCenter = tester.getCenter(find.byKey(firstKey));
    predictedHeroCenter = popCenterTween.lerp(curve.transform(0.25));
    expect(actualHeroCenter, within<Offset>(distance: epsilon, from: predictedHeroCenter));

    await tester.pump(duration * 0.25);
    actualHeroCenter = tester.getCenter(find.byKey(firstKey));
    predictedHeroCenter = popCenterTween.lerp(curve.transform(0.5));
    expect(actualHeroCenter, within<Offset>(distance: epsilon, from: predictedHeroCenter));

    await tester.pump(duration * 0.25);
    actualHeroCenter = tester.getCenter(find.byKey(firstKey));
    predictedHeroCenter = popCenterTween.lerp(curve.transform(0.75));
    expect(actualHeroCenter, within<Offset>(distance: epsilon, from: predictedHeroCenter));

    await tester.pumpAndSettle();
    expect(tester.getCenter(find.byKey(firstKey)), const Offset(50.0, 50.0));
  });

  testWidgets('Hero createRectTween for Navigator that is not full screen', (
    WidgetTester tester,
  ) async {
    // Regression test for https://github.com/flutter/flutter/issues/25272

    RectTween createRectTween(Rect? begin, Rect? end) {
      return RectTween(begin: begin, end: end);
    }

    final Map<String, WidgetBuilder> createRectTweenHeroRoutes = <String, WidgetBuilder>{
      '/':
          (BuildContext context) => Material(
            child: Column(
              crossAxisAlignment: CrossAxisAlignment.start,
              children: <Widget>[
                Hero(
                  tag: 'a',
                  createRectTween: createRectTween,
                  child: SizedBox(height: 100.0, width: 100.0, key: firstKey),
                ),
                TextButton(
                  child: const Text('two'),
                  onPressed: () {
                    Navigator.pushNamed(context, '/two');
                  },
                ),
              ],
            ),
          ),
      '/two':
          (BuildContext context) => Material(
            child: Column(
              children: <Widget>[
                SizedBox(
                  height: 200.0,
                  child: TextButton(
                    child: const Text('pop'),
                    onPressed: () {
                      Navigator.pop(context);
                    },
                  ),
                ),
                Hero(
                  tag: 'a',
                  createRectTween: createRectTween,
                  child: SizedBox(height: 200.0, width: 100.0, key: secondKey),
                ),
              ],
            ),
          ),
    };

    const double leftPadding = 10.0;

    // MaterialApp and its Navigator are offset from the left
    await tester.pumpWidget(
      Padding(
        padding: const EdgeInsets.only(left: leftPadding),
        child: MaterialApp(routes: createRectTweenHeroRoutes),
      ),
    );
    expect(tester.getCenter(find.byKey(firstKey)), const Offset(leftPadding + 50.0, 50.0));

    const double epsilon = 0.001;
    const Duration duration = Duration(milliseconds: 300);
    const Curve curve = Curves.fastOutSlowIn;
    final RectTween pushRectTween = RectTween(
      begin: const Rect.fromLTWH(leftPadding, 0.0, 100.0, 100.0),
      end: const Rect.fromLTWH(350.0 + leftPadding / 2, 200.0, 100.0, 200.0),
    );

    await tester.tap(find.text('two'));
    await tester.pump(); // begin navigation

    // Verify that the rect of the secondKey Hero transforms as the
    // pushRectTween rect for the push /two flight.

    await tester.pump();
    expect(tester.getCenter(find.byKey(secondKey)), const Offset(50.0 + leftPadding, 50.0));

    await tester.pump(duration * 0.25);
    Rect actualHeroRect = tester.getRect(find.byKey(secondKey));
    Rect predictedHeroRect = pushRectTween.lerp(curve.transform(0.25))!;
    expect(actualHeroRect, within<Rect>(distance: epsilon, from: predictedHeroRect));

    await tester.pump(duration * 0.25);
    actualHeroRect = tester.getRect(find.byKey(secondKey));
    predictedHeroRect = pushRectTween.lerp(curve.transform(0.5))!;
    expect(actualHeroRect, within<Rect>(distance: epsilon, from: predictedHeroRect));

    await tester.pump(duration * 0.25);
    actualHeroRect = tester.getRect(find.byKey(secondKey));
    predictedHeroRect = pushRectTween.lerp(curve.transform(0.75))!;
    expect(actualHeroRect, within<Rect>(distance: epsilon, from: predictedHeroRect));

    await tester.pumpAndSettle();
    expect(tester.getCenter(find.byKey(secondKey)), const Offset(400.0 + leftPadding / 2, 300.0));

    // Verify that the rect of the firstKey Hero transforms as the
    // pushRectTween rect for the pop /two flight.

    await tester.tap(find.text('pop'));
    await tester.pump(); // begin navigation

    final RectTween popRectTween = RectTween(
      begin: const Rect.fromLTWH(350.0 + leftPadding / 2, 200.0, 100.0, 200.0),
      end: const Rect.fromLTWH(leftPadding, 0.0, 100.0, 100.0),
    );
    await tester.pump();
    expect(tester.getCenter(find.byKey(firstKey)), const Offset(400.0 + leftPadding / 2, 300.0));

    await tester.pump(duration * 0.25);
    actualHeroRect = tester.getRect(find.byKey(firstKey));
    predictedHeroRect = popRectTween.lerp(curve.transform(0.25))!;
    expect(actualHeroRect, within<Rect>(distance: epsilon, from: predictedHeroRect));

    await tester.pump(duration * 0.25);
    actualHeroRect = tester.getRect(find.byKey(firstKey));
    predictedHeroRect = popRectTween.lerp(curve.transform(0.5))!;
    expect(actualHeroRect, within<Rect>(distance: epsilon, from: predictedHeroRect));

    await tester.pump(duration * 0.25);
    actualHeroRect = tester.getRect(find.byKey(firstKey));
    predictedHeroRect = popRectTween.lerp(curve.transform(0.75))!;
    expect(actualHeroRect, within<Rect>(distance: epsilon, from: predictedHeroRect));

    await tester.pumpAndSettle();
    expect(tester.getCenter(find.byKey(firstKey)), const Offset(50.0 + leftPadding, 50.0));
  });

  testWidgets('Pop interrupts push, reverses flight', (WidgetTester tester) async {
    await tester.pumpWidget(MaterialApp(routes: routes));
    await tester.tap(find.text('twoInset'));
    await tester.pump(); // begin navigation from / to /twoInset.

    const double epsilon = 0.001;
    const Duration duration = Duration(milliseconds: 300);

    await tester.pump();
    final double x0 = tester.getTopLeft(find.byKey(secondKey)).dx;

    // Flight begins with the secondKey Hero widget lined up with the firstKey widget.
    expect(x0, 4.0);

    await tester.pump(duration * 0.1);
    final double x1 = tester.getTopLeft(find.byKey(secondKey)).dx;

    await tester.pump(duration * 0.1);
    final double x2 = tester.getTopLeft(find.byKey(secondKey)).dx;

    await tester.pump(duration * 0.1);
    final double x3 = tester.getTopLeft(find.byKey(secondKey)).dx;

    await tester.pump(duration * 0.1);
    final double x4 = tester.getTopLeft(find.byKey(secondKey)).dx;

    // Pop route /twoInset before the push transition from / to /twoInset has finished.
    await tester.tap(find.text('pop'));

    // We expect the hero to take the same path as it did flying from /
    // to /twoInset as it does now, flying from '/twoInset' back to /. The most
    // important checks below are the first (x4) and last (x0): the hero should
    // not jump from where it was when the push transition was interrupted by a
    // pop, and it should end up where the push started.

    await tester.pump();
    expect(tester.getTopLeft(find.byKey(secondKey)).dx, moreOrLessEquals(x4, epsilon: epsilon));

    await tester.pump(duration * 0.1);
    expect(tester.getTopLeft(find.byKey(secondKey)).dx, moreOrLessEquals(x3, epsilon: epsilon));

    await tester.pump(duration * 0.1);
    expect(tester.getTopLeft(find.byKey(secondKey)).dx, moreOrLessEquals(x2, epsilon: epsilon));

    await tester.pump(duration * 0.1);
    expect(tester.getTopLeft(find.byKey(secondKey)).dx, moreOrLessEquals(x1, epsilon: epsilon));

    await tester.pump(duration * 0.1);
    expect(tester.getTopLeft(find.byKey(secondKey)).dx, moreOrLessEquals(x0, epsilon: epsilon));

    // Below: show that a different pop Hero path is in fact taken after
    // a completed push transition.

    // Complete the pop transition and we're back to showing /.
    await tester.pumpAndSettle();
    expect(tester.getTopLeft(find.byKey(firstKey)).dx, 4.0); // Card contents are inset by 4.0.

    // Push /twoInset and wait for the transition to finish.
    await tester.tap(find.text('twoInset'));
    await tester.pumpAndSettle();
    expect(tester.getTopLeft(find.byKey(secondKey)).dx, 54.0);

    // Start the pop transition from /twoInset to /.
    await tester.tap(find.text('pop'));
    await tester.pump();

    // Now the firstKey widget is the flying hero widget and it starts
    // out lined up with the secondKey widget.
    await tester.pump();
    expect(tester.getTopLeft(find.byKey(firstKey)).dx, 54.0);

    // x0-x4 are the top left x coordinates for the beginning 40% of
    // the incoming flight. Advance the outgoing flight to the same
    // place.
    await tester.pump(duration * 0.6);

    await tester.pump(duration * 0.1);
    expect(
      tester.getTopLeft(find.byKey(firstKey)).dx,
      isNot(moreOrLessEquals(x4, epsilon: epsilon)),
    );

    await tester.pump(duration * 0.1);
    expect(
      tester.getTopLeft(find.byKey(firstKey)).dx,
      isNot(moreOrLessEquals(x3, epsilon: epsilon)),
    );

    // At this point the flight path arcs do start to get pretty close so
    // there's no point in comparing them.
    await tester.pump(duration * 0.1);

    // After the remaining 40% of the incoming flight is complete, we
    // expect to end up where the outgoing flight started.
    await tester.pump(duration * 0.1);
    expect(tester.getTopLeft(find.byKey(firstKey)).dx, x0);
  });

  testWidgets('Can override flight shuttle in to hero', (WidgetTester tester) async {
    await tester.pumpWidget(
      MaterialApp(
        home: Material(
          child: ListView(
            children: <Widget>[
              const Hero(tag: 'a', child: Text('foo')),
              Builder(
                builder: (BuildContext context) {
                  return TextButton(
                    child: const Text('two'),
                    onPressed:
                        () => Navigator.push<void>(
                          context,
                          MaterialPageRoute<void>(
                            builder: (BuildContext context) {
                              return Material(
                                child: Hero(
                                  tag: 'a',
                                  child: const Text('bar'),
                                  flightShuttleBuilder: (
                                    BuildContext flightContext,
                                    Animation<double> animation,
                                    HeroFlightDirection flightDirection,
                                    BuildContext fromHeroContext,
                                    BuildContext toHeroContext,
                                  ) {
                                    return const Text('baz');
                                  },
                                ),
                              );
                            },
                          ),
                        ),
                  );
                },
              ),
            ],
          ),
        ),
      ),
    );

    await tester.tap(find.text('two'));
    await tester.pump();
    await tester.pump(const Duration(milliseconds: 10));

    expect(find.text('foo'), findsNothing);
    expect(find.text('bar'), findsNothing);
    expect(find.text('baz'), findsOneWidget);
  });

  testWidgets('Can override flight shuttle in from hero', (WidgetTester tester) async {
    await tester.pumpWidget(
      MaterialApp(
        home: Material(
          child: ListView(
            children: <Widget>[
              Hero(
                tag: 'a',
                child: const Text('foo'),
                flightShuttleBuilder: (
                  BuildContext flightContext,
                  Animation<double> animation,
                  HeroFlightDirection flightDirection,
                  BuildContext fromHeroContext,
                  BuildContext toHeroContext,
                ) {
                  return const Text('baz');
                },
              ),
              Builder(
                builder: (BuildContext context) {
                  return TextButton(
                    child: const Text('two'),
                    onPressed:
                        () => Navigator.push<void>(
                          context,
                          MaterialPageRoute<void>(
                            builder: (BuildContext context) {
                              return const Material(child: Hero(tag: 'a', child: Text('bar')));
                            },
                          ),
                        ),
                  );
                },
              ),
            ],
          ),
        ),
      ),
    );

    await tester.tap(find.text('two'));
    await tester.pump();
    await tester.pump(const Duration(milliseconds: 10));

    expect(find.text('foo'), findsNothing);
    expect(find.text('bar'), findsNothing);
    expect(find.text('baz'), findsOneWidget);
  });

  // Regression test for https://github.com/flutter/flutter/issues/77720.
  testWidgets("toHero's shuttle builder over fromHero's shuttle builder", (
    WidgetTester tester,
  ) async {
    await tester.pumpWidget(
      MaterialApp(
        home: Material(
          child: ListView(
            children: <Widget>[
              Hero(
                tag: 'a',
                child: const Text('foo'),
                flightShuttleBuilder: (
                  BuildContext flightContext,
                  Animation<double> animation,
                  HeroFlightDirection flightDirection,
                  BuildContext fromHeroContext,
                  BuildContext toHeroContext,
                ) {
                  return const Text('fromHero text');
                },
              ),
              Builder(
                builder: (BuildContext context) {
                  return TextButton(
                    child: const Text('two'),
                    onPressed:
                        () => Navigator.push<void>(
                          context,
                          MaterialPageRoute<void>(
                            builder: (BuildContext context) {
                              return Material(
                                child: Hero(
                                  tag: 'a',
                                  child: const Text('bar'),
                                  flightShuttleBuilder: (
                                    BuildContext flightContext,
                                    Animation<double> animation,
                                    HeroFlightDirection flightDirection,
                                    BuildContext fromHeroContext,
                                    BuildContext toHeroContext,
                                  ) {
                                    return const Text('toHero text');
                                  },
                                ),
                              );
                            },
                          ),
                        ),
                  );
                },
              ),
            ],
          ),
        ),
      ),
    );

    await tester.tap(find.text('two'));
    await tester.pump();
    await tester.pump(const Duration(milliseconds: 10));

    expect(find.text('foo'), findsNothing);
    expect(find.text('bar'), findsNothing);
    expect(find.text('fromHero text'), findsNothing);
    expect(find.text('toHero text'), findsOneWidget);
  });

  testWidgets('Can override flight launch pads', (WidgetTester tester) async {
    await tester.pumpWidget(
      MaterialApp(
        home: Material(
          child: ListView(
            children: <Widget>[
              Hero(
                tag: 'a',
                child: const Text('Batman'),
                placeholderBuilder: (BuildContext context, Size heroSize, Widget child) {
                  return const Text('Venom');
                },
              ),
              Builder(
                builder: (BuildContext context) {
                  return TextButton(
                    child: const Text('two'),
                    onPressed:
                        () => Navigator.push<void>(
                          context,
                          MaterialPageRoute<void>(
                            builder: (BuildContext context) {
                              return Material(
                                child: Hero(
                                  tag: 'a',
                                  child: const Text('Wolverine'),
                                  placeholderBuilder: (
                                    BuildContext context,
                                    Size size,
                                    Widget child,
                                  ) {
                                    return const Text('Joker');
                                  },
                                ),
                              );
                            },
                          ),
                        ),
                  );
                },
              ),
            ],
          ),
        ),
      ),
    );

    await tester.tap(find.text('two'));
    await tester.pump();
    await tester.pump(const Duration(milliseconds: 10));

    expect(find.text('Batman'), findsNothing);
    // This shows up once but in the Hero because by default, the destination
    // Hero child is the widget in flight.
    expect(find.text('Wolverine'), findsOneWidget);
    expect(find.text('Venom'), findsOneWidget);
    expect(find.text('Joker'), findsOneWidget);
  });

  testWidgets(
    'Heroes do not transition on back gestures by default',
    (WidgetTester tester) async {
      await tester.pumpWidget(MaterialApp(routes: routes));

      expect(find.byKey(firstKey), isOnstage);
      expect(find.byKey(firstKey), isInCard);
      expect(find.byKey(secondKey), findsNothing);

      await tester.tap(find.text('two'));
      await tester.pump();
      await tester.pump(const Duration(milliseconds: 500));

      expect(find.byKey(firstKey), findsNothing);
      expect(find.byKey(secondKey), isOnstage);
      expect(find.byKey(secondKey), isInCard);

      final TestGesture gesture = await tester.startGesture(const Offset(5.0, 200.0));
      await gesture.moveBy(const Offset(20.0, 0.0));
      await gesture.moveBy(const Offset(180.0, 0.0));
      await gesture.up();
      await tester.pump();

      await tester.pump();

      // Both Heroes exist and are seated in their normal parents.
      expect(find.byKey(firstKey), isOnstage);
      expect(find.byKey(firstKey), isInCard);
      expect(find.byKey(secondKey), isOnstage);
      expect(find.byKey(secondKey), isInCard);

      // To make sure the hero had all chances of starting.
      await tester.pump(const Duration(milliseconds: 100));
      expect(find.byKey(firstKey), isOnstage);
      expect(find.byKey(firstKey), isInCard);
      expect(find.byKey(secondKey), isOnstage);
      expect(find.byKey(secondKey), isInCard);
    },
    variant: const TargetPlatformVariant(<TargetPlatform>{
      TargetPlatform.iOS,
      TargetPlatform.macOS,
    }),
  );

  testWidgets(
    'Heroes can transition on gesture in one frame',
    (WidgetTester tester) async {
      transitionFromUserGestures = true;
      await tester.pumpWidget(MaterialApp(routes: routes));

      await tester.tap(find.text('two'));
      await tester.pump();
      await tester.pump(const Duration(milliseconds: 500));

      expect(find.byKey(firstKey), findsNothing);
      expect(find.byKey(secondKey), isOnstage);
      expect(find.byKey(secondKey), isInCard);

      final TestGesture gesture = await tester.startGesture(const Offset(5.0, 200.0));
      await gesture.moveBy(const Offset(200.0, 0.0));
      await tester.pump();

      // We're going to page 1 so page 1's Hero is lifted into flight.
      expect(find.byKey(firstKey), isOnstage);
      expect(find.byKey(firstKey), isNotInCard);
      expect(find.byKey(secondKey), findsNothing);

      // Move further along.
      await gesture.moveBy(const Offset(500.0, 0.0));
      await tester.pump();

      // Same results.
      expect(find.byKey(firstKey), isOnstage);
      expect(find.byKey(firstKey), isNotInCard);
      expect(find.byKey(secondKey), findsNothing);

      await gesture.up();
      // Finish transition.
      await tester.pump();
      await tester.pump(const Duration(milliseconds: 500));

      // Hero A is back in the card.
      expect(find.byKey(firstKey), isOnstage);
      expect(find.byKey(firstKey), isInCard);
      expect(find.byKey(secondKey), findsNothing);
    },
    variant: const TargetPlatformVariant(<TargetPlatform>{
      TargetPlatform.iOS,
      TargetPlatform.macOS,
    }),
  );

  testWidgets(
    'Heroes animate should hide destination hero and display original hero in case of dismissed',
    (WidgetTester tester) async {
      transitionFromUserGestures = true;
      await tester.pumpWidget(MaterialApp(routes: routes));

      await tester.tap(find.text('two'));
      await tester.pumpAndSettle();

      expect(find.byKey(firstKey), findsNothing);
      expect(find.byKey(secondKey), isOnstage);
      expect(find.byKey(secondKey), isInCard);

      final TestGesture gesture = await tester.startGesture(const Offset(5.0, 200.0));
      await gesture.moveBy(const Offset(50.0, 0.0));
      await tester.pump();
      // It will only register the drag if we move a second time.
      await gesture.moveBy(const Offset(50.0, 0.0));
      await tester.pump();

      // We're going to page 1 so page 1's Hero is lifted into flight.
      expect(find.byKey(firstKey), isOnstage);
      expect(find.byKey(firstKey), isNotInCard);
      expect(find.byKey(secondKey), findsNothing);

      // Dismisses hero transition.
      await gesture.up();
      await tester.pump();
      await tester.pumpAndSettle();

      // We goes back to second page.
      expect(find.byKey(firstKey), findsNothing);
      expect(find.byKey(secondKey), isOnstage);
      expect(find.byKey(secondKey), isInCard);
    },
    variant: const TargetPlatformVariant(<TargetPlatform>{
      TargetPlatform.iOS,
      TargetPlatform.macOS,
    }),
  );

  testWidgets('Handles transitions when a non-default initial route is set', (
    WidgetTester tester,
  ) async {
    await tester.pumpWidget(MaterialApp(routes: routes, initialRoute: '/two'));
    expect(tester.takeException(), isNull);
    expect(find.text('two'), findsNothing);
    expect(find.text('three'), findsOneWidget);
  });

  testWidgets('Can push/pop on outer Navigator if nested Navigator contains Heroes', (
    WidgetTester tester,
  ) async {
    // Regression test for https://github.com/flutter/flutter/issues/28042.

    const String heroTag = 'You are my hero!';
    final GlobalKey<NavigatorState> rootNavigator = GlobalKey();
    final GlobalKey<NavigatorState> nestedNavigator = GlobalKey();
    final Key nestedRouteHeroBottom = UniqueKey();
    final Key nestedRouteHeroTop = UniqueKey();

    await tester.pumpWidget(
      MaterialApp(
        navigatorKey: rootNavigator,
        home: Navigator(
          key: nestedNavigator,
          onGenerateRoute: (RouteSettings settings) {
            return MaterialPageRoute<void>(
              builder: (BuildContext context) {
                return Hero(tag: heroTag, child: Placeholder(key: nestedRouteHeroBottom));
              },
            );
          },
        ),
      ),
    );

    nestedNavigator.currentState!.push(
      MaterialPageRoute<void>(
        builder: (BuildContext context) {
          return Hero(tag: heroTag, child: Placeholder(key: nestedRouteHeroTop));
        },
      ),
    );
    await tester.pumpAndSettle();

    // Both heroes are in the tree, one is offstage
    expect(find.byKey(nestedRouteHeroTop), findsOneWidget);
    expect(find.byKey(nestedRouteHeroBottom), findsNothing);
    expect(find.byKey(nestedRouteHeroBottom, skipOffstage: false), findsOneWidget);

    rootNavigator.currentState!.push(
      MaterialPageRoute<void>(
        builder: (BuildContext context) {
          return const Text('Foo');
        },
      ),
    );
    await tester.pumpAndSettle();

    expect(find.text('Foo'), findsOneWidget);
    // Both heroes are still in the tree, both are offstage.
    expect(find.byKey(nestedRouteHeroBottom), findsNothing);
    expect(find.byKey(nestedRouteHeroTop), findsNothing);
    expect(find.byKey(nestedRouteHeroBottom, skipOffstage: false), findsOneWidget);
    expect(find.byKey(nestedRouteHeroTop, skipOffstage: false), findsOneWidget);

    // Doesn't crash.
    expect(tester.takeException(), isNull);

    rootNavigator.currentState!.pop();
    await tester.pumpAndSettle();

    expect(find.text('Foo'), findsNothing);
    // Both heroes are in the tree, one is offstage
    expect(find.byKey(nestedRouteHeroTop), findsOneWidget);
    expect(find.byKey(nestedRouteHeroBottom), findsNothing);
    expect(find.byKey(nestedRouteHeroBottom, skipOffstage: false), findsOneWidget);
  });

  testWidgets('Can hero from route in root Navigator to route in nested Navigator', (
    WidgetTester tester,
  ) async {
    const String heroTag = 'foo';
    final GlobalKey<NavigatorState> rootNavigator = GlobalKey();
    final Key smallContainer = UniqueKey();
    final Key largeContainer = UniqueKey();

    await tester.pumpWidget(
      MaterialApp(
        navigatorKey: rootNavigator,
        home: Center(
          child: Card(
            child: Hero(
              tag: heroTag,
              child: Container(key: largeContainer, color: Colors.red, height: 200.0, width: 200.0),
            ),
          ),
        ),
      ),
    );

    // The initial setup.
    expect(find.byKey(largeContainer), isOnstage);
    expect(find.byKey(largeContainer), isInCard);
    expect(find.byKey(smallContainer, skipOffstage: false), findsNothing);

    rootNavigator.currentState!.push(
      MaterialPageRoute<void>(
        builder: (BuildContext context) {
          return Center(
            child: Card(
              child: Hero(
                tag: heroTag,
                child: Container(
                  key: smallContainer,
                  color: Colors.red,
                  height: 100.0,
                  width: 100.0,
                ),
              ),
            ),
          );
        },
      ),
    );
    await tester.pump();

    // The second route exists offstage.
    expect(find.byKey(largeContainer), isOnstage);
    expect(find.byKey(largeContainer), isInCard);
    expect(find.byKey(smallContainer, skipOffstage: false), isOffstage);
    expect(find.byKey(smallContainer, skipOffstage: false), isInCard);

    await tester.pump();

    // The hero started flying.
    expect(find.byKey(largeContainer), findsNothing);
    expect(find.byKey(smallContainer), isOnstage);
    expect(find.byKey(smallContainer), isNotInCard);

    await tester.pump(const Duration(milliseconds: 100));

    // The hero is in-flight.
    expect(find.byKey(largeContainer), findsNothing);
    expect(find.byKey(smallContainer), isOnstage);
    expect(find.byKey(smallContainer), isNotInCard);
    final Size size = tester.getSize(find.byKey(smallContainer));
    expect(size.height, greaterThan(100));
    expect(size.width, greaterThan(100));
    expect(size.height, lessThan(200));
    expect(size.width, lessThan(200));

    await tester.pumpAndSettle();

    // The transition has ended.
    expect(find.byKey(largeContainer), findsNothing);
    expect(find.byKey(smallContainer), isOnstage);
    expect(find.byKey(smallContainer), isInCard);
    expect(tester.getSize(find.byKey(smallContainer)), const Size(100, 100));
  });

  testWidgets('Hero within a Hero, throws', (WidgetTester tester) async {
    await tester.pumpWidget(
      const MaterialApp(
        home: Material(
          child: Hero(tag: 'a', child: Hero(tag: 'b', child: Text('Child of a Hero'))),
        ),
      ),
    );

    expect(tester.takeException(), isAssertionError);
  });

  testWidgets('Can push/pop on outer Navigator if nested Navigators contains same Heroes', (
    WidgetTester tester,
  ) async {
    const String heroTag = 'foo';
    final GlobalKey<NavigatorState> rootNavigator = GlobalKey<NavigatorState>();
    final Key rootRouteHero = UniqueKey();
    final Key nestedRouteHeroOne = UniqueKey();
    final Key nestedRouteHeroTwo = UniqueKey();
    final List<Key> keys = <Key>[nestedRouteHeroOne, nestedRouteHeroTwo];

    await tester.pumpWidget(
      CupertinoApp(
        navigatorKey: rootNavigator,
        home: CupertinoTabScaffold(
          tabBar: CupertinoTabBar(
            items: const <BottomNavigationBarItem>[
              BottomNavigationBarItem(icon: Icon(Icons.home)),
              BottomNavigationBarItem(icon: Icon(Icons.favorite)),
            ],
          ),
          tabBuilder: (BuildContext context, int index) {
            return CupertinoTabView(
              builder:
                  (BuildContext context) =>
                      Hero(tag: heroTag, child: Placeholder(key: keys[index])),
            );
          },
        ),
      ),
    );

    // Show both tabs to init.
    await tester.tap(find.byIcon(Icons.home));
    await tester.pump();

    await tester.tap(find.byIcon(Icons.favorite));
    await tester.pump();

    // Inner heroes are in the tree, one is offstage.
    expect(find.byKey(nestedRouteHeroTwo), findsOneWidget);
    expect(find.byKey(nestedRouteHeroOne), findsNothing);
    expect(find.byKey(nestedRouteHeroOne, skipOffstage: false), findsOneWidget);

    // Root hero is not in the tree.
    expect(find.byKey(rootRouteHero), findsNothing);

    rootNavigator.currentState!.push(
      MaterialPageRoute<void>(
        builder:
            (BuildContext context) => Hero(tag: heroTag, child: Placeholder(key: rootRouteHero)),
      ),
    );

    await tester.pumpAndSettle();

    // Inner heroes are still in the tree, both are offstage.
    expect(find.byKey(nestedRouteHeroOne), findsNothing);
    expect(find.byKey(nestedRouteHeroTwo), findsNothing);
    expect(find.byKey(nestedRouteHeroOne, skipOffstage: false), findsOneWidget);
    expect(find.byKey(nestedRouteHeroTwo, skipOffstage: false), findsOneWidget);

    // Root hero is in the tree.
    expect(find.byKey(rootRouteHero), findsOneWidget);

    // Doesn't crash.
    expect(tester.takeException(), isNull);

    rootNavigator.currentState!.pop();
    await tester.pumpAndSettle();

    // Root hero is not in the tree
    expect(find.byKey(rootRouteHero), findsNothing);

    // Both heroes are in the tree, one is offstage
    expect(find.byKey(nestedRouteHeroTwo), findsOneWidget);
    expect(find.byKey(nestedRouteHeroOne), findsNothing);
    expect(find.byKey(nestedRouteHeroOne, skipOffstage: false), findsOneWidget);
  });

  testWidgets('Hero within a Hero subtree, throws', (WidgetTester tester) async {
    await tester.pumpWidget(
      const MaterialApp(
        home: Material(
          child: Hero(tag: 'a', child: Hero(tag: 'b', child: Text('Child of a Hero'))),
        ),
      ),
    );

    expect(tester.takeException(), isAssertionError);
  });

  testWidgets('Hero within a Hero subtree with Builder, throws', (WidgetTester tester) async {
    await tester.pumpWidget(
      MaterialApp(
        home: Material(
          child: Hero(
            tag: 'a',
            child: Builder(
              builder: (BuildContext context) {
                return const Hero(tag: 'b', child: Text('Child of a Hero'));
              },
            ),
          ),
        ),
      ),
    );

    expect(tester.takeException(), isAssertionError);
  });

  testWidgets('Hero within a Hero subtree with LayoutBuilder, throws', (WidgetTester tester) async {
    await tester.pumpWidget(
      MaterialApp(
        home: Material(
          child: Hero(
            tag: 'a',
            child: LayoutBuilder(
              builder: (BuildContext context, BoxConstraints constraints) {
                return const Hero(tag: 'b', child: Text('Child of a Hero'));
              },
            ),
          ),
        ),
      ),
    );

    expect(tester.takeException(), isAssertionError);
  });

  testWidgets('Heroes fly on pushReplacement', (WidgetTester tester) async {
    // Regression test for https://github.com/flutter/flutter/issues/28041.

    const String heroTag = 'foo';
    final GlobalKey<NavigatorState> navigator = GlobalKey();
    final Key smallContainer = UniqueKey();
    final Key largeContainer = UniqueKey();

    await tester.pumpWidget(
      MaterialApp(
        navigatorKey: navigator,
        home: Center(
          child: Card(
            child: Hero(
              tag: heroTag,
              child: Container(key: largeContainer, color: Colors.red, height: 200.0, width: 200.0),
            ),
          ),
        ),
      ),
    );

    // The initial setup.
    expect(find.byKey(largeContainer), isOnstage);
    expect(find.byKey(largeContainer), isInCard);
    expect(find.byKey(smallContainer, skipOffstage: false), findsNothing);

    navigator.currentState!.pushReplacement(
      MaterialPageRoute<void>(
        builder: (BuildContext context) {
          return Center(
            child: Card(
              child: Hero(
                tag: heroTag,
                child: Container(
                  key: smallContainer,
                  color: Colors.red,
                  height: 100.0,
                  width: 100.0,
                ),
              ),
            ),
          );
        },
      ),
    );
    await tester.pump();

    // The second route exists offstage.
    expect(find.byKey(largeContainer), isOnstage);
    expect(find.byKey(largeContainer), isInCard);
    expect(find.byKey(smallContainer, skipOffstage: false), isOffstage);
    expect(find.byKey(smallContainer, skipOffstage: false), isInCard);

    await tester.pump();

    // The hero started flying.
    expect(find.byKey(largeContainer), findsNothing);
    expect(find.byKey(smallContainer), isOnstage);
    expect(find.byKey(smallContainer), isNotInCard);

    await tester.pump(const Duration(milliseconds: 100));

    // The hero is in-flight.
    expect(find.byKey(largeContainer), findsNothing);
    expect(find.byKey(smallContainer), isOnstage);
    expect(find.byKey(smallContainer), isNotInCard);
    final Size size = tester.getSize(find.byKey(smallContainer));
    expect(size.height, greaterThan(100));
    expect(size.width, greaterThan(100));
    expect(size.height, lessThan(200));
    expect(size.width, lessThan(200));

    await tester.pumpAndSettle();

    // The transition has ended.
    expect(find.byKey(largeContainer), findsNothing);
    expect(find.byKey(smallContainer), isOnstage);
    expect(find.byKey(smallContainer), isInCard);
    expect(tester.getSize(find.byKey(smallContainer)), const Size(100, 100));
  });

  testWidgets('Can add two page with heroes simultaneously using page API.', (
    WidgetTester tester,
  ) async {
    // Regression test for https://github.com/flutter/flutter/issues/115358.

    const String heroTag = 'foo';
    final GlobalKey<NavigatorState> navigator = GlobalKey();
    final Key smallContainer = UniqueKey();
    final Key largeContainer = UniqueKey();
    final MaterialPage<void> page1 = MaterialPage<void>(
      child: Center(
        child: Card(
          child: Hero(
            tag: heroTag,
            child: Container(key: largeContainer, color: Colors.red, height: 200.0, width: 200.0),
          ),
        ),
      ),
    );
    final MaterialPage<void> page2 = MaterialPage<void>(
      child: Center(
        child: Card(
          child: Hero(
            tag: heroTag,
            child: Container(color: Colors.red, height: 1000.0, width: 1000.0),
          ),
        ),
      ),
    );
    final MaterialPage<void> page3 = MaterialPage<void>(
      child: Center(
        child: Card(
          child: Hero(
            tag: heroTag,
            child: Container(key: smallContainer, color: Colors.red, height: 100.0, width: 100.0),
          ),
        ),
      ),
    );
    final HeroController controller = HeroController();
    addTearDown(controller.dispose);

    await tester.pumpWidget(
      MaterialApp(
        navigatorKey: navigator,
        home: Navigator(
          observers: <NavigatorObserver>[controller],
          pages: <Page<void>>[page1],
          onPopPage: (_, __) => false,
        ),
      ),
    );

    // The initial setup.
    expect(find.byKey(largeContainer), isOnstage);
    expect(find.byKey(largeContainer), isInCard);
    expect(find.byKey(smallContainer, skipOffstage: false), findsNothing);

    await tester.pumpWidget(
      MaterialApp(
        navigatorKey: navigator,
        home: Navigator(
          observers: <NavigatorObserver>[controller],
          pages: <Page<void>>[page1, page2, page3],
          onPopPage: (_, __) => false,
        ),
      ),
    );

    expect(find.byKey(largeContainer), isOnstage);
    expect(find.byKey(largeContainer), isInCard);
    expect(find.byKey(smallContainer, skipOffstage: false), isOffstage);
    expect(find.byKey(smallContainer, skipOffstage: false), isInCard);

    await tester.pump();

    // The hero started flying.
    expect(find.byKey(largeContainer), findsNothing);
    expect(find.byKey(smallContainer), isOnstage);
    expect(find.byKey(smallContainer), isNotInCard);

    await tester.pump(const Duration(milliseconds: 100));

    // The hero is in-flight.
    expect(find.byKey(largeContainer), findsNothing);
    expect(find.byKey(smallContainer), isOnstage);
    expect(find.byKey(smallContainer), isNotInCard);
    final Size size = tester.getSize(find.byKey(smallContainer));
    expect(size.height, greaterThan(100));
    expect(size.width, greaterThan(100));
    expect(size.height, lessThan(200));
    expect(size.width, lessThan(200));

    await tester.pumpAndSettle();

    // The transition has ended.
    expect(find.byKey(largeContainer), findsNothing);
    expect(find.byKey(smallContainer), isOnstage);
    expect(find.byKey(smallContainer), isInCard);
    expect(tester.getSize(find.byKey(smallContainer)), const Size(100, 100));
  });

  testWidgets('Can still trigger hero even if page underneath changes', (
    WidgetTester tester,
  ) async {
    // Regression test for https://github.com/flutter/flutter/issues/88578.

    const String heroTag = 'foo';
    final GlobalKey<NavigatorState> navigator = GlobalKey();
    final Key smallContainer = UniqueKey();
    final Key largeContainer = UniqueKey();
    final MaterialPage<void> unrelatedPage1 = MaterialPage<void>(
      key: UniqueKey(),
      child: Center(
        child: Card(child: Container(color: Colors.red, height: 1000.0, width: 1000.0)),
      ),
    );
    final MaterialPage<void> unrelatedPage2 = MaterialPage<void>(
      key: UniqueKey(),
      child: Center(
        child: Card(child: Container(color: Colors.red, height: 1000.0, width: 1000.0)),
      ),
    );
    final MaterialPage<void> page1 = MaterialPage<void>(
      key: UniqueKey(),
      child: Center(
        child: Card(
          child: Hero(
            tag: heroTag,
            child: Container(key: largeContainer, color: Colors.red, height: 200.0, width: 200.0),
          ),
        ),
      ),
    );
    final MaterialPage<void> page2 = MaterialPage<void>(
      key: UniqueKey(),
      child: Center(
        child: Card(
          child: Hero(
            tag: heroTag,
            child: Container(key: smallContainer, color: Colors.red, height: 100.0, width: 100.0),
          ),
        ),
      ),
    );
    final HeroController controller = HeroController();
    addTearDown(controller.dispose);

    await tester.pumpWidget(
      MaterialApp(
        navigatorKey: navigator,
        home: Navigator(
          observers: <NavigatorObserver>[controller],
          pages: <Page<void>>[unrelatedPage1, page1],
          onPopPage: (_, __) => false,
        ),
      ),
    );

    // The initial setup.
    expect(find.byKey(largeContainer), isOnstage);
    expect(find.byKey(largeContainer), isInCard);
    expect(find.byKey(smallContainer, skipOffstage: false), findsNothing);

    await tester.pumpWidget(
      MaterialApp(
        navigatorKey: navigator,
        home: Navigator(
          observers: <NavigatorObserver>[controller],
          pages: <Page<void>>[unrelatedPage2, page2],
          onPopPage: (_, __) => false,
        ),
      ),
    );

    expect(find.byKey(largeContainer), isOnstage);
    expect(find.byKey(largeContainer), isInCard);
    expect(find.byKey(smallContainer, skipOffstage: false), isOffstage);
    expect(find.byKey(smallContainer, skipOffstage: false), isInCard);

    await tester.pump();

    // The hero started flying.
    expect(find.byKey(largeContainer), findsNothing);
    expect(find.byKey(smallContainer), isOnstage);
    expect(find.byKey(smallContainer), isNotInCard);

    await tester.pump(const Duration(milliseconds: 100));

    // The hero is in-flight.
    expect(find.byKey(largeContainer), findsNothing);
    expect(find.byKey(smallContainer), isOnstage);
    expect(find.byKey(smallContainer), isNotInCard);
    final Size size = tester.getSize(find.byKey(smallContainer));
    expect(size.height, greaterThan(100));
    expect(size.width, greaterThan(100));
    expect(size.height, lessThan(200));
    expect(size.width, lessThan(200));

    await tester.pumpAndSettle();

    // The transition has ended.
    expect(find.byKey(largeContainer), findsNothing);
    expect(find.byKey(smallContainer), isOnstage);
    expect(find.byKey(smallContainer), isInCard);
    expect(tester.getSize(find.byKey(smallContainer)), const Size(100, 100));
  });

  testWidgets('On an iOS back swipe and snap, only a single flight should take place', (
    WidgetTester tester,
  ) async {
    int shuttlesBuilt = 0;
    Widget shuttleBuilder(
      BuildContext flightContext,
      Animation<double> animation,
      HeroFlightDirection flightDirection,
      BuildContext fromHeroContext,
      BuildContext toHeroContext,
    ) {
      shuttlesBuilt += 1;
      return const Text("I'm flying in a jetplane");
    }

    final GlobalKey<NavigatorState> navigatorKey = GlobalKey();
    await tester.pumpWidget(
      CupertinoApp(
        navigatorKey: navigatorKey,
        home: Hero(
          tag: navigatorKey,
          // Since we're popping, only the destination route's builder is used.
          flightShuttleBuilder: shuttleBuilder,
          transitionOnUserGestures: true,
          child: const Text('1'),
        ),
      ),
    );

    final CupertinoPageRoute<void> route2 = CupertinoPageRoute<void>(
      builder: (BuildContext context) {
        return CupertinoPageScaffold(
          child: Hero(tag: navigatorKey, transitionOnUserGestures: true, child: const Text('2')),
        );
      },
    );

    navigatorKey.currentState!.push(route2);
    await tester.pumpAndSettle();

    expect(shuttlesBuilt, 1);

    final TestGesture gesture = await tester.startGesture(const Offset(5.0, 200.0));
    await gesture.moveBy(const Offset(500.0, 0.0));
    await tester.pump();
    // Starting the back swipe creates a new hero shuttle.
    expect(shuttlesBuilt, 2);

    await gesture.up();
    await tester.pump();
    // After the lift, no additional shuttles should be created since it's the
    // same hero flight.
    expect(shuttlesBuilt, 2);

    // Did go far enough to snap out of this route.
    await tester.pump(const Duration(milliseconds: 301));
    expect(find.text('2'), findsNothing);
    // Still one shuttle.
    expect(shuttlesBuilt, 2);
  });

  testWidgets("From hero's state should be preserved, "
      'heroes work well with child widgets that has global keys', (WidgetTester tester) async {
    final GlobalKey<NavigatorState> navigatorKey = GlobalKey();
    final GlobalKey<_SimpleState> key1 = GlobalKey<_SimpleState>();
    final GlobalKey key2 = GlobalKey();

    await tester.pumpWidget(
      CupertinoApp(
        navigatorKey: navigatorKey,
        home: Row(
          crossAxisAlignment: CrossAxisAlignment.start,
          children: <Widget>[
            Hero(
              tag: 'hero',
              transitionOnUserGestures: true,
              child: _SimpleStatefulWidget(key: key1),
            ),
            const SizedBox(width: 10, height: 10, child: Text('1')),
          ],
        ),
      ),
    );

    final CupertinoPageRoute<void> route2 = CupertinoPageRoute<void>(
      builder: (BuildContext context) {
        return CupertinoPageScaffold(
          child: Hero(
            tag: 'hero',
            transitionOnUserGestures: true,
            // key2 is a `GlobalKey`. The hero animation should not
            // assert by having the same global keyed widget in more
            // than one place in the tree.
            child: _SimpleStatefulWidget(key: key2),
          ),
        );
      },
    );

    final _SimpleState state1 = key1.currentState!;
    state1.state = 1;

    navigatorKey.currentState!.push(route2);
    await tester.pump();

    expect(state1.mounted, isTrue);

    await tester.pumpAndSettle();
    expect(state1.state, 1);
    // The element should be mounted and unique.
    expect(state1.mounted, isTrue);

    navigatorKey.currentState!.pop();
    await tester.pumpAndSettle();

    // State is preserved.
    expect(state1.state, 1);
    // The element should be mounted and unique.
    expect(state1.mounted, isTrue);
  });

  testWidgets(
    "Hero works with images that don't have both width and height specified",
    // Regression test for https://github.com/flutter/flutter/issues/32356
    // and https://github.com/flutter/flutter/issues/31503
    (WidgetTester tester) async {
      final GlobalKey<NavigatorState> navigatorKey = GlobalKey();
      const Key imageKey1 = Key('image1');
      const Key imageKey2 = Key('image2');
      final TestImageProvider imageProvider = TestImageProvider(testImage);

      await tester.pumpWidget(
        CupertinoApp(
          navigatorKey: navigatorKey,
          home: Row(
            crossAxisAlignment: CrossAxisAlignment.start,
            children: <Widget>[
              Hero(
                tag: 'hero',
                transitionOnUserGestures: true,
                child: SizedBox(width: 100, child: Image(image: imageProvider, key: imageKey1)),
              ),
              const SizedBox(width: 10, height: 10, child: Text('1')),
            ],
          ),
        ),
      );

      final CupertinoPageRoute<void> route2 = CupertinoPageRoute<void>(
        builder: (BuildContext context) {
          return CupertinoPageScaffold(
            child: Hero(
              tag: 'hero',
              transitionOnUserGestures: true,
              child: Image(image: imageProvider, key: imageKey2),
            ),
          );
        },
      );

      // Load image before measuring the `Rect` of the `RenderImage`.
      imageProvider.complete();
      await tester.pump();
      final RenderImage renderImage = tester.renderObject(
        find.descendant(of: find.byKey(imageKey1), matching: find.byType(RawImage)),
      );

      // Before push image1 should be laid out correctly.
      expect(renderImage.size, const Size(100, 100));

      navigatorKey.currentState!.push(route2);
      await tester.pump();

      final TestGesture gesture = await tester.startGesture(const Offset(0.01, 300));
      await tester.pump();

      // Move (almost) across the screen, to make the animation as close to finish
      // as possible.
      await gesture.moveTo(const Offset(800, 200));
      await tester.pump();

      // image1 should snap to the top left corner of the Row widget.
      expect(
        tester.getRect(find.byKey(imageKey1, skipOffstage: false)),
        rectMoreOrLessEquals(
          tester.getTopLeft(find.widgetWithText(Row, '1')) & const Size(100, 100),
          epsilon: 0.01,
        ),
      );

      // Text should respect the correct final size of image1.
      expect(
        tester.getTopRight(find.byKey(imageKey1, skipOffstage: false)).dx,
        moreOrLessEquals(tester.getTopLeft(find.text('1')).dx, epsilon: 0.01),
      );
    },
  );

  // Regression test for https://github.com/flutter/flutter/issues/38183.
  testWidgets(
    'Remove user gesture driven flights when the gesture is invalid',
    (WidgetTester tester) async {
      transitionFromUserGestures = true;
      await tester.pumpWidget(MaterialApp(routes: routes));

      await tester.tap(find.text('simple'));
      await tester.pump();
      await tester.pumpAndSettle();

      expect(find.byKey(simpleKey), findsOneWidget);

      // Tap once to trigger a flight.
      await tester.tapAt(const Offset(10, 200));
      await tester.pumpAndSettle();

      // Wait till the previous gesture is accepted.
      await tester.pump(const Duration(milliseconds: 500));

      // Tap again to trigger another flight, see if it throws.
      await tester.tapAt(const Offset(10, 200));
      await tester.pumpAndSettle();

      // The simple route should still be on top.
      expect(find.byKey(simpleKey), findsOneWidget);
      expect(tester.takeException(), isNull);
    },
    variant: const TargetPlatformVariant(<TargetPlatform>{
      TargetPlatform.iOS,
      TargetPlatform.macOS,
    }),
  );

  // Regression test for https://github.com/flutter/flutter/issues/40239.
  testWidgets(
    'In a pop transition, when fromHero is null, the to hero should eventually become visible',
    (WidgetTester tester) async {
      final GlobalKey<NavigatorState> navigatorKey = GlobalKey();
      late StateSetter setState;
      bool shouldDisplayHero = true;
      await tester.pumpWidget(
        CupertinoApp(
          navigatorKey: navigatorKey,
          home: Hero(tag: navigatorKey, child: const Placeholder()),
        ),
      );

      final CupertinoPageRoute<void> route2 = CupertinoPageRoute<void>(
        builder: (BuildContext context) {
          return CupertinoPageScaffold(
            child: StatefulBuilder(
              builder: (BuildContext context, StateSetter setter) {
                setState = setter;
                return shouldDisplayHero
                    ? Hero(tag: navigatorKey, child: const Text('text'))
                    : const SizedBox();
              },
            ),
          );
        },
      );

      navigatorKey.currentState!.push(route2);
      await tester.pumpAndSettle();

      expect(find.text('text'), findsOneWidget);
      expect(find.byType(Placeholder), findsNothing);

      setState(() {
        shouldDisplayHero = false;
      });
      await tester.pumpAndSettle();

      expect(find.text('text'), findsNothing);

      navigatorKey.currentState!.pop();
      await tester.pumpAndSettle();

      expect(find.byType(Placeholder), findsOneWidget);
    },
  );

  testWidgets('popped hero uses fastOutSlowIn curve', (WidgetTester tester) async {
    final Key container1 = UniqueKey();
    final Key container2 = UniqueKey();
    final GlobalKey<NavigatorState> navigator = GlobalKey<NavigatorState>();

    final Animatable<Size?> tween = SizeTween(
      begin: const Size(200, 200),
      end: const Size(100, 100),
    ).chain(CurveTween(curve: Curves.fastOutSlowIn));

    await tester.pumpWidget(
      MaterialApp(
        navigatorKey: navigator,
        home: Scaffold(
          body: Center(
            child: Hero(
              tag: 'test',
              createRectTween: (Rect? begin, Rect? end) {
                return RectTween(begin: begin, end: end);
              },
              child: SizedBox(key: container1, height: 100, width: 100),
            ),
          ),
        ),
      ),
    );
    final Size originalSize = tester.getSize(find.byKey(container1));
    expect(originalSize, const Size(100, 100));

    navigator.currentState!.push(
      MaterialPageRoute<void>(
        builder: (BuildContext context) {
          return Scaffold(
            body: Center(
              child: Hero(
                tag: 'test',
                createRectTween: (Rect? begin, Rect? end) {
                  return RectTween(begin: begin, end: end);
                },
                child: SizedBox(key: container2, height: 200, width: 200),
              ),
            ),
          );
        },
      ),
    );
    await tester.pumpAndSettle();
    final Size newSize = tester.getSize(find.byKey(container2));
    expect(newSize, const Size(200, 200));

    navigator.currentState!.pop();
    await tester.pump();

    // Jump 25% into the transition (total length = 300ms)
    await tester.pump(const Duration(milliseconds: 75)); // 25% of 300ms
    Size heroSize = tester.getSize(find.byKey(container1));
    expect(heroSize, tween.transform(0.25));

    // Jump to 50% into the transition.
    await tester.pump(const Duration(milliseconds: 75)); // 25% of 300ms
    heroSize = tester.getSize(find.byKey(container1));
    expect(heroSize, tween.transform(0.50));

    // Jump to 75% into the transition.
    await tester.pump(const Duration(milliseconds: 75)); // 25% of 300ms
    heroSize = tester.getSize(find.byKey(container1));
    expect(heroSize, tween.transform(0.75));

    // Jump to 100% into the transition.
    await tester.pump(const Duration(milliseconds: 75)); // 25% of 300ms
    heroSize = tester.getSize(find.byKey(container1));
    expect(heroSize, tween.transform(1.0));
  });

  testWidgets('Heroes in enabled HeroMode do transition', (WidgetTester tester) async {
    await tester.pumpWidget(
      MaterialApp(
        home: Material(
          child: Column(
            children: <Widget>[
              HeroMode(
                child: Card(
                  child: Hero(
                    tag: 'a',
                    child: SizedBox(height: 100.0, width: 100.0, key: firstKey),
                  ),
                ),
              ),
              Builder(
                builder: (BuildContext context) {
                  return TextButton(
                    child: const Text('push'),
                    onPressed: () {
                      Navigator.push(
                        context,
                        PageRouteBuilder<void>(
                          pageBuilder: (
                            BuildContext context,
                            Animation<double> _,
                            Animation<double> __,
                          ) {
                            return Card(
                              child: Hero(
                                tag: 'a',
                                child: SizedBox(height: 150.0, width: 150.0, key: secondKey),
                              ),
                            );
                          },
                        ),
                      );
                    },
                  );
                },
              ),
            ],
          ),
        ),
      ),
    );

    expect(find.byKey(firstKey), isOnstage);
    expect(find.byKey(firstKey), isInCard);
    expect(find.byKey(secondKey), findsNothing);

    await tester.tap(find.text('push'));
    await tester.pump();

    expect(find.byKey(firstKey), isOnstage);
    expect(find.byKey(firstKey), isInCard);
    expect(find.byKey(secondKey, skipOffstage: false), isOffstage);
    expect(find.byKey(secondKey, skipOffstage: false), isInCard);

    await tester.pump();

    expect(find.byKey(firstKey), findsNothing);
    expect(find.byKey(secondKey), findsOneWidget);
    expect(find.byKey(secondKey), isNotInCard);
    expect(find.byKey(secondKey), isOnstage);

    await tester.pump(const Duration(seconds: 1));

    expect(find.byKey(firstKey), findsNothing);
    expect(find.byKey(secondKey), isOnstage);
    expect(find.byKey(secondKey), isInCard);
  });

  testWidgets('Heroes in disabled HeroMode do not transition', (WidgetTester tester) async {
    await tester.pumpWidget(
      MaterialApp(
        home: Material(
          child: Column(
            children: <Widget>[
              HeroMode(
                enabled: false,
                child: Card(
                  child: Hero(
                    tag: 'a',
                    child: SizedBox(height: 100.0, width: 100.0, key: firstKey),
                  ),
                ),
              ),
              Builder(
                builder: (BuildContext context) {
                  return TextButton(
                    child: const Text('push'),
                    onPressed: () {
                      Navigator.push(
                        context,
                        PageRouteBuilder<void>(
                          pageBuilder: (
                            BuildContext context,
                            Animation<double> _,
                            Animation<double> __,
                          ) {
                            return Card(
                              child: Hero(
                                tag: 'a',
                                child: SizedBox(height: 150.0, width: 150.0, key: secondKey),
                              ),
                            );
                          },
                        ),
                      );
                    },
                  );
                },
              ),
            ],
          ),
        ),
      ),
    );

    expect(find.byKey(firstKey), isOnstage);
    expect(find.byKey(firstKey), isInCard);
    expect(find.byKey(secondKey), findsNothing);

    await tester.tap(find.text('push'));
    await tester.pump();

    expect(find.byKey(firstKey), isOnstage);
    expect(find.byKey(firstKey), isInCard);
    expect(find.byKey(secondKey, skipOffstage: false), isOffstage);
    expect(find.byKey(secondKey, skipOffstage: false), isInCard);

    await tester.pump();

    // When HeroMode is disabled, heroes will not move.
    // So the original page contains the hero.
    expect(find.byKey(firstKey), findsOneWidget);

    // The hero should be in the new page, onstage, soon.
    expect(find.byKey(secondKey), findsOneWidget);
    expect(find.byKey(secondKey), isInCard);
    expect(find.byKey(secondKey), isOnstage);

    await tester.pump(const Duration(seconds: 1));

    expect(find.byKey(firstKey), findsNothing);

    expect(find.byKey(secondKey), findsOneWidget);
    expect(find.byKey(secondKey), isInCard);
    expect(find.byKey(secondKey), isOnstage);
  });

  testWidgets('kept alive Hero does not throw when the transition begins', (
    WidgetTester tester,
  ) async {
    final GlobalKey<NavigatorState> navigatorKey = GlobalKey<NavigatorState>();

    await tester.pumpWidget(
      MaterialApp(
        navigatorKey: navigatorKey,
        home: Scaffold(
          body: ListView(
            addAutomaticKeepAlives: false,
            addRepaintBoundaries: false,
            addSemanticIndexes: false,
            children: <Widget>[
              const KeepAlive(keepAlive: true, child: Hero(tag: 'a', child: Placeholder())),
              Container(height: 1000.0),
            ],
          ),
        ),
      ),
    );

    // Scroll to make the Hero invisible.
    await tester.drag(find.byType(ListView), const Offset(0.0, -1000.0));
    await tester.pump();

    expect(find.byType(TextField), findsNothing);

    navigatorKey.currentState?.push(
      MaterialPageRoute<void>(
        builder: (BuildContext context) {
          return const Scaffold(body: Center(child: Hero(tag: 'a', child: Placeholder())));
        },
      ),
    );
    await tester.pumpAndSettle();

    expect(tester.takeException(), isNull);
    // The Hero on the new route should be visible .
    expect(find.byType(Placeholder), findsOneWidget);
  });

  testWidgets('toHero becomes unpaintable after the transition begins', (
    WidgetTester tester,
  ) async {
    final GlobalKey<NavigatorState> navigatorKey = GlobalKey<NavigatorState>();
    final ScrollController controller = ScrollController();
    addTearDown(controller.dispose);

    RenderAnimatedOpacity? findRenderAnimatedOpacity() {
      RenderObject? parent = tester.renderObject(find.byType(Placeholder));
      while (parent is RenderObject && parent is! RenderAnimatedOpacity) {
        parent = parent.parent;
      }
      return parent is RenderAnimatedOpacity ? parent : null;
    }

    await tester.pumpWidget(
      MaterialApp(
        navigatorKey: navigatorKey,
        home: Scaffold(
          body: ListView(
            controller: controller,
            addAutomaticKeepAlives: false,
            addRepaintBoundaries: false,
            addSemanticIndexes: false,
            children: <Widget>[
              const KeepAlive(keepAlive: true, child: Hero(tag: 'a', child: Placeholder())),
              Container(height: 1000.0),
            ],
          ),
        ),
      ),
    );

    navigatorKey.currentState?.push(
      MaterialPageRoute<void>(
        builder: (BuildContext context) {
          return const Scaffold(body: Center(child: Hero(tag: 'a', child: Placeholder())));
        },
      ),
    );
    await tester.pump();
    await tester.pumpAndSettle();

    // Pop the new route, and before the animation finishes we scroll the toHero
    // to make it unpaintable.
    navigatorKey.currentState?.pop();
    await tester.pump();
    controller.jumpTo(1000);
    // Starts Hero animation and scroll animation almost simultaneously.
    // Scroll to make the Hero invisible.
    await tester.pump();
    expect(findRenderAnimatedOpacity()?.opacity.value, anyOf(isNull, 1.0));

    // In this frame the Hero animation finds out the toHero is not paintable,
    // and starts fading.
    await tester.pump();
    await tester.pump(const Duration(milliseconds: 100));

    expect(findRenderAnimatedOpacity()?.opacity.value, lessThan(1.0));

    await tester.pumpAndSettle();
    // The Hero on the new route should be invisible.
    expect(find.byType(Placeholder), findsNothing);
  });

  testWidgets('diverting to a keepalive but unpaintable hero', (WidgetTester tester) async {
    final GlobalKey<NavigatorState> navigatorKey = GlobalKey<NavigatorState>();

    await tester.pumpWidget(
      CupertinoApp(
        navigatorKey: navigatorKey,
        home: CupertinoPageScaffold(
          child: ListView(
            addAutomaticKeepAlives: false,
            addRepaintBoundaries: false,
            addSemanticIndexes: false,
            children: <Widget>[
              const KeepAlive(keepAlive: true, child: Hero(tag: 'a', child: Placeholder())),
              Container(height: 1000.0),
            ],
          ),
        ),
      ),
    );

    // Scroll to make the Hero invisible.
    await tester.drag(find.byType(ListView), const Offset(0.0, -1000.0));
    await tester.pump();

    expect(find.byType(Placeholder), findsNothing);
    expect(find.byType(Placeholder, skipOffstage: false), findsOneWidget);

    navigatorKey.currentState?.push(
      MaterialPageRoute<void>(
        builder: (BuildContext context) {
          return const Scaffold(body: Center(child: Hero(tag: 'a', child: Placeholder())));
        },
      ),
    );
    await tester.pumpAndSettle();

    // Yet another route that contains Hero 'a'.
    navigatorKey.currentState?.push(
      MaterialPageRoute<void>(
        builder: (BuildContext context) {
          return const Scaffold(body: Center(child: Hero(tag: 'a', child: Placeholder())));
        },
      ),
    );
    await tester.pumpAndSettle();

    // Pop both routes.
    navigatorKey.currentState?.pop();
    await tester.pump();
    await tester.pump(const Duration(milliseconds: 10));

    navigatorKey.currentState?.pop();
    await tester.pump();
    await tester.pump(const Duration(milliseconds: 10));
    expect(find.byType(Placeholder), findsOneWidget);

    await tester.pumpAndSettle();
    expect(tester.takeException(), isNull);
  });

  testWidgets('smooth transition between different incoming data', (WidgetTester tester) async {
    addTearDown(tester.view.reset);

    final GlobalKey<NavigatorState> navigatorKey = GlobalKey();
    const Key imageKey1 = Key('image1');
    const Key imageKey2 = Key('image2');
    final TestImageProvider imageProvider = TestImageProvider(testImage);

    tester.view.padding = const FakeViewPadding(top: 50);

    await tester.pumpWidget(
      MaterialApp(
        navigatorKey: navigatorKey,
        home: Scaffold(
          appBar: AppBar(title: const Text('test')),
          body: Hero(
            tag: 'imageHero',
            child: GridView.count(
              crossAxisCount: 3,
              shrinkWrap: true,
              children: <Widget>[Image(image: imageProvider, key: imageKey1)],
            ),
          ),
        ),
      ),
    );

    final MaterialPageRoute<void> route2 = MaterialPageRoute<void>(
      builder: (BuildContext context) {
        return Scaffold(
          body: Hero(
            tag: 'imageHero',
            child: GridView.count(
              crossAxisCount: 3,
              shrinkWrap: true,
              children: <Widget>[Image(image: imageProvider, key: imageKey2)],
            ),
          ),
        );
      },
    );

    // Load images.
    imageProvider.complete();
    await tester.pump();

    final double forwardRest = tester.getTopLeft(find.byType(Image)).dy;
    navigatorKey.currentState!.push(route2);
    await tester.pump();
    await tester.pump(const Duration(milliseconds: 1));
    expect(tester.getTopLeft(find.byType(Image)).dy, moreOrLessEquals(forwardRest, epsilon: 0.1));
    await tester.pumpAndSettle();

    navigatorKey.currentState!.pop(route2);
    await tester.pump();
    await tester.pump(const Duration(milliseconds: 300));
    expect(tester.getTopLeft(find.byType(Image)).dy, moreOrLessEquals(forwardRest, epsilon: 0.1));
    await tester.pumpAndSettle();
    expect(tester.getTopLeft(find.byType(Image)).dy, moreOrLessEquals(forwardRest, epsilon: 0.1));
  });

  test('HeroController dispatches memory events', () async {
    await expectLater(
      await memoryEvents(() => HeroController().dispose(), HeroController),
      areCreateAndDispose,
    );
  });
}

class TestDependencies extends StatelessWidget {
  const TestDependencies({required this.child, super.key});

  final Widget child;

  @override
  Widget build(BuildContext context) {
    return Directionality(
      textDirection: TextDirection.ltr,
      child: MediaQuery(data: MediaQueryData.fromView(View.of(context)), child: child),
    );
  }
}
