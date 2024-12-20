// Copyright 2014 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

import 'package:flutter/cupertino.dart';
import 'package:flutter_test/flutter_test.dart';

void main() {
  testWidgets('test iOS page transition (LTR)', (WidgetTester tester) async {
    await tester.pumpWidget(
      CupertinoApp(
        onGenerateRoute: (RouteSettings settings) {
          return CupertinoPageRoute<void>(
            settings: settings,
            builder: (BuildContext context) {
              final String pageNumber = settings.name == '/' ? '1' : '2';
              return Center(child: Text('Page $pageNumber'));
            },
          );
        },
      ),
    );

    final Offset widget1InitialTopLeft = tester.getTopLeft(find.text('Page 1'));

    tester.state<NavigatorState>(find.byType(Navigator)).pushNamed('/next');

    await tester.pump();
    await tester.pump(const Duration(milliseconds: 150));

    Offset widget1TransientTopLeft = tester.getTopLeft(find.text('Page 1'));
    Offset widget2TopLeft = tester.getTopLeft(find.text('Page 2'));

    // Page 1 is moving to the left.
    expect(widget1TransientTopLeft.dx, lessThan(widget1InitialTopLeft.dx));
    // Page 1 isn't moving vertically.
    expect(widget1TransientTopLeft.dy, equals(widget1InitialTopLeft.dy));
    // iOS transition is horizontal only.
    expect(widget1InitialTopLeft.dy, equals(widget2TopLeft.dy));
    // Page 2 is coming in from the right.
    expect(widget2TopLeft.dx, greaterThan(widget1InitialTopLeft.dx));

    // Will need to be changed if the animation curve or duration changes.
    expect(widget1TransientTopLeft.dx, moreOrLessEquals(158, epsilon: 1.0));

    await tester.pumpAndSettle();

    // Page 2 covers page 1.
    expect(find.text('Page 1'), findsNothing);
    expect(find.text('Page 2'), isOnstage);

    tester.state<NavigatorState>(find.byType(Navigator)).pop();
    await tester.pump();
    await tester.pump(const Duration(milliseconds: 100));

    widget1TransientTopLeft = tester.getTopLeft(find.text('Page 1'));
    widget2TopLeft = tester.getTopLeft(find.text('Page 2'));

    // Page 1 is coming back from the left.
    expect(widget1TransientTopLeft.dx, lessThan(widget1InitialTopLeft.dx));
    // Page 1 isn't moving vertically.
    expect(widget1TransientTopLeft.dy, equals(widget1InitialTopLeft.dy));
    // iOS transition is horizontal only.
    expect(widget1InitialTopLeft.dy, equals(widget2TopLeft.dy));
    // Page 2 is leaving towards the right.
    expect(widget2TopLeft.dx, greaterThan(widget1InitialTopLeft.dx));

    // Will need to be changed if the animation curve or duration changes.
    expect(widget1TransientTopLeft.dx, moreOrLessEquals(220, epsilon: 1.0));

    await tester.pumpAndSettle();

    expect(find.text('Page 1'), isOnstage);
    expect(find.text('Page 2'), findsNothing);

    widget1TransientTopLeft = tester.getTopLeft(find.text('Page 1'));

    // Page 1 is back where it started.
    expect(widget1InitialTopLeft, equals(widget1TransientTopLeft));
  });

  testWidgets('test iOS page transition (RTL)', (WidgetTester tester) async {
    await tester.pumpWidget(
      CupertinoApp(
        localizationsDelegates: const <LocalizationsDelegate<dynamic>>[
          RtlOverrideWidgetsDelegate(),
        ],
        onGenerateRoute: (RouteSettings settings) {
          return CupertinoPageRoute<void>(
            settings: settings,
            builder: (BuildContext context) {
              final String pageNumber = settings.name == '/' ? '1' : '2';
              return Center(child: Text('Page $pageNumber'));
            },
          );
        },
      ),
    );
    await tester.pump(); // to load the localization, since it doesn't use a synchronous future

    final Offset widget1InitialTopLeft = tester.getTopLeft(find.text('Page 1'));

    tester.state<NavigatorState>(find.byType(Navigator)).pushNamed('/next');

    await tester.pump();
    await tester.pump(const Duration(milliseconds: 150));

    Offset widget1TransientTopLeft = tester.getTopLeft(find.text('Page 1'));
    Offset widget2TopLeft = tester.getTopLeft(find.text('Page 2'));

    // Page 1 is moving to the right.
    expect(widget1TransientTopLeft.dx, greaterThan(widget1InitialTopLeft.dx));
    // Page 1 isn't moving vertically.
    expect(widget1TransientTopLeft.dy, equals(widget1InitialTopLeft.dy));
    // iOS transition is horizontal only.
    expect(widget1InitialTopLeft.dy, equals(widget2TopLeft.dy));
    // Page 2 is coming in from the left.
    expect(widget2TopLeft.dx, lessThan(widget1InitialTopLeft.dx));

    await tester.pumpAndSettle();

    // Page 2 covers page 1.
    expect(find.text('Page 1'), findsNothing);
    expect(find.text('Page 2'), isOnstage);

    tester.state<NavigatorState>(find.byType(Navigator)).pop();
    await tester.pump();
    await tester.pump(const Duration(milliseconds: 100));

    widget1TransientTopLeft = tester.getTopLeft(find.text('Page 1'));
    widget2TopLeft = tester.getTopLeft(find.text('Page 2'));

    // Page 1 is coming back from the right.
    expect(widget1TransientTopLeft.dx, greaterThan(widget1InitialTopLeft.dx));
    // Page 1 isn't moving vertically.
    expect(widget1TransientTopLeft.dy, equals(widget1InitialTopLeft.dy));
    // iOS transition is horizontal only.
    expect(widget1InitialTopLeft.dy, equals(widget2TopLeft.dy));
    // Page 2 is leaving towards the left.
    expect(widget2TopLeft.dx, lessThan(widget1InitialTopLeft.dx));

    await tester.pumpAndSettle();

    expect(find.text('Page 1'), isOnstage);
    expect(find.text('Page 2'), findsNothing);

    widget1TransientTopLeft = tester.getTopLeft(find.text('Page 1'));

    // Page 1 is back where it started.
    expect(widget1InitialTopLeft, equals(widget1TransientTopLeft));
  });

  testWidgets('test iOS fullscreen dialog transition', (WidgetTester tester) async {
    await tester.pumpWidget(const CupertinoApp(home: Center(child: Text('Page 1'))));

    final Offset widget1InitialTopLeft = tester.getTopLeft(find.text('Page 1'));

    tester
        .state<NavigatorState>(find.byType(Navigator))
        .push(
          CupertinoPageRoute<void>(
            builder: (BuildContext context) {
              return const Center(child: Text('Page 2'));
            },
            fullscreenDialog: true,
          ),
        );

    await tester.pump();
    await tester.pump(const Duration(milliseconds: 100));

    Offset widget1TransientTopLeft = tester.getTopLeft(find.text('Page 1'));
    Offset widget2TopLeft = tester.getTopLeft(find.text('Page 2'));

    // Page 1 doesn't move.
    expect(widget1TransientTopLeft, equals(widget1InitialTopLeft));
    // Fullscreen dialogs transitions vertically only.
    expect(widget1InitialTopLeft.dx, equals(widget2TopLeft.dx));
    // Page 2 is coming in from the bottom.
    expect(widget2TopLeft.dy, greaterThan(widget1InitialTopLeft.dy));

    await tester.pumpAndSettle();

    // Page 2 covers page 1.
    expect(find.text('Page 1'), findsNothing);
    expect(find.text('Page 2'), isOnstage);

    tester.state<NavigatorState>(find.byType(Navigator)).pop();
    await tester.pump();
    await tester.pump(const Duration(milliseconds: 100));

    widget1TransientTopLeft = tester.getTopLeft(find.text('Page 1'));
    widget2TopLeft = tester.getTopLeft(find.text('Page 2'));

    // Page 1 doesn't move.
    expect(widget1TransientTopLeft, equals(widget1InitialTopLeft));
    // Fullscreen dialogs transitions vertically only.
    expect(widget1InitialTopLeft.dx, equals(widget2TopLeft.dx));
    // Page 2 is leaving towards the bottom.
    expect(widget2TopLeft.dy, greaterThan(widget1InitialTopLeft.dy));

    await tester.pumpAndSettle();

    expect(find.text('Page 1'), isOnstage);
    expect(find.text('Page 2'), findsNothing);

    widget1TransientTopLeft = tester.getTopLeft(find.text('Page 1'));

    // Page 1 is back where it started.
    expect(widget1InitialTopLeft, equals(widget1TransientTopLeft));
  });

  testWidgets('test only edge swipes work (LTR)', (WidgetTester tester) async {
    await tester.pumpWidget(
      CupertinoApp(
        onGenerateRoute: (RouteSettings settings) {
          return CupertinoPageRoute<void>(
            settings: settings,
            builder: (BuildContext context) {
              final String pageNumber = settings.name == '/' ? '1' : '2';
              return Center(child: Text('Page $pageNumber'));
            },
          );
        },
      ),
    );

    tester.state<NavigatorState>(find.byType(Navigator)).pushNamed('/next');

    await tester.pump();
    await tester.pump(const Duration(seconds: 1));

    // Page 2 covers page 1.
    expect(find.text('Page 1'), findsNothing);
    expect(find.text('Page 2'), isOnstage);

    // Drag from the middle to the right.
    TestGesture gesture = await tester.startGesture(const Offset(200.0, 200.0));
    await gesture.moveBy(const Offset(300.0, 0.0));
    await tester.pump();

    // Nothing should happen.
    expect(find.text('Page 1'), findsNothing);
    expect(find.text('Page 2'), isOnstage);

    // Drag from the right to the left.
    gesture = await tester.startGesture(const Offset(795.0, 200.0));
    await gesture.moveBy(const Offset(-300.0, 0.0));
    await tester.pump();

    // Nothing should happen.
    expect(find.text('Page 1'), findsNothing);
    expect(find.text('Page 2'), isOnstage);

    // Drag from the right to the further right.
    gesture = await tester.startGesture(const Offset(795.0, 200.0));
    await gesture.moveBy(const Offset(300.0, 0.0));
    await tester.pump();

    // Nothing should happen.
    expect(find.text('Page 1'), findsNothing);
    expect(find.text('Page 2'), isOnstage);

    // Now drag from the left edge.
    gesture = await tester.startGesture(const Offset(5.0, 200.0));
    await gesture.moveBy(const Offset(300.0, 0.0));
    await tester.pump();

    // Page 1 is now visible.
    expect(find.text('Page 1'), isOnstage);
    expect(find.text('Page 2'), isOnstage);
  });

  testWidgets('test edge swipes work with media query padding (LTR)', (WidgetTester tester) async {
    await tester.pumpWidget(
      CupertinoApp(
        builder: (BuildContext context, Widget? navigator) {
          return MediaQuery(
            data: const MediaQueryData(padding: EdgeInsets.only(left: 40)),
            child: navigator!,
          );
        },
        home: const Placeholder(),
      ),
    );

    tester
        .state<NavigatorState>(find.byType(Navigator))
        .push(
          CupertinoPageRoute<void>(
            builder: (BuildContext context) => const Center(child: Text('Page 1')),
          ),
        );

    await tester.pump();
    await tester.pump(const Duration(seconds: 2));

    tester
        .state<NavigatorState>(find.byType(Navigator))
        .push(
          CupertinoPageRoute<void>(
            builder: (BuildContext context) => const Center(child: Text('Page 2')),
          ),
        );
    await tester.pump();
    await tester.pumpAndSettle();

    expect(find.text('Page 1'), findsNothing);
    expect(find.text('Page 2'), isOnstage);

    // Now drag from the left edge.
    final TestGesture gesture = await tester.startGesture(const Offset(35.0, 200.0));
    await gesture.moveBy(const Offset(300.0, 0.0));
    await tester.pump();
    await tester.pumpAndSettle();

    // Page 1 is now visible.
    expect(find.text('Page 1'), isOnstage);
    expect(find.text('Page 2'), isOnstage);
  });

  testWidgets('test edge swipes work with media query padding (RLT)', (WidgetTester tester) async {
    await tester.pumpWidget(
      CupertinoApp(
        builder: (BuildContext context, Widget? navigator) {
          return Directionality(
            textDirection: TextDirection.rtl,
            child: MediaQuery(
              data: const MediaQueryData(padding: EdgeInsets.only(right: 40)),
              child: navigator!,
            ),
          );
        },
        home: const Placeholder(),
      ),
    );

    tester
        .state<NavigatorState>(find.byType(Navigator))
        .push(
          CupertinoPageRoute<void>(
            builder: (BuildContext context) => const Center(child: Text('Page 1')),
          ),
        );

    await tester.pump();
    await tester.pumpAndSettle();

    tester
        .state<NavigatorState>(find.byType(Navigator))
        .push(
          CupertinoPageRoute<void>(
            builder: (BuildContext context) => const Center(child: Text('Page 2')),
          ),
        );

    await tester.pump();
    await tester.pumpAndSettle();

    expect(find.text('Page 1'), findsNothing);
    expect(find.text('Page 2'), isOnstage);

    // Now drag from the left edge.
    final TestGesture gesture = await tester.startGesture(const Offset(765.0, 200.0));
    await gesture.moveBy(const Offset(-300.0, 0.0));
    await tester.pump();
    await tester.pumpAndSettle();

    // Page 1 is now visible.
    expect(find.text('Page 1'), isOnstage);
    expect(find.text('Page 2'), isOnstage);
  });

  testWidgets('test only edge swipes work (RTL)', (WidgetTester tester) async {
    await tester.pumpWidget(
      CupertinoApp(
        localizationsDelegates: const <LocalizationsDelegate<dynamic>>[
          RtlOverrideWidgetsDelegate(),
        ],
        onGenerateRoute: (RouteSettings settings) {
          return CupertinoPageRoute<void>(
            settings: settings,
            builder: (BuildContext context) {
              final String pageNumber = settings.name == '/' ? '1' : '2';
              return Center(child: Text('Page $pageNumber'));
            },
          );
        },
      ),
    );
    await tester.pump(); // to load the localization, since it doesn't use a synchronous future

    tester.state<NavigatorState>(find.byType(Navigator)).pushNamed('/next');

    await tester.pump();
    await tester.pumpAndSettle();

    // Page 2 covers page 1.
    expect(find.text('Page 1'), findsNothing);
    expect(find.text('Page 2'), isOnstage);

    // Drag from the middle to the left.
    TestGesture gesture = await tester.startGesture(const Offset(200.0, 200.0));
    await gesture.moveBy(const Offset(-300.0, 0.0));
    await tester.pump();

    // Nothing should happen.
    expect(find.text('Page 1'), findsNothing);
    expect(find.text('Page 2'), isOnstage);

    // Drag from the left to the right.
    gesture = await tester.startGesture(const Offset(5.0, 200.0));
    await gesture.moveBy(const Offset(300.0, 0.0));
    await tester.pump();

    // Nothing should happen.
    expect(find.text('Page 1'), findsNothing);
    expect(find.text('Page 2'), isOnstage);

    // Drag from the left to the further left.
    gesture = await tester.startGesture(const Offset(5.0, 200.0));
    await gesture.moveBy(const Offset(-300.0, 0.0));
    await tester.pump();

    // Nothing should happen.
    expect(find.text('Page 1'), findsNothing);
    expect(find.text('Page 2'), isOnstage);

    // Now drag from the right edge.
    gesture = await tester.startGesture(const Offset(795.0, 200.0));
    await gesture.moveBy(const Offset(-300.0, 0.0));
    await tester.pump();

    // Page 1 is now visible.
    expect(find.text('Page 1'), isOnstage);
    expect(find.text('Page 2'), isOnstage);
  });

  testWidgets('test edge swipe then drop back at starting point works', (
    WidgetTester tester,
  ) async {
    await tester.pumpWidget(
      CupertinoApp(
        onGenerateRoute: (RouteSettings settings) {
          return CupertinoPageRoute<void>(
            settings: settings,
            builder: (BuildContext context) {
              final String pageNumber = settings.name == '/' ? '1' : '2';
              return Center(child: Text('Page $pageNumber'));
            },
          );
        },
      ),
    );

    tester.state<NavigatorState>(find.byType(Navigator)).pushNamed('/next');

    await tester.pump();
    await tester.pump(const Duration(seconds: 1));

    expect(find.text('Page 1'), findsNothing);
    expect(find.text('Page 2'), isOnstage);

    final TestGesture gesture = await tester.startGesture(const Offset(5, 200));
    await gesture.moveBy(const Offset(300, 0));
    await tester.pump();
    // Bring it exactly back such that there's nothing to animate when releasing.
    await gesture.moveBy(const Offset(-300, 0));
    await gesture.up();
    await tester.pump();

    expect(find.text('Page 1'), findsNothing);
    expect(find.text('Page 2'), isOnstage);
  });

  testWidgets('CupertinoPage does not lose its state when transitioning out', (
    WidgetTester tester,
  ) async {
    final GlobalKey<NavigatorState> navigator = GlobalKey<NavigatorState>();
    await tester.pumpWidget(KeepsStateTestWidget(navigatorKey: navigator));
    expect(find.text('subpage'), findsOneWidget);
    expect(find.text('home'), findsNothing);

    navigator.currentState!.pop();
    await tester.pump();

    expect(find.text('subpage'), findsOneWidget);
    expect(find.text('home'), findsOneWidget);
  });

  testWidgets('CupertinoPage restores its state', (WidgetTester tester) async {
    await tester.pumpWidget(
      RootRestorationScope(
        restorationId: 'root',
        child: MediaQuery(
          data: const MediaQueryData(),
          child: Directionality(
            textDirection: TextDirection.ltr,
            child: Navigator(
              onPopPage: (Route<dynamic> route, dynamic result) {
                return false;
              },
              pages: const <Page<Object?>>[
                CupertinoPage<void>(
                  restorationId: 'p1',
                  child: TestRestorableWidget(restorationId: 'p1'),
                ),
              ],
              restorationScopeId: 'nav',
              onGenerateRoute: (RouteSettings settings) {
                return CupertinoPageRoute<void>(
                  settings: settings,
                  builder: (BuildContext context) {
                    return TestRestorableWidget(restorationId: settings.name!);
                  },
                );
              },
            ),
          ),
        ),
      ),
    );

    expect(find.text('p1'), findsOneWidget);
    expect(find.text('count: 0'), findsOneWidget);

    await tester.tap(find.text('increment'));
    await tester.pump();
    expect(find.text('count: 1'), findsOneWidget);

    tester.state<NavigatorState>(find.byType(Navigator)).restorablePushNamed('p2');
    await tester.pumpAndSettle();

    expect(find.text('p1'), findsNothing);
    expect(find.text('p2'), findsOneWidget);

    await tester.tap(find.text('increment'));
    await tester.pump();
    await tester.tap(find.text('increment'));
    await tester.pump();
    expect(find.text('count: 2'), findsOneWidget);

    await tester.restartAndRestore();

    expect(find.text('p2'), findsOneWidget);
    expect(find.text('count: 2'), findsOneWidget);

    tester.state<NavigatorState>(find.byType(Navigator)).pop();
    await tester.pumpAndSettle();

    expect(find.text('p1'), findsOneWidget);
    expect(find.text('count: 1'), findsOneWidget);
  });
}

class RtlOverrideWidgetsDelegate extends LocalizationsDelegate<WidgetsLocalizations> {
  const RtlOverrideWidgetsDelegate();
  @override
  bool isSupported(Locale locale) => true;
  @override
  Future<WidgetsLocalizations> load(Locale locale) async => const RtlOverrideWidgetsLocalization();
  @override
  bool shouldReload(LocalizationsDelegate<WidgetsLocalizations> oldDelegate) => false;
}

class RtlOverrideWidgetsLocalization extends DefaultWidgetsLocalizations {
  const RtlOverrideWidgetsLocalization();
  @override
  TextDirection get textDirection => TextDirection.rtl;
}

class KeepsStateTestWidget extends StatefulWidget {
  const KeepsStateTestWidget({super.key, this.navigatorKey});

  final Key? navigatorKey;

  @override
  State<KeepsStateTestWidget> createState() => _KeepsStateTestWidgetState();
}

class _KeepsStateTestWidgetState extends State<KeepsStateTestWidget> {
  String? _subpage = 'subpage';

  @override
  Widget build(BuildContext context) {
    return CupertinoApp(
      home: Navigator(
        key: widget.navigatorKey,
        pages: <Page<void>>[
          const CupertinoPage<void>(child: Text('home')),
          if (_subpage != null) CupertinoPage<void>(child: Text(_subpage!)),
        ],
        onPopPage: (Route<dynamic> route, dynamic result) {
          if (!route.didPop(result)) {
            return false;
          }
          setState(() {
            _subpage = null;
          });
          return true;
        },
      ),
    );
  }
}

class TestRestorableWidget extends StatefulWidget {
  const TestRestorableWidget({super.key, required this.restorationId});

  final String restorationId;

  @override
  State<StatefulWidget> createState() => _TestRestorableWidgetState();
}

class _TestRestorableWidgetState extends State<TestRestorableWidget> with RestorationMixin {
  @override
  String? get restorationId => widget.restorationId;

  final RestorableInt counter = RestorableInt(0);

  @override
  void restoreState(RestorationBucket? oldBucket, bool initialRestore) {
    registerForRestoration(counter, 'counter');
  }

  @override
  void dispose() {
    super.dispose();
    counter.dispose();
  }

  @override
  Widget build(BuildContext context) {
    return Column(
      children: <Widget>[
        Text(widget.restorationId),
        Text('count: ${counter.value}'),
        CupertinoButton(
          onPressed: () {
            setState(() {
              counter.value++;
            });
          },
          child: const Text('increment'),
        ),
      ],
    );
  }
}
