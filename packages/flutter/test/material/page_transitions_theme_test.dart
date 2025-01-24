// Copyright 2014 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

import 'package:flutter/cupertino.dart';
import 'package:flutter/foundation.dart';
import 'package:flutter/material.dart';
import 'package:flutter/rendering.dart';
import 'package:flutter/services.dart';
import 'package:flutter_test/flutter_test.dart';

void main() {
  final TestWidgetsFlutterBinding binding = TestWidgetsFlutterBinding.ensureInitialized();

  testWidgets('Default PageTransitionsTheme platform', (WidgetTester tester) async {
    await tester.pumpWidget(const MaterialApp(home: Text('home')));
    final PageTransitionsTheme theme =
        Theme.of(tester.element(find.text('home'))).pageTransitionsTheme;
    expect(theme.builders, isNotNull);
    for (final TargetPlatform platform in TargetPlatform.values) {
      switch (platform) {
        case TargetPlatform.android:
        case TargetPlatform.iOS:
        case TargetPlatform.macOS:
          expect(
            theme.builders[platform],
            isNotNull,
            reason: 'theme builder for $platform is null',
          );
        case TargetPlatform.fuchsia:
        case TargetPlatform.linux:
        case TargetPlatform.windows:
          expect(
            theme.builders[platform],
            isNull,
            reason: 'theme builder for $platform is not null',
          );
      }
    }
  });

  testWidgets(
    'Default PageTransitionsTheme builds a CupertinoPageTransition',
    (WidgetTester tester) async {
      final Map<String, WidgetBuilder> routes = <String, WidgetBuilder>{
        '/':
            (BuildContext context) => Material(
              child: TextButton(
                child: const Text('push'),
                onPressed: () {
                  Navigator.of(context).pushNamed('/b');
                },
              ),
            ),
        '/b': (BuildContext context) => const Text('page b'),
      };

      await tester.pumpWidget(MaterialApp(routes: routes));

      expect(
        Theme.of(tester.element(find.text('push'))).platform,
        debugDefaultTargetPlatformOverride,
      );
      expect(find.byType(CupertinoPageTransition), findsOneWidget);

      await tester.tap(find.text('push'));
      await tester.pumpAndSettle();
      expect(find.text('page b'), findsOneWidget);
      expect(find.byType(CupertinoPageTransition), findsOneWidget);
    },
    variant: const TargetPlatformVariant(<TargetPlatform>{
      TargetPlatform.iOS,
      TargetPlatform.macOS,
    }),
  );

  testWidgets(
    'Default PageTransitionsTheme builds a _ZoomPageTransition for android',
    (WidgetTester tester) async {
      final Map<String, WidgetBuilder> routes = <String, WidgetBuilder>{
        '/':
            (BuildContext context) => Material(
              child: TextButton(
                child: const Text('push'),
                onPressed: () {
                  Navigator.of(context).pushNamed('/b');
                },
              ),
            ),
        '/b': (BuildContext context) => const Text('page b'),
      };

      await tester.pumpWidget(MaterialApp(routes: routes));

      Finder findZoomPageTransition() {
        return find.descendant(
          of: find.byType(MaterialApp),
          matching: find.byWidgetPredicate(
            (Widget w) => '${w.runtimeType}' == '_ZoomPageTransition',
          ),
        );
      }

      expect(
        Theme.of(tester.element(find.text('push'))).platform,
        debugDefaultTargetPlatformOverride,
      );
      expect(findZoomPageTransition(), findsOneWidget);

      await tester.tap(find.text('push'));
      await tester.pumpAndSettle();
      expect(find.text('page b'), findsOneWidget);
      expect(findZoomPageTransition(), findsOneWidget);
    },
    variant: TargetPlatformVariant.only(TargetPlatform.android),
  );

  testWidgets(
    'Default background color when FadeForwardsPageTransitionBuilder is used',
    (WidgetTester tester) async {
      final Map<String, WidgetBuilder> routes = <String, WidgetBuilder>{
        '/':
            (BuildContext context) => Material(
              child: TextButton(
                child: const Text('push'),
                onPressed: () {
                  Navigator.of(context).pushNamed('/b');
                },
              ),
            ),
        '/b': (BuildContext context) => const Text('page b'),
      };

      await tester.pumpWidget(
        MaterialApp(
          theme: ThemeData(
            pageTransitionsTheme: const PageTransitionsTheme(
              builders: <TargetPlatform, PageTransitionsBuilder>{
                TargetPlatform.android: FadeForwardsPageTransitionsBuilder(),
              },
            ),
            colorScheme: ThemeData().colorScheme.copyWith(surface: Colors.pink),
          ),
          routes: routes,
        ),
      );

      Finder findFadeForwardsPageTransition() {
        return find.descendant(
          of: find.byType(MaterialApp),
          matching: find.byWidgetPredicate(
            (Widget w) => '${w.runtimeType}' == '_FadeForwardsPageTransition',
          ),
        );
      }

      expect(findFadeForwardsPageTransition(), findsOneWidget);

      await tester.tap(find.text('push'));
      await tester.pump(const Duration(milliseconds: 400));

      final Finder coloredBoxFinder = find.byType(ColoredBox).last;
      expect(coloredBoxFinder, findsOneWidget);
      final ColoredBox coloredBox = tester.widget<ColoredBox>(coloredBoxFinder);
      expect(coloredBox.color, Colors.pink);

      await tester.pumpAndSettle();
      expect(find.text('page b'), findsOneWidget);
      expect(findFadeForwardsPageTransition(), findsOneWidget);
    },
    variant: TargetPlatformVariant.only(TargetPlatform.android),
  );

  testWidgets(
    'Override background color in FadeForwardsPageTransitionBuilder',
    (WidgetTester tester) async {
      final Map<String, WidgetBuilder> routes = <String, WidgetBuilder>{
        '/':
            (BuildContext context) => Material(
              child: TextButton(
                child: const Text('push'),
                onPressed: () {
                  Navigator.of(context).pushNamed('/b');
                },
              ),
            ),
        '/b': (BuildContext context) => const Text('page b'),
      };

      await tester.pumpWidget(
        MaterialApp(
          theme: ThemeData(
            pageTransitionsTheme: const PageTransitionsTheme(
              builders: <TargetPlatform, PageTransitionsBuilder>{
                TargetPlatform.android: FadeForwardsPageTransitionsBuilder(
                  backgroundColor: Colors.lightGreen,
                ),
              },
            ),
            colorScheme: ThemeData().colorScheme.copyWith(surface: Colors.pink),
          ),
          routes: routes,
        ),
      );

      Finder findFadeForwardsPageTransition() {
        return find.descendant(
          of: find.byType(MaterialApp),
          matching: find.byWidgetPredicate(
            (Widget w) => '${w.runtimeType}' == '_FadeForwardsPageTransition',
          ),
        );
      }

      expect(findFadeForwardsPageTransition(), findsOneWidget);

      await tester.tap(find.text('push'));
      await tester.pump(const Duration(milliseconds: 400));

      final Finder coloredBoxFinder = find.byType(ColoredBox).last;
      expect(coloredBoxFinder, findsOneWidget);
      final ColoredBox coloredBox = tester.widget<ColoredBox>(coloredBoxFinder);
      expect(coloredBox.color, Colors.lightGreen);

      await tester.pumpAndSettle();
      expect(find.text('page b'), findsOneWidget);
      expect(findFadeForwardsPageTransition(), findsOneWidget);
    },
    variant: TargetPlatformVariant.only(TargetPlatform.android),
  );

  testWidgets(
    'FadeForwardsPageTransitionBuilder default duration is 800ms',
    (WidgetTester tester) async {
      final Map<String, WidgetBuilder> routes = <String, WidgetBuilder>{
        '/':
            (BuildContext context) => Material(
              child: TextButton(
                child: const Text('push'),
                onPressed: () {
                  Navigator.of(context).pushNamed('/b');
                },
              ),
            ),
        '/b': (BuildContext context) => const Text('page b'),
      };

      await tester.pumpWidget(
        MaterialApp(
          theme: ThemeData(
            pageTransitionsTheme: const PageTransitionsTheme(
              builders: <TargetPlatform, PageTransitionsBuilder>{
                TargetPlatform.android: FadeForwardsPageTransitionsBuilder(),
              },
            ),
          ),
          routes: routes,
        ),
      );

      Finder findFadeForwardsPageTransition() {
        return find.descendant(
          of: find.byType(MaterialApp),
          matching: find.byWidgetPredicate(
            (Widget w) => '${w.runtimeType}' == '_FadeForwardsPageTransition',
          ),
        );
      }

      expect(findFadeForwardsPageTransition(), findsOneWidget);

      await tester.tap(find.text('push'));
      await tester.pump(const Duration(milliseconds: 799));
      expect(find.text('page b'), findsNothing);
      ColoredBox coloredBox = tester.widget(find.byType(ColoredBox).last);
      expect(
        coloredBox.color,
        isNot(Colors.transparent),
      ); // Color is not transparent during animation.

      await tester.pump(const Duration(milliseconds: 801));
      expect(find.text('page b'), findsOneWidget);
      coloredBox = tester.widget(find.byType(ColoredBox).last);
      expect(coloredBox.color, Colors.transparent); // Color is transparent during animation.
    },
    variant: TargetPlatformVariant.only(TargetPlatform.android),
  );

  testWidgets(
    'CupertinoPageTransitionsBuilder default duration is 500ms',
    (WidgetTester tester) async {
      final Map<String, WidgetBuilder> routes = <String, WidgetBuilder>{
        '/':
            (BuildContext context) => Material(
              child: TextButton(
                child: const Text('push'),
                onPressed: () {
                  Navigator.of(context).pushNamed('/b');
                },
              ),
            ),
        '/b': (BuildContext context) => const Text('page b'),
      };

      await tester.pumpWidget(
        MaterialApp(
          theme: ThemeData(
            pageTransitionsTheme: const PageTransitionsTheme(
              builders: <TargetPlatform, PageTransitionsBuilder>{
                TargetPlatform.iOS: CupertinoPageTransitionsBuilder(),
              },
            ),
          ),
          routes: routes,
        ),
      );

      expect(find.byType(CupertinoPageTransition), findsOneWidget);

      await tester.tap(find.text('push'));
      await tester.pump();
      await tester.pump(const Duration(milliseconds: 499));
      expect(tester.hasRunningAnimations, isTrue);

      await tester.pump(const Duration(milliseconds: 10));
      expect(tester.hasRunningAnimations, isFalse);
    },
    variant: TargetPlatformVariant.only(TargetPlatform.iOS),
  );

  testWidgets(
    'Animation duration changes accordingly when page transition builder changes',
    (WidgetTester tester) async {
      Widget buildApp(PageTransitionsBuilder pageTransitionBuilder) {
        return MaterialApp(
          theme: ThemeData(
            pageTransitionsTheme: PageTransitionsTheme(
              builders: <TargetPlatform, PageTransitionsBuilder>{
                TargetPlatform.android: pageTransitionBuilder,
              },
            ),
          ),
          routes: <String, WidgetBuilder>{
            '/':
                (BuildContext context) => Material(
                  child: TextButton(
                    child: const Text('push'),
                    onPressed: () {
                      Navigator.of(context).pushNamed('/b');
                    },
                  ),
                ),
            '/b':
                (BuildContext context) => Material(
                  child: Column(
                    mainAxisAlignment: MainAxisAlignment.center,
                    children: <Widget>[
                      TextButton(
                        child: const Text('pop'),
                        onPressed: () {
                          Navigator.of(context).pop();
                        },
                      ),
                      const Text('page b'),
                    ],
                  ),
                ),
          },
        );
      }

      await tester.pumpWidget(buildApp(const FadeForwardsPageTransitionsBuilder()));

      Finder findFadeForwardsPageTransition() {
        return find.descendant(
          of: find.byType(MaterialApp),
          matching: find.byWidgetPredicate(
            (Widget w) => '${w.runtimeType}' == '_FadeForwardsPageTransition',
          ),
        );
      }

      expect(findFadeForwardsPageTransition(), findsOneWidget);

      await tester.tap(find.text('push'));
      await tester.pump(const Duration(milliseconds: 799));
      expect(find.text('page b'), findsNothing);
      ColoredBox coloredBox = tester.widget(find.byType(ColoredBox).last);
      expect(
        coloredBox.color,
        isNot(Colors.transparent),
      ); // The color is not transparent during animation.

      await tester.pump(const Duration(milliseconds: 801));
      expect(find.text('page b'), findsOneWidget);
      coloredBox = tester.widget(find.byType(ColoredBox).last);
      expect(coloredBox.color, Colors.transparent); // The color is transparent during animation.

      await tester.pumpWidget(buildApp(const FadeUpwardsPageTransitionsBuilder()));
      await tester.pumpAndSettle();
      expect(
        find.descendant(
          of: find.byType(MaterialApp),
          matching: find.byWidgetPredicate(
            (Widget w) => '${w.runtimeType}' == '_FadeUpwardsPageTransition',
          ),
        ),
        findsOneWidget,
      );
      await tester.tap(find.text('pop'));
      await tester.pump(const Duration(milliseconds: 299));
      expect(find.text('page b'), findsOneWidget);
      expect(
        find.byType(ColoredBox),
        findsNothing,
      ); // ColoredBox doesn't exist in FadeUpwardsPageTransition.

      await tester.pump(const Duration(milliseconds: 301));
      expect(find.text('page b'), findsNothing);
      expect(find.text('push'), findsOneWidget); // The first page
      expect(find.byType(ColoredBox), findsNothing);
    },
    variant: TargetPlatformVariant.only(TargetPlatform.android),
  );

  testWidgets(
    'PageTransitionsTheme override builds a _OpenUpwardsPageTransition',
    (WidgetTester tester) async {
      final Map<String, WidgetBuilder> routes = <String, WidgetBuilder>{
        '/':
            (BuildContext context) => Material(
              child: TextButton(
                child: const Text('push'),
                onPressed: () {
                  Navigator.of(context).pushNamed('/b');
                },
              ),
            ),
        '/b': (BuildContext context) => const Text('page b'),
      };

      await tester.pumpWidget(
        MaterialApp(
          theme: ThemeData(
            pageTransitionsTheme: const PageTransitionsTheme(
              builders: <TargetPlatform, PageTransitionsBuilder>{
                TargetPlatform.android:
                    OpenUpwardsPageTransitionsBuilder(), // creates a _OpenUpwardsPageTransition
              },
            ),
          ),
          routes: routes,
        ),
      );

      Finder findOpenUpwardsPageTransition() {
        return find.descendant(
          of: find.byType(MaterialApp),
          matching: find.byWidgetPredicate(
            (Widget w) => '${w.runtimeType}' == '_OpenUpwardsPageTransition',
          ),
        );
      }

      expect(
        Theme.of(tester.element(find.text('push'))).platform,
        debugDefaultTargetPlatformOverride,
      );
      expect(findOpenUpwardsPageTransition(), findsOneWidget);

      await tester.tap(find.text('push'));
      await tester.pumpAndSettle();
      expect(find.text('page b'), findsOneWidget);
      expect(findOpenUpwardsPageTransition(), findsOneWidget);
    },
    variant: TargetPlatformVariant.only(TargetPlatform.android),
  );

  testWidgets(
    'PageTransitionsTheme override builds a CupertinoPageTransition on android',
    (WidgetTester tester) async {
      final Map<String, WidgetBuilder> routes = <String, WidgetBuilder>{
        '/':
            (BuildContext context) => Material(
              child: TextButton(
                child: const Text('push'),
                onPressed: () {
                  Navigator.of(context).pushNamed('/b');
                },
              ),
            ),
        '/b': (BuildContext context) => const Text('page b'),
      };

      await tester.pumpWidget(
        MaterialApp(
          theme: ThemeData(
            pageTransitionsTheme: const PageTransitionsTheme(
              builders: <TargetPlatform, PageTransitionsBuilder>{
                TargetPlatform.android: CupertinoPageTransitionsBuilder(),
              },
            ),
          ),
          routes: routes,
        ),
      );

      expect(
        Theme.of(tester.element(find.text('push'))).platform,
        debugDefaultTargetPlatformOverride,
      );
      expect(find.byType(CupertinoPageTransition), findsOneWidget);

      await tester.tap(find.text('push'));
      await tester.pumpAndSettle();
      expect(find.text('page b'), findsOneWidget);
      expect(find.byType(CupertinoPageTransition), findsOneWidget);
    },
    variant: TargetPlatformVariant.only(TargetPlatform.android),
  );

  testWidgets(
    'CupertinoPageTransition on android does not block gestures on backswipe',
    (WidgetTester tester) async {
      final Map<String, WidgetBuilder> routes = <String, WidgetBuilder>{
        '/':
            (BuildContext context) => Material(
              child: TextButton(
                child: const Text('push'),
                onPressed: () {
                  Navigator.of(context).pushNamed('/b');
                },
              ),
            ),
        '/b': (BuildContext context) => const Text('page b'),
      };

      await tester.pumpWidget(
        MaterialApp(
          theme: ThemeData(
            pageTransitionsTheme: const PageTransitionsTheme(
              builders: <TargetPlatform, PageTransitionsBuilder>{
                TargetPlatform.android: CupertinoPageTransitionsBuilder(),
              },
            ),
          ),
          routes: routes,
        ),
      );

      expect(
        Theme.of(tester.element(find.text('push'))).platform,
        debugDefaultTargetPlatformOverride,
      );
      expect(find.byType(CupertinoPageTransition), findsOneWidget);

      await tester.tap(find.text('push'));
      await tester.pumpAndSettle();
      expect(find.text('page b'), findsOneWidget);
      expect(find.byType(CupertinoPageTransition), findsOneWidget);

      await tester.pumpAndSettle(const Duration(minutes: 1));

      final TestGesture gesture = await tester.startGesture(const Offset(5.0, 100.0));
      await gesture.moveBy(const Offset(400.0, 0.0));
      await gesture.up();
      await tester.pump();

      await tester.pumpAndSettle(const Duration(minutes: 1));

      expect(find.text('push'), findsOneWidget);
      await tester.tap(find.text('push'));
      await tester.pumpAndSettle();
      expect(find.text('page b'), findsOneWidget);
    },
    variant: TargetPlatformVariant.only(TargetPlatform.android),
  );

  testWidgets(
    'PageTransitionsTheme override builds a _FadeUpwardsTransition',
    (WidgetTester tester) async {
      final Map<String, WidgetBuilder> routes = <String, WidgetBuilder>{
        '/':
            (BuildContext context) => Material(
              child: TextButton(
                child: const Text('push'),
                onPressed: () {
                  Navigator.of(context).pushNamed('/b');
                },
              ),
            ),
        '/b': (BuildContext context) => const Text('page b'),
      };

      await tester.pumpWidget(
        MaterialApp(
          theme: ThemeData(
            pageTransitionsTheme: const PageTransitionsTheme(
              builders: <TargetPlatform, PageTransitionsBuilder>{
                TargetPlatform.android:
                    FadeUpwardsPageTransitionsBuilder(), // creates a _FadeUpwardsTransition
              },
            ),
          ),
          routes: routes,
        ),
      );

      Finder findFadeUpwardsPageTransition() {
        return find.descendant(
          of: find.byType(MaterialApp),
          matching: find.byWidgetPredicate(
            (Widget w) => '${w.runtimeType}' == '_FadeUpwardsPageTransition',
          ),
        );
      }

      expect(
        Theme.of(tester.element(find.text('push'))).platform,
        debugDefaultTargetPlatformOverride,
      );
      expect(findFadeUpwardsPageTransition(), findsOneWidget);

      await tester.tap(find.text('push'));
      await tester.pumpAndSettle();
      expect(find.text('page b'), findsOneWidget);
      expect(findFadeUpwardsPageTransition(), findsOneWidget);
    },
    variant: TargetPlatformVariant.only(TargetPlatform.android),
  );

  Widget boilerplate({
    required bool themeAllowSnapshotting,
    bool secondRouteAllowSnapshotting = true,
  }) {
    return MaterialApp(
      theme: ThemeData(
        useMaterial3: true,
        pageTransitionsTheme: PageTransitionsTheme(
          builders: <TargetPlatform, PageTransitionsBuilder>{
            TargetPlatform.android: ZoomPageTransitionsBuilder(
              allowSnapshotting: themeAllowSnapshotting,
            ),
          },
        ),
      ),
      onGenerateRoute: (RouteSettings settings) {
        if (settings.name == '/') {
          return MaterialPageRoute<Widget>(builder: (_) => const Material(child: Text('Page 1')));
        }
        return MaterialPageRoute<Widget>(
          builder: (_) => const Material(child: Text('Page 2')),
          allowSnapshotting: secondRouteAllowSnapshotting,
        );
      },
    );
  }

  bool isTransitioningWithSnapshotting(WidgetTester tester, Finder of) {
    final Iterable<Layer> layers = tester.layerListOf(
      find.ancestor(of: of, matching: find.byType(SnapshotWidget)).first,
    );
    final bool hasOneOpacityLayer = layers.whereType<OpacityLayer>().length == 1;
    final bool hasOneTransformLayer = layers.whereType<TransformLayer>().length == 1;
    // When snapshotting is on, the OpacityLayer and TransformLayer will not be
    // applied directly.
    return !(hasOneOpacityLayer && hasOneTransformLayer);
  }

  testWidgets(
    'ZoomPageTransitionsBuilder default route snapshotting behavior',
    (WidgetTester tester) async {
      await tester.pumpWidget(boilerplate(themeAllowSnapshotting: true));

      final Finder page1 = find.text('Page 1');
      final Finder page2 = find.text('Page 2');

      // Transitioning from page 1 to page 2.
      tester.state<NavigatorState>(find.byType(Navigator)).pushNamed('/2');
      await tester.pump();
      await tester.pump(const Duration(milliseconds: 50));

      // Exiting route should be snapshotted.
      expect(isTransitioningWithSnapshotting(tester, page1), isTrue);

      // Entering route should be snapshotted.
      expect(isTransitioningWithSnapshotting(tester, page2), isTrue);

      await tester.pumpAndSettle();

      // Transitioning back from page 2 to page 1.
      tester.state<NavigatorState>(find.byType(Navigator)).pop();
      await tester.pump();
      await tester.pump(const Duration(milliseconds: 50));

      // Exiting route should be snapshotted.
      expect(isTransitioningWithSnapshotting(tester, page2), isTrue);

      // Entering route should be snapshotted.
      expect(isTransitioningWithSnapshotting(tester, page1), isTrue);
    },
    variant: TargetPlatformVariant.only(TargetPlatform.android),
    skip: kIsWeb, // [intended] rasterization is not used on the web.
  );

  testWidgets(
    'ZoomPageTransitionsBuilder.allowSnapshotting can disable route snapshotting',
    (WidgetTester tester) async {
      await tester.pumpWidget(boilerplate(themeAllowSnapshotting: false));

      final Finder page1 = find.text('Page 1');
      final Finder page2 = find.text('Page 2');

      // Transitioning from page 1 to page 2.
      tester.state<NavigatorState>(find.byType(Navigator)).pushNamed('/2');
      await tester.pump();
      await tester.pump(const Duration(milliseconds: 50));

      // Exiting route should not be snapshotted.
      expect(isTransitioningWithSnapshotting(tester, page1), isFalse);

      // Entering route should not be snapshotted.
      expect(isTransitioningWithSnapshotting(tester, page2), isFalse);

      await tester.pumpAndSettle();

      // Transitioning back from page 2 to page 1.
      tester.state<NavigatorState>(find.byType(Navigator)).pop();
      await tester.pump();
      await tester.pump(const Duration(milliseconds: 50));

      // Exiting route should not be snapshotted.
      expect(isTransitioningWithSnapshotting(tester, page2), isFalse);

      // Entering route should not be snapshotted.
      expect(isTransitioningWithSnapshotting(tester, page1), isFalse);
    },
    variant: TargetPlatformVariant.only(TargetPlatform.android),
    skip: kIsWeb, // [intended] rasterization is not used on the web.
  );

  testWidgets(
    'Setting PageRoute.allowSnapshotting to false overrides ZoomPageTransitionsBuilder.allowSnapshotting = true',
    (WidgetTester tester) async {
      await tester.pumpWidget(
        boilerplate(themeAllowSnapshotting: true, secondRouteAllowSnapshotting: false),
      );

      final Finder page1 = find.text('Page 1');
      final Finder page2 = find.text('Page 2');

      // Transitioning from page 1 to page 2.
      tester.state<NavigatorState>(find.byType(Navigator)).pushNamed('/2');
      await tester.pump();
      await tester.pump(const Duration(milliseconds: 50));

      // First route should be snapshotted.
      expect(isTransitioningWithSnapshotting(tester, page1), isTrue);

      // Second route should not be snapshotted.
      expect(isTransitioningWithSnapshotting(tester, page2), isFalse);

      await tester.pumpAndSettle();
    },
    variant: TargetPlatformVariant.only(TargetPlatform.android),
    skip: kIsWeb, // [intended] rasterization is not used on the web.
  );

  testWidgets(
    '_ZoomPageTransition only causes child widget built once',
    (WidgetTester tester) async {
      // Regression test for https://github.com/flutter/flutter/issues/58345

      int builtCount = 0;

      final Map<String, WidgetBuilder> routes = <String, WidgetBuilder>{
        '/':
            (BuildContext context) => Material(
              child: TextButton(
                child: const Text('push'),
                onPressed: () {
                  Navigator.of(context).pushNamed('/b');
                },
              ),
            ),
        '/b':
            (BuildContext context) => StatefulBuilder(
              builder: (BuildContext context, StateSetter setState) {
                builtCount++; // Increase [builtCount] each time the widget build
                return TextButton(
                  child: const Text('pop'),
                  onPressed: () {
                    Navigator.pop(context);
                  },
                );
              },
            ),
      };

      await tester.pumpWidget(
        MaterialApp(
          theme: ThemeData(
            pageTransitionsTheme: const PageTransitionsTheme(
              builders: <TargetPlatform, PageTransitionsBuilder>{
                TargetPlatform.android:
                    ZoomPageTransitionsBuilder(), // creates a _ZoomPageTransition
              },
            ),
          ),
          routes: routes,
        ),
      );

      // No matter push or pop was called, the child widget should built only once.
      await tester.tap(find.text('push'));
      await tester.pumpAndSettle();
      expect(builtCount, 1);

      await tester.tap(find.text('pop'));
      await tester.pumpAndSettle();
      expect(builtCount, 1);
    },
    variant: TargetPlatformVariant.only(TargetPlatform.android),
  );

  testWidgets(
    'predictive back gestures pop the route on all platforms regardless of whether their transition handles predictive back',
    (WidgetTester tester) async {
      final Map<String, WidgetBuilder> routes = <String, WidgetBuilder>{
        '/':
            (BuildContext context) => Material(
              child: TextButton(
                child: const Text('push'),
                onPressed: () {
                  Navigator.of(context).pushNamed('/b');
                },
              ),
            ),
        '/b': (BuildContext context) => const Text('page b'),
      };

      await tester.pumpWidget(MaterialApp(routes: routes));

      expect(find.text('push'), findsOneWidget);
      expect(find.text('page b'), findsNothing);

      await tester.tap(find.text('push'));
      await tester.pumpAndSettle();

      expect(find.text('push'), findsNothing);
      expect(find.text('page b'), findsOneWidget);

      // Start a system pop gesture.
      final ByteData startMessage = const StandardMethodCodec().encodeMethodCall(
        const MethodCall('startBackGesture', <String, dynamic>{
          'touchOffset': <double>[5.0, 300.0],
          'progress': 0.0,
          'swipeEdge': 0, // left
        }),
      );
      await binding.defaultBinaryMessenger.handlePlatformMessage(
        'flutter/backgesture',
        startMessage,
        (ByteData? _) {},
      );
      await tester.pump();

      expect(find.text('push'), findsNothing);
      expect(find.text('page b'), findsOneWidget);

      // Drag the system back gesture far enough to commit.
      final ByteData updateMessage = const StandardMethodCodec().encodeMethodCall(
        const MethodCall('updateBackGestureProgress', <String, dynamic>{
          'x': 100.0,
          'y': 300.0,
          'progress': 0.35,
          'swipeEdge': 0, // left
        }),
      );
      await binding.defaultBinaryMessenger.handlePlatformMessage(
        'flutter/backgesture',
        updateMessage,
        (ByteData? _) {},
      );
      await tester.pumpAndSettle();

      expect(find.text('push'), findsNothing);
      expect(find.text('page b'), findsOneWidget);

      // Commit the system back gesture.
      final ByteData commitMessage = const StandardMethodCodec().encodeMethodCall(
        const MethodCall('commitBackGesture'),
      );
      await binding.defaultBinaryMessenger.handlePlatformMessage(
        'flutter/backgesture',
        commitMessage,
        (ByteData? _) {},
      );
      await tester.pumpAndSettle();

      expect(find.text('push'), findsOneWidget);
      expect(find.text('page b'), findsNothing);
    },
    variant: TargetPlatformVariant.all(),
  );

  testWidgets(
    'ZoomPageTransitionsBuilder uses theme color during transition effects',
    (WidgetTester tester) async {
      // Color that is being tested for presence.
      const Color themeTestSurfaceColor = Color.fromARGB(255, 195, 255, 0);

      final Map<String, WidgetBuilder> routes = <String, WidgetBuilder>{
        '/':
            (BuildContext context) => Material(
              child: Scaffold(
                appBar: AppBar(title: const Text('Home Page')),
                body: Center(
                  child: Column(
                    mainAxisAlignment: MainAxisAlignment.center,
                    children: <Widget>[
                      ElevatedButton(
                        onPressed: () {
                          Navigator.pushNamed(context, '/scaffolded');
                        },
                        child: const Text('Route with scaffold!'),
                      ),
                      ElevatedButton(
                        onPressed: () {
                          Navigator.pushNamed(context, '/not-scaffolded');
                        },
                        child: const Text('Route with NO scaffold!'),
                      ),
                    ],
                  ),
                ),
              ),
            ),
        '/scaffolded':
            (BuildContext context) => Material(
              child: Scaffold(
                appBar: AppBar(title: const Text('Scaffolded Page')),
                body: Center(
                  child: ElevatedButton(
                    onPressed: () {
                      Navigator.pop(context);
                    },
                    child: const Text('Back to home route...'),
                  ),
                ),
              ),
            ),
        '/not-scaffolded':
            (BuildContext context) => Material(
              child: Center(
                child: ElevatedButton(
                  onPressed: () {
                    Navigator.pop(context);
                  },
                  child: const Text('Back to home route...'),
                ),
              ),
            ),
      };

      await tester.pumpWidget(
        MaterialApp(
          theme: ThemeData(
            colorScheme: ColorScheme.fromSeed(
              seedColor: Colors.blue,
              surface: themeTestSurfaceColor,
            ),
            pageTransitionsTheme: PageTransitionsTheme(
              builders: <TargetPlatform, PageTransitionsBuilder>{
                // Force all platforms to use ZoomPageTransitionsBuilder to test each one.
                for (final TargetPlatform platform in TargetPlatform.values)
                  platform: const ZoomPageTransitionsBuilder(),
              },
            ),
          ),
          routes: routes,
        ),
      );

      // Go to scaffolded page.
      await tester.tap(find.text('Route with scaffold!'));

      // Pump till animation is half-way through.
      await tester.pump();
      await tester.pump(const Duration(milliseconds: 75));

      // Verify that the render box is painting the right color for scaffolded pages.
      final RenderBox scaffoldedRenderBox = tester.firstRenderObject<RenderBox>(
        find.byType(MaterialApp),
      );
      // Expect the color to be at exactly 12.2% opacity at this time.
      expect(scaffoldedRenderBox, paints..rect(color: themeTestSurfaceColor.withOpacity(0.122)));

      await tester.pumpAndSettle();

      // Go back home and then go to non-scaffolded page.
      await tester.tap(find.text('Back to home route...'));
      await tester.pumpAndSettle();
      await tester.tap(find.text('Route with NO scaffold!'));

      // Pump till animation is half-way through.
      await tester.pump();
      await tester.pump(const Duration(milliseconds: 125));

      // Verify that the render box is painting the right color for non-scaffolded pages.
      final RenderBox nonScaffoldedRenderBox = tester.firstRenderObject<RenderBox>(
        find.byType(MaterialApp),
      );
      // Expect the color to be at exactly 59.6% opacity at this time.
      expect(nonScaffoldedRenderBox, paints..rect(color: themeTestSurfaceColor.withOpacity(0.596)));

      await tester.pumpAndSettle();

      // Verify that the transition successfully completed.
      expect(find.text('Back to home route...'), findsOneWidget);
    },
    variant: TargetPlatformVariant.all(),
  );

  testWidgets(
    'ZoomPageTransitionsBuilder uses developer-provided color during transition effects if provided',
    (WidgetTester tester) async {
      // Color that is being tested for presence.
      const Color testSurfaceColor = Colors.red;

      final Map<String, WidgetBuilder> routes = <String, WidgetBuilder>{
        '/':
            (BuildContext context) => Material(
              child: Scaffold(
                appBar: AppBar(title: const Text('Home Page')),
                body: Center(
                  child: Column(
                    mainAxisAlignment: MainAxisAlignment.center,
                    children: <Widget>[
                      ElevatedButton(
                        onPressed: () {
                          Navigator.pushNamed(context, '/scaffolded');
                        },
                        child: const Text('Route with scaffold!'),
                      ),
                      ElevatedButton(
                        onPressed: () {
                          Navigator.pushNamed(context, '/not-scaffolded');
                        },
                        child: const Text('Route with NO scaffold!'),
                      ),
                    ],
                  ),
                ),
              ),
            ),
        '/scaffolded':
            (BuildContext context) => Material(
              child: Scaffold(
                appBar: AppBar(title: const Text('Scaffolded Page')),
                body: Center(
                  child: ElevatedButton(
                    onPressed: () {
                      Navigator.pop(context);
                    },
                    child: const Text('Back to home route...'),
                  ),
                ),
              ),
            ),
        '/not-scaffolded':
            (BuildContext context) => Material(
              child: Center(
                child: ElevatedButton(
                  onPressed: () {
                    Navigator.pop(context);
                  },
                  child: const Text('Back to home route...'),
                ),
              ),
            ),
      };

      await tester.pumpWidget(
        MaterialApp(
          theme: ThemeData(
            colorScheme: ColorScheme.fromSeed(seedColor: Colors.blue, surface: Colors.blue),
            pageTransitionsTheme: PageTransitionsTheme(
              builders: <TargetPlatform, PageTransitionsBuilder>{
                // Force all platforms to use ZoomPageTransitionsBuilder to test each one.
                for (final TargetPlatform platform in TargetPlatform.values)
                  platform: const ZoomPageTransitionsBuilder(backgroundColor: testSurfaceColor),
              },
            ),
          ),
          routes: routes,
        ),
      );

      // Go to scaffolded page.
      await tester.tap(find.text('Route with scaffold!'));

      // Pump till animation is half-way through.
      await tester.pump();
      await tester.pump(const Duration(milliseconds: 75));

      // Verify that the render box is painting the right color for scaffolded pages.
      final RenderBox scaffoldedRenderBox = tester.firstRenderObject<RenderBox>(
        find.byType(MaterialApp),
      );
      // Expect the color to be at exactly 12.2% opacity at this time.
      expect(scaffoldedRenderBox, paints..rect(color: testSurfaceColor.withOpacity(0.122)));

      await tester.pumpAndSettle();

      // Go back home and then go to non-scaffolded page.
      await tester.tap(find.text('Back to home route...'));
      await tester.pumpAndSettle();
      await tester.tap(find.text('Route with NO scaffold!'));

      // Pump till animation is half-way through.
      await tester.pump();
      await tester.pump(const Duration(milliseconds: 125));

      // Verify that the render box is painting the right color for non-scaffolded pages.
      final RenderBox nonScaffoldedRenderBox = tester.firstRenderObject<RenderBox>(
        find.byType(MaterialApp),
      );
      // Expect the color to be at exactly 59.6% opacity at this time.
      expect(nonScaffoldedRenderBox, paints..rect(color: testSurfaceColor.withOpacity(0.596)));

      await tester.pumpAndSettle();

      // Verify that the transition successfully completed.
      expect(find.text('Back to home route...'), findsOneWidget);
    },
    variant: TargetPlatformVariant.all(),
  );
}
