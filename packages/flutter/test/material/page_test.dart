// Copyright 2014 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

@Tags(<String>['reduced-test-set'])
library;

import 'dart:ui' as ui;

import 'package:flutter/cupertino.dart' show CupertinoPageRoute;
import 'package:flutter/foundation.dart';
import 'package:flutter/material.dart';
import 'package:flutter/rendering.dart';
import 'package:flutter/services.dart';
import 'package:flutter_test/flutter_test.dart';

void main() {
  testWidgets(
    'test page transition (_FadeUpwardsPageTransition)',
    (WidgetTester tester) async {
      await tester.pumpWidget(
        MaterialApp(
          home: const Material(child: Text('Page 1')),
          theme: ThemeData(
            pageTransitionsTheme: const PageTransitionsTheme(
              builders: <TargetPlatform, PageTransitionsBuilder>{
                TargetPlatform.android: FadeUpwardsPageTransitionsBuilder(),
              },
            ),
          ),
          routes: <String, WidgetBuilder>{
            '/next': (BuildContext context) {
              return const Material(child: Text('Page 2'));
            },
          },
        ),
      );

      final Offset widget1TopLeft = tester.getTopLeft(find.text('Page 1'));

      tester.state<NavigatorState>(find.byType(Navigator)).pushNamed('/next');
      await tester.pump();
      await tester.pump(const Duration(milliseconds: 1));

      FadeTransition widget2Opacity =
          tester.element(find.text('Page 2')).findAncestorWidgetOfExactType<FadeTransition>()!;
      Offset widget2TopLeft = tester.getTopLeft(find.text('Page 2'));
      final Size widget2Size = tester.getSize(find.text('Page 2'));

      // Android transition is vertical only.
      expect(widget1TopLeft.dx == widget2TopLeft.dx, true);
      // Page 1 is above page 2 mid-transition.
      expect(widget1TopLeft.dy < widget2TopLeft.dy, true);
      // Animation begins 3/4 of the way up the page.
      expect(widget2TopLeft.dy < widget2Size.height / 4.0, true);
      // Animation starts with page 2 being near transparent.
      expect(widget2Opacity.opacity.value < 0.01, true);

      await tester.pump(const Duration(milliseconds: 300));

      // Page 2 covers page 1.
      expect(find.text('Page 1'), findsNothing);
      expect(find.text('Page 2'), isOnstage);

      tester.state<NavigatorState>(find.byType(Navigator)).pop();
      await tester.pump();
      await tester.pump(const Duration(milliseconds: 1));

      widget2Opacity =
          tester.element(find.text('Page 2')).findAncestorWidgetOfExactType<FadeTransition>()!;
      widget2TopLeft = tester.getTopLeft(find.text('Page 2'));

      // Page 2 starts to move down.
      expect(widget1TopLeft.dy < widget2TopLeft.dy, true);
      // Page 2 starts to lose opacity.
      expect(widget2Opacity.opacity.value < 1.0, true);

      await tester.pump(const Duration(milliseconds: 300));

      expect(find.text('Page 1'), isOnstage);
      expect(find.text('Page 2'), findsNothing);
    },
    variant: TargetPlatformVariant.only(TargetPlatform.android),
  );

  testWidgets(
    'test page transition (CupertinoPageTransition)',
    (WidgetTester tester) async {
      final Key page2Key = UniqueKey();
      await tester.pumpWidget(
        MaterialApp(
          home: const Material(child: Text('Page 1')),
          routes: <String, WidgetBuilder>{
            '/next': (BuildContext context) {
              return Material(key: page2Key, child: const Text('Page 2'));
            },
          },
        ),
      );

      final Offset widget1InitialTopLeft = tester.getTopLeft(find.text('Page 1'));

      tester.state<NavigatorState>(find.byType(Navigator)).pushNamed('/next');

      await tester.pump();
      await tester.pump(const Duration(milliseconds: 150));

      Offset widget1TransientTopLeft = tester.getTopLeft(find.text('Page 1'));
      Offset widget2TopLeft = tester.getTopLeft(find.text('Page 2'));
      final RenderDecoratedBox box =
          tester
              .element(find.byKey(page2Key))
              .findAncestorRenderObjectOfType<RenderDecoratedBox>()!;

      // Page 1 is moving to the left.
      expect(widget1TransientTopLeft.dx < widget1InitialTopLeft.dx, true);
      // Page 1 isn't moving vertically.
      expect(widget1TransientTopLeft.dy == widget1InitialTopLeft.dy, true);
      // iOS transition is horizontal only.
      expect(widget1InitialTopLeft.dy == widget2TopLeft.dy, true);
      // Page 2 is coming in from the right.
      expect(widget2TopLeft.dx > widget1InitialTopLeft.dx, true);
      // As explained in _CupertinoEdgeShadowPainter.paint the shadow is drawn
      // as a bunch of rects. The rects are covering an area to the left of
      // where the page 2 box is and a width of 5% of the page 2 box width.
      // `paints` tests relative to the painter's given canvas
      // rather than relative to the screen so assert that the shadow starts at
      // offset.dx = 0.
      final PaintPattern paintsShadow = paints;
      for (int i = 0; i < 0.05 * 800; i += 1) {
        paintsShadow.rect(rect: Rect.fromLTWH(-i.toDouble() - 1.0, 0.0, 1.0, 600));
      }
      expect(box, paintsShadow);

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
      expect(widget1TransientTopLeft.dx < widget1InitialTopLeft.dx, true);
      // Page 1 isn't moving vertically.
      expect(widget1TransientTopLeft.dy == widget1InitialTopLeft.dy, true);
      // iOS transition is horizontal only.
      expect(widget1InitialTopLeft.dy == widget2TopLeft.dy, true);
      // Page 2 is leaving towards the right.
      expect(widget2TopLeft.dx > widget1InitialTopLeft.dx, true);

      await tester.pumpAndSettle();

      expect(find.text('Page 1'), isOnstage);
      expect(find.text('Page 2'), findsNothing);

      widget1TransientTopLeft = tester.getTopLeft(find.text('Page 1'));

      // Page 1 is back where it started.
      expect(widget1InitialTopLeft == widget1TransientTopLeft, true);
    },
    variant: const TargetPlatformVariant(<TargetPlatform>{
      TargetPlatform.iOS,
      TargetPlatform.macOS,
    }),
  );

  testWidgets(
    'test page transition (_ZoomPageTransition) without rasterization',
    (WidgetTester tester) async {
      Iterable<Layer> findLayers(Finder of) {
        return tester.layerListOf(
          find.ancestor(of: of, matching: find.byType(SnapshotWidget)).first,
        );
      }

      OpacityLayer findForwardFadeTransition(Finder of) {
        return findLayers(of).whereType<OpacityLayer>().first;
      }

      TransformLayer findForwardScaleTransition(Finder of) {
        return findLayers(of).whereType<TransformLayer>().first;
      }

      await tester.pumpWidget(
        MaterialApp(
          onGenerateRoute: (RouteSettings settings) {
            return MaterialPageRoute<void>(
              allowSnapshotting: false,
              builder: (BuildContext context) {
                if (settings.name == '/') {
                  return const Material(child: Text('Page 1'));
                }
                return const Material(child: Text('Page 2'));
              },
            );
          },
        ),
      );

      tester.state<NavigatorState>(find.byType(Navigator)).pushNamed('/next');
      await tester.pump();
      await tester.pump(const Duration(milliseconds: 50));

      TransformLayer widget1Scale = findForwardScaleTransition(find.text('Page 1'));
      TransformLayer widget2Scale = findForwardScaleTransition(find.text('Page 2'));
      OpacityLayer widget2Opacity = findForwardFadeTransition(find.text('Page 2'));

      double getScale(TransformLayer layer) {
        return layer.transform!.storage[0];
      }

      // Page 1 is enlarging, starts from 1.0.
      expect(getScale(widget1Scale), greaterThan(1.0));
      // Page 2 is enlarging from the value less than 1.0.
      expect(getScale(widget2Scale), lessThan(1.0));
      // Page 2 is becoming none transparent.
      expect(widget2Opacity.alpha, lessThan(255));

      await tester.pump(const Duration(milliseconds: 250));
      await tester.pump(const Duration(milliseconds: 1));

      // Page 2 covers page 1.
      expect(find.text('Page 1'), findsNothing);
      expect(find.text('Page 2'), isOnstage);

      tester.state<NavigatorState>(find.byType(Navigator)).pop();
      await tester.pump();
      await tester.pump(const Duration(milliseconds: 100));

      widget1Scale = findForwardScaleTransition(find.text('Page 1'));
      widget2Scale = findForwardScaleTransition(find.text('Page 2'));
      widget2Opacity = findForwardFadeTransition(find.text('Page 2'));

      // Page 1 is narrowing down, but still larger than 1.0.
      expect(getScale(widget1Scale), greaterThan(1.0));
      // Page 2 is smaller than 1.0.
      expect(getScale(widget2Scale), lessThan(1.0));
      // Page 2 is becoming transparent.
      expect(widget2Opacity.alpha, lessThan(255));

      await tester.pump(const Duration(milliseconds: 200));
      await tester.pump(const Duration(milliseconds: 1));

      expect(find.text('Page 1'), isOnstage);
      expect(find.text('Page 2'), findsNothing);
    },
    variant: TargetPlatformVariant.only(TargetPlatform.android),
  );

  testWidgets(
    'Material2 - test page transition (_ZoomPageTransition) with rasterization re-rasterizes when view insets change',
    (WidgetTester tester) async {
      addTearDown(tester.view.reset);
      tester.view.physicalSize = const Size(1000, 1000);
      tester.view.viewInsets = FakeViewPadding.zero;

      // Intentionally use nested scaffolds to simulate the view insets being
      // consumed.
      final Key key = GlobalKey();
      await tester.pumpWidget(
        RepaintBoundary(
          key: key,
          child: MaterialApp(
            theme: ThemeData(useMaterial3: false),
            onGenerateRoute: (RouteSettings settings) {
              return MaterialPageRoute<void>(
                builder: (BuildContext context) {
                  return const Scaffold(body: Scaffold(body: Material(child: SizedBox.shrink())));
                },
              );
            },
          ),
        ),
      );

      tester.state<NavigatorState>(find.byType(Navigator)).pushNamed('/next');
      await tester.pump();
      await tester.pump(const Duration(milliseconds: 50));

      await expectLater(find.byKey(key), matchesGoldenFile('m2_zoom_page_transition.small.png'));

      // Change the view insets.
      tester.view.viewInsets = const FakeViewPadding(bottom: 500);

      await tester.pump();
      await tester.pump(const Duration(milliseconds: 50));

      await expectLater(find.byKey(key), matchesGoldenFile('m2_zoom_page_transition.big.png'));
    },
    variant: TargetPlatformVariant.only(TargetPlatform.android),
    skip: kIsWeb, // [intended] rasterization is not used on the web.
  );

  testWidgets(
    'Material3 - test page transition (_ZoomPageTransition) with rasterization re-rasterizes when view insets change',
    (WidgetTester tester) async {
      addTearDown(tester.view.reset);
      tester.view.physicalSize = const Size(1000, 1000);
      tester.view.viewInsets = FakeViewPadding.zero;

      // Intentionally use nested scaffolds to simulate the view insets being
      // consumed.
      final Key key = GlobalKey();
      await tester.pumpWidget(
        RepaintBoundary(
          key: key,
          child: MaterialApp(
            debugShowCheckedModeBanner: false, // https://github.com/flutter/flutter/issues/143616
            onGenerateRoute: (RouteSettings settings) {
              return MaterialPageRoute<void>(
                builder: (BuildContext context) {
                  return const Scaffold(body: Scaffold(body: Material(child: SizedBox.shrink())));
                },
              );
            },
          ),
        ),
      );

      tester.state<NavigatorState>(find.byType(Navigator)).pushNamed('/next');
      await tester.pump();
      await tester.pump(const Duration(milliseconds: 50));

      await expectLater(find.byKey(key), matchesGoldenFile('m3_zoom_page_transition.small.png'));

      // Change the view insets.
      tester.view.viewInsets = const FakeViewPadding(bottom: 500);

      await tester.pump();
      await tester.pump(const Duration(milliseconds: 50));

      await expectLater(find.byKey(key), matchesGoldenFile('m3_zoom_page_transition.big.png'));
    },
    variant: TargetPlatformVariant.only(TargetPlatform.android),
    skip: kIsWeb, // [intended] rasterization is not used on the web.
  );

  testWidgets(
    'test page transition (_ZoomPageTransition) with rasterization disables snapshotting for enter route',
    (WidgetTester tester) async {
      Iterable<Layer> findLayers(Finder of) {
        return tester.layerListOf(
          find.ancestor(of: of, matching: find.byType(SnapshotWidget)).first,
        );
      }

      bool isTransitioningWithoutSnapshotting(Finder of) {
        // When snapshotting is off, the OpacityLayer and TransformLayer will be
        // applied directly.
        final Iterable<Layer> layers = findLayers(of);
        return layers.whereType<OpacityLayer>().length == 1 &&
            layers.whereType<TransformLayer>().length == 1;
      }

      bool isSnapshotted(Finder of) {
        final Iterable<Layer> layers = findLayers(of);
        // The scrim and the snapshot image are the only two layers.
        return layers.length == 2 &&
            layers.whereType<OffsetLayer>().length == 1 &&
            layers.whereType<PictureLayer>().length == 1;
      }

      await tester.pumpWidget(
        MaterialApp(
          routes: <String, WidgetBuilder>{
            '/1': (_) => const Material(child: Text('Page 1')),
            '/2': (_) => const Material(child: Text('Page 2')),
          },
          initialRoute: '/1',
          builder: (BuildContext context, Widget? child) {
            final ThemeData themeData = Theme.of(context);
            return Theme(
              data: themeData.copyWith(
                pageTransitionsTheme: PageTransitionsTheme(
                  builders: <TargetPlatform, PageTransitionsBuilder>{
                    ...themeData.pageTransitionsTheme.builders,
                    TargetPlatform.android: const ZoomPageTransitionsBuilder(
                      allowEnterRouteSnapshotting: false,
                    ),
                  },
                ),
              ),
              child: Builder(builder: (_) => child!),
            );
          },
        ),
      );

      final Finder page1Finder = find.text('Page 1');
      final Finder page2Finder = find.text('Page 2');

      // Page 1 on top.
      expect(isSnapshotted(page1Finder), isFalse);

      // Transitioning from page 1 to page 2.
      tester.state<NavigatorState>(find.byType(Navigator)).pushNamed('/2');
      await tester.pump();
      await tester.pump(const Duration(milliseconds: 50));

      expect(isSnapshotted(page1Finder), isTrue);
      expect(isTransitioningWithoutSnapshotting(page2Finder), isTrue);

      // Page 2 on top.
      await tester.pumpAndSettle();
      expect(isSnapshotted(page2Finder), isFalse);

      // Transitioning back from page 2 to page 1.
      tester.state<NavigatorState>(find.byType(Navigator)).pop();
      await tester.pump();
      await tester.pump(const Duration(milliseconds: 50));

      expect(isTransitioningWithoutSnapshotting(page1Finder), isTrue);
      expect(isSnapshotted(page2Finder), isTrue);

      // Page 1 on top.
      await tester.pumpAndSettle();
      expect(isSnapshotted(page1Finder), isFalse);
    },
    variant: TargetPlatformVariant.only(TargetPlatform.android),
    skip: kIsWeb, // [intended] rasterization is not used on the web.
  );
  testWidgets(
    'test fullscreen dialog transition',
    (WidgetTester tester) async {
      await tester.pumpWidget(const MaterialApp(home: Material(child: Text('Page 1'))));

      final Offset widget1InitialTopLeft = tester.getTopLeft(find.text('Page 1'));

      tester
          .state<NavigatorState>(find.byType(Navigator))
          .push(
            MaterialPageRoute<void>(
              builder: (BuildContext context) {
                return const Material(child: Text('Page 2'));
              },
              fullscreenDialog: true,
            ),
          );

      await tester.pump();
      await tester.pump(const Duration(milliseconds: 100));

      Offset widget1TransientTopLeft = tester.getTopLeft(find.text('Page 1'));
      Offset widget2TopLeft = tester.getTopLeft(find.text('Page 2'));

      // Page 1 doesn't move.
      expect(widget1TransientTopLeft == widget1InitialTopLeft, true);
      // Fullscreen dialogs transitions vertically only.
      expect(widget1InitialTopLeft.dx == widget2TopLeft.dx, true);
      // Page 2 is coming in from the bottom.
      expect(widget2TopLeft.dy > widget1InitialTopLeft.dy, true);

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
      expect(widget1TransientTopLeft == widget1InitialTopLeft, true);
      // Fullscreen dialogs transitions vertically only.
      expect(widget1InitialTopLeft.dx == widget2TopLeft.dx, true);
      // Page 2 is leaving towards the bottom.
      expect(widget2TopLeft.dy > widget1InitialTopLeft.dy, true);

      await tester.pumpAndSettle();

      expect(find.text('Page 1'), isOnstage);
      expect(find.text('Page 2'), findsNothing);

      widget1TransientTopLeft = tester.getTopLeft(find.text('Page 1'));

      // Page 1 is back where it started.
      expect(widget1InitialTopLeft == widget1TransientTopLeft, true);
    },
    variant: const TargetPlatformVariant(<TargetPlatform>{
      TargetPlatform.iOS,
      TargetPlatform.macOS,
    }),
  );

  testWidgets('test no back gesture on Android', (WidgetTester tester) async {
    await tester.pumpWidget(
      MaterialApp(
        home: const Scaffold(body: Text('Page 1')),
        routes: <String, WidgetBuilder>{
          '/next': (BuildContext context) {
            return const Scaffold(body: Text('Page 2'));
          },
        },
      ),
    );

    tester.state<NavigatorState>(find.byType(Navigator)).pushNamed('/next');
    await tester.pumpAndSettle();

    expect(find.text('Page 1'), findsNothing);
    expect(find.text('Page 2'), isOnstage);

    // Drag from left edge to invoke the gesture.
    final TestGesture gesture = await tester.startGesture(const Offset(5.0, 100.0));
    await gesture.moveBy(const Offset(400.0, 0.0));
    await tester.pump();

    expect(find.text('Page 1'), findsNothing);
    expect(find.text('Page 2'), isOnstage);

    // Page 2 didn't move.
    expect(tester.getTopLeft(find.text('Page 2')), Offset.zero);
  }, variant: TargetPlatformVariant.only(TargetPlatform.android));

  testWidgets(
    'test back gesture',
    (WidgetTester tester) async {
      await tester.pumpWidget(
        MaterialApp(
          home: const Scaffold(body: Text('Page 1')),
          routes: <String, WidgetBuilder>{
            '/next': (BuildContext context) {
              return const Scaffold(body: Text('Page 2'));
            },
          },
        ),
      );

      tester.state<NavigatorState>(find.byType(Navigator)).pushNamed('/next');
      await tester.pumpAndSettle();

      expect(find.text('Page 1'), findsNothing);
      expect(find.text('Page 2'), isOnstage);

      // Drag from left edge to invoke the gesture.
      final TestGesture gesture = await tester.startGesture(const Offset(5.0, 100.0));
      await gesture.moveBy(const Offset(400.0, 0.0));
      await tester.pump();

      // Page 1 is now visible.
      expect(find.text('Page 1'), isOnstage);
      expect(find.text('Page 2'), isOnstage);

      // The route widget position needs to track the finger position very exactly.
      expect(tester.getTopLeft(find.text('Page 2')), const Offset(400.0, 0.0));

      await gesture.moveBy(const Offset(-200.0, 0.0));
      await tester.pump();

      expect(tester.getTopLeft(find.text('Page 2')), const Offset(200.0, 0.0));

      await gesture.moveBy(const Offset(-100.0, 200.0));
      await tester.pump();

      expect(tester.getTopLeft(find.text('Page 2')), const Offset(100.0, 0.0));
    },
    variant: const TargetPlatformVariant(<TargetPlatform>{
      TargetPlatform.iOS,
      TargetPlatform.macOS,
    }),
  );

  testWidgets('back gesture while OS changes', (WidgetTester tester) async {
    final Map<String, WidgetBuilder> routes = <String, WidgetBuilder>{
      '/':
          (BuildContext context) => Material(
            child: TextButton(
              child: const Text('PUSH'),
              onPressed: () {
                Navigator.of(context).pushNamed('/b');
              },
            ),
          ),
      '/b': (BuildContext context) => const Text('HELLO'),
    };
    await tester.pumpWidget(
      MaterialApp(theme: ThemeData(platform: TargetPlatform.iOS), routes: routes),
    );
    await tester.tap(find.text('PUSH'));
    expect(await tester.pumpAndSettle(const Duration(minutes: 1)), 2);
    expect(find.text('PUSH'), findsNothing);
    expect(find.text('HELLO'), findsOneWidget);
    final Offset helloPosition1 = tester.getCenter(find.text('HELLO'));
    final TestGesture gesture = await tester.startGesture(const Offset(2.5, 300.0));
    await tester.pump(const Duration(milliseconds: 20));
    await gesture.moveBy(const Offset(100.0, 0.0));
    expect(find.text('PUSH'), findsNothing);
    expect(find.text('HELLO'), findsOneWidget);
    await tester.pump(const Duration(milliseconds: 20));
    expect(find.text('PUSH'), findsOneWidget);
    expect(find.text('HELLO'), findsOneWidget);
    final Offset helloPosition2 = tester.getCenter(find.text('HELLO'));
    expect(helloPosition1.dx, lessThan(helloPosition2.dx));
    expect(helloPosition1.dy, helloPosition2.dy);
    expect(Theme.of(tester.element(find.text('HELLO'))).platform, TargetPlatform.iOS);
    await tester.pumpWidget(
      MaterialApp(theme: ThemeData(platform: TargetPlatform.android), routes: routes),
    );
    // Now we have to let the theme animation run through.
    // This takes three frames (including the first one above):
    //  1. Start the Theme animation. It's at t=0 so everything else is identical.
    //  2. Start any animations that are informed by the Theme, for example, the
    //     DefaultTextStyle, on the first frame that the theme is not at t=0. In
    //     this case, it's at t=1.0 of the theme animation, so this is also the
    //     frame in which the theme animation ends.
    //  3. End all the other animations.
    expect(await tester.pumpAndSettle(const Duration(minutes: 1)), 2);
    expect(Theme.of(tester.element(find.text('HELLO'))).platform, TargetPlatform.android);
    final Offset helloPosition3 = tester.getCenter(find.text('HELLO'));
    expect(helloPosition3, helloPosition2);
    expect(find.text('PUSH'), findsOneWidget);
    expect(find.text('HELLO'), findsOneWidget);
    await gesture.moveBy(const Offset(100.0, 0.0));
    await tester.pump(const Duration(milliseconds: 20));
    expect(find.text('PUSH'), findsOneWidget);
    expect(find.text('HELLO'), findsOneWidget);
    final Offset helloPosition4 = tester.getCenter(find.text('HELLO'));
    expect(helloPosition3.dx, lessThan(helloPosition4.dx));
    expect(helloPosition3.dy, helloPosition4.dy);
    await gesture.moveBy(const Offset(500.0, 0.0));
    await gesture.up();
    expect(await tester.pumpAndSettle(const Duration(minutes: 1)), 3);
    expect(find.text('PUSH'), findsOneWidget);
    expect(find.text('HELLO'), findsNothing);

    await tester.pumpWidget(
      MaterialApp(theme: ThemeData(platform: TargetPlatform.macOS), routes: routes),
    );
    await tester.tap(find.text('PUSH'));
    expect(await tester.pumpAndSettle(const Duration(minutes: 1)), 2);
    expect(find.text('PUSH'), findsNothing);
    expect(find.text('HELLO'), findsOneWidget);
    final Offset helloPosition5 = tester.getCenter(find.text('HELLO'));
    await gesture.down(const Offset(2.5, 300.0));
    await tester.pump(const Duration(milliseconds: 20));
    await gesture.moveBy(const Offset(100.0, 0.0));
    expect(find.text('PUSH'), findsNothing);
    expect(find.text('HELLO'), findsOneWidget);
    await tester.pump(const Duration(milliseconds: 20));
    expect(find.text('PUSH'), findsOneWidget);
    expect(find.text('HELLO'), findsOneWidget);
    final Offset helloPosition6 = tester.getCenter(find.text('HELLO'));
    expect(helloPosition5.dx, lessThan(helloPosition6.dx));
    expect(helloPosition5.dy, helloPosition6.dy);
    expect(Theme.of(tester.element(find.text('HELLO'))).platform, TargetPlatform.macOS);
  });

  testWidgets(
    'test no back gesture on fullscreen dialogs',
    (WidgetTester tester) async {
      await tester.pumpWidget(const MaterialApp(home: Scaffold(body: Text('Page 1'))));

      tester
          .state<NavigatorState>(find.byType(Navigator))
          .push(
            MaterialPageRoute<void>(
              builder: (BuildContext context) {
                return const Scaffold(body: Text('Page 2'));
              },
              fullscreenDialog: true,
            ),
          );
      await tester.pumpAndSettle();

      expect(find.text('Page 1'), findsNothing);
      expect(find.text('Page 2'), isOnstage);

      // Drag from left edge to invoke the gesture.
      final TestGesture gesture = await tester.startGesture(const Offset(5.0, 100.0));
      await gesture.moveBy(const Offset(400.0, 0.0));
      await tester.pump();

      expect(find.text('Page 1'), findsNothing);
      expect(find.text('Page 2'), isOnstage);

      // Page 2 didn't move.
      expect(tester.getTopLeft(find.text('Page 2')), Offset.zero);
    },
    variant: const TargetPlatformVariant(<TargetPlatform>{
      TargetPlatform.iOS,
      TargetPlatform.macOS,
    }),
  );

  testWidgets(
    'test fullscreen routes do not transition previous route',
    (WidgetTester tester) async {
      await tester.pumpWidget(
        MaterialApp(
          initialRoute: '/',
          onGenerateRoute: (RouteSettings settings) {
            if (settings.name == '/') {
              return PageRouteBuilder<void>(
                pageBuilder: (
                  BuildContext context,
                  Animation<double> animation,
                  Animation<double> secondaryAnimation,
                ) {
                  return Scaffold(appBar: AppBar(title: const Text('Page 1')), body: Container());
                },
              );
            }
            return MaterialPageRoute<void>(
              builder: (BuildContext context) {
                return Scaffold(appBar: AppBar(title: const Text('Page 2')), body: Container());
              },
              fullscreenDialog: true,
            );
          },
        ),
      );

      expect(find.text('Page 1'), findsOneWidget);
      expect(find.text('Page 2'), findsNothing);

      final double pageTitleDX = tester.getTopLeft(find.text('Page 1')).dx;

      tester.state<NavigatorState>(find.byType(Navigator)).pushNamed('/next');
      await tester.pump();
      await tester.pump(const Duration(milliseconds: 100));

      // Second page transition has started.
      expect(find.text('Page 2'), findsOneWidget);

      // First page has not moved.
      expect(tester.getTopLeft(find.text('Page 1')).dx, equals(pageTitleDX));
    },
    variant: const TargetPlatformVariant(<TargetPlatform>{
      TargetPlatform.iOS,
      TargetPlatform.macOS,
    }),
  );

  testWidgets(
    'test adaptable transitions switch during execution',
    (WidgetTester tester) async {
      await tester.pumpWidget(
        MaterialApp(
          theme: ThemeData(
            platform: TargetPlatform.android,
            pageTransitionsTheme: const PageTransitionsTheme(
              builders: <TargetPlatform, PageTransitionsBuilder>{
                TargetPlatform.android: FadeUpwardsPageTransitionsBuilder(),
              },
            ),
          ),
          home: const Material(child: Text('Page 1')),
          routes: <String, WidgetBuilder>{
            '/next': (BuildContext context) {
              return const Material(child: Text('Page 2'));
            },
          },
        ),
      );

      final Offset widget1InitialTopLeft = tester.getTopLeft(find.text('Page 1'));

      tester.state<NavigatorState>(find.byType(Navigator)).pushNamed('/next');
      await tester.pump();
      await tester.pump(const Duration(milliseconds: 100));

      Offset widget2TopLeft = tester.getTopLeft(find.text('Page 2'));
      final Size widget2Size = tester.getSize(find.text('Page 2'));

      // Android transition is vertical only.
      expect(widget1InitialTopLeft.dx == widget2TopLeft.dx, true);
      // Page 1 is above page 2 mid-transition.
      expect(widget1InitialTopLeft.dy < widget2TopLeft.dy, true);
      // Animation begins from the top of the page.
      expect(widget2TopLeft.dy < widget2Size.height, true);

      await tester.pump(const Duration(milliseconds: 300));

      // Page 2 covers page 1.
      expect(find.text('Page 1'), findsNothing);
      expect(find.text('Page 2'), isOnstage);

      // Re-pump the same app but with iOS instead of Android.
      await tester.pumpWidget(
        MaterialApp(
          home: const Material(child: Text('Page 1')),
          routes: <String, WidgetBuilder>{
            '/next': (BuildContext context) {
              return const Material(child: Text('Page 2'));
            },
          },
        ),
      );

      tester.state<NavigatorState>(find.byType(Navigator)).pop();
      await tester.pump();
      await tester.pump(const Duration(milliseconds: 100));

      Offset widget1TransientTopLeft = tester.getTopLeft(find.text('Page 1'));
      widget2TopLeft = tester.getTopLeft(find.text('Page 2'));

      // Page 1 is coming back from the left.
      expect(widget1TransientTopLeft.dx < widget1InitialTopLeft.dx, true);
      // Page 1 isn't moving vertically.
      expect(widget1TransientTopLeft.dy == widget1InitialTopLeft.dy, true);
      // iOS transition is horizontal only.
      expect(widget1InitialTopLeft.dy == widget2TopLeft.dy, true);
      // Page 2 is leaving towards the right.
      expect(widget2TopLeft.dx > widget1InitialTopLeft.dx, true);

      await tester.pump(const Duration(milliseconds: 300));

      expect(find.text('Page 1'), isOnstage);
      expect(find.text('Page 2'), findsNothing);

      widget1TransientTopLeft = tester.getTopLeft(find.text('Page 1'));

      // Page 1 is back where it started.
      expect(widget1InitialTopLeft == widget1TransientTopLeft, true);
    },
    variant: const TargetPlatformVariant(<TargetPlatform>{
      TargetPlatform.iOS,
      TargetPlatform.macOS,
    }),
  );

  testWidgets(
    'test edge swipe then drop back at starting point works',
    (WidgetTester tester) async {
      await tester.pumpWidget(
        MaterialApp(
          onGenerateRoute: (RouteSettings settings) {
            return MaterialPageRoute<void>(
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
    },
    variant: const TargetPlatformVariant(<TargetPlatform>{
      TargetPlatform.iOS,
      TargetPlatform.macOS,
    }),
  );

  testWidgets(
    'test edge swipe then drop back at ending point works',
    (WidgetTester tester) async {
      await tester.pumpWidget(
        MaterialApp(
          onGenerateRoute: (RouteSettings settings) {
            return MaterialPageRoute<void>(
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
      // The width of the page.
      await gesture.moveBy(const Offset(800, 0));
      await gesture.up();
      await tester.pump();

      expect(find.text('Page 1'), isOnstage);
      expect(find.text('Page 2'), findsNothing);
    },
    variant: const TargetPlatformVariant(<TargetPlatform>{
      TargetPlatform.iOS,
      TargetPlatform.macOS,
    }),
  );

  testWidgets(
    'Back swipe dismiss interrupted by route push',
    (WidgetTester tester) async {
      // Regression test for https://github.com/flutter/flutter/issues/28728
      final GlobalKey scaffoldKey = GlobalKey();

      await tester.pumpWidget(
        MaterialApp(
          home: Scaffold(
            key: scaffoldKey,
            body: Center(
              child: ElevatedButton(
                onPressed: () {
                  Navigator.push<void>(
                    scaffoldKey.currentContext!,
                    MaterialPageRoute<void>(
                      builder: (BuildContext context) {
                        return const Scaffold(body: Center(child: Text('route')));
                      },
                    ),
                  );
                },
                child: const Text('push'),
              ),
            ),
          ),
        ),
      );

      // Check the basic iOS back-swipe dismiss transition. Dragging the pushed
      // route halfway across the screen will trigger the iOS dismiss animation.

      await tester.tap(find.text('push'));
      await tester.pumpAndSettle();
      expect(find.text('route'), findsOneWidget);
      expect(find.text('push'), findsNothing);

      TestGesture gesture = await tester.startGesture(const Offset(5, 300));
      await gesture.moveBy(const Offset(400, 0));
      await gesture.up();
      await tester.pump();
      expect(
        // The 'route' route has been dragged to the right, halfway across the screen.
        tester.getTopLeft(find.ancestor(of: find.text('route'), matching: find.byType(Scaffold))),
        const Offset(400, 0),
      );
      expect(
        // The 'push' route is sliding in from the left.
        tester.getTopLeft(find.ancestor(of: find.text('push'), matching: find.byType(Scaffold))).dx,
        lessThan(0),
      );
      await tester.pumpAndSettle();
      expect(find.text('push'), findsOneWidget);
      expect(
        tester.getTopLeft(find.ancestor(of: find.text('push'), matching: find.byType(Scaffold))),
        Offset.zero,
      );
      expect(find.text('route'), findsNothing);

      // Run the dismiss animation 60%, which exposes the route "push" button,
      // and then press the button. A drag dropped animation is 400ms when dropped
      // exactly halfway. It follows a curve that is very steep initially.

      await tester.tap(find.text('push'));
      await tester.pumpAndSettle();
      expect(find.text('route'), findsOneWidget);
      expect(find.text('push'), findsNothing);

      gesture = await tester.startGesture(const Offset(5, 300));
      await gesture.moveBy(const Offset(400, 0)); // Drag halfway.
      await gesture.up();
      await tester.pump(); // Trigger the dropped snapping animation.
      expect(
        tester.getTopLeft(find.ancestor(of: find.text('route'), matching: find.byType(Scaffold))),
        const Offset(400, 0),
      );
      // Let the dismissing snapping animation go 60%.
      await tester.pump(const Duration(milliseconds: 240));
      expect(
        tester
            .getTopLeft(find.ancestor(of: find.text('route'), matching: find.byType(Scaffold)))
            .dx,
        moreOrLessEquals(794, epsilon: 1),
      );

      // Use the navigator to push a route instead of tapping the 'push' button.
      // The topmost route (the one that's animating away), ignores input while
      // the pop is underway because route.navigator.userGestureInProgress.
      Navigator.push<void>(
        scaffoldKey.currentContext!,
        MaterialPageRoute<void>(
          builder: (BuildContext context) {
            return const Scaffold(body: Center(child: Text('route')));
          },
        ),
      );

      await tester.pumpAndSettle();
      expect(find.text('route'), findsOneWidget);
      expect(find.text('push'), findsNothing);
    },
    variant: const TargetPlatformVariant(<TargetPlatform>{
      TargetPlatform.iOS,
      TargetPlatform.macOS,
    }),
  );

  testWidgets(
    'During back swipe the route ignores input',
    (WidgetTester tester) async {
      // Regression test for https://github.com/flutter/flutter/issues/39989

      final GlobalKey homeScaffoldKey = GlobalKey();
      final GlobalKey pageScaffoldKey = GlobalKey();
      int homeTapCount = 0;
      int pageTapCount = 0;

      await tester.pumpWidget(
        MaterialApp(
          home: Scaffold(
            key: homeScaffoldKey,
            body: GestureDetector(
              onTap: () {
                homeTapCount += 1;
              },
            ),
          ),
        ),
      );

      await tester.tap(find.byKey(homeScaffoldKey));
      expect(homeTapCount, 1);
      expect(pageTapCount, 0);

      Navigator.push<void>(
        homeScaffoldKey.currentContext!,
        MaterialPageRoute<void>(
          builder: (BuildContext context) {
            return Scaffold(
              key: pageScaffoldKey,
              appBar: AppBar(title: const Text('Page')),
              body: Padding(
                padding: const EdgeInsets.all(16),
                child: GestureDetector(
                  onTap: () {
                    pageTapCount += 1;
                  },
                ),
              ),
            );
          },
        ),
      );

      await tester.pumpAndSettle();
      await tester.tap(find.byKey(pageScaffoldKey));
      expect(homeTapCount, 1);
      expect(pageTapCount, 1);

      // Start the basic iOS back-swipe dismiss transition. Drag the pushed
      // "page" route halfway across the screen. The underlying "home" will
      // start sliding in from the left.

      final TestGesture gesture = await tester.startGesture(const Offset(5, 300));
      await gesture.moveBy(const Offset(400, 0));
      await tester.pump();
      expect(tester.getTopLeft(find.byKey(pageScaffoldKey)), const Offset(400, 0));
      expect(tester.getTopLeft(find.byKey(homeScaffoldKey)).dx, lessThan(0));

      // Tapping on the "page" route doesn't trigger the GestureDetector because
      // it's being dragged.
      await tester.tap(find.byKey(pageScaffoldKey), warnIfMissed: false);
      expect(homeTapCount, 1);
      expect(pageTapCount, 1);

      // Tapping the "page" route's back button doesn't do anything either.
      await tester.tap(find.byTooltip('Back'), warnIfMissed: false);
      await tester.pumpAndSettle();
      expect(tester.getTopLeft(find.byKey(pageScaffoldKey)), const Offset(400, 0));
      expect(tester.getTopLeft(find.byKey(homeScaffoldKey)).dx, lessThan(0));
    },
    variant: const TargetPlatformVariant(<TargetPlatform>{
      TargetPlatform.iOS,
      TargetPlatform.macOS,
    }),
  );

  testWidgets(
    'After a pop caused by a back-swipe, input reaches the exposed route',
    (WidgetTester tester) async {
      // Regression test for https://github.com/flutter/flutter/issues/41024

      final GlobalKey homeScaffoldKey = GlobalKey();
      final GlobalKey pageScaffoldKey = GlobalKey();
      int homeTapCount = 0;
      int pageTapCount = 0;

      await tester.pumpWidget(
        MaterialApp(
          home: Scaffold(
            key: homeScaffoldKey,
            body: GestureDetector(
              onTap: () {
                homeTapCount += 1;
              },
            ),
          ),
        ),
      );

      await tester.tap(find.byKey(homeScaffoldKey));
      expect(homeTapCount, 1);
      expect(pageTapCount, 0);

      final ValueNotifier<bool> notifier =
          Navigator.of(homeScaffoldKey.currentContext!).userGestureInProgressNotifier;
      expect(notifier.value, false);

      Navigator.push<void>(
        homeScaffoldKey.currentContext!,
        MaterialPageRoute<void>(
          builder: (BuildContext context) {
            return Scaffold(
              key: pageScaffoldKey,
              appBar: AppBar(title: const Text('Page')),
              body: Padding(
                padding: const EdgeInsets.all(16),
                child: GestureDetector(
                  onTap: () {
                    pageTapCount += 1;
                  },
                ),
              ),
            );
          },
        ),
      );

      await tester.pumpAndSettle();
      await tester.tap(find.byKey(pageScaffoldKey));
      expect(homeTapCount, 1);
      expect(pageTapCount, 1);

      // Trigger the basic iOS back-swipe dismiss transition. Drag the pushed
      // "page" route more than halfway across the screen and then release it.

      final TestGesture gesture = await tester.startGesture(const Offset(5, 300));
      await gesture.moveBy(const Offset(500, 0));
      await tester.pump();
      expect(tester.getTopLeft(find.byKey(pageScaffoldKey)), const Offset(500, 0));
      expect(tester.getTopLeft(find.byKey(homeScaffoldKey)).dx, lessThan(0));
      expect(notifier.value, true);
      await gesture.up();
      await tester.pumpAndSettle();
      expect(notifier.value, false);
      expect(find.byKey(pageScaffoldKey), findsNothing);

      // The back-swipe dismiss pop transition has finished and input on the
      // home page still works.
      await tester.tap(find.byKey(homeScaffoldKey));
      expect(homeTapCount, 2);
      expect(pageTapCount, 1);
    },
    variant: const TargetPlatformVariant(<TargetPlatform>{
      TargetPlatform.iOS,
      TargetPlatform.macOS,
    }),
  );

  testWidgets(
    'A MaterialPageRoute should slide out with CupertinoPageTransition when a compatible PageRoute is pushed on top of it',
    (WidgetTester tester) async {
      // Regression test for https://github.com/flutter/flutter/issues/44864.

      await tester.pumpWidget(
        MaterialApp(
          theme: ThemeData(platform: TargetPlatform.iOS),
          home: Scaffold(appBar: AppBar(title: const Text('Title'))),
        ),
      );

      final Offset titleInitialTopLeft = tester.getTopLeft(find.text('Title'));

      tester
          .state<NavigatorState>(find.byType(Navigator))
          .push<void>(
            CupertinoPageRoute<void>(builder: (BuildContext context) => const Placeholder()),
          );

      await tester.pump();
      await tester.pump(const Duration(milliseconds: 150));

      final Offset titleTransientTopLeft = tester.getTopLeft(find.text('Title'));

      // Title of the first route slides to the left.
      expect(titleInitialTopLeft.dy, equals(titleTransientTopLeft.dy));
      expect(titleInitialTopLeft.dx, greaterThan(titleTransientTopLeft.dx));
    },
    variant: const TargetPlatformVariant(<TargetPlatform>{
      TargetPlatform.iOS,
      TargetPlatform.macOS,
    }),
  );

  testWidgets('MaterialPage works', (WidgetTester tester) async {
    final LocalKey pageKey = UniqueKey();
    final TransitionDetector detector = TransitionDetector();
    List<Page<void>> myPages = <Page<void>>[
      MaterialPage<void>(key: pageKey, child: const Text('first')),
    ];
    await tester.pumpWidget(
      buildNavigator(
        view: tester.view,
        pages: myPages,
        onPopPage: (Route<dynamic> route, dynamic result) {
          assert(false); // The test should never execute this.
          return true;
        },
        transitionDelegate: detector,
      ),
    );

    expect(detector.hasTransition, isFalse);
    expect(find.text('first'), findsOneWidget);

    myPages = <Page<void>>[MaterialPage<void>(key: pageKey, child: const Text('second'))];

    await tester.pumpWidget(
      buildNavigator(
        view: tester.view,
        pages: myPages,
        onPopPage: (Route<dynamic> route, dynamic result) {
          assert(false); // The test should never execute this.
          return true;
        },
        transitionDelegate: detector,
      ),
    );
    // There should be no transition because the page has the same key.
    expect(detector.hasTransition, isFalse);
    // The content does update.
    expect(find.text('first'), findsNothing);
    expect(find.text('second'), findsOneWidget);
  });

  testWidgets('MaterialPage can toggle MaintainState', (WidgetTester tester) async {
    final LocalKey pageKeyOne = UniqueKey();
    final LocalKey pageKeyTwo = UniqueKey();
    final TransitionDetector detector = TransitionDetector();
    List<Page<void>> myPages = <Page<void>>[
      MaterialPage<void>(key: pageKeyOne, maintainState: false, child: const Text('first')),
      MaterialPage<void>(key: pageKeyTwo, child: const Text('second')),
    ];
    await tester.pumpWidget(
      buildNavigator(
        view: tester.view,
        pages: myPages,
        onPopPage: (Route<dynamic> route, dynamic result) {
          assert(false); // The test should never execute this.
          return true;
        },
        transitionDelegate: detector,
      ),
    );

    expect(detector.hasTransition, isFalse);
    // Page one does not maintain state.
    expect(find.text('first', skipOffstage: false), findsNothing);
    expect(find.text('second'), findsOneWidget);

    myPages = <Page<void>>[
      MaterialPage<void>(key: pageKeyOne, child: const Text('first')),
      MaterialPage<void>(key: pageKeyTwo, child: const Text('second')),
    ];

    await tester.pumpWidget(
      buildNavigator(
        view: tester.view,
        pages: myPages,
        onPopPage: (Route<dynamic> route, dynamic result) {
          assert(false); // The test should never execute this.
          return true;
        },
        transitionDelegate: detector,
      ),
    );
    // There should be no transition because the page has the same key.
    expect(detector.hasTransition, isFalse);
    // Page one sets the maintain state to be true, its widget tree should be
    // built.
    expect(find.text('first', skipOffstage: false), findsOneWidget);
    expect(find.text('second'), findsOneWidget);
  });

  testWidgets('MaterialPage does not lose its state when transitioning out', (
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

  testWidgets('MaterialPage restores its state', (WidgetTester tester) async {
    await tester.pumpWidget(
      RootRestorationScope(
        restorationId: 'root',
        child: TestDependencies(
          child: Navigator(
            onPopPage: (Route<dynamic> route, dynamic result) {
              return false;
            },
            pages: const <Page<Object?>>[
              MaterialPage<void>(
                restorationId: 'p1',
                child: TestRestorableWidget(restorationId: 'p1'),
              ),
            ],
            restorationScopeId: 'nav',
            onGenerateRoute: (RouteSettings settings) {
              return MaterialPageRoute<void>(
                settings: settings,
                builder: (BuildContext context) {
                  return TestRestorableWidget(restorationId: settings.name!);
                },
              );
            },
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

  testWidgets('MaterialPageRoute can be dismissed with escape keyboard shortcut', (
    WidgetTester tester,
  ) async {
    // Regression test for https://github.com/flutter/flutter/issues/132138.
    final GlobalKey scaffoldKey = GlobalKey();

    await tester.pumpWidget(
      MaterialApp(
        home: Scaffold(
          key: scaffoldKey,
          body: Center(
            child: ElevatedButton(
              onPressed: () {
                Navigator.push<void>(
                  scaffoldKey.currentContext!,
                  MaterialPageRoute<void>(
                    builder: (BuildContext context) {
                      return const Scaffold(body: Center(child: Text('route')));
                    },
                    barrierDismissible: true,
                  ),
                );
              },
              child: const Text('push'),
            ),
          ),
        ),
      ),
    );

    await tester.tap(find.text('push'));
    await tester.pumpAndSettle();
    expect(find.text('route'), findsOneWidget);
    expect(find.text('push'), findsNothing);

    // Try to dismiss the route with the escape key.
    await tester.sendKeyEvent(LogicalKeyboardKey.escape);
    await tester.pumpAndSettle();

    expect(find.text('route'), findsNothing);
  });

  testWidgets(
    'Setting MaterialPageRoute.requestFocus to false does not request focus on the page',
    (WidgetTester tester) async {
      late BuildContext savedContext;
      const String pageTwoText = 'Page Two';
      await tester.pumpWidget(
        MaterialApp(
          home: Builder(
            builder: (BuildContext context) {
              savedContext = context;
              return Container();
            },
          ),
        ),
      );
      await tester.pump();

      // Check page two is not on the screen.
      expect(find.text(pageTwoText), findsNothing);

      // Navigate to page two with text.
      final NavigatorState navigator = Navigator.of(savedContext);
      navigator.push(
        MaterialPageRoute<void>(
          builder: (BuildContext context) {
            return const Text(pageTwoText);
          },
        ),
      );
      await tester.pump();
      await tester.pump(const Duration(milliseconds: 100));

      // The page two is showing and the text widget has focus.
      Element textOnPageTwo = tester.element(find.text(pageTwoText));
      FocusScopeNode focusScopeNode = FocusScope.of(textOnPageTwo);
      expect(focusScopeNode.hasFocus, isTrue);

      // Navigate back to page one.
      navigator.pop();
      await tester.pump();
      await tester.pump(const Duration(milliseconds: 100));

      // Navigate to page two again with requestFocus set to false.
      navigator.push(
        MaterialPageRoute<void>(
          requestFocus: false,
          builder: (BuildContext context) {
            return const Text(pageTwoText);
          },
        ),
      );
      await tester.pump();
      await tester.pump(const Duration(milliseconds: 100));

      // The page two is showing and the text widget is not focused.
      textOnPageTwo = tester.element(find.text(pageTwoText));
      focusScopeNode = FocusScope.of(textOnPageTwo);
      expect(focusScopeNode.hasFocus, isFalse);
    },
  );
}

class TransitionDetector extends DefaultTransitionDelegate<void> {
  bool hasTransition = false;
  @override
  Iterable<RouteTransitionRecord> resolve({
    required List<RouteTransitionRecord> newPageRouteHistory,
    required Map<RouteTransitionRecord?, RouteTransitionRecord> locationToExitingPageRoute,
    required Map<RouteTransitionRecord?, List<RouteTransitionRecord>> pageRouteToPagelessRoutes,
  }) {
    hasTransition = true;
    return super.resolve(
      newPageRouteHistory: newPageRouteHistory,
      locationToExitingPageRoute: locationToExitingPageRoute,
      pageRouteToPagelessRoutes: pageRouteToPagelessRoutes,
    );
  }
}

Widget buildNavigator({
  required List<Page<dynamic>> pages,
  required PopPageCallback onPopPage,
  required ui.FlutterView view,
  GlobalKey<NavigatorState>? key,
  TransitionDelegate<dynamic>? transitionDelegate,
}) {
  return MediaQuery(
    data: MediaQueryData.fromView(view),
    child: Localizations(
      locale: const Locale('en', 'US'),
      delegates: const <LocalizationsDelegate<dynamic>>[
        DefaultMaterialLocalizations.delegate,
        DefaultWidgetsLocalizations.delegate,
      ],
      child: Directionality(
        textDirection: TextDirection.ltr,
        child: Navigator(
          key: key,
          pages: pages,
          onPopPage: onPopPage,
          transitionDelegate: transitionDelegate ?? const DefaultTransitionDelegate<dynamic>(),
        ),
      ),
    ),
  );
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
    return MaterialApp(
      home: Navigator(
        key: widget.navigatorKey,
        pages: <Page<void>>[
          const MaterialPage<void>(child: Text('home')),
          if (_subpage != null) MaterialPage<void>(child: Text(_subpage!)),
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
  Widget build(BuildContext context) {
    return Column(
      children: <Widget>[
        Text(widget.restorationId),
        Text('count: ${counter.value}'),
        ElevatedButton(
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

  @override
  void dispose() {
    counter.dispose();
    super.dispose();
  }
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
