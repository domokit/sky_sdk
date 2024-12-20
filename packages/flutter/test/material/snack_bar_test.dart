// Copyright 2014 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

// This file is run as part of a reduced test set in CI on Mac and Windows
// machines.
@Tags(<String>['reduced-test-set'])
library;

import 'dart:async';
import 'dart:ui';

import 'package:flutter/foundation.dart';
import 'package:flutter/material.dart';
import 'package:flutter/rendering.dart';
import 'package:flutter_test/flutter_test.dart';

void main() {
  testWidgets('SnackBar control test', (WidgetTester tester) async {
    const String helloSnackBar = 'Hello SnackBar';
    const Key tapTarget = Key('tap-target');
    await tester.pumpWidget(
      MaterialApp(
        home: Scaffold(
          body: Builder(
            builder: (BuildContext context) {
              return GestureDetector(
                key: tapTarget,
                onTap: () {
                  ScaffoldMessenger.of(context).showSnackBar(
                    const SnackBar(content: Text(helloSnackBar), duration: Duration(seconds: 2)),
                  );
                },
                behavior: HitTestBehavior.opaque,
                child: const SizedBox(height: 100.0, width: 100.0),
              );
            },
          ),
        ),
      ),
    );
    expect(find.text(helloSnackBar), findsNothing);
    await tester.tap(find.byKey(tapTarget));
    expect(find.text(helloSnackBar), findsNothing);
    await tester.pump(); // schedule animation
    expect(find.text(helloSnackBar), findsOneWidget);
    await tester.pump(); // begin animation
    expect(find.text(helloSnackBar), findsOneWidget);
    await tester.pump(
      const Duration(milliseconds: 750),
    ); // 0.75s // animation last frame; two second timer starts here
    expect(find.text(helloSnackBar), findsOneWidget);
    await tester.pump(const Duration(milliseconds: 750)); // 1.50s
    expect(find.text(helloSnackBar), findsOneWidget);
    await tester.pump(const Duration(milliseconds: 750)); // 2.25s
    expect(find.text(helloSnackBar), findsOneWidget);
    await tester.pump(
      const Duration(milliseconds: 750),
    ); // 3.00s // timer triggers to dismiss snackbar, reverse animation is scheduled
    await tester.pump(); // begin animation
    expect(find.text(helloSnackBar), findsOneWidget); // frame 0 of dismiss animation
    await tester.pump(
      const Duration(milliseconds: 750),
    ); // 3.75s // last frame of animation, snackbar removed from build
    expect(find.text(helloSnackBar), findsNothing);
  });

  testWidgets('SnackBar twice test', (WidgetTester tester) async {
    int snackBarCount = 0;
    const Key tapTarget = Key('tap-target');
    await tester.pumpWidget(
      MaterialApp(
        home: Scaffold(
          body: Builder(
            builder: (BuildContext context) {
              return GestureDetector(
                key: tapTarget,
                onTap: () {
                  snackBarCount += 1;
                  ScaffoldMessenger.of(context).showSnackBar(
                    SnackBar(
                      content: Text('bar$snackBarCount'),
                      duration: const Duration(seconds: 2),
                    ),
                  );
                },
                behavior: HitTestBehavior.opaque,
                child: const SizedBox(height: 100.0, width: 100.0),
              );
            },
          ),
        ),
      ),
    );
    expect(find.text('bar1'), findsNothing);
    expect(find.text('bar2'), findsNothing);
    await tester.tap(find.byKey(tapTarget)); // queue bar1
    await tester.tap(find.byKey(tapTarget)); // queue bar2
    expect(find.text('bar1'), findsNothing);
    expect(find.text('bar2'), findsNothing);
    await tester.pump(); // schedule animation for bar1
    expect(find.text('bar1'), findsOneWidget);
    expect(find.text('bar2'), findsNothing);
    await tester.pump(); // begin animation
    expect(find.text('bar1'), findsOneWidget);
    expect(find.text('bar2'), findsNothing);
    await tester.pump(
      const Duration(milliseconds: 750),
    ); // 0.75s // animation last frame; two second timer starts here
    expect(find.text('bar1'), findsOneWidget);
    expect(find.text('bar2'), findsNothing);
    await tester.pump(const Duration(milliseconds: 750)); // 1.50s
    expect(find.text('bar1'), findsOneWidget);
    expect(find.text('bar2'), findsNothing);
    await tester.pump(const Duration(milliseconds: 750)); // 2.25s
    expect(find.text('bar1'), findsOneWidget);
    expect(find.text('bar2'), findsNothing);
    await tester.pump(
      const Duration(milliseconds: 750),
    ); // 3.00s // timer triggers to dismiss snackbar, reverse animation is scheduled
    await tester.pump(); // begin animation
    expect(find.text('bar1'), findsOneWidget);
    expect(find.text('bar2'), findsNothing);
    await tester.pump(
      const Duration(milliseconds: 750),
    ); // 3.75s // last frame of animation, snackbar removed from build, new snack bar put in its place
    expect(find.text('bar1'), findsNothing);
    expect(find.text('bar2'), findsOneWidget);
    await tester.pump(); // begin animation
    expect(find.text('bar1'), findsNothing);
    expect(find.text('bar2'), findsOneWidget);
    await tester.pump(
      const Duration(milliseconds: 750),
    ); // 4.50s // animation last frame; two second timer starts here
    expect(find.text('bar1'), findsNothing);
    expect(find.text('bar2'), findsOneWidget);
    await tester.pump(const Duration(milliseconds: 750)); // 5.25s
    expect(find.text('bar1'), findsNothing);
    expect(find.text('bar2'), findsOneWidget);
    await tester.pump(const Duration(milliseconds: 750)); // 6.00s
    expect(find.text('bar1'), findsNothing);
    expect(find.text('bar2'), findsOneWidget);
    await tester.pump(
      const Duration(milliseconds: 750),
    ); // 6.75s // timer triggers to dismiss snackbar, reverse animation is scheduled
    await tester.pump(); // begin animation
    expect(find.text('bar1'), findsNothing);
    expect(find.text('bar2'), findsOneWidget);
    await tester.pump(
      const Duration(milliseconds: 750),
    ); // 7.50s // last frame of animation, snackbar removed from build, new snack bar put in its place
    expect(find.text('bar1'), findsNothing);
    expect(find.text('bar2'), findsNothing);
  });

  testWidgets('SnackBar cancel test', (WidgetTester tester) async {
    int snackBarCount = 0;
    const Key tapTarget = Key('tap-target');
    late int time;
    late ScaffoldFeatureController<SnackBar, SnackBarClosedReason> lastController;
    await tester.pumpWidget(
      MaterialApp(
        home: Scaffold(
          body: Builder(
            builder: (BuildContext context) {
              return GestureDetector(
                key: tapTarget,
                onTap: () {
                  snackBarCount += 1;
                  lastController = ScaffoldMessenger.of(context).showSnackBar(
                    SnackBar(content: Text('bar$snackBarCount'), duration: Duration(seconds: time)),
                  );
                },
                behavior: HitTestBehavior.opaque,
                child: const SizedBox(height: 100.0, width: 100.0),
              );
            },
          ),
        ),
      ),
    );
    expect(find.text('bar1'), findsNothing);
    expect(find.text('bar2'), findsNothing);
    time = 1000;
    await tester.tap(find.byKey(tapTarget)); // queue bar1
    final ScaffoldFeatureController<SnackBar, SnackBarClosedReason> firstController =
        lastController;
    time = 2;
    await tester.tap(find.byKey(tapTarget)); // queue bar2
    expect(find.text('bar1'), findsNothing);
    expect(find.text('bar2'), findsNothing);
    await tester.pump(); // schedule animation for bar1
    expect(find.text('bar1'), findsOneWidget);
    expect(find.text('bar2'), findsNothing);
    await tester.pump(); // begin animation
    expect(find.text('bar1'), findsOneWidget);
    expect(find.text('bar2'), findsNothing);
    await tester.pump(
      const Duration(milliseconds: 750),
    ); // 0.75s // animation last frame; two second timer starts here
    expect(find.text('bar1'), findsOneWidget);
    expect(find.text('bar2'), findsNothing);
    await tester.pump(const Duration(milliseconds: 750)); // 1.50s
    expect(find.text('bar1'), findsOneWidget);
    expect(find.text('bar2'), findsNothing);
    await tester.pump(const Duration(milliseconds: 750)); // 2.25s
    expect(find.text('bar1'), findsOneWidget);
    expect(find.text('bar2'), findsNothing);
    await tester.pump(const Duration(milliseconds: 10000)); // 12.25s
    expect(find.text('bar1'), findsOneWidget);
    expect(find.text('bar2'), findsNothing);

    firstController.close(); // snackbar is manually dismissed

    await tester.pump(
      const Duration(milliseconds: 750),
    ); // 13.00s // reverse animation is scheduled
    await tester.pump(); // begin animation
    expect(find.text('bar1'), findsOneWidget);
    expect(find.text('bar2'), findsNothing);
    await tester.pump(
      const Duration(milliseconds: 750),
    ); // 13.75s // last frame of animation, snackbar removed from build, new snack bar put in its place
    expect(find.text('bar1'), findsNothing);
    expect(find.text('bar2'), findsOneWidget);
    await tester.pump(); // begin animation
    expect(find.text('bar1'), findsNothing);
    expect(find.text('bar2'), findsOneWidget);
    await tester.pump(
      const Duration(milliseconds: 750),
    ); // 14.50s // animation last frame; two second timer starts here
    expect(find.text('bar1'), findsNothing);
    expect(find.text('bar2'), findsOneWidget);
    await tester.pump(const Duration(milliseconds: 750)); // 15.25s
    expect(find.text('bar1'), findsNothing);
    expect(find.text('bar2'), findsOneWidget);
    await tester.pump(const Duration(milliseconds: 750)); // 16.00s
    expect(find.text('bar1'), findsNothing);
    expect(find.text('bar2'), findsOneWidget);
    await tester.pump(
      const Duration(milliseconds: 750),
    ); // 16.75s // timer triggers to dismiss snackbar, reverse animation is scheduled
    await tester.pump(); // begin animation
    expect(find.text('bar1'), findsNothing);
    expect(find.text('bar2'), findsOneWidget);
    await tester.pump(
      const Duration(milliseconds: 750),
    ); // 17.50s // last frame of animation, snackbar removed from build, new snack bar put in its place
    expect(find.text('bar1'), findsNothing);
    expect(find.text('bar2'), findsNothing);
  });

  testWidgets('SnackBar dismiss test', (WidgetTester tester) async {
    const Key tapTarget = Key('tap-target');
    late DismissDirection dismissDirection;
    late double width;
    int snackBarCount = 0;

    await tester.pumpWidget(
      MaterialApp(
        home: Scaffold(
          body: Builder(
            builder: (BuildContext context) {
              width = MediaQuery.sizeOf(context).width;

              return GestureDetector(
                key: tapTarget,
                onTap: () {
                  snackBarCount += 1;
                  ScaffoldMessenger.of(context).showSnackBar(
                    SnackBar(
                      content: Text('bar$snackBarCount'),
                      duration: const Duration(seconds: 2),
                      dismissDirection: dismissDirection,
                    ),
                  );
                },
                behavior: HitTestBehavior.opaque,
                child: const SizedBox(height: 100.0, width: 100.0),
              );
            },
          ),
        ),
      ),
    );

    await _testSnackBarDismiss(
      tester: tester,
      tapTarget: tapTarget,
      scaffoldWidth: width,
      onDismissDirectionChange: (DismissDirection dir) => dismissDirection = dir,
      onDragGestureChange: () => snackBarCount = 0,
    );
  });

  testWidgets('SnackBar dismissDirection can be customised from SnackBarThemeData', (
    WidgetTester tester,
  ) async {
    const Key tapTarget = Key('tap-target');
    late double width;

    await tester.pumpWidget(
      MaterialApp(
        theme: ThemeData(
          snackBarTheme: const SnackBarThemeData(dismissDirection: DismissDirection.startToEnd),
        ),
        home: Scaffold(
          body: Builder(
            builder: (BuildContext context) {
              width = MediaQuery.sizeOf(context).width;

              return GestureDetector(
                key: tapTarget,
                onTap: () {
                  ScaffoldMessenger.of(context).showSnackBar(
                    const SnackBar(content: Text('swipe ltr'), duration: Duration(seconds: 2)),
                  );
                },
                behavior: HitTestBehavior.opaque,
                child: const SizedBox(height: 100.0, width: 100.0),
              );
            },
          ),
        ),
      ),
    );

    expect(find.text('swipe ltr'), findsNothing);
    await tester.tap(find.byKey(tapTarget));
    expect(find.text('swipe ltr'), findsNothing);
    await tester.pump(); // schedule animation for snack bar
    expect(find.text('swipe ltr'), findsOneWidget);
    await tester.pump(); // begin animation
    expect(find.text('swipe ltr'), findsOneWidget);
    await tester.pump(const Duration(milliseconds: 750));
    await tester.drag(find.text('swipe ltr'), Offset(width, 0.0));
    await tester.pump(); // snack bar dismissed
    expect(find.text('swipe ltr'), findsNothing);
  });

  testWidgets('dismissDirection from SnackBar should be preferred over SnackBarThemeData', (
    WidgetTester tester,
  ) async {
    const Key tapTarget = Key('tap-target');
    late double width;

    await tester.pumpWidget(
      MaterialApp(
        theme: ThemeData(
          snackBarTheme: const SnackBarThemeData(dismissDirection: DismissDirection.startToEnd),
        ),
        home: Scaffold(
          body: Builder(
            builder: (BuildContext context) {
              width = MediaQuery.sizeOf(context).width;

              return GestureDetector(
                key: tapTarget,
                onTap: () {
                  ScaffoldMessenger.of(context).showSnackBar(
                    const SnackBar(
                      content: Text('swipe rtl'),
                      duration: Duration(seconds: 2),
                      dismissDirection: DismissDirection.endToStart,
                    ),
                  );
                },
                behavior: HitTestBehavior.opaque,
                child: const SizedBox(height: 100.0, width: 100.0),
              );
            },
          ),
        ),
      ),
    );

    expect(find.text('swipe rtl'), findsNothing);
    await tester.tap(find.byKey(tapTarget));
    expect(find.text('swipe rtl'), findsNothing);
    await tester.pump(); // schedule animation for snack bar
    expect(find.text('swipe rtl'), findsOneWidget);
    await tester.pump(); // begin animation
    expect(find.text('swipe rtl'), findsOneWidget);
    await tester.pump(const Duration(milliseconds: 750));
    await tester.drag(find.text('swipe rtl'), Offset(-width, 0.0));
    await tester.pump(); // snack bar dismissed
    expect(find.text('swipe rtl'), findsNothing);
  });

  testWidgets('SnackBar cannot be tapped twice', (WidgetTester tester) async {
    int tapCount = 0;
    await tester.pumpWidget(
      MaterialApp(
        home: Scaffold(
          body: Builder(
            builder: (BuildContext context) {
              return GestureDetector(
                onTap: () {
                  ScaffoldMessenger.of(context).showSnackBar(
                    SnackBar(
                      content: const Text('I am a snack bar.'),
                      duration: const Duration(seconds: 2),
                      action: SnackBarAction(
                        label: 'ACTION',
                        onPressed: () {
                          ++tapCount;
                        },
                      ),
                    ),
                  );
                },
                child: const Text('X'),
              );
            },
          ),
        ),
      ),
    );
    await tester.tap(find.text('X'));
    await tester.pump(); // start animation
    await tester.pump(const Duration(milliseconds: 750));

    expect(tapCount, equals(0));
    await tester.tap(find.text('ACTION'));
    expect(tapCount, equals(1));
    await tester.tap(find.text('ACTION'));
    expect(tapCount, equals(1));
    await tester.pump();
    await tester.tap(find.text('ACTION'));
    expect(tapCount, equals(1));
  });

  testWidgets('Material2 - Light theme SnackBar has dark background', (WidgetTester tester) async {
    final ThemeData lightTheme = ThemeData.light(useMaterial3: false);
    await tester.pumpWidget(
      MaterialApp(
        theme: lightTheme,
        home: Scaffold(
          body: Builder(
            builder: (BuildContext context) {
              return GestureDetector(
                onTap: () {
                  ScaffoldMessenger.of(context).showSnackBar(
                    SnackBar(
                      content: const Text('I am a snack bar.'),
                      duration: const Duration(seconds: 2),
                      action: SnackBarAction(label: 'ACTION', onPressed: () {}),
                    ),
                  );
                },
                child: const Text('X'),
              );
            },
          ),
        ),
      ),
    );

    await tester.tap(find.text('X'));
    await tester.pump(); // start animation
    await tester.pump(const Duration(milliseconds: 750));

    final RenderPhysicalModel renderModel = tester.renderObject(
      find.widgetWithText(Material, 'I am a snack bar.').first,
    );
    // There is a somewhat complicated background color calculation based
    // off of the surface color. For the default light theme it
    // should be this value.
    expect(renderModel.color, isSameColorAs(const Color(0xFF333333)));
  });

  testWidgets('Material3 - Light theme SnackBar has dark background', (WidgetTester tester) async {
    final ThemeData lightTheme = ThemeData.light(useMaterial3: true);
    await tester.pumpWidget(
      MaterialApp(
        theme: lightTheme,
        home: Scaffold(
          body: Builder(
            builder: (BuildContext context) {
              return GestureDetector(
                onTap: () {
                  ScaffoldMessenger.of(context).showSnackBar(
                    SnackBar(
                      content: const Text('I am a snack bar.'),
                      duration: const Duration(seconds: 2),
                      action: SnackBarAction(label: 'ACTION', onPressed: () {}),
                    ),
                  );
                },
                child: const Text('X'),
              );
            },
          ),
        ),
      ),
    );

    await tester.tap(find.text('X'));
    await tester.pump(); // start animation
    await tester.pump(const Duration(milliseconds: 750));

    final Finder material = find.widgetWithText(Material, 'I am a snack bar.').first;
    final RenderPhysicalModel renderModel = tester.renderObject(material);

    expect(renderModel.color, equals(lightTheme.colorScheme.inverseSurface));
  });

  testWidgets('Dark theme SnackBar has light background', (WidgetTester tester) async {
    final ThemeData darkTheme = ThemeData.dark();
    await tester.pumpWidget(
      MaterialApp(
        theme: darkTheme,
        home: Scaffold(
          body: Builder(
            builder: (BuildContext context) {
              return GestureDetector(
                onTap: () {
                  ScaffoldMessenger.of(context).showSnackBar(
                    SnackBar(
                      content: const Text('I am a snack bar.'),
                      duration: const Duration(seconds: 2),
                      action: SnackBarAction(label: 'ACTION', onPressed: () {}),
                    ),
                  );
                },
                child: const Text('X'),
              );
            },
          ),
        ),
      ),
    );

    await tester.tap(find.text('X'));
    await tester.pump(); // start animation
    await tester.pump(const Duration(milliseconds: 750));

    final RenderPhysicalModel renderModel = tester.renderObject(
      find.widgetWithText(Material, 'I am a snack bar.').first,
    );
    expect(renderModel.color, equals(darkTheme.colorScheme.onSurface));
  });

  testWidgets('Material2 - Dark theme SnackBar has primary text buttons', (
    WidgetTester tester,
  ) async {
    final ThemeData darkTheme = ThemeData.dark(useMaterial3: false);
    await tester.pumpWidget(
      MaterialApp(
        theme: darkTheme,
        home: Scaffold(
          body: Builder(
            builder: (BuildContext context) {
              return GestureDetector(
                onTap: () {
                  ScaffoldMessenger.of(context).showSnackBar(
                    SnackBar(
                      content: const Text('I am a snack bar.'),
                      duration: const Duration(seconds: 2),
                      action: SnackBarAction(label: 'ACTION', onPressed: () {}),
                    ),
                  );
                },
                child: const Text('X'),
              );
            },
          ),
        ),
      ),
    );

    await tester.tap(find.text('X'));
    await tester.pump(); // start animation
    await tester.pump(const Duration(milliseconds: 750));

    final TextStyle buttonTextStyle =
        tester
            .widget<RichText>(
              find.descendant(of: find.text('ACTION'), matching: find.byType(RichText)),
            )
            .text
            .style!;
    expect(buttonTextStyle.color, equals(darkTheme.colorScheme.primary));
  });

  testWidgets('Material3 - Dark theme SnackBar has primary text buttons', (
    WidgetTester tester,
  ) async {
    final ThemeData darkTheme = ThemeData.dark(useMaterial3: true);
    await tester.pumpWidget(
      MaterialApp(
        theme: darkTheme,
        home: Scaffold(
          body: Builder(
            builder: (BuildContext context) {
              return GestureDetector(
                onTap: () {
                  ScaffoldMessenger.of(context).showSnackBar(
                    SnackBar(
                      content: const Text('I am a snack bar.'),
                      duration: const Duration(seconds: 2),
                      action: SnackBarAction(label: 'ACTION', onPressed: () {}),
                    ),
                  );
                },
                child: const Text('X'),
              );
            },
          ),
        ),
      ),
    );

    await tester.tap(find.text('X'));
    await tester.pump(); // start animation
    await tester.pump(const Duration(milliseconds: 750));

    final TextStyle buttonTextStyle =
        tester
            .widget<RichText>(
              find.descendant(of: find.text('ACTION'), matching: find.byType(RichText)),
            )
            .text
            .style!;
    expect(buttonTextStyle.color, equals(darkTheme.colorScheme.inversePrimary));
  });

  testWidgets('SnackBar should inherit theme data from its ancestor', (WidgetTester tester) async {
    final ThemeData theme = ThemeData(
      colorScheme: ColorScheme.fromSeed(seedColor: Colors.green, brightness: Brightness.dark),
    );
    ThemeData? themeBeforeSnackBar;
    ThemeData? themeAfterSnackBar;
    await tester.pumpWidget(
      MaterialApp(
        theme: theme,
        home: Scaffold(
          body: Builder(
            builder: (BuildContext context) {
              themeBeforeSnackBar = Theme.of(context);
              return GestureDetector(
                onTap: () {
                  ScaffoldMessenger.of(context).showSnackBar(
                    SnackBar(
                      content: Builder(
                        builder: (BuildContext context) {
                          themeAfterSnackBar = Theme.of(context);
                          return const Text('I am a snack bar.');
                        },
                      ),
                      duration: const Duration(seconds: 2),
                      action: SnackBarAction(label: 'ACTION', onPressed: () {}),
                    ),
                  );
                },
                child: const Text('X'),
              );
            },
          ),
        ),
      ),
    );

    await tester.tap(find.text('X'));
    await tester.pump(); // start animation
    await tester.pump(const Duration(milliseconds: 750));

    final ThemeData comparedTheme = themeBeforeSnackBar!.copyWith(
      colorScheme: themeAfterSnackBar!.colorScheme,
    ); // Fields replaced by SnackBar.
    expect(comparedTheme, themeAfterSnackBar);
  });

  testWidgets('Snackbar margin can be customized', (WidgetTester tester) async {
    const double padding = 20.0;
    await tester.pumpWidget(
      MaterialApp(
        home: Scaffold(
          body: Builder(
            builder: (BuildContext context) {
              return GestureDetector(
                onTap: () {
                  ScaffoldMessenger.of(context).showSnackBar(
                    const SnackBar(
                      content: Text('I am a snack bar.'),
                      margin: EdgeInsets.all(padding),
                      behavior: SnackBarBehavior.floating,
                    ),
                  );
                },
                child: const Text('X'),
              );
            },
          ),
        ),
      ),
    );

    await tester.tap(find.text('X'));
    await tester.pump(); // start animation
    await tester.pump(const Duration(milliseconds: 750));

    final Finder materialFinder = find.descendant(
      of: find.byType(SnackBar),
      matching: find.byType(Material),
    );
    final Offset snackBarBottomLeft = tester.getBottomLeft(materialFinder);
    final Offset snackBarBottomRight = tester.getBottomRight(materialFinder);
    expect(snackBarBottomLeft.dx, padding);
    expect(snackBarBottomLeft.dy, 600 - padding); // Device height is 600.
    expect(snackBarBottomRight.dx, 800 - padding); // Device width is 800.
  });

  testWidgets('SnackbarBehavior.floating is positioned within safe area', (
    WidgetTester tester,
  ) async {
    const double viewPadding = 50.0;
    const double floatingSnackBarDefaultBottomMargin = 10.0;
    await tester.pumpWidget(
      MaterialApp(
        home: MediaQuery(
          data: const MediaQueryData(
            // Simulate non-safe area.
            viewPadding: EdgeInsets.only(bottom: viewPadding),
          ),
          child: Scaffold(
            body: Builder(
              builder: (BuildContext context) {
                return GestureDetector(
                  onTap: () {
                    ScaffoldMessenger.of(context).showSnackBar(
                      const SnackBar(
                        content: Text('I am a snack bar.'),
                        behavior: SnackBarBehavior.floating,
                      ),
                    );
                  },
                  child: const Text('X'),
                );
              },
            ),
          ),
        ),
      ),
    );

    await tester.tap(find.text('X'));
    await tester.pump(); // Start animation
    await tester.pump(const Duration(milliseconds: 750));

    final Finder materialFinder = find.descendant(
      of: find.byType(SnackBar),
      matching: find.byType(Material),
    );
    final Offset snackBarBottomLeft = tester.getBottomLeft(materialFinder);
    expect(
      snackBarBottomLeft.dy,
      // Device height is 600.
      600 - viewPadding - floatingSnackBarDefaultBottomMargin,
    );
  });

  testWidgets('Snackbar padding can be customized', (WidgetTester tester) async {
    const double padding = 20.0;
    await tester.pumpWidget(
      MaterialApp(
        home: Scaffold(
          body: Builder(
            builder: (BuildContext context) {
              return GestureDetector(
                onTap: () {
                  ScaffoldMessenger.of(context).showSnackBar(
                    const SnackBar(
                      content: Text('I am a snack bar.'),
                      padding: EdgeInsets.all(padding),
                    ),
                  );
                },
                child: const Text('X'),
              );
            },
          ),
        ),
      ),
    );

    await tester.tap(find.text('X'));
    await tester.pump(); // start animation
    await tester.pump(const Duration(milliseconds: 750));

    final Finder textFinder = find.text('I am a snack bar.');
    final Finder materialFinder = find.descendant(
      of: find.byType(SnackBar),
      matching: find.byType(Material),
    );
    final Offset textBottomLeft = tester.getBottomLeft(textFinder);
    final Offset textTopRight = tester.getTopRight(textFinder);
    final Offset snackBarBottomLeft = tester.getBottomLeft(materialFinder);
    final Offset snackBarTopRight = tester.getTopRight(materialFinder);
    expect(textBottomLeft.dx - snackBarBottomLeft.dx, padding);
    expect(snackBarTopRight.dx - textTopRight.dx, padding);
    expect(snackBarBottomLeft.dy - textBottomLeft.dy, padding);
    expect(textTopRight.dy - snackBarTopRight.dy, padding);
  });

  testWidgets('Snackbar width can be customized', (WidgetTester tester) async {
    const double width = 200.0;
    await tester.pumpWidget(
      MaterialApp(
        home: Scaffold(
          body: Builder(
            builder: (BuildContext context) {
              return GestureDetector(
                onTap: () {
                  ScaffoldMessenger.of(context).showSnackBar(
                    const SnackBar(
                      content: Text('I am a snack bar.'),
                      width: width,
                      behavior: SnackBarBehavior.floating,
                    ),
                  );
                },
                child: const Text('X'),
              );
            },
          ),
        ),
      ),
    );

    await tester.tap(find.text('X'));
    await tester.pump(); // start animation
    await tester.pump(const Duration(milliseconds: 750));

    final Finder materialFinder = find.descendant(
      of: find.byType(SnackBar),
      matching: find.byType(Material),
    );
    final Offset snackBarBottomLeft = tester.getBottomLeft(materialFinder);
    final Offset snackBarBottomRight = tester.getBottomRight(materialFinder);
    expect(snackBarBottomLeft.dx, (800 - width) / 2); // Device width is 800.
    expect(snackBarBottomRight.dx, (800 + width) / 2); // Device width is 800.
  });

  testWidgets('Snackbar width can be customized from ThemeData', (WidgetTester tester) async {
    const double width = 200.0;
    await tester.pumpWidget(
      MaterialApp(
        theme: ThemeData(
          snackBarTheme: const SnackBarThemeData(width: width, behavior: SnackBarBehavior.floating),
        ),
        home: Scaffold(
          body: Builder(
            builder: (BuildContext context) {
              return GestureDetector(
                onTap: () {
                  ScaffoldMessenger.of(
                    context,
                  ).showSnackBar(const SnackBar(content: Text('Feeling snackish')));
                },
                child: const Text('X'),
              );
            },
          ),
        ),
      ),
    );

    await tester.tap(find.text('X'));
    await tester.pump(); // start animation
    await tester.pump(const Duration(milliseconds: 750));

    final Finder materialFinder = find.descendant(
      of: find.byType(SnackBar),
      matching: find.byType(Material),
    );
    final Offset snackBarBottomLeft = tester.getBottomLeft(materialFinder);
    final Offset snackBarBottomRight = tester.getBottomRight(materialFinder);
    expect(snackBarBottomLeft.dx, (800 - width) / 2); // Device width is 800.
    expect(snackBarBottomRight.dx, (800 + width) / 2); // Device width is 800.
  });

  testWidgets('Snackbar width customization takes preference of widget over theme', (
    WidgetTester tester,
  ) async {
    const double themeWidth = 200.0;
    const double widgetWidth = 400.0;
    await tester.pumpWidget(
      MaterialApp(
        theme: ThemeData(
          snackBarTheme: const SnackBarThemeData(
            width: themeWidth,
            behavior: SnackBarBehavior.floating,
          ),
        ),
        home: Scaffold(
          body: Builder(
            builder: (BuildContext context) {
              return GestureDetector(
                onTap: () {
                  ScaffoldMessenger.of(context).showSnackBar(
                    const SnackBar(content: Text('Feeling super snackish'), width: widgetWidth),
                  );
                },
                child: const Text('X'),
              );
            },
          ),
        ),
      ),
    );

    await tester.tap(find.text('X'));
    await tester.pump(); // start animation
    await tester.pump(const Duration(milliseconds: 750));

    final Finder materialFinder = find.descendant(
      of: find.byType(SnackBar),
      matching: find.byType(Material),
    );
    final Offset snackBarBottomLeft = tester.getBottomLeft(materialFinder);
    final Offset snackBarBottomRight = tester.getBottomRight(materialFinder);
    expect(snackBarBottomLeft.dx, (800 - widgetWidth) / 2); // Device width is 800.
    expect(snackBarBottomRight.dx, (800 + widgetWidth) / 2); // Device width is 800.
  });

  testWidgets('Material2 - Snackbar labels can be colored as MaterialColor', (
    WidgetTester tester,
  ) async {
    await tester.pumpWidget(
      MaterialApp(
        theme: ThemeData(useMaterial3: false),
        home: Scaffold(
          body: Builder(
            builder: (BuildContext context) {
              return GestureDetector(
                onTap: () {
                  ScaffoldMessenger.of(context).showSnackBar(
                    SnackBar(
                      content: const Text('I am a snack bar.'),
                      duration: const Duration(seconds: 2),
                      action: SnackBarAction(
                        textColor: Colors.lightBlue,
                        disabledTextColor: Colors.red,
                        label: 'ACTION',
                        onPressed: () {},
                      ),
                    ),
                  );
                },
                child: const Text('X'),
              );
            },
          ),
        ),
      ),
    );

    await tester.tap(find.text('X'));
    await tester.pump(); // start animation
    await tester.pump(const Duration(milliseconds: 750));

    final Element actionTextBox = tester.element(find.text('ACTION'));
    expect(actionTextBox.widget, isA<Text>());
    final Text(:TextStyle? style) = actionTextBox.widget as Text;

    final TextStyle defaultStyle = DefaultTextStyle.of(actionTextBox).style;
    expect(defaultStyle.merge(style).color, Colors.lightBlue);
  });

  testWidgets('Material3 - Snackbar labels can be colored as MaterialColor', (
    WidgetTester tester,
  ) async {
    const MaterialColor usedColor = Colors.teal;

    await tester.pumpWidget(
      MaterialApp(
        theme: ThemeData(useMaterial3: true),
        home: Scaffold(
          body: Builder(
            builder: (BuildContext context) {
              return GestureDetector(
                onTap: () {
                  ScaffoldMessenger.of(context).showSnackBar(
                    SnackBar(
                      content: const Text('I am a snack bar.'),
                      duration: const Duration(seconds: 2),
                      action: SnackBarAction(
                        textColor: usedColor,
                        label: 'ACTION',
                        onPressed: () {},
                      ),
                    ),
                  );
                },
                child: const Text('X'),
              );
            },
          ),
        ),
      ),
    );

    await tester.tap(find.text('X'));
    await tester.pump(); // start animation
    await tester.pump(const Duration(milliseconds: 750));

    final Element actionTextButton = tester.element(find.widgetWithText(TextButton, 'ACTION'));
    final Widget textButton = actionTextButton.widget;
    if (textButton is TextButton) {
      final ButtonStyle buttonStyle = textButton.style!;
      if (buttonStyle.foregroundColor is MaterialStateColor) {
        // Same color when resolved
        expect(buttonStyle.foregroundColor!.resolve(<MaterialState>{}), usedColor);
      } else {
        expect(false, true);
      }
    } else {
      expect(false, true);
    }
  });

  testWidgets('Snackbar labels can be colored as MaterialStateColor (Material 3)', (
    WidgetTester tester,
  ) async {
    const _TestMaterialStateColor usedColor = _TestMaterialStateColor();

    await tester.pumpWidget(
      MaterialApp(
        theme: ThemeData(useMaterial3: true),
        home: Scaffold(
          body: Builder(
            builder: (BuildContext context) {
              return GestureDetector(
                onTap: () {
                  ScaffoldMessenger.of(context).showSnackBar(
                    SnackBar(
                      content: const Text('I am a snack bar.'),
                      duration: const Duration(seconds: 2),
                      action: SnackBarAction(
                        textColor: usedColor,
                        label: 'ACTION',
                        onPressed: () {},
                      ),
                    ),
                  );
                },
                child: const Text('X'),
              );
            },
          ),
        ),
      ),
    );

    await tester.tap(find.text('X'));
    await tester.pump(); // start animation
    await tester.pump(const Duration(milliseconds: 750));

    final Element actionTextButton = tester.element(find.widgetWithText(TextButton, 'ACTION'));
    final Widget textButton = actionTextButton.widget;
    if (textButton is TextButton) {
      final ButtonStyle buttonStyle = textButton.style!;
      if (buttonStyle.foregroundColor is MaterialStateColor) {
        // Exactly the same object
        expect(buttonStyle.foregroundColor, usedColor);
      } else {
        expect(false, true);
      }
    } else {
      expect(false, true);
    }
  });

  testWidgets('Material2 - SnackBar button text alignment', (WidgetTester tester) async {
    await tester.pumpWidget(
      MaterialApp(
        theme: ThemeData(useMaterial3: false),
        home: MediaQuery(
          data: const MediaQueryData(
            padding: EdgeInsets.only(left: 10.0, top: 20.0, right: 30.0, bottom: 40.0),
          ),
          child: Scaffold(
            body: Builder(
              builder: (BuildContext context) {
                return GestureDetector(
                  onTap: () {
                    ScaffoldMessenger.of(context).showSnackBar(
                      SnackBar(
                        content: const Text('I am a snack bar.'),
                        duration: const Duration(seconds: 2),
                        action: SnackBarAction(label: 'ACTION', onPressed: () {}),
                      ),
                    );
                  },
                  child: const Text('X'),
                );
              },
            ),
          ),
        ),
      ),
    );
    await tester.tap(find.text('X'));
    await tester.pump(); // start animation
    await tester.pump(const Duration(milliseconds: 750)); // Animation last frame.

    final Offset textBottomLeft = tester.getBottomLeft(find.text('I am a snack bar.'));
    final Offset textBottomRight = tester.getBottomRight(find.text('I am a snack bar.'));
    final Offset actionTextBottomLeft = tester.getBottomLeft(find.text('ACTION'));
    final Offset actionTextBottomRight = tester.getBottomRight(find.text('ACTION'));
    final Offset snackBarBottomLeft = tester.getBottomLeft(find.byType(SnackBar));
    final Offset snackBarBottomRight = tester.getBottomRight(find.byType(SnackBar));

    expect(textBottomLeft.dx - snackBarBottomLeft.dx, 24.0 + 10.0); // margin + left padding
    expect(snackBarBottomLeft.dy - textBottomLeft.dy, 17.0 + 40.0); // margin + bottom padding
    expect(actionTextBottomLeft.dx - textBottomRight.dx, 24.0 + 12.0); // action padding + margin
    expect(
      snackBarBottomRight.dx - actionTextBottomRight.dx,
      24.0 + 12.0 + 30.0,
    ); // action (padding + margin) + right padding
    expect(
      snackBarBottomRight.dy - actionTextBottomRight.dy,
      17.0 + 40.0,
    ); // margin + bottom padding
  });

  testWidgets('Material3 - SnackBar button text alignment', (WidgetTester tester) async {
    await tester.pumpWidget(
      MaterialApp(
        theme: ThemeData(useMaterial3: true),
        home: MediaQuery(
          data: const MediaQueryData(
            padding: EdgeInsets.only(left: 10.0, top: 20.0, right: 30.0, bottom: 40.0),
          ),
          child: Scaffold(
            body: Builder(
              builder: (BuildContext context) {
                return GestureDetector(
                  onTap: () {
                    ScaffoldMessenger.of(context).showSnackBar(
                      SnackBar(
                        content: const Text('I am a snack bar.'),
                        duration: const Duration(seconds: 2),
                        action: SnackBarAction(label: 'ACTION', onPressed: () {}),
                      ),
                    );
                  },
                  child: const Text('X'),
                );
              },
            ),
          ),
        ),
      ),
    );
    await tester.tap(find.text('X'));
    await tester.pump(); // start animation
    await tester.pump(const Duration(milliseconds: 750)); // Animation last frame.

    final Offset textBottomLeft = tester.getBottomLeft(find.text('I am a snack bar.'));
    final Offset textBottomRight = tester.getBottomRight(find.text('I am a snack bar.'));
    final Offset actionTextBottomLeft = tester.getBottomLeft(find.text('ACTION'));
    final Offset actionTextBottomRight = tester.getBottomRight(find.text('ACTION'));
    final Offset snackBarBottomLeft = tester.getBottomLeft(find.byType(SnackBar));
    final Offset snackBarBottomRight = tester.getBottomRight(find.byType(SnackBar));

    expect(textBottomLeft.dx - snackBarBottomLeft.dx, 24.0 + 10.0); // margin + left padding
    expect(snackBarBottomLeft.dy - textBottomLeft.dy, 14.0 + 40.0); // margin + bottom padding
    expect(actionTextBottomLeft.dx - textBottomRight.dx, 24.0 + 12.0); // action padding + margin
    expect(
      snackBarBottomRight.dx - actionTextBottomRight.dx,
      24.0 + 12.0 + 30.0,
    ); // action (padding + margin) + right padding
    expect(
      snackBarBottomRight.dy - actionTextBottomRight.dy,
      14.0 + 40.0,
    ); // margin + bottom padding
  });

  testWidgets(
    'Material2 - Custom padding between SnackBar and its contents when set to SnackBarBehavior.fixed',
    (WidgetTester tester) async {
      await tester.pumpWidget(
        MaterialApp(
          theme: ThemeData(useMaterial3: false),
          home: MediaQuery(
            data: const MediaQueryData(
              padding: EdgeInsets.only(left: 10.0, top: 20.0, right: 30.0, bottom: 40.0),
            ),
            child: Scaffold(
              bottomNavigationBar: BottomNavigationBar(
                items: const <BottomNavigationBarItem>[
                  BottomNavigationBarItem(icon: Icon(Icons.favorite), label: 'Animutation'),
                  BottomNavigationBarItem(icon: Icon(Icons.block), label: 'Zombo.com'),
                ],
              ),
              body: Builder(
                builder: (BuildContext context) {
                  return GestureDetector(
                    onTap: () {
                      ScaffoldMessenger.of(context).showSnackBar(
                        SnackBar(
                          content: const Text('I am a snack bar.'),
                          duration: const Duration(seconds: 2),
                          action: SnackBarAction(label: 'ACTION', onPressed: () {}),
                        ),
                      );
                    },
                    child: const Text('X'),
                  );
                },
              ),
            ),
          ),
        ),
      );
      await tester.tap(find.text('X'));
      await tester.pump(); // start animation
      await tester.pump(const Duration(milliseconds: 750)); // Animation last frame.

      final Offset textBottomLeft = tester.getBottomLeft(find.text('I am a snack bar.'));
      final Offset textBottomRight = tester.getBottomRight(find.text('I am a snack bar.'));
      final Offset actionTextBottomLeft = tester.getBottomLeft(find.text('ACTION'));
      final Offset actionTextBottomRight = tester.getBottomRight(find.text('ACTION'));
      final Offset snackBarBottomLeft = tester.getBottomLeft(find.byType(SnackBar));
      final Offset snackBarBottomRight = tester.getBottomRight(find.byType(SnackBar));

      expect(textBottomLeft.dx - snackBarBottomLeft.dx, 24.0 + 10.0); // margin + left padding
      expect(snackBarBottomLeft.dy - textBottomLeft.dy, 17.0); // margin (with no bottom padding)
      expect(actionTextBottomLeft.dx - textBottomRight.dx, 24.0 + 12.0); // action padding + margin
      expect(
        snackBarBottomRight.dx - actionTextBottomRight.dx,
        24.0 + 12.0 + 30.0,
      ); // action (padding + margin) + right padding
      expect(
        snackBarBottomRight.dy - actionTextBottomRight.dy,
        17.0,
      ); // margin (with no bottom padding)
    },
  );

  testWidgets(
    'Material3 - Custom padding between SnackBar and its contents when set to SnackBarBehavior.fixed',
    (WidgetTester tester) async {
      await tester.pumpWidget(
        MaterialApp(
          theme: ThemeData(useMaterial3: true),
          home: MediaQuery(
            data: const MediaQueryData(
              padding: EdgeInsets.only(left: 10.0, top: 20.0, right: 30.0, bottom: 40.0),
            ),
            child: Scaffold(
              bottomNavigationBar: BottomNavigationBar(
                items: const <BottomNavigationBarItem>[
                  BottomNavigationBarItem(icon: Icon(Icons.favorite), label: 'Animutation'),
                  BottomNavigationBarItem(icon: Icon(Icons.block), label: 'Zombo.com'),
                ],
              ),
              body: Builder(
                builder: (BuildContext context) {
                  return GestureDetector(
                    onTap: () {
                      ScaffoldMessenger.of(context).showSnackBar(
                        SnackBar(
                          content: const Text('I am a snack bar.'),
                          duration: const Duration(seconds: 2),
                          action: SnackBarAction(label: 'ACTION', onPressed: () {}),
                        ),
                      );
                    },
                    child: const Text('X'),
                  );
                },
              ),
            ),
          ),
        ),
      );
      await tester.tap(find.text('X'));
      await tester.pump(); // start animation
      await tester.pump(const Duration(milliseconds: 750)); // Animation last frame.

      final Offset textBottomLeft = tester.getBottomLeft(find.text('I am a snack bar.'));
      final Offset textBottomRight = tester.getBottomRight(find.text('I am a snack bar.'));
      final Offset actionTextBottomLeft = tester.getBottomLeft(find.text('ACTION'));
      final Offset actionTextBottomRight = tester.getBottomRight(find.text('ACTION'));
      final Offset snackBarBottomLeft = tester.getBottomLeft(find.byType(SnackBar));
      final Offset snackBarBottomRight = tester.getBottomRight(find.byType(SnackBar));

      expect(textBottomLeft.dx - snackBarBottomLeft.dx, 24.0 + 10.0); // margin + left padding
      expect(snackBarBottomLeft.dy - textBottomLeft.dy, 14.0); // margin (with no bottom padding)
      expect(actionTextBottomLeft.dx - textBottomRight.dx, 24.0 + 12.0); // action padding + margin
      expect(
        snackBarBottomRight.dx - actionTextBottomRight.dx,
        24.0 + 12.0 + 30.0,
      ); // action (padding + margin) + right padding
      expect(
        snackBarBottomRight.dy - actionTextBottomRight.dy,
        14.0,
      ); // margin (with no bottom padding)
    },
  );

  testWidgets('SnackBar should push FloatingActionButton above', (WidgetTester tester) async {
    await tester.pumpWidget(
      MaterialApp(
        home: MediaQuery(
          data: const MediaQueryData(
            padding: EdgeInsets.only(left: 10.0, top: 20.0, right: 30.0, bottom: 40.0),
          ),
          child: Scaffold(
            floatingActionButton: FloatingActionButton(
              child: const Icon(Icons.send),
              onPressed: () {},
            ),
            body: Builder(
              builder: (BuildContext context) {
                return GestureDetector(
                  onTap: () {
                    ScaffoldMessenger.of(context).showSnackBar(
                      SnackBar(
                        content: const Text('I am a snack bar.'),
                        duration: const Duration(seconds: 2),
                        action: SnackBarAction(label: 'ACTION', onPressed: () {}),
                      ),
                    );
                  },
                  child: const Text('X'),
                );
              },
            ),
          ),
        ),
      ),
    );

    // Get the Rect of the FAB to compare after the SnackBar appears.
    final Rect originalFabRect = tester.getRect(find.byType(FloatingActionButton));

    await tester.tap(find.text('X'));
    await tester.pump(); // start animation
    await tester.pump(const Duration(milliseconds: 750)); // Animation last frame.

    final Rect fabRect = tester.getRect(find.byType(FloatingActionButton));

    // FAB should shift upwards after SnackBar appears.
    expect(fabRect.center.dy, lessThan(originalFabRect.center.dy));

    final Offset snackBarTopRight = tester.getTopRight(find.byType(SnackBar));

    // FAB's surrounding padding is set to [kFloatingActionButtonMargin] in floating_action_button_location.dart by default.
    const int defaultFabPadding = 16;

    // FAB should be positioned above the SnackBar by the default padding.
    expect(fabRect.bottomRight.dy, snackBarTopRight.dy - defaultFabPadding);
  });

  testWidgets('Material2 - Floating SnackBar button text alignment', (WidgetTester tester) async {
    await tester.pumpWidget(
      MaterialApp(
        theme: ThemeData(
          useMaterial3: false,
          snackBarTheme: const SnackBarThemeData(behavior: SnackBarBehavior.floating),
        ),
        home: MediaQuery(
          data: const MediaQueryData(
            padding: EdgeInsets.only(left: 10.0, top: 20.0, right: 30.0, bottom: 40.0),
          ),
          child: Scaffold(
            body: Builder(
              builder: (BuildContext context) {
                return GestureDetector(
                  onTap: () {
                    ScaffoldMessenger.of(context).showSnackBar(
                      SnackBar(
                        content: const Text('I am a snack bar.'),
                        duration: const Duration(seconds: 2),
                        action: SnackBarAction(label: 'ACTION', onPressed: () {}),
                      ),
                    );
                  },
                  child: const Text('X'),
                );
              },
            ),
          ),
        ),
      ),
    );
    await tester.tap(find.text('X'));
    await tester.pump(); // start animation
    await tester.pump(const Duration(milliseconds: 750)); // Animation last frame.

    final Offset textBottomLeft = tester.getBottomLeft(find.text('I am a snack bar.'));
    final Offset textBottomRight = tester.getBottomRight(find.text('I am a snack bar.'));
    final Offset actionTextBottomLeft = tester.getBottomLeft(find.text('ACTION'));
    final Offset actionTextBottomRight = tester.getBottomRight(find.text('ACTION'));
    final Offset snackBarBottomLeft = tester.getBottomLeft(find.byType(SnackBar));
    final Offset snackBarBottomRight = tester.getBottomRight(find.byType(SnackBar));

    expect(textBottomLeft.dx - snackBarBottomLeft.dx, 31.0 + 10.0); // margin + left padding
    expect(snackBarBottomLeft.dy - textBottomLeft.dy, 27.0); // margin (with no bottom padding)
    expect(actionTextBottomLeft.dx - textBottomRight.dx, 16.0 + 8.0); // action padding + margin
    expect(
      snackBarBottomRight.dx - actionTextBottomRight.dx,
      31.0 + 30.0 + 8.0,
    ); // margin + right (padding + margin)
    expect(
      snackBarBottomRight.dy - actionTextBottomRight.dy,
      27.0,
    ); // margin (with no bottom padding)
  });

  testWidgets('Material3 - Floating SnackBar button text alignment', (WidgetTester tester) async {
    await tester.pumpWidget(
      MaterialApp(
        theme: ThemeData(
          useMaterial3: true,
          snackBarTheme: const SnackBarThemeData(behavior: SnackBarBehavior.floating),
        ),
        home: MediaQuery(
          data: const MediaQueryData(
            padding: EdgeInsets.only(left: 10.0, top: 20.0, right: 30.0, bottom: 40.0),
          ),
          child: Scaffold(
            body: Builder(
              builder: (BuildContext context) {
                return GestureDetector(
                  onTap: () {
                    ScaffoldMessenger.of(context).showSnackBar(
                      SnackBar(
                        content: const Text('I am a snack bar.'),
                        duration: const Duration(seconds: 2),
                        action: SnackBarAction(label: 'ACTION', onPressed: () {}),
                      ),
                    );
                  },
                  child: const Text('X'),
                );
              },
            ),
          ),
        ),
      ),
    );
    await tester.tap(find.text('X'));
    await tester.pump(); // start animation
    await tester.pump(const Duration(milliseconds: 750)); // Animation last frame.

    final Offset textBottomLeft = tester.getBottomLeft(find.text('I am a snack bar.'));
    final Offset textBottomRight = tester.getBottomRight(find.text('I am a snack bar.'));
    final Offset actionTextBottomLeft = tester.getBottomLeft(find.text('ACTION'));
    final Offset actionTextBottomRight = tester.getBottomRight(find.text('ACTION'));
    final Offset snackBarBottomLeft = tester.getBottomLeft(find.byType(SnackBar));
    final Offset snackBarBottomRight = tester.getBottomRight(find.byType(SnackBar));

    expect(textBottomLeft.dx - snackBarBottomLeft.dx, 31.0 + 10.0); // margin + left padding
    expect(snackBarBottomLeft.dy - textBottomLeft.dy, 24.0); // margin (with no bottom padding)
    expect(actionTextBottomLeft.dx - textBottomRight.dx, 16.0 + 8.0); // action padding + margin
    expect(
      snackBarBottomRight.dx - actionTextBottomRight.dx,
      31.0 + 30.0 + 8.0,
    ); // margin + right (padding + margin)
    expect(
      snackBarBottomRight.dy - actionTextBottomRight.dy,
      24.0,
    ); // margin (with no bottom padding)
  });

  testWidgets(
    'Material2 - Custom padding between SnackBar and its contents when set to SnackBarBehavior.floating',
    (WidgetTester tester) async {
      await tester.pumpWidget(
        MaterialApp(
          theme: ThemeData(
            useMaterial3: false,
            snackBarTheme: const SnackBarThemeData(behavior: SnackBarBehavior.floating),
          ),
          home: MediaQuery(
            data: const MediaQueryData(
              padding: EdgeInsets.only(left: 10.0, top: 20.0, right: 30.0, bottom: 40.0),
            ),
            child: Scaffold(
              bottomNavigationBar: BottomNavigationBar(
                items: const <BottomNavigationBarItem>[
                  BottomNavigationBarItem(icon: Icon(Icons.favorite), label: 'Animutation'),
                  BottomNavigationBarItem(icon: Icon(Icons.block), label: 'Zombo.com'),
                ],
              ),
              body: Builder(
                builder: (BuildContext context) {
                  return GestureDetector(
                    onTap: () {
                      ScaffoldMessenger.of(context).showSnackBar(
                        SnackBar(
                          content: const Text('I am a snack bar.'),
                          duration: const Duration(seconds: 2),
                          action: SnackBarAction(label: 'ACTION', onPressed: () {}),
                        ),
                      );
                    },
                    child: const Text('X'),
                  );
                },
              ),
            ),
          ),
        ),
      );
      await tester.tap(find.text('X'));
      await tester.pump(); // start animation
      await tester.pump(const Duration(milliseconds: 750)); // Animation last frame.

      final Offset textBottomLeft = tester.getBottomLeft(find.text('I am a snack bar.'));
      final Offset textBottomRight = tester.getBottomRight(find.text('I am a snack bar.'));
      final Offset actionTextBottomLeft = tester.getBottomLeft(find.text('ACTION'));
      final Offset actionTextBottomRight = tester.getBottomRight(find.text('ACTION'));
      final Offset snackBarBottomLeft = tester.getBottomLeft(find.byType(SnackBar));
      final Offset snackBarBottomRight = tester.getBottomRight(find.byType(SnackBar));

      expect(textBottomLeft.dx - snackBarBottomLeft.dx, 31.0 + 10.0); // margin + left padding
      expect(snackBarBottomLeft.dy - textBottomLeft.dy, 27.0); // margin (with no bottom padding)
      expect(actionTextBottomLeft.dx - textBottomRight.dx, 16.0 + 8.0); // action (margin + padding)
      expect(
        snackBarBottomRight.dx - actionTextBottomRight.dx,
        31.0 + 30.0 + 8.0,
      ); // margin + right (padding + margin)
      expect(
        snackBarBottomRight.dy - actionTextBottomRight.dy,
        27.0,
      ); // margin (with no bottom padding)
    },
  );

  testWidgets(
    'Material3 - Custom padding between SnackBar and its contents when set to SnackBarBehavior.floating',
    (WidgetTester tester) async {
      await tester.pumpWidget(
        MaterialApp(
          theme: ThemeData(
            useMaterial3: true,
            snackBarTheme: const SnackBarThemeData(behavior: SnackBarBehavior.floating),
          ),
          home: MediaQuery(
            data: const MediaQueryData(
              padding: EdgeInsets.only(left: 10.0, top: 20.0, right: 30.0, bottom: 40.0),
            ),
            child: Scaffold(
              bottomNavigationBar: BottomNavigationBar(
                items: const <BottomNavigationBarItem>[
                  BottomNavigationBarItem(icon: Icon(Icons.favorite), label: 'Animutation'),
                  BottomNavigationBarItem(icon: Icon(Icons.block), label: 'Zombo.com'),
                ],
              ),
              body: Builder(
                builder: (BuildContext context) {
                  return GestureDetector(
                    onTap: () {
                      ScaffoldMessenger.of(context).showSnackBar(
                        SnackBar(
                          content: const Text('I am a snack bar.'),
                          duration: const Duration(seconds: 2),
                          action: SnackBarAction(label: 'ACTION', onPressed: () {}),
                        ),
                      );
                    },
                    child: const Text('X'),
                  );
                },
              ),
            ),
          ),
        ),
      );
      await tester.tap(find.text('X'));
      await tester.pump(); // start animation
      await tester.pump(const Duration(milliseconds: 750)); // Animation last frame.

      final Offset textBottomLeft = tester.getBottomLeft(find.text('I am a snack bar.'));
      final Offset textBottomRight = tester.getBottomRight(find.text('I am a snack bar.'));
      final Offset actionTextBottomLeft = tester.getBottomLeft(find.text('ACTION'));
      final Offset actionTextBottomRight = tester.getBottomRight(find.text('ACTION'));
      final Offset snackBarBottomLeft = tester.getBottomLeft(find.byType(SnackBar));
      final Offset snackBarBottomRight = tester.getBottomRight(find.byType(SnackBar));

      expect(textBottomLeft.dx - snackBarBottomLeft.dx, 31.0 + 10.0); // margin + left padding
      expect(snackBarBottomLeft.dy - textBottomLeft.dy, 24.0); // margin (with no bottom padding)
      expect(actionTextBottomLeft.dx - textBottomRight.dx, 16.0 + 8.0); // action (margin + padding)
      expect(
        snackBarBottomRight.dx - actionTextBottomRight.dx,
        31.0 + 30.0 + 8.0,
      ); // margin + right (padding + margin)
      expect(
        snackBarBottomRight.dy - actionTextBottomRight.dy,
        24.0,
      ); // margin (with no bottom padding)
    },
  );

  testWidgets('SnackBarClosedReason', (WidgetTester tester) async {
    final GlobalKey<ScaffoldMessengerState> scaffoldMessengerKey =
        GlobalKey<ScaffoldMessengerState>();
    bool actionPressed = false;
    SnackBarClosedReason? closedReason;

    await tester.pumpWidget(
      MaterialApp(
        scaffoldMessengerKey: scaffoldMessengerKey,
        home: Scaffold(
          body: Builder(
            builder: (BuildContext context) {
              return GestureDetector(
                onTap: () {
                  ScaffoldMessenger.of(context)
                      .showSnackBar(
                        SnackBar(
                          content: const Text('snack'),
                          duration: const Duration(seconds: 2),
                          action: SnackBarAction(
                            label: 'ACTION',
                            onPressed: () {
                              actionPressed = true;
                            },
                          ),
                        ),
                      )
                      .closed
                      .then<void>((SnackBarClosedReason reason) {
                        closedReason = reason;
                      });
                },
                child: const Text('X'),
              );
            },
          ),
        ),
      ),
    );

    // Pop up the snack bar and then press its action button.
    await tester.tap(find.text('X'));
    await tester.pump(); // start animation
    await tester.pump(const Duration(milliseconds: 750));
    expect(actionPressed, isFalse);
    await tester.tap(find.text('ACTION'));
    expect(actionPressed, isTrue);
    // Closed reason is only set when the animation is complete.
    await tester.pump(const Duration(milliseconds: 250));
    expect(closedReason, isNull);
    // Wait for animation to complete.
    await tester.pumpAndSettle(const Duration(seconds: 1));
    expect(closedReason, equals(SnackBarClosedReason.action));

    // Pop up the snack bar and then swipe downwards to dismiss it.
    await tester.tap(find.text('X'));
    await tester.pump(const Duration(milliseconds: 750));
    await tester.pump(const Duration(milliseconds: 750));
    await tester.drag(find.text('snack'), const Offset(0.0, 50.0));
    await tester.pumpAndSettle(const Duration(seconds: 1));
    expect(closedReason, equals(SnackBarClosedReason.swipe));

    // Pop up the snack bar and then remove it.
    await tester.tap(find.text('X'));
    await tester.pump(const Duration(milliseconds: 750));
    scaffoldMessengerKey.currentState!.removeCurrentSnackBar();
    await tester.pumpAndSettle(const Duration(seconds: 1));
    expect(closedReason, equals(SnackBarClosedReason.remove));

    // Pop up the snack bar and then hide it.
    await tester.tap(find.text('X'));
    await tester.pump(const Duration(milliseconds: 750));
    scaffoldMessengerKey.currentState!.hideCurrentSnackBar();
    await tester.pumpAndSettle(const Duration(seconds: 1));
    expect(closedReason, equals(SnackBarClosedReason.hide));

    // Pop up the snack bar and then let it time out.
    await tester.tap(find.text('X'));
    await tester.pump(const Duration(milliseconds: 750));
    await tester.pump(const Duration(milliseconds: 750));
    await tester.pump(const Duration(milliseconds: 1500));
    await tester.pump(); // begin animation
    await tester.pumpAndSettle(const Duration(seconds: 1));
    expect(closedReason, equals(SnackBarClosedReason.timeout));
  });

  testWidgets('accessible navigation behavior with action', (WidgetTester tester) async {
    final GlobalKey<ScaffoldState> scaffoldKey = GlobalKey<ScaffoldState>();

    await tester.pumpWidget(
      MaterialApp(
        home: MediaQuery(
          data: const MediaQueryData(accessibleNavigation: true),
          child: ScaffoldMessenger(
            child: Builder(
              builder: (BuildContext context) {
                return Scaffold(
                  key: scaffoldKey,
                  body: GestureDetector(
                    onTap: () {
                      ScaffoldMessenger.of(context).showSnackBar(
                        SnackBar(
                          content: const Text('snack'),
                          duration: const Duration(seconds: 1),
                          action: SnackBarAction(label: 'ACTION', onPressed: () {}),
                        ),
                      );
                    },
                    child: const Text('X'),
                  ),
                );
              },
            ),
          ),
        ),
      ),
    );
    await tester.tap(find.text('X'));
    await tester.pump();
    // Find action immediately
    expect(find.text('ACTION'), findsOneWidget);
    // Snackbar doesn't close
    await tester.pump(const Duration(seconds: 10));
    expect(find.text('ACTION'), findsOneWidget);
    await tester.tap(find.text('ACTION'));
    await tester.pump();
    // Snackbar closes immediately
    expect(find.text('ACTION'), findsNothing);
  });

  testWidgets('contributes dismiss semantics', (WidgetTester tester) async {
    final SemanticsHandle handle = tester.ensureSemantics();
    final GlobalKey<ScaffoldState> scaffoldKey = GlobalKey<ScaffoldState>();

    await tester.pumpWidget(
      MaterialApp(
        home: MediaQuery(
          data: const MediaQueryData(accessibleNavigation: true),
          child: ScaffoldMessenger(
            child: Builder(
              builder: (BuildContext context) {
                return Scaffold(
                  key: scaffoldKey,
                  body: GestureDetector(
                    onTap: () {
                      ScaffoldMessenger.of(context).showSnackBar(
                        SnackBar(
                          content: const Text('snack'),
                          duration: const Duration(seconds: 1),
                          action: SnackBarAction(label: 'ACTION', onPressed: () {}),
                        ),
                      );
                    },
                    child: const Text('X'),
                  ),
                );
              },
            ),
          ),
        ),
      ),
    );
    await tester.tap(find.text('X'));
    await tester.pumpAndSettle();

    expect(
      tester.getSemantics(find.text('snack')),
      matchesSemantics(
        isLiveRegion: true,
        hasDismissAction: true,
        hasScrollDownAction: true,
        hasScrollUpAction: true,
        label: 'snack',
        textDirection: TextDirection.ltr,
      ),
    );
    handle.dispose();
  });

  testWidgets('SnackBar default display duration test', (WidgetTester tester) async {
    const String helloSnackBar = 'Hello SnackBar';
    const Key tapTarget = Key('tap-target');
    await tester.pumpWidget(
      MaterialApp(
        home: Scaffold(
          body: Builder(
            builder: (BuildContext context) {
              return GestureDetector(
                key: tapTarget,
                onTap: () {
                  ScaffoldMessenger.of(
                    context,
                  ).showSnackBar(const SnackBar(content: Text(helloSnackBar)));
                },
                behavior: HitTestBehavior.opaque,
                child: const SizedBox(height: 100.0, width: 100.0),
              );
            },
          ),
        ),
      ),
    );
    expect(find.text(helloSnackBar), findsNothing);
    await tester.tap(find.byKey(tapTarget));
    expect(find.text(helloSnackBar), findsNothing);
    await tester.pump(); // schedule animation
    expect(find.text(helloSnackBar), findsOneWidget);
    await tester.pump(); // begin animation
    expect(find.text(helloSnackBar), findsOneWidget);
    await tester.pump(
      const Duration(milliseconds: 750),
    ); // 0.75s // animation last frame; four second timer starts here
    expect(find.text(helloSnackBar), findsOneWidget);
    await tester.pump(const Duration(milliseconds: 750)); // 1.50s
    expect(find.text(helloSnackBar), findsOneWidget);
    await tester.pump(const Duration(milliseconds: 750)); // 2.25s
    expect(find.text(helloSnackBar), findsOneWidget);
    await tester.pump(const Duration(milliseconds: 750)); // 3.00s
    expect(find.text(helloSnackBar), findsOneWidget);
    await tester.pump(const Duration(milliseconds: 750)); // 3.75s
    expect(find.text(helloSnackBar), findsOneWidget);
    await tester.pump(
      const Duration(milliseconds: 1000),
    ); // 4.75s // timer triggers to dismiss snackbar, reverse animation is scheduled
    await tester.pump(); // begin animation
    expect(find.text(helloSnackBar), findsOneWidget); // frame 0 of dismiss animation
    await tester.pump(
      const Duration(milliseconds: 750),
    ); // 5.50s // last frame of animation, snackbar removed from build
    expect(find.text(helloSnackBar), findsNothing);
  });

  testWidgets('SnackBar handles updates to accessibleNavigation', (WidgetTester tester) async {
    Future<void> boilerplate({required bool accessibleNavigation}) {
      return tester.pumpWidget(
        MaterialApp(
          home: MediaQuery(
            data: MediaQueryData(accessibleNavigation: accessibleNavigation),
            child: Scaffold(
              body: Builder(
                builder: (BuildContext context) {
                  return GestureDetector(
                    onTap: () {
                      ScaffoldMessenger.of(context).showSnackBar(
                        SnackBar(
                          content: const Text('test'),
                          action: SnackBarAction(label: 'foo', onPressed: () {}),
                        ),
                      );
                    },
                    behavior: HitTestBehavior.opaque,
                    child: const Text('X'),
                  );
                },
              ),
            ),
          ),
        ),
      );
    }

    await boilerplate(accessibleNavigation: false);
    expect(find.text('test'), findsNothing);
    await tester.tap(find.text('X'));
    await tester.pump(); // schedule animation
    expect(find.text('test'), findsOneWidget);
    await tester.pump(); // begin animation
    await tester.pump(const Duration(milliseconds: 4750)); // 4.75s
    expect(find.text('test'), findsOneWidget);

    // Enabled accessible navigation
    await boilerplate(accessibleNavigation: true);

    await tester.pump(const Duration(milliseconds: 4000)); // 8.75s
    await tester.pump();
    expect(find.text('test'), findsOneWidget);

    // disable accessible navigation
    await boilerplate(accessibleNavigation: false);
    await tester.pumpAndSettle(const Duration(milliseconds: 5750));

    expect(find.text('test'), findsNothing);
  });

  testWidgets('Snackbar calls onVisible once', (WidgetTester tester) async {
    const Key tapTarget = Key('tap-target');
    int called = 0;
    await tester.pumpWidget(
      MaterialApp(
        home: Scaffold(
          body: Builder(
            builder: (BuildContext context) {
              return GestureDetector(
                key: tapTarget,
                onTap: () {
                  ScaffoldMessenger.of(context).showSnackBar(
                    SnackBar(
                      content: const Text('hello'),
                      duration: const Duration(seconds: 1),
                      onVisible: () {
                        called += 1;
                      },
                    ),
                  );
                },
                behavior: HitTestBehavior.opaque,
                child: const SizedBox(height: 100.0, width: 100.0),
              );
            },
          ),
        ),
      ),
    );

    await tester.tap(find.byKey(tapTarget));
    await tester.pump(); // start animation
    await tester.pumpAndSettle();

    expect(find.text('hello'), findsOneWidget);
    expect(called, 1);
  });

  testWidgets('Snackbar does not call onVisible when it is queued', (WidgetTester tester) async {
    const Key tapTarget = Key('tap-target');
    int called = 0;
    await tester.pumpWidget(
      MaterialApp(
        home: Scaffold(
          body: Builder(
            builder: (BuildContext context) {
              return GestureDetector(
                key: tapTarget,
                onTap: () {
                  ScaffoldMessenger.of(context).showSnackBar(
                    SnackBar(
                      content: const Text('hello'),
                      duration: const Duration(seconds: 1),
                      onVisible: () {
                        called += 1;
                      },
                    ),
                  );
                  ScaffoldMessenger.of(context).showSnackBar(
                    SnackBar(
                      content: const Text('hello 2'),
                      duration: const Duration(seconds: 1),
                      onVisible: () {
                        called += 1;
                      },
                    ),
                  );
                },
                behavior: HitTestBehavior.opaque,
                child: const SizedBox(height: 100.0, width: 100.0),
              );
            },
          ),
        ),
      ),
    );

    await tester.tap(find.byKey(tapTarget));
    await tester.pump(); // start animation
    await tester.pumpAndSettle();

    expect(find.text('hello'), findsOneWidget);
    expect(called, 1);
  });

  group('SnackBar position', () {
    for (final SnackBarBehavior behavior in SnackBarBehavior.values) {
      final SnackBar snackBar = SnackBar(content: const Text('SnackBar text'), behavior: behavior);

      testWidgets('$behavior should align SnackBar with the bottom of Scaffold '
          'when Scaffold has no other elements', (WidgetTester tester) async {
        await tester.pumpWidget(MaterialApp(home: Scaffold(body: Container())));

        final ScaffoldMessengerState scaffoldMessengerState = tester.state(
          find.byType(ScaffoldMessenger),
        );
        scaffoldMessengerState.showSnackBar(snackBar);

        await tester.pumpAndSettle(); // Have the SnackBar fully animate out.

        final Offset snackBarBottomRight = tester.getBottomRight(find.byType(SnackBar));
        final Offset scaffoldBottomRight = tester.getBottomRight(find.byType(Scaffold));

        expect(snackBarBottomRight, equals(scaffoldBottomRight));

        final Offset snackBarBottomLeft = tester.getBottomLeft(find.byType(SnackBar));
        final Offset scaffoldBottomLeft = tester.getBottomLeft(find.byType(Scaffold));

        expect(snackBarBottomLeft, equals(scaffoldBottomLeft));
      });

      testWidgets('$behavior should align SnackBar with the top of BottomNavigationBar '
          'when Scaffold has no FloatingActionButton', (WidgetTester tester) async {
        final UniqueKey boxKey = UniqueKey();
        await tester.pumpWidget(
          MaterialApp(
            home: Scaffold(
              body: Container(),
              bottomNavigationBar: SizedBox(key: boxKey, width: 800, height: 60),
            ),
          ),
        );

        final ScaffoldMessengerState scaffoldMessengerState = tester.state(
          find.byType(ScaffoldMessenger),
        );
        scaffoldMessengerState.showSnackBar(snackBar);

        await tester.pumpAndSettle(); // Have the SnackBar fully animate out.

        final Offset snackBarBottomRight = tester.getBottomRight(find.byType(SnackBar));
        final Offset bottomNavigationBarTopRight = tester.getTopRight(find.byKey(boxKey));

        expect(snackBarBottomRight, equals(bottomNavigationBarTopRight));

        final Offset snackBarBottomLeft = tester.getBottomLeft(find.byType(SnackBar));
        final Offset bottomNavigationBarTopLeft = tester.getTopLeft(find.byKey(boxKey));

        expect(snackBarBottomLeft, equals(bottomNavigationBarTopLeft));
      });
    }

    testWidgets('Padding of ${SnackBarBehavior.fixed} is not consumed by viewInsets', (
      WidgetTester tester,
    ) async {
      final Widget child = MaterialApp(
        home: Scaffold(
          resizeToAvoidBottomInset: false,
          floatingActionButton: FloatingActionButton(
            child: const Icon(Icons.send),
            onPressed: () {},
          ),
          body: Builder(
            builder: (BuildContext context) {
              return GestureDetector(
                onTap: () {
                  ScaffoldMessenger.of(context).showSnackBar(
                    SnackBar(
                      content: const Text('I am a snack bar.'),
                      duration: const Duration(seconds: 2),
                      action: SnackBarAction(label: 'ACTION', onPressed: () {}),
                      behavior: SnackBarBehavior.fixed,
                    ),
                  );
                },
                child: const Text('X'),
              );
            },
          ),
        ),
      );

      await tester.pumpWidget(
        MediaQuery(
          data: const MediaQueryData(padding: EdgeInsets.only(bottom: 20.0)),
          child: child,
        ),
      );
      await tester.tap(find.text('X'));
      await tester.pumpAndSettle(); // Show snackbar
      final Offset initialBottomLeft = tester.getBottomLeft(find.byType(SnackBar));
      final Offset initialBottomRight = tester.getBottomRight(find.byType(SnackBar));
      // Consume bottom padding - as if by the keyboard opening
      await tester.pumpWidget(
        MediaQuery(
          data: const MediaQueryData(
            viewPadding: EdgeInsets.all(20),
            viewInsets: EdgeInsets.all(100),
          ),
          child: child,
        ),
      );
      await tester.tap(find.text('X'));
      await tester.pumpAndSettle(); // Have the SnackBar fully animate out.

      final Offset finalBottomLeft = tester.getBottomLeft(find.byType(SnackBar));
      final Offset finalBottomRight = tester.getBottomRight(find.byType(SnackBar));

      expect(initialBottomLeft, finalBottomLeft);
      expect(initialBottomRight, finalBottomRight);
    });

    testWidgets('${SnackBarBehavior.fixed} should align SnackBar with the bottom of Scaffold '
        'when Scaffold has a FloatingActionButton', (WidgetTester tester) async {
      await tester.pumpWidget(
        MaterialApp(
          home: Scaffold(
            body: Container(),
            floatingActionButton: FloatingActionButton(onPressed: () {}),
          ),
        ),
      );

      final ScaffoldMessengerState scaffoldMessengerState = tester.state(
        find.byType(ScaffoldMessenger),
      );
      scaffoldMessengerState.showSnackBar(
        const SnackBar(content: Text('Snackbar text'), behavior: SnackBarBehavior.fixed),
      );

      await tester.pumpAndSettle(); // Have the SnackBar fully animate out.

      final Offset snackBarBottomRight = tester.getBottomRight(find.byType(SnackBar));
      final Offset scaffoldBottomRight = tester.getBottomRight(find.byType(Scaffold));

      expect(snackBarBottomRight, equals(scaffoldBottomRight));

      final Offset snackBarBottomLeft = tester.getBottomLeft(find.byType(SnackBar));
      final Offset scaffoldBottomLeft = tester.getBottomLeft(find.byType(Scaffold));

      expect(snackBarBottomLeft, equals(scaffoldBottomLeft));
    });

    testWidgets(
      '${SnackBarBehavior.floating} should align SnackBar with the top of FloatingActionButton when Scaffold has a FloatingActionButton',
      (WidgetTester tester) async {
        await tester.pumpWidget(
          MaterialApp(
            home: Scaffold(
              floatingActionButton: FloatingActionButton(
                child: const Icon(Icons.send),
                onPressed: () {},
              ),
              body: Builder(
                builder: (BuildContext context) {
                  return GestureDetector(
                    onTap: () {
                      ScaffoldMessenger.of(context).showSnackBar(
                        SnackBar(
                          content: const Text('I am a snack bar.'),
                          duration: const Duration(seconds: 2),
                          action: SnackBarAction(label: 'ACTION', onPressed: () {}),
                          behavior: SnackBarBehavior.floating,
                        ),
                      );
                    },
                    child: const Text('X'),
                  );
                },
              ),
            ),
          ),
        );
        await tester.tap(find.text('X'));
        await tester.pumpAndSettle(); // Have the SnackBar fully animate out.

        final Offset snackBarBottomLeft = tester.getBottomLeft(find.byType(SnackBar));
        final Offset floatingActionButtonTopLeft = tester.getTopLeft(
          find.byType(FloatingActionButton),
        );

        // Since padding between the SnackBar and the FAB is created by the SnackBar,
        // the bottom offset of the SnackBar should be equal to the top offset of the FAB
        expect(snackBarBottomLeft.dy, floatingActionButtonTopLeft.dy);
      },
    );

    testWidgets(
      '${SnackBarBehavior.floating} should not align SnackBar with the top of FloatingActionButton '
      'when Scaffold has a FloatingActionButton and floatingActionButtonLocation is set to a top position',
      (WidgetTester tester) async {
        Future<void> pumpApp({required FloatingActionButtonLocation fabLocation}) async {
          return tester.pumpWidget(
            MaterialApp(
              home: Scaffold(
                floatingActionButton: FloatingActionButton(
                  child: const Icon(Icons.send),
                  onPressed: () {},
                ),
                floatingActionButtonLocation: fabLocation,
                body: Builder(
                  builder: (BuildContext context) {
                    return GestureDetector(
                      onTap: () {
                        ScaffoldMessenger.of(context).showSnackBar(
                          SnackBar(
                            content: const Text('I am a snack bar.'),
                            duration: const Duration(seconds: 2),
                            action: SnackBarAction(label: 'ACTION', onPressed: () {}),
                            behavior: SnackBarBehavior.floating,
                          ),
                        );
                      },
                      child: const Text('X'),
                    );
                  },
                ),
              ),
            ),
          );
        }

        const List<FloatingActionButtonLocation> topLocations = <FloatingActionButtonLocation>[
          FloatingActionButtonLocation.startTop,
          FloatingActionButtonLocation.centerTop,
          FloatingActionButtonLocation.endTop,
          FloatingActionButtonLocation.miniStartTop,
          FloatingActionButtonLocation.miniCenterTop,
          FloatingActionButtonLocation.miniEndTop,
        ];

        for (final FloatingActionButtonLocation location in topLocations) {
          await pumpApp(fabLocation: location);

          await tester.tap(find.text('X'));
          await tester.pumpAndSettle(); // Have the SnackBar fully animate out.

          final Offset snackBarBottomLeft = tester.getBottomLeft(find.byType(SnackBar));

          expect(snackBarBottomLeft.dy, 600); // Device height is 600.
        }
      },
    );

    testWidgets(
      '${SnackBarBehavior.floating} should align SnackBar with the top of FloatingActionButton '
      'when Scaffold has a FloatingActionButton and floatingActionButtonLocation is not set to a top position',
      (WidgetTester tester) async {
        Future<void> pumpApp({required FloatingActionButtonLocation fabLocation}) async {
          return tester.pumpWidget(
            MaterialApp(
              home: Scaffold(
                floatingActionButton: FloatingActionButton(
                  child: const Icon(Icons.send),
                  onPressed: () {},
                ),
                floatingActionButtonLocation: fabLocation,
                body: Builder(
                  builder: (BuildContext context) {
                    return GestureDetector(
                      onTap: () {
                        ScaffoldMessenger.of(context).showSnackBar(
                          SnackBar(
                            content: const Text('I am a snack bar.'),
                            duration: const Duration(seconds: 2),
                            action: SnackBarAction(label: 'ACTION', onPressed: () {}),
                            behavior: SnackBarBehavior.floating,
                          ),
                        );
                      },
                      child: const Text('X'),
                    );
                  },
                ),
              ),
            ),
          );
        }

        const List<FloatingActionButtonLocation> nonTopLocations = <FloatingActionButtonLocation>[
          FloatingActionButtonLocation.startDocked,
          FloatingActionButtonLocation.startFloat,
          FloatingActionButtonLocation.centerDocked,
          FloatingActionButtonLocation.centerFloat,
          FloatingActionButtonLocation.endContained,
          FloatingActionButtonLocation.endDocked,
          FloatingActionButtonLocation.endFloat,
          FloatingActionButtonLocation.miniStartDocked,
          FloatingActionButtonLocation.miniStartFloat,
          FloatingActionButtonLocation.miniCenterDocked,
          FloatingActionButtonLocation.miniCenterFloat,
          FloatingActionButtonLocation.miniEndDocked,
          FloatingActionButtonLocation.miniEndFloat,
          // Regression test related to https://github.com/flutter/flutter/pull/131303.
          _CustomFloatingActionButtonLocation(),
        ];

        for (final FloatingActionButtonLocation location in nonTopLocations) {
          await pumpApp(fabLocation: location);

          await tester.tap(find.text('X'));
          await tester.pumpAndSettle(); // Have the SnackBar fully animate out.

          final Offset snackBarBottomLeft = tester.getBottomLeft(find.byType(SnackBar));
          final Offset floatingActionButtonTopLeft = tester.getTopLeft(
            find.byType(FloatingActionButton),
          );

          // Since padding between the SnackBar and the FAB is created by the SnackBar,
          // the bottom offset of the SnackBar should be equal to the top offset of the FAB
          expect(snackBarBottomLeft.dy, floatingActionButtonTopLeft.dy);
        }
      },
    );

    testWidgets(
      '${SnackBarBehavior.fixed} should align SnackBar with the top of BottomNavigationBar '
      'when Scaffold has a BottomNavigationBar and FloatingActionButton',
      (WidgetTester tester) async {
        final UniqueKey boxKey = UniqueKey();
        await tester.pumpWidget(
          MaterialApp(
            home: Scaffold(
              body: Container(),
              bottomNavigationBar: SizedBox(key: boxKey, width: 800, height: 60),
              floatingActionButton: FloatingActionButton(onPressed: () {}),
            ),
          ),
        );

        final ScaffoldMessengerState scaffoldMessengerState = tester.state(
          find.byType(ScaffoldMessenger),
        );
        scaffoldMessengerState.showSnackBar(
          const SnackBar(content: Text('SnackBar text'), behavior: SnackBarBehavior.fixed),
        );

        await tester.pumpAndSettle(); // Have the SnackBar fully animate out.

        final Offset snackBarBottomRight = tester.getBottomRight(find.byType(SnackBar));
        final Offset bottomNavigationBarTopRight = tester.getTopRight(find.byKey(boxKey));

        expect(snackBarBottomRight, equals(bottomNavigationBarTopRight));

        final Offset snackBarBottomLeft = tester.getBottomLeft(find.byType(SnackBar));
        final Offset bottomNavigationBarTopLeft = tester.getTopLeft(find.byKey(boxKey));

        expect(snackBarBottomLeft, equals(bottomNavigationBarTopLeft));
      },
    );

    testWidgets(
      '${SnackBarBehavior.floating} should align SnackBar with the top of FloatingActionButton '
      'when Scaffold has BottomNavigationBar and FloatingActionButton',
      (WidgetTester tester) async {
        final UniqueKey boxKey = UniqueKey();
        await tester.pumpWidget(
          MaterialApp(
            home: Scaffold(
              body: Container(),
              bottomNavigationBar: SizedBox(key: boxKey, width: 800, height: 60),
              floatingActionButton: FloatingActionButton(onPressed: () {}),
            ),
          ),
        );

        final ScaffoldMessengerState scaffoldMessengerState = tester.state(
          find.byType(ScaffoldMessenger),
        );
        scaffoldMessengerState.showSnackBar(
          const SnackBar(content: Text('SnackBar text'), behavior: SnackBarBehavior.floating),
        );

        await tester.pumpAndSettle(); // Have the SnackBar fully animate out.

        final Offset snackBarBottomRight = tester.getBottomRight(find.byType(SnackBar));
        final Offset fabTopRight = tester.getTopRight(find.byType(FloatingActionButton));

        expect(snackBarBottomRight.dy, equals(fabTopRight.dy));
      },
    );

    testWidgets(
      '${SnackBarBehavior.floating} should align SnackBar with the top of BottomNavigationBar '
      'when Scaffold has both BottomNavigationBar and FloatingActionButton and '
      'BottomNavigationBar.top is higher than FloatingActionButton.top',
      (WidgetTester tester) async {
        final UniqueKey boxKey = UniqueKey();
        await tester.pumpWidget(
          MaterialApp(
            home: Scaffold(
              body: Container(),
              bottomNavigationBar: SizedBox(key: boxKey, width: 800, height: 200),
              floatingActionButton: FloatingActionButton(onPressed: () {}),
              floatingActionButtonLocation: FloatingActionButtonLocation.endContained,
            ),
          ),
        );

        final ScaffoldMessengerState scaffoldMessengerState = tester.state(
          find.byType(ScaffoldMessenger),
        );
        scaffoldMessengerState.showSnackBar(
          const SnackBar(content: Text('SnackBar text'), behavior: SnackBarBehavior.floating),
        );

        await tester.pumpAndSettle(); // Have the SnackBar fully animate out.

        final Offset snackBarBottomRight = tester.getBottomRight(find.byType(SnackBar));
        final Offset fabTopRight = tester.getTopRight(find.byType(FloatingActionButton));
        final Offset navBarTopRight = tester.getTopRight(find.byKey(boxKey));

        // Test the top of the navigation bar is higher than the top of the floating action button.
        expect(fabTopRight.dy, greaterThan(navBarTopRight.dy));

        expect(snackBarBottomRight.dy, equals(navBarTopRight.dy));
      },
    );

    Future<void> openFloatingSnackBar(WidgetTester tester) async {
      final ScaffoldMessengerState scaffoldMessengerState = tester.state(
        find.byType(ScaffoldMessenger),
      );
      scaffoldMessengerState.showSnackBar(
        const SnackBar(content: Text('SnackBar text'), behavior: SnackBarBehavior.floating),
      );
      await tester.pumpAndSettle(); // Have the SnackBar fully animate out.
    }

    const String offScreenMessage =
        'Floating SnackBar presented off screen.\n'
        'A SnackBar with behavior property set to SnackBarBehavior.floating is fully '
        'or partially off screen because some or all the widgets provided to '
        'Scaffold.floatingActionButton, Scaffold.persistentFooterButtons and '
        'Scaffold.bottomNavigationBar take up too much vertical space.\n'
        'Consider constraining the size of these widgets to allow room for the SnackBar to be visible.';

    testWidgets(
      'Snackbar with SnackBarBehavior.floating will assert when offset too high by a large Scaffold.floatingActionButton',
      (WidgetTester tester) async {
        // Regression test for https://github.com/flutter/flutter/issues/84263
        Future<void> boilerplate({required double? fabHeight}) {
          return tester.pumpWidget(
            MaterialApp(home: Scaffold(floatingActionButton: Container(height: fabHeight))),
          );
        }

        // Run once with a visible SnackBar to compute the empty space above SnackBar.
        const double mediumFabHeight = 100;
        await boilerplate(fabHeight: mediumFabHeight);
        await openFloatingSnackBar(tester);
        expect(tester.takeException(), isNull);
        final double spaceAboveSnackBar = tester.getTopLeft(find.byType(SnackBar)).dy;

        // Run with the Snackbar fully off screen.
        await boilerplate(fabHeight: spaceAboveSnackBar + mediumFabHeight * 2);
        await openFloatingSnackBar(tester);
        AssertionError exception = tester.takeException() as AssertionError;
        expect(exception.message, offScreenMessage);

        // Run with the Snackbar partially off screen.
        await boilerplate(fabHeight: spaceAboveSnackBar + mediumFabHeight + 10);
        await openFloatingSnackBar(tester);
        exception = tester.takeException() as AssertionError;
        expect(exception.message, offScreenMessage);

        // Run with the Snackbar fully visible right on the top of the screen.
        await boilerplate(fabHeight: spaceAboveSnackBar + mediumFabHeight);
        await openFloatingSnackBar(tester);
        expect(tester.takeException(), isNull);
      },
    );

    testWidgets(
      'Material2 - Snackbar with SnackBarBehavior.floating will assert when offset too high by a large Scaffold.persistentFooterButtons',
      (WidgetTester tester) async {
        // Regression test for https://github.com/flutter/flutter/issues/84263
        await tester.pumpWidget(
          MaterialApp(
            theme: ThemeData(useMaterial3: false),
            home: const Scaffold(persistentFooterButtons: <Widget>[SizedBox(height: 1000)]),
          ),
        );

        await openFloatingSnackBar(tester);
        await tester.pumpAndSettle(); // Have the SnackBar fully animate out.

        final AssertionError exception = tester.takeException() as AssertionError;
        expect(exception.message, offScreenMessage);
      },
    );

    testWidgets(
      'Material3 - Snackbar with SnackBarBehavior.floating will assert when offset too high by a large Scaffold.persistentFooterButtons',
      (WidgetTester tester) async {
        // Regression test for https://github.com/flutter/flutter/issues/84263
        await tester.pumpWidget(
          MaterialApp(
            theme: ThemeData(useMaterial3: true),
            home: const Scaffold(persistentFooterButtons: <Widget>[SizedBox(height: 1000)]),
          ),
        );

        final FlutterExceptionHandler? handler = FlutterError.onError;
        final List<String> errorMessages = <String>[];
        FlutterError.onError = (FlutterErrorDetails details) {
          errorMessages.add(details.exceptionAsString());
        };
        addTearDown(() => FlutterError.onError = handler);

        await openFloatingSnackBar(tester);
        await tester.pumpAndSettle(); // Have the SnackBar fully animate out.

        expect(errorMessages.contains(offScreenMessage), isTrue);
      },
    );

    testWidgets(
      'Material2 - Snackbar with SnackBarBehavior.floating will assert when offset too high by a large Scaffold.bottomNavigationBar',
      (WidgetTester tester) async {
        // Regression test for https://github.com/flutter/flutter/issues/84263
        await tester.pumpWidget(
          MaterialApp(
            theme: ThemeData(useMaterial3: false),
            home: const Scaffold(bottomNavigationBar: SizedBox(height: 1000)),
          ),
        );

        await openFloatingSnackBar(tester);
        await tester.pumpAndSettle(); // Have the SnackBar fully animate out.
        final AssertionError exception = tester.takeException() as AssertionError;
        expect(exception.message, offScreenMessage);
      },
    );

    testWidgets(
      'Material3 - Snackbar with SnackBarBehavior.floating will assert when offset too high by a large Scaffold.bottomNavigationBar',
      (WidgetTester tester) async {
        // Regression test for https://github.com/flutter/flutter/issues/84263
        await tester.pumpWidget(
          MaterialApp(
            theme: ThemeData(useMaterial3: true),
            home: const Scaffold(bottomNavigationBar: SizedBox(height: 1000)),
          ),
        );

        final FlutterExceptionHandler? handler = FlutterError.onError;
        final List<String> errorMessages = <String>[];
        FlutterError.onError = (FlutterErrorDetails details) {
          errorMessages.add(details.exceptionAsString());
        };
        addTearDown(() => FlutterError.onError = handler);

        await openFloatingSnackBar(tester);
        await tester.pumpAndSettle(); // Have the SnackBar fully animate out.

        expect(errorMessages.contains(offScreenMessage), isTrue);
      },
    );

    testWidgets('SnackBar has correct end padding when it contains an action with fixed behavior', (
      WidgetTester tester,
    ) async {
      await tester.pumpWidget(
        MaterialApp(
          home: Scaffold(
            body: Builder(
              builder: (BuildContext context) {
                return GestureDetector(
                  onTap: () {
                    ScaffoldMessenger.of(context).showSnackBar(
                      SnackBar(
                        content: const Text('Some content'),
                        behavior: SnackBarBehavior.fixed,
                        action: SnackBarAction(label: 'ACTION', onPressed: () {}),
                      ),
                    );
                  },
                  child: const Text('X'),
                );
              },
            ),
          ),
        ),
      );

      await tester.tap(find.text('X'));
      await tester.pumpAndSettle();

      final Offset snackBarTopRight = tester.getTopRight(find.byType(SnackBar));
      final Offset actionTopRight = tester.getTopRight(find.byType(SnackBarAction));

      expect(snackBarTopRight.dx - actionTopRight.dx, 12.0);
    });

    testWidgets(
      'SnackBar has correct end padding when it contains an action with floating behavior',
      (WidgetTester tester) async {
        await tester.pumpWidget(
          MaterialApp(
            home: Scaffold(
              body: Builder(
                builder: (BuildContext context) {
                  return GestureDetector(
                    onTap: () {
                      ScaffoldMessenger.of(context).showSnackBar(
                        SnackBar(
                          content: const Text('Some content'),
                          behavior: SnackBarBehavior.floating,
                          action: SnackBarAction(label: 'ACTION', onPressed: () {}),
                        ),
                      );
                    },
                    child: const Text('X'),
                  );
                },
              ),
            ),
          ),
        );

        await tester.tap(find.text('X'));
        await tester.pumpAndSettle();
        final Offset snackBarTopRight = tester.getTopRight(find.byType(SnackBar));
        final Offset actionTopRight = tester.getTopRight(find.byType(SnackBarAction));

        expect(
          snackBarTopRight.dx - actionTopRight.dx,
          8.0 + 15.0,
        ); // button margin + horizontal scaffold outside margin
      },
    );

    testWidgets(
      'Material3 - Floating snackbar with custom width is centered when text direction is rtl',
      (WidgetTester tester) async {
        // Regression test for https://github.com/flutter/flutter/issues/140125.
        const double customWidth = 400.0;
        await tester.pumpWidget(
          MaterialApp(
            home: Directionality(
              textDirection: TextDirection.rtl,
              child: Scaffold(
                body: Builder(
                  builder: (BuildContext context) {
                    return GestureDetector(
                      onTap: () {
                        ScaffoldMessenger.of(context).showSnackBar(
                          const SnackBar(
                            behavior: SnackBarBehavior.floating,
                            width: customWidth,
                            content: Text('Feeling super snackish'),
                          ),
                        );
                      },
                      child: const Text('X'),
                    );
                  },
                ),
              ),
            ),
          ),
        );

        await tester.tap(find.text('X'));
        await tester.pump(); // Start animation.
        await tester.pump(const Duration(milliseconds: 750));

        final Finder materialFinder = find.descendant(
          of: find.byType(SnackBar),
          matching: find.byType(Material),
        );
        final Offset snackBarBottomLeft = tester.getBottomLeft(materialFinder);
        final Offset snackBarBottomRight = tester.getBottomRight(materialFinder);
        expect(snackBarBottomLeft.dx, (800 - customWidth) / 2); // Device width is 800.
        expect(snackBarBottomRight.dx, (800 + customWidth) / 2); // Device width is 800.
      },
    );

    testWidgets(
      'Material2 - Floating snackbar with custom width is centered when text direction is rtl',
      (WidgetTester tester) async {
        // Regression test for https://github.com/flutter/flutter/issues/147838.
        const double customWidth = 400.0;
        await tester.pumpWidget(
          MaterialApp(
            theme: ThemeData(useMaterial3: false),
            home: Directionality(
              textDirection: TextDirection.rtl,
              child: Scaffold(
                body: Builder(
                  builder: (BuildContext context) {
                    return GestureDetector(
                      onTap: () {
                        ScaffoldMessenger.of(context).showSnackBar(
                          const SnackBar(
                            behavior: SnackBarBehavior.floating,
                            width: customWidth,
                            content: Text('Feeling super snackish'),
                          ),
                        );
                      },
                      child: const Text('X'),
                    );
                  },
                ),
              ),
            ),
          ),
        );

        await tester.tap(find.text('X'));
        await tester.pump(); // Start animation.
        await tester.pump(const Duration(milliseconds: 750));

        final Finder materialFinder = find.descendant(
          of: find.byType(SnackBar),
          matching: find.byType(Material),
        );
        final Offset snackBarBottomLeft = tester.getBottomLeft(materialFinder);
        final Offset snackBarBottomRight = tester.getBottomRight(materialFinder);
        expect(snackBarBottomLeft.dx, (800 - customWidth) / 2); // Device width is 800.
        expect(snackBarBottomRight.dx, (800 + customWidth) / 2); // Device width is 800.
      },
    );
  });

  testWidgets('SnackBars hero across transitions when using ScaffoldMessenger', (
    WidgetTester tester,
  ) async {
    const String snackBarText = 'hello snackbar';
    const String firstHeader = 'home';
    const String secondHeader = 'second';
    const Key snackTarget = Key('snack-target');
    const Key transitionTarget = Key('transition-target');

    Widget buildApp() {
      return MaterialApp(
        routes: <String, WidgetBuilder>{
          '/': (BuildContext context) {
            return Scaffold(
              appBar: AppBar(title: const Text(firstHeader)),
              body: Center(
                child: ElevatedButton(
                  key: transitionTarget,
                  child: const Text('PUSH'),
                  onPressed: () {
                    Navigator.of(context).pushNamed('/second');
                  },
                ),
              ),
              floatingActionButton: FloatingActionButton(
                key: snackTarget,
                onPressed: () async {
                  ScaffoldMessenger.of(
                    context,
                  ).showSnackBar(const SnackBar(content: Text(snackBarText)));
                },
                child: const Text('X'),
              ),
            );
          },
          '/second':
              (BuildContext context) => Scaffold(appBar: AppBar(title: const Text(secondHeader))),
        },
      );
    }

    await tester.pumpWidget(buildApp());

    expect(find.text(snackBarText), findsNothing);
    expect(find.text(firstHeader), findsOneWidget);
    expect(find.text(secondHeader), findsNothing);

    // Present SnackBar
    await tester.tap(find.byKey(snackTarget));
    await tester.pump(); // schedule animation
    expect(find.text(snackBarText), findsOneWidget);
    await tester.pump(); // begin animation
    expect(find.text(snackBarText), findsOneWidget);
    await tester.pump(const Duration(milliseconds: 750));
    expect(find.text(snackBarText), findsOneWidget);
    // Push new route
    await tester.tap(find.byKey(transitionTarget));
    await tester.pump();
    expect(find.text(snackBarText), findsOneWidget);
    expect(find.text(firstHeader), findsOneWidget);
    expect(find.text(secondHeader, skipOffstage: false), findsOneWidget);
    await tester.pump();
    expect(find.text(snackBarText), findsOneWidget);
    expect(find.text(firstHeader), findsOneWidget);
    expect(find.text(secondHeader), findsOneWidget);
    await tester.pump(const Duration(milliseconds: 750));
    expect(find.text(snackBarText), findsOneWidget);
    expect(find.text(firstHeader), findsNothing);
    expect(find.text(secondHeader), findsOneWidget);
  });

  testWidgets('Should have only one SnackBar during back swipe navigation', (
    WidgetTester tester,
  ) async {
    const String snackBarText = 'hello snackbar';
    const Key snackTarget = Key('snack-target');
    const Key transitionTarget = Key('transition-target');

    Widget buildApp() {
      final PageTransitionsTheme pageTransitionTheme = PageTransitionsTheme(
        builders: <TargetPlatform, PageTransitionsBuilder>{
          for (final TargetPlatform platform in TargetPlatform.values)
            platform: const CupertinoPageTransitionsBuilder(),
        },
      );
      return MaterialApp(
        theme: ThemeData(pageTransitionsTheme: pageTransitionTheme),
        initialRoute: '/',
        routes: <String, WidgetBuilder>{
          '/': (BuildContext context) {
            return Scaffold(
              body: Center(
                child: ElevatedButton(
                  key: transitionTarget,
                  child: const Text('PUSH'),
                  onPressed: () {
                    Navigator.of(context).pushNamed('/second');
                  },
                ),
              ),
            );
          },
          '/second': (BuildContext context) {
            return Scaffold(
              floatingActionButton: FloatingActionButton(
                key: snackTarget,
                onPressed: () async {
                  ScaffoldMessenger.of(
                    context,
                  ).showSnackBar(const SnackBar(content: Text(snackBarText)));
                },
                child: const Text('X'),
              ),
            );
          },
        },
      );
    }

    await tester.pumpWidget(buildApp());

    // Transition to second page.
    await tester.tap(find.byKey(transitionTarget));
    await tester.pumpAndSettle();

    // Present SnackBar
    await tester.tap(find.byKey(snackTarget));
    await tester.pump(); // schedule animation
    expect(find.text(snackBarText), findsOneWidget);
    await tester.pump(); // begin animation
    expect(find.text(snackBarText), findsOneWidget);
    await tester.pump(const Duration(milliseconds: 750));
    expect(find.text(snackBarText), findsOneWidget);

    // Start the gesture at the edge of the screen.
    final TestGesture gesture = await tester.startGesture(const Offset(5.0, 200.0));
    // Trigger the swipe.
    await gesture.moveBy(const Offset(100.0, 0.0));

    // Back gestures should trigger and draw the hero transition in the very same
    // frame (since the "from" route has already moved to reveal the "to" route).
    await tester.pump();

    // We should have only one SnackBar displayed on the screen.
    expect(find.text(snackBarText), findsOneWidget);
  });

  testWidgets('Material2 - SnackBars should be shown above the bottomSheet', (
    WidgetTester tester,
  ) async {
    await tester.pumpWidget(
      MaterialApp(
        debugShowCheckedModeBanner: false, // https://github.com/flutter/flutter/issues/143616
        theme: ThemeData(useMaterial3: false),
        home: const Scaffold(
          bottomSheet: SizedBox(width: 200, height: 50, child: ColoredBox(color: Colors.pink)),
        ),
      ),
    );

    final ScaffoldMessengerState scaffoldMessengerState = tester.state(
      find.byType(ScaffoldMessenger),
    );
    scaffoldMessengerState.showSnackBar(
      SnackBar(
        content: const Text('I love Flutter!'),
        duration: const Duration(seconds: 2),
        action: SnackBarAction(label: 'ACTION', onPressed: () {}),
        behavior: SnackBarBehavior.floating,
      ),
    );
    await tester.pumpAndSettle(); // Have the SnackBar fully animate out.

    await expectLater(
      find.byType(MaterialApp),
      matchesGoldenFile('m2_snack_bar.goldenTest.workWithBottomSheet.png'),
    );
  });

  testWidgets('Material3 - SnackBars should be shown above the bottomSheet', (
    WidgetTester tester,
  ) async {
    await tester.pumpWidget(
      MaterialApp(
        theme: ThemeData(useMaterial3: true),
        debugShowCheckedModeBanner: false, // https://github.com/flutter/flutter/issues/143616
        home: const Scaffold(
          bottomSheet: SizedBox(width: 200, height: 50, child: ColoredBox(color: Colors.pink)),
        ),
      ),
    );

    final ScaffoldMessengerState scaffoldMessengerState = tester.state(
      find.byType(ScaffoldMessenger),
    );
    scaffoldMessengerState.showSnackBar(
      SnackBar(
        content: const Text('I love Flutter!'),
        duration: const Duration(seconds: 2),
        action: SnackBarAction(label: 'ACTION', onPressed: () {}),
        behavior: SnackBarBehavior.floating,
      ),
    );
    await tester.pumpAndSettle(); // Have the SnackBar fully animate out.

    await expectLater(
      find.byType(MaterialApp),
      matchesGoldenFile('m3_snack_bar.goldenTest.workWithBottomSheet.png'),
    );
  });

  testWidgets('ScaffoldMessenger does not duplicate a SnackBar when presenting a MaterialBanner.', (
    WidgetTester tester,
  ) async {
    const Key materialBannerTapTarget = Key('materialbanner-tap-target');
    const Key snackBarTapTarget = Key('snackbar-tap-target');
    const String snackBarText = 'SnackBar';
    const String materialBannerText = 'MaterialBanner';
    await tester.pumpWidget(
      MaterialApp(
        home: Scaffold(
          body: Builder(
            builder: (BuildContext context) {
              return Column(
                children: <Widget>[
                  GestureDetector(
                    key: snackBarTapTarget,
                    onTap: () {
                      ScaffoldMessenger.of(
                        context,
                      ).showSnackBar(const SnackBar(content: Text(snackBarText)));
                    },
                    behavior: HitTestBehavior.opaque,
                    child: const SizedBox(height: 100.0, width: 100.0),
                  ),
                  GestureDetector(
                    key: materialBannerTapTarget,
                    onTap: () {
                      ScaffoldMessenger.of(context).showMaterialBanner(
                        MaterialBanner(
                          content: const Text(materialBannerText),
                          actions: <Widget>[
                            TextButton(
                              child: const Text('DISMISS'),
                              onPressed:
                                  () => ScaffoldMessenger.of(context).hideCurrentMaterialBanner(),
                            ),
                          ],
                        ),
                      );
                    },
                    behavior: HitTestBehavior.opaque,
                    child: const SizedBox(height: 100.0, width: 100.0),
                  ),
                ],
              );
            },
          ),
        ),
      ),
    );
    await tester.tap(find.byKey(snackBarTapTarget));
    await tester.tap(find.byKey(materialBannerTapTarget));
    await tester.pumpAndSettle();

    expect(find.text(snackBarText), findsOneWidget);
    expect(find.text(materialBannerText), findsOneWidget);
  });

  testWidgets(
    'Material2 - ScaffoldMessenger presents SnackBars to only the root Scaffold when Scaffolds are nested.',
    (WidgetTester tester) async {
      await tester.pumpWidget(
        MaterialApp(
          theme: ThemeData(useMaterial3: false),
          debugShowCheckedModeBanner: false, // https://github.com/flutter/flutter/issues/143616
          home: Scaffold(
            body: const Scaffold(),
            floatingActionButton: FloatingActionButton(onPressed: () {}),
          ),
        ),
      );

      final ScaffoldMessengerState scaffoldMessengerState = tester.state<ScaffoldMessengerState>(
        find.byType(ScaffoldMessenger),
      );
      scaffoldMessengerState.showSnackBar(
        SnackBar(
          content: const Text('ScaffoldMessenger'),
          duration: const Duration(seconds: 2),
          action: SnackBarAction(label: 'ACTION', onPressed: () {}),
          behavior: SnackBarBehavior.floating,
        ),
      );
      await tester.pumpAndSettle();

      expect(find.byType(SnackBar), findsOneWidget);
      // The FloatingActionButton helps us identify which Scaffold has the
      // SnackBar here. Since the outer Scaffold contains a FAB, the SnackBar
      // should be above it. If the inner Scaffold had the SnackBar, it would be
      // overlapping the FAB.
      await expectLater(
        find.byType(MaterialApp),
        matchesGoldenFile('m2_snack_bar.scaffold.nested.png'),
      );
      final Offset snackBarTopRight = tester.getTopRight(find.byType(SnackBar));
      expect(snackBarTopRight.dy, 465.0);
    },
  );

  testWidgets(
    'Material3 - ScaffoldMessenger presents SnackBars to only the root Scaffold when Scaffolds are nested.',
    (WidgetTester tester) async {
      await tester.pumpWidget(
        MaterialApp(
          theme: ThemeData(useMaterial3: true),
          debugShowCheckedModeBanner: false, // https://github.com/flutter/flutter/issues/143616
          home: Scaffold(
            body: const Scaffold(),
            floatingActionButton: FloatingActionButton(onPressed: () {}),
          ),
        ),
      );

      final ScaffoldMessengerState scaffoldMessengerState = tester.state<ScaffoldMessengerState>(
        find.byType(ScaffoldMessenger),
      );
      scaffoldMessengerState.showSnackBar(
        SnackBar(
          content: const Text('ScaffoldMessenger'),
          duration: const Duration(seconds: 2),
          action: SnackBarAction(label: 'ACTION', onPressed: () {}),
          behavior: SnackBarBehavior.floating,
        ),
      );
      await tester.pumpAndSettle();

      expect(find.byType(SnackBar), findsOneWidget);
      // The FloatingActionButton helps us identify which Scaffold has the
      // SnackBar here. Since the outer Scaffold contains a FAB, the SnackBar
      // should be above it. If the inner Scaffold had the SnackBar, it would be
      // overlapping the FAB.
      await expectLater(
        find.byType(MaterialApp),
        matchesGoldenFile('m3_snack_bar.scaffold.nested.png'),
      );
      final Offset snackBarTopRight = tester.getTopRight(find.byType(SnackBar));

      if (!kIsWeb || isSkiaWeb) {
        // https://github.com/flutter/flutter/issues/99933
        expect(snackBarTopRight.dy, 465.0);
      }
    },
  );

  testWidgets('ScaffoldMessengerState clearSnackBars works as expected', (
    WidgetTester tester,
  ) async {
    final List<String> snackBars = <String>['Hello Snackbar', 'Hi Snackbar', 'Bye Snackbar'];
    int snackBarCounter = 0;
    const Key tapTarget = Key('tap-target');
    final GlobalKey<ScaffoldMessengerState> scaffoldMessengerKey = GlobalKey();

    await tester.pumpWidget(
      MaterialApp(
        home: ScaffoldMessenger(
          key: scaffoldMessengerKey,
          child: Scaffold(
            body: Builder(
              builder: (BuildContext context) {
                return GestureDetector(
                  key: tapTarget,
                  onTap: () {
                    ScaffoldMessenger.of(context).showSnackBar(
                      SnackBar(
                        content: Text(snackBars[snackBarCounter++]),
                        duration: const Duration(seconds: 2),
                      ),
                    );
                  },
                  behavior: HitTestBehavior.opaque,
                  child: const SizedBox(height: 100.0, width: 100.0),
                );
              },
            ),
          ),
        ),
      ),
    );
    expect(find.text(snackBars[0]), findsNothing);
    expect(find.text(snackBars[1]), findsNothing);
    expect(find.text(snackBars[2]), findsNothing);
    await tester.tap(find.byKey(tapTarget));
    await tester.tap(find.byKey(tapTarget));
    await tester.tap(find.byKey(tapTarget));
    expect(find.text(snackBars[0]), findsNothing);
    expect(find.text(snackBars[1]), findsNothing);
    expect(find.text(snackBars[2]), findsNothing);
    await tester.pump(); // schedule animation
    expect(find.text(snackBars[0]), findsOneWidget);
    scaffoldMessengerKey.currentState!.clearSnackBars();
    expect(find.text(snackBars[0]), findsOneWidget);
    await tester.pump(const Duration(seconds: 2));
    expect(find.text(snackBars[0]), findsNothing);
    expect(find.text(snackBars[1]), findsNothing);
    expect(find.text(snackBars[2]), findsNothing);
  });

  Widget doBuildApp({
    required SnackBarBehavior? behavior,
    EdgeInsetsGeometry? margin,
    double? width,
    double? actionOverflowThreshold,
  }) {
    return MaterialApp(
      home: Scaffold(
        floatingActionButton: FloatingActionButton(child: const Icon(Icons.send), onPressed: () {}),
        body: Builder(
          builder: (BuildContext context) {
            return GestureDetector(
              onTap: () {
                ScaffoldMessenger.of(context).showSnackBar(
                  SnackBar(
                    behavior: behavior,
                    margin: margin,
                    width: width,
                    content: const Text('I am a snack bar.'),
                    duration: const Duration(seconds: 2),
                    action: SnackBarAction(label: 'ACTION', onPressed: () {}),
                    actionOverflowThreshold: actionOverflowThreshold,
                  ),
                );
              },
              child: const Text('X'),
            );
          },
        ),
      ),
    );
  }

  testWidgets('Setting SnackBarBehavior.fixed will still assert for margin', (
    WidgetTester tester,
  ) async {
    // Regression test for https://github.com/flutter/flutter/issues/84935
    await tester.pumpWidget(
      doBuildApp(behavior: SnackBarBehavior.fixed, margin: const EdgeInsets.all(8.0)),
    );
    await tester.tap(find.text('X'));
    await tester.pump(); // start animation
    await tester.pump(const Duration(milliseconds: 750));
    final AssertionError exception = tester.takeException() as AssertionError;
    expect(
      exception.message,
      'Margin can only be used with floating behavior. SnackBarBehavior.fixed '
      'was set in the SnackBar constructor.',
    );
  });

  testWidgets('Default SnackBarBehavior will still assert for margin', (WidgetTester tester) async {
    // Regression test for https://github.com/flutter/flutter/issues/84935
    await tester.pumpWidget(doBuildApp(behavior: null, margin: const EdgeInsets.all(8.0)));
    await tester.tap(find.text('X'));
    await tester.pump(); // start animation
    await tester.pump(const Duration(milliseconds: 750));
    final AssertionError exception = tester.takeException() as AssertionError;
    expect(
      exception.message,
      'Margin can only be used with floating behavior. SnackBarBehavior.fixed '
      'was set by default.',
    );
  });

  testWidgets('Setting SnackBarBehavior.fixed will still assert for width', (
    WidgetTester tester,
  ) async {
    // Regression test for https://github.com/flutter/flutter/issues/84935
    await tester.pumpWidget(doBuildApp(behavior: SnackBarBehavior.fixed, width: 5.0));
    await tester.tap(find.text('X'));
    await tester.pump(); // start animation
    await tester.pump(const Duration(milliseconds: 750));
    final AssertionError exception = tester.takeException() as AssertionError;
    expect(
      exception.message,
      'Width can only be used with floating behavior. SnackBarBehavior.fixed '
      'was set in the SnackBar constructor.',
    );
  });

  testWidgets('Default SnackBarBehavior will still assert for width', (WidgetTester tester) async {
    // Regression test for https://github.com/flutter/flutter/issues/84935
    await tester.pumpWidget(doBuildApp(behavior: null, width: 5.0));
    await tester.tap(find.text('X'));
    await tester.pump(); // start animation
    await tester.pump(const Duration(milliseconds: 750));
    final AssertionError exception = tester.takeException() as AssertionError;
    expect(
      exception.message,
      'Width can only be used with floating behavior. SnackBarBehavior.fixed '
      'was set by default.',
    );
  });

  for (final double overflowThreshold in <double>[-1.0, -.0001, 1.000001, 5]) {
    testWidgets('SnackBar will assert for actionOverflowThreshold outside of 0-1 range', (
      WidgetTester tester,
    ) async {
      await tester.pumpWidget(
        doBuildApp(actionOverflowThreshold: overflowThreshold, behavior: SnackBarBehavior.fixed),
      );
      await tester.tap(find.text('X'));
      await tester.pump(); // start animation
      await tester.pump(const Duration(milliseconds: 750));

      final AssertionError exception = tester.takeException() as AssertionError;
      expect(exception.message, 'Action overflow threshold must be between 0 and 1 inclusive');
    });
  }

  testWidgets('Material2 - Snackbar by default clips BackdropFilter', (WidgetTester tester) async {
    // Regression test for https://github.com/flutter/flutter/issues/98205
    await tester.pumpWidget(
      MaterialApp(
        theme: ThemeData(useMaterial3: false),
        debugShowCheckedModeBanner: false, // https://github.com/flutter/flutter/issues/143616
        home: Scaffold(
          body: const Scaffold(),
          floatingActionButton: FloatingActionButton(onPressed: () {}),
        ),
      ),
    );

    final ScaffoldMessengerState scaffoldMessengerState = tester.state<ScaffoldMessengerState>(
      find.byType(ScaffoldMessenger),
    );
    scaffoldMessengerState.showSnackBar(
      SnackBar(
        backgroundColor: Colors.transparent,
        content: BackdropFilter(
          filter: ImageFilter.blur(sigmaX: 20.0, sigmaY: 20.0),
          child: const Text('I am a snack bar.'),
        ),
        duration: const Duration(seconds: 2),
        action: SnackBarAction(label: 'ACTION', onPressed: () {}),
        behavior: SnackBarBehavior.fixed,
      ),
    );
    await tester.pumpAndSettle();
    await tester.tap(find.text('I am a snack bar.'));
    await tester.pump(); // start animation
    await tester.pump(const Duration(milliseconds: 750));
    await expectLater(
      find.byType(MaterialApp),
      matchesGoldenFile('m2_snack_bar.goldenTest.backdropFilter.png'),
    );
  });

  testWidgets('Material3 - Snackbar by default clips BackdropFilter', (WidgetTester tester) async {
    // Regression test for https://github.com/flutter/flutter/issues/98205
    await tester.pumpWidget(
      MaterialApp(
        theme: ThemeData(useMaterial3: true),
        debugShowCheckedModeBanner: false, // https://github.com/flutter/flutter/issues/143616
        home: Scaffold(
          body: const Scaffold(),
          floatingActionButton: FloatingActionButton(onPressed: () {}),
        ),
      ),
    );

    final ScaffoldMessengerState scaffoldMessengerState = tester.state<ScaffoldMessengerState>(
      find.byType(ScaffoldMessenger),
    );
    scaffoldMessengerState.showSnackBar(
      SnackBar(
        backgroundColor: Colors.transparent,
        content: BackdropFilter(
          filter: ImageFilter.blur(sigmaX: 20.0, sigmaY: 20.0),
          child: const Text('I am a snack bar.'),
        ),
        duration: const Duration(seconds: 2),
        action: SnackBarAction(label: 'ACTION', onPressed: () {}),
        behavior: SnackBarBehavior.fixed,
      ),
    );
    await tester.pumpAndSettle();
    await tester.tap(find.text('I am a snack bar.'));
    await tester.pump(); // start animation
    await tester.pump(const Duration(milliseconds: 750));
    await expectLater(
      find.byType(MaterialApp),
      matchesGoldenFile('m3_snack_bar.goldenTest.backdropFilter.png'),
    );
  });

  testWidgets('Floating snackbar can display optional icon', (WidgetTester tester) async {
    await tester.pumpWidget(
      MaterialApp(
        theme: ThemeData(useMaterial3: false),
        debugShowCheckedModeBanner: false, // https://github.com/flutter/flutter/issues/143616
        home: const Scaffold(
          bottomSheet: SizedBox(width: 200, height: 50, child: ColoredBox(color: Colors.pink)),
        ),
      ),
    );

    final ScaffoldMessengerState scaffoldMessengerState = tester.state(
      find.byType(ScaffoldMessenger),
    );
    scaffoldMessengerState.showSnackBar(
      SnackBar(
        content: const Text('Feeling snackish'),
        duration: const Duration(seconds: 2),
        action: SnackBarAction(label: 'ACTION', onPressed: () {}),
        behavior: SnackBarBehavior.floating,
        showCloseIcon: true,
      ),
    );
    await tester.pumpAndSettle(); // Have the SnackBar fully animate out.

    await expectLater(
      find.byType(MaterialApp),
      matchesGoldenFile('snack_bar.goldenTest.floatingWithActionWithIcon.png'),
    );
  });

  testWidgets('SnackBar has tooltip for Close Button', (WidgetTester tester) async {
    // Regression test for https://github.com/flutter/flutter/issues/143793
    await tester.pumpWidget(
      MaterialApp(
        theme: ThemeData(useMaterial3: false),
        debugShowCheckedModeBanner: false, // https://github.com/flutter/flutter/issues/143616
        home: const Scaffold(
          bottomSheet: SizedBox(width: 200, height: 50, child: ColoredBox(color: Colors.pink)),
        ),
      ),
    );

    final ScaffoldMessengerState scaffoldMessengerState = tester.state(
      find.byType(ScaffoldMessenger),
    );
    scaffoldMessengerState.showSnackBar(
      SnackBar(
        content: const Text('Snackbar with close button'),
        duration: const Duration(days: 365),
        action: SnackBarAction(label: 'ACTION', onPressed: () {}),
        behavior: SnackBarBehavior.floating,
        showCloseIcon: true,
      ),
    );
    await tester.pumpAndSettle(); // Have the SnackBar fully animate in.

    expect(
      find.byTooltip(MaterialLocalizations.of(scaffoldMessengerState.context).closeButtonLabel),
      findsOneWidget,
    );
  });

  testWidgets('Material2 - Fixed width snackbar can display optional icon', (
    WidgetTester tester,
  ) async {
    await tester.pumpWidget(
      MaterialApp(
        theme: ThemeData(useMaterial3: false),
        debugShowCheckedModeBanner: false, // https://github.com/flutter/flutter/issues/143616
        home: const Scaffold(
          bottomSheet: SizedBox(width: 200, height: 50, child: ColoredBox(color: Colors.pink)),
        ),
      ),
    );

    final ScaffoldMessengerState scaffoldMessengerState = tester.state(
      find.byType(ScaffoldMessenger),
    );
    scaffoldMessengerState.showSnackBar(
      SnackBar(
        content: const Text('Go get a snack'),
        duration: const Duration(seconds: 2),
        action: SnackBarAction(label: 'ACTION', onPressed: () {}),
        showCloseIcon: true,
        behavior: SnackBarBehavior.fixed,
      ),
    );
    await tester.pumpAndSettle(); // Have the SnackBar fully animate out.

    await expectLater(
      find.byType(MaterialApp),
      matchesGoldenFile('m2_snack_bar.goldenTest.fixedWithActionWithIcon.png'),
    );
  });

  testWidgets('Material3 - Fixed width snackbar can display optional icon', (
    WidgetTester tester,
  ) async {
    await tester.pumpWidget(
      MaterialApp(
        theme: ThemeData(useMaterial3: true),
        debugShowCheckedModeBanner: false, // https://github.com/flutter/flutter/issues/143616
        home: const Scaffold(
          bottomSheet: SizedBox(width: 200, height: 50, child: ColoredBox(color: Colors.pink)),
        ),
      ),
    );

    final ScaffoldMessengerState scaffoldMessengerState = tester.state(
      find.byType(ScaffoldMessenger),
    );
    scaffoldMessengerState.showSnackBar(
      SnackBar(
        content: const Text('Go get a snack'),
        duration: const Duration(seconds: 2),
        action: SnackBarAction(label: 'ACTION', onPressed: () {}),
        showCloseIcon: true,
        behavior: SnackBarBehavior.fixed,
      ),
    );
    await tester.pumpAndSettle(); // Have the SnackBar fully animate out.

    await expectLater(
      find.byType(MaterialApp),
      matchesGoldenFile('m3_snack_bar.goldenTest.fixedWithActionWithIcon.png'),
    );
  });

  testWidgets('Material2 - Fixed snackbar can display optional icon without action', (
    WidgetTester tester,
  ) async {
    await tester.pumpWidget(
      MaterialApp(
        theme: ThemeData(useMaterial3: false),
        debugShowCheckedModeBanner: false, // https://github.com/flutter/flutter/issues/143616
        home: const Scaffold(
          bottomSheet: SizedBox(width: 200, height: 50, child: ColoredBox(color: Colors.pink)),
        ),
      ),
    );

    final ScaffoldMessengerState scaffoldMessengerState = tester.state(
      find.byType(ScaffoldMessenger),
    );
    scaffoldMessengerState.showSnackBar(
      const SnackBar(
        content: Text('I wonder if there are snacks nearby?'),
        duration: Duration(seconds: 2),
        behavior: SnackBarBehavior.fixed,
        showCloseIcon: true,
      ),
    );
    await tester.pumpAndSettle(); // Have the SnackBar fully animate out.

    await expectLater(
      find.byType(MaterialApp),
      matchesGoldenFile('m2_snack_bar.goldenTest.fixedWithIcon.png'),
    );
  });

  testWidgets('Material3 - Fixed snackbar can display optional icon without action', (
    WidgetTester tester,
  ) async {
    await tester.pumpWidget(
      MaterialApp(
        theme: ThemeData(useMaterial3: true),
        debugShowCheckedModeBanner: false, // https://github.com/flutter/flutter/issues/143616
        home: const Scaffold(
          bottomSheet: SizedBox(width: 200, height: 50, child: ColoredBox(color: Colors.pink)),
        ),
      ),
    );

    final ScaffoldMessengerState scaffoldMessengerState = tester.state(
      find.byType(ScaffoldMessenger),
    );
    scaffoldMessengerState.showSnackBar(
      const SnackBar(
        content: Text('I wonder if there are snacks nearby?'),
        duration: Duration(seconds: 2),
        behavior: SnackBarBehavior.fixed,
        showCloseIcon: true,
      ),
    );
    await tester.pumpAndSettle(); // Have the SnackBar fully animate out.

    await expectLater(
      find.byType(MaterialApp),
      matchesGoldenFile('m3_snack_bar.goldenTest.fixedWithIcon.png'),
    );
  });

  testWidgets('Material2 - Floating width snackbar can display optional icon without action', (
    WidgetTester tester,
  ) async {
    await tester.pumpWidget(
      MaterialApp(
        theme: ThemeData(useMaterial3: false),
        debugShowCheckedModeBanner: false, // https://github.com/flutter/flutter/issues/143616
        home: const Scaffold(
          bottomSheet: SizedBox(width: 200, height: 50, child: ColoredBox(color: Colors.pink)),
        ),
      ),
    );

    final ScaffoldMessengerState scaffoldMessengerState = tester.state(
      find.byType(ScaffoldMessenger),
    );
    scaffoldMessengerState.showSnackBar(
      const SnackBar(
        content: Text('Must go get a snack!'),
        duration: Duration(seconds: 2),
        showCloseIcon: true,
        behavior: SnackBarBehavior.floating,
      ),
    );
    await tester.pumpAndSettle(); // Have the SnackBar fully animate out.

    await expectLater(
      find.byType(MaterialApp),
      matchesGoldenFile('m2_snack_bar.goldenTest.floatingWithIcon.png'),
    );
  });

  testWidgets('Material3 - Floating width snackbar can display optional icon without action', (
    WidgetTester tester,
  ) async {
    await tester.pumpWidget(
      MaterialApp(
        theme: ThemeData(useMaterial3: true),
        debugShowCheckedModeBanner: false, // https://github.com/flutter/flutter/issues/143616
        home: const Scaffold(
          bottomSheet: SizedBox(width: 200, height: 50, child: ColoredBox(color: Colors.pink)),
        ),
      ),
    );

    final ScaffoldMessengerState scaffoldMessengerState = tester.state(
      find.byType(ScaffoldMessenger),
    );
    scaffoldMessengerState.showSnackBar(
      const SnackBar(
        content: Text('Must go get a snack!'),
        duration: Duration(seconds: 2),
        showCloseIcon: true,
        behavior: SnackBarBehavior.floating,
      ),
    );
    await tester.pumpAndSettle(); // Have the SnackBar fully animate out.

    await expectLater(
      find.byType(MaterialApp),
      matchesGoldenFile('m3_snack_bar.goldenTest.floatingWithIcon.png'),
    );
  });

  testWidgets('Material2 - Floating multi-line snackbar with icon is aligned correctly', (
    WidgetTester tester,
  ) async {
    await tester.pumpWidget(
      MaterialApp(
        theme: ThemeData(useMaterial3: false),
        debugShowCheckedModeBanner: false, // https://github.com/flutter/flutter/issues/143616
        home: const Scaffold(
          bottomSheet: SizedBox(width: 200, height: 50, child: ColoredBox(color: Colors.pink)),
        ),
      ),
    );

    final ScaffoldMessengerState scaffoldMessengerState = tester.state(
      find.byType(ScaffoldMessenger),
    );
    scaffoldMessengerState.showSnackBar(
      const SnackBar(
        content: Text(
          'This is a really long snackbar message. So long, it spans across more than one line!',
        ),
        duration: Duration(seconds: 2),
        showCloseIcon: true,
        behavior: SnackBarBehavior.floating,
      ),
    );
    await tester.pumpAndSettle(); // Have the SnackBar fully animate out.

    await expectLater(
      find.byType(MaterialApp),
      matchesGoldenFile('m2_snack_bar.goldenTest.multiLineWithIcon.png'),
    );
  });

  testWidgets('Material3 - Floating multi-line snackbar with icon is aligned correctly', (
    WidgetTester tester,
  ) async {
    await tester.pumpWidget(
      MaterialApp(
        theme: ThemeData(useMaterial3: true),
        debugShowCheckedModeBanner: false, // https://github.com/flutter/flutter/issues/143616
        home: const Scaffold(
          bottomSheet: SizedBox(width: 200, height: 50, child: ColoredBox(color: Colors.pink)),
        ),
      ),
    );

    final ScaffoldMessengerState scaffoldMessengerState = tester.state(
      find.byType(ScaffoldMessenger),
    );
    scaffoldMessengerState.showSnackBar(
      const SnackBar(
        content: Text(
          'This is a really long snackbar message. So long, it spans across more than one line!',
        ),
        duration: Duration(seconds: 2),
        showCloseIcon: true,
        behavior: SnackBarBehavior.floating,
      ),
    );
    await tester.pumpAndSettle(); // Have the SnackBar fully animate out.

    await expectLater(
      find.byType(MaterialApp),
      matchesGoldenFile('m3_snack_bar.goldenTest.multiLineWithIcon.png'),
    );
  });

  testWidgets(
    'Material2 - Floating multi-line snackbar with icon and actionOverflowThreshold=1 is aligned correctly',
    (WidgetTester tester) async {
      await tester.pumpWidget(
        MaterialApp(
          theme: ThemeData(useMaterial3: false),
          debugShowCheckedModeBanner: false, // https://github.com/flutter/flutter/issues/143616
          home: const Scaffold(
            bottomSheet: SizedBox(width: 200, height: 50, child: ColoredBox(color: Colors.pink)),
          ),
        ),
      );

      final ScaffoldMessengerState scaffoldMessengerState = tester.state(
        find.byType(ScaffoldMessenger),
      );
      scaffoldMessengerState.showSnackBar(
        const SnackBar(
          content: Text(
            'This is a really long snackbar message. So long, it spans across more than one line!',
          ),
          duration: Duration(seconds: 2),
          showCloseIcon: true,
          behavior: SnackBarBehavior.floating,
          actionOverflowThreshold: 1,
        ),
      );
      await tester.pumpAndSettle(); // Have the SnackBar fully animate in.

      await expectLater(
        find.byType(MaterialApp),
        matchesGoldenFile(
          'm2_snack_bar.goldenTest.multiLineWithIconWithZeroActionOverflowThreshold.png',
        ),
      );
    },
  );

  testWidgets(
    'Material3 - Floating multi-line snackbar with icon and actionOverflowThreshold=1 is aligned correctly',
    (WidgetTester tester) async {
      await tester.pumpWidget(
        MaterialApp(
          theme: ThemeData(useMaterial3: true),
          debugShowCheckedModeBanner: false, // https://github.com/flutter/flutter/issues/143616
          home: const Scaffold(
            bottomSheet: SizedBox(width: 200, height: 50, child: ColoredBox(color: Colors.pink)),
          ),
        ),
      );

      final ScaffoldMessengerState scaffoldMessengerState = tester.state(
        find.byType(ScaffoldMessenger),
      );
      scaffoldMessengerState.showSnackBar(
        const SnackBar(
          content: Text(
            'This is a really long snackbar message. So long, it spans across more than one line!',
          ),
          duration: Duration(seconds: 2),
          showCloseIcon: true,
          behavior: SnackBarBehavior.floating,
          actionOverflowThreshold: 1,
        ),
      );
      await tester.pumpAndSettle(); // Have the SnackBar fully animate in.

      await expectLater(
        find.byType(MaterialApp),
        matchesGoldenFile(
          'm3_snack_bar.goldenTest.multiLineWithIconWithZeroActionOverflowThreshold.png',
        ),
      );
    },
  );

  testWidgets('ScaffoldMessenger will alert for snackbars that cannot be presented', (
    WidgetTester tester,
  ) async {
    // Regression test for https://github.com/flutter/flutter/issues/103004
    await tester.pumpWidget(const MaterialApp(home: Center()));

    final ScaffoldMessengerState scaffoldMessengerState = tester.state<ScaffoldMessengerState>(
      find.byType(ScaffoldMessenger),
    );
    expect(
      () {
        scaffoldMessengerState.showSnackBar(const SnackBar(content: Text('SnackBar')));
      },
      throwsA(
        isA<AssertionError>().having(
          (AssertionError error) => error.toString(),
          'description',
          contains(
            'ScaffoldMessenger.showSnackBar was called, but there are currently '
            'no descendant Scaffolds to present to.',
          ),
        ),
      ),
    );
  });

  testWidgets('SnackBarAction backgroundColor works as a Color', (WidgetTester tester) async {
    const Color backgroundColor = Colors.blue;

    await tester.pumpWidget(
      MaterialApp(
        home: Scaffold(
          body: Builder(
            builder: (BuildContext context) {
              return GestureDetector(
                onTap: () {
                  ScaffoldMessenger.of(context).showSnackBar(
                    SnackBar(
                      content: const Text('I am a snack bar.'),
                      duration: const Duration(seconds: 2),
                      action: SnackBarAction(
                        backgroundColor: backgroundColor,
                        label: 'ACTION',
                        onPressed: () {},
                      ),
                    ),
                  );
                },
                child: const Text('Tap'),
              );
            },
          ),
        ),
      ),
    );

    await tester.tap(find.text('Tap'));
    await tester.pumpAndSettle();

    final Material materialBeforeDismissed = tester.widget<Material>(
      find.descendant(
        of: find.widgetWithText(TextButton, 'ACTION'),
        matching: find.byType(Material),
      ),
    );
    expect(materialBeforeDismissed.color, backgroundColor);

    await tester.tap(find.text('ACTION'));
    await tester.pump();

    final Material materialAfterDismissed = tester.widget<Material>(
      find.descendant(
        of: find.widgetWithText(TextButton, 'ACTION'),
        matching: find.byType(Material),
      ),
    );
    expect(materialAfterDismissed.color, Colors.transparent);
  });

  testWidgets('SnackBarAction backgroundColor works as a MaterialStateColor', (
    WidgetTester tester,
  ) async {
    final MaterialStateColor backgroundColor = MaterialStateColor.resolveWith((
      Set<MaterialState> states,
    ) {
      if (states.contains(MaterialState.disabled)) {
        return Colors.blue;
      }
      return Colors.purple;
    });

    await tester.pumpWidget(
      MaterialApp(
        home: Scaffold(
          body: Builder(
            builder: (BuildContext context) {
              return GestureDetector(
                onTap: () {
                  ScaffoldMessenger.of(context).showSnackBar(
                    SnackBar(
                      content: const Text('I am a snack bar.'),
                      duration: const Duration(seconds: 2),
                      action: SnackBarAction(
                        backgroundColor: backgroundColor,
                        label: 'ACTION',
                        onPressed: () {},
                      ),
                    ),
                  );
                },
                child: const Text('Tap'),
              );
            },
          ),
        ),
      ),
    );

    await tester.tap(find.text('Tap'));
    await tester.pumpAndSettle();

    final Material materialBeforeDismissed = tester.widget<Material>(
      find.descendant(
        of: find.widgetWithText(TextButton, 'ACTION'),
        matching: find.byType(Material),
      ),
    );
    expect(materialBeforeDismissed.color, Colors.purple);

    await tester.tap(find.text('ACTION'));
    await tester.pump();

    final Material materialAfterDismissed = tester.widget<Material>(
      find.descendant(
        of: find.widgetWithText(TextButton, 'ACTION'),
        matching: find.byType(Material),
      ),
    );
    expect(materialAfterDismissed.color, Colors.blue);
  });

  testWidgets('SnackBarAction disabledBackgroundColor works as expected', (
    WidgetTester tester,
  ) async {
    const Color backgroundColor = Colors.blue;
    const Color disabledBackgroundColor = Colors.red;

    await tester.pumpWidget(
      MaterialApp(
        home: Scaffold(
          body: Builder(
            builder: (BuildContext context) {
              return GestureDetector(
                onTap: () {
                  ScaffoldMessenger.of(context).showSnackBar(
                    SnackBar(
                      content: const Text('I am a snack bar.'),
                      duration: const Duration(seconds: 2),
                      action: SnackBarAction(
                        backgroundColor: backgroundColor,
                        disabledBackgroundColor: disabledBackgroundColor,
                        label: 'ACTION',
                        onPressed: () {},
                      ),
                    ),
                  );
                },
                child: const Text('Tap'),
              );
            },
          ),
        ),
      ),
    );

    await tester.tap(find.text('Tap'));
    await tester.pumpAndSettle();

    final Material materialBeforeDismissed = tester.widget<Material>(
      find.descendant(
        of: find.widgetWithText(TextButton, 'ACTION'),
        matching: find.byType(Material),
      ),
    );
    expect(materialBeforeDismissed.color, backgroundColor);

    await tester.tap(find.text('ACTION'));
    await tester.pump();

    final Material materialAfterDismissed = tester.widget<Material>(
      find.descendant(
        of: find.widgetWithText(TextButton, 'ACTION'),
        matching: find.byType(Material),
      ),
    );
    expect(materialAfterDismissed.color, disabledBackgroundColor);
  });

  testWidgets(
    'SnackBarAction asserts when backgroundColor is a MaterialStateColor and disabledBackgroundColor is also provided',
    (WidgetTester tester) async {
      final Color backgroundColor = MaterialStateColor.resolveWith((Set<MaterialState> states) {
        if (states.contains(MaterialState.disabled)) {
          return Colors.blue;
        }
        return Colors.purple;
      });
      const Color disabledBackgroundColor = Colors.red;

      await tester.pumpWidget(
        MaterialApp(
          home: Scaffold(
            body: Builder(
              builder: (BuildContext context) {
                return GestureDetector(
                  onTap: () {
                    ScaffoldMessenger.of(context).showSnackBar(
                      SnackBar(
                        content: const Text('I am a snack bar.'),
                        duration: const Duration(seconds: 2),
                        action: SnackBarAction(
                          backgroundColor: backgroundColor,
                          disabledBackgroundColor: disabledBackgroundColor,
                          label: 'ACTION',
                          onPressed: () {},
                        ),
                      ),
                    );
                  },
                  child: const Text('Tap'),
                );
              },
            ),
          ),
        ),
      );

      await tester.tap(find.text('Tap'));
      await tester.pumpAndSettle();

      expect(
        tester.takeException(),
        isAssertionError.having(
          (AssertionError e) => e.toString(),
          'description',
          contains(
            'disabledBackgroundColor must not be provided when background color is a MaterialStateColor',
          ),
        ),
      );
    },
  );

  testWidgets('SnackBar material applies SnackBar.clipBehavior', (WidgetTester tester) async {
    await tester.pumpWidget(MaterialApp(home: Scaffold(body: Container())));

    final ScaffoldMessengerState scaffoldMessengerState = tester.state(
      find.byType(ScaffoldMessenger),
    );
    scaffoldMessengerState.showSnackBar(const SnackBar(content: Text('I am a snack bar.')));

    await tester.pumpAndSettle(); // Have the SnackBar fully animate out.

    Material material = tester.widget<Material>(
      find.descendant(of: find.byType(SnackBar), matching: find.byType(Material)),
    );

    expect(material.clipBehavior, Clip.hardEdge);

    scaffoldMessengerState.hideCurrentSnackBar(); // Hide the SnackBar.

    await tester.pumpAndSettle(); // Have the SnackBar fully animate out.

    scaffoldMessengerState.showSnackBar(
      const SnackBar(content: Text('I am a snack bar.'), clipBehavior: Clip.antiAlias),
    );

    await tester.pumpAndSettle(); // Have the SnackBar fully animate in.

    material = tester.widget<Material>(
      find.descendant(of: find.byType(SnackBar), matching: find.byType(Material)),
    );

    expect(material.clipBehavior, Clip.antiAlias);
  });

  testWidgets('Tap on button behind snack bar defined by width', (WidgetTester tester) async {
    tester.view.physicalSize = const Size.square(200);
    tester.view.devicePixelRatio = 1;
    addTearDown(tester.view.resetPhysicalSize);
    addTearDown(tester.view.resetDevicePixelRatio);

    const String buttonText = 'Show snackbar';
    const String snackbarContent = 'Snackbar';
    const String buttonText2 = 'Try press me';

    final Completer<void> completer = Completer<void>();

    await tester.pumpWidget(
      MaterialApp(
        home: Scaffold(
          body: Builder(
            builder: (BuildContext context) {
              return Column(
                mainAxisAlignment: MainAxisAlignment.spaceBetween,
                crossAxisAlignment: CrossAxisAlignment.stretch,
                children: <Widget>[
                  ElevatedButton(
                    onPressed: () {
                      ScaffoldMessenger.of(context).showSnackBar(
                        const SnackBar(
                          behavior: SnackBarBehavior.floating,
                          width: 100,
                          content: Text(snackbarContent),
                        ),
                      );
                    },
                    child: const Text(buttonText),
                  ),
                  ElevatedButton(
                    onPressed: () {
                      completer.complete();
                    },
                    child: const Text(buttonText2),
                  ),
                ],
              );
            },
          ),
        ),
      ),
    );

    await tester.tap(find.text(buttonText));
    await tester.pumpAndSettle();

    expect(find.text(snackbarContent), findsOneWidget);
    await tester.tapAt(tester.getTopLeft(find.text(buttonText2)));
    expect(find.text(snackbarContent), findsOneWidget);

    expect(completer.isCompleted, true);
  });

  testWidgets('Tap on button behind snack bar defined by margin', (WidgetTester tester) async {
    // Regression test for https://github.com/flutter/flutter/issues/78537.
    tester.view.physicalSize = const Size.square(200);
    tester.view.devicePixelRatio = 1;
    addTearDown(tester.view.resetPhysicalSize);
    addTearDown(tester.view.resetDevicePixelRatio);

    const String buttonText = 'Show snackbar';
    const String snackbarContent = 'Snackbar';
    const String buttonText2 = 'Try press me';

    final Completer<void> completer = Completer<void>();

    await tester.pumpWidget(
      MaterialApp(
        home: Scaffold(
          body: Builder(
            builder: (BuildContext context) {
              return Column(
                mainAxisAlignment: MainAxisAlignment.spaceBetween,
                crossAxisAlignment: CrossAxisAlignment.stretch,
                children: <Widget>[
                  ElevatedButton(
                    onPressed: () {
                      ScaffoldMessenger.of(context).showSnackBar(
                        const SnackBar(
                          behavior: SnackBarBehavior.floating,
                          margin: EdgeInsets.only(left: 100),
                          content: Text(snackbarContent),
                        ),
                      );
                    },
                    child: const Text(buttonText),
                  ),
                  ElevatedButton(
                    onPressed: () {
                      completer.complete();
                    },
                    child: const Text(buttonText2),
                  ),
                ],
              );
            },
          ),
        ),
      ),
    );

    await tester.tap(find.text(buttonText));
    await tester.pumpAndSettle();

    expect(find.text(snackbarContent), findsOneWidget);
    await tester.tapAt(tester.getTopLeft(find.text(buttonText2)));
    expect(find.text(snackbarContent), findsOneWidget);

    expect(completer.isCompleted, true);
  });

  testWidgets("Can't tap on button behind snack bar defined by margin and HitTestBehavior.opaque", (
    WidgetTester tester,
  ) async {
    // Regression test for https://github.com/flutter/flutter/issues/78537.
    tester.view.physicalSize = const Size.square(200);
    tester.view.devicePixelRatio = 1;
    addTearDown(tester.view.resetPhysicalSize);
    addTearDown(tester.view.resetDevicePixelRatio);

    const String buttonText = 'Show snackbar';
    const String snackbarContent = 'Snackbar';
    const String buttonText2 = 'Try press me';

    final Completer<void> completer = Completer<void>();

    await tester.pumpWidget(
      MaterialApp(
        home: Scaffold(
          body: Builder(
            builder: (BuildContext context) {
              return Column(
                mainAxisAlignment: MainAxisAlignment.spaceBetween,
                crossAxisAlignment: CrossAxisAlignment.stretch,
                children: <Widget>[
                  ElevatedButton(
                    onPressed: () {
                      ScaffoldMessenger.of(context).showSnackBar(
                        const SnackBar(
                          hitTestBehavior: HitTestBehavior.opaque,
                          behavior: SnackBarBehavior.floating,
                          margin: EdgeInsets.only(left: 100),
                          content: Text(snackbarContent),
                        ),
                      );
                    },
                    child: const Text(buttonText),
                  ),
                  ElevatedButton(
                    onPressed: () {
                      completer.complete();
                    },
                    child: const Text(buttonText2),
                  ),
                ],
              );
            },
          ),
        ),
      ),
    );

    await tester.tap(find.text(buttonText));
    await tester.pumpAndSettle();

    expect(find.text(snackbarContent), findsOneWidget);
    await tester.tapAt(tester.getTopLeft(find.text(buttonText2)));
    expect(find.text(snackbarContent), findsOneWidget);

    expect(completer.isCompleted, false);
  });

  testWidgets('Action text button uses correct overlay color', (WidgetTester tester) async {
    final ThemeData theme = ThemeData();
    await tester.pumpWidget(
      MaterialApp(
        theme: theme,
        home: Scaffold(
          body: Builder(
            builder: (BuildContext context) {
              return GestureDetector(
                onTap: () {
                  ScaffoldMessenger.of(context).showSnackBar(
                    SnackBar(
                      content: const Text('I am a snack bar.'),
                      action: SnackBarAction(label: 'ACTION', onPressed: () {}),
                    ),
                  );
                },
                child: const Text('X'),
              );
            },
          ),
        ),
      ),
    );

    await tester.tap(find.text('X'));
    await tester.pumpAndSettle();

    final ButtonStyle? actionButtonStyle =
        tester.widget<TextButton>(find.widgetWithText(TextButton, 'ACTION')).style;
    expect(
      actionButtonStyle?.overlayColor?.resolve(<MaterialState>{MaterialState.hovered}),
      theme.colorScheme.inversePrimary.withOpacity(0.08),
    );
  });

  testWidgets(
    'Can interact with widgets behind SnackBar when insetPadding is set in SnackBarThemeData',
    (WidgetTester tester) async {
      // Regression test for https://github.com/flutter/flutter/issues/148566.
      tester.view.physicalSize = const Size.square(200);
      tester.view.devicePixelRatio = 1;
      addTearDown(tester.view.resetPhysicalSize);
      addTearDown(tester.view.resetDevicePixelRatio);

      const String buttonText = 'Show snackbar';
      const String snackbarContent = 'Snackbar';
      const String buttonText2 = 'Try press me';

      final Completer<void> completer = Completer<void>();

      await tester.pumpWidget(
        MaterialApp(
          theme: ThemeData(
            snackBarTheme: const SnackBarThemeData(insetPadding: EdgeInsets.only(left: 100)),
          ),
          home: Scaffold(
            body: Builder(
              builder: (BuildContext context) {
                return Column(
                  mainAxisAlignment: MainAxisAlignment.spaceBetween,
                  crossAxisAlignment: CrossAxisAlignment.stretch,
                  children: <Widget>[
                    ElevatedButton(
                      onPressed: () {
                        ScaffoldMessenger.of(context).showSnackBar(
                          const SnackBar(
                            behavior: SnackBarBehavior.floating,
                            content: Text(snackbarContent),
                          ),
                        );
                      },
                      child: const Text(buttonText),
                    ),
                    ElevatedButton(
                      onPressed: () {
                        completer.complete();
                      },
                      child: const Text(buttonText2),
                    ),
                  ],
                );
              },
            ),
          ),
        ),
      );

      await tester.tap(find.text(buttonText));
      await tester.pumpAndSettle();

      expect(find.text(snackbarContent), findsOneWidget);
      await tester.tapAt(tester.getTopLeft(find.text(buttonText2)));
      expect(find.text(snackbarContent), findsOneWidget);

      expect(completer.isCompleted, true);
    },
  );
}

/// Start test for "SnackBar dismiss test".
Future<void> _testSnackBarDismiss({
  required WidgetTester tester,
  required Key tapTarget,
  required double scaffoldWidth,
  required ValueChanged<DismissDirection> onDismissDirectionChange,
  required VoidCallback onDragGestureChange,
}) async {
  final Map<DismissDirection, List<Offset>> dragGestures = _getDragGesturesOfDismissDirections(
    scaffoldWidth,
  );

  for (final DismissDirection key in dragGestures.keys) {
    onDismissDirectionChange(key);

    for (final Offset dragGesture in dragGestures[key]!) {
      onDragGestureChange();

      expect(find.text('bar1'), findsNothing);
      expect(find.text('bar2'), findsNothing);
      await tester.tap(find.byKey(tapTarget)); // queue bar1
      await tester.tap(find.byKey(tapTarget)); // queue bar2
      expect(find.text('bar1'), findsNothing);
      expect(find.text('bar2'), findsNothing);
      await tester.pump(); // schedule animation for bar1
      expect(find.text('bar1'), findsOneWidget);
      expect(find.text('bar2'), findsNothing);
      await tester.pump(); // begin animation
      expect(find.text('bar1'), findsOneWidget);
      expect(find.text('bar2'), findsNothing);
      await tester.pump(
        const Duration(milliseconds: 750),
      ); // 0.75s // animation last frame; two second timer starts here
      await tester.drag(find.text('bar1'), dragGesture);
      await tester.pump(); // bar1 dismissed, bar2 begins animating
      expect(find.text('bar1'), findsNothing);
      expect(find.text('bar2'), findsOneWidget);
      await tester.pump(
        const Duration(milliseconds: 750),
      ); // 0.75s // animation last frame; two second timer starts here
      await tester.drag(find.text('bar2'), dragGesture);
      await tester.pump(); // bar2 dismissed
      expect(find.text('bar1'), findsNothing);
      expect(find.text('bar2'), findsNothing);
    }
  }
}

/// Create drag gestures for DismissDirections.
Map<DismissDirection, List<Offset>> _getDragGesturesOfDismissDirections(double scaffoldWidth) {
  final Map<DismissDirection, List<Offset>> dragGestures = <DismissDirection, List<Offset>>{};

  for (final DismissDirection val in DismissDirection.values) {
    switch (val) {
      case DismissDirection.down:
        dragGestures[val] = <Offset>[const Offset(0.0, 50.0)]; // drag to bottom gesture
      case DismissDirection.up:
        dragGestures[val] = <Offset>[const Offset(0.0, -50.0)]; // drag to top gesture
      case DismissDirection.vertical:
        dragGestures[val] = <Offset>[
          const Offset(0.0, 50.0), // drag to bottom gesture
          const Offset(0.0, -50.0), // drag to top gesture
        ];
      case DismissDirection.startToEnd:
        dragGestures[val] = <Offset>[Offset(scaffoldWidth, 0.0)]; // drag to right gesture
      case DismissDirection.endToStart:
        dragGestures[val] = <Offset>[Offset(-scaffoldWidth, 0.0)]; // drag to left gesture
      case DismissDirection.horizontal:
        dragGestures[val] = <Offset>[
          Offset(scaffoldWidth, 0.0), // drag to right gesture
          Offset(-scaffoldWidth, 0.0), // drag to left gesture
        ];
      case DismissDirection.none:
        break;
    }
  }

  return dragGestures;
}

class _TestMaterialStateColor extends MaterialStateColor {
  const _TestMaterialStateColor() : super(_colorRed);

  static const int _colorRed = 0xFFF44336;
  static const int _colorBlue = 0xFF2196F3;

  @override
  Color resolve(Set<MaterialState> states) {
    if (states.contains(MaterialState.pressed)) {
      return const Color(_colorBlue);
    }

    return const Color(_colorRed);
  }
}

class _CustomFloatingActionButtonLocation extends StandardFabLocation
    with FabEndOffsetX, FabFloatOffsetY {
  const _CustomFloatingActionButtonLocation();
}
