// Copyright 2014 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

import 'dart:ui';

import 'package:flutter/foundation.dart';
import 'package:flutter/material.dart';
import 'package:flutter_test/flutter_test.dart';
import '../widgets/semantics_tester.dart';

void main() {
  // Pumps and ensures that the BottomSheet animates non-linearly.
  Future<void> checkNonLinearAnimation(WidgetTester tester) async {
    final Offset firstPosition = tester.getCenter(find.text('BottomSheet'));
    await tester.pump(const Duration(milliseconds: 30));
    final Offset secondPosition = tester.getCenter(find.text('BottomSheet'));
    await tester.pump(const Duration(milliseconds: 30));
    final Offset thirdPosition = tester.getCenter(find.text('BottomSheet'));

    final double dyDelta1 = secondPosition.dy - firstPosition.dy;
    final double dyDelta2 = thirdPosition.dy - secondPosition.dy;

    // If the animation were linear, these two values would be the same.
    expect(dyDelta1, isNot(moreOrLessEquals(dyDelta2, epsilon: 0.1)));
  }

  testWidgets('Throw if enable drag without an animation controller', (WidgetTester tester) async {
    // Regression test for https://github.com/flutter/flutter/issues/89168
    await tester.pumpWidget(
      MaterialApp(
        home: BottomSheet(
          onClosing: () {},
          builder:
              (_) => Container(height: 200, color: Colors.red, child: const Text('BottomSheet')),
        ),
      ),
    );

    final FlutterExceptionHandler? handler = FlutterError.onError;
    FlutterErrorDetails? error;
    FlutterError.onError = (FlutterErrorDetails details) {
      error = details;
    };

    await tester.drag(find.text('BottomSheet'), const Offset(0.0, 150.0));

    expect(error, isNotNull);
    FlutterError.onError = handler;
  });

  testWidgets('Disposing app while bottom sheet is disappearing does not crash', (
    WidgetTester tester,
  ) async {
    late BuildContext savedContext;

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
    expect(find.text('BottomSheet'), findsNothing);

    // Bring up bottom sheet.
    bool showBottomSheetThenCalled = false;
    showModalBottomSheet<void>(
      context: savedContext,
      builder: (BuildContext context) => const Text('BottomSheet'),
    ).then<void>((void value) {
      showBottomSheetThenCalled = true;
    });
    await tester.pumpAndSettle();
    expect(find.text('BottomSheet'), findsOneWidget);
    expect(showBottomSheetThenCalled, isFalse);

    // Start closing animation of Bottom sheet.
    tester.state<NavigatorState>(find.byType(Navigator)).pop();
    await tester.pump();

    // Dispose app by replacing it with a container. This shouldn't crash.
    await tester.pumpWidget(Container());
  });

  testWidgets('Swiping down a BottomSheet should dismiss it by default', (
    WidgetTester tester,
  ) async {
    final GlobalKey<ScaffoldState> scaffoldKey = GlobalKey<ScaffoldState>();
    bool showBottomSheetThenCalled = false;

    await tester.pumpWidget(
      MaterialApp(home: Scaffold(key: scaffoldKey, body: const Center(child: Text('body')))),
    );

    await tester.pump();
    expect(showBottomSheetThenCalled, isFalse);
    expect(find.text('BottomSheet'), findsNothing);

    scaffoldKey.currentState!
        .showBottomSheet((BuildContext context) {
          return const SizedBox(height: 200.0, child: Text('BottomSheet'));
        })
        .closed
        .whenComplete(() {
          showBottomSheetThenCalled = true;
        });

    await tester.pumpAndSettle();
    expect(showBottomSheetThenCalled, isFalse);
    expect(find.text('BottomSheet'), findsOneWidget);

    // Swipe the bottom sheet to dismiss it.
    await tester.drag(find.text('BottomSheet'), const Offset(0.0, 150.0));
    await tester.pumpAndSettle(); // Bottom sheet dismiss animation.
    expect(showBottomSheetThenCalled, isTrue);
    expect(find.text('BottomSheet'), findsNothing);
  });

  testWidgets('Swiping down a BottomSheet should not dismiss it when enableDrag is false', (
    WidgetTester tester,
  ) async {
    final GlobalKey<ScaffoldState> scaffoldKey = GlobalKey<ScaffoldState>();
    bool showBottomSheetThenCalled = false;

    await tester.pumpWidget(
      MaterialApp(home: Scaffold(key: scaffoldKey, body: const Center(child: Text('body')))),
    );

    await tester.pump();
    expect(showBottomSheetThenCalled, isFalse);
    expect(find.text('BottomSheet'), findsNothing);

    scaffoldKey.currentState!
        .showBottomSheet((BuildContext context) {
          return const SizedBox(height: 200.0, child: Text('BottomSheet'));
        }, enableDrag: false)
        .closed
        .whenComplete(() {
          showBottomSheetThenCalled = true;
        });

    await tester.pumpAndSettle();
    expect(showBottomSheetThenCalled, isFalse);
    expect(find.text('BottomSheet'), findsOneWidget);

    // Swipe the bottom sheet, attempting to dismiss it.
    await tester.drag(find.text('BottomSheet'), const Offset(0.0, 150.0));
    await tester.pumpAndSettle(); // Bottom sheet should not dismiss.
    expect(showBottomSheetThenCalled, isFalse);
    expect(find.text('BottomSheet'), findsOneWidget);
  });

  testWidgets('Swiping down a BottomSheet should dismiss it when enableDrag is true', (
    WidgetTester tester,
  ) async {
    final GlobalKey<ScaffoldState> scaffoldKey = GlobalKey<ScaffoldState>();
    bool showBottomSheetThenCalled = false;

    await tester.pumpWidget(
      MaterialApp(home: Scaffold(key: scaffoldKey, body: const Center(child: Text('body')))),
    );

    await tester.pump();
    expect(showBottomSheetThenCalled, isFalse);
    expect(find.text('BottomSheet'), findsNothing);

    scaffoldKey.currentState!
        .showBottomSheet((BuildContext context) {
          return const SizedBox(height: 200.0, child: Text('BottomSheet'));
        }, enableDrag: true)
        .closed
        .whenComplete(() {
          showBottomSheetThenCalled = true;
        });

    await tester.pumpAndSettle();
    expect(showBottomSheetThenCalled, isFalse);
    expect(find.text('BottomSheet'), findsOneWidget);

    // Swipe the bottom sheet to dismiss it.
    await tester.drag(find.text('BottomSheet'), const Offset(0.0, 150.0));
    await tester.pumpAndSettle(); // Bottom sheet dismiss animation.
    expect(showBottomSheetThenCalled, isTrue);
    expect(find.text('BottomSheet'), findsNothing);
  });

  testWidgets('Tapping on a BottomSheet should not trigger a rebuild when enableDrag is true', (
    WidgetTester tester,
  ) async {
    // Regression test for https://github.com/flutter/flutter/issues/126833.
    final GlobalKey<ScaffoldState> scaffoldKey = GlobalKey<ScaffoldState>();
    int buildCount = 0;

    await tester.pumpWidget(
      MaterialApp(home: Scaffold(key: scaffoldKey, body: const Center(child: Text('body')))),
    );

    await tester.pump();
    expect(buildCount, 0);
    expect(find.text('BottomSheet'), findsNothing);

    scaffoldKey.currentState!.showBottomSheet((BuildContext context) {
      buildCount++;
      return const SizedBox(height: 200.0, child: Text('BottomSheet'));
    }, enableDrag: true);

    await tester.pumpAndSettle();
    expect(buildCount, 1);
    expect(find.text('BottomSheet'), findsOneWidget);

    // Tap on bottom sheet should not trigger a rebuild.
    await tester.tap(find.text('BottomSheet'));
    await tester.pumpAndSettle();
    expect(buildCount, 1);
    expect(find.text('BottomSheet'), findsOneWidget);
  });

  testWidgets('Modal BottomSheet builder should only be called once', (WidgetTester tester) async {
    late BuildContext savedContext;

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

    int numBuilderCalls = 0;
    showModalBottomSheet<void>(
      context: savedContext,
      isDismissible: false,
      builder: (BuildContext context) {
        numBuilderCalls++;
        return const Text('BottomSheet');
      },
    );

    await tester.pumpAndSettle();
    expect(numBuilderCalls, 1);

    // Swipe the bottom sheet to dismiss it.
    await tester.drag(find.text('BottomSheet'), const Offset(0.0, 150.0));
    await tester.pumpAndSettle(); // Bottom sheet dismiss animation.
    expect(numBuilderCalls, 1);
  });

  testWidgets('Tapping on a modal BottomSheet should not dismiss it', (WidgetTester tester) async {
    late BuildContext savedContext;

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
    expect(find.text('BottomSheet'), findsNothing);

    bool showBottomSheetThenCalled = false;
    showModalBottomSheet<void>(
      context: savedContext,
      builder: (BuildContext context) => const Text('BottomSheet'),
    ).then<void>((void value) {
      showBottomSheetThenCalled = true;
    });

    await tester.pumpAndSettle();
    expect(find.text('BottomSheet'), findsOneWidget);
    expect(showBottomSheetThenCalled, isFalse);

    // Tap on the bottom sheet itself, it should not be dismissed
    await tester.tap(find.text('BottomSheet'));
    await tester.pumpAndSettle();
    expect(find.text('BottomSheet'), findsOneWidget);
    expect(showBottomSheetThenCalled, isFalse);
  });

  testWidgets('Tapping outside a modal BottomSheet should dismiss it by default', (
    WidgetTester tester,
  ) async {
    late BuildContext savedContext;

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
    expect(find.text('BottomSheet'), findsNothing);

    bool showBottomSheetThenCalled = false;
    showModalBottomSheet<void>(
      context: savedContext,
      builder: (BuildContext context) => const Text('BottomSheet'),
    ).then<void>((void value) {
      showBottomSheetThenCalled = true;
    });

    await tester.pumpAndSettle();
    expect(find.text('BottomSheet'), findsOneWidget);
    expect(showBottomSheetThenCalled, isFalse);

    // Tap above the bottom sheet to dismiss it.
    await tester.tapAt(const Offset(20.0, 20.0));
    await tester.pumpAndSettle(); // Bottom sheet dismiss animation.
    expect(showBottomSheetThenCalled, isTrue);
    expect(find.text('BottomSheet'), findsNothing);
  });

  testWidgets('Tapping outside a modal BottomSheet should dismiss it when isDismissible=true', (
    WidgetTester tester,
  ) async {
    late BuildContext savedContext;

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
    expect(find.text('BottomSheet'), findsNothing);

    bool showBottomSheetThenCalled = false;
    showModalBottomSheet<void>(
      context: savedContext,
      builder: (BuildContext context) => const Text('BottomSheet'),
    ).then<void>((void value) {
      showBottomSheetThenCalled = true;
    });

    await tester.pumpAndSettle();
    expect(find.text('BottomSheet'), findsOneWidget);
    expect(showBottomSheetThenCalled, isFalse);

    // Tap above the bottom sheet to dismiss it.
    await tester.tapAt(const Offset(20.0, 20.0));
    await tester.pumpAndSettle(); // Bottom sheet dismiss animation.
    expect(showBottomSheetThenCalled, isTrue);
    expect(find.text('BottomSheet'), findsNothing);
  });

  testWidgets('Verify that the BottomSheet animates non-linearly', (WidgetTester tester) async {
    late BuildContext savedContext;

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
    expect(find.text('BottomSheet'), findsNothing);

    showModalBottomSheet<void>(
      context: savedContext,
      builder: (BuildContext context) => const Text('BottomSheet'),
    );
    await tester.pump();

    await checkNonLinearAnimation(tester);
    await tester.pumpAndSettle();

    // Tap above the bottom sheet to dismiss it.
    await tester.tapAt(const Offset(20.0, 20.0));
    await tester.pump();
    await checkNonLinearAnimation(tester);
    await tester.pumpAndSettle(); // Bottom sheet dismiss animation.
    expect(find.text('BottomSheet'), findsNothing);
  });

  // Regression test for https://github.com/flutter/flutter/issues/121098
  testWidgets('Verify that accessibleNavigation has no impact on the BottomSheet animation', (
    WidgetTester tester,
  ) async {
    await tester.pumpWidget(
      MaterialApp(
        builder: (BuildContext context, Widget? child) {
          return MediaQuery(data: const MediaQueryData(accessibleNavigation: true), child: child!);
        },
        home: const Center(child: Text('Test')),
      ),
    );

    await tester.pump();
    expect(find.text('BottomSheet'), findsNothing);

    final BuildContext homeContext = tester.element(find.text('Test'));
    showModalBottomSheet<void>(
      context: homeContext,
      builder: (BuildContext context) => const Text('BottomSheet'),
    );
    await tester.pump();

    await checkNonLinearAnimation(tester);
    await tester.pumpAndSettle();
  });

  testWidgets(
    'Tapping outside a modal BottomSheet should not dismiss it when isDismissible=false',
    (WidgetTester tester) async {
      late BuildContext savedContext;

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
      expect(find.text('BottomSheet'), findsNothing);

      bool showBottomSheetThenCalled = false;
      showModalBottomSheet<void>(
        context: savedContext,
        builder: (BuildContext context) => const Text('BottomSheet'),
        isDismissible: false,
      ).then<void>((void value) {
        showBottomSheetThenCalled = true;
      });

      await tester.pumpAndSettle();
      expect(find.text('BottomSheet'), findsOneWidget);
      expect(showBottomSheetThenCalled, isFalse);

      // Tap above the bottom sheet, attempting to dismiss it.
      await tester.tapAt(const Offset(20.0, 20.0));
      await tester.pumpAndSettle(); // Bottom sheet should not dismiss.
      expect(showBottomSheetThenCalled, isFalse);
      expect(find.text('BottomSheet'), findsOneWidget);
    },
  );

  testWidgets('Swiping down a modal BottomSheet should dismiss it by default', (
    WidgetTester tester,
  ) async {
    late BuildContext savedContext;

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
    expect(find.text('BottomSheet'), findsNothing);

    bool showBottomSheetThenCalled = false;
    showModalBottomSheet<void>(
      context: savedContext,
      isDismissible: false,
      builder: (BuildContext context) => const Text('BottomSheet'),
    ).then<void>((void value) {
      showBottomSheetThenCalled = true;
    });

    await tester.pumpAndSettle();
    expect(find.text('BottomSheet'), findsOneWidget);
    expect(showBottomSheetThenCalled, isFalse);

    // Swipe the bottom sheet to dismiss it.
    await tester.drag(find.text('BottomSheet'), const Offset(0.0, 150.0));
    await tester.pumpAndSettle(); // Bottom sheet dismiss animation.
    expect(showBottomSheetThenCalled, isTrue);
    expect(find.text('BottomSheet'), findsNothing);
  });

  testWidgets('Swiping down a modal BottomSheet should not dismiss it when enableDrag is false', (
    WidgetTester tester,
  ) async {
    late BuildContext savedContext;

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
    expect(find.text('BottomSheet'), findsNothing);

    bool showBottomSheetThenCalled = false;
    showModalBottomSheet<void>(
      context: savedContext,
      isDismissible: false,
      enableDrag: false,
      builder: (BuildContext context) => const Text('BottomSheet'),
    ).then<void>((void value) {
      showBottomSheetThenCalled = true;
    });

    await tester.pumpAndSettle();
    expect(find.text('BottomSheet'), findsOneWidget);
    expect(showBottomSheetThenCalled, isFalse);

    // Swipe the bottom sheet, attempting to dismiss it.
    await tester.drag(find.text('BottomSheet'), const Offset(0.0, 150.0));
    await tester.pumpAndSettle(); // Bottom sheet should not dismiss.
    expect(showBottomSheetThenCalled, isFalse);
    expect(find.text('BottomSheet'), findsOneWidget);
  });

  testWidgets('Swiping down a modal BottomSheet should dismiss it when enableDrag is true', (
    WidgetTester tester,
  ) async {
    late BuildContext savedContext;

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
    expect(find.text('BottomSheet'), findsNothing);

    bool showBottomSheetThenCalled = false;
    showModalBottomSheet<void>(
      context: savedContext,
      isDismissible: false,
      builder: (BuildContext context) => const Text('BottomSheet'),
    ).then<void>((void value) {
      showBottomSheetThenCalled = true;
    });

    await tester.pumpAndSettle();
    expect(find.text('BottomSheet'), findsOneWidget);
    expect(showBottomSheetThenCalled, isFalse);

    // Swipe the bottom sheet to dismiss it.
    await tester.drag(find.text('BottomSheet'), const Offset(0.0, 150.0));
    await tester.pumpAndSettle(); // Bottom sheet dismiss animation.
    expect(showBottomSheetThenCalled, isTrue);
    expect(find.text('BottomSheet'), findsNothing);
  });

  testWidgets('Modal BottomSheet builder should only be called once', (WidgetTester tester) async {
    late BuildContext savedContext;

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

    int numBuilderCalls = 0;
    showModalBottomSheet<void>(
      context: savedContext,
      isDismissible: false,
      builder: (BuildContext context) {
        numBuilderCalls++;
        return const Text('BottomSheet');
      },
    );

    await tester.pumpAndSettle();
    expect(numBuilderCalls, 1);

    // Swipe the bottom sheet to dismiss it.
    await tester.drag(find.text('BottomSheet'), const Offset(0.0, 150.0));
    await tester.pumpAndSettle(); // Bottom sheet dismiss animation.
    expect(numBuilderCalls, 1);
  });

  testWidgets('Verify that a downwards fling dismisses a persistent BottomSheet', (
    WidgetTester tester,
  ) async {
    final GlobalKey<ScaffoldState> scaffoldKey = GlobalKey<ScaffoldState>();
    bool showBottomSheetThenCalled = false;

    await tester.pumpWidget(
      MaterialApp(home: Scaffold(key: scaffoldKey, body: const Center(child: Text('body')))),
    );

    expect(showBottomSheetThenCalled, isFalse);
    expect(find.text('BottomSheet'), findsNothing);

    scaffoldKey.currentState!
        .showBottomSheet((BuildContext context) {
          return Container(margin: const EdgeInsets.all(40.0), child: const Text('BottomSheet'));
        })
        .closed
        .whenComplete(() {
          showBottomSheetThenCalled = true;
        });

    expect(showBottomSheetThenCalled, isFalse);
    expect(find.text('BottomSheet'), findsNothing);

    await tester.pump(); // bottom sheet show animation starts

    expect(showBottomSheetThenCalled, isFalse);
    expect(find.text('BottomSheet'), findsOneWidget);

    await tester.pump(const Duration(seconds: 1)); // animation done

    expect(showBottomSheetThenCalled, isFalse);
    expect(find.text('BottomSheet'), findsOneWidget);

    // The fling below must be such that the velocity estimation examines an
    // offset greater than the kTouchSlop. Too slow or too short a distance, and
    // it won't trigger. Also, it must not be so much that it drags the bottom
    // sheet off the screen, or we won't see it after we pump!
    await tester.fling(find.text('BottomSheet'), const Offset(0.0, 50.0), 2000.0);
    await tester.pump(); // drain the microtask queue (Future completion callback)

    expect(showBottomSheetThenCalled, isTrue);
    expect(find.text('BottomSheet'), findsOneWidget);

    await tester.pump(); // bottom sheet dismiss animation starts

    expect(showBottomSheetThenCalled, isTrue);
    expect(find.text('BottomSheet'), findsOneWidget);

    await tester.pump(const Duration(seconds: 1)); // animation done

    expect(showBottomSheetThenCalled, isTrue);
    expect(find.text('BottomSheet'), findsNothing);
  });

  testWidgets('Verify that dragging past the bottom dismisses a persistent BottomSheet', (
    WidgetTester tester,
  ) async {
    // This is a regression test for https://github.com/flutter/flutter/issues/5528
    final GlobalKey<ScaffoldState> scaffoldKey = GlobalKey<ScaffoldState>();

    await tester.pumpWidget(
      MaterialApp(home: Scaffold(key: scaffoldKey, body: const Center(child: Text('body')))),
    );

    scaffoldKey.currentState!.showBottomSheet((BuildContext context) {
      return Container(margin: const EdgeInsets.all(40.0), child: const Text('BottomSheet'));
    });

    await tester.pump(); // bottom sheet show animation starts
    await tester.pump(const Duration(seconds: 1)); // animation done
    expect(find.text('BottomSheet'), findsOneWidget);

    await tester.fling(find.text('BottomSheet'), const Offset(0.0, 400.0), 1000.0);
    await tester.pump(); // drain the microtask queue (Future completion callback)
    await tester.pump(); // bottom sheet dismiss animation starts
    await tester.pump(const Duration(seconds: 1)); // animation done

    expect(find.text('BottomSheet'), findsNothing);
  });

  testWidgets('modal BottomSheet has no top MediaQuery', (WidgetTester tester) async {
    late BuildContext outerContext;
    late BuildContext innerContext;

    await tester.pumpWidget(
      Localizations(
        locale: const Locale('en', 'US'),
        delegates: const <LocalizationsDelegate<dynamic>>[
          DefaultWidgetsLocalizations.delegate,
          DefaultMaterialLocalizations.delegate,
        ],
        child: Directionality(
          textDirection: TextDirection.ltr,
          child: MediaQuery(
            data: const MediaQueryData(padding: EdgeInsets.all(50.0), size: Size(400.0, 600.0)),
            child: Navigator(
              onGenerateRoute: (_) {
                return PageRouteBuilder<void>(
                  pageBuilder: (
                    BuildContext context,
                    Animation<double> animation,
                    Animation<double> secondaryAnimation,
                  ) {
                    outerContext = context;
                    return Container();
                  },
                );
              },
            ),
          ),
        ),
      ),
    );

    showModalBottomSheet<void>(
      context: outerContext,
      builder: (BuildContext context) {
        innerContext = context;
        return Container();
      },
    );
    await tester.pump();
    await tester.pump(const Duration(seconds: 1));

    expect(MediaQuery.of(outerContext).padding, const EdgeInsets.all(50.0));
    expect(
      MediaQuery.of(innerContext).padding,
      const EdgeInsets.only(left: 50.0, right: 50.0, bottom: 50.0),
    );
  });

  testWidgets('modal BottomSheet can insert a SafeArea', (WidgetTester tester) async {
    late BuildContext outerContext;
    late BuildContext innerContext;

    await tester.pumpWidget(
      Localizations(
        locale: const Locale('en', 'US'),
        delegates: const <LocalizationsDelegate<dynamic>>[
          DefaultWidgetsLocalizations.delegate,
          DefaultMaterialLocalizations.delegate,
        ],
        child: Directionality(
          textDirection: TextDirection.ltr,
          child: MediaQuery(
            data: const MediaQueryData(padding: EdgeInsets.all(50.0), size: Size(400.0, 600.0)),
            child: Navigator(
              onGenerateRoute: (_) {
                return PageRouteBuilder<void>(
                  pageBuilder: (
                    BuildContext context,
                    Animation<double> animation,
                    Animation<double> secondaryAnimation,
                  ) {
                    outerContext = context;
                    return Container();
                  },
                );
              },
            ),
          ),
        ),
      ),
    );

    // Without a SafeArea (useSafeArea is false by default)
    showModalBottomSheet<void>(
      context: outerContext,
      builder: (BuildContext context) {
        innerContext = context;
        return Container();
      },
    );
    await tester.pump();
    await tester.pump(const Duration(seconds: 1));

    // Top padding is consumed and there is no SafeArea
    expect(MediaQuery.of(innerContext).padding.top, 0);
    expect(find.byType(SafeArea), findsNothing);

    // With a SafeArea
    showModalBottomSheet<void>(
      context: outerContext,
      useSafeArea: true,
      builder: (BuildContext context) {
        innerContext = context;
        return Container();
      },
    );
    await tester.pump();
    await tester.pump(const Duration(seconds: 1));

    // A SafeArea is inserted, with left / top / right true but bottom false.
    final Finder safeAreaWidgetFinder = find.byType(SafeArea);
    expect(safeAreaWidgetFinder, findsOneWidget);
    final SafeArea safeAreaWidget = safeAreaWidgetFinder.evaluate().single.widget as SafeArea;
    expect(safeAreaWidget.left, true);
    expect(safeAreaWidget.top, true);
    expect(safeAreaWidget.right, true);
    expect(safeAreaWidget.bottom, false);

    // Because that SafeArea is inserted, no left / top / right padding remains
    // for `builder` to consume. Bottom padding does remain.
    expect(MediaQuery.of(innerContext).padding, const EdgeInsets.fromLTRB(0, 0, 0, 50.0));
  });

  testWidgets('modal BottomSheet has semantics', (WidgetTester tester) async {
    final SemanticsTester semantics = SemanticsTester(tester);
    final GlobalKey<ScaffoldState> scaffoldKey = GlobalKey<ScaffoldState>();

    await tester.pumpWidget(
      MaterialApp(home: Scaffold(key: scaffoldKey, body: const Center(child: Text('body')))),
    );

    showModalBottomSheet<void>(
      context: scaffoldKey.currentContext!,
      builder: (BuildContext context) {
        return const Text('BottomSheet');
      },
    );

    await tester.pump(); // bottom sheet show animation starts
    await tester.pump(const Duration(seconds: 1)); // animation done

    expect(
      semantics,
      hasSemantics(
        TestSemantics.root(
          children: <TestSemantics>[
            TestSemantics.rootChild(
              children: <TestSemantics>[
                TestSemantics(
                  children: <TestSemantics>[
                    TestSemantics(
                      label: 'Dialog',
                      textDirection: TextDirection.ltr,
                      flags: <SemanticsFlag>[SemanticsFlag.scopesRoute, SemanticsFlag.namesRoute],
                      children: <TestSemantics>[
                        TestSemantics(label: 'BottomSheet', textDirection: TextDirection.ltr),
                      ],
                    ),
                  ],
                ),
                TestSemantics(
                  children: <TestSemantics>[
                    TestSemantics(
                      actions: <SemanticsAction>[SemanticsAction.tap, SemanticsAction.dismiss],
                      label: 'Scrim',
                      textDirection: TextDirection.ltr,
                    ),
                  ],
                ),
              ],
            ),
          ],
        ),
        ignoreTransform: true,
        ignoreRect: true,
        ignoreId: true,
      ),
    );
    semantics.dispose();
  });

  testWidgets('Verify that visual properties are passed through', (WidgetTester tester) async {
    final GlobalKey<ScaffoldState> scaffoldKey = GlobalKey<ScaffoldState>();
    const Color color = Colors.pink;
    const double elevation = 9.0;
    const ShapeBorder shape = BeveledRectangleBorder(
      borderRadius: BorderRadius.all(Radius.circular(12)),
    );
    const Clip clipBehavior = Clip.antiAlias;
    const Color barrierColor = Colors.red;

    await tester.pumpWidget(
      MaterialApp(home: Scaffold(key: scaffoldKey, body: const Center(child: Text('body')))),
    );

    showModalBottomSheet<void>(
      context: scaffoldKey.currentContext!,
      backgroundColor: color,
      barrierColor: barrierColor,
      elevation: elevation,
      shape: shape,
      clipBehavior: clipBehavior,
      builder: (BuildContext context) {
        return const Text('BottomSheet');
      },
    );

    await tester.pump();
    await tester.pump(const Duration(seconds: 1));

    final BottomSheet bottomSheet = tester.widget(find.byType(BottomSheet));
    expect(bottomSheet.backgroundColor, color);
    expect(bottomSheet.elevation, elevation);
    expect(bottomSheet.shape, shape);
    expect(bottomSheet.clipBehavior, clipBehavior);

    final ModalBarrier modalBarrier = tester.widget(find.byType(ModalBarrier).last);
    expect(modalBarrier.color, barrierColor);
  });

  testWidgets('Material3 - BottomSheet uses fallback values', (WidgetTester tester) async {
    const Color surfaceColor = Colors.pink;
    const Color surfaceTintColor = Colors.blue;
    const ShapeBorder defaultShape = RoundedRectangleBorder(
      borderRadius: BorderRadius.vertical(top: Radius.circular(28.0)),
    );

    await tester.pumpWidget(
      MaterialApp(
        theme: ThemeData(
          colorScheme: const ColorScheme.light(
            surface: surfaceColor,
            surfaceTint: surfaceTintColor,
          ),
          useMaterial3: true,
        ),
        home: Scaffold(
          body: BottomSheet(
            onClosing: () {},
            builder: (BuildContext context) {
              return Container();
            },
          ),
        ),
      ),
    );

    final Finder finder = find.descendant(
      of: find.byType(BottomSheet),
      matching: find.byType(Material),
    );
    final Material material = tester.widget<Material>(finder);

    expect(material.color, surfaceColor);
    // Surface tint is no longer used by default.
    expect(material.surfaceTintColor, Colors.transparent);
    expect(material.elevation, 1.0);
    expect(material.shape, defaultShape);
    expect(tester.getSize(finder).width, 640);
  });

  testWidgets('Material3 - BottomSheet has transparent shadow', (WidgetTester tester) async {
    await tester.pumpWidget(
      MaterialApp(
        theme: ThemeData(useMaterial3: true),
        home: Scaffold(
          body: BottomSheet(
            onClosing: () {},
            builder: (BuildContext context) {
              return Container();
            },
          ),
        ),
      ),
    );

    final Material material = tester.widget<Material>(
      find.descendant(of: find.byType(BottomSheet), matching: find.byType(Material)),
    );
    expect(material.shadowColor, Colors.transparent);
  });

  testWidgets('Material2 - Modal BottomSheet with ScrollController has semantics', (
    WidgetTester tester,
  ) async {
    final SemanticsTester semantics = SemanticsTester(tester);
    final GlobalKey<ScaffoldState> scaffoldKey = GlobalKey<ScaffoldState>();

    await tester.pumpWidget(
      MaterialApp(
        theme: ThemeData(useMaterial3: false),
        home: Scaffold(key: scaffoldKey, body: const Center(child: Text('body'))),
      ),
    );

    showModalBottomSheet<void>(
      context: scaffoldKey.currentContext!,
      builder: (BuildContext context) {
        return DraggableScrollableSheet(
          expand: false,
          builder: (_, ScrollController controller) {
            return SingleChildScrollView(controller: controller, child: const Text('BottomSheet'));
          },
        );
      },
    );

    await tester.pump(); // bottom sheet show animation starts
    await tester.pump(const Duration(seconds: 1)); // animation done

    expect(
      semantics,
      hasSemantics(
        TestSemantics.root(
          children: <TestSemantics>[
            TestSemantics.rootChild(
              children: <TestSemantics>[
                TestSemantics(
                  children: <TestSemantics>[
                    TestSemantics(
                      label: 'Dialog',
                      textDirection: TextDirection.ltr,
                      flags: <SemanticsFlag>[SemanticsFlag.scopesRoute, SemanticsFlag.namesRoute],
                      children: <TestSemantics>[
                        TestSemantics(
                          flags: <SemanticsFlag>[SemanticsFlag.hasImplicitScrolling],
                          children: <TestSemantics>[
                            TestSemantics(label: 'BottomSheet', textDirection: TextDirection.ltr),
                          ],
                        ),
                      ],
                    ),
                  ],
                ),
                TestSemantics(
                  children: <TestSemantics>[
                    TestSemantics(
                      actions: <SemanticsAction>[SemanticsAction.tap, SemanticsAction.dismiss],
                      label: 'Scrim',
                      textDirection: TextDirection.ltr,
                    ),
                  ],
                ),
              ],
            ),
          ],
        ),
        ignoreTransform: true,
        ignoreRect: true,
        ignoreId: true,
      ),
    );
    semantics.dispose();
  });

  testWidgets('Material3 - Modal BottomSheet with ScrollController has semantics', (
    WidgetTester tester,
  ) async {
    final SemanticsTester semantics = SemanticsTester(tester);
    final GlobalKey<ScaffoldState> scaffoldKey = GlobalKey<ScaffoldState>();

    await tester.pumpWidget(
      MaterialApp(
        theme: ThemeData(useMaterial3: true),
        home: Scaffold(key: scaffoldKey, body: const Center(child: Text('body'))),
      ),
    );

    showModalBottomSheet<void>(
      context: scaffoldKey.currentContext!,
      builder: (BuildContext context) {
        return DraggableScrollableSheet(
          expand: false,
          builder: (_, ScrollController controller) {
            return SingleChildScrollView(controller: controller, child: const Text('BottomSheet'));
          },
        );
      },
    );

    await tester.pump(); // bottom sheet show animation starts
    await tester.pump(const Duration(seconds: 1)); // animation done

    expect(
      semantics,
      hasSemantics(
        TestSemantics.root(
          children: <TestSemantics>[
            TestSemantics.rootChild(
              children: <TestSemantics>[
                TestSemantics(
                  children: <TestSemantics>[
                    TestSemantics(
                      label: 'Dialog',
                      textDirection: TextDirection.ltr,
                      flags: <SemanticsFlag>[SemanticsFlag.scopesRoute, SemanticsFlag.namesRoute],
                      children: <TestSemantics>[
                        TestSemantics(
                          children: <TestSemantics>[
                            TestSemantics(
                              flags: <SemanticsFlag>[SemanticsFlag.hasImplicitScrolling],
                              children: <TestSemantics>[
                                TestSemantics(
                                  label: 'BottomSheet',
                                  textDirection: TextDirection.ltr,
                                ),
                              ],
                            ),
                          ],
                        ),
                      ],
                    ),
                  ],
                ),
                TestSemantics(
                  children: <TestSemantics>[
                    TestSemantics(
                      actions: <SemanticsAction>[SemanticsAction.tap, SemanticsAction.dismiss],
                      label: 'Scrim',
                      textDirection: TextDirection.ltr,
                    ),
                  ],
                ),
              ],
            ),
          ],
        ),
        ignoreTransform: true,
        ignoreRect: true,
        ignoreId: true,
      ),
    );
    semantics.dispose();
  });

  testWidgets('Material3 - Modal BottomSheet with drag handle has semantics', (
    WidgetTester tester,
  ) async {
    final SemanticsTester semantics = SemanticsTester(tester);
    final GlobalKey<ScaffoldState> scaffoldKey = GlobalKey<ScaffoldState>();

    await tester.pumpWidget(
      MaterialApp(
        theme: ThemeData(useMaterial3: true),
        home: Scaffold(key: scaffoldKey, body: const Center(child: Text('body'))),
      ),
    );

    showModalBottomSheet<void>(
      context: scaffoldKey.currentContext!,
      showDragHandle: true,
      builder: (BuildContext context) {
        return const Text('BottomSheet');
      },
    );

    await tester.pump(); // bottom sheet show animation starts
    await tester.pump(const Duration(seconds: 1)); // animation done

    expect(
      semantics,
      hasSemantics(
        TestSemantics.root(
          children: <TestSemantics>[
            TestSemantics.rootChild(
              children: <TestSemantics>[
                TestSemantics(
                  children: <TestSemantics>[
                    TestSemantics(
                      label: 'Dialog',
                      textDirection: TextDirection.ltr,
                      flags: <SemanticsFlag>[SemanticsFlag.scopesRoute, SemanticsFlag.namesRoute],
                      children: <TestSemantics>[
                        TestSemantics(
                          label: 'BottomSheet',
                          textDirection: TextDirection.ltr,
                          children: <TestSemantics>[
                            TestSemantics(
                              actions: <SemanticsAction>[SemanticsAction.tap],
                              label: 'Dismiss',
                              textDirection: TextDirection.ltr,
                            ),
                          ],
                        ),
                      ],
                    ),
                  ],
                ),
                TestSemantics(
                  children: <TestSemantics>[
                    TestSemantics(
                      actions: <SemanticsAction>[SemanticsAction.tap, SemanticsAction.dismiss],
                      label: 'Scrim',
                      textDirection: TextDirection.ltr,
                    ),
                  ],
                ),
              ],
            ),
          ],
        ),
        ignoreTransform: true,
        ignoreRect: true,
        ignoreId: true,
      ),
    );
    semantics.dispose();
  });

  testWidgets('Drag handle color can take MaterialStateProperty', (WidgetTester tester) async {
    const Color defaultColor = Colors.blue;
    const Color hoveringColor = Colors.green;

    Future<void> checkDragHandleAndColors() async {
      await tester.pump(); // bottom sheet show animation starts
      await tester.pump(const Duration(seconds: 1)); // animation done

      final Finder dragHandle = find.bySemanticsLabel('Dismiss');
      expect(tester.getSize(dragHandle), const Size(48, 48));
      final Offset center = tester.getCenter(dragHandle);
      final Offset edge = tester.getTopLeft(dragHandle) - const Offset(1, 1);

      // Shows default drag handle color
      final TestGesture gesture = await tester.createGesture(
        kind: PointerDeviceKind.mouse,
        pointer: 1,
      );
      await gesture.addPointer(location: edge);
      await tester.pump();
      BoxDecoration boxDecoration =
          tester
                  .widget<Container>(
                    find.descendant(
                      of: dragHandle,
                      matching: find.byWidgetPredicate(
                        (Widget widget) => widget is Container && widget.decoration != null,
                      ),
                    ),
                  )
                  .decoration!
              as BoxDecoration;
      expect(boxDecoration.color, defaultColor);

      // Shows hovering drag handle color
      await gesture.moveTo(center);
      await tester.pump();
      boxDecoration =
          tester
                  .widget<Container>(
                    find.descendant(
                      of: dragHandle,
                      matching: find.byWidgetPredicate(
                        (Widget widget) => widget is Container && widget.decoration != null,
                      ),
                    ),
                  )
                  .decoration!
              as BoxDecoration;

      expect(boxDecoration.color, hoveringColor);
      await gesture.removePointer();
    }

    Widget buildScaffold(GlobalKey scaffoldKey) {
      return MaterialApp(
        theme: ThemeData.light().copyWith(
          bottomSheetTheme: BottomSheetThemeData(
            dragHandleColor: MaterialStateColor.resolveWith((Set<MaterialState> states) {
              if (states.contains(MaterialState.hovered)) {
                return hoveringColor;
              }
              return defaultColor;
            }),
          ),
        ),
        home: Scaffold(key: scaffoldKey, body: const Center(child: Text('body'))),
      );
    }

    GlobalKey<ScaffoldState> scaffoldKey = GlobalKey<ScaffoldState>();
    await tester.pumpWidget(buildScaffold(scaffoldKey));

    showModalBottomSheet<void>(
      context: scaffoldKey.currentContext!,
      showDragHandle: true,
      builder: (BuildContext context) {
        return const Text('BottomSheet');
      },
    );

    await checkDragHandleAndColors();

    await tester.pumpWidget(Container()); // Reset
    scaffoldKey = GlobalKey<ScaffoldState>();
    await tester.pumpWidget(buildScaffold(scaffoldKey));

    scaffoldKey.currentState!.showBottomSheet((_) {
      return Builder(
        builder: (BuildContext context) {
          return const SizedBox(height: 200.0, child: Text('Bottom Sheet'));
        },
      );
    }, showDragHandle: true);

    await checkDragHandleAndColors();
  });

  testWidgets('Drag handle interactive area size at minimum possible size', (
    WidgetTester tester,
  ) async {
    Widget buildScaffold(GlobalKey scaffoldKey, {Size? dragHandleSize}) {
      return MaterialApp(
        theme: ThemeData.light().copyWith(
          bottomSheetTheme: BottomSheetThemeData(dragHandleSize: dragHandleSize),
        ),
        home: Scaffold(key: scaffoldKey),
      );
    }

    const Size smallerDragHandleSize = Size(20, 20);

    final GlobalKey<ScaffoldState> scaffoldKey = GlobalKey<ScaffoldState>();
    await tester.pumpWidget(buildScaffold(scaffoldKey, dragHandleSize: smallerDragHandleSize));

    showModalBottomSheet<void>(
      context: scaffoldKey.currentContext!,
      showDragHandle: true,
      builder: (BuildContext context) {
        return const Text('BottomSheet');
      },
    );

    await tester.pump(); // Bottom sheet show animation starts.
    await tester.pump(const Duration(seconds: 1)); // Animation done.

    final Finder dragHandle = find.bySemanticsLabel('Dismiss');
    expect(
      tester.getSize(dragHandle),
      const Size(kMinInteractiveDimension, kMinInteractiveDimension),
    );
  });

  testWidgets('Drag handle interactive area size at given dragHandleSize', (
    WidgetTester tester,
  ) async {
    Widget buildScaffold(GlobalKey scaffoldKey, {Size? dragHandleSize}) {
      return MaterialApp(
        theme: ThemeData.light().copyWith(
          bottomSheetTheme: BottomSheetThemeData(dragHandleSize: dragHandleSize),
        ),
        home: Scaffold(key: scaffoldKey),
      );
    }

    const Size extendedDragHandleSize = Size(100, 50);

    final GlobalKey<ScaffoldState> scaffoldKey = GlobalKey<ScaffoldState>();
    await tester.pumpWidget(buildScaffold(scaffoldKey, dragHandleSize: extendedDragHandleSize));

    showModalBottomSheet<void>(
      context: scaffoldKey.currentContext!,
      showDragHandle: true,
      builder: (BuildContext context) {
        return const Text('BottomSheet');
      },
    );

    await tester.pump(); // Bottom sheet show animation starts.
    await tester.pump(const Duration(seconds: 1)); // Animation done.

    final Finder dragHandle = find.bySemanticsLabel('Dismiss');
    expect(tester.getSize(dragHandle), extendedDragHandleSize);
  });

  testWidgets('showModalBottomSheet does not use root Navigator by default', (
    WidgetTester tester,
  ) async {
    await tester.pumpWidget(
      MaterialApp(
        home: Scaffold(
          body: Navigator(
            onGenerateRoute:
                (RouteSettings settings) => MaterialPageRoute<void>(
                  builder: (_) {
                    return const _TestPage();
                  },
                ),
          ),
          bottomNavigationBar: BottomNavigationBar(
            items: const <BottomNavigationBarItem>[
              BottomNavigationBarItem(icon: Icon(Icons.ac_unit), label: 'Item 1'),
              BottomNavigationBarItem(icon: Icon(Icons.style), label: 'Item 2'),
            ],
          ),
        ),
      ),
    );

    await tester.tap(find.text('Show bottom sheet'));
    await tester.pumpAndSettle();

    // Bottom sheet is displayed in correct position within the inner navigator
    // and above the BottomNavigationBar.
    final double tabBarHeight = tester.getSize(find.byType(BottomNavigationBar)).height;
    expect(tester.getBottomLeft(find.byType(BottomSheet)).dy, 600 - tabBarHeight);
  });

  testWidgets('showModalBottomSheet uses root Navigator when specified', (
    WidgetTester tester,
  ) async {
    await tester.pumpWidget(
      MaterialApp(
        home: Scaffold(
          body: Navigator(
            onGenerateRoute:
                (RouteSettings settings) => MaterialPageRoute<void>(
                  builder: (_) {
                    return const _TestPage(useRootNavigator: true);
                  },
                ),
          ),
          bottomNavigationBar: BottomNavigationBar(
            items: const <BottomNavigationBarItem>[
              BottomNavigationBarItem(icon: Icon(Icons.ac_unit), label: 'Item 1'),
              BottomNavigationBarItem(icon: Icon(Icons.style), label: 'Item 2'),
            ],
          ),
        ),
      ),
    );

    await tester.tap(find.text('Show bottom sheet'));
    await tester.pumpAndSettle();

    // Bottom sheet is displayed in correct position above all content including
    // the BottomNavigationBar.
    expect(tester.getBottomLeft(find.byType(BottomSheet)).dy, 600.0);
  });

  testWidgets('Verify that route settings can be set in the showModalBottomSheet', (
    WidgetTester tester,
  ) async {
    final GlobalKey<ScaffoldState> scaffoldKey = GlobalKey<ScaffoldState>();
    const RouteSettings routeSettings = RouteSettings(
      name: 'route_name',
      arguments: 'route_argument',
    );

    await tester.pumpWidget(
      MaterialApp(home: Scaffold(key: scaffoldKey, body: const Center(child: Text('body')))),
    );

    late RouteSettings retrievedRouteSettings;

    showModalBottomSheet<void>(
      context: scaffoldKey.currentContext!,
      routeSettings: routeSettings,
      builder: (BuildContext context) {
        retrievedRouteSettings = ModalRoute.settingsOf(context)!;
        return const Text('BottomSheet');
      },
    );

    await tester.pump();
    await tester.pump(const Duration(seconds: 1));

    expect(retrievedRouteSettings, routeSettings);
  });

  testWidgets('Verify showModalBottomSheet use AnimationController if provided.', (
    WidgetTester tester,
  ) async {
    const Key tapTarget = Key('tap-target');
    final AnimationController controller = AnimationController(
      vsync: const TestVSync(),
      duration: const Duration(seconds: 2),
      reverseDuration: const Duration(seconds: 2),
    );
    addTearDown(controller.dispose);

    await tester.pumpWidget(
      MaterialApp(
        home: Scaffold(
          body: Builder(
            builder: (BuildContext context) {
              return GestureDetector(
                key: tapTarget,
                onTap: () {
                  showModalBottomSheet<void>(
                    context: context,
                    // The default duration and reverseDuration is 1 second
                    transitionAnimationController: controller,
                    builder: (BuildContext context) {
                      return const Text('BottomSheet');
                    },
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

    expect(find.text('BottomSheet'), findsNothing);

    await tester.tap(find.byKey(tapTarget)); // Opening animation will start after tapping
    await tester.pump();

    expect(find.text('BottomSheet'), findsOneWidget);
    await tester.pump(const Duration(milliseconds: 2000));
    expect(find.text('BottomSheet'), findsOneWidget);

    // Tapping above the bottom sheet to dismiss it.
    await tester.tapAt(const Offset(20.0, 20.0)); // Closing animation will start after tapping
    await tester.pump();

    expect(find.text('BottomSheet'), findsOneWidget);
    await tester.pump(const Duration(milliseconds: 2000));
    // The bottom sheet should still be present at the very end of the animation.
    expect(find.text('BottomSheet'), findsOneWidget);

    await tester.pump(const Duration(milliseconds: 1));
    // The bottom sheet should not be showing any longer.
    expect(find.text('BottomSheet'), findsNothing);
  });

  // Regression test for https://github.com/flutter/flutter/issues/87592
  testWidgets('the framework do not dispose the transitionAnimationController provided by user.', (
    WidgetTester tester,
  ) async {
    const Key tapTarget = Key('tap-target');
    final AnimationController controller = AnimationController(
      vsync: const TestVSync(),
      duration: const Duration(seconds: 2),
      reverseDuration: const Duration(seconds: 2),
    );

    await tester.pumpWidget(
      MaterialApp(
        home: Scaffold(
          body: Builder(
            builder: (BuildContext context) {
              return GestureDetector(
                key: tapTarget,
                onTap: () {
                  showModalBottomSheet<void>(
                    context: context,
                    // The default duration and reverseDuration is 1 second
                    transitionAnimationController: controller,
                    builder: (BuildContext context) {
                      return const Text('BottomSheet');
                    },
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

    expect(find.text('BottomSheet'), findsNothing);

    await tester.tap(find.byKey(tapTarget)); // Opening animation will start after tapping
    await tester.pump();

    expect(find.text('BottomSheet'), findsOneWidget);
    await tester.pump(const Duration(milliseconds: 2000));
    expect(find.text('BottomSheet'), findsOneWidget);

    // Tapping above the bottom sheet to dismiss it.
    await tester.tapAt(const Offset(20.0, 20.0)); // Closing animation will start after tapping
    await tester.pump();

    expect(find.text('BottomSheet'), findsOneWidget);
    await tester.pump(const Duration(milliseconds: 2000));
    // The bottom sheet should still be present at the very end of the animation.
    expect(find.text('BottomSheet'), findsOneWidget);

    await tester.pump(const Duration(milliseconds: 1));
    // The bottom sheet should not be showing any longer.
    expect(find.text('BottomSheet'), findsNothing);

    controller.dispose();
    // Double disposal will throw.
    expect(tester.takeException(), isNull);
  });

  testWidgets('Verify persistence BottomSheet use AnimationController if provided.', (
    WidgetTester tester,
  ) async {
    const Key tapTarget = Key('tap-target');
    const Key tapTargetToClose = Key('tap-target-to-close');
    final AnimationController controller = AnimationController(
      vsync: const TestVSync(),
      duration: const Duration(seconds: 2),
      reverseDuration: const Duration(seconds: 2),
    );
    addTearDown(controller.dispose);

    await tester.pumpWidget(
      MaterialApp(
        home: Scaffold(
          body: Builder(
            builder: (BuildContext context) {
              return GestureDetector(
                key: tapTarget,
                onTap: () {
                  showBottomSheet(
                    context: context,
                    // The default duration and reverseDuration is 1 second
                    transitionAnimationController: controller,
                    builder: (BuildContext context) {
                      return ElevatedButton(
                        key: tapTargetToClose,
                        onPressed: () => Navigator.pop(context),
                        child: const Text('BottomSheet'),
                      );
                    },
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

    expect(find.text('BottomSheet'), findsNothing);

    await tester.tap(find.byKey(tapTarget)); // Opening animation will start after tapping
    await tester.pump();

    expect(find.text('BottomSheet'), findsOneWidget);
    await tester.pump(const Duration(milliseconds: 2000));
    expect(find.text('BottomSheet'), findsOneWidget);

    // Tapping button on the bottom sheet to dismiss it.
    await tester.tap(find.byKey(tapTargetToClose)); // Closing animation will start after tapping
    await tester.pump();

    expect(find.text('BottomSheet'), findsOneWidget);
    await tester.pump(const Duration(milliseconds: 2000));
    // The bottom sheet should still be present at the very end of the animation.
    expect(find.text('BottomSheet'), findsOneWidget);

    await tester.pump(const Duration(milliseconds: 1));
    // The bottom sheet should not be showing any longer.
    expect(find.text('BottomSheet'), findsNothing);
  });

  // Regression test for https://github.com/flutter/flutter/issues/87708
  testWidgets('Each of the internal animation controllers should be disposed by the framework.', (
    WidgetTester tester,
  ) async {
    final GlobalKey<ScaffoldState> scaffoldKey = GlobalKey();
    await tester.pumpWidget(
      MaterialApp(home: Scaffold(key: scaffoldKey, body: const Center(child: Text('body')))),
    );

    scaffoldKey.currentState!.showBottomSheet((_) {
      return Builder(
        builder: (BuildContext context) {
          return Container(height: 200.0);
        },
      );
    });

    await tester.pump();
    expect(find.byType(BottomSheet), findsOneWidget);

    // The first sheet's animation is still running.

    // Trigger the second sheet will remove the first sheet from tree.
    scaffoldKey.currentState!.showBottomSheet((_) {
      return Builder(
        builder: (BuildContext context) {
          return Container(height: 200.0);
        },
      );
    });
    await tester.pump();
    expect(find.byType(BottomSheet), findsOneWidget);

    // Remove the Scaffold from the tree.
    await tester.pumpWidget(const SizedBox.shrink());

    // If the internal animation controller do not dispose will throw
    // FlutterError:<ScaffoldState#1981a(tickers: tracking 1 ticker) was disposed with an active
    // Ticker.
    expect(tester.takeException(), isNull);
  });

  // Regression test for https://github.com/flutter/flutter/issues/99627
  testWidgets('The old route entry should be removed when a new sheet popup', (
    WidgetTester tester,
  ) async {
    final GlobalKey<ScaffoldState> scaffoldKey = GlobalKey();
    PersistentBottomSheetController? sheetController;

    await tester.pumpWidget(
      MaterialApp(home: Scaffold(key: scaffoldKey, body: const Center(child: Text('body')))),
    );

    final ModalRoute<dynamic> route = ModalRoute.of(scaffoldKey.currentContext!)!;
    expect(route.canPop, false);

    scaffoldKey.currentState!.showBottomSheet((_) {
      return Builder(
        builder: (BuildContext context) {
          return Container(height: 200.0);
        },
      );
    });

    await tester.pump();
    expect(find.byType(BottomSheet), findsOneWidget);
    expect(route.canPop, true);

    // Trigger the second sheet will remove the first sheet from tree.
    sheetController = scaffoldKey.currentState!.showBottomSheet((_) {
      return Builder(
        builder: (BuildContext context) {
          return Container(height: 200.0);
        },
      );
    });
    await tester.pump();
    expect(find.byType(BottomSheet), findsOneWidget);
    expect(route.canPop, true);

    sheetController.close();

    expect(route.canPop, false);
  });

  // Regression test for https://github.com/flutter/flutter/issues/87708
  testWidgets(
    'The framework does not dispose of the transitionAnimationController provided by user.',
    (WidgetTester tester) async {
      const Key tapTarget = Key('tap-target');
      const Key tapTargetToClose = Key('tap-target-to-close');
      final AnimationController controller = AnimationController(
        vsync: const TestVSync(),
        duration: const Duration(seconds: 2),
        reverseDuration: const Duration(seconds: 2),
      );
      await tester.pumpWidget(
        MaterialApp(
          home: Scaffold(
            body: Builder(
              builder: (BuildContext context) {
                return GestureDetector(
                  key: tapTarget,
                  onTap: () {
                    showBottomSheet(
                      context: context,
                      transitionAnimationController: controller,
                      builder: (BuildContext context) {
                        return ElevatedButton(
                          key: tapTargetToClose,
                          onPressed: () => Navigator.pop(context),
                          child: const Text('BottomSheet'),
                        );
                      },
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

      expect(find.text('BottomSheet'), findsNothing);

      await tester.tap(find.byKey(tapTarget)); // Open the sheet.
      await tester.pumpAndSettle(); // Finish the animation.
      expect(find.text('BottomSheet'), findsOneWidget);

      // Tapping button on the bottom sheet to dismiss it.
      await tester.tap(find.byKey(tapTargetToClose)); // Closing the sheet.
      await tester.pumpAndSettle(); // Finish the animation.
      expect(find.text('BottomSheet'), findsNothing);

      await tester.pumpWidget(const SizedBox.shrink());
      controller.dispose();

      // Double dispose will throw.
      expect(tester.takeException(), isNull);
    },
  );

  testWidgets(
    'Calling PersistentBottomSheetController.close does not crash when it is not the current bottom sheet',
    (WidgetTester tester) async {
      // Regression test for https://github.com/flutter/flutter/issues/93717
      PersistentBottomSheetController? sheetController1;
      await tester.pumpWidget(
        MaterialApp(
          home: Scaffold(
            body: Builder(
              builder: (BuildContext context) {
                return SafeArea(
                  child: Column(
                    children: <Widget>[
                      ElevatedButton(
                        child: const Text('show 1'),
                        onPressed: () {
                          sheetController1 = Scaffold.of(
                            context,
                          ).showBottomSheet((BuildContext context) => const Text('BottomSheet 1'));
                        },
                      ),
                      ElevatedButton(
                        child: const Text('show 2'),
                        onPressed: () {
                          Scaffold.of(
                            context,
                          ).showBottomSheet((BuildContext context) => const Text('BottomSheet 2'));
                        },
                      ),
                      ElevatedButton(
                        child: const Text('close 1'),
                        onPressed: () {
                          sheetController1!.close();
                        },
                      ),
                    ],
                  ),
                );
              },
            ),
          ),
        ),
      );

      await tester.tap(find.text('show 1'));
      await tester.pumpAndSettle();
      expect(find.text('BottomSheet 1'), findsOneWidget);

      await tester.tap(find.text('show 2'));
      await tester.pumpAndSettle();
      expect(find.text('BottomSheet 2'), findsOneWidget);

      // This will throw an assertion if regressed
      await tester.tap(find.text('close 1'));
      await tester.pumpAndSettle();
      expect(find.text('BottomSheet 2'), findsOneWidget);
    },
  );

  testWidgets('ModalBottomSheetRoute shows BottomSheet correctly', (WidgetTester tester) async {
    late BuildContext savedContext;

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
    expect(find.byType(BottomSheet), findsNothing);

    // Bring up bottom sheet.
    final NavigatorState navigator = Navigator.of(savedContext);
    navigator.push(
      ModalBottomSheetRoute<void>(
        isScrollControlled: false,
        builder: (BuildContext context) => Container(),
      ),
    );
    await tester.pumpAndSettle();
    expect(find.byType(BottomSheet), findsOneWidget);
  });

  group('Modal BottomSheet avoids overlapping display features', () {
    testWidgets('positioning using anchorPoint', (WidgetTester tester) async {
      await tester.pumpWidget(
        MaterialApp(
          builder: (BuildContext context, Widget? child) {
            return MediaQuery(
              // Display has a vertical hinge down the middle
              data: const MediaQueryData(
                size: Size(800, 600),
                displayFeatures: <DisplayFeature>[
                  DisplayFeature(
                    bounds: Rect.fromLTRB(390, 0, 410, 600),
                    type: DisplayFeatureType.hinge,
                    state: DisplayFeatureState.unknown,
                  ),
                ],
              ),
              child: child!,
            );
          },
          home: const Center(child: Text('Test')),
        ),
      );

      final BuildContext context = tester.element(find.text('Test'));
      showModalBottomSheet<void>(
        context: context,
        builder: (BuildContext context) {
          return const Placeholder();
        },
        anchorPoint: const Offset(1000, 0),
      );
      await tester.pumpAndSettle();

      // Should take the right side of the screen
      expect(tester.getTopLeft(find.byType(Placeholder)).dx, 410);
      expect(tester.getBottomRight(find.byType(Placeholder)).dx, 800);
    });

    testWidgets('positioning using Directionality', (WidgetTester tester) async {
      await tester.pumpWidget(
        MaterialApp(
          builder: (BuildContext context, Widget? child) {
            return MediaQuery(
              // Display has a vertical hinge down the middle
              data: const MediaQueryData(
                size: Size(800, 600),
                displayFeatures: <DisplayFeature>[
                  DisplayFeature(
                    bounds: Rect.fromLTRB(390, 0, 410, 600),
                    type: DisplayFeatureType.hinge,
                    state: DisplayFeatureState.unknown,
                  ),
                ],
              ),
              child: Directionality(textDirection: TextDirection.rtl, child: child!),
            );
          },
          home: const Center(child: Text('Test')),
        ),
      );

      final BuildContext context = tester.element(find.text('Test'));
      showModalBottomSheet<void>(
        context: context,
        builder: (BuildContext context) {
          return const Placeholder();
        },
      );
      await tester.pumpAndSettle();

      // This is RTL, so it should place the dialog on the right screen
      expect(tester.getTopLeft(find.byType(Placeholder)).dx, 410);
      expect(tester.getBottomRight(find.byType(Placeholder)).dx, 800);
    });

    testWidgets('default positioning', (WidgetTester tester) async {
      await tester.pumpWidget(
        MaterialApp(
          builder: (BuildContext context, Widget? child) {
            return MediaQuery(
              // Display has a vertical hinge down the middle
              data: const MediaQueryData(
                size: Size(800, 600),
                displayFeatures: <DisplayFeature>[
                  DisplayFeature(
                    bounds: Rect.fromLTRB(390, 0, 410, 600),
                    type: DisplayFeatureType.hinge,
                    state: DisplayFeatureState.unknown,
                  ),
                ],
              ),
              child: child!,
            );
          },
          home: const Center(child: Text('Test')),
        ),
      );

      final BuildContext context = tester.element(find.text('Test'));
      showModalBottomSheet<void>(
        context: context,
        builder: (BuildContext context) {
          return const Placeholder();
        },
      );
      await tester.pumpAndSettle();

      // By default it should place the dialog on the left screen
      expect(tester.getTopLeft(find.byType(Placeholder)).dx, 0.0);
      expect(tester.getBottomRight(find.byType(Placeholder)).dx, 390.0);
    });
  });

  group('constraints', () {
    testWidgets('Material3 - Default constraints are max width 640', (WidgetTester tester) async {
      await tester.pumpWidget(
        MaterialApp(
          theme: ThemeData(useMaterial3: true),
          home: const MediaQuery(
            data: MediaQueryData(size: Size(1000, 1000)),
            child: Scaffold(
              body: Center(child: Text('body')),
              bottomSheet: Placeholder(fallbackWidth: 800),
            ),
          ),
        ),
      );
      expect(tester.getSize(find.byType(Placeholder)).width, 640);
    });

    testWidgets('Material2 - No constraints by default for bottomSheet property', (
      WidgetTester tester,
    ) async {
      await tester.pumpWidget(
        MaterialApp(
          // This test is specific to Material2 because Material3 sets constraints by default for BottomSheet.
          theme: ThemeData(useMaterial3: false),
          home: const Scaffold(body: Center(child: Text('body')), bottomSheet: Text('BottomSheet')),
        ),
      );
      expect(find.text('BottomSheet'), findsOneWidget);
      expect(tester.getRect(find.text('BottomSheet')), const Rect.fromLTRB(0, 586, 154, 600));
    });

    testWidgets('No constraints by default for showBottomSheet', (WidgetTester tester) async {
      await tester.pumpWidget(
        MaterialApp(
          // This test is specific to Material2 because Material3 sets constraints by default for BottomSheet.
          theme: ThemeData(useMaterial3: false),
          home: Scaffold(
            body: Builder(
              builder: (BuildContext context) {
                return Center(
                  child: ElevatedButton(
                    child: const Text('Press me'),
                    onPressed: () {
                      Scaffold.of(
                        context,
                      ).showBottomSheet((BuildContext context) => const Text('BottomSheet'));
                    },
                  ),
                );
              },
            ),
          ),
        ),
      );
      expect(find.text('BottomSheet'), findsNothing);
      await tester.tap(find.text('Press me'));
      await tester.pumpAndSettle();
      expect(find.text('BottomSheet'), findsOneWidget);
      expect(tester.getRect(find.text('BottomSheet')), const Rect.fromLTRB(0, 586, 154, 600));
    });

    testWidgets('No constraints by default for showModalBottomSheet', (WidgetTester tester) async {
      await tester.pumpWidget(
        MaterialApp(
          // This test is specific to Material2 because Material3 sets constraints by default for BottomSheet.
          theme: ThemeData(useMaterial3: false),
          home: Scaffold(
            body: Builder(
              builder: (BuildContext context) {
                return Center(
                  child: ElevatedButton(
                    child: const Text('Press me'),
                    onPressed: () {
                      showModalBottomSheet<void>(
                        context: context,
                        builder: (BuildContext context) => const Text('BottomSheet'),
                      );
                    },
                  ),
                );
              },
            ),
          ),
        ),
      );
      expect(find.text('BottomSheet'), findsNothing);
      await tester.tap(find.text('Press me'));
      await tester.pumpAndSettle();
      expect(find.text('BottomSheet'), findsOneWidget);
      expect(tester.getRect(find.text('BottomSheet')), const Rect.fromLTRB(0, 586, 800, 600));
    });

    testWidgets('Material3 - Theme constraints used for bottomSheet property', (
      WidgetTester tester,
    ) async {
      const double sheetMaxWidth = 80.0;
      await tester.pumpWidget(
        MaterialApp(
          theme: ThemeData(
            useMaterial3: true,
            bottomSheetTheme: const BottomSheetThemeData(
              constraints: BoxConstraints(maxWidth: sheetMaxWidth),
            ),
          ),
          home: Scaffold(
            body: const Center(child: Text('body')),
            bottomSheet: const Text('BottomSheet'),
            floatingActionButton: FloatingActionButton(
              onPressed: () {},
              child: const Icon(Icons.add),
            ),
          ),
        ),
      );
      expect(find.text('BottomSheet'), findsOneWidget);

      // Should be centered and only 80dp wide.
      final Rect bottomSheetRect = tester.getRect(find.text('BottomSheet'));
      expect(bottomSheetRect.left, 800 / 2 - sheetMaxWidth / 2);
      expect(bottomSheetRect.width, sheetMaxWidth);

      // Ensure the FAB is overlapping the top of the sheet.
      expect(find.byIcon(Icons.add), findsOneWidget);
      final Rect iconRect = tester.getRect(find.byIcon(Icons.add));
      expect(iconRect.top, bottomSheetRect.top - iconRect.height / 2);
    });

    testWidgets('Material2 - Theme constraints used for bottomSheet property', (
      WidgetTester tester,
    ) async {
      await tester.pumpWidget(
        MaterialApp(
          theme: ThemeData(
            useMaterial3: false,
            bottomSheetTheme: const BottomSheetThemeData(constraints: BoxConstraints(maxWidth: 80)),
          ),
          home: Scaffold(
            body: const Center(child: Text('body')),
            bottomSheet: const Text('BottomSheet'),
            floatingActionButton: FloatingActionButton(
              onPressed: () {},
              child: const Icon(Icons.add),
            ),
          ),
        ),
      );
      expect(find.text('BottomSheet'), findsOneWidget);
      // Should be centered and only 80dp wide
      expect(tester.getRect(find.text('BottomSheet')), const Rect.fromLTRB(360, 558, 440, 600));
      // Ensure the FAB is overlapping the top of the sheet
      expect(find.byIcon(Icons.add), findsOneWidget);
      expect(tester.getRect(find.byIcon(Icons.add)), const Rect.fromLTRB(744, 544, 768, 568));
    });

    testWidgets('Theme constraints used for showBottomSheet', (WidgetTester tester) async {
      const double sheetMaxWidth = 80.0;
      await tester.pumpWidget(
        MaterialApp(
          theme: ThemeData(
            bottomSheetTheme: const BottomSheetThemeData(
              constraints: BoxConstraints(maxWidth: sheetMaxWidth),
            ),
          ),
          home: Scaffold(
            body: Builder(
              builder: (BuildContext context) {
                return Center(
                  child: ElevatedButton(
                    child: const Text('Press me'),
                    onPressed: () {
                      Scaffold.of(
                        context,
                      ).showBottomSheet((BuildContext context) => const Text('BottomSheet'));
                    },
                  ),
                );
              },
            ),
          ),
        ),
      );
      expect(find.text('BottomSheet'), findsNothing);
      await tester.tap(find.text('Press me'));
      await tester.pumpAndSettle();
      expect(find.text('BottomSheet'), findsOneWidget);

      // Should be centered and only 80dp wide.
      final Rect bottomSheetRect = tester.getRect(find.text('BottomSheet'));
      expect(bottomSheetRect.left, 800 / 2 - sheetMaxWidth / 2);
      expect(bottomSheetRect.width, sheetMaxWidth);
    });

    testWidgets('Theme constraints used for showModalBottomSheet', (WidgetTester tester) async {
      const double sheetMaxWidth = 80.0;
      await tester.pumpWidget(
        MaterialApp(
          theme: ThemeData(
            bottomSheetTheme: const BottomSheetThemeData(
              constraints: BoxConstraints(maxWidth: sheetMaxWidth),
            ),
          ),
          home: Scaffold(
            body: Builder(
              builder: (BuildContext context) {
                return Center(
                  child: ElevatedButton(
                    child: const Text('Press me'),
                    onPressed: () {
                      showModalBottomSheet<void>(
                        context: context,
                        builder: (BuildContext context) => const Text('BottomSheet'),
                      );
                    },
                  ),
                );
              },
            ),
          ),
        ),
      );
      expect(find.text('BottomSheet'), findsNothing);
      await tester.tap(find.text('Press me'));
      await tester.pumpAndSettle();
      expect(find.text('BottomSheet'), findsOneWidget);

      // Should be centered and only 80dp wide.
      final Rect bottomSheetRect = tester.getRect(find.text('BottomSheet'));
      expect(bottomSheetRect.left, 800 / 2 - sheetMaxWidth / 2);
      expect(bottomSheetRect.width, sheetMaxWidth);
    });

    testWidgets('constraints param overrides theme for showBottomSheet', (
      WidgetTester tester,
    ) async {
      const double sheetMaxWidth = 100.0;
      await tester.pumpWidget(
        MaterialApp(
          theme: ThemeData(
            bottomSheetTheme: const BottomSheetThemeData(constraints: BoxConstraints(maxWidth: 80)),
          ),
          home: Scaffold(
            body: Builder(
              builder: (BuildContext context) {
                return Center(
                  child: ElevatedButton(
                    child: const Text('Press me'),
                    onPressed: () {
                      Scaffold.of(context).showBottomSheet(
                        (BuildContext context) => const Text('BottomSheet'),
                        constraints: const BoxConstraints(maxWidth: sheetMaxWidth),
                      );
                    },
                  ),
                );
              },
            ),
          ),
        ),
      );
      expect(find.text('BottomSheet'), findsNothing);
      await tester.tap(find.text('Press me'));
      await tester.pumpAndSettle();
      expect(find.text('BottomSheet'), findsOneWidget);

      // Should be centered and only 80dp wide.
      final Rect bottomSheetRect = tester.getRect(find.text('BottomSheet'));
      expect(bottomSheetRect.left, 800 / 2 - sheetMaxWidth / 2);
      expect(bottomSheetRect.width, sheetMaxWidth);
    });

    testWidgets('constraints param overrides theme for showModalBottomSheet', (
      WidgetTester tester,
    ) async {
      const double sheetMaxWidth = 100.0;
      await tester.pumpWidget(
        MaterialApp(
          theme: ThemeData(
            bottomSheetTheme: const BottomSheetThemeData(constraints: BoxConstraints(maxWidth: 80)),
          ),
          home: Scaffold(
            body: Builder(
              builder: (BuildContext context) {
                return Center(
                  child: ElevatedButton(
                    child: const Text('Press me'),
                    onPressed: () {
                      showModalBottomSheet<void>(
                        context: context,
                        builder: (BuildContext context) => const Text('BottomSheet'),
                        constraints: const BoxConstraints(maxWidth: sheetMaxWidth),
                      );
                    },
                  ),
                );
              },
            ),
          ),
        ),
      );
      expect(find.text('BottomSheet'), findsNothing);
      await tester.tap(find.text('Press me'));
      await tester.pumpAndSettle();
      expect(find.text('BottomSheet'), findsOneWidget);

      // Should be centered and only 80dp wide.
      final Rect bottomSheetRect = tester.getRect(find.text('BottomSheet'));
      expect(bottomSheetRect.left, 800 / 2 - sheetMaxWidth / 2);
      expect(bottomSheetRect.width, sheetMaxWidth);
    });

    group('scrollControlDisabledMaxHeightRatio', () {
      Future<void> test(
        WidgetTester tester,
        bool isScrollControlled,
        double scrollControlDisabledMaxHeightRatio,
      ) async {
        await tester.pumpWidget(
          MaterialApp(
            home: Scaffold(
              body: Builder(
                builder: (BuildContext context) {
                  return Center(
                    child: ElevatedButton(
                      child: const Text('Press me'),
                      onPressed: () {
                        showModalBottomSheet<void>(
                          context: context,
                          isScrollControlled: isScrollControlled,
                          scrollControlDisabledMaxHeightRatio: scrollControlDisabledMaxHeightRatio,
                          builder:
                              (BuildContext context) =>
                                  const SizedBox.expand(child: Text('BottomSheet')),
                        );
                      },
                    ),
                  );
                },
              ),
            ),
          ),
        );
        await tester.tap(find.text('Press me'));
        await tester.pumpAndSettle();
        expect(
          tester.getRect(find.text('BottomSheet')),
          Rect.fromLTRB(
            80,
            600 * (isScrollControlled ? 0 : (1 - scrollControlDisabledMaxHeightRatio)),
            720,
            600,
          ),
        );
      }

      testWidgets('works at 9 / 16', (WidgetTester tester) {
        return test(tester, false, 9.0 / 16.0);
      });
      testWidgets('works at 8 / 16', (WidgetTester tester) {
        return test(tester, false, 8.0 / 16.0);
      });
      testWidgets('works at isScrollControlled', (WidgetTester tester) {
        return test(tester, true, 8.0 / 16.0);
      });
    });
  });

  group('showModalBottomSheet modalBarrierDismissLabel', () {
    testWidgets('Verify that modalBarrierDismissLabel is used if provided', (
      WidgetTester tester,
    ) async {
      final GlobalKey<ScaffoldState> scaffoldKey = GlobalKey<ScaffoldState>();
      const String customLabel = 'custom label';
      await tester.pumpWidget(
        MaterialApp(home: Scaffold(key: scaffoldKey, body: const Center(child: Text('body')))),
      );

      showModalBottomSheet<void>(
        barrierLabel: 'custom label',
        context: scaffoldKey.currentContext!,
        builder: (BuildContext context) {
          return const Text('BottomSheet');
        },
      );
      await tester.pump();
      await tester.pump(const Duration(seconds: 1));

      final ModalBarrier modalBarrier = tester.widget(find.byType(ModalBarrier).last);
      expect(modalBarrier.semanticsLabel, customLabel);
    });

    testWidgets(
      'Verify that modalBarrierDismissLabel from context is used if barrierLabel is not provided',
      (WidgetTester tester) async {
        final GlobalKey<ScaffoldState> scaffoldKey = GlobalKey<ScaffoldState>();
        await tester.pumpWidget(
          MaterialApp(home: Scaffold(key: scaffoldKey, body: const Center(child: Text('body')))),
        );

        showModalBottomSheet<void>(
          context: scaffoldKey.currentContext!,
          builder: (BuildContext context) {
            return const Text('BottomSheet');
          },
        );
        await tester.pump();
        await tester.pump(const Duration(seconds: 1));

        final ModalBarrier modalBarrier = tester.widget(find.byType(ModalBarrier).last);
        expect(
          modalBarrier.semanticsLabel,
          MaterialLocalizations.of(scaffoldKey.currentContext!).scrimLabel,
        );
      },
    );
  });

  testWidgets('Bottom sheet animation can be customized', (WidgetTester tester) async {
    final Key sheetKey = UniqueKey();

    Widget buildWidget({AnimationStyle? sheetAnimationStyle}) {
      return MaterialApp(
        home: Scaffold(
          body: Builder(
            builder: (BuildContext context) {
              return GestureDetector(
                onTap: () {
                  showBottomSheet(
                    context: context,
                    sheetAnimationStyle: sheetAnimationStyle,
                    builder: (BuildContext context) {
                      return SizedBox.expand(
                        child: ColoredBox(
                          key: sheetKey,
                          color: Theme.of(context).colorScheme.primary,
                          child: FilledButton(
                            onPressed: () {
                              Navigator.pop(context);
                            },
                            child: const Text('Close'),
                          ),
                        ),
                      );
                    },
                  );
                },
                child: const Text('X'),
              );
            },
          ),
        ),
      );
    }

    // Test custom animation style.
    await tester.pumpWidget(
      buildWidget(
        sheetAnimationStyle: AnimationStyle(
          duration: const Duration(milliseconds: 800),
          reverseDuration: const Duration(milliseconds: 400),
        ),
      ),
    );
    await tester.tap(find.text('X'));
    await tester.pump();
    // Advance the animation by 1/2 of the custom forward duration.
    await tester.pump(const Duration(milliseconds: 400));

    // The bottom sheet is partially visible.
    expect(tester.getTopLeft(find.byKey(sheetKey)).dy, closeTo(134.6, 0.1));

    // Advance the animation by 1/2 of the custom forward duration.
    await tester.pump(const Duration(milliseconds: 400));

    // The bottom sheet is fully visible.
    expect(tester.getTopLeft(find.byKey(sheetKey)).dy, equals(0.0));

    // Dismiss the bottom sheet.
    await tester.tap(find.widgetWithText(FilledButton, 'Close'));
    await tester.pump();
    // Advance the animation by 1/2 of the custom reverse duration.
    await tester.pump(const Duration(milliseconds: 200));

    // The bottom sheet is partially visible.
    expect(tester.getTopLeft(find.byKey(sheetKey)).dy, closeTo(134.6, 0.1));

    // Advance the animation by 1/2 of the custom reverse duration.
    await tester.pump(const Duration(milliseconds: 200));

    // The bottom sheet is dismissed.
    expect(tester.getTopLeft(find.byKey(sheetKey)).dy, equals(600.0));

    // Test no animation style.
    await tester.pumpWidget(buildWidget(sheetAnimationStyle: AnimationStyle.noAnimation));
    await tester.pumpAndSettle();
    await tester.tap(find.text('X'));
    await tester.pump();

    // The bottom sheet is fully visible.
    expect(tester.getTopLeft(find.byKey(sheetKey)).dy, equals(0.0));

    // Dismiss the bottom sheet.
    await tester.tap(find.widgetWithText(FilledButton, 'Close'));
    await tester.pump();

    // The bottom sheet is dismissed.
    expect(find.byKey(sheetKey), findsNothing);
  });

  testWidgets('Modal bottom sheet default animation', (WidgetTester tester) async {
    final Key sheetKey = UniqueKey();

    // Test default modal bottom sheet animation.
    await tester.pumpWidget(
      MaterialApp(
        home: Scaffold(
          body: Builder(
            builder: (BuildContext context) {
              return GestureDetector(
                onTap: () {
                  showModalBottomSheet<void>(
                    context: context,
                    builder: (BuildContext context) {
                      return SizedBox.expand(
                        child: ColoredBox(
                          key: sheetKey,
                          color: Theme.of(context).colorScheme.primary,
                          child: FilledButton(
                            onPressed: () {
                              Navigator.pop(context);
                            },
                            child: const Text('Close'),
                          ),
                        ),
                      );
                    },
                  );
                },
                child: const Text('X'),
              );
            },
          ),
        ),
      ),
    );

    // Tap the 'X' to show the bottom sheet.
    await tester.tap(find.text('X'));
    await tester.pump();
    // Advance the animation by 1/2 of the default forward duration.
    await tester.pump(const Duration(milliseconds: 125));

    // The modal bottom sheet is partially visible.
    expect(tester.getTopLeft(find.byKey(sheetKey)).dy, closeTo(316.7, 0.1));

    // Advance the animation by 1/2 of the default forward duration.
    await tester.pump(const Duration(milliseconds: 125));

    // The modal bottom sheet is fully visible.
    expect(tester.getTopLeft(find.byKey(sheetKey)).dy, equals(262.5));

    // Dismiss the bottom sheet.
    await tester.tap(find.widgetWithText(FilledButton, 'Close'));
    await tester.pump();
    // Advance the animation by 1/2 of the default reverse duration.
    await tester.pump(const Duration(milliseconds: 100));

    // The modal bottom sheet is partially visible.
    expect(tester.getTopLeft(find.byKey(sheetKey)).dy, closeTo(316.7, 0.1));

    // Advance the animation by 1/2 of the default reverse duration.
    await tester.pump(const Duration(milliseconds: 100));

    // The modal bottom sheet is dismissed.
    expect(tester.getTopLeft(find.byKey(sheetKey)).dy, equals(600.0));
  });

  testWidgets('Modal bottom sheet animation can be customized', (WidgetTester tester) async {
    final Key sheetKey = UniqueKey();

    Widget buildWidget({AnimationStyle? sheetAnimationStyle}) {
      return MaterialApp(
        home: Scaffold(
          body: Builder(
            builder: (BuildContext context) {
              return GestureDetector(
                onTap: () {
                  showModalBottomSheet<void>(
                    context: context,
                    sheetAnimationStyle: sheetAnimationStyle,
                    builder: (BuildContext context) {
                      return SizedBox.expand(
                        child: ColoredBox(
                          key: sheetKey,
                          color: Theme.of(context).colorScheme.primary,
                          child: FilledButton(
                            onPressed: () {
                              Navigator.pop(context);
                            },
                            child: const Text('Close'),
                          ),
                        ),
                      );
                    },
                  );
                },
                child: const Text('X'),
              );
            },
          ),
        ),
      );
    }

    // Test custom animation style.
    await tester.pumpWidget(
      buildWidget(
        sheetAnimationStyle: AnimationStyle(
          duration: const Duration(milliseconds: 800),
          reverseDuration: const Duration(milliseconds: 400),
        ),
      ),
    );
    await tester.tap(find.text('X'));
    await tester.pump();
    // Advance the animation by 1/2 of the custom forward duration.
    await tester.pump(const Duration(milliseconds: 400));

    // The bottom sheet is partially visible.
    expect(tester.getTopLeft(find.byKey(sheetKey)).dy, closeTo(316.7, 0.1));

    // Advance the animation by 1/2 of the custom forward duration.
    await tester.pump(const Duration(milliseconds: 400));

    // The bottom sheet is fully visible.
    expect(tester.getTopLeft(find.byKey(sheetKey)).dy, equals(262.5));

    // Dismiss the bottom sheet.
    await tester.tap(find.widgetWithText(FilledButton, 'Close'));
    await tester.pump();
    // Advance the animation by 1/2 of the custom reverse duration.
    await tester.pump(const Duration(milliseconds: 200));

    // The bottom sheet is partially visible.
    expect(tester.getTopLeft(find.byKey(sheetKey)).dy, closeTo(316.7, 0.1));

    // Advance the animation by 1/2 of the custom reverse duration.
    await tester.pump(const Duration(milliseconds: 200));

    // The bottom sheet is dismissed.
    expect(tester.getTopLeft(find.byKey(sheetKey)).dy, equals(600.0));

    // Test no animation style.
    await tester.pumpWidget(buildWidget(sheetAnimationStyle: AnimationStyle.noAnimation));
    await tester.pumpAndSettle();
    await tester.tap(find.text('X'));
    await tester.pump();

    // The bottom sheet is fully visible.
    expect(tester.getTopLeft(find.byKey(sheetKey)).dy, equals(262.5));

    // Dismiss the bottom sheet.
    await tester.tap(find.widgetWithText(FilledButton, 'Close'));
    await tester.pump();

    // The bottom sheet is dismissed.
    expect(find.byKey(sheetKey), findsNothing);
  });

  testWidgets(
    'Setting ModalBottomSheetRoute.requestFocus to false does not request focus on the bottom sheet',
    (WidgetTester tester) async {
      late BuildContext savedContext;
      final FocusNode focusNode = FocusNode();
      addTearDown(focusNode.dispose);
      await tester.pumpWidget(
        MaterialApp(
          home: Material(
            child: Builder(
              builder: (BuildContext context) {
                savedContext = context;
                return TextField(focusNode: focusNode);
              },
            ),
          ),
        ),
      );
      await tester.pump();

      FocusNode? getTextFieldFocusNode() {
        return tester
            .widget<Focus>(
              find.descendant(of: find.byType(TextField), matching: find.byType(Focus)),
            )
            .focusNode;
      }

      // Initially, there is no bottom sheet and the text field has no focus.
      expect(find.byType(BottomSheet), findsNothing);
      expect(getTextFieldFocusNode()?.hasFocus, false);

      // Request focus on the text field.
      focusNode.requestFocus();
      await tester.pump();
      expect(getTextFieldFocusNode()?.hasFocus, true);

      // Bring up bottom sheet.
      final NavigatorState navigator = Navigator.of(savedContext);
      navigator.push(
        ModalBottomSheetRoute<void>(
          isScrollControlled: false,
          builder: (BuildContext context) => Container(),
        ),
      );
      await tester.pump();

      // The bottom sheet is showing and the text field has lost focus.
      expect(find.byType(BottomSheet), findsOneWidget);
      expect(getTextFieldFocusNode()?.hasFocus, false);

      // Dismiss the bottom sheet.
      navigator.pop();
      await tester.pump();

      // The bottom sheet is dismissed and the focus is shifted back to the text field.
      expect(find.byType(BottomSheet), findsNothing);
      expect(getTextFieldFocusNode()?.hasFocus, true);

      // Bring up bottom sheet again with requestFocus to false.
      navigator.push(
        ModalBottomSheetRoute<void>(
          requestFocus: false,
          isScrollControlled: false,
          builder: (BuildContext context) => Container(),
        ),
      );
      await tester.pump();

      // The bottom sheet is showing and the text field still has focus.
      expect(find.byType(BottomSheet), findsOneWidget);
      expect(getTextFieldFocusNode()?.hasFocus, true);
    },
  );
}

class _TestPage extends StatelessWidget {
  const _TestPage({this.useRootNavigator});

  final bool? useRootNavigator;

  @override
  Widget build(BuildContext context) {
    return Center(
      child: TextButton(
        child: const Text('Show bottom sheet'),
        onPressed: () {
          if (useRootNavigator != null) {
            showModalBottomSheet<void>(
              useRootNavigator: useRootNavigator!,
              context: context,
              builder: (_) => const Text('Modal bottom sheet'),
            );
          } else {
            showModalBottomSheet<void>(
              context: context,
              builder: (_) => const Text('Modal bottom sheet'),
            );
          }
        },
      ),
    );
  }
}
