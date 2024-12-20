// Copyright 2014 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

import 'package:flutter/foundation.dart';
import 'package:flutter/material.dart';
import 'package:flutter/services.dart';
import 'package:flutter_test/flutter_test.dart';

import 'navigator_utils.dart';

void main() {
  bool? lastFrameworkHandlesBack;
  setUp(() async {
    lastFrameworkHandlesBack = null;
    TestDefaultBinaryMessengerBinding.instance.defaultBinaryMessenger.setMockMethodCallHandler(
      SystemChannels.platform,
      (MethodCall methodCall) async {
        if (methodCall.method == 'SystemNavigator.setFrameworkHandlesBack') {
          expect(methodCall.arguments, isA<bool>());
          lastFrameworkHandlesBack = methodCall.arguments as bool;
        }
        return;
      },
    );
    await TestDefaultBinaryMessengerBinding.instance.defaultBinaryMessenger.handlePlatformMessage(
      'flutter/lifecycle',
      const StringCodec().encodeMessage(AppLifecycleState.resumed.toString()),
      (ByteData? data) {},
    );
  });

  tearDown(() {
    TestDefaultBinaryMessengerBinding.instance.defaultBinaryMessenger.setMockMethodCallHandler(
      SystemChannels.platform,
      null,
    );
  });

  testWidgets('toggling canPop on root route allows/prevents backs', (WidgetTester tester) async {
    bool canPop = false;
    late StateSetter setState;
    late BuildContext context;
    await tester.pumpWidget(
      MaterialApp(
        initialRoute: '/',
        routes: <String, WidgetBuilder>{
          '/':
              (BuildContext buildContext) => Scaffold(
                body: StatefulBuilder(
                  builder: (BuildContext buildContext, StateSetter stateSetter) {
                    context = buildContext;
                    setState = stateSetter;
                    return PopScope<Object?>(
                      canPop: canPop,
                      child: const Center(
                        child: Column(
                          mainAxisAlignment: MainAxisAlignment.center,
                          children: <Widget>[Text('Home/PopScope Page')],
                        ),
                      ),
                    );
                  },
                ),
              ),
        },
      ),
    );

    expect(ModalRoute.of(context)!.popDisposition, RoutePopDisposition.doNotPop);

    setState(() {
      canPop = true;
    });
    await tester.pump();
    if (!kIsWeb && defaultTargetPlatform == TargetPlatform.android) {
      expect(lastFrameworkHandlesBack, isFalse);
    }
    expect(ModalRoute.of(context)!.popDisposition, RoutePopDisposition.bubble);
  }, variant: TargetPlatformVariant.all());

  testWidgets('pop scope can receive result', (WidgetTester tester) async {
    Object? receivedResult;
    final Object poppedResult = Object();
    final GlobalKey<NavigatorState> nav = GlobalKey<NavigatorState>();
    await tester.pumpWidget(
      MaterialApp(
        initialRoute: '/',
        navigatorKey: nav,
        home: Scaffold(
          body: PopScope<Object?>(
            canPop: false,
            onPopInvokedWithResult: (bool didPop, Object? result) {
              receivedResult = result;
            },
            child: const Center(
              child: Column(
                mainAxisAlignment: MainAxisAlignment.center,
                children: <Widget>[Text('Home/PopScope Page')],
              ),
            ),
          ),
        ),
      ),
    );

    nav.currentState!.maybePop(poppedResult);
    await tester.pumpAndSettle();
    expect(receivedResult, poppedResult);
  }, variant: TargetPlatformVariant.all());

  testWidgets(
    'pop scope can have Object? generic type while route has stricter generic type',
    (WidgetTester tester) async {
      Object? receivedResult;
      const int poppedResult = 13;
      final GlobalKey<NavigatorState> nav = GlobalKey<NavigatorState>();
      await tester.pumpWidget(
        MaterialApp(
          initialRoute: '/',
          navigatorKey: nav,
          home: Scaffold(
            body: PopScope<Object?>(
              canPop: false,
              onPopInvokedWithResult: (bool didPop, Object? result) {
                receivedResult = result;
              },
              child: const Center(
                child: Column(
                  mainAxisAlignment: MainAxisAlignment.center,
                  children: <Widget>[Text('Home/PopScope Page')],
                ),
              ),
            ),
          ),
        ),
      );

      nav.currentState!.push(
        MaterialPageRoute<int>(
          builder: (BuildContext context) {
            return Scaffold(
              body: PopScope<Object?>(
                canPop: false,
                onPopInvokedWithResult: (bool didPop, Object? result) {
                  receivedResult = result;
                },
                child: const Center(child: Text('new page')),
              ),
            );
          },
        ),
      );
      await tester.pumpAndSettle();
      expect(find.text('new page'), findsOneWidget);

      nav.currentState!.maybePop(poppedResult);
      await tester.pumpAndSettle();
      expect(receivedResult, poppedResult);
    },
    variant: TargetPlatformVariant.all(),
  );

  testWidgets('toggling canPop on secondary route allows/prevents backs', (
    WidgetTester tester,
  ) async {
    final GlobalKey<NavigatorState> nav = GlobalKey<NavigatorState>();
    bool canPop = true;
    late StateSetter setState;
    late BuildContext homeContext;
    late BuildContext oneContext;
    late bool lastPopSuccess;
    await tester.pumpWidget(
      MaterialApp(
        navigatorKey: nav,
        initialRoute: '/',
        routes: <String, WidgetBuilder>{
          '/': (BuildContext context) {
            homeContext = context;
            return Scaffold(
              body: Center(
                child: Column(
                  mainAxisAlignment: MainAxisAlignment.center,
                  children: <Widget>[
                    const Text('Home Page'),
                    TextButton(
                      onPressed: () {
                        Navigator.of(context).pushNamed('/one');
                      },
                      child: const Text('Next'),
                    ),
                  ],
                ),
              ),
            );
          },
          '/one':
              (BuildContext context) => Scaffold(
                body: StatefulBuilder(
                  builder: (BuildContext context, StateSetter stateSetter) {
                    oneContext = context;
                    setState = stateSetter;
                    return PopScope<Object?>(
                      canPop: canPop,
                      onPopInvokedWithResult: (bool didPop, Object? result) {
                        lastPopSuccess = didPop;
                      },
                      child: const Center(
                        child: Column(
                          mainAxisAlignment: MainAxisAlignment.center,
                          children: <Widget>[Text('PopScope Page')],
                        ),
                      ),
                    );
                  },
                ),
              ),
        },
      ),
    );

    expect(find.text('Home Page'), findsOneWidget);
    expect(ModalRoute.of(homeContext)!.popDisposition, RoutePopDisposition.bubble);

    await tester.tap(find.text('Next'));
    await tester.pumpAndSettle();
    expect(find.text('PopScope Page'), findsOneWidget);
    expect(find.text('Home Page'), findsNothing);
    expect(ModalRoute.of(oneContext)!.popDisposition, RoutePopDisposition.pop);
    if (!kIsWeb && defaultTargetPlatform == TargetPlatform.android) {
      expect(lastFrameworkHandlesBack, isTrue);
    }

    // When canPop is true, can use pop to go back.
    nav.currentState!.maybePop();
    await tester.pumpAndSettle();
    expect(lastPopSuccess, true);
    expect(find.text('Home Page'), findsOneWidget);
    expect(find.text('PopScope Page'), findsNothing);
    expect(ModalRoute.of(homeContext)!.popDisposition, RoutePopDisposition.bubble);
    if (!kIsWeb && defaultTargetPlatform == TargetPlatform.android) {
      expect(lastFrameworkHandlesBack, isFalse);
    }

    await tester.tap(find.text('Next'));
    await tester.pumpAndSettle();
    expect(find.text('PopScope Page'), findsOneWidget);
    expect(find.text('Home Page'), findsNothing);
    expect(ModalRoute.of(oneContext)!.popDisposition, RoutePopDisposition.pop);
    if (!kIsWeb && defaultTargetPlatform == TargetPlatform.android) {
      expect(lastFrameworkHandlesBack, isTrue);
    }

    // When canPop is true, can use system back to go back.
    await simulateSystemBack();
    await tester.pumpAndSettle();
    expect(lastPopSuccess, true);
    expect(find.text('Home Page'), findsOneWidget);
    expect(find.text('PopScope Page'), findsNothing);
    expect(ModalRoute.of(homeContext)!.popDisposition, RoutePopDisposition.bubble);
    if (!kIsWeb && defaultTargetPlatform == TargetPlatform.android) {
      expect(lastFrameworkHandlesBack, isFalse);
    }

    await tester.tap(find.text('Next'));
    await tester.pumpAndSettle();
    expect(find.text('PopScope Page'), findsOneWidget);
    expect(find.text('Home Page'), findsNothing);
    expect(ModalRoute.of(oneContext)!.popDisposition, RoutePopDisposition.pop);
    if (!kIsWeb && defaultTargetPlatform == TargetPlatform.android) {
      expect(lastFrameworkHandlesBack, isTrue);
    }

    setState(() {
      canPop = false;
    });
    await tester.pump();

    // When canPop is false, can't use pop to go back.
    nav.currentState!.maybePop();
    await tester.pumpAndSettle();
    expect(lastPopSuccess, false);
    expect(find.text('PopScope Page'), findsOneWidget);
    expect(find.text('Home Page'), findsNothing);
    expect(ModalRoute.of(oneContext)!.popDisposition, RoutePopDisposition.doNotPop);

    // When canPop is false, can't use system back to go back.
    await simulateSystemBack();
    await tester.pumpAndSettle();
    expect(lastPopSuccess, false);
    expect(find.text('PopScope Page'), findsOneWidget);
    expect(find.text('Home Page'), findsNothing);
    expect(ModalRoute.of(oneContext)!.popDisposition, RoutePopDisposition.doNotPop);

    // Toggle canPop back to true and back works again.
    setState(() {
      canPop = true;
    });
    await tester.pump();

    nav.currentState!.maybePop();
    await tester.pumpAndSettle();
    expect(lastPopSuccess, true);
    expect(find.text('Home Page'), findsOneWidget);
    expect(find.text('PopScope Page'), findsNothing);
    expect(ModalRoute.of(homeContext)!.popDisposition, RoutePopDisposition.bubble);
    if (!kIsWeb && defaultTargetPlatform == TargetPlatform.android) {
      expect(lastFrameworkHandlesBack, isFalse);
    }

    await tester.tap(find.text('Next'));
    await tester.pumpAndSettle();
    expect(find.text('PopScope Page'), findsOneWidget);
    expect(find.text('Home Page'), findsNothing);
    expect(ModalRoute.of(oneContext)!.popDisposition, RoutePopDisposition.pop);
    if (!kIsWeb && defaultTargetPlatform == TargetPlatform.android) {
      expect(lastFrameworkHandlesBack, isTrue);
    }

    await simulateSystemBack();
    await tester.pumpAndSettle();
    expect(lastPopSuccess, true);
    expect(find.text('Home Page'), findsOneWidget);
    expect(find.text('PopScope Page'), findsNothing);
    expect(ModalRoute.of(homeContext)!.popDisposition, RoutePopDisposition.bubble);
    if (!kIsWeb && defaultTargetPlatform == TargetPlatform.android) {
      expect(lastFrameworkHandlesBack, isFalse);
    }
  }, variant: TargetPlatformVariant.all());

  testWidgets(
    'removing PopScope from the tree removes its effect on navigation',
    (WidgetTester tester) async {
      bool usePopScope = true;
      late StateSetter setState;
      late BuildContext context;
      await tester.pumpWidget(
        MaterialApp(
          initialRoute: '/',
          routes: <String, WidgetBuilder>{
            '/':
                (BuildContext buildContext) => Scaffold(
                  body: StatefulBuilder(
                    builder: (BuildContext buildContext, StateSetter stateSetter) {
                      context = buildContext;
                      setState = stateSetter;
                      const Widget child = Center(
                        child: Column(
                          mainAxisAlignment: MainAxisAlignment.center,
                          children: <Widget>[Text('Home/PopScope Page')],
                        ),
                      );
                      if (!usePopScope) {
                        return child;
                      }
                      return const PopScope<Object?>(canPop: false, child: child);
                    },
                  ),
                ),
          },
        ),
      );

      if (!kIsWeb && defaultTargetPlatform == TargetPlatform.android) {
        expect(lastFrameworkHandlesBack, isTrue);
      }
      expect(ModalRoute.of(context)!.popDisposition, RoutePopDisposition.doNotPop);

      setState(() {
        usePopScope = false;
      });
      await tester.pump();
      if (!kIsWeb && defaultTargetPlatform == TargetPlatform.android) {
        expect(lastFrameworkHandlesBack, isFalse);
      }
      expect(ModalRoute.of(context)!.popDisposition, RoutePopDisposition.bubble);
    },
    variant: TargetPlatformVariant.all(),
  );

  testWidgets('identical PopScopes', (WidgetTester tester) async {
    bool usePopScope1 = true;
    bool usePopScope2 = true;
    late StateSetter setState;
    late BuildContext context;
    await tester.pumpWidget(
      MaterialApp(
        home: Scaffold(
          body: StatefulBuilder(
            builder: (BuildContext buildContext, StateSetter stateSetter) {
              context = buildContext;
              setState = stateSetter;
              return Column(
                children: <Widget>[
                  if (usePopScope1) const PopScope<Object?>(canPop: false, child: Text('hello')),
                  if (usePopScope2) const PopScope<Object?>(canPop: false, child: Text('hello')),
                ],
              );
            },
          ),
        ),
      ),
    );

    if (!kIsWeb && defaultTargetPlatform == TargetPlatform.android) {
      expect(lastFrameworkHandlesBack, isTrue);
    }
    expect(ModalRoute.of(context)!.popDisposition, RoutePopDisposition.doNotPop);

    // Despite being in the widget tree twice, the ModalRoute has only ever
    // registered one PopScopeInterface for it. Removing one makes it think that
    // both have been removed.
    setState(() {
      usePopScope1 = false;
    });
    await tester.pump();
    if (!kIsWeb && defaultTargetPlatform == TargetPlatform.android) {
      expect(lastFrameworkHandlesBack, isTrue);
    }
    expect(ModalRoute.of(context)!.popDisposition, RoutePopDisposition.doNotPop);

    setState(() {
      usePopScope2 = false;
    });
    await tester.pump();
    if (!kIsWeb && defaultTargetPlatform == TargetPlatform.android) {
      expect(lastFrameworkHandlesBack, isFalse);
    }
    expect(ModalRoute.of(context)!.popDisposition, RoutePopDisposition.bubble);
  }, variant: TargetPlatformVariant.all());
}
