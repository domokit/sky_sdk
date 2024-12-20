// Copyright 2014 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

import 'package:flutter/foundation.dart';
import 'package:flutter/material.dart';
import 'package:flutter/services.dart';
import 'package:flutter_test/flutter_test.dart';

class TestIntent extends Intent {
  const TestIntent();
}

class TestAction extends Action<Intent> {
  TestAction();

  int calls = 0;

  @override
  void invoke(Intent intent) {
    calls += 1;
  }
}

void main() {
  testWidgets('WidgetsApp with builder only', (WidgetTester tester) async {
    final GlobalKey key = GlobalKey();
    await tester.pumpWidget(
      WidgetsApp(
        key: key,
        builder: (BuildContext context, Widget? child) {
          return const Placeholder();
        },
        color: const Color(0xFF123456),
      ),
    );
    expect(find.byKey(key), findsOneWidget);
  });

  testWidgets('WidgetsApp default key bindings', (WidgetTester tester) async {
    bool? checked = false;
    final GlobalKey key = GlobalKey();
    await tester.pumpWidget(
      WidgetsApp(
        key: key,
        builder: (BuildContext context, Widget? child) {
          return Material(
            child: Checkbox(
              value: checked,
              autofocus: true,
              onChanged: (bool? value) {
                checked = value;
              },
            ),
          );
        },
        color: const Color(0xFF123456),
      ),
    );
    await tester.pump(); // Wait for focus to take effect.
    await tester.sendKeyEvent(LogicalKeyboardKey.space);
    await tester.pumpAndSettle();
    // Default key mapping worked.
    expect(checked, isTrue);
  });

  testWidgets('WidgetsApp can override default key bindings', (WidgetTester tester) async {
    final TestAction action = TestAction();
    bool? checked = false;
    final GlobalKey key = GlobalKey();
    await tester.pumpWidget(
      WidgetsApp(
        key: key,
        actions: <Type, Action<Intent>>{TestIntent: action},
        shortcuts: const <ShortcutActivator, Intent>{
          SingleActivator(LogicalKeyboardKey.space): TestIntent(),
        },
        builder: (BuildContext context, Widget? child) {
          return Material(
            child: Checkbox(
              value: checked,
              autofocus: true,
              onChanged: (bool? value) {
                checked = value;
              },
            ),
          );
        },
        color: const Color(0xFF123456),
      ),
    );
    await tester.pump();

    await tester.sendKeyEvent(LogicalKeyboardKey.space);
    await tester.pumpAndSettle();
    // Default key mapping was not invoked.
    expect(checked, isFalse);
    // Overridden mapping was invoked.
    expect(action.calls, equals(1));
  });

  testWidgets('WidgetsApp default activation key mappings work', (WidgetTester tester) async {
    bool? checked = false;

    await tester.pumpWidget(
      WidgetsApp(
        builder: (BuildContext context, Widget? child) {
          return Material(
            child: Checkbox(
              value: checked,
              autofocus: true,
              onChanged: (bool? value) {
                checked = value;
              },
            ),
          );
        },
        color: const Color(0xFF123456),
      ),
    );
    await tester.pump();

    // Test three default buttons for the activation action.
    await tester.sendKeyEvent(LogicalKeyboardKey.space);
    await tester.pumpAndSettle();
    expect(checked, isTrue);

    // Only space is used as an activation key on web.
    if (kIsWeb) {
      return;
    }

    checked = false;
    await tester.sendKeyEvent(LogicalKeyboardKey.enter);
    await tester.pumpAndSettle();
    expect(checked, isTrue);

    checked = false;
    await tester.sendKeyEvent(LogicalKeyboardKey.numpadEnter);
    await tester.pumpAndSettle();
    expect(checked, isTrue);

    checked = false;
    await tester.sendKeyEvent(LogicalKeyboardKey.gameButtonA);
    await tester.pumpAndSettle();
    expect(checked, isTrue);

    checked = false;
    await tester.sendKeyEvent(LogicalKeyboardKey.select);
    await tester.pumpAndSettle();
    expect(checked, isTrue);
  }, variant: KeySimulatorTransitModeVariant.all());

  testWidgets('Title is not created if title is not passed and kIsWeb', (
    WidgetTester tester,
  ) async {
    await tester.pumpWidget(
      WidgetsApp(
        color: const Color(0xFF123456),
        builder: (BuildContext context, Widget? child) => Container(),
      ),
    );

    expect(find.byType(Title), kIsWeb ? findsNothing : findsOneWidget);
  });

  group('error control test', () {
    Future<void> expectFlutterError({
      required GlobalKey<NavigatorState> key,
      required Widget widget,
      required WidgetTester tester,
      required String errorMessage,
    }) async {
      await tester.pumpWidget(widget);
      late FlutterError error;
      try {
        key.currentState!.pushNamed('/path');
      } on FlutterError catch (e) {
        error = e;
      } finally {
        expect(error, isNotNull);
        expect(error, isFlutterError);
        expect(error.toStringDeep(), errorMessage);
      }
    }

    testWidgets('push unknown route when onUnknownRoute is null', (WidgetTester tester) async {
      final GlobalKey<NavigatorState> key = GlobalKey<NavigatorState>();
      expectFlutterError(
        key: key,
        tester: tester,
        widget: MaterialApp(navigatorKey: key, home: Container(), onGenerateRoute: (_) => null),
        errorMessage:
            'FlutterError\n'
            '   Could not find a generator for route RouteSettings("/path", null)\n'
            '   in the _WidgetsAppState.\n'
            '   Make sure your root app widget has provided a way to generate\n'
            '   this route.\n'
            '   Generators for routes are searched for in the following order:\n'
            '    1. For the "/" route, the "home" property, if non-null, is used.\n'
            '    2. Otherwise, the "routes" table is used, if it has an entry for\n'
            '   the route.\n'
            '    3. Otherwise, onGenerateRoute is called. It should return a\n'
            '   non-null value for any valid route not handled by "home" and\n'
            '   "routes".\n'
            '    4. Finally if all else fails onUnknownRoute is called.\n'
            '   Unfortunately, onUnknownRoute was not set.\n',
      );
    });

    testWidgets('push unknown route when onUnknownRoute returns null', (WidgetTester tester) async {
      final GlobalKey<NavigatorState> key = GlobalKey<NavigatorState>();
      expectFlutterError(
        key: key,
        tester: tester,
        widget: MaterialApp(
          navigatorKey: key,
          home: Container(),
          onGenerateRoute: (_) => null,
          onUnknownRoute: (_) => null,
        ),
        errorMessage:
            'FlutterError\n'
            '   The onUnknownRoute callback returned null.\n'
            '   When the _WidgetsAppState requested the route\n'
            '   RouteSettings("/path", null) from its onUnknownRoute callback,\n'
            '   the callback returned null. Such callbacks must never return\n'
            '   null.\n',
      );
    });
  });

  testWidgets('WidgetsApp can customize initial routes', (WidgetTester tester) async {
    final GlobalKey<NavigatorState> navigatorKey = GlobalKey<NavigatorState>();
    await tester.pumpWidget(
      WidgetsApp(
        navigatorKey: navigatorKey,
        onGenerateInitialRoutes: (String initialRoute) {
          expect(initialRoute, '/abc');
          return <Route<void>>[
            PageRouteBuilder<void>(
              pageBuilder: (
                BuildContext context,
                Animation<double> animation,
                Animation<double> secondaryAnimation,
              ) {
                return const Text('non-regular page one');
              },
            ),
            PageRouteBuilder<void>(
              pageBuilder: (
                BuildContext context,
                Animation<double> animation,
                Animation<double> secondaryAnimation,
              ) {
                return const Text('non-regular page two');
              },
            ),
          ];
        },
        initialRoute: '/abc',
        onGenerateRoute: (RouteSettings settings) {
          return PageRouteBuilder<void>(
            pageBuilder: (
              BuildContext context,
              Animation<double> animation,
              Animation<double> secondaryAnimation,
            ) {
              return const Text('regular page');
            },
          );
        },
        color: const Color(0xFF123456),
      ),
    );
    expect(find.text('non-regular page two'), findsOneWidget);
    expect(find.text('non-regular page one'), findsNothing);
    expect(find.text('regular page'), findsNothing);
    navigatorKey.currentState!.pop();
    await tester.pumpAndSettle();
    expect(find.text('non-regular page two'), findsNothing);
    expect(find.text('non-regular page one'), findsOneWidget);
    expect(find.text('regular page'), findsNothing);
  });

  testWidgets('WidgetsApp.router works', (WidgetTester tester) async {
    final PlatformRouteInformationProvider provider = PlatformRouteInformationProvider(
      initialRouteInformation: RouteInformation(uri: Uri.parse('initial')),
    );
    addTearDown(provider.dispose);
    final SimpleNavigatorRouterDelegate delegate = SimpleNavigatorRouterDelegate(
      builder: (BuildContext context, RouteInformation information) {
        return Text(information.uri.toString());
      },
      onPopPage: (Route<void> route, void result, SimpleNavigatorRouterDelegate delegate) {
        delegate.routeInformation = RouteInformation(uri: Uri.parse('popped'));
        return route.didPop(result);
      },
    );
    addTearDown(delegate.dispose);
    await tester.pumpWidget(
      WidgetsApp.router(
        routeInformationProvider: provider,
        routeInformationParser: SimpleRouteInformationParser(),
        routerDelegate: delegate,
        color: const Color(0xFF123456),
      ),
    );
    expect(find.text('initial'), findsOneWidget);

    // Simulate android back button intent.
    final ByteData message = const JSONMethodCodec().encodeMethodCall(const MethodCall('popRoute'));
    await tester.binding.defaultBinaryMessenger.handlePlatformMessage(
      'flutter/navigation',
      message,
      (_) {},
    );
    await tester.pumpAndSettle();
    expect(find.text('popped'), findsOneWidget);
  });

  testWidgets('WidgetsApp.router route information parser is optional', (
    WidgetTester tester,
  ) async {
    final SimpleNavigatorRouterDelegate delegate = SimpleNavigatorRouterDelegate(
      builder: (BuildContext context, RouteInformation information) {
        return Text(information.uri.toString());
      },
      onPopPage: (Route<void> route, void result, SimpleNavigatorRouterDelegate delegate) {
        delegate.routeInformation = RouteInformation(uri: Uri.parse('popped'));
        return route.didPop(result);
      },
    );
    addTearDown(delegate.dispose);
    delegate.routeInformation = RouteInformation(uri: Uri.parse('initial'));
    await tester.pumpWidget(
      WidgetsApp.router(routerDelegate: delegate, color: const Color(0xFF123456)),
    );
    expect(find.text('initial'), findsOneWidget);

    // Simulate android back button intent.
    final ByteData message = const JSONMethodCodec().encodeMethodCall(const MethodCall('popRoute'));
    await tester.binding.defaultBinaryMessenger.handlePlatformMessage(
      'flutter/navigation',
      message,
      (_) {},
    );
    await tester.pumpAndSettle();
    expect(find.text('popped'), findsOneWidget);
  });

  testWidgets(
    'WidgetsApp.router throw if route information provider is provided but no route information parser',
    (WidgetTester tester) async {
      final SimpleNavigatorRouterDelegate delegate = SimpleNavigatorRouterDelegate(
        builder: (BuildContext context, RouteInformation information) {
          return Text(information.uri.toString());
        },
        onPopPage: (Route<void> route, void result, SimpleNavigatorRouterDelegate delegate) {
          delegate.routeInformation = RouteInformation(uri: Uri.parse('popped'));
          return route.didPop(result);
        },
      );
      addTearDown(delegate.dispose);
      delegate.routeInformation = RouteInformation(uri: Uri.parse('initial'));
      final PlatformRouteInformationProvider provider = PlatformRouteInformationProvider(
        initialRouteInformation: RouteInformation(uri: Uri.parse('initial')),
      );
      addTearDown(provider.dispose);
      await expectLater(() async {
        await tester.pumpWidget(
          WidgetsApp.router(
            routeInformationProvider: provider,
            routerDelegate: delegate,
            color: const Color(0xFF123456),
          ),
        );
      }, throwsAssertionError);
    },
  );

  testWidgets(
    'WidgetsApp.router throw if route configuration is provided along with other delegate',
    (WidgetTester tester) async {
      final SimpleNavigatorRouterDelegate delegate = SimpleNavigatorRouterDelegate(
        builder: (BuildContext context, RouteInformation information) {
          return Text(information.uri.toString());
        },
        onPopPage: (Route<void> route, void result, SimpleNavigatorRouterDelegate delegate) {
          delegate.routeInformation = RouteInformation(uri: Uri.parse('popped'));
          return route.didPop(result);
        },
      );
      addTearDown(delegate.dispose);
      delegate.routeInformation = RouteInformation(uri: Uri.parse('initial'));
      final RouterConfig<RouteInformation> routerConfig = RouterConfig<RouteInformation>(
        routerDelegate: delegate,
      );
      await expectLater(() async {
        await tester.pumpWidget(
          WidgetsApp.router(
            routerDelegate: delegate,
            routerConfig: routerConfig,
            color: const Color(0xFF123456),
          ),
        );
      }, throwsAssertionError);
    },
  );

  testWidgets('WidgetsApp.router router config works', (WidgetTester tester) async {
    final SimpleNavigatorRouterDelegate delegate = SimpleNavigatorRouterDelegate(
      builder: (BuildContext context, RouteInformation information) {
        return Text(information.uri.toString());
      },
      onPopPage: (Route<void> route, void result, SimpleNavigatorRouterDelegate delegate) {
        delegate.routeInformation = RouteInformation(uri: Uri.parse('popped'));
        return route.didPop(result);
      },
    );
    addTearDown(delegate.dispose);
    final PlatformRouteInformationProvider provider = PlatformRouteInformationProvider(
      initialRouteInformation: RouteInformation(uri: Uri.parse('initial')),
    );
    addTearDown(provider.dispose);
    final RouterConfig<RouteInformation> routerConfig = RouterConfig<RouteInformation>(
      routeInformationProvider: provider,
      routeInformationParser: SimpleRouteInformationParser(),
      routerDelegate: delegate,
      backButtonDispatcher: RootBackButtonDispatcher(),
    );
    await tester.pumpWidget(
      WidgetsApp.router(routerConfig: routerConfig, color: const Color(0xFF123456)),
    );
    expect(find.text('initial'), findsOneWidget);

    // Simulate android back button intent.
    final ByteData message = const JSONMethodCodec().encodeMethodCall(const MethodCall('popRoute'));
    await tester.binding.defaultBinaryMessenger.handlePlatformMessage(
      'flutter/navigation',
      message,
      (_) {},
    );
    await tester.pumpAndSettle();
    expect(find.text('popped'), findsOneWidget);
  });

  testWidgets('WidgetsApp.router has correct default', (WidgetTester tester) async {
    final SimpleNavigatorRouterDelegate delegate = SimpleNavigatorRouterDelegate(
      builder: (BuildContext context, RouteInformation information) {
        return Text(information.uri.toString());
      },
      onPopPage:
          (Route<Object?> route, Object? result, SimpleNavigatorRouterDelegate delegate) => true,
    );
    addTearDown(delegate.dispose);
    await tester.pumpWidget(
      WidgetsApp.router(
        routeInformationParser: SimpleRouteInformationParser(),
        routerDelegate: delegate,
        color: const Color(0xFF123456),
      ),
    );
    expect(find.text('/'), findsOneWidget);
  });

  testWidgets('WidgetsApp has correct default ScrollBehavior', (WidgetTester tester) async {
    late BuildContext capturedContext;
    await tester.pumpWidget(
      WidgetsApp(
        builder: (BuildContext context, Widget? child) {
          capturedContext = context;
          return const Placeholder();
        },
        color: const Color(0xFF123456),
      ),
    );
    expect(ScrollConfiguration.of(capturedContext).runtimeType, ScrollBehavior);
  });

  test('basicLocaleListResolution', () {
    // Matches exactly for language code.
    expect(
      basicLocaleListResolution(
        <Locale>[const Locale('zh'), const Locale('un'), const Locale('en')],
        <Locale>[const Locale('en')],
      ),
      const Locale('en'),
    );

    // Matches exactly for language code and country code.
    expect(
      basicLocaleListResolution(
        <Locale>[const Locale('en'), const Locale('en', 'US')],
        <Locale>[const Locale('en', 'US')],
      ),
      const Locale('en', 'US'),
    );

    // Matches language+script over language+country
    expect(
      basicLocaleListResolution(
        <Locale>[
          const Locale.fromSubtags(languageCode: 'zh', scriptCode: 'Hant', countryCode: 'HK'),
        ],
        <Locale>[
          const Locale.fromSubtags(languageCode: 'zh', countryCode: 'HK'),
          const Locale.fromSubtags(languageCode: 'zh', scriptCode: 'Hant'),
        ],
      ),
      const Locale.fromSubtags(languageCode: 'zh', scriptCode: 'Hant'),
    );

    // Matches exactly for language code, script code and country code.
    expect(
      basicLocaleListResolution(
        <Locale>[
          const Locale.fromSubtags(languageCode: 'zh'),
          const Locale.fromSubtags(languageCode: 'zh', scriptCode: 'Hant', countryCode: 'TW'),
        ],
        <Locale>[
          const Locale.fromSubtags(languageCode: 'zh', scriptCode: 'Hant', countryCode: 'TW'),
        ],
      ),
      const Locale.fromSubtags(languageCode: 'zh', scriptCode: 'Hant', countryCode: 'TW'),
    );

    // Selects for country code if the language code is not found in the
    // preferred locales list.
    expect(
      basicLocaleListResolution(
        <Locale>[
          const Locale.fromSubtags(languageCode: 'en'),
          const Locale.fromSubtags(languageCode: 'ar', countryCode: 'tn'),
        ],
        <Locale>[const Locale.fromSubtags(languageCode: 'fr', countryCode: 'tn')],
      ),
      const Locale.fromSubtags(languageCode: 'fr', countryCode: 'tn'),
    );

    // Selects first (default) locale when no match at all is found.
    expect(
      basicLocaleListResolution(
        <Locale>[const Locale('tn')],
        <Locale>[const Locale('zh'), const Locale('un'), const Locale('en')],
      ),
      const Locale('zh'),
    );
  });

  testWidgets("WidgetsApp reports an exception if the selected locale isn't supported", (
    WidgetTester tester,
  ) async {
    late final List<Locale>? localesArg;
    late final Iterable<Locale> supportedLocalesArg;
    await tester.pumpWidget(
      MaterialApp(
        // This uses a MaterialApp because it introduces some actual localizations.
        localeListResolutionCallback: (List<Locale>? locales, Iterable<Locale> supportedLocales) {
          localesArg = locales;
          supportedLocalesArg = supportedLocales;
          return const Locale('C_UTF-8');
        },
        builder: (BuildContext context, Widget? child) => const Placeholder(),
        color: const Color(0xFF000000),
      ),
    );
    if (!kIsWeb) {
      // On web, `flutter test` does not guarantee a particular locale, but
      // when using `flutter_tester`, we guarantee that it's en-US, zh-CN.
      // https://github.com/flutter/flutter/issues/93290
      expect(localesArg, const <Locale>[Locale('en', 'US'), Locale('zh', 'CN')]);
    }
    expect(supportedLocalesArg, const <Locale>[Locale('en', 'US')]);
    expect(
      tester.takeException(),
      "Warning: This application's locale, C_UTF-8, is not supported by all of its localization delegates.",
    );
  });

  testWidgets("WidgetsApp doesn't have dependency on MediaQuery", (WidgetTester tester) async {
    int routeBuildCount = 0;

    final Widget widget = WidgetsApp(
      color: const Color.fromARGB(255, 255, 255, 255),
      onGenerateRoute: (_) {
        return PageRouteBuilder<void>(
          pageBuilder: (_, __, ___) {
            routeBuildCount++;
            return const Placeholder();
          },
        );
      },
    );

    await tester.pumpWidget(
      MediaQuery(data: const MediaQueryData(textScaler: TextScaler.linear(10)), child: widget),
    );

    expect(routeBuildCount, equals(1));

    await tester.pumpWidget(
      MediaQuery(data: const MediaQueryData(textScaler: TextScaler.linear(20)), child: widget),
    );

    expect(routeBuildCount, equals(1));
  });

  testWidgets(
    'WidgetsApp provides meta based shortcuts for iOS and macOS',
    (WidgetTester tester) async {
      final FocusNode focusNode = FocusNode();
      addTearDown(focusNode.dispose);

      final SelectAllSpy selectAllSpy = SelectAllSpy();
      final CopySpy copySpy = CopySpy();
      final PasteSpy pasteSpy = PasteSpy();
      final Map<Type, Action<Intent>> actions = <Type, Action<Intent>>{
        // Copy Paste
        SelectAllTextIntent: selectAllSpy,
        CopySelectionTextIntent: copySpy,
        PasteTextIntent: pasteSpy,
      };
      await tester.pumpWidget(
        WidgetsApp(
          builder: (BuildContext context, Widget? child) {
            return Actions(
              actions: actions,
              child: Focus(focusNode: focusNode, child: const Placeholder()),
            );
          },
          color: const Color(0xFF123456),
        ),
      );
      focusNode.requestFocus();
      await tester.pump();
      expect(selectAllSpy.invoked, isFalse);
      expect(copySpy.invoked, isFalse);
      expect(pasteSpy.invoked, isFalse);

      // Select all.
      await tester.sendKeyDownEvent(LogicalKeyboardKey.metaLeft);
      await tester.sendKeyDownEvent(LogicalKeyboardKey.keyA);
      await tester.sendKeyUpEvent(LogicalKeyboardKey.keyA);
      await tester.sendKeyUpEvent(LogicalKeyboardKey.metaLeft);
      await tester.pump();

      expect(selectAllSpy.invoked, isTrue);
      expect(copySpy.invoked, isFalse);
      expect(pasteSpy.invoked, isFalse);

      // Copy.
      await tester.sendKeyDownEvent(LogicalKeyboardKey.metaLeft);
      await tester.sendKeyDownEvent(LogicalKeyboardKey.keyC);
      await tester.sendKeyUpEvent(LogicalKeyboardKey.keyC);
      await tester.sendKeyUpEvent(LogicalKeyboardKey.metaLeft);
      await tester.pump();

      expect(selectAllSpy.invoked, isTrue);
      expect(copySpy.invoked, isTrue);
      expect(pasteSpy.invoked, isFalse);

      // Paste.
      await tester.sendKeyDownEvent(LogicalKeyboardKey.metaLeft);
      await tester.sendKeyDownEvent(LogicalKeyboardKey.keyV);
      await tester.sendKeyUpEvent(LogicalKeyboardKey.keyV);
      await tester.sendKeyUpEvent(LogicalKeyboardKey.metaLeft);
      await tester.pump();

      expect(selectAllSpy.invoked, isTrue);
      expect(copySpy.invoked, isTrue);
      expect(pasteSpy.invoked, isTrue);
    },
    variant: const TargetPlatformVariant(<TargetPlatform>{
      TargetPlatform.iOS,
      TargetPlatform.macOS,
    }),
  );

  group('Android Predictive Back', () {
    Future<void> setAppLifeCycleState(AppLifecycleState state) async {
      final ByteData? message = const StringCodec().encodeMessage(state.toString());
      await TestDefaultBinaryMessengerBinding.instance.defaultBinaryMessenger.handlePlatformMessage(
        'flutter/lifecycle',
        message,
        (ByteData? data) {},
      );
    }

    final List<bool> frameworkHandlesBacks = <bool>[];
    setUp(() async {
      frameworkHandlesBacks.clear();
      TestDefaultBinaryMessengerBinding.instance.defaultBinaryMessenger.setMockMethodCallHandler(
        SystemChannels.platform,
        (MethodCall methodCall) async {
          if (methodCall.method == 'SystemNavigator.setFrameworkHandlesBack') {
            expect(methodCall.arguments, isA<bool>());
            frameworkHandlesBacks.add(methodCall.arguments as bool);
          }
          return;
        },
      );
    });

    tearDown(() async {
      TestDefaultBinaryMessengerBinding.instance.defaultBinaryMessenger.setMockMethodCallHandler(
        SystemChannels.platform,
        null,
      );
      await setAppLifeCycleState(AppLifecycleState.resumed);
    });

    testWidgets(
      'WidgetsApp calls setFrameworkHandlesBack only when app is ready',
      (WidgetTester tester) async {
        // Start in the `resumed` state, where setFrameworkHandlesBack should be
        // called like normal.
        await setAppLifeCycleState(AppLifecycleState.resumed);

        late BuildContext currentContext;
        await tester.pumpWidget(
          WidgetsApp(
            color: const Color(0xFF123456),
            builder: (BuildContext context, Widget? child) {
              currentContext = context;
              return const Placeholder();
            },
          ),
        );

        expect(frameworkHandlesBacks, isEmpty);

        const NavigationNotification(canHandlePop: true).dispatch(currentContext);
        await tester.pumpAndSettle();
        expect(frameworkHandlesBacks, isNotEmpty);
        expect(frameworkHandlesBacks.last, isTrue);

        const NavigationNotification(canHandlePop: false).dispatch(currentContext);
        await tester.pumpAndSettle();
        expect(frameworkHandlesBacks.last, isFalse);

        // Set the app state to inactive, where setFrameworkHandlesBack is still
        // called. This could happen when responding to a tap on a notification
        // when the app is not active and immediately navigating, for example.
        // See https://github.com/flutter/flutter/pull/154313.
        await setAppLifeCycleState(AppLifecycleState.inactive);

        final int inactiveStartCallsLength = frameworkHandlesBacks.length;
        const NavigationNotification(canHandlePop: true).dispatch(currentContext);
        await tester.pumpAndSettle();
        expect(frameworkHandlesBacks, hasLength(inactiveStartCallsLength + 1));

        const NavigationNotification(canHandlePop: false).dispatch(currentContext);
        await tester.pumpAndSettle();
        expect(frameworkHandlesBacks, hasLength(inactiveStartCallsLength + 2));

        // Set the app state to detached, where setFrameworkHandlesBack shouldn't
        // be called.
        await setAppLifeCycleState(AppLifecycleState.detached);

        final int finalCallsLength = frameworkHandlesBacks.length;
        const NavigationNotification(canHandlePop: true).dispatch(currentContext);
        await tester.pumpAndSettle();
        expect(frameworkHandlesBacks, hasLength(finalCallsLength));

        const NavigationNotification(canHandlePop: false).dispatch(currentContext);
        await tester.pumpAndSettle();
        expect(frameworkHandlesBacks, hasLength(finalCallsLength));
      },
      skip: kIsWeb, // [intended] predictive back is only native Android.
      variant: const TargetPlatformVariant(<TargetPlatform>{TargetPlatform.android}),
    );
  });
}

typedef SimpleRouterDelegateBuilder =
    Widget Function(BuildContext context, RouteInformation information);
typedef SimpleNavigatorRouterDelegatePopPage<T> =
    bool Function(Route<T> route, T result, SimpleNavigatorRouterDelegate delegate);

class SelectAllSpy extends Action<SelectAllTextIntent> {
  bool invoked = false;
  @override
  void invoke(SelectAllTextIntent intent) {
    invoked = true;
  }
}

class CopySpy extends Action<CopySelectionTextIntent> {
  bool invoked = false;
  @override
  void invoke(CopySelectionTextIntent intent) {
    invoked = true;
  }
}

class PasteSpy extends Action<PasteTextIntent> {
  bool invoked = false;
  @override
  void invoke(PasteTextIntent intent) {
    invoked = true;
  }
}

class SimpleRouteInformationParser extends RouteInformationParser<RouteInformation> {
  SimpleRouteInformationParser();

  @override
  Future<RouteInformation> parseRouteInformation(RouteInformation information) {
    return SynchronousFuture<RouteInformation>(information);
  }

  @override
  RouteInformation restoreRouteInformation(RouteInformation configuration) {
    return configuration;
  }
}

class SimpleNavigatorRouterDelegate extends RouterDelegate<RouteInformation>
    with PopNavigatorRouterDelegateMixin<RouteInformation>, ChangeNotifier {
  SimpleNavigatorRouterDelegate({required this.builder, required this.onPopPage});

  @override
  GlobalKey<NavigatorState> navigatorKey = GlobalKey<NavigatorState>();

  RouteInformation get routeInformation => _routeInformation;
  late RouteInformation _routeInformation;
  set routeInformation(RouteInformation newValue) {
    _routeInformation = newValue;
    notifyListeners();
  }

  final SimpleRouterDelegateBuilder builder;
  final SimpleNavigatorRouterDelegatePopPage<void> onPopPage;

  @override
  Future<void> setNewRoutePath(RouteInformation configuration) {
    _routeInformation = configuration;
    return SynchronousFuture<void>(null);
  }

  bool _handlePopPage(Route<void> route, void data) {
    return onPopPage(route, data, this);
  }

  @override
  Widget build(BuildContext context) {
    return Navigator(
      key: navigatorKey,
      onPopPage: _handlePopPage,
      pages: <Page<void>>[
        // We need at least two pages for the pop to propagate through.
        // Otherwise, the navigator will bubble the pop to the system navigator.
        const MaterialPage<void>(child: Text('base')),
        MaterialPage<void>(
          key: ValueKey<String>(routeInformation.uri.toString()),
          child: builder(context, routeInformation),
        ),
      ],
    );
  }
}
