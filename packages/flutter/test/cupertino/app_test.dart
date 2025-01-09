// Copyright 2014 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

import 'package:flutter/cupertino.dart';
import 'package:flutter/foundation.dart';
import 'package:flutter/gestures.dart';
import 'package:flutter/material.dart';
import 'package:flutter/rendering.dart';
import 'package:flutter/services.dart';
import 'package:flutter_test/flutter_test.dart';

void main() {
  testWidgets('Heroes work', (WidgetTester tester) async {
    await tester.pumpWidget(
      CupertinoApp(
        home: ListView(
          children: <Widget>[
            const Hero(tag: 'a', child: Text('foo')),
            Builder(
              builder: (BuildContext context) {
                return CupertinoButton(
                  child: const Text('next'),
                  onPressed: () {
                    Navigator.push(
                      context,
                      CupertinoPageRoute<void>(
                        builder: (BuildContext context) {
                          return const Hero(tag: 'a', child: Text('foo'));
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
    );

    await tester.tap(find.text('next'));
    await tester.pump();
    await tester.pump(const Duration(milliseconds: 100));

    // During the hero transition, the hero widget is lifted off of both
    // page routes and exists as its own overlay on top of both routes.
    expect(find.widgetWithText(CupertinoPageRoute, 'foo'), findsNothing);
    expect(find.widgetWithText(Navigator, 'foo'), findsOneWidget);
  });

  testWidgets('Has default cupertino localizations', (WidgetTester tester) async {
    await tester.pumpWidget(
      CupertinoApp(
        home: Builder(
          builder: (BuildContext context) {
            return Column(
              children: <Widget>[
                Text(CupertinoLocalizations.of(context).selectAllButtonLabel),
                Text(
                  CupertinoLocalizations.of(context).datePickerMediumDate(DateTime(2018, 10, 4)),
                ),
              ],
            );
          },
        ),
      ),
    );

    expect(find.text('Select All'), findsOneWidget);
    expect(find.text('Thu Oct 4 '), findsOneWidget);
  });

  testWidgets('Can use dynamic color', (WidgetTester tester) async {
    const CupertinoDynamicColor dynamicColor = CupertinoDynamicColor.withBrightness(
      color: Color(0xFF000000),
      darkColor: Color(0xFF000001),
    );
    await tester.pumpWidget(
      const CupertinoApp(
        theme: CupertinoThemeData(brightness: Brightness.light),
        title: '',
        color: dynamicColor,
        home: Placeholder(),
      ),
    );

    expect(tester.widget<Title>(find.byType(Title)).color.value, 0xFF000000);

    await tester.pumpWidget(
      const CupertinoApp(
        theme: CupertinoThemeData(brightness: Brightness.dark),
        color: dynamicColor,
        title: '',
        home: Placeholder(),
      ),
    );

    expect(tester.widget<Title>(find.byType(Title)).color.value, 0xFF000001);
  });

  testWidgets('Can customize initial routes', (WidgetTester tester) async {
    final GlobalKey<NavigatorState> navigatorKey = GlobalKey<NavigatorState>();
    await tester.pumpWidget(
      CupertinoApp(
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
        routes: <String, WidgetBuilder>{
          '/': (BuildContext context) => const Text('regular page one'),
          '/abc': (BuildContext context) => const Text('regular page two'),
        },
      ),
    );
    expect(find.text('non-regular page two'), findsOneWidget);
    expect(find.text('non-regular page one'), findsNothing);
    expect(find.text('regular page one'), findsNothing);
    expect(find.text('regular page two'), findsNothing);
    navigatorKey.currentState!.pop();
    await tester.pumpAndSettle();
    expect(find.text('non-regular page two'), findsNothing);
    expect(find.text('non-regular page one'), findsOneWidget);
    expect(find.text('regular page one'), findsNothing);
    expect(find.text('regular page two'), findsNothing);
  });

  testWidgets('CupertinoApp.navigatorKey can be updated', (WidgetTester tester) async {
    final GlobalKey<NavigatorState> key1 = GlobalKey<NavigatorState>();
    await tester.pumpWidget(CupertinoApp(navigatorKey: key1, home: const Placeholder()));
    expect(key1.currentState, isA<NavigatorState>());
    final GlobalKey<NavigatorState> key2 = GlobalKey<NavigatorState>();
    await tester.pumpWidget(CupertinoApp(navigatorKey: key2, home: const Placeholder()));
    expect(key2.currentState, isA<NavigatorState>());
    expect(key1.currentState, isNull);
  });

  testWidgets('CupertinoApp.router works', (WidgetTester tester) async {
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
      CupertinoApp.router(
        routeInformationProvider: provider,
        routeInformationParser: SimpleRouteInformationParser(),
        routerDelegate: delegate,
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

  testWidgets('CupertinoApp.router works with onNavigationNotification', (
    WidgetTester tester,
  ) async {
    // This is a regression test for https://github.com/flutter/flutter/issues/139903.
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

    int navigationCount = 0;

    await tester.pumpWidget(
      CupertinoApp.router(
        routeInformationProvider: provider,
        routeInformationParser: SimpleRouteInformationParser(),
        routerDelegate: delegate,
        onNavigationNotification: (NavigationNotification? notification) {
          navigationCount += 1;
          return true;
        },
      ),
    );
    expect(find.text('initial'), findsOneWidget);

    expect(navigationCount, greaterThan(0));
    final int navigationCountAfterBuild = navigationCount;

    // Simulate android back button intent.
    final ByteData message = const JSONMethodCodec().encodeMethodCall(const MethodCall('popRoute'));
    await tester.binding.defaultBinaryMessenger.handlePlatformMessage(
      'flutter/navigation',
      message,
      (_) {},
    );
    await tester.pumpAndSettle();
    expect(find.text('popped'), findsOneWidget);

    expect(navigationCount, greaterThan(navigationCountAfterBuild));
  });

  testWidgets('CupertinoApp.router route information parser is optional', (
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
    await tester.pumpWidget(CupertinoApp.router(routerDelegate: delegate));
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
    'CupertinoApp.router throw if route information provider is provided but no route information parser',
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
      await tester.pumpWidget(
        CupertinoApp.router(routeInformationProvider: provider, routerDelegate: delegate),
      );
      expect(tester.takeException(), isAssertionError);
    },
  );

  testWidgets(
    'CupertinoApp.router throw if route configuration is provided along with other delegate',
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
      await tester.pumpWidget(
        CupertinoApp.router(routerDelegate: delegate, routerConfig: routerConfig),
      );
      expect(tester.takeException(), isAssertionError);
    },
  );

  testWidgets('CupertinoApp.router router config works', (WidgetTester tester) async {
    late SimpleNavigatorRouterDelegate delegate;
    addTearDown(() => delegate.dispose());
    final PlatformRouteInformationProvider provider = PlatformRouteInformationProvider(
      initialRouteInformation: RouteInformation(uri: Uri.parse('initial')),
    );
    addTearDown(provider.dispose);
    final RouterConfig<RouteInformation> routerConfig = RouterConfig<RouteInformation>(
      routeInformationProvider: provider,
      routeInformationParser: SimpleRouteInformationParser(),
      routerDelegate:
          delegate = SimpleNavigatorRouterDelegate(
            builder: (BuildContext context, RouteInformation information) {
              return Text(information.uri.toString());
            },
            onPopPage: (Route<void> route, void result, SimpleNavigatorRouterDelegate delegate) {
              delegate.routeInformation = RouteInformation(uri: Uri.parse('popped'));
              return route.didPop(result);
            },
          ),
      backButtonDispatcher: RootBackButtonDispatcher(),
    );
    await tester.pumpWidget(CupertinoApp.router(routerConfig: routerConfig));
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

  testWidgets('CupertinoApp has correct default ScrollBehavior', (WidgetTester tester) async {
    late BuildContext capturedContext;
    await tester.pumpWidget(
      CupertinoApp(
        home: Builder(
          builder: (BuildContext context) {
            capturedContext = context;
            return const Placeholder();
          },
        ),
      ),
    );
    expect(ScrollConfiguration.of(capturedContext).runtimeType, CupertinoScrollBehavior);
  });

  testWidgets('CupertinoApp has correct default multitouchDragStrategy', (
    WidgetTester tester,
  ) async {
    late BuildContext capturedContext;
    await tester.pumpWidget(
      CupertinoApp(
        home: Builder(
          builder: (BuildContext context) {
            capturedContext = context;
            return const Placeholder();
          },
        ),
      ),
    );

    final ScrollBehavior scrollBehavior = ScrollConfiguration.of(capturedContext);
    expect(scrollBehavior.runtimeType, CupertinoScrollBehavior);
    expect(
      scrollBehavior.getMultitouchDragStrategy(capturedContext),
      MultitouchDragStrategy.averageBoundaryPointers,
    );
  });

  testWidgets('A ScrollBehavior can be set for CupertinoApp', (WidgetTester tester) async {
    late BuildContext capturedContext;
    await tester.pumpWidget(
      CupertinoApp(
        scrollBehavior: const MockScrollBehavior(),
        home: Builder(
          builder: (BuildContext context) {
            capturedContext = context;
            return const Placeholder();
          },
        ),
      ),
    );
    final ScrollBehavior scrollBehavior = ScrollConfiguration.of(capturedContext);
    expect(scrollBehavior.runtimeType, MockScrollBehavior);
    expect(
      scrollBehavior.getScrollPhysics(capturedContext).runtimeType,
      NeverScrollableScrollPhysics,
    );
  });

  testWidgets(
    'When `useInheritedMediaQuery` is true an existing MediaQuery is used if one is available',
    (WidgetTester tester) async {
      late BuildContext capturedContext;
      final UniqueKey uniqueKey = UniqueKey();
      await tester.pumpWidget(
        MediaQuery(
          key: uniqueKey,
          data: const MediaQueryData(),
          child: CupertinoApp(
            useInheritedMediaQuery: true,
            builder: (BuildContext context, Widget? child) {
              capturedContext = context;
              return const Placeholder();
            },
            color: const Color(0xFF123456),
          ),
        ),
      );
      expect(capturedContext.dependOnInheritedWidgetOfExactType<MediaQuery>()?.key, uniqueKey);
    },
  );

  testWidgets('Text color is correctly resolved when CupertinoThemeData.brightness is null', (
    WidgetTester tester,
  ) async {
    debugBrightnessOverride = Brightness.dark;

    await tester.pumpWidget(const CupertinoApp(home: CupertinoPageScaffold(child: Text('Hello'))));

    final RenderParagraph paragraph = tester.renderObject(find.text('Hello'));
    final CupertinoDynamicColor textColor = paragraph.text.style!.color! as CupertinoDynamicColor;

    // App with non-null brightness, so resolving color
    // doesn't depend on the MediaQuery.platformBrightness.
    late BuildContext capturedContext;
    await tester.pumpWidget(
      CupertinoApp(
        theme: const CupertinoThemeData(brightness: Brightness.dark),
        home: Builder(
          builder: (BuildContext context) {
            capturedContext = context;

            return const Placeholder();
          },
        ),
      ),
    );

    // We expect the string representations of the colors to have darkColor indicated (*) as effective color.
    // (color = Color(0xff000000), *darkColor = Color(0xffffffff)*, resolved by: Builder)
    expect(textColor.toString(), CupertinoColors.label.resolveFrom(capturedContext).toString());

    debugBrightnessOverride = null;
  });

  testWidgets('CupertinoApp creates a Material theme with colors based off of Cupertino theme', (
    WidgetTester tester,
  ) async {
    late ThemeData appliedTheme;
    await tester.pumpWidget(
      CupertinoApp(
        theme: const CupertinoThemeData(primaryColor: CupertinoColors.activeGreen),
        home: Builder(
          builder: (BuildContext context) {
            appliedTheme = Theme.of(context);
            return const SizedBox();
          },
        ),
      ),
    );

    expect(appliedTheme.colorScheme.primary, CupertinoColors.activeGreen);
  });

  testWidgets('Cursor color is resolved when CupertinoThemeData.brightness is null', (
    WidgetTester tester,
  ) async {
    debugBrightnessOverride = Brightness.dark;

    RenderEditable findRenderEditable(WidgetTester tester) {
      final RenderObject root = tester.renderObject(find.byType(EditableText));
      expect(root, isNotNull);

      RenderEditable? renderEditable;
      void recursiveFinder(RenderObject child) {
        if (child is RenderEditable) {
          renderEditable = child;
          return;
        }
        child.visitChildren(recursiveFinder);
      }

      root.visitChildren(recursiveFinder);
      expect(renderEditable, isNotNull);
      return renderEditable!;
    }

    final FocusNode focusNode = FocusNode();
    addTearDown(focusNode.dispose);
    final TextEditingController controller = TextEditingController();
    addTearDown(controller.dispose);

    await tester.pumpWidget(
      CupertinoApp(
        theme: const CupertinoThemeData(primaryColor: CupertinoColors.activeOrange),
        home: CupertinoPageScaffold(
          child: Builder(
            builder: (BuildContext context) {
              return EditableText(
                backgroundCursorColor: DefaultSelectionStyle.of(context).selectionColor!,
                cursorColor: DefaultSelectionStyle.of(context).cursorColor!,
                controller: controller,
                focusNode: focusNode,
                style: const TextStyle(),
              );
            },
          ),
        ),
      ),
    );

    final RenderEditable editableText = findRenderEditable(tester);
    final Color cursorColor = editableText.cursorColor!;

    // Cursor color should be equal to the dark variant of the primary color.
    // Alpha value needs to be 0, because cursor is not visible by default.
    expect(cursorColor, CupertinoColors.activeOrange.darkColor.withAlpha(0));

    debugBrightnessOverride = null;
  });

  testWidgets(
    'Assert in buildScrollbar that controller != null when using it',
    (WidgetTester tester) async {
      const ScrollBehavior defaultBehavior = CupertinoScrollBehavior();
      late BuildContext capturedContext;

      await tester.pumpWidget(
        ScrollConfiguration(
          // Avoid the default ones here.
          behavior: const CupertinoScrollBehavior().copyWith(scrollbars: false),
          child: SingleChildScrollView(
            child: Builder(
              builder: (BuildContext context) {
                capturedContext = context;
                return Container(height: 1000.0);
              },
            ),
          ),
        ),
      );

      const ScrollableDetails details = ScrollableDetails(direction: AxisDirection.down);
      final Widget child = Container();

      switch (defaultTargetPlatform) {
        case TargetPlatform.android:
        case TargetPlatform.fuchsia:
        case TargetPlatform.iOS:
          // Does not throw if we aren't using it.
          defaultBehavior.buildScrollbar(capturedContext, child, details);
        case TargetPlatform.linux:
        case TargetPlatform.macOS:
        case TargetPlatform.windows:
          expect(
            () {
              defaultBehavior.buildScrollbar(capturedContext, child, details);
            },
            throwsA(
              isA<AssertionError>().having(
                (AssertionError error) => error.toString(),
                'description',
                contains('details.controller != null'),
              ),
            ),
          );
      }
    },
    variant: TargetPlatformVariant.all(),
  );
}

class MockScrollBehavior extends ScrollBehavior {
  const MockScrollBehavior();

  @override
  ScrollPhysics getScrollPhysics(BuildContext context) => const NeverScrollableScrollPhysics();
}

typedef SimpleRouterDelegateBuilder =
    Widget Function(BuildContext context, RouteInformation information);
typedef SimpleNavigatorRouterDelegatePopPage<T> =
    bool Function(Route<T> route, T result, SimpleNavigatorRouterDelegate delegate);

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
  SimpleNavigatorRouterDelegate({required this.builder, this.onPopPage});

  @override
  GlobalKey<NavigatorState> navigatorKey = GlobalKey<NavigatorState>();

  RouteInformation get routeInformation => _routeInformation;
  late RouteInformation _routeInformation;
  set routeInformation(RouteInformation newValue) {
    _routeInformation = newValue;
    notifyListeners();
  }

  SimpleRouterDelegateBuilder builder;
  SimpleNavigatorRouterDelegatePopPage<void>? onPopPage;

  @override
  Future<void> setNewRoutePath(RouteInformation configuration) {
    _routeInformation = configuration;
    return SynchronousFuture<void>(null);
  }

  bool _handlePopPage(Route<void> route, void data) {
    return onPopPage!(route, data, this);
  }

  @override
  Widget build(BuildContext context) {
    return Navigator(
      key: navigatorKey,
      onPopPage: _handlePopPage,
      pages: <Page<void>>[
        // We need at least two pages for the pop to propagate through.
        // Otherwise, the navigator will bubble the pop to the system navigator.
        const CupertinoPage<void>(child: Text('base')),
        CupertinoPage<void>(
          key: ValueKey<String?>(routeInformation.uri.toString()),
          child: builder(context, routeInformation),
        ),
      ],
    );
  }
}
