// Copyright 2014 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

import 'package:flutter/cupertino.dart';
import 'package:flutter/foundation.dart';
import 'package:flutter/material.dart';

/// This sample demonstrates creating a custom page transition that is able to
/// override the outgoing transition of the route behind it in the navigation
/// stack using [DelegatedTransitionBuilder], using a [MaterialApp.router]
/// pattern of navigation.

void main() {
  runApp(FlexibleRouteTransitionsApp());
}

class FlexibleRouteTransitionsApp extends StatelessWidget {
  FlexibleRouteTransitionsApp({super.key});

  final _MyRouteInformationParser _routeInformationParser = _MyRouteInformationParser();
  final MyRouterDelegate _routerDelegate = MyRouterDelegate();

  @override
  Widget build(BuildContext context) {
    return MaterialApp.router(
      title: 'Mixing Routes',
      theme: ThemeData(
        colorScheme: ColorScheme.fromSeed(seedColor: Colors.deepPurple),
        pageTransitionsTheme: const PageTransitionsTheme(
          builders: <TargetPlatform, PageTransitionsBuilder>{
            // By default the zoom builder is used on all platforms but iOS. Normally
            // on iOS the default is the Cupertino sliding transition. Setting
            // it to use zoom on all platforms allows the example to show multiple
            // transitions in one app for all platforms.
            TargetPlatform.iOS: ZoomPageTransitionsBuilder(),
          },
        ),
      ),
      routeInformationParser: _routeInformationParser,
      routerDelegate: _routerDelegate,
    );
  }
}

class _MyRouteInformationParser extends RouteInformationParser<MyPageConfiguration> {
  @override
  SynchronousFuture<MyPageConfiguration> parseRouteInformation(RouteInformation routeInformation) {
    return SynchronousFuture<MyPageConfiguration>(
      MyPageConfiguration.values.firstWhere((MyPageConfiguration pageConfiguration) {
        return pageConfiguration.uriString == routeInformation.uri.toString();
      }, orElse: () => MyPageConfiguration.unknown),
    );
  }

  @override
  RouteInformation? restoreRouteInformation(MyPageConfiguration configuration) {
    return RouteInformation(uri: configuration.uri);
  }
}

class MyRouterDelegate extends RouterDelegate<MyPageConfiguration> {
  final Set<VoidCallback> _listeners = <VoidCallback>{};
  final List<MyPageConfiguration> _pages = <MyPageConfiguration>[];

  void _notifyListeners() {
    for (final VoidCallback listener in _listeners) {
      listener();
    }
  }

  void onNavigateToHome() {
    _pages.clear();
    _pages.add(MyPageConfiguration.home);
    _notifyListeners();
  }

  void onNavigateToZoom() {
    _pages.add(MyPageConfiguration.zoom);
    _notifyListeners();
  }

  void onNavigateToIOS() {
    _pages.add(MyPageConfiguration.iOS);
    _notifyListeners();
  }

  void onNavigateToVertical() {
    _pages.add(MyPageConfiguration.vertical);
    _notifyListeners();
  }

  @override
  void addListener(VoidCallback listener) {
    _listeners.add(listener);
  }

  @override
  void removeListener(VoidCallback listener) {
    _listeners.remove(listener);
  }

  @override
  Future<bool> popRoute() {
    if (_pages.isEmpty) {
      return SynchronousFuture<bool>(false);
    }
    _pages.removeLast();
    _notifyListeners();
    return SynchronousFuture<bool>(true);
  }

  @override
  Future<void> setNewRoutePath(MyPageConfiguration configuration) {
    _pages.add(configuration);
    _notifyListeners();
    return SynchronousFuture<void>(null);
  }

  @override
  Widget build(BuildContext context) {
    return Navigator(
      restorationScopeId: 'root',
      onDidRemovePage: (Page<dynamic> page) {
        _pages.remove(MyPageConfiguration.fromName(page.name!));
      },
      pages:
          _pages
              .map(
                (MyPageConfiguration page) => switch (page) {
                  MyPageConfiguration.unknown => _MyUnknownPage<void>(),
                  MyPageConfiguration.home => _MyHomePage<void>(routerDelegate: this),
                  MyPageConfiguration.zoom => _ZoomPage<void>(routerDelegate: this),
                  MyPageConfiguration.iOS => _IOSPage<void>(routerDelegate: this),
                  MyPageConfiguration.vertical => _VerticalPage<void>(routerDelegate: this),
                },
              )
              .toList(),
    );
  }
}

class _MyUnknownPage<T> extends MaterialPage<T> {
  _MyUnknownPage()
    : super(
        restorationId: 'unknown-page',
        child: Scaffold(
          appBar: AppBar(title: const Text('404')),
          body: const Center(child: Text('404')),
        ),
      );

  @override
  String get name => MyPageConfiguration.unknown.name;
}

class _MyHomePage<T> extends MaterialPage<T> {
  _MyHomePage({required this.routerDelegate})
    : super(
        restorationId: 'home-page',
        child: _MyPageScaffold(title: 'Home', routerDelegate: routerDelegate),
      );

  final MyRouterDelegate routerDelegate;

  @override
  String get name => MyPageConfiguration.home.name;
}

class _ZoomPage<T> extends MaterialPage<T> {
  _ZoomPage({required this.routerDelegate})
    : super(
        restorationId: 'zoom-page',
        child: _MyPageScaffold(title: 'Zoom Route', routerDelegate: routerDelegate),
      );

  final MyRouterDelegate routerDelegate;

  @override
  String get name => MyPageConfiguration.zoom.name;
}

class _IOSPage<T> extends CupertinoPage<T> {
  _IOSPage({required this.routerDelegate})
    : super(
        restorationId: 'ios-page',
        child: _MyPageScaffold(title: 'Cupertino Route', routerDelegate: routerDelegate),
      );

  final MyRouterDelegate routerDelegate;

  @override
  String get name => MyPageConfiguration.iOS.name;
}

class _VerticalPage<T> extends _VerticalTransitionPage<T> {
  _VerticalPage({required this.routerDelegate})
    : super(
        restorationId: 'vertical-page',
        child: _MyPageScaffold(title: 'Vertical Route', routerDelegate: routerDelegate),
      );

  final MyRouterDelegate routerDelegate;

  @override
  String get name => MyPageConfiguration.vertical.name;
}

class _MyPageScaffold extends StatelessWidget {
  const _MyPageScaffold({required this.title, required this.routerDelegate});

  final String title;

  final MyRouterDelegate routerDelegate;

  @override
  Widget build(BuildContext context) {
    return Scaffold(
      appBar: AppBar(
        backgroundColor: Theme.of(context).colorScheme.inversePrimary,
        title: Text(title),
      ),
      body: Center(
        child: Column(
          mainAxisAlignment: MainAxisAlignment.center,
          children: <Widget>[
            TextButton(
              onPressed: () {
                routerDelegate.onNavigateToVertical();
              },
              child: const Text('Crazy Vertical Transition'),
            ),
            TextButton(
              onPressed: () {
                routerDelegate.onNavigateToZoom();
              },
              child: const Text('Zoom Transition'),
            ),
            TextButton(
              onPressed: () {
                routerDelegate.onNavigateToIOS();
              },
              child: const Text('Cupertino Transition'),
            ),
          ],
        ),
      ),
    );
  }
}

// A Page that applies a _VerticalPageTransition.
class _VerticalTransitionPage<T> extends Page<T> {
  const _VerticalTransitionPage({
    required this.child,
    this.maintainState = true,
    this.fullscreenDialog = false,
    this.allowSnapshotting = true,
    super.key,
    super.canPop,
    super.onPopInvoked,
    super.name,
    super.arguments,
    super.restorationId,
  });

  final Widget child;

  final bool maintainState;

  final bool fullscreenDialog;

  final bool allowSnapshotting;

  @override
  Route<T> createRoute(BuildContext context) {
    return _PageBasedVerticalPageRoute<T>(page: this);
  }
}

class _PageBasedVerticalPageRoute<T> extends PageRoute<T> {
  _PageBasedVerticalPageRoute({required _VerticalTransitionPage<T> page, super.allowSnapshotting})
    : super(settings: page);

  _VerticalTransitionPage<T> get _page => settings as _VerticalTransitionPage<T>;

  @override
  bool get maintainState => _page.maintainState;

  @override
  bool get fullscreenDialog => _page.fullscreenDialog;

  @override
  String get debugLabel => '${super.debugLabel}(${_page.name})';

  @override
  DelegatedTransitionBuilder? get delegatedTransition =>
      _VerticalPageTransition._delegatedTransitionBuilder;

  @override
  Color? get barrierColor => const Color(0x00000000);

  @override
  bool get barrierDismissible => false;

  @override
  String? get barrierLabel => 'Should be no visible barrier...';

  @override
  bool get opaque => false;

  @override
  Duration get transitionDuration => const Duration(milliseconds: 2000);

  @override
  Widget buildPage(
    BuildContext context,
    Animation<double> animation,
    Animation<double> secondaryAnimation,
  ) {
    return _page.child;
  }

  @override
  Widget buildTransitions(
    BuildContext context,
    Animation<double> animation,
    Animation<double> secondaryAnimation,
    Widget child,
  ) {
    return _VerticalPageTransition(
      primaryRouteAnimation: animation,
      secondaryRouteAnimation: secondaryAnimation,
      child: child,
    );
  }
}

// A page transition that slides off the screen vertically, and uses
// delegatedTransition to ensure that the outgoing route slides with it.
class _VerticalPageTransition extends StatelessWidget {
  _VerticalPageTransition({
    required Animation<double> primaryRouteAnimation,
    required this.secondaryRouteAnimation,
    required this.child,
  }) : _primaryPositionAnimation = CurvedAnimation(
         parent: primaryRouteAnimation,
         curve: _curve,
         reverseCurve: _curve,
       ).drive(_kBottomUpTween),
       _secondaryPositionAnimation = CurvedAnimation(
         parent: secondaryRouteAnimation,
         curve: _curve,
         reverseCurve: _curve,
       ).drive(_kTopDownTween);

  final Animation<Offset> _primaryPositionAnimation;

  final Animation<Offset> _secondaryPositionAnimation;

  final Animation<double> secondaryRouteAnimation;

  final Widget child;

  static const Curve _curve = Curves.decelerate;

  static final Animatable<Offset> _kBottomUpTween = Tween<Offset>(
    begin: const Offset(0.0, 1.0),
    end: Offset.zero,
  );

  static final Animatable<Offset> _kTopDownTween = Tween<Offset>(
    begin: Offset.zero,
    end: const Offset(0.0, -1.0),
  );

  // When the _VerticalTransitionPageRoute animates onto or off of the navigation
  // stack, this transition is given to the route below it so that they animate in
  // sync.
  static Widget _delegatedTransitionBuilder(
    BuildContext context,
    Animation<double> animation,
    Animation<double> secondaryAnimation,
    bool allowSnapshotting,
    Widget? child,
  ) {
    final Animatable<Offset> tween = Tween<Offset>(
      begin: Offset.zero,
      end: const Offset(0.0, -1.0),
    ).chain(CurveTween(curve: _curve));

    return SlideTransition(position: secondaryAnimation.drive(tween), child: child);
  }

  @override
  Widget build(BuildContext context) {
    assert(debugCheckHasDirectionality(context));
    final TextDirection textDirection = Directionality.of(context);
    return SlideTransition(
      position: _secondaryPositionAnimation,
      textDirection: textDirection,
      transformHitTests: false,
      child: SlideTransition(
        position: _primaryPositionAnimation,
        textDirection: textDirection,
        child: ClipRRect(
          borderRadius: const BorderRadius.vertical(top: Radius.circular(12)),
          child: child,
        ),
      ),
    );
  }
}

enum MyPageConfiguration {
  home(uriString: '/'),
  zoom(uriString: '/zoom'),
  iOS(uriString: '/iOS'),
  vertical(uriString: '/vertical'),
  unknown(uriString: '/404');

  const MyPageConfiguration({required this.uriString});

  final String uriString;

  static MyPageConfiguration fromName(String testName) {
    return values.firstWhere((MyPageConfiguration page) => page.name == testName);
  }

  Uri get uri => Uri.parse(uriString);
}
