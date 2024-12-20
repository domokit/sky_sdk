// Copyright 2014 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

/// @docImport 'tab_scaffold.dart';
library;

import 'package:flutter/widgets.dart';

import 'app.dart' show CupertinoApp;
import 'route.dart';

/// A single tab view with its own [Navigator] state and history.
///
/// A typical tab view is used as the content of each tab in a
/// [CupertinoTabScaffold] where multiple tabs with parallel navigation states
/// and history can co-exist.
///
/// [CupertinoTabView] configures the top-level [Navigator] to search for routes
/// in the following order:
///
///  1. For the `/` route, the [builder] property, if non-null, is used.
///
///  2. Otherwise, the [routes] table is used, if it has an entry for the route,
///     including `/` if [builder] is not specified.
///
///  3. Otherwise, [onGenerateRoute] is called, if provided. It should return a
///     non-null value for any _valid_ route not handled by [builder] and [routes].
///
///  4. Finally if all else fails [onUnknownRoute] is called.
///
/// These navigation properties are not shared with any sibling [CupertinoTabView]
/// nor any ancestor or descendant [Navigator] instances.
///
/// To push a route above this [CupertinoTabView] instead of inside it (such
/// as when showing a dialog on top of all tabs), use
/// `Navigator.of(rootNavigator: true)`.
///
/// See also:
///
///  * [CupertinoTabScaffold], a typical host that supports switching between tabs.
///  * [CupertinoPageRoute], a typical modal page route pushed onto the
///    [CupertinoTabView]'s [Navigator].
class CupertinoTabView extends StatefulWidget {
  /// Creates the content area for a tab in a [CupertinoTabScaffold].
  const CupertinoTabView({
    super.key,
    this.builder,
    this.navigatorKey,
    this.defaultTitle,
    this.routes,
    this.onGenerateRoute,
    this.onUnknownRoute,
    this.navigatorObservers = const <NavigatorObserver>[],
    this.restorationScopeId,
  });

  /// The widget builder for the default route of the tab view
  /// ([Navigator.defaultRouteName], which is `/`).
  ///
  /// If a [builder] is specified, then [routes] must not include an entry for `/`,
  /// as [builder] takes its place.
  ///
  /// Rebuilding a [CupertinoTabView] with a different [builder] will not clear
  /// its current navigation stack or update its descendant. Instead, trigger a
  /// rebuild from a descendant in its subtree. This can be done via methods such
  /// as:
  ///
  ///  * Calling [State.setState] on a descendant [StatefulWidget]'s [State]
  ///  * Modifying an [InheritedWidget] that a descendant registered itself
  ///    as a dependent to.
  final WidgetBuilder? builder;

  /// A key to use when building this widget's [Navigator].
  ///
  /// If a [navigatorKey] is specified, the [Navigator] can be directly
  /// manipulated without first obtaining it from a [BuildContext] via
  /// [Navigator.of]: from the [navigatorKey], use the [GlobalKey.currentState]
  /// getter.
  ///
  /// If this is changed, a new [Navigator] will be created, losing all the
  /// tab's state in the process; in that case, the [navigatorObservers]
  /// must also be changed, since the previous observers will be attached to the
  /// previous navigator.
  final GlobalKey<NavigatorState>? navigatorKey;

  /// The title of the default route.
  final String? defaultTitle;

  /// This tab view's routing table.
  ///
  /// When a named route is pushed with [Navigator.pushNamed] inside this tab view,
  /// the route name is looked up in this map. If the name is present,
  /// the associated [WidgetBuilder] is used to construct a
  /// [CupertinoPageRoute] that performs an appropriate transition to the new
  /// route.
  ///
  /// If the tab view only has one page, then you can specify it using [builder] instead.
  ///
  /// If [builder] is specified, then it implies an entry in this table for the
  /// [Navigator.defaultRouteName] route (`/`), and it is an error to
  /// redundantly provide such a route in the [routes] table.
  ///
  /// If a route is requested that is not specified in this table (or by
  /// [builder]), then the [onGenerateRoute] callback is called to build the page
  /// instead.
  ///
  /// This routing table is not shared with any routing tables of ancestor or
  /// descendant [Navigator]s.
  final Map<String, WidgetBuilder>? routes;

  /// The route generator callback used when the tab view is navigated to a named route.
  ///
  /// This is used if [routes] does not contain the requested route.
  final RouteFactory? onGenerateRoute;

  /// Called when [onGenerateRoute] also fails to generate a route.
  ///
  /// This callback is typically used for error handling. For example, this
  /// callback might always generate a "not found" page that describes the route
  /// that wasn't found.
  ///
  /// The default implementation pushes a route that displays an ugly error
  /// message.
  final RouteFactory? onUnknownRoute;

  /// The list of observers for the [Navigator] created in this tab view.
  ///
  /// This list of observers is not shared with ancestor or descendant [Navigator]s.
  final List<NavigatorObserver> navigatorObservers;

  /// Restoration ID to save and restore the state of the [Navigator] built by
  /// this [CupertinoTabView].
  ///
  /// {@macro flutter.widgets.navigator.restorationScopeId}
  final String? restorationScopeId;

  @override
  State<CupertinoTabView> createState() => _CupertinoTabViewState();
}

class _CupertinoTabViewState extends State<CupertinoTabView> {
  late HeroController _heroController;
  late List<NavigatorObserver> _navigatorObservers;

  @override
  void initState() {
    super.initState();
    _heroController = CupertinoApp.createCupertinoHeroController();
    _updateObservers();
  }

  @override
  void didUpdateWidget(CupertinoTabView oldWidget) {
    super.didUpdateWidget(oldWidget);
    if (widget.navigatorKey != oldWidget.navigatorKey ||
        widget.navigatorObservers != oldWidget.navigatorObservers) {
      _updateObservers();
    }
  }

  @override
  void dispose() {
    _heroController.dispose();
    super.dispose();
  }

  void _updateObservers() {
    _navigatorObservers = List<NavigatorObserver>.of(widget.navigatorObservers)
      ..add(_heroController);
  }

  GlobalKey<NavigatorState>? _ownedNavigatorKey;
  GlobalKey<NavigatorState> get _navigatorKey {
    if (widget.navigatorKey != null) {
      return widget.navigatorKey!;
    }
    _ownedNavigatorKey ??= GlobalKey<NavigatorState>();
    return _ownedNavigatorKey!;
  }

  // Whether this tab is currently the active tab.
  bool get _isActive => TickerMode.of(context);

  @override
  Widget build(BuildContext context) {
    final Widget child = Navigator(
      key: _navigatorKey,
      onGenerateRoute: _onGenerateRoute,
      onUnknownRoute: _onUnknownRoute,
      observers: _navigatorObservers,
      restorationScopeId: widget.restorationScopeId,
    );

    // Handle system back gestures only if the tab is currently active.
    return NavigatorPopHandler(
      enabled: _isActive,
      onPop: () {
        if (!_isActive) {
          return;
        }
        _navigatorKey.currentState!.maybePop();
      },
      child: child,
    );
  }

  Route<dynamic>? _onGenerateRoute(RouteSettings settings) {
    final String? name = settings.name;
    final WidgetBuilder? routeBuilder;
    String? title;
    if (name == Navigator.defaultRouteName && widget.builder != null) {
      routeBuilder = widget.builder;
      title = widget.defaultTitle;
    } else {
      routeBuilder = widget.routes?[name];
    }
    if (routeBuilder != null) {
      return CupertinoPageRoute<dynamic>(builder: routeBuilder, title: title, settings: settings);
    }
    return widget.onGenerateRoute?.call(settings);
  }

  Route<dynamic>? _onUnknownRoute(RouteSettings settings) {
    assert(() {
      if (widget.onUnknownRoute == null) {
        throw FlutterError(
          'Could not find a generator for route $settings in the $runtimeType.\n'
          'Generators for routes are searched for in the following order:\n'
          ' 1. For the "/" route, the "builder" property, if non-null, is used.\n'
          ' 2. Otherwise, the "routes" table is used, if it has an entry for '
          'the route.\n'
          ' 3. Otherwise, onGenerateRoute is called. It should return a '
          'non-null value for any valid route not handled by "builder" and "routes".\n'
          ' 4. Finally if all else fails onUnknownRoute is called.\n'
          'Unfortunately, onUnknownRoute was not set.',
        );
      }
      return true;
    }());
    final Route<dynamic>? result = widget.onUnknownRoute!(settings);
    assert(() {
      if (result == null) {
        throw FlutterError(
          'The onUnknownRoute callback returned null.\n'
          'When the $runtimeType requested the route $settings from its '
          'onUnknownRoute callback, the callback returned null. Such callbacks '
          'must never return null.',
        );
      }
      return true;
    }());
    return result;
  }
}
