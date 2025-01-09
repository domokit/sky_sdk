// Copyright 2014 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

/// @docImport 'package:flutter/material.dart';
/// @docImport 'package:flutter/services.dart';
///
/// @docImport 'page_scaffold.dart';
/// @docImport 'tab_view.dart';
library;

import 'package:flutter/gestures.dart';
import 'package:flutter/widgets.dart';

import 'button.dart';
import 'colors.dart';
import 'icons.dart';
import 'interface_level.dart';
import 'localizations.dart';
import 'route.dart';
import 'scrollbar.dart';
import 'theme.dart';

/// An application that uses Cupertino design.
///
/// A convenience widget that wraps a number of widgets that are commonly
/// required for an iOS-design targeting application. It builds upon a
/// [WidgetsApp] by iOS specific defaulting such as fonts and scrolling
/// physics.
///
/// The [CupertinoApp] configures the top-level [Navigator] to search for routes
/// in the following order:
///
///  1. For the `/` route, the [home] property, if non-null, is used.
///
///  2. Otherwise, the [routes] table is used, if it has an entry for the route.
///
///  3. Otherwise, [onGenerateRoute] is called, if provided. It should return a
///     non-null value for any _valid_ route not handled by [home] and [routes].
///
///  4. Finally if all else fails [onUnknownRoute] is called.
///
/// If [home], [routes], [onGenerateRoute], and [onUnknownRoute] are all null,
/// and [builder] is not null, then no [Navigator] is created.
///
/// This widget also configures the observer of the top-level [Navigator] (if
/// any) to perform [Hero] animations.
///
/// The [CupertinoApp] widget isn't a required ancestor for other Cupertino
/// widgets, but many Cupertino widgets could depend on the [CupertinoTheme]
/// widget, which the [CupertinoApp] composes. If you use Material widgets, a
/// [MaterialApp] also creates the needed dependencies for Cupertino widgets.
///
/// {@template flutter.cupertino.CupertinoApp.defaultSelectionStyle}
/// The [CupertinoApp] automatically creates a [DefaultSelectionStyle] with
/// selectionColor sets to [CupertinoThemeData.primaryColor] with 0.2 opacity
/// and cursorColor sets to [CupertinoThemeData.primaryColor].
/// {@endtemplate}
///
/// Use this widget with caution on Android since it may produce behaviors
/// Android users are not expecting such as:
///
///  * Pages will be dismissible via a back swipe.
///  * Scrolling past extremities will trigger iOS-style spring overscrolls.
///  * The San Francisco font family is unavailable on Android and can result
///    in undefined font behavior.
///
/// {@tool snippet}
/// This example shows how to create a [CupertinoApp] that disables the "debug"
/// banner with a [home] route that will be displayed when the app is launched.
///
/// ![The CupertinoApp displays a CupertinoPageScaffold](https://flutter.github.io/assets-for-api-docs/assets/cupertino/basic_cupertino_app.png)
///
/// ```dart
/// const CupertinoApp(
///   home: CupertinoPageScaffold(
///     navigationBar: CupertinoNavigationBar(
///       middle: Text('Home'),
///     ),
///     child: Center(child: Icon(CupertinoIcons.share)),
///   ),
///   debugShowCheckedModeBanner: false,
/// )
/// ```
/// {@end-tool}
///
/// {@tool snippet}
/// This example shows how to create a [CupertinoApp] that uses the [routes]
/// `Map` to define the "home" route and an "about" route.
///
/// ```dart
/// CupertinoApp(
///   routes: <String, WidgetBuilder>{
///     '/': (BuildContext context) {
///       return const CupertinoPageScaffold(
///         navigationBar: CupertinoNavigationBar(
///           middle: Text('Home Route'),
///         ),
///         child: Center(child: Icon(CupertinoIcons.share)),
///       );
///     },
///     '/about': (BuildContext context) {
///       return const CupertinoPageScaffold(
///         navigationBar: CupertinoNavigationBar(
///           middle: Text('About Route'),
///         ),
///         child: Center(child: Icon(CupertinoIcons.share)),
///       );
///     }
///   },
/// )
/// ```
/// {@end-tool}
///
/// {@tool snippet}
/// This example shows how to create a [CupertinoApp] that defines a [theme] that
/// will be used for Cupertino widgets in the app.
///
/// ![The CupertinoApp displays a CupertinoPageScaffold with orange-colored icons](https://flutter.github.io/assets-for-api-docs/assets/cupertino/theme_cupertino_app.png)
///
/// ```dart
/// const CupertinoApp(
///   theme: CupertinoThemeData(
///     brightness: Brightness.dark,
///     primaryColor: CupertinoColors.systemOrange,
///   ),
///   home: CupertinoPageScaffold(
///     navigationBar: CupertinoNavigationBar(
///       middle: Text('CupertinoApp Theme'),
///     ),
///     child: Center(child: Icon(CupertinoIcons.share)),
///   ),
/// )
/// ```
/// {@end-tool}
///
/// See also:
///
///  * [CupertinoPageScaffold], which provides a standard page layout default
///    with nav bars.
///  * [Navigator], which is used to manage the app's stack of pages.
///  * [CupertinoPageRoute], which defines an app page that transitions in an
///    iOS-specific way.
///  * [WidgetsApp], which defines the basic app elements but does not depend
///    on the Cupertino library.
class CupertinoApp extends StatefulWidget {
  /// Creates a CupertinoApp.
  ///
  /// At least one of [home], [routes], [onGenerateRoute], or [builder] must be
  /// non-null. If only [routes] is given, it must include an entry for the
  /// [Navigator.defaultRouteName] (`/`), since that is the route used when the
  /// application is launched with an intent that specifies an otherwise
  /// unsupported route.
  ///
  /// This class creates an instance of [WidgetsApp].
  const CupertinoApp({
    super.key,
    this.navigatorKey,
    this.home,
    this.theme,
    Map<String, Widget Function(BuildContext)> this.routes = const <String, WidgetBuilder>{},
    this.initialRoute,
    this.onGenerateRoute,
    this.onGenerateInitialRoutes,
    this.onUnknownRoute,
    this.onNavigationNotification,
    List<NavigatorObserver> this.navigatorObservers = const <NavigatorObserver>[],
    this.builder,
    this.title,
    this.onGenerateTitle,
    this.color,
    this.locale,
    this.localizationsDelegates,
    this.localeListResolutionCallback,
    this.localeResolutionCallback,
    this.supportedLocales = const <Locale>[Locale('en', 'US')],
    this.showPerformanceOverlay = false,
    this.checkerboardRasterCacheImages = false,
    this.checkerboardOffscreenLayers = false,
    this.showSemanticsDebugger = false,
    this.debugShowCheckedModeBanner = true,
    this.shortcuts,
    this.actions,
    this.restorationScopeId,
    this.scrollBehavior,
    @Deprecated(
      'Remove this parameter as it is now ignored. '
      'CupertinoApp never introduces its own MediaQuery; the View widget takes care of that. '
      'This feature was deprecated after v3.7.0-29.0.pre.',
    )
    this.useInheritedMediaQuery = false,
  }) : routeInformationProvider = null,
       routeInformationParser = null,
       routerDelegate = null,
       backButtonDispatcher = null,
       routerConfig = null;

  /// Creates a [CupertinoApp] that uses the [Router] instead of a [Navigator].
  ///
  /// {@macro flutter.widgets.WidgetsApp.router}
  const CupertinoApp.router({
    super.key,
    this.routeInformationProvider,
    this.routeInformationParser,
    this.routerDelegate,
    this.backButtonDispatcher,
    this.routerConfig,
    this.theme,
    this.builder,
    this.title,
    this.onGenerateTitle,
    this.onNavigationNotification,
    this.color,
    this.locale,
    this.localizationsDelegates,
    this.localeListResolutionCallback,
    this.localeResolutionCallback,
    this.supportedLocales = const <Locale>[Locale('en', 'US')],
    this.showPerformanceOverlay = false,
    this.checkerboardRasterCacheImages = false,
    this.checkerboardOffscreenLayers = false,
    this.showSemanticsDebugger = false,
    this.debugShowCheckedModeBanner = true,
    this.shortcuts,
    this.actions,
    this.restorationScopeId,
    this.scrollBehavior,
    @Deprecated(
      'Remove this parameter as it is now ignored. '
      'CupertinoApp never introduces its own MediaQuery; the View widget takes care of that. '
      'This feature was deprecated after v3.7.0-29.0.pre.',
    )
    this.useInheritedMediaQuery = false,
  }) : assert(routerDelegate != null || routerConfig != null),
       navigatorObservers = null,
       navigatorKey = null,
       onGenerateRoute = null,
       home = null,
       onGenerateInitialRoutes = null,
       onUnknownRoute = null,
       routes = null,
       initialRoute = null;

  /// {@macro flutter.widgets.widgetsApp.navigatorKey}
  final GlobalKey<NavigatorState>? navigatorKey;

  /// {@macro flutter.widgets.widgetsApp.home}
  final Widget? home;

  /// The top-level [CupertinoTheme] styling.
  ///
  /// A null [theme] or unspecified [theme] attributes will default to iOS
  /// system values.
  final CupertinoThemeData? theme;

  /// The application's top-level routing table.
  ///
  /// When a named route is pushed with [Navigator.pushNamed], the route name is
  /// looked up in this map. If the name is present, the associated
  /// [WidgetBuilder] is used to construct a [CupertinoPageRoute] that
  /// performs an appropriate transition, including [Hero] animations, to the
  /// new route.
  ///
  /// {@macro flutter.widgets.widgetsApp.routes}
  final Map<String, WidgetBuilder>? routes;

  /// {@macro flutter.widgets.widgetsApp.initialRoute}
  final String? initialRoute;

  /// {@macro flutter.widgets.widgetsApp.onGenerateRoute}
  final RouteFactory? onGenerateRoute;

  /// {@macro flutter.widgets.widgetsApp.onGenerateInitialRoutes}
  final InitialRouteListFactory? onGenerateInitialRoutes;

  /// {@macro flutter.widgets.widgetsApp.onUnknownRoute}
  final RouteFactory? onUnknownRoute;

  /// {@macro flutter.widgets.widgetsApp.onNavigationNotification}
  final NotificationListenerCallback<NavigationNotification>? onNavigationNotification;

  /// {@macro flutter.widgets.widgetsApp.navigatorObservers}
  final List<NavigatorObserver>? navigatorObservers;

  /// {@macro flutter.widgets.widgetsApp.routeInformationProvider}
  final RouteInformationProvider? routeInformationProvider;

  /// {@macro flutter.widgets.widgetsApp.routeInformationParser}
  final RouteInformationParser<Object>? routeInformationParser;

  /// {@macro flutter.widgets.widgetsApp.routerDelegate}
  final RouterDelegate<Object>? routerDelegate;

  /// {@macro flutter.widgets.widgetsApp.backButtonDispatcher}
  final BackButtonDispatcher? backButtonDispatcher;

  /// {@macro flutter.widgets.widgetsApp.routerConfig}
  final RouterConfig<Object>? routerConfig;

  /// {@macro flutter.widgets.widgetsApp.builder}
  final TransitionBuilder? builder;

  /// {@macro flutter.widgets.widgetsApp.title}
  ///
  /// This value is passed unmodified to [WidgetsApp.title].
  final String? title;

  /// {@macro flutter.widgets.widgetsApp.onGenerateTitle}
  ///
  /// This value is passed unmodified to [WidgetsApp.onGenerateTitle].
  final GenerateAppTitle? onGenerateTitle;

  /// {@macro flutter.widgets.widgetsApp.color}
  final Color? color;

  /// {@macro flutter.widgets.widgetsApp.locale}
  final Locale? locale;

  /// {@macro flutter.widgets.widgetsApp.localizationsDelegates}
  final Iterable<LocalizationsDelegate<dynamic>>? localizationsDelegates;

  /// {@macro flutter.widgets.widgetsApp.localeListResolutionCallback}
  ///
  /// This callback is passed along to the [WidgetsApp] built by this widget.
  final LocaleListResolutionCallback? localeListResolutionCallback;

  /// {@macro flutter.widgets.LocaleResolutionCallback}
  ///
  /// This callback is passed along to the [WidgetsApp] built by this widget.
  final LocaleResolutionCallback? localeResolutionCallback;

  /// {@macro flutter.widgets.widgetsApp.supportedLocales}
  ///
  /// It is passed along unmodified to the [WidgetsApp] built by this widget.
  final Iterable<Locale> supportedLocales;

  /// Turns on a performance overlay.
  ///
  /// See also:
  ///
  ///  * <https://flutter.dev/to/performance-overlay>
  final bool showPerformanceOverlay;

  /// Turns on checkerboarding of raster cache images.
  final bool checkerboardRasterCacheImages;

  /// Turns on checkerboarding of layers rendered to offscreen bitmaps.
  final bool checkerboardOffscreenLayers;

  /// Turns on an overlay that shows the accessibility information
  /// reported by the framework.
  final bool showSemanticsDebugger;

  /// {@macro flutter.widgets.widgetsApp.debugShowCheckedModeBanner}
  final bool debugShowCheckedModeBanner;

  /// {@macro flutter.widgets.widgetsApp.shortcuts}
  /// {@tool snippet}
  /// This example shows how to add a single shortcut for
  /// [LogicalKeyboardKey.select] to the default shortcuts without needing to
  /// add your own [Shortcuts] widget.
  ///
  /// Alternatively, you could insert a [Shortcuts] widget with just the mapping
  /// you want to add between the [WidgetsApp] and its child and get the same
  /// effect.
  ///
  /// ```dart
  /// Widget build(BuildContext context) {
  ///   return WidgetsApp(
  ///     shortcuts: <ShortcutActivator, Intent>{
  ///       ... WidgetsApp.defaultShortcuts,
  ///       const SingleActivator(LogicalKeyboardKey.select): const ActivateIntent(),
  ///     },
  ///     color: const Color(0xFFFF0000),
  ///     builder: (BuildContext context, Widget? child) {
  ///       return const Placeholder();
  ///     },
  ///   );
  /// }
  /// ```
  /// {@end-tool}
  /// {@macro flutter.widgets.widgetsApp.shortcuts.seeAlso}
  final Map<ShortcutActivator, Intent>? shortcuts;

  /// {@macro flutter.widgets.widgetsApp.actions}
  /// {@tool snippet}
  /// This example shows how to add a single action handling an
  /// [ActivateAction] to the default actions without needing to
  /// add your own [Actions] widget.
  ///
  /// Alternatively, you could insert a [Actions] widget with just the mapping
  /// you want to add between the [WidgetsApp] and its child and get the same
  /// effect.
  ///
  /// ```dart
  /// Widget build(BuildContext context) {
  ///   return WidgetsApp(
  ///     actions: <Type, Action<Intent>>{
  ///       ... WidgetsApp.defaultActions,
  ///       ActivateAction: CallbackAction<Intent>(
  ///         onInvoke: (Intent intent) {
  ///           // Do something here...
  ///           return null;
  ///         },
  ///       ),
  ///     },
  ///     color: const Color(0xFFFF0000),
  ///     builder: (BuildContext context, Widget? child) {
  ///       return const Placeholder();
  ///     },
  ///   );
  /// }
  /// ```
  /// {@end-tool}
  /// {@macro flutter.widgets.widgetsApp.actions.seeAlso}
  final Map<Type, Action<Intent>>? actions;

  /// {@macro flutter.widgets.widgetsApp.restorationScopeId}
  final String? restorationScopeId;

  /// {@macro flutter.material.materialApp.scrollBehavior}
  ///
  /// When null, defaults to [CupertinoScrollBehavior].
  ///
  /// See also:
  ///
  ///  * [ScrollConfiguration], which controls how [Scrollable] widgets behave
  ///    in a subtree.
  final ScrollBehavior? scrollBehavior;

  /// {@macro flutter.widgets.widgetsApp.useInheritedMediaQuery}
  @Deprecated(
    'This setting is now ignored. '
    'CupertinoApp never introduces its own MediaQuery; the View widget takes care of that. '
    'This feature was deprecated after v3.7.0-29.0.pre.',
  )
  final bool useInheritedMediaQuery;

  @override
  State<CupertinoApp> createState() => _CupertinoAppState();

  /// The [HeroController] used for Cupertino page transitions.
  ///
  /// Used by [CupertinoTabView] and [CupertinoApp].
  static HeroController createCupertinoHeroController() => HeroController(); // Linear tweening.
}

/// Describes how [Scrollable] widgets behave for [CupertinoApp]s.
///
/// {@macro flutter.widgets.scrollBehavior}
///
/// Setting a [CupertinoScrollBehavior] will result in descendant [Scrollable] widgets
/// using [BouncingScrollPhysics] by default. No [GlowingOverscrollIndicator] is
/// applied when using a [CupertinoScrollBehavior] either, regardless of platform.
/// When executing on desktop platforms, a [CupertinoScrollbar] is applied to the child.
///
/// See also:
///
///  * [ScrollBehavior], the default scrolling behavior extended by this class.
class CupertinoScrollBehavior extends ScrollBehavior {
  /// Creates a CupertinoScrollBehavior that uses [BouncingScrollPhysics] and
  /// adds [CupertinoScrollbar]s on desktop platforms.
  const CupertinoScrollBehavior();

  @override
  Widget buildScrollbar(BuildContext context, Widget child, ScrollableDetails details) {
    // When modifying this function, consider modifying the implementation in
    // the base class as well.
    switch (getPlatform(context)) {
      case TargetPlatform.linux:
      case TargetPlatform.macOS:
      case TargetPlatform.windows:
        assert(details.controller != null);
        return CupertinoScrollbar(controller: details.controller, child: child);
      case TargetPlatform.android:
      case TargetPlatform.fuchsia:
      case TargetPlatform.iOS:
        return child;
    }
  }

  @override
  Widget buildOverscrollIndicator(BuildContext context, Widget child, ScrollableDetails details) {
    // No overscroll indicator.
    // When modifying this function, consider modifying the implementation in
    // the base class as well.
    return child;
  }

  @override
  ScrollPhysics getScrollPhysics(BuildContext context) {
    // When modifying this function, consider modifying the implementation in
    // the base class ScrollBehavior as well.
    if (getPlatform(context) == TargetPlatform.macOS) {
      return const BouncingScrollPhysics(decelerationRate: ScrollDecelerationRate.fast);
    }
    return const BouncingScrollPhysics();
  }

  @override
  MultitouchDragStrategy getMultitouchDragStrategy(BuildContext context) =>
      MultitouchDragStrategy.averageBoundaryPointers;
}

class _CupertinoAppState extends State<CupertinoApp> {
  late HeroController _heroController;
  bool get _usesRouter => widget.routerDelegate != null || widget.routerConfig != null;

  @override
  void initState() {
    super.initState();
    _heroController = CupertinoApp.createCupertinoHeroController();
  }

  @override
  void dispose() {
    _heroController.dispose();
    super.dispose();
  }

  // Combine the default localization for Cupertino with the ones contributed
  // by the localizationsDelegates parameter, if any. Only the first delegate
  // of a particular LocalizationsDelegate.type is loaded so the
  // localizationsDelegate parameter can be used to override
  // _CupertinoLocalizationsDelegate.
  Iterable<LocalizationsDelegate<dynamic>> get _localizationsDelegates {
    return <LocalizationsDelegate<dynamic>>[
      if (widget.localizationsDelegates != null) ...widget.localizationsDelegates!,
      DefaultCupertinoLocalizations.delegate,
    ];
  }

  Widget _exitWidgetSelectionButtonBuilder(
    BuildContext context, {
    required VoidCallback onPressed,
    required GlobalKey key,
  }) {
    return CupertinoButton(
      key: key,
      color: _widgetSelectionButtonsBackgroundColor(context),
      padding: EdgeInsets.zero,
      onPressed: onPressed,
      child: Icon(
        CupertinoIcons.xmark,
        size: 28.0,
        color: _widgetSelectionButtonsForegroundColor(context),
        semanticLabel: 'Exit Select Widget mode.',
      ),
    );
  }

  Widget _moveExitWidgetSelectionButtonBuilder(
    BuildContext context, {
    required VoidCallback onPressed,
    bool isLeftAligned = true,
  }) {
    return CupertinoButton(
      onPressed: onPressed,
      padding: EdgeInsets.zero,
      child: Icon(
        isLeftAligned ? CupertinoIcons.arrow_right : CupertinoIcons.arrow_left,
        size: 32.0,
        color: _widgetSelectionButtonsBackgroundColor(context),
        semanticLabel:
            'Move "Exit Select Widget mode" button to the ${isLeftAligned ? 'right' : 'left'}.',
      ),
    );
  }

  Color _widgetSelectionButtonsForegroundColor(BuildContext context) {
    return CupertinoTheme.of(context).primaryContrastingColor;
  }

  Color _widgetSelectionButtonsBackgroundColor(BuildContext context) {
    return CupertinoTheme.of(context).primaryColor;
  }

  WidgetsApp _buildWidgetApp(BuildContext context) {
    final CupertinoThemeData effectiveThemeData = CupertinoTheme.of(context);
    final Color color = CupertinoDynamicColor.resolve(
      widget.color ?? effectiveThemeData.primaryColor,
      context,
    );

    if (_usesRouter) {
      return WidgetsApp.router(
        key: GlobalObjectKey(this),
        routeInformationProvider: widget.routeInformationProvider,
        routeInformationParser: widget.routeInformationParser,
        routerDelegate: widget.routerDelegate,
        routerConfig: widget.routerConfig,
        backButtonDispatcher: widget.backButtonDispatcher,
        onNavigationNotification: widget.onNavigationNotification,
        builder: widget.builder,
        title: widget.title,
        onGenerateTitle: widget.onGenerateTitle,
        textStyle: effectiveThemeData.textTheme.textStyle,
        color: color,
        locale: widget.locale,
        localizationsDelegates: _localizationsDelegates,
        localeResolutionCallback: widget.localeResolutionCallback,
        localeListResolutionCallback: widget.localeListResolutionCallback,
        supportedLocales: widget.supportedLocales,
        showPerformanceOverlay: widget.showPerformanceOverlay,
        showSemanticsDebugger: widget.showSemanticsDebugger,
        debugShowCheckedModeBanner: widget.debugShowCheckedModeBanner,
        exitWidgetSelectionButtonBuilder: _exitWidgetSelectionButtonBuilder,
        moveExitWidgetSelectionButtonBuilder: _moveExitWidgetSelectionButtonBuilder,
        shortcuts: widget.shortcuts,
        actions: widget.actions,
        restorationScopeId: widget.restorationScopeId,
      );
    }

    return WidgetsApp(
      key: GlobalObjectKey(this),
      navigatorKey: widget.navigatorKey,
      navigatorObservers: widget.navigatorObservers!,
      pageRouteBuilder: <T>(RouteSettings settings, WidgetBuilder builder) {
        return CupertinoPageRoute<T>(settings: settings, builder: builder);
      },
      home: widget.home,
      routes: widget.routes!,
      initialRoute: widget.initialRoute,
      onGenerateRoute: widget.onGenerateRoute,
      onGenerateInitialRoutes: widget.onGenerateInitialRoutes,
      onUnknownRoute: widget.onUnknownRoute,
      onNavigationNotification: widget.onNavigationNotification,
      builder: widget.builder,
      title: widget.title,
      onGenerateTitle: widget.onGenerateTitle,
      textStyle: effectiveThemeData.textTheme.textStyle,
      color: color,
      locale: widget.locale,
      localizationsDelegates: _localizationsDelegates,
      localeResolutionCallback: widget.localeResolutionCallback,
      localeListResolutionCallback: widget.localeListResolutionCallback,
      supportedLocales: widget.supportedLocales,
      showPerformanceOverlay: widget.showPerformanceOverlay,
      showSemanticsDebugger: widget.showSemanticsDebugger,
      debugShowCheckedModeBanner: widget.debugShowCheckedModeBanner,
      exitWidgetSelectionButtonBuilder: _exitWidgetSelectionButtonBuilder,
      moveExitWidgetSelectionButtonBuilder: _moveExitWidgetSelectionButtonBuilder,
      shortcuts: widget.shortcuts,
      actions: widget.actions,
      restorationScopeId: widget.restorationScopeId,
    );
  }

  @override
  Widget build(BuildContext context) {
    final CupertinoThemeData effectiveThemeData = (widget.theme ?? const CupertinoThemeData())
        .resolveFrom(context);

    return ScrollConfiguration(
      behavior: widget.scrollBehavior ?? const CupertinoScrollBehavior(),
      child: CupertinoUserInterfaceLevel(
        data: CupertinoUserInterfaceLevelData.base,
        child: CupertinoTheme(
          data: effectiveThemeData,
          child: DefaultSelectionStyle(
            selectionColor: effectiveThemeData.primaryColor.withOpacity(0.2),
            cursorColor: effectiveThemeData.primaryColor,
            child: HeroControllerScope(
              controller: _heroController,
              child: Builder(builder: _buildWidgetApp),
            ),
          ),
        ),
      ),
    );
  }
}
