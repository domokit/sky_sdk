// Copyright 2014 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

/// @docImport 'package:flutter/cupertino.dart';
/// @docImport 'package:flutter/material.dart';
///
/// @docImport 'app.dart';
/// @docImport 'scroll_view.dart';
library;

import 'dart:async';
import 'dart:collection';

import 'package:collection/collection.dart';
import 'package:flutter/foundation.dart';
import 'package:flutter/scheduler.dart';
import 'package:flutter/services.dart';

import 'basic.dart';
import 'binding.dart';
import 'framework.dart';
import 'navigator.dart';
import 'restoration.dart';
import 'restoration_properties.dart';

/// A piece of routing information.
///
/// The route information consists of a location string of the application and
/// a state object that configures the application in that location.
///
/// This information flows two ways, from the [RouteInformationProvider] to the
/// [Router] or from the [Router] to [RouteInformationProvider].
///
/// In the former case, the [RouteInformationProvider] notifies the [Router]
/// widget when a new [RouteInformation] is available. The [Router] widget takes
/// these information and navigates accordingly.
///
/// The latter case happens in web application where the [Router] reports route
/// changes back to the web engine.
///
/// The current [RouteInformation] of an application is also used for state
/// restoration purposes. Before an application is killed, the [Router] converts
/// its current configurations into a [RouteInformation] object utilizing the
/// [RouteInformationProvider]. The [RouteInformation] object is then serialized
/// out and persisted. During state restoration, the object is deserialized and
/// passed back to the [RouteInformationProvider], which turns it into a
/// configuration for the [Router] again to restore its state from.
class RouteInformation {
  /// Creates a route information object.
  ///
  /// Either `location` or `uri` must not be null.
  const RouteInformation({
    @Deprecated(
      'Pass Uri.parse(location) to uri parameter instead. '
      'This feature was deprecated after v3.8.0-3.0.pre.',
    )
    String? location,
    Uri? uri,
    this.state,
  }) : _location = location,
       _uri = uri,
       assert((location != null) != (uri != null));

  /// The location of the application.
  ///
  /// The string is usually in the format of multiple string identifiers with
  /// slashes in between. ex: `/`, `/path`, `/path/to/the/app`.
  @Deprecated(
    'Use uri instead. '
    'This feature was deprecated after v3.8.0-3.0.pre.',
  )
  String get location {
    return _location ??
        Uri.decodeComponent(
          Uri(
            path: uri.path.isEmpty ? '/' : uri.path,
            queryParameters: uri.queryParametersAll.isEmpty ? null : uri.queryParametersAll,
            fragment: uri.fragment.isEmpty ? null : uri.fragment,
          ).toString(),
        );
  }

  final String? _location;

  /// The uri location of the application.
  ///
  /// The host and scheme will not be empty if this object is created from a
  /// deep link request. They represents the website that redirect the deep
  /// link.
  ///
  /// In web platform, the host and scheme are always empty.
  Uri get uri {
    if (_uri != null) {
      return _uri;
    }
    return Uri.parse(_location!);
  }

  final Uri? _uri;

  /// The state of the application in the [uri].
  ///
  /// The app can have different states even in the same location. For example,
  /// the text inside a [TextField] or the scroll position in a [ScrollView].
  /// These widget states can be stored in the [state].
  ///
  /// On the web, this information is stored in the browser history when the
  /// [Router] reports this route information back to the web engine
  /// through the [PlatformRouteInformationProvider]. The information
  /// is then passed back, along with the [uri], when the user
  /// clicks the back or forward buttons.
  ///
  /// This information is also serialized and persisted alongside the
  /// [uri] for state restoration purposes. During state restoration,
  /// the information is made available again to the [Router] so it can restore
  /// its configuration to the previous state.
  ///
  /// The state must be serializable.
  final Object? state;
}

/// A convenient bundle to configure a [Router] widget.
///
/// To configure a [Router] widget, one needs to provide several delegates,
/// [RouteInformationProvider], [RouteInformationParser], [RouterDelegate],
/// and [BackButtonDispatcher]. This abstract class provides way to bundle these
/// delegates into a single object to configure a [Router].
///
/// The [backButtonDispatcher], [routeInformationProvider], and
/// [routeInformationProvider] are optional.
///
/// The [routeInformationProvider] and [routeInformationParser] must
/// both be provided or both not provided.
class RouterConfig<T> {
  /// Creates a [RouterConfig].
  ///
  /// The [backButtonDispatcher], [routeInformationProvider], and
  /// [routeInformationParser] are optional.
  ///
  /// The [routeInformationProvider] and [routeInformationParser] must both be
  /// provided or both not provided.
  const RouterConfig({
    this.routeInformationProvider,
    this.routeInformationParser,
    required this.routerDelegate,
    this.backButtonDispatcher,
  }) : assert((routeInformationProvider == null) == (routeInformationParser == null));

  /// The [RouteInformationProvider] that is used to configure the [Router].
  final RouteInformationProvider? routeInformationProvider;

  /// The [RouteInformationParser] that is used to configure the [Router].
  final RouteInformationParser<T>? routeInformationParser;

  /// The [RouterDelegate] that is used to configure the [Router].
  final RouterDelegate<T> routerDelegate;

  /// The [BackButtonDispatcher] that is used to configure the [Router].
  final BackButtonDispatcher? backButtonDispatcher;
}

/// The dispatcher for opening and closing pages of an application.
///
/// This widget listens for routing information from the operating system (e.g.
/// an initial route provided on app startup, a new route obtained when an
/// intent is received, or a notification that the user hit the system back
/// button), parses route information into data of type `T`, and then converts
/// that data into [Page] objects that it passes to a [Navigator].
///
/// Each part of this process can be overridden and configured as desired.
///
/// The [routeInformationProvider] can be overridden to change how the name of
/// the route is obtained. The [RouteInformationProvider.value] is used as the
/// initial route when the [Router] is first created. Subsequent notifications
/// from the [RouteInformationProvider] to its listeners are treated as
/// notifications that the route information has changed.
///
/// The [backButtonDispatcher] can be overridden to change how back button
/// notifications are received. This must be a [BackButtonDispatcher], which is
/// an object where callbacks can be registered, and which can be chained so
/// that back button presses are delegated to subsidiary routers. The callbacks
/// are invoked to indicate that the user is trying to close the current route
/// (by pressing the system back button); the [Router] ensures that when this
/// callback is invoked, the message is passed to the [routerDelegate] and its
/// result is provided back to the [backButtonDispatcher]. Some platforms don't
/// have back buttons (e.g. iOS and desktop platforms); on those platforms this
/// notification is never sent. Typically, the [backButtonDispatcher] for the
/// root router is an instance of [RootBackButtonDispatcher], which uses a
/// [WidgetsBindingObserver] to listen to the `popRoute` notifications from
/// [SystemChannels.navigation]. Nested [Router]s typically use a
/// [ChildBackButtonDispatcher], which must be provided the
/// [BackButtonDispatcher] of its ancestor [Router] (available via [Router.of]).
///
/// The [routeInformationParser] can be overridden to change how names obtained
/// from the [routeInformationProvider] are interpreted. It must implement the
/// [RouteInformationParser] interface, specialized with the same type as the
/// [Router] itself. This type, `T`, represents the data type that the
/// [routeInformationParser] will generate.
///
/// The [routerDelegate] can be overridden to change how the output of the
/// [routeInformationParser] is interpreted. It must implement the
/// [RouterDelegate] interface, also specialized with `T`; it takes as input
/// the data (of type `T`) from the [routeInformationParser], and is responsible
/// for providing a navigating widget to insert into the widget tree. The
/// [RouterDelegate] interface is also [Listenable]; notifications are taken
/// to mean that the [Router] needs to rebuild.
///
/// ## Concerns regarding asynchrony
///
/// Some of the APIs (notably those involving [RouteInformationParser] and
/// [RouterDelegate]) are asynchronous.
///
/// When developing objects implementing these APIs, if the work can be done
/// entirely synchronously, then consider using [SynchronousFuture] for the
/// future returned from the relevant methods. This will allow the [Router] to
/// proceed in a completely synchronous way, which removes a number of
/// complications.
///
/// Using asynchronous computation is entirely reasonable, however, and the API
/// is designed to support it. For example, maybe a set of images need to be
/// loaded before a route can be shown; waiting for those images to be loaded
/// before [RouterDelegate.setNewRoutePath] returns is a reasonable approach to
/// handle this case.
///
/// If an asynchronous operation is ongoing when a new one is to be started, the
/// precise behavior will depend on the exact circumstances, as follows:
///
/// If the active operation is a [routeInformationParser] parsing a new route information:
/// that operation's result, if it ever completes, will be discarded.
///
/// If the active operation is a [routerDelegate] handling a pop request:
/// the previous pop is immediately completed with "false", claiming that the
/// previous pop was not handled (this may cause the application to close).
///
/// If the active operation is a [routerDelegate] handling an initial route
/// or a pushed route, the result depends on the new operation. If the new
/// operation is a pop request, then the original operation's result, if it ever
/// completes, will be discarded. If the new operation is a push request,
/// however, the [routeInformationParser] will be requested to start the parsing, and
/// only if that finishes before the original [routerDelegate] request
/// completes will that original request's result be discarded.
///
/// If the identity of the [Router] widget's delegates change while an
/// asynchronous operation is in progress, to keep matters simple, all active
/// asynchronous operations will have their results discarded. It is generally
/// considered unusual for these delegates to change during the lifetime of the
/// [Router].
///
/// If the [Router] itself is disposed while an asynchronous operation is in
/// progress, all active asynchronous operations will have their results
/// discarded also.
///
/// No explicit signals are provided to the [routeInformationParser] or
/// [routerDelegate] to indicate when any of the above happens, so it is
/// strongly recommended that [RouteInformationParser] and [RouterDelegate]
/// implementations not perform extensive computation.
///
/// ## Application architectural design
///
/// An application can have zero, one, or many [Router] widgets, depending on
/// its needs.
///
/// An application might have no [Router] widgets if it has only one "screen",
/// or if the facilities provided by [Navigator] are sufficient. This is common
/// for desktop applications, where subsidiary "screens" are represented using
/// different windows rather than changing the active interface.
///
/// A particularly elaborate application might have multiple [Router] widgets,
/// in a tree configuration, with the first handling the entire route parsing
/// and making the result available for routers in the subtree. The routers in
/// the subtree do not participate in route information parsing but merely take the
/// result from the first router to build their sub routes.
///
/// Most applications only need a single [Router].
///
/// ## URL updates for web applications
///
/// In the web platform, keeping the URL in the browser's location bar up to
/// date with the application state ensures that the browser constructs its
/// history entry correctly, allowing its back and forward buttons to function
/// as the user expects.
///
/// If an app state change leads to the [Router] rebuilding, the [Router] will
/// retrieve the new route information from the [routerDelegate]'s
/// [RouterDelegate.currentConfiguration] method and the
/// [routeInformationParser]'s [RouteInformationParser.restoreRouteInformation]
/// method.
///
/// If the location in the new route information is different from the
/// current location, this is considered to be a navigation event, the
/// [PlatformRouteInformationProvider.routerReportsNewRouteInformation] method
/// calls [SystemNavigator.routeInformationUpdated] with `replace = false` to
/// notify the engine, and through that the browser, to create a history entry
/// with the new url. Otherwise,
/// [PlatformRouteInformationProvider.routerReportsNewRouteInformation] calls
/// [SystemNavigator.routeInformationUpdated] with `replace = true` to update
/// the current history entry with the latest [RouteInformation].
///
/// One can force the [Router] to report new route information as navigation
/// event to the [routeInformationProvider] (and thus the browser) even if the
/// [RouteInformation.uri] has not changed by calling the [Router.navigate]
/// method with a callback that performs the state change. This causes [Router]
/// to call the [RouteInformationProvider.routerReportsNewRouteInformation] with
/// [RouteInformationReportingType.navigate], and thus causes
/// [PlatformRouteInformationProvider] to push a new history entry regardlessly.
/// This allows one to support the browser's back and forward buttons without
/// changing the URL. For example, the scroll position of a scroll view may be
/// saved in the [RouteInformation.state]. Using [Router.navigate] to update the
/// scroll position causes the browser to create a new history entry with the
/// [RouteInformation.state] that stores this new scroll position. When the user
/// clicks the back button, the app will go back to the previous scroll position
/// without changing the URL in the location bar.
///
/// One can also force the [Router] to ignore a navigation event by making
/// those changes during a callback passed to [Router.neglect]. The [Router]
/// calls the [RouteInformationProvider.routerReportsNewRouteInformation] with
/// [RouteInformationReportingType.neglect], and thus causes
/// [PlatformRouteInformationProvider] to replace the current history entry
/// regardlessly even if it detects location change.
///
/// To opt out of URL updates entirely, pass null for [routeInformationProvider]
/// and [routeInformationParser]. This is not recommended in general, but may be
/// appropriate in the following cases:
///
/// * The application does not target the web platform.
///
/// * There are multiple router widgets in the application. Only one [Router]
///   widget should update the URL (typically the top-most one created by the
///   [WidgetsApp.router], [MaterialApp.router], or [CupertinoApp.router]).
///
/// * The application does not need to implement in-app navigation using the
///   browser's back and forward buttons.
///
/// In other cases, it is strongly recommended to implement the
/// [RouterDelegate.currentConfiguration] and
/// [RouteInformationParser.restoreRouteInformation] APIs to provide an optimal
/// user experience when running on the web platform.
///
/// ## State Restoration
///
/// The [Router] will restore the current configuration of the [routerDelegate]
/// during state restoration if it is configured with a [restorationScopeId] and
/// state restoration is enabled for the subtree. For that, the value of
/// [RouterDelegate.currentConfiguration] is serialized and persisted before the
/// app is killed by the operating system. After the app is restarted, the value
/// is deserialized and passed back to the [RouterDelegate] via a call to
/// [RouterDelegate.setRestoredRoutePath] (which by default just calls
/// [RouterDelegate.setNewRoutePath]). It is the responsibility of the
/// [RouterDelegate] to use the configuration information provided to restore
/// its internal state.
///
/// To serialize [RouterDelegate.currentConfiguration] and to deserialize it
/// again, the [Router] calls [RouteInformationParser.restoreRouteInformation]
/// and [RouteInformationParser.parseRouteInformation], respectively. Therefore,
/// if a [restorationScopeId] is provided, a [routeInformationParser] must be
/// configured as well.
class Router<T> extends StatefulWidget {
  /// Creates a router.
  ///
  /// The [routeInformationProvider] and [routeInformationParser] can be null if this
  /// router does not depend on route information. A common example is a sub router
  /// that builds its content completely based on the app state.
  ///
  /// The [routeInformationProvider] and [routeInformationParser] must
  /// both be provided or not provided.
  const Router({
    super.key,
    this.routeInformationProvider,
    this.routeInformationParser,
    required this.routerDelegate,
    this.backButtonDispatcher,
    this.restorationScopeId,
  }) : assert(
         routeInformationProvider == null || routeInformationParser != null,
         'A routeInformationParser must be provided when a routeInformationProvider is specified.',
       );

  /// Creates a router with a [RouterConfig].
  ///
  /// The [RouterConfig.routeInformationProvider] and
  /// [RouterConfig.routeInformationParser] can be null if this router does not
  /// depend on route information. A common example is a sub router that builds
  /// its content completely based on the app state.
  ///
  /// If the [RouterConfig.routeInformationProvider] is not null, then
  /// [RouterConfig.routeInformationParser] must also not be
  /// null.
  factory Router.withConfig({
    Key? key,
    required RouterConfig<T> config,
    String? restorationScopeId,
  }) {
    return Router<T>(
      key: key,
      routeInformationProvider: config.routeInformationProvider,
      routeInformationParser: config.routeInformationParser,
      routerDelegate: config.routerDelegate,
      backButtonDispatcher: config.backButtonDispatcher,
      restorationScopeId: restorationScopeId,
    );
  }

  /// The route information provider for the router.
  ///
  /// The value at the time of first build will be used as the initial route.
  /// The [Router] listens to this provider and rebuilds with new names when
  /// it notifies.
  ///
  /// This can be null if this router does not rely on the route information
  /// to build its content. In such case, the [routeInformationParser] must also
  /// be null.
  final RouteInformationProvider? routeInformationProvider;

  /// The route information parser for the router.
  ///
  /// When the [Router] gets a new route information from the [routeInformationProvider],
  /// the [Router] uses this delegate to parse the route information and produce a
  /// configuration. The configuration will be used by [routerDelegate] and
  /// eventually rebuilds the [Router] widget.
  ///
  /// Since this delegate is the primary consumer of the [routeInformationProvider],
  /// it must not be null if [routeInformationProvider] is not null.
  final RouteInformationParser<T>? routeInformationParser;

  /// The router delegate for the router.
  ///
  /// This delegate consumes the configuration from [routeInformationParser] and
  /// builds a navigating widget for the [Router].
  ///
  /// It is also the primary respondent for the [backButtonDispatcher]. The
  /// [Router] relies on [RouterDelegate.popRoute] to handle the back
  /// button.
  ///
  /// If the [RouterDelegate.currentConfiguration] returns a non-null object,
  /// this [Router] will opt for URL updates.
  final RouterDelegate<T> routerDelegate;

  /// The back button dispatcher for the router.
  ///
  /// The two common alternatives are the [RootBackButtonDispatcher] for root
  /// router, or the [ChildBackButtonDispatcher] for other routers.
  final BackButtonDispatcher? backButtonDispatcher;

  /// Restoration ID to save and restore the state of the [Router].
  ///
  /// If non-null, the [Router] will persist the [RouterDelegate]'s current
  /// configuration (i.e. [RouterDelegate.currentConfiguration]). During state
  /// restoration, the [Router] informs the [RouterDelegate] of the previous
  /// configuration by calling [RouterDelegate.setRestoredRoutePath] (which by
  /// default just calls [RouterDelegate.setNewRoutePath]). It is the
  /// responsibility of the [RouterDelegate] to restore its internal state based
  /// on the provided configuration.
  ///
  /// The router uses the [RouteInformationParser] to serialize and deserialize
  /// [RouterDelegate.currentConfiguration]. Therefore, a
  /// [routeInformationParser] must be provided when [restorationScopeId] is
  /// non-null.
  ///
  /// See also:
  ///
  ///  * [RestorationManager], which explains how state restoration works in
  ///    Flutter.
  final String? restorationScopeId;

  /// Retrieves the immediate [Router] ancestor from the given context.
  ///
  /// This method provides access to the delegates in the [Router]. For example,
  /// this can be used to access the [backButtonDispatcher] of the parent router
  /// when creating a [ChildBackButtonDispatcher] for a nested [Router].
  ///
  /// If no [Router] ancestor exists for the given context, this will assert in
  /// debug mode, and throw an exception in release mode.
  ///
  /// See also:
  ///
  ///  * [maybeOf], which is a similar function, but it will return null instead
  ///    of throwing an exception if no [Router] ancestor exists.
  static Router<T> of<T extends Object?>(BuildContext context) {
    final _RouterScope? scope = context.dependOnInheritedWidgetOfExactType<_RouterScope>();
    assert(() {
      if (scope == null) {
        throw FlutterError(
          'Router operation requested with a context that does not include a Router.\n'
          'The context used to retrieve the Router must be that of a widget that '
          'is a descendant of a Router widget.',
        );
      }
      return true;
    }());
    return scope!.routerState.widget as Router<T>;
  }

  /// Retrieves the immediate [Router] ancestor from the given context.
  ///
  /// This method provides access to the delegates in the [Router]. For example,
  /// this can be used to access the [backButtonDispatcher] of the parent router
  /// when creating a [ChildBackButtonDispatcher] for a nested [Router].
  ///
  /// If no `Router` ancestor exists for the given context, this will return
  /// null.
  ///
  /// See also:
  ///
  ///  * [of], a similar method that returns a non-nullable value, and will
  ///    throw if no [Router] ancestor exists.
  static Router<T>? maybeOf<T extends Object?>(BuildContext context) {
    final _RouterScope? scope = context.dependOnInheritedWidgetOfExactType<_RouterScope>();
    return scope?.routerState.widget as Router<T>?;
  }

  /// Forces the [Router] to run the [callback] and create a new history
  /// entry in the browser.
  ///
  /// The web application relies on the [Router] to report new route information
  /// in order to create browser history entry. The [Router] will only report
  /// them if it detects the [RouteInformation.uri] changes. Use this
  /// method if you want the [Router] to report the route information even if
  /// the location does not change. This can be useful when you want to
  /// support the browser backward and forward button without changing the URL.
  ///
  /// For example, you can store certain state such as the scroll position into
  /// the [RouteInformation.state]. If you use this method to update the
  /// scroll position multiple times with the same URL, the browser will create
  /// a stack of new history entries with the same URL but different
  /// [RouteInformation.state]s that store the new scroll positions. If the user
  /// click the backward button in the browser, the browser will restore the
  /// scroll positions saved in history entries without changing the URL.
  ///
  /// See also:
  ///
  ///  * [Router]: see the "URL updates for web applications" section for more
  ///    information about route information reporting.
  ///  * [neglect]: which forces the [Router] to not create a new history entry
  ///    even if location does change.
  static void navigate(BuildContext context, VoidCallback callback) {
    final _RouterScope scope =
        context.getElementForInheritedWidgetOfExactType<_RouterScope>()!.widget as _RouterScope;
    scope.routerState._setStateWithExplicitReportStatus(
      RouteInformationReportingType.navigate,
      callback,
    );
  }

  /// Forces the [Router] to run the [callback] without creating a new history
  /// entry in the browser.
  ///
  /// The web application relies on the [Router] to report new route information
  /// in order to create browser history entry. The [Router] will report them
  /// automatically if it detects the [RouteInformation.uri] changes.
  ///
  /// Creating a new route history entry makes users feel they have visited a
  /// new page, and the browser back button brings them back to previous history
  /// entry. Use this method if you don't want the [Router] to create a new
  /// route information even if it detects changes as a result of running the
  /// [callback].
  ///
  /// Using this method will still update the URL and state in current history
  /// entry.
  ///
  /// See also:
  ///
  ///  * [Router]: see the "URL updates for web applications" section for more
  ///    information about route information reporting.
  ///  * [navigate]: which forces the [Router] to create a new history entry
  ///    even if location does not change.
  static void neglect(BuildContext context, VoidCallback callback) {
    final _RouterScope scope =
        context.getElementForInheritedWidgetOfExactType<_RouterScope>()!.widget as _RouterScope;
    scope.routerState._setStateWithExplicitReportStatus(
      RouteInformationReportingType.neglect,
      callback,
    );
  }

  @override
  State<Router<T>> createState() => _RouterState<T>();
}

typedef _AsyncPassthrough<Q> = Future<Q> Function(Q);
typedef _RouteSetter<T> = Future<void> Function(T);

/// The [Router]'s intention when it reports a new [RouteInformation] to the
/// [RouteInformationProvider].
///
/// See also:
///
///  * [RouteInformationProvider.routerReportsNewRouteInformation]: which is
///    called by the router when it has a new route information to report.
enum RouteInformationReportingType {
  /// Router does not have a specific intention.
  ///
  /// The router generates a new route information every time it detects route
  /// information may have change due to a rebuild. This is the default type if
  /// neither [Router.neglect] nor [Router.navigate] was used during the
  /// rebuild.
  none,

  /// The accompanying [RouteInformation] were generated during a
  /// [Router.neglect] call.
  neglect,

  /// The accompanying [RouteInformation] were generated during a
  /// [Router.navigate] call.
  navigate,
}

class _RouterState<T> extends State<Router<T>> with RestorationMixin {
  Object? _currentRouterTransaction;
  RouteInformationReportingType? _currentIntentionToReport;
  final _RestorableRouteInformation _routeInformation = _RestorableRouteInformation();
  late bool _routeParsePending;

  @override
  String? get restorationId => widget.restorationScopeId;

  @override
  void initState() {
    super.initState();
    widget.routeInformationProvider?.addListener(_handleRouteInformationProviderNotification);
    widget.backButtonDispatcher?.addCallback(_handleBackButtonDispatcherNotification);
    widget.routerDelegate.addListener(_handleRouterDelegateNotification);
  }

  @override
  void restoreState(RestorationBucket? oldBucket, bool initialRestore) {
    registerForRestoration(_routeInformation, 'route');
    if (_routeInformation.value != null) {
      assert(widget.routeInformationParser != null);
      _processRouteInformation(
        _routeInformation.value!,
        () => widget.routerDelegate.setRestoredRoutePath,
      );
    } else if (widget.routeInformationProvider != null) {
      _processRouteInformation(
        widget.routeInformationProvider!.value,
        () => widget.routerDelegate.setInitialRoutePath,
      );
    }
  }

  bool _routeInformationReportingTaskScheduled = false;

  void _scheduleRouteInformationReportingTask() {
    if (_routeInformationReportingTaskScheduled || widget.routeInformationProvider == null) {
      return;
    }
    assert(_currentIntentionToReport != null);
    _routeInformationReportingTaskScheduled = true;
    SchedulerBinding.instance.addPostFrameCallback(
      _reportRouteInformation,
      debugLabel: 'Router.reportRouteInfo',
    );
  }

  void _reportRouteInformation(Duration timestamp) {
    if (!mounted) {
      return;
    }

    assert(_routeInformationReportingTaskScheduled);
    _routeInformationReportingTaskScheduled = false;

    if (_routeInformation.value != null) {
      final RouteInformation currentRouteInformation = _routeInformation.value!;
      assert(_currentIntentionToReport != null);
      widget.routeInformationProvider!.routerReportsNewRouteInformation(
        currentRouteInformation,
        type: _currentIntentionToReport!,
      );
    }
    _currentIntentionToReport = RouteInformationReportingType.none;
  }

  RouteInformation? _retrieveNewRouteInformation() {
    final T? configuration = widget.routerDelegate.currentConfiguration;
    if (configuration == null) {
      return null;
    }
    return widget.routeInformationParser?.restoreRouteInformation(configuration);
  }

  void _setStateWithExplicitReportStatus(RouteInformationReportingType status, VoidCallback fn) {
    assert(status.index >= RouteInformationReportingType.neglect.index);
    assert(() {
      if (_currentIntentionToReport != null &&
          _currentIntentionToReport != RouteInformationReportingType.none &&
          _currentIntentionToReport != status) {
        FlutterError.reportError(
          const FlutterErrorDetails(
            exception:
                'Both Router.navigate and Router.neglect have been called in this '
                'build cycle, and the Router cannot decide whether to report the '
                'route information. Please make sure only one of them is called '
                'within the same build cycle.',
          ),
        );
      }
      return true;
    }());
    _currentIntentionToReport = status;
    _scheduleRouteInformationReportingTask();
    fn();
  }

  void _maybeNeedToReportRouteInformation() {
    _routeInformation.value = _retrieveNewRouteInformation();
    _currentIntentionToReport ??= RouteInformationReportingType.none;
    _scheduleRouteInformationReportingTask();
  }

  @override
  void didChangeDependencies() {
    _routeParsePending = true;
    super.didChangeDependencies();
    // The super.didChangeDependencies may have parsed the route information.
    // This can happen if the didChangeDependencies is triggered by state
    // restoration or first build.
    final RouteInformation? currentRouteInformation =
        _routeInformation.value ?? widget.routeInformationProvider?.value;
    if (currentRouteInformation != null && _routeParsePending) {
      _processRouteInformation(
        currentRouteInformation,
        () => widget.routerDelegate.setNewRoutePath,
      );
    }
    _routeParsePending = false;
    _maybeNeedToReportRouteInformation();
  }

  @override
  void didUpdateWidget(Router<T> oldWidget) {
    super.didUpdateWidget(oldWidget);
    if (widget.routeInformationProvider != oldWidget.routeInformationProvider ||
        widget.backButtonDispatcher != oldWidget.backButtonDispatcher ||
        widget.routeInformationParser != oldWidget.routeInformationParser ||
        widget.routerDelegate != oldWidget.routerDelegate) {
      _currentRouterTransaction = Object();
    }
    if (widget.routeInformationProvider != oldWidget.routeInformationProvider) {
      oldWidget.routeInformationProvider?.removeListener(
        _handleRouteInformationProviderNotification,
      );
      widget.routeInformationProvider?.addListener(_handleRouteInformationProviderNotification);
      if (oldWidget.routeInformationProvider?.value != widget.routeInformationProvider?.value) {
        _handleRouteInformationProviderNotification();
      }
    }
    if (widget.backButtonDispatcher != oldWidget.backButtonDispatcher) {
      oldWidget.backButtonDispatcher?.removeCallback(_handleBackButtonDispatcherNotification);
      widget.backButtonDispatcher?.addCallback(_handleBackButtonDispatcherNotification);
    }
    if (widget.routerDelegate != oldWidget.routerDelegate) {
      oldWidget.routerDelegate.removeListener(_handleRouterDelegateNotification);
      widget.routerDelegate.addListener(_handleRouterDelegateNotification);
      _maybeNeedToReportRouteInformation();
    }
  }

  @override
  void dispose() {
    _routeInformation.dispose();
    widget.routeInformationProvider?.removeListener(_handleRouteInformationProviderNotification);
    widget.backButtonDispatcher?.removeCallback(_handleBackButtonDispatcherNotification);
    widget.routerDelegate.removeListener(_handleRouterDelegateNotification);
    _currentRouterTransaction = null;
    super.dispose();
  }

  void _processRouteInformation(
    RouteInformation information,
    ValueGetter<_RouteSetter<T>> delegateRouteSetter,
  ) {
    assert(_routeParsePending);
    _routeParsePending = false;
    _currentRouterTransaction = Object();
    widget.routeInformationParser!
        .parseRouteInformationWithDependencies(information, context)
        .then<void>(_processParsedRouteInformation(_currentRouterTransaction, delegateRouteSetter));
  }

  _RouteSetter<T> _processParsedRouteInformation(
    Object? transaction,
    ValueGetter<_RouteSetter<T>> delegateRouteSetter,
  ) {
    return (T data) async {
      if (_currentRouterTransaction != transaction) {
        return;
      }
      await delegateRouteSetter()(data);
      if (_currentRouterTransaction == transaction) {
        _rebuild();
      }
    };
  }

  void _handleRouteInformationProviderNotification() {
    _routeParsePending = true;
    _processRouteInformation(
      widget.routeInformationProvider!.value,
      () => widget.routerDelegate.setNewRoutePath,
    );
  }

  Future<bool> _handleBackButtonDispatcherNotification() {
    _currentRouterTransaction = Object();
    return widget.routerDelegate.popRoute().then<bool>(
      _handleRoutePopped(_currentRouterTransaction),
    );
  }

  _AsyncPassthrough<bool> _handleRoutePopped(Object? transaction) {
    return (bool data) {
      if (transaction != _currentRouterTransaction) {
        // A rebuilt was trigger from a different source. Returns true to
        // prevent bubbling.
        return SynchronousFuture<bool>(true);
      }
      _rebuild();
      return SynchronousFuture<bool>(data);
    };
  }

  Future<void> _rebuild([void value]) {
    setState(() {
      /* routerDelegate is ready to rebuild */
    });
    _maybeNeedToReportRouteInformation();
    return SynchronousFuture<void>(value);
  }

  void _handleRouterDelegateNotification() {
    setState(() {
      /* routerDelegate wants to rebuild */
    });
    _maybeNeedToReportRouteInformation();
  }

  @override
  Widget build(BuildContext context) {
    return UnmanagedRestorationScope(
      bucket: bucket,
      child: _RouterScope(
        routeInformationProvider: widget.routeInformationProvider,
        backButtonDispatcher: widget.backButtonDispatcher,
        routeInformationParser: widget.routeInformationParser,
        routerDelegate: widget.routerDelegate,
        routerState: this,
        child: Builder(
          // Use a Builder so that the build method below will have a
          // BuildContext that contains the _RouterScope. This also prevents
          // dependencies look ups in routerDelegate from rebuilding Router
          // widget that may result in re-parsing the route information.
          builder: widget.routerDelegate.build,
        ),
      ),
    );
  }
}

class _RouterScope extends InheritedWidget {
  const _RouterScope({
    required this.routeInformationProvider,
    required this.backButtonDispatcher,
    required this.routeInformationParser,
    required this.routerDelegate,
    required this.routerState,
    required super.child,
  }) : assert(routeInformationProvider == null || routeInformationParser != null);

  final ValueListenable<RouteInformation?>? routeInformationProvider;
  final BackButtonDispatcher? backButtonDispatcher;
  final RouteInformationParser<Object?>? routeInformationParser;
  final RouterDelegate<Object?> routerDelegate;
  final _RouterState<Object?> routerState;

  @override
  bool updateShouldNotify(_RouterScope oldWidget) {
    return routeInformationProvider != oldWidget.routeInformationProvider ||
        backButtonDispatcher != oldWidget.backButtonDispatcher ||
        routeInformationParser != oldWidget.routeInformationParser ||
        routerDelegate != oldWidget.routerDelegate ||
        routerState != oldWidget.routerState;
  }
}

/// A class that can be extended or mixed in that invokes a single callback,
/// which then returns a value.
///
/// While multiple callbacks can be registered, when a notification is
/// dispatched there must be only a single callback. The return values of
/// multiple callbacks are not aggregated.
///
/// `T` is the return value expected from the callback.
///
/// See also:
///
///  * [Listenable] and its subclasses, which provide a similar mechanism for
///    one-way signaling.
class _CallbackHookProvider<T> {
  final ObserverList<ValueGetter<T>> _callbacks = ObserverList<ValueGetter<T>>();

  /// Whether a callback is currently registered.
  @protected
  bool get hasCallbacks => _callbacks.isNotEmpty;

  /// Register the callback to be called when the object changes.
  ///
  /// If other callbacks have already been registered, they must be removed
  /// (with [removeCallback]) before the callback is next called.
  void addCallback(ValueGetter<T> callback) => _callbacks.add(callback);

  /// Remove a previously registered callback.
  ///
  /// If the given callback is not registered, the call is ignored.
  void removeCallback(ValueGetter<T> callback) => _callbacks.remove(callback);

  /// Calls the (single) registered callback and returns its result.
  ///
  /// If no callback is registered, or if the callback throws, returns
  /// `defaultValue`.
  ///
  /// Call this method whenever the callback is to be invoked. If there is more
  /// than one callback registered, this method will throw a [StateError].
  ///
  /// Exceptions thrown by callbacks will be caught and reported using
  /// [FlutterError.reportError].
  @protected
  @pragma('vm:notify-debugger-on-exception')
  T invokeCallback(T defaultValue) {
    if (_callbacks.isEmpty) {
      return defaultValue;
    }
    try {
      return _callbacks.single();
    } catch (exception, stack) {
      FlutterError.reportError(
        FlutterErrorDetails(
          exception: exception,
          stack: stack,
          library: 'widget library',
          context: ErrorDescription('while invoking the callback for $runtimeType'),
          informationCollector:
              () => <DiagnosticsNode>[
                DiagnosticsProperty<_CallbackHookProvider<T>>(
                  'The $runtimeType that invoked the callback was',
                  this,
                  style: DiagnosticsTreeStyle.errorProperty,
                ),
              ],
        ),
      );
      return defaultValue;
    }
  }
}

/// Report to a [Router] when the user taps the back button on platforms that
/// support back buttons (such as Android).
///
/// When [Router] widgets are nested, consider using a
/// [ChildBackButtonDispatcher], passing it the parent [BackButtonDispatcher],
/// so that the back button requests get dispatched to the appropriate [Router].
/// To make this work properly, it's important that whenever a [Router] thinks
/// it should get the back button messages (e.g. after the user taps inside it),
/// it calls [takePriority] on its [BackButtonDispatcher] (or
/// [ChildBackButtonDispatcher]) instance.
///
/// The class takes a single callback, which must return a [Future<bool>]. The
/// callback's semantics match [WidgetsBindingObserver.didPopRoute]'s, namely,
/// the callback should return a future that completes to true if it can handle
/// the pop request, and a future that completes to false otherwise.
abstract class BackButtonDispatcher extends _CallbackHookProvider<Future<bool>> {
  late final LinkedHashSet<ChildBackButtonDispatcher> _children =
      <ChildBackButtonDispatcher>{} as LinkedHashSet<ChildBackButtonDispatcher>;

  @override
  bool get hasCallbacks => super.hasCallbacks || (_children.isNotEmpty);

  /// Handles a pop route request.
  ///
  /// This method prioritizes the children list in reverse order and calls
  /// [ChildBackButtonDispatcher.notifiedByParent] on them. If any of them
  /// handles the request (by returning a future with true), it exits this
  /// method by returning this future. Otherwise, it keeps moving on to the next
  /// child until a child handles the request. If none of the children handles
  /// the request, this back button dispatcher will then try to handle the request
  /// by itself. This back button dispatcher handles the request by notifying the
  /// router which in turn calls the [RouterDelegate.popRoute] and returns its
  /// result.
  ///
  /// To decide whether this back button dispatcher will handle the pop route
  /// request, you can override the [RouterDelegate.popRoute] of the router
  /// delegate you pass into the router with this back button dispatcher to
  /// return a future of true or false.
  @override
  Future<bool> invokeCallback(Future<bool> defaultValue) {
    if (_children.isNotEmpty) {
      final List<ChildBackButtonDispatcher> children = _children.toList();
      int childIndex = children.length - 1;

      Future<bool> notifyNextChild(bool result) {
        // If the previous child handles the callback, we return the result.
        if (result) {
          return SynchronousFuture<bool>(result);
        }
        // If the previous child did not handle the callback, we ask the next
        // child to handle the it.
        if (childIndex > 0) {
          childIndex -= 1;
          return children[childIndex].notifiedByParent(defaultValue).then<bool>(notifyNextChild);
        }
        // If none of the child handles the callback, the parent will then handle it.
        return super.invokeCallback(defaultValue);
      }

      return children[childIndex].notifiedByParent(defaultValue).then<bool>(notifyNextChild);
    }
    return super.invokeCallback(defaultValue);
  }

  /// Creates a [ChildBackButtonDispatcher] that is a direct descendant of this
  /// back button dispatcher.
  ///
  /// To participate in handling the pop route request, call the [takePriority]
  /// on the [ChildBackButtonDispatcher] created from this method.
  ///
  /// When the pop route request is handled by this back button dispatcher, it
  /// propagate the request to its direct descendants that have called the
  /// [takePriority] method. If there are multiple candidates, the latest one
  /// that called the [takePriority] wins the right to handle the request. If
  /// the latest one does not handle the request (by returning a future of
  /// false in [ChildBackButtonDispatcher.notifiedByParent]), the second latest
  /// one will then have the right to handle the request. This dispatcher
  /// continues finding the next candidate until there are no more candidates
  /// and finally handles the request itself.
  ChildBackButtonDispatcher createChildBackButtonDispatcher() {
    return ChildBackButtonDispatcher(this);
  }

  /// Make this [BackButtonDispatcher] take priority among its peers.
  ///
  /// This has no effect when a [BackButtonDispatcher] has no parents and no
  /// children. If a [BackButtonDispatcher] does have parents or children,
  /// however, it causes this object to be the one to dispatch the notification
  /// when the parent would normally notify its callback.
  ///
  /// The [BackButtonDispatcher] must have a listener registered before it can
  /// be told to take priority.
  void takePriority() => _children.clear();

  /// Mark the given child as taking priority over this object and the other
  /// children.
  ///
  /// This causes [invokeCallback] to defer to the given child instead of
  /// calling this object's callback.
  ///
  /// Children are stored in a list, so that if the current child is removed
  /// using [forget], a previous child will return to take its place. When
  /// [takePriority] is called, the list is cleared.
  ///
  /// Calling this again without first calling [forget] moves the child back to
  /// the head of the list.
  ///
  /// The [BackButtonDispatcher] must have a listener registered before it can
  /// be told to defer to a child.
  void deferTo(ChildBackButtonDispatcher child) {
    assert(hasCallbacks);
    _children.remove(child); // child may or may not be in the set already
    _children.add(child);
  }

  /// Causes the given child to be removed from the list of children to which
  /// this object might defer, as if [deferTo] had never been called for that
  /// child.
  ///
  /// This should only be called once per child, even if [deferTo] was called
  /// multiple times for that child.
  ///
  /// If no children are left in the list, this object will stop deferring to
  /// its children. (This is not the same as calling [takePriority], since, if
  /// this object itself is a [ChildBackButtonDispatcher], [takePriority] would
  /// additionally attempt to claim priority from its parent, whereas removing
  /// the last child does not.)
  void forget(ChildBackButtonDispatcher child) => _children.remove(child);
}

/// The default implementation of back button dispatcher for the root router.
///
/// This dispatcher listens to platform pop route notifications. When the
/// platform wants to pop the current route, this dispatcher calls the
/// [BackButtonDispatcher.invokeCallback] method to handle the request.
class RootBackButtonDispatcher extends BackButtonDispatcher with WidgetsBindingObserver {
  /// Create a root back button dispatcher.
  RootBackButtonDispatcher();

  @override
  void addCallback(ValueGetter<Future<bool>> callback) {
    if (!hasCallbacks) {
      WidgetsBinding.instance.addObserver(this);
    }
    super.addCallback(callback);
  }

  @override
  void removeCallback(ValueGetter<Future<bool>> callback) {
    super.removeCallback(callback);
    if (!hasCallbacks) {
      WidgetsBinding.instance.removeObserver(this);
    }
  }

  @override
  Future<bool> didPopRoute() => invokeCallback(Future<bool>.value(false));
}

/// A variant of [BackButtonDispatcher] which listens to notifications from a
/// parent back button dispatcher, and can take priority from its parent for the
/// handling of such notifications.
///
/// Useful when [Router]s are being nested within each other.
///
/// Use [Router.of] to obtain a reference to the nearest ancestor [Router], from
/// which the [Router.backButtonDispatcher] can be found, and then used as the
/// [parent] of the [ChildBackButtonDispatcher].
class ChildBackButtonDispatcher extends BackButtonDispatcher {
  /// Creates a back button dispatcher that acts as the child of another.
  ChildBackButtonDispatcher(this.parent);

  /// The back button dispatcher that this object will attempt to take priority
  /// over when [takePriority] is called.
  ///
  /// The parent must have a listener registered before this child object can
  /// have its [takePriority] or [deferTo] methods used.
  final BackButtonDispatcher parent;

  /// The parent of this child back button dispatcher decide to let this
  /// child to handle the invoke the callback request in
  /// [BackButtonDispatcher.invokeCallback].
  ///
  /// Return a boolean future with true if this child will handle the request;
  /// otherwise, return a boolean future with false.
  @protected
  Future<bool> notifiedByParent(Future<bool> defaultValue) {
    return invokeCallback(defaultValue);
  }

  @override
  void takePriority() {
    parent.deferTo(this);
    super.takePriority();
  }

  @override
  void deferTo(ChildBackButtonDispatcher child) {
    assert(hasCallbacks);
    parent.deferTo(this);
    super.deferTo(child);
  }

  @override
  void removeCallback(ValueGetter<Future<bool>> callback) {
    super.removeCallback(callback);
    if (!hasCallbacks) {
      parent.forget(this);
    }
  }
}

/// A convenience widget that registers a callback for when the back button is pressed.
///
/// In order to use this widget, there must be an ancestor [Router] widget in the tree
/// that has a [RootBackButtonDispatcher]. e.g. The [Router] widget created by the
/// [MaterialApp.router] has a built-in [RootBackButtonDispatcher] by default.
///
/// It only applies to platforms that accept back button clicks, such as Android.
///
/// It can be useful for scenarios, in which you create a different state in your
/// screen but don't want to use a new page for that.
class BackButtonListener extends StatefulWidget {
  /// Creates a BackButtonListener widget .
  const BackButtonListener({super.key, required this.child, required this.onBackButtonPressed});

  /// The widget below this widget in the tree.
  final Widget child;

  /// The callback function that will be called when the back button is pressed.
  ///
  /// It must return a boolean future with true if this child will handle the request;
  /// otherwise, return a boolean future with false.
  final ValueGetter<Future<bool>> onBackButtonPressed;

  @override
  State<BackButtonListener> createState() => _BackButtonListenerState();
}

class _BackButtonListenerState extends State<BackButtonListener> {
  BackButtonDispatcher? dispatcher;

  @override
  void didChangeDependencies() {
    dispatcher?.removeCallback(widget.onBackButtonPressed);

    final BackButtonDispatcher? rootBackDispatcher = Router.of(context).backButtonDispatcher;
    assert(
      rootBackDispatcher != null,
      'The parent router must have a backButtonDispatcher to use this widget',
    );

    dispatcher =
        rootBackDispatcher!.createChildBackButtonDispatcher()
          ..addCallback(widget.onBackButtonPressed)
          ..takePriority();
    super.didChangeDependencies();
  }

  @override
  void didUpdateWidget(covariant BackButtonListener oldWidget) {
    super.didUpdateWidget(oldWidget);
    if (oldWidget.onBackButtonPressed != widget.onBackButtonPressed) {
      dispatcher?.removeCallback(oldWidget.onBackButtonPressed);
      dispatcher?.addCallback(widget.onBackButtonPressed);
      dispatcher?.takePriority();
    }
  }

  @override
  void dispose() {
    dispatcher?.removeCallback(widget.onBackButtonPressed);
    super.dispose();
  }

  @override
  Widget build(BuildContext context) => widget.child;
}

/// A delegate that is used by the [Router] widget to parse a route information
/// into a configuration of type T.
///
/// This delegate is used when the [Router] widget is first built with initial
/// route information from [Router.routeInformationProvider] and any subsequent
/// new route notifications from it. The [Router] widget calls the [parseRouteInformation]
/// with the route information from [Router.routeInformationProvider].
///
/// One of the [parseRouteInformation] or
/// [parseRouteInformationWithDependencies] must be implemented, otherwise a
/// runtime error will be thrown.
abstract class RouteInformationParser<T> {
  /// Abstract const constructor. This constructor enables subclasses to provide
  /// const constructors so that they can be used in const expressions.
  const RouteInformationParser();

  /// {@template flutter.widgets.RouteInformationParser.parseRouteInformation}
  /// Converts the given route information into parsed data to pass to a
  /// [RouterDelegate].
  ///
  /// The method should return a future which completes when the parsing is
  /// complete. The parsing may be asynchronous if, e.g., the parser needs to
  /// communicate with the OEM thread to obtain additional data about the route.
  ///
  /// Consider using a [SynchronousFuture] if the result can be computed
  /// synchronously, so that the [Router] does not need to wait for the next
  /// microtask to pass the data to the [RouterDelegate].
  /// {@endtemplate}
  ///
  /// One can implement [parseRouteInformationWithDependencies] instead if
  /// the parsing depends on other dependencies from the [BuildContext].
  Future<T> parseRouteInformation(RouteInformation routeInformation) {
    throw UnimplementedError(
      'One of the parseRouteInformation or '
      'parseRouteInformationWithDependencies must be implemented',
    );
  }

  /// {@macro flutter.widgets.RouteInformationParser.parseRouteInformation}
  ///
  /// The input [BuildContext] can be used for looking up [InheritedWidget]s
  /// If one uses [BuildContext.dependOnInheritedWidgetOfExactType], a
  /// dependency will be created. The [Router] will re-parse the
  /// [RouteInformation] from its [RouteInformationProvider] if the dependency
  /// notifies its listeners.
  ///
  /// One can also use [BuildContext.getElementForInheritedWidgetOfExactType] to
  /// look up [InheritedWidget]s without creating dependencies.
  Future<T> parseRouteInformationWithDependencies(
    RouteInformation routeInformation,
    BuildContext context,
  ) {
    return parseRouteInformation(routeInformation);
  }

  /// Restore the route information from the given configuration.
  ///
  /// This may return null, in which case the browser history will not be
  /// updated and state restoration is disabled. See [Router]'s documentation
  /// for details.
  ///
  /// The [parseRouteInformation] method must produce an equivalent
  /// configuration when passed this method's return value.
  RouteInformation? restoreRouteInformation(T configuration) => null;
}

/// A delegate that is used by the [Router] widget to build and configure a
/// navigating widget.
///
/// This delegate is the core piece of the [Router] widget. It responds to
/// push route and pop route intents from the engine and notifies the [Router]
/// to rebuild. It also acts as a builder for the [Router] widget and builds a
/// navigating widget, typically a [Navigator], when the [Router] widget
/// builds.
///
/// When the engine pushes a new route, the route information is parsed by the
/// [RouteInformationParser] to produce a configuration of type T. The router
/// delegate receives the configuration through [setInitialRoutePath] or
/// [setNewRoutePath] to configure itself and builds the latest navigating
/// widget when asked ([build]).
///
/// When implementing subclasses, consider defining a [Listenable] app state object to be
/// used for building the navigating widget. The router delegate would update
/// the app state accordingly and notify its own listeners when the app state has
/// changed and when it receive route related engine intents (e.g.
/// [setNewRoutePath], [setInitialRoutePath], or [popRoute]).
///
/// All subclass must implement [setNewRoutePath], [popRoute], and [build].
///
/// ## State Restoration
///
/// If the [Router] owning this delegate is configured for state restoration, it
/// will persist and restore the configuration of this [RouterDelegate] using
/// the following mechanism: Before the app is killed by the operating system,
/// the value of [currentConfiguration] is serialized out and persisted. After
/// the app has restarted, the value is deserialized and passed back to the
/// [RouterDelegate] via a call to [setRestoredRoutePath] (which by default just
/// calls [setNewRoutePath]). It is the responsibility of the [RouterDelegate]
/// to use the configuration information provided to restore its internal state.
///
/// See also:
///
///  * [RouteInformationParser], which is responsible for parsing the route
///    information to a configuration before passing in to router delegate.
///  * [Router], which is the widget that wires all the delegates together to
///    provide a fully functional routing solution.
abstract class RouterDelegate<T> extends Listenable {
  /// Called by the [Router] at startup with the structure that the
  /// [RouteInformationParser] obtained from parsing the initial route.
  ///
  /// This should configure the [RouterDelegate] so that when [build] is
  /// invoked, it will create a widget tree that matches the initial route.
  ///
  /// By default, this method forwards the [configuration] to [setNewRoutePath].
  ///
  /// Consider using a [SynchronousFuture] if the result can be computed
  /// synchronously, so that the [Router] does not need to wait for the next
  /// microtask to schedule a build.
  ///
  /// See also:
  ///
  ///  * [setRestoredRoutePath], which is called instead of this method during
  ///    state restoration.
  Future<void> setInitialRoutePath(T configuration) {
    return setNewRoutePath(configuration);
  }

  /// Called by the [Router] during state restoration.
  ///
  /// When the [Router] is configured for state restoration, it will persist
  /// the value of [currentConfiguration] during state serialization. During
  /// state restoration, the [Router] calls this method (instead of
  /// [setInitialRoutePath]) to pass the previous configuration back to the
  /// delegate. It is the responsibility of the delegate to restore its internal
  /// state based on the provided configuration.
  ///
  /// By default, this method forwards the `configuration` to [setNewRoutePath].
  Future<void> setRestoredRoutePath(T configuration) {
    return setNewRoutePath(configuration);
  }

  /// Called by the [Router] when the [Router.routeInformationProvider] reports that a
  /// new route has been pushed to the application by the operating system.
  ///
  /// Consider using a [SynchronousFuture] if the result can be computed
  /// synchronously, so that the [Router] does not need to wait for the next
  /// microtask to schedule a build.
  Future<void> setNewRoutePath(T configuration);

  /// Called by the [Router] when the [Router.backButtonDispatcher] reports that
  /// the operating system is requesting that the current route be popped.
  ///
  /// The method should return a boolean [Future] to indicate whether this
  /// delegate handles the request. Returning false will cause the entire app
  /// to be popped.
  ///
  /// Consider using a [SynchronousFuture] if the result can be computed
  /// synchronously, so that the [Router] does not need to wait for the next
  /// microtask to schedule a build.
  Future<bool> popRoute();

  /// Called by the [Router] when it detects a route information may have
  /// changed as a result of rebuild.
  ///
  /// If this getter returns non-null, the [Router] will start to report new
  /// route information back to the engine. In web applications, the new
  /// route information is used for populating browser history in order to
  /// support the forward and the backward buttons.
  ///
  /// When overriding this method, the configuration returned by this getter
  /// must be able to construct the current app state and build the widget
  /// with the same configuration in the [build] method if it is passed back
  /// to the [setNewRoutePath]. Otherwise, the browser backward and forward
  /// buttons will not work properly.
  ///
  /// By default, this getter returns null, which prevents the [Router] from
  /// reporting the route information. To opt in, a subclass can override this
  /// getter to return the current configuration.
  ///
  /// At most one [Router] can opt in to route information reporting. Typically,
  /// only the top-most [Router] created by [WidgetsApp.router] should opt for
  /// route information reporting.
  ///
  /// ## State Restoration
  ///
  /// This getter is also used by the [Router] to implement state restoration.
  /// During state serialization, the [Router] will persist the current
  /// configuration and during state restoration pass it back to the delegate
  /// by calling [setRestoredRoutePath].
  T? get currentConfiguration => null;

  /// Called by the [Router] to obtain the widget tree that represents the
  /// current state.
  ///
  /// This is called whenever the [Future]s returned by [setInitialRoutePath],
  /// [setNewRoutePath], or [setRestoredRoutePath] complete as well as when this
  /// notifies its clients (see the [Listenable] interface, which this interface
  /// includes). In addition, it may be called at other times. It is important,
  /// therefore, that the methods above do not update the state that the [build]
  /// method uses before they complete their respective futures.
  ///
  /// Typically this method returns a suitably-configured [Navigator]. If you do
  /// plan to create a navigator, consider using the
  /// [PopNavigatorRouterDelegateMixin]. If state restoration is enabled for the
  /// [Router] using this delegate, consider providing a non-null
  /// [Navigator.restorationScopeId] to the [Navigator] returned by this method.
  ///
  /// This method must not return null.
  ///
  /// The `context` is the [Router]'s build context.
  Widget build(BuildContext context);
}

/// A route information provider that provides route information for the
/// [Router] widget
///
/// This provider is responsible for handing the route information through [value]
/// getter and notifies listeners, typically the [Router] widget, when a new
/// route information is available.
///
/// When the router opts for route information reporting (by overriding the
/// [RouterDelegate.currentConfiguration] to return non-null), override the
/// [routerReportsNewRouteInformation] method to process the route information.
///
/// See also:
///
///  * [PlatformRouteInformationProvider], which wires up the itself with the
///    [WidgetsBindingObserver.didPushRoute] to propagate platform push route
///    intent to the [Router] widget, as well as reports new route information
///    from the [Router] back to the engine by overriding the
///    [routerReportsNewRouteInformation].
abstract class RouteInformationProvider extends ValueListenable<RouteInformation> {
  /// A callback called when the [Router] widget reports new route information
  ///
  /// The subclasses can override this method to update theirs values or trigger
  /// other side effects. For example, the [PlatformRouteInformationProvider]
  /// overrides this method to report the route information back to the engine.
  ///
  /// The `routeInformation` is the new route information generated by the
  /// Router rebuild, and it can be the same or different from the
  /// [value].
  ///
  /// The `type` denotes the [Router]'s intention when it reports this
  /// `routeInformation`. It is useful when deciding how to update the internal
  /// state of [RouteInformationProvider] subclass with the `routeInformation`.
  /// For example, [PlatformRouteInformationProvider] uses this property to
  /// decide whether to push or replace the browser history entry with the new
  /// `routeInformation`.
  ///
  /// For more information on how [Router] determines a navigation event, see
  /// the "URL updates for web applications" section in the [Router]
  /// documentation.
  void routerReportsNewRouteInformation(
    RouteInformation routeInformation, {
    RouteInformationReportingType type = RouteInformationReportingType.none,
  }) {}
}

/// The route information provider that propagates the platform route information changes.
///
/// This provider also reports the new route information from the [Router] widget
/// back to engine using message channel method, the
/// [SystemNavigator.routeInformationUpdated].
///
/// Each time [SystemNavigator.routeInformationUpdated] is called, the
/// [SystemNavigator.selectMultiEntryHistory] method is also called. This
/// overrides the initialization behavior of
/// [Navigator.reportsRouteUpdateToEngine].
class PlatformRouteInformationProvider extends RouteInformationProvider
    with WidgetsBindingObserver, ChangeNotifier {
  /// Create a platform route information provider.
  ///
  /// Use the [initialRouteInformation] to set the default route information for this
  /// provider.
  PlatformRouteInformationProvider({required RouteInformation initialRouteInformation})
    : _value = initialRouteInformation {
    if (kFlutterMemoryAllocationsEnabled) {
      ChangeNotifier.maybeDispatchObjectCreation(this);
    }
  }

  static bool _equals(Uri a, Uri b) {
    return a.path == b.path &&
        a.fragment == b.fragment &&
        const DeepCollectionEquality.unordered().equals(a.queryParametersAll, b.queryParametersAll);
  }

  @override
  void routerReportsNewRouteInformation(
    RouteInformation routeInformation, {
    RouteInformationReportingType type = RouteInformationReportingType.none,
  }) {
    SystemNavigator.selectMultiEntryHistory();
    SystemNavigator.routeInformationUpdated(
      uri: routeInformation.uri,
      state: routeInformation.state,
      replace: switch (type) {
        RouteInformationReportingType.neglect => true,
        RouteInformationReportingType.navigate => false,
        RouteInformationReportingType.none => _equals(_valueInEngine.uri, routeInformation.uri),
      },
    );
    _value = routeInformation;
    _valueInEngine = routeInformation;
  }

  @override
  RouteInformation get value => _value;
  RouteInformation _value;

  RouteInformation _valueInEngine = RouteInformation(
    uri: Uri.parse(WidgetsBinding.instance.platformDispatcher.defaultRouteName),
  );

  void _platformReportsNewRouteInformation(RouteInformation routeInformation) {
    if (_value == routeInformation) {
      return;
    }
    _value = routeInformation;
    _valueInEngine = routeInformation;
    notifyListeners();
  }

  @override
  void addListener(VoidCallback listener) {
    if (!hasListeners) {
      WidgetsBinding.instance.addObserver(this);
    }
    super.addListener(listener);
  }

  @override
  void removeListener(VoidCallback listener) {
    super.removeListener(listener);
    if (!hasListeners) {
      WidgetsBinding.instance.removeObserver(this);
    }
  }

  @override
  void dispose() {
    // In practice, this will rarely be called. We assume that the listeners
    // will be added and removed in a coherent fashion such that when the object
    // is no longer being used, there's no listener, and so it will get garbage
    // collected.
    if (hasListeners) {
      WidgetsBinding.instance.removeObserver(this);
    }
    super.dispose();
  }

  @override
  Future<bool> didPushRouteInformation(RouteInformation routeInformation) async {
    assert(hasListeners);
    _platformReportsNewRouteInformation(routeInformation);
    return true;
  }
}

/// A mixin that wires [RouterDelegate.popRoute] to the [Navigator] it builds.
///
/// This mixin calls [Navigator.maybePop] when it receives an Android back
/// button intent through the [RouterDelegate.popRoute]. Using this mixin
/// guarantees that the back button still respects pageless routes in the
/// navigator.
///
/// Only use this mixin if you plan to build a navigator in the
/// [RouterDelegate.build].
mixin PopNavigatorRouterDelegateMixin<T> on RouterDelegate<T> {
  /// The key used for retrieving the current navigator.
  ///
  /// When using this mixin, be sure to use this key to create the navigator.
  GlobalKey<NavigatorState>? get navigatorKey;

  @override
  Future<bool> popRoute() {
    final NavigatorState? navigator = navigatorKey?.currentState;
    return navigator?.maybePop() ?? SynchronousFuture<bool>(false);
  }
}

class _RestorableRouteInformation extends RestorableValue<RouteInformation?> {
  @override
  RouteInformation? createDefaultValue() => null;

  @override
  void didUpdateValue(RouteInformation? oldValue) {
    notifyListeners();
  }

  @override
  RouteInformation? fromPrimitives(Object? data) {
    if (data == null) {
      return null;
    }
    assert(data is List<Object?> && data.length == 2);
    final List<Object?> castedData = data as List<Object?>;
    final String? uri = castedData.first as String?;
    if (uri == null) {
      return null;
    }
    return RouteInformation(uri: Uri.parse(uri), state: castedData.last);
  }

  @override
  Object? toPrimitives() {
    return value == null ? null : <Object?>[value!.uri.toString(), value!.state];
  }
}
