// Copyright 2014 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

/// @docImport 'dart:ui';
/// @docImport 'package:flutter/animation.dart';
/// @docImport 'package:flutter/material.dart';
/// @docImport 'package:flutter_test/flutter_test.dart';
///
/// @docImport 'adapter.dart';
/// @docImport 'app_lifecycle_listener.dart';
/// @docImport 'basic.dart';
/// @docImport 'focus_scope.dart';
/// @docImport 'media_query.dart';
/// @docImport 'navigator.dart';
/// @docImport 'pop_scope.dart';
library;

import 'dart:async';
import 'dart:developer' as developer;
import 'dart:ui'
    show
        AccessibilityFeatures,
        AppExitResponse,
        AppLifecycleState,
        FrameTiming,
        Locale,
        PlatformDispatcher,
        TimingsCallback,
        ViewFocusEvent;

import 'package:flutter/foundation.dart';
import 'package:flutter/gestures.dart';
import 'package:flutter/rendering.dart';
import 'package:flutter/scheduler.dart';
import 'package:flutter/services.dart';

import 'app.dart';
import 'debug.dart';
import 'focus_manager.dart';
import 'framework.dart';
import 'platform_menu_bar.dart';
import 'router.dart';
import 'service_extensions.dart';
import 'view.dart';
import 'widget_inspector.dart';

export 'dart:ui' show AppLifecycleState, Locale;

// Examples can assume:
// late FlutterView myFlutterView;
// class MyApp extends StatelessWidget { const MyApp({super.key}); @override Widget build(BuildContext context) => const Placeholder(); }

/// Interface for classes that register with the Widgets layer binding.
///
/// This can be used by any class, not just widgets. It provides an interface
/// which is used by [WidgetsBinding.addObserver] and
/// [WidgetsBinding.removeObserver] to notify objects of changes in the
/// environment, such as changes to the device metrics or accessibility
/// settings. It is used to implement features such as [MediaQuery].
///
/// This class can be extended directly, or mixed in, to get default behaviors
/// for all of the handlers. Alternatively it can be used with the
/// `implements` keyword, in which case all the handlers must be implemented
/// (and the analyzer will list those that have been omitted).
///
/// To start receiving notifications, call `WidgetsBinding.instance.addObserver`
/// with a reference to the object implementing the [WidgetsBindingObserver]
/// interface. To avoid memory leaks, call
/// `WidgetsBinding.instance.removeObserver` to unregister the object when it
/// reaches the end of its lifecycle.
///
/// {@tool dartpad}
/// This sample shows how to implement parts of the [State] and
/// [WidgetsBindingObserver] protocols necessary to react to application
/// lifecycle messages. See [didChangeAppLifecycleState].
///
/// To respond to other notifications, replace the [didChangeAppLifecycleState]
/// method in this example with other methods from this class.
///
/// ** See code in examples/api/lib/widgets/binding/widget_binding_observer.0.dart **
/// {@end-tool}
abstract mixin class WidgetsBindingObserver {
  /// Called when the system tells the app to pop the current route, such as
  /// after a system back button press or back gesture.
  ///
  /// Observers are notified in registration order until one returns
  /// true. If none return true, the application quits.
  ///
  /// Observers are expected to return true if they were able to
  /// handle the notification, for example by closing an active dialog
  /// box, and false otherwise. The [WidgetsApp] widget uses this
  /// mechanism to notify the [Navigator] widget that it should pop
  /// its current route if possible.
  ///
  /// This method exposes the `popRoute` notification from
  /// [SystemChannels.navigation].
  ///
  /// {@macro flutter.widgets.AndroidPredictiveBack}
  Future<bool> didPopRoute() => Future<bool>.value(false);

  /// Called at the start of a predictive back gesture.
  ///
  /// Observers are notified in registration order until one returns true or all
  /// observers have been notified. If an observer returns true then that
  /// observer, and only that observer, will be notified of subsequent events in
  /// this same gesture (for example [handleUpdateBackGestureProgress], etc.).
  ///
  /// Observers are expected to return true if they were able to handle the
  /// notification, for example by starting a predictive back animation, and
  /// false otherwise. [PredictiveBackPageTransitionsBuilder] uses this
  /// mechanism to listen for predictive back gestures.
  ///
  /// If all observers indicate they are not handling this back gesture by
  /// returning false, then a navigation pop will result when
  /// [handleCommitBackGesture] is called, as in a non-predictive system back
  /// gesture.
  ///
  /// Currently, this is only used on Android devices that support the
  /// predictive back feature.
  bool handleStartBackGesture(PredictiveBackEvent backEvent) => false;

  /// Called when a predictive back gesture moves.
  ///
  /// The observer which was notified of this gesture's [handleStartBackGesture]
  /// is the same observer notified for this.
  ///
  /// Currently, this is only used on Android devices that support the
  /// predictive back feature.
  void handleUpdateBackGestureProgress(PredictiveBackEvent backEvent) {}

  /// Called when a predictive back gesture is finished successfully, indicating
  /// that the current route should be popped.
  ///
  /// The observer which was notified of this gesture's [handleStartBackGesture]
  /// is the same observer notified for this. If there is none, then a
  /// navigation pop will result, as in a non-predictive system back gesture.
  ///
  /// Currently, this is only used on Android devices that support the
  /// predictive back feature.
  void handleCommitBackGesture() {}

  /// Called when a predictive back gesture is canceled, indicating that no
  /// navigation should occur.
  ///
  /// The observer which was notified of this gesture's [handleStartBackGesture]
  /// is the same observer notified for this.
  ///
  /// Currently, this is only used on Android devices that support the
  /// predictive back feature.
  void handleCancelBackGesture() {}

  /// Called when the host tells the application to push a new route onto the
  /// navigator.
  ///
  /// Observers are expected to return true if they were able to
  /// handle the notification. Observers are notified in registration
  /// order until one returns true.
  ///
  /// This method exposes the `pushRoute` notification from
  /// [SystemChannels.navigation].
  @Deprecated(
    'Use didPushRouteInformation instead. '
    'This feature was deprecated after v3.8.0-14.0.pre.',
  )
  Future<bool> didPushRoute(String route) => Future<bool>.value(false);

  /// Called when the host tells the application to push a new
  /// [RouteInformation] and a restoration state onto the router.
  ///
  /// Observers are expected to return true if they were able to
  /// handle the notification. Observers are notified in registration
  /// order until one returns true.
  ///
  /// This method exposes the `pushRouteInformation` notification from
  /// [SystemChannels.navigation].
  ///
  /// The default implementation is to call the [didPushRoute] directly with the
  /// string constructed from [RouteInformation.uri]'s path and query parameters.
  // TODO(chunhtai): remove the default implementation once `didPushRoute` is
  // removed.
  Future<bool> didPushRouteInformation(RouteInformation routeInformation) {
    final Uri uri = routeInformation.uri;
    return didPushRoute(
      Uri.decodeComponent(
        Uri(
          path: uri.path.isEmpty ? '/' : uri.path,
          queryParameters: uri.queryParametersAll.isEmpty ? null : uri.queryParametersAll,
          fragment: uri.fragment.isEmpty ? null : uri.fragment,
        ).toString(),
      ),
    );
  }

  /// Called when the application's dimensions change. For example,
  /// when a phone is rotated.
  ///
  /// This method exposes notifications from
  /// [dart:ui.PlatformDispatcher.onMetricsChanged].
  ///
  /// {@tool snippet}
  ///
  /// This [StatefulWidget] implements the parts of the [State] and
  /// [WidgetsBindingObserver] protocols necessary to react when the device is
  /// rotated (or otherwise changes dimensions).
  ///
  /// ```dart
  /// class MetricsReactor extends StatefulWidget {
  ///   const MetricsReactor({ super.key });
  ///
  ///   @override
  ///   State<MetricsReactor> createState() => _MetricsReactorState();
  /// }
  ///
  /// class _MetricsReactorState extends State<MetricsReactor> with WidgetsBindingObserver {
  ///   late Size _lastSize;
  ///
  ///   @override
  ///   void initState() {
  ///     super.initState();
  ///     WidgetsBinding.instance.addObserver(this);
  ///   }
  ///
  ///   @override
  ///   void didChangeDependencies() {
  ///     super.didChangeDependencies();
  ///     // [View.of] exposes the view from `WidgetsBinding.instance.platformDispatcher.views`
  ///     // into which this widget is drawn.
  ///     _lastSize = View.of(context).physicalSize;
  ///   }
  ///
  ///   @override
  ///   void dispose() {
  ///     WidgetsBinding.instance.removeObserver(this);
  ///     super.dispose();
  ///   }
  ///
  ///   @override
  ///   void didChangeMetrics() {
  ///     setState(() { _lastSize = View.of(context).physicalSize; });
  ///   }
  ///
  ///   @override
  ///   Widget build(BuildContext context) {
  ///     return Text('Current size: $_lastSize');
  ///   }
  /// }
  /// ```
  /// {@end-tool}
  ///
  /// In general, this is unnecessary as the layout system takes care of
  /// automatically recomputing the application geometry when the application
  /// size changes.
  ///
  /// See also:
  ///
  ///  * [MediaQuery.sizeOf], which provides a similar service with less
  ///    boilerplate.
  void didChangeMetrics() {}

  /// Called when the platform's text scale factor changes.
  ///
  /// This typically happens as the result of the user changing system
  /// preferences, and it should affect all of the text sizes in the
  /// application.
  ///
  /// This method exposes notifications from
  /// [dart:ui.PlatformDispatcher.onTextScaleFactorChanged].
  ///
  /// {@tool snippet}
  ///
  /// ```dart
  /// class TextScaleFactorReactor extends StatefulWidget {
  ///   const TextScaleFactorReactor({ super.key });
  ///
  ///   @override
  ///   State<TextScaleFactorReactor> createState() => _TextScaleFactorReactorState();
  /// }
  ///
  /// class _TextScaleFactorReactorState extends State<TextScaleFactorReactor> with WidgetsBindingObserver {
  ///   @override
  ///   void initState() {
  ///     super.initState();
  ///     WidgetsBinding.instance.addObserver(this);
  ///   }
  ///
  ///   @override
  ///   void dispose() {
  ///     WidgetsBinding.instance.removeObserver(this);
  ///     super.dispose();
  ///   }
  ///
  ///   late double _lastTextScaleFactor;
  ///
  ///   @override
  ///   void didChangeTextScaleFactor() {
  ///     setState(() { _lastTextScaleFactor = WidgetsBinding.instance.platformDispatcher.textScaleFactor; });
  ///   }
  ///
  ///   @override
  ///   Widget build(BuildContext context) {
  ///     return Text('Current scale factor: $_lastTextScaleFactor');
  ///   }
  /// }
  /// ```
  /// {@end-tool}
  ///
  /// See also:
  ///
  ///  * [MediaQuery.textScaleFactorOf], which provides a similar service with less
  ///    boilerplate.
  void didChangeTextScaleFactor() {}

  /// Called when the platform brightness changes.
  ///
  /// This method exposes notifications from
  /// [dart:ui.PlatformDispatcher.onPlatformBrightnessChanged].
  ///
  /// See also:
  ///
  /// * [MediaQuery.platformBrightnessOf], which provides a similar service with
  ///   less boilerplate.
  void didChangePlatformBrightness() {}

  /// Called when the system tells the app that the user's locale has
  /// changed. For example, if the user changes the system language
  /// settings.
  ///
  /// This method exposes notifications from
  /// [dart:ui.PlatformDispatcher.onLocaleChanged].
  void didChangeLocales(List<Locale>? locales) {}

  /// Called when the system puts the app in the background or returns
  /// the app to the foreground.
  ///
  /// An example of implementing this method is provided in the class-level
  /// documentation for the [WidgetsBindingObserver] class.
  ///
  /// This method exposes notifications from [SystemChannels.lifecycle].
  ///
  /// See also:
  ///
  ///  * [AppLifecycleListener], an alternative API for responding to
  ///    application lifecycle changes.
  void didChangeAppLifecycleState(AppLifecycleState state) {}

  /// Called whenever the [PlatformDispatcher] receives a notification that the
  /// focus state on a view has changed.
  ///
  /// The [event] contains the view ID for the view that changed its focus
  /// state.
  ///
  /// The view ID of the [FlutterView] in which a particular [BuildContext]
  /// resides can be retrieved with `View.of(context).viewId`, so that it may be
  /// compared with the view ID in the `event` to see if the event pertains to
  /// the given context.
  void didChangeViewFocus(ViewFocusEvent event) {}

  /// Called when a request is received from the system to exit the application.
  ///
  /// If any observer responds with [AppExitResponse.cancel], it will cancel the
  /// exit. All observers will be asked before exiting.
  ///
  /// {@macro flutter.services.binding.ServicesBinding.requestAppExit}
  ///
  /// See also:
  ///
  /// * [ServicesBinding.exitApplication] for a function to call that will request
  ///   that the application exits.
  Future<AppExitResponse> didRequestAppExit() async {
    return AppExitResponse.exit;
  }

  /// Called when the system is running low on memory.
  ///
  /// This method exposes the `memoryPressure` notification from
  /// [SystemChannels.system].
  void didHaveMemoryPressure() {}

  /// Called when the system changes the set of currently active accessibility
  /// features.
  ///
  /// This method exposes notifications from
  /// [dart:ui.PlatformDispatcher.onAccessibilityFeaturesChanged].
  void didChangeAccessibilityFeatures() {}
}

/// The glue between the widgets layer and the Flutter engine.
///
/// The [WidgetsBinding] manages a single [Element] tree rooted at [rootElement].
/// Calling [runApp] (which indirectly calls [attachRootWidget]) bootstraps that
/// element tree.
///
/// ## Relationship to render trees
///
/// Multiple render trees may be associated with the element tree. Those are
/// managed by the underlying [RendererBinding].
///
/// The element tree is segmented into two types of zones: rendering zones and
/// non-rendering zones.
///
/// A rendering zone is a part of the element tree that is backed by a render
/// tree and it describes the pixels that are drawn on screen. For elements in
/// this zone, [Element.renderObject] never returns null because the elements
/// are all associated with [RenderObject]s. Almost all widgets can be placed in
/// a rendering zone; notable exceptions are the [View] widget, [ViewCollection]
/// widget, and [RootWidget].
///
/// A non-rendering zone is a part of the element tree that is not backed by a
/// render tree. For elements in this zone, [Element.renderObject] returns null
/// because the elements are not associated with any [RenderObject]s. Only
/// widgets that do not produce a [RenderObject] can be used in this zone
/// because there is no render tree to attach the render object to. In other
/// words, [RenderObjectWidget]s cannot be used in this zone. Typically, one
/// would find [InheritedWidget]s, [View]s, and [ViewCollection]s in this zone
/// to inject data across rendering zones into the tree and to organize the
/// rendering zones (and by extension their associated render trees) into a
/// unified element tree.
///
/// The root of the element tree at [rootElement] starts a non-rendering zone.
/// Within a non-rendering zone, the [View] widget is used to start a rendering
/// zone by bootstrapping a render tree. Within a rendering zone, the
/// [ViewAnchor] can be used to start a new non-rendering zone.
///
// TODO(goderbauer): Include an example graph showcasing the different zones.
///
/// To figure out if an element is in a rendering zone it may walk up the tree
/// calling [Element.debugExpectsRenderObjectForSlot] on its ancestors. If it
/// reaches an element that returns false, it is in a non-rendering zone. If it
/// reaches a [RenderObjectElement] ancestor it is in a rendering zone.
mixin WidgetsBinding
    on
        BindingBase,
        ServicesBinding,
        SchedulerBinding,
        GestureBinding,
        RendererBinding,
        SemanticsBinding {
  @override
  void initInstances() {
    super.initInstances();
    _instance = this;

    assert(() {
      _debugAddStackFilters();
      return true;
    }());

    // Initialization of [_buildOwner] has to be done after
    // [super.initInstances] is called, as it requires [ServicesBinding] to
    // properly setup the [defaultBinaryMessenger] instance.
    _buildOwner = BuildOwner();
    buildOwner!.onBuildScheduled = _handleBuildScheduled;
    platformDispatcher.onLocaleChanged = handleLocaleChanged;
    SystemChannels.navigation.setMethodCallHandler(_handleNavigationInvocation);
    SystemChannels.backGesture.setMethodCallHandler(_handleBackGestureInvocation);
    assert(() {
      FlutterErrorDetails.propertiesTransformers.add(debugTransformDebugCreator);
      return true;
    }());
    platformMenuDelegate = DefaultPlatformMenuDelegate();
  }

  /// The current [WidgetsBinding], if one has been created.
  ///
  /// Provides access to the features exposed by this mixin. The binding must
  /// be initialized before using this getter; this is typically done by calling
  /// [runApp] or [WidgetsFlutterBinding.ensureInitialized].
  static WidgetsBinding get instance => BindingBase.checkInstance(_instance);
  static WidgetsBinding? _instance;

  /// If true, forces the widget inspector to be visible.
  ///
  /// Overrides the `debugShowWidgetInspector` value set in [WidgetsApp].
  ///
  /// Used by the `debugShowWidgetInspector` debugging extension.
  ///
  /// The inspector allows the selection of a location on your device or emulator
  /// and view what widgets and render objects associated with it. An outline of
  /// the selected widget and some summary information is shown on device and
  /// more detailed information is shown in the IDE or DevTools.
  bool get debugShowWidgetInspectorOverride {
    return debugShowWidgetInspectorOverrideNotifier.value;
  }

  set debugShowWidgetInspectorOverride(bool value) {
    debugShowWidgetInspectorOverrideNotifier.value = value;
  }

  /// Notifier for [debugShowWidgetInspectorOverride].
  ValueNotifier<bool> get debugShowWidgetInspectorOverrideNotifier =>
      _debugShowWidgetInspectorOverrideNotifierObject ??= ValueNotifier<bool>(false);
  ValueNotifier<bool>? _debugShowWidgetInspectorOverrideNotifierObject;

  @visibleForTesting
  @override
  void resetInternalState() {
    // ignore: invalid_use_of_visible_for_testing_member, https://github.com/dart-lang/sdk/issues/41998
    super.resetInternalState();
    _debugShowWidgetInspectorOverrideNotifierObject?.dispose();
    _debugShowWidgetInspectorOverrideNotifierObject = null;
  }

  void _debugAddStackFilters() {
    const PartialStackFrame elementInflateWidget = PartialStackFrame(
      package: 'package:flutter/src/widgets/framework.dart',
      className: 'Element',
      method: 'inflateWidget',
    );
    const PartialStackFrame elementUpdateChild = PartialStackFrame(
      package: 'package:flutter/src/widgets/framework.dart',
      className: 'Element',
      method: 'updateChild',
    );
    const PartialStackFrame elementRebuild = PartialStackFrame(
      package: 'package:flutter/src/widgets/framework.dart',
      className: 'Element',
      method: 'rebuild',
    );
    const PartialStackFrame componentElementPerformRebuild = PartialStackFrame(
      package: 'package:flutter/src/widgets/framework.dart',
      className: 'ComponentElement',
      method: 'performRebuild',
    );
    const PartialStackFrame componentElementFirstBuild = PartialStackFrame(
      package: 'package:flutter/src/widgets/framework.dart',
      className: 'ComponentElement',
      method: '_firstBuild',
    );
    const PartialStackFrame componentElementMount = PartialStackFrame(
      package: 'package:flutter/src/widgets/framework.dart',
      className: 'ComponentElement',
      method: 'mount',
    );
    const PartialStackFrame statefulElementFirstBuild = PartialStackFrame(
      package: 'package:flutter/src/widgets/framework.dart',
      className: 'StatefulElement',
      method: '_firstBuild',
    );
    const PartialStackFrame singleChildMount = PartialStackFrame(
      package: 'package:flutter/src/widgets/framework.dart',
      className: 'SingleChildRenderObjectElement',
      method: 'mount',
    );
    const PartialStackFrame statefulElementRebuild = PartialStackFrame(
      package: 'package:flutter/src/widgets/framework.dart',
      className: 'StatefulElement',
      method: 'performRebuild',
    );

    const String replacementString = '...     Normal element mounting';

    // ComponentElement variations
    FlutterError.addDefaultStackFilter(
      const RepetitiveStackFrameFilter(
        frames: <PartialStackFrame>[
          elementInflateWidget,
          elementUpdateChild,
          componentElementPerformRebuild,
          elementRebuild,
          componentElementFirstBuild,
          componentElementMount,
        ],
        replacement: replacementString,
      ),
    );
    FlutterError.addDefaultStackFilter(
      const RepetitiveStackFrameFilter(
        frames: <PartialStackFrame>[
          elementUpdateChild,
          componentElementPerformRebuild,
          elementRebuild,
          componentElementFirstBuild,
          componentElementMount,
        ],
        replacement: replacementString,
      ),
    );

    // StatefulElement variations
    FlutterError.addDefaultStackFilter(
      const RepetitiveStackFrameFilter(
        frames: <PartialStackFrame>[
          elementInflateWidget,
          elementUpdateChild,
          componentElementPerformRebuild,
          statefulElementRebuild,
          elementRebuild,
          componentElementFirstBuild,
          statefulElementFirstBuild,
          componentElementMount,
        ],
        replacement: replacementString,
      ),
    );
    FlutterError.addDefaultStackFilter(
      const RepetitiveStackFrameFilter(
        frames: <PartialStackFrame>[
          elementUpdateChild,
          componentElementPerformRebuild,
          statefulElementRebuild,
          elementRebuild,
          componentElementFirstBuild,
          statefulElementFirstBuild,
          componentElementMount,
        ],
        replacement: replacementString,
      ),
    );

    // SingleChildRenderObjectElement variations
    FlutterError.addDefaultStackFilter(
      const RepetitiveStackFrameFilter(
        frames: <PartialStackFrame>[elementInflateWidget, elementUpdateChild, singleChildMount],
        replacement: replacementString,
      ),
    );
    FlutterError.addDefaultStackFilter(
      const RepetitiveStackFrameFilter(
        frames: <PartialStackFrame>[elementUpdateChild, singleChildMount],
        replacement: replacementString,
      ),
    );
  }

  @override
  void initServiceExtensions() {
    super.initServiceExtensions();

    if (!kReleaseMode) {
      registerServiceExtension(
        name: WidgetsServiceExtensions.debugDumpApp.name,
        callback: (Map<String, String> parameters) async {
          final String data = _debugDumpAppString();
          return <String, Object>{'data': data};
        },
      );

      registerServiceExtension(
        name: WidgetsServiceExtensions.debugDumpFocusTree.name,
        callback: (Map<String, String> parameters) async {
          final String data = focusManager.toStringDeep();
          return <String, Object>{'data': data};
        },
      );

      if (!kIsWeb) {
        registerBoolServiceExtension(
          name: WidgetsServiceExtensions.showPerformanceOverlay.name,
          getter: () => Future<bool>.value(WidgetsApp.showPerformanceOverlayOverride),
          setter: (bool value) {
            if (WidgetsApp.showPerformanceOverlayOverride == value) {
              return Future<void>.value();
            }
            WidgetsApp.showPerformanceOverlayOverride = value;
            return _forceRebuild();
          },
        );
      }

      registerServiceExtension(
        name: WidgetsServiceExtensions.didSendFirstFrameEvent.name,
        callback: (_) async {
          return <String, dynamic>{
            // This is defined to return a STRING, not a boolean.
            // Devtools, the Intellij plugin, and the flutter tool all depend
            // on it returning a string and not a boolean.
            'enabled': _needToReportFirstFrame ? 'false' : 'true',
          };
        },
      );

      registerServiceExtension(
        name: WidgetsServiceExtensions.didSendFirstFrameRasterizedEvent.name,
        callback: (_) async {
          return <String, dynamic>{
            // This is defined to return a STRING, not a boolean.
            // Devtools, the Intellij plugin, and the flutter tool all depend
            // on it returning a string and not a boolean.
            'enabled': firstFrameRasterized ? 'true' : 'false',
          };
        },
      );

      // Expose the ability to send Widget rebuilds as [Timeline] events.
      registerBoolServiceExtension(
        name: WidgetsServiceExtensions.profileWidgetBuilds.name,
        getter: () async => debugProfileBuildsEnabled,
        setter: (bool value) async {
          debugProfileBuildsEnabled = value;
        },
      );
      registerBoolServiceExtension(
        name: WidgetsServiceExtensions.profileUserWidgetBuilds.name,
        getter: () async => debugProfileBuildsEnabledUserWidgets,
        setter: (bool value) async {
          debugProfileBuildsEnabledUserWidgets = value;
        },
      );
    }

    assert(() {
      registerBoolServiceExtension(
        name: WidgetsServiceExtensions.debugAllowBanner.name,
        getter: () => Future<bool>.value(WidgetsApp.debugAllowBannerOverride),
        setter: (bool value) {
          if (WidgetsApp.debugAllowBannerOverride == value) {
            return Future<void>.value();
          }
          WidgetsApp.debugAllowBannerOverride = value;
          return _forceRebuild();
        },
      );

      WidgetInspectorService.instance.initServiceExtensions(registerServiceExtension);

      return true;
    }());
  }

  Future<void> _forceRebuild() {
    if (rootElement != null) {
      buildOwner!.reassemble(rootElement!);
      return endOfFrame;
    }
    return Future<void>.value();
  }

  /// The [BuildOwner] in charge of executing the build pipeline for the
  /// widget tree rooted at this binding.
  BuildOwner? get buildOwner => _buildOwner;
  // Initialization of [_buildOwner] has to be done within the [initInstances]
  // method, as it requires [ServicesBinding] to properly setup the
  // [defaultBinaryMessenger] instance.
  BuildOwner? _buildOwner;

  /// The object in charge of the focus tree.
  ///
  /// Rarely used directly. Instead, consider using [FocusScope.of] to obtain
  /// the [FocusScopeNode] for a given [BuildContext].
  ///
  /// See [FocusManager] for more details.
  FocusManager get focusManager => _buildOwner!.focusManager;

  /// A delegate that communicates with a platform plugin for serializing and
  /// managing platform-rendered menu bars created by [PlatformMenuBar].
  ///
  /// This is set by default to a [DefaultPlatformMenuDelegate] instance in
  /// [initInstances].
  late PlatformMenuDelegate platformMenuDelegate;

  final List<WidgetsBindingObserver> _observers = <WidgetsBindingObserver>[];

  /// Registers the given object as a binding observer. Binding
  /// observers are notified when various application events occur,
  /// for example when the system locale changes. Generally, one
  /// widget in the widget tree registers itself as a binding
  /// observer, and converts the system state into inherited widgets.
  ///
  /// For example, the [WidgetsApp] widget registers as a binding
  /// observer and passes the screen size to a [MediaQuery] widget
  /// each time it is built, which enables other widgets to use the
  /// [MediaQuery.sizeOf] static method and (implicitly) the
  /// [InheritedWidget] mechanism to be notified whenever the screen
  /// size changes (e.g. whenever the screen rotates).
  ///
  /// See also:
  ///
  ///  * [removeObserver], to release the resources reserved by this method.
  ///  * [WidgetsBindingObserver], which has an example of using this method.
  void addObserver(WidgetsBindingObserver observer) => _observers.add(observer);

  /// Unregisters the given observer. This should be used sparingly as
  /// it is relatively expensive (O(N) in the number of registered
  /// observers).
  ///
  /// See also:
  ///
  ///  * [addObserver], for the method that adds observers in the first place.
  ///  * [WidgetsBindingObserver], which has an example of using this method.
  bool removeObserver(WidgetsBindingObserver observer) {
    if (observer == _backGestureObserver) {
      _backGestureObserver = null;
    }
    return _observers.remove(observer);
  }

  @override
  Future<AppExitResponse> handleRequestAppExit() async {
    bool didCancel = false;
    for (final WidgetsBindingObserver observer in List<WidgetsBindingObserver>.of(_observers)) {
      if ((await observer.didRequestAppExit()) == AppExitResponse.cancel) {
        didCancel = true;
        // Don't early return. For the case where someone is just using the
        // observer to know when exit happens, we want to call all the
        // observers, even if we already know we're going to cancel.
      }
    }
    return didCancel ? AppExitResponse.cancel : AppExitResponse.exit;
  }

  @override
  void handleMetricsChanged() {
    super.handleMetricsChanged();
    for (final WidgetsBindingObserver observer in List<WidgetsBindingObserver>.of(_observers)) {
      observer.didChangeMetrics();
    }
  }

  @override
  void handleTextScaleFactorChanged() {
    super.handleTextScaleFactorChanged();
    for (final WidgetsBindingObserver observer in List<WidgetsBindingObserver>.of(_observers)) {
      observer.didChangeTextScaleFactor();
    }
  }

  @override
  void handlePlatformBrightnessChanged() {
    super.handlePlatformBrightnessChanged();
    for (final WidgetsBindingObserver observer in List<WidgetsBindingObserver>.of(_observers)) {
      observer.didChangePlatformBrightness();
    }
  }

  @override
  void handleAccessibilityFeaturesChanged() {
    super.handleAccessibilityFeaturesChanged();
    for (final WidgetsBindingObserver observer in List<WidgetsBindingObserver>.of(_observers)) {
      observer.didChangeAccessibilityFeatures();
    }
  }

  /// Called when the system locale changes.
  ///
  /// Calls [dispatchLocalesChanged] to notify the binding observers.
  ///
  /// See [dart:ui.PlatformDispatcher.onLocaleChanged].
  @protected
  @mustCallSuper
  @visibleForTesting
  void handleLocaleChanged() {
    dispatchLocalesChanged(platformDispatcher.locales);
  }

  /// Notify all the observers that the locale has changed (using
  /// [WidgetsBindingObserver.didChangeLocales]), giving them the
  /// `locales` argument.
  ///
  /// This is called by [handleLocaleChanged] when the
  /// [PlatformDispatcher.onLocaleChanged] notification is received.
  @protected
  @mustCallSuper
  void dispatchLocalesChanged(List<Locale>? locales) {
    for (final WidgetsBindingObserver observer in List<WidgetsBindingObserver>.of(_observers)) {
      observer.didChangeLocales(locales);
    }
  }

  /// Notify all the observers that the active set of [AccessibilityFeatures]
  /// has changed (using [WidgetsBindingObserver.didChangeAccessibilityFeatures]),
  /// giving them the `features` argument.
  ///
  /// This is called by [handleAccessibilityFeaturesChanged] when the
  /// [PlatformDispatcher.onAccessibilityFeaturesChanged] notification is received.
  @protected
  @mustCallSuper
  void dispatchAccessibilityFeaturesChanged() {
    for (final WidgetsBindingObserver observer in List<WidgetsBindingObserver>.of(_observers)) {
      observer.didChangeAccessibilityFeatures();
    }
  }

  /// Called when the system pops the current route.
  ///
  /// This first notifies the binding observers (using
  /// [WidgetsBindingObserver.didPopRoute]), in registration order, until one
  /// returns true, meaning that it was able to handle the request (e.g. by
  /// closing a dialog box). If none return true, then the application is shut
  /// down by calling [SystemNavigator.pop].
  ///
  /// [WidgetsApp] uses this in conjunction with a [Navigator] to
  /// cause the back button to close dialog boxes, return from modal
  /// pages, and so forth.
  ///
  /// This method exposes the `popRoute` notification from
  /// [SystemChannels.navigation].
  ///
  /// {@template flutter.widgets.AndroidPredictiveBack}
  /// ## Handling backs ahead of time
  ///
  /// Not all system backs will result in a call to this method. Some are
  /// handled entirely by the system without informing the Flutter framework.
  ///
  /// Android API 33+ introduced a feature called predictive back, which allows
  /// the user to peek behind the current app or route during a back gesture and
  /// then decide to cancel or commit the back. Flutter enables or disables this
  /// feature ahead of time, before a back gesture occurs, and back gestures
  /// that trigger predictive back are handled entirely by the system and do not
  /// trigger this method here in the framework.
  ///
  /// By default, the framework communicates when it would like to handle system
  /// back gestures using [SystemNavigator.setFrameworkHandlesBack] in
  /// [WidgetsApp]. This is done automatically based on the status of the
  /// [Navigator] stack and the state of any [PopScope] widgets present.
  /// Developers can manually set this by calling the method directly or by
  /// using [NavigationNotification].
  /// {@endtemplate}
  @protected
  @visibleForTesting
  Future<bool> handlePopRoute() async {
    for (final WidgetsBindingObserver observer in List<WidgetsBindingObserver>.of(_observers)) {
      if (await observer.didPopRoute()) {
        return true;
      }
    }
    SystemNavigator.pop();
    return false;
  }

  // The observer that is currently handling an active predictive back gesture.
  WidgetsBindingObserver? _backGestureObserver;

  Future<bool> _handleStartBackGesture(Map<String?, Object?> arguments) {
    _backGestureObserver = null;
    final PredictiveBackEvent backEvent = PredictiveBackEvent.fromMap(arguments);
    for (final WidgetsBindingObserver observer in List<WidgetsBindingObserver>.of(_observers)) {
      if (observer.handleStartBackGesture(backEvent)) {
        _backGestureObserver = observer;
        return Future<bool>.value(true);
      }
    }
    return Future<bool>.value(false);
  }

  Future<void> _handleUpdateBackGestureProgress(Map<String?, Object?> arguments) async {
    if (_backGestureObserver == null) {
      return;
    }

    final PredictiveBackEvent backEvent = PredictiveBackEvent.fromMap(arguments);
    _backGestureObserver!.handleUpdateBackGestureProgress(backEvent);
  }

  Future<void> _handleCommitBackGesture() async {
    if (_backGestureObserver == null) {
      // If the predictive back was not handled, then the route should be popped
      // like a normal, non-predictive back. For example, this will happen if a
      // back gesture occurs but no predictive back route transition exists to
      // handle it. The back gesture should still cause normal pop even if it
      // doesn't cause a predictive transition.
      await handlePopRoute();
      return;
    }
    _backGestureObserver?.handleCommitBackGesture();
  }

  Future<void> _handleCancelBackGesture() async {
    if (_backGestureObserver == null) {
      return;
    }

    _backGestureObserver!.handleCancelBackGesture();
  }

  /// Called when the host tells the app to push a new route onto the
  /// navigator.
  ///
  /// This notifies the binding observers (using
  /// [WidgetsBindingObserver.didPushRoute]), in registration order, until one
  /// returns true, meaning that it was able to handle the request (e.g. by
  /// opening a dialog box). If none return true, then nothing happens.
  ///
  /// This method exposes the `pushRoute` notification from
  /// [SystemChannels.navigation].
  @protected
  @mustCallSuper
  @visibleForTesting
  Future<bool> handlePushRoute(String route) async {
    final RouteInformation routeInformation = RouteInformation(uri: Uri.parse(route));
    for (final WidgetsBindingObserver observer in List<WidgetsBindingObserver>.of(_observers)) {
      if (await observer.didPushRouteInformation(routeInformation)) {
        return true;
      }
    }
    return false;
  }

  Future<bool> _handlePushRouteInformation(Map<dynamic, dynamic> routeArguments) async {
    final RouteInformation routeInformation = RouteInformation(
      uri: Uri.parse(routeArguments['location'] as String),
      state: routeArguments['state'] as Object?,
    );
    for (final WidgetsBindingObserver observer in List<WidgetsBindingObserver>.of(_observers)) {
      if (await observer.didPushRouteInformation(routeInformation)) {
        return true;
      }
    }
    return false;
  }

  Future<bool> _handleNavigationInvocation(MethodCall methodCall) {
    return switch (methodCall.method) {
      'popRoute' => handlePopRoute(),
      'pushRoute' => handlePushRoute(methodCall.arguments as String),
      'pushRouteInformation' => _handlePushRouteInformation(
        methodCall.arguments as Map<dynamic, dynamic>,
      ),
      // Return false for unhandled method.
      _ => Future<bool>.value(false),
    };
  }

  Future<dynamic> _handleBackGestureInvocation(MethodCall methodCall) {
    final Map<String?, Object?>? arguments =
        (methodCall.arguments as Map<Object?, Object?>?)?.cast<String?, Object?>();
    return switch (methodCall.method) {
      'startBackGesture' => _handleStartBackGesture(arguments!),
      'updateBackGestureProgress' => _handleUpdateBackGestureProgress(arguments!),
      'commitBackGesture' => _handleCommitBackGesture(),
      'cancelBackGesture' => _handleCancelBackGesture(),
      _ => throw MissingPluginException(),
    };
  }

  @override
  void handleAppLifecycleStateChanged(AppLifecycleState state) {
    super.handleAppLifecycleStateChanged(state);
    for (final WidgetsBindingObserver observer in List<WidgetsBindingObserver>.of(_observers)) {
      observer.didChangeAppLifecycleState(state);
    }
  }

  @override
  void handleViewFocusChanged(ViewFocusEvent event) {
    super.handleViewFocusChanged(event);
    for (final WidgetsBindingObserver observer in List<WidgetsBindingObserver>.of(_observers)) {
      observer.didChangeViewFocus(event);
    }
  }

  @override
  void handleMemoryPressure() {
    super.handleMemoryPressure();
    for (final WidgetsBindingObserver observer in List<WidgetsBindingObserver>.of(_observers)) {
      observer.didHaveMemoryPressure();
    }
  }

  bool _needToReportFirstFrame = true;

  final Completer<void> _firstFrameCompleter = Completer<void>();

  /// Whether the Flutter engine has rasterized the first frame.
  ///
  /// Usually, the time that a frame is rasterized is very close to the time that
  /// it gets presented on the display. Specifically, rasterization is the last
  /// expensive phase of a frame that's still in Flutter's control.
  ///
  /// See also:
  ///
  ///  * [waitUntilFirstFrameRasterized], the future when [firstFrameRasterized]
  ///    becomes true.
  bool get firstFrameRasterized => _firstFrameCompleter.isCompleted;

  /// A future that completes when the Flutter engine has rasterized the first
  /// frame.
  ///
  /// Usually, the time that a frame is rasterized is very close to the time that
  /// it gets presented on the display. Specifically, rasterization is the last
  /// expensive phase of a frame that's still in Flutter's control.
  ///
  /// See also:
  ///
  ///  * [firstFrameRasterized], whether this future has completed or not.
  Future<void> get waitUntilFirstFrameRasterized => _firstFrameCompleter.future;

  /// Whether the first frame has finished building.
  ///
  /// This value can also be obtained over the VM service protocol as
  /// `ext.flutter.didSendFirstFrameEvent`.
  ///
  /// See also:
  ///
  ///  * [firstFrameRasterized], whether the first frame has finished rendering.
  bool get debugDidSendFirstFrameEvent => !_needToReportFirstFrame;

  void _handleBuildScheduled() {
    // If we're in the process of building dirty elements, then changes
    // should not trigger a new frame.
    assert(() {
      if (debugBuildingDirtyElements) {
        throw FlutterError.fromParts(<DiagnosticsNode>[
          ErrorSummary('Build scheduled during frame.'),
          ErrorDescription(
            'While the widget tree was being built, laid out, and painted, '
            'a new frame was scheduled to rebuild the widget tree.',
          ),
          ErrorHint(
            'This might be because setState() was called from a layout or '
            'paint callback. '
            'If a change is needed to the widget tree, it should be applied '
            'as the tree is being built. Scheduling a change for the subsequent '
            'frame instead results in an interface that lags behind by one frame. '
            'If this was done to make your build dependent on a size measured at '
            'layout time, consider using a LayoutBuilder, CustomSingleChildLayout, '
            'or CustomMultiChildLayout. If, on the other hand, the one frame delay '
            'is the desired effect, for example because this is an '
            'animation, consider scheduling the frame in a post-frame callback '
            'using SchedulerBinding.addPostFrameCallback or '
            'using an AnimationController to trigger the animation.',
          ),
        ]);
      }
      return true;
    }());
    ensureVisualUpdate();
  }

  /// Whether we are currently in a frame. This is used to verify
  /// that frames are not scheduled redundantly.
  ///
  /// This is public so that test frameworks can change it.
  ///
  /// This flag is not used in release builds.
  @protected
  bool debugBuildingDirtyElements = false;

  /// Pump the build and rendering pipeline to generate a frame.
  ///
  /// This method is called by [handleDrawFrame], which itself is called
  /// automatically by the engine when it is time to lay out and paint a
  /// frame.
  ///
  /// Each frame consists of the following phases:
  ///
  /// 1. The animation phase: The [handleBeginFrame] method, which is registered
  /// with [PlatformDispatcher.onBeginFrame], invokes all the transient frame
  /// callbacks registered with [scheduleFrameCallback], in registration order.
  /// This includes all the [Ticker] instances that are driving
  /// [AnimationController] objects, which means all of the active [Animation]
  /// objects tick at this point.
  ///
  /// 2. Microtasks: After [handleBeginFrame] returns, any microtasks that got
  /// scheduled by transient frame callbacks get to run. This typically includes
  /// callbacks for futures from [Ticker]s and [AnimationController]s that
  /// completed this frame.
  ///
  /// After [handleBeginFrame], [handleDrawFrame], which is registered with
  /// [PlatformDispatcher.onDrawFrame], is called, which invokes all the
  /// persistent frame callbacks, of which the most notable is this method,
  /// [drawFrame], which proceeds as follows:
  ///
  /// 3. The build phase: All the dirty [Element]s in the widget tree are
  /// rebuilt (see [State.build]). See [State.setState] for further details on
  /// marking a widget dirty for building. See [BuildOwner] for more information
  /// on this step.
  ///
  /// 4. The layout phase: All the dirty [RenderObject]s in the system are laid
  /// out (see [RenderObject.performLayout]). See [RenderObject.markNeedsLayout]
  /// for further details on marking an object dirty for layout.
  ///
  /// 5. The compositing bits phase: The compositing bits on any dirty
  /// [RenderObject] objects are updated. See
  /// [RenderObject.markNeedsCompositingBitsUpdate].
  ///
  /// 6. The paint phase: All the dirty [RenderObject]s in the system are
  /// repainted (see [RenderObject.paint]). This generates the [Layer] tree. See
  /// [RenderObject.markNeedsPaint] for further details on marking an object
  /// dirty for paint.
  ///
  /// 7. The compositing phase: The layer tree is turned into a [Scene] and
  /// sent to the GPU.
  ///
  /// 8. The semantics phase: All the dirty [RenderObject]s in the system have
  /// their semantics updated (see [RenderObject.assembleSemanticsNode]). This
  /// generates the [SemanticsNode] tree. See
  /// [RenderObject.markNeedsSemanticsUpdate] for further details on marking an
  /// object dirty for semantics.
  ///
  /// For more details on steps 4-8, see [PipelineOwner].
  ///
  /// 9. The finalization phase in the widgets layer: The widgets tree is
  /// finalized. This causes [State.dispose] to be invoked on any objects that
  /// were removed from the widgets tree this frame. See
  /// [BuildOwner.finalizeTree] for more details.
  ///
  /// 10. The finalization phase in the scheduler layer: After [drawFrame]
  /// returns, [handleDrawFrame] then invokes post-frame callbacks (registered
  /// with [addPostFrameCallback]).
  //
  // When editing the above, also update rendering/binding.dart's copy.
  @override
  void drawFrame() {
    assert(!debugBuildingDirtyElements);
    assert(() {
      debugBuildingDirtyElements = true;
      return true;
    }());

    TimingsCallback? firstFrameCallback;
    bool debugFrameWasSentToEngine = false;
    if (_needToReportFirstFrame) {
      assert(!_firstFrameCompleter.isCompleted);

      firstFrameCallback = (List<FrameTiming> timings) {
        assert(debugFrameWasSentToEngine);
        if (!kReleaseMode) {
          // Change the current user tag back to the default tag. At this point,
          // the user tag should be set to "AppStartUp" (originally set in the
          // engine), so we need to change it back to the default tag to mark
          // the end of app start up for CPU profiles.
          developer.UserTag.defaultTag.makeCurrent();
          developer.Timeline.instantSync('Rasterized first useful frame');
          developer.postEvent('Flutter.FirstFrame', <String, dynamic>{});
        }
        SchedulerBinding.instance.removeTimingsCallback(firstFrameCallback!);
        firstFrameCallback = null;
        _firstFrameCompleter.complete();
      };
      // Callback is only invoked when FlutterView.render is called. When
      // sendFramesToEngine is set to false during the frame, it will not be
      // called and we need to remove the callback (see below).
      SchedulerBinding.instance.addTimingsCallback(firstFrameCallback!);
    }

    try {
      if (rootElement != null) {
        buildOwner!.buildScope(rootElement!);
      }
      super.drawFrame();
      assert(() {
        debugFrameWasSentToEngine = sendFramesToEngine;
        return true;
      }());
      buildOwner!.finalizeTree();
    } finally {
      assert(() {
        debugBuildingDirtyElements = false;
        return true;
      }());
    }
    if (!kReleaseMode) {
      if (_needToReportFirstFrame && sendFramesToEngine) {
        developer.Timeline.instantSync('Widgets built first useful frame');
      }
    }
    _needToReportFirstFrame = false;
    if (firstFrameCallback != null && !sendFramesToEngine) {
      // This frame is deferred and not the first frame sent to the engine that
      // should be reported.
      _needToReportFirstFrame = true;
      SchedulerBinding.instance.removeTimingsCallback(firstFrameCallback!);
    }
  }

  /// The [Element] that is at the root of the element tree hierarchy.
  ///
  /// This is initialized the first time [runApp] is called.
  Element? get rootElement => _rootElement;
  Element? _rootElement;

  /// Deprecated. Will be removed in a future version of Flutter.
  ///
  /// Use [rootElement] instead.
  @Deprecated(
    'Use rootElement instead. '
    'This feature was deprecated after v3.9.0-16.0.pre.',
  )
  Element? get renderViewElement => rootElement;

  bool _readyToProduceFrames = false;

  @override
  bool get framesEnabled => super.framesEnabled && _readyToProduceFrames;

  /// Used by [runApp] to wrap the provided `rootWidget` in the default [View].
  ///
  /// The [View] determines into what [FlutterView] the app is rendered into.
  /// This is currently [PlatformDispatcher.implicitView] from [platformDispatcher].
  /// This method will throw a [StateError] if the [PlatformDispatcher.implicitView]
  /// is null.
  ///
  /// The `rootWidget` widget provided to this method must not already be
  /// wrapped in a [View].
  Widget wrapWithDefaultView(Widget rootWidget) {
    if (platformDispatcher.implicitView == null) {
      throw StateError(
        'The app requested a view, but the platform did not provide one.\n'
        'This is likely because the app called `runApp` to render its root '
        'widget, which expects the platform to provide a default view to '
        'render into (the "implicit" view).\n'
        'However, the platform likely has multi-view mode enabled, which does '
        'not create this default "implicit" view.\n'
        'Try using `runWidget` instead of `runApp` to start your app.\n'
        '`runWidget` allows you to provide a `View` widget, without requiring '
        'a default view.'
        '${kIsWeb ? "\nSee: https://flutter.dev/to/web-multiview-runwidget" : ""}',
      );
    }

    return View(
      view: platformDispatcher.implicitView!,
      deprecatedDoNotUseWillBeRemovedWithoutNoticePipelineOwner: pipelineOwner,
      deprecatedDoNotUseWillBeRemovedWithoutNoticeRenderView: renderView,
      child: rootWidget,
    );
  }

  /// Schedules a [Timer] for attaching the root widget.
  ///
  /// This is called by [runApp] to configure the widget tree. Consider using
  /// [attachRootWidget] if you want to build the widget tree synchronously.
  @protected
  void scheduleAttachRootWidget(Widget rootWidget) {
    Timer.run(() {
      attachRootWidget(rootWidget);
    });
  }

  /// Takes a widget and attaches it to the [rootElement], creating it if
  /// necessary.
  ///
  /// This is called by [runApp] to configure the widget tree.
  ///
  /// See also:
  ///
  ///  * [RenderObjectToWidgetAdapter.attachToRenderTree], which inflates a
  ///    widget and attaches it to the render tree.
  void attachRootWidget(Widget rootWidget) {
    attachToBuildOwner(RootWidget(debugShortDescription: '[root]', child: rootWidget));
  }

  /// Called by [attachRootWidget] to attach the provided [RootWidget] to the
  /// [buildOwner].
  ///
  /// This creates the [rootElement], if necessary, or re-uses an existing one.
  ///
  /// This method is rarely called directly, but it can be useful in tests to
  /// restore the element tree to a previous version by providing the
  /// [RootWidget] of that version (see [WidgetTester.restartAndRestore] for an
  /// exemplary use case).
  void attachToBuildOwner(RootWidget widget) {
    final bool isBootstrapFrame = rootElement == null;
    _readyToProduceFrames = true;
    _rootElement = widget.attach(buildOwner!, rootElement as RootElement?);
    if (isBootstrapFrame) {
      SchedulerBinding.instance.ensureVisualUpdate();
    }
  }

  /// Whether the [rootElement] has been initialized.
  ///
  /// This will be false until [runApp] is called (or [WidgetTester.pumpWidget]
  /// is called in the context of a [TestWidgetsFlutterBinding]).
  bool get isRootWidgetAttached => _rootElement != null;

  @override
  Future<void> performReassemble() {
    assert(() {
      WidgetInspectorService.instance.performReassemble();
      return true;
    }());

    if (rootElement != null) {
      buildOwner!.reassemble(rootElement!);
    }
    return super.performReassemble();
  }

  /// Computes the locale the current platform would resolve to.
  ///
  /// This method is meant to be used as part of a
  /// [WidgetsApp.localeListResolutionCallback]. Since this method may return
  /// null, a Flutter/dart algorithm should still be provided as a fallback in
  /// case a native resolved locale cannot be determined or if the native
  /// resolved locale is undesirable.
  ///
  /// This method may return a null [Locale] if the platform does not support
  /// native locale resolution, or if the resolution failed.
  ///
  /// The first `supportedLocale` is treated as the default locale and will be returned
  /// if no better match is found.
  ///
  /// Android and iOS are currently supported.
  ///
  /// On Android, the algorithm described in
  /// https://developer.android.com/guide/topics/resources/multilingual-support
  /// is used to determine the resolved locale. Depending on the android version
  /// of the device, either the modern (>= API 24) or legacy (< API 24) algorithm
  /// will be used.
  ///
  /// On iOS, the result of `preferredLocalizationsFromArray` method of `NSBundle`
  /// is returned. See:
  /// https://developer.apple.com/documentation/foundation/nsbundle/1417249-preferredlocalizationsfromarray?language=objc
  /// for details on the used method.
  ///
  /// iOS treats script code as necessary for a match, so a user preferred locale of
  /// `zh_Hans_CN` will not resolve to a supported locale of `zh_CN`.
  ///
  /// Since implementation may vary by platform and has potential to be heavy,
  /// it is recommended to cache the results of this method if the value is
  /// used multiple times.
  ///
  /// Second-best (and n-best) matching locales should be obtained by calling this
  /// method again with the matched locale of the first call omitted from
  /// `supportedLocales`.
  Locale? computePlatformResolvedLocale(List<Locale> supportedLocales) {
    return platformDispatcher.computePlatformResolvedLocale(supportedLocales);
  }
}

/// Inflate the given widget and attach it to the view.
///
// TODO(goderbauer): Update the paragraph below to include the Window widget once that exists.
/// The [runApp] method renders the provided `app` widget into the
/// [PlatformDispatcher.implicitView] by wrapping it in a [View] widget, which
/// will bootstrap the render tree for the app. Apps that want to control which
/// [FlutterView] they render into can use [runWidget] instead.
///
/// The widget is given constraints during layout that force it to fill the
/// entire view. If you wish to align your widget to one side of the view
/// (e.g., the top), consider using the [Align] widget. If you wish to center
/// your widget, you can also use the [Center] widget.
///
/// Calling [runApp] again will detach the previous root widget from the view
/// and attach the given widget in its place. The new widget tree is compared
/// against the previous widget tree and any differences are applied to the
/// underlying render tree, similar to what happens when a [StatefulWidget]
/// rebuilds after calling [State.setState].
///
/// Initializes the binding using [WidgetsFlutterBinding] if necessary.
///
/// {@template flutter.widgets.runApp.shutdown}
/// ## Application shutdown
///
/// This widget tree is not torn down when the application shuts down, because
/// there is no way to predict when that will happen. For example, a user could
/// physically remove power from their device, or the application could crash
/// unexpectedly, or the malware on the device could forcibly terminate the
/// process.
///
/// Applications are responsible for ensuring that they are well-behaved
/// even in the face of a rapid unscheduled termination.
///
/// To listen for platform shutdown messages (and other lifecycle changes),
/// consider the [AppLifecycleListener] API.
/// {@endtemplate}
///
/// To artificially cause the entire widget tree to be disposed, consider
/// calling [runApp] with a widget such as [SizedBox.shrink].
///
/// {@template flutter.widgets.runApp.dismissal}
/// ## Dismissing Flutter UI via platform native methods
///
/// An application may have both Flutter and non-Flutter UI in it. If the
/// application calls non-Flutter methods to remove Flutter based UI such as
/// platform native API to manipulate the platform native navigation stack,
/// the framework does not know if the developer intends to eagerly free
/// resources or not. The widget tree remains mounted and ready to render
/// as soon as it is displayed again.
/// {@endtemplate}
///
/// To release resources more eagerly, establish a [platform channel](https://flutter.dev/to/platform-channels)
/// and use it to call [runApp] with a widget such as [SizedBox.shrink] when
/// the framework should dispose of the active widget tree.
///
/// See also:
///
///  * [runWidget], which bootstraps a widget tree without assuming the
///    [FlutterView] into which it will be rendered.
///  * [WidgetsBinding.attachRootWidget], which creates the root widget for the
///    widget hierarchy.
///  * [RenderObjectToWidgetAdapter.attachToRenderTree], which creates the root
///    element for the element hierarchy.
///  * [WidgetsBinding.handleBeginFrame], which pumps the widget pipeline to
///    ensure the widget, element, and render trees are all built.
void runApp(Widget app) {
  final WidgetsBinding binding = WidgetsFlutterBinding.ensureInitialized();
  _runWidget(binding.wrapWithDefaultView(app), binding, 'runApp');
}

/// Inflate the given widget and bootstrap the widget tree.
///
// TODO(goderbauer): Update the paragraph below to include the Window widget once that exists.
/// Unlike [runApp], this method does not define a [FlutterView] into which the
/// provided `app` widget is rendered into. It is up to the caller to include at
/// least one [View] widget in the provided `app` widget that will bootstrap a
/// render tree and define the [FlutterView] into which content is rendered.
/// [RenderObjectWidget]s without an ancestor [View] widget will result in an
/// exception. Apps that want to render into the default view without dealing
/// with view management should consider calling [runApp] instead.
///
/// {@tool snippet}
/// The sample shows how to utilize [runWidget] to specify the [FlutterView]
/// into which the `MyApp` widget will be drawn:
///
/// ```dart
/// runWidget(
///   View(
///     view: myFlutterView,
///     child: const MyApp(),
///   ),
/// );
/// ```
/// {@end-tool}
///
/// Calling [runWidget] again will detach the previous root widget and attach
/// the given widget in its place. The new widget tree is compared against the
/// previous widget tree and any differences are applied to the underlying
/// render tree, similar to what happens when a [StatefulWidget] rebuilds after
/// calling [State.setState].
///
/// Initializes the binding using [WidgetsFlutterBinding] if necessary.
///
/// {@macro flutter.widgets.runApp.shutdown}
///
/// To artificially cause the entire widget tree to be disposed, consider
/// calling [runWidget] with a [ViewCollection] that does not specify any
/// [ViewCollection.views].
///
/// {@macro flutter.widgets.runApp.dismissal}
///
/// To release resources more eagerly, establish a [platform channel](https://flutter.dev/to/platform-channels)
/// and use it to remove the [View] whose widget resources should be released
/// from the `app` widget tree provided to [runWidget].
///
/// See also:
///
///  * [runApp], which bootstraps a widget tree and renders it into a default
///    [FlutterView].
///  * [WidgetsBinding.attachRootWidget], which creates the root widget for the
///    widget hierarchy.
///  * [RenderObjectToWidgetAdapter.attachToRenderTree], which creates the root
///    element for the element hierarchy.
///  * [WidgetsBinding.handleBeginFrame], which pumps the widget pipeline to
///    ensure the widget, element, and render trees are all built.
void runWidget(Widget app) {
  final WidgetsBinding binding = WidgetsFlutterBinding.ensureInitialized();
  _runWidget(app, binding, 'runWidget');
}

void _runWidget(Widget app, WidgetsBinding binding, String debugEntryPoint) {
  assert(binding.debugCheckZone(debugEntryPoint));
  binding
    ..scheduleAttachRootWidget(app)
    ..scheduleWarmUpFrame();
}

String _debugDumpAppString() {
  const String mode =
      kDebugMode
          ? 'DEBUG MODE'
          : kReleaseMode
          ? 'RELEASE MODE'
          : 'PROFILE MODE';
  final StringBuffer buffer = StringBuffer();
  buffer.writeln('${WidgetsBinding.instance.runtimeType} - $mode');
  if (WidgetsBinding.instance.rootElement != null) {
    buffer.writeln(WidgetsBinding.instance.rootElement!.toStringDeep());
  } else {
    buffer.writeln('<no tree currently mounted>');
  }
  return buffer.toString();
}

/// Print a string representation of the currently running app.
void debugDumpApp() {
  debugPrint(_debugDumpAppString());
}

/// A widget for the root of the widget tree.
///
/// Exposes an [attach] method to attach the widget tree to a [BuildOwner]. That
/// method also bootstraps the element tree.
///
/// Used by [WidgetsBinding.attachRootWidget] (which is indirectly called by
/// [runApp]) to bootstrap applications.
class RootWidget extends Widget {
  /// Creates a [RootWidget].
  const RootWidget({super.key, this.child, this.debugShortDescription});

  /// The widget below this widget in the tree.
  ///
  /// {@macro flutter.widgets.ProxyWidget.child}
  final Widget? child;

  /// A short description of this widget used by debugging aids.
  final String? debugShortDescription;

  @override
  RootElement createElement() => RootElement(this);

  /// Inflate this widget and attaches it to the provided [BuildOwner].
  ///
  /// If `element` is null, this function will create a new element. Otherwise,
  /// the given element will have an update scheduled to switch to this widget.
  ///
  /// Used by [WidgetsBinding.attachToBuildOwner] (which is indirectly called by
  /// [runApp]) to bootstrap applications.
  RootElement attach(BuildOwner owner, [RootElement? element]) {
    if (element == null) {
      owner.lockState(() {
        element = createElement();
        assert(element != null);
        element!.assignOwner(owner);
      });
      owner.buildScope(element!, () {
        element!.mount(/* parent */ null, /* slot */ null);
      });
    } else {
      element._newWidget = this;
      element.markNeedsBuild();
    }
    return element!;
  }

  @override
  String toStringShort() => debugShortDescription ?? super.toStringShort();
}

/// The root of the element tree.
///
/// This element class is the instantiation of a [RootWidget]. It can be used
/// only as the root of an [Element] tree (it cannot be mounted into another
/// [Element]; its parent must be null).
///
/// In typical usage, it will be instantiated for a [RootWidget] by calling
/// [RootWidget.attach]. In this usage, it is normally instantiated by the
/// bootstrapping logic in the [WidgetsFlutterBinding] singleton created by
/// [runApp].
class RootElement extends Element with RootElementMixin {
  /// Creates a [RootElement] for the provided [RootWidget].
  RootElement(RootWidget super.widget);

  Element? _child;

  @override
  void visitChildren(ElementVisitor visitor) {
    if (_child != null) {
      visitor(_child!);
    }
  }

  @override
  void forgetChild(Element child) {
    assert(child == _child);
    _child = null;
    super.forgetChild(child);
  }

  @override
  void mount(Element? parent, Object? newSlot) {
    assert(parent == null); // We are the root!
    super.mount(parent, newSlot);
    _rebuild();
    assert(_child != null);
    super.performRebuild(); // clears the "dirty" flag
  }

  @override
  void update(RootWidget newWidget) {
    super.update(newWidget);
    assert(widget == newWidget);
    _rebuild();
  }

  // When we are assigned a new widget, we store it here
  // until we are ready to update to it.
  RootWidget? _newWidget;

  @override
  void performRebuild() {
    if (_newWidget != null) {
      // _newWidget can be null if, for instance, we were rebuilt
      // due to a reassemble.
      final RootWidget newWidget = _newWidget!;
      _newWidget = null;
      update(newWidget);
    }
    super.performRebuild();
    assert(_newWidget == null);
  }

  void _rebuild() {
    try {
      _child = updateChild(_child, (widget as RootWidget).child, /* slot */ null);
    } catch (exception, stack) {
      final FlutterErrorDetails details = FlutterErrorDetails(
        exception: exception,
        stack: stack,
        library: 'widgets library',
        context: ErrorDescription('attaching to the render tree'),
      );
      FlutterError.reportError(details);
      // No error widget possible here since it wouldn't have a view to render into.
      _child = null;
    }
  }

  @override
  bool get debugDoingBuild => false; // This element doesn't have a build phase.

  @override
  // There is no ancestor RenderObjectElement that the render object could be attached to.
  bool debugExpectsRenderObjectForSlot(Object? slot) => false;
}

/// A concrete binding for applications based on the Widgets framework.
///
/// This is the glue that binds the framework to the Flutter engine.
///
/// When using the widgets framework, this binding, or one that
/// implements the same interfaces, must be used. The following
/// mixins are used to implement this binding:
///
/// * [GestureBinding], which implements the basics of hit testing.
/// * [SchedulerBinding], which introduces the concepts of frames.
/// * [ServicesBinding], which provides access to the plugin subsystem.
/// * [PaintingBinding], which enables decoding images.
/// * [SemanticsBinding], which supports accessibility.
/// * [RendererBinding], which handles the render tree.
/// * [WidgetsBinding], which handles the widget tree.
class WidgetsFlutterBinding extends BindingBase
    with
        GestureBinding,
        SchedulerBinding,
        ServicesBinding,
        PaintingBinding,
        SemanticsBinding,
        RendererBinding,
        WidgetsBinding {
  /// Returns an instance of the binding that implements
  /// [WidgetsBinding]. If no binding has yet been initialized, the
  /// [WidgetsFlutterBinding] class is used to create and initialize
  /// one.
  ///
  /// You only need to call this method if you need the binding to be
  /// initialized before calling [runApp].
  ///
  /// In the `flutter_test` framework, [testWidgets] initializes the
  /// binding instance to a [TestWidgetsFlutterBinding], not a
  /// [WidgetsFlutterBinding]. See
  /// [TestWidgetsFlutterBinding.ensureInitialized].
  static WidgetsBinding ensureInitialized() {
    if (WidgetsBinding._instance == null) {
      WidgetsFlutterBinding();
    }
    return WidgetsBinding.instance;
  }
}
