// Copyright 2013 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

import 'dart:async';
import 'dart:convert';
import 'dart:js_interop';
import 'dart:typed_data';

import 'package:meta/meta.dart';
import 'package:ui/ui.dart' as ui;
import 'package:ui/ui_web/src/ui_web.dart' as ui_web;

import '../engine.dart';

/// Requests that the browser schedule a frame.
///
/// This may be overridden in tests, for example, to pump fake frames.
ui.VoidCallback? scheduleFrameCallback;

/// Signature of functions added as a listener to high contrast changes
typedef HighContrastListener = void Function(bool enabled);
typedef _KeyDataResponseCallback = void Function(bool handled);

const StandardMethodCodec standardCodec = StandardMethodCodec();
const JSONMethodCodec jsonCodec = JSONMethodCodec();

/// Determines if high contrast is enabled using media query 'forced-colors: active' for Windows
class HighContrastSupport {
  static HighContrastSupport instance = HighContrastSupport();
  static const String _highContrastMediaQueryString = '(forced-colors: active)';

  final List<HighContrastListener> _listeners = <HighContrastListener>[];

  /// Reference to css media query that indicates whether high contrast is on.
  final DomMediaQueryList _highContrastMediaQuery = domWindow.matchMedia(
    _highContrastMediaQueryString,
  );
  late final DomEventListener _onHighContrastChangeListener = createDomEventListener(
    _onHighContrastChange,
  );

  bool get isHighContrastEnabled => _highContrastMediaQuery.matches;

  /// Adds function to the list of listeners on high contrast changes
  void addListener(HighContrastListener listener) {
    if (_listeners.isEmpty) {
      _highContrastMediaQuery.addListener(_onHighContrastChangeListener);
    }
    _listeners.add(listener);
  }

  /// Removes function from the list of listeners on high contrast changes
  void removeListener(HighContrastListener listener) {
    _listeners.remove(listener);
    if (_listeners.isEmpty) {
      _highContrastMediaQuery.removeListener(_onHighContrastChangeListener);
    }
  }

  JSVoid _onHighContrastChange(DomEvent event) {
    final DomMediaQueryListEvent mqEvent = event as DomMediaQueryListEvent;
    final bool isHighContrastEnabled = mqEvent.matches!;
    for (final HighContrastListener listener in _listeners) {
      listener(isHighContrastEnabled);
    }
  }
}

/// Platform event dispatcher.
///
/// This is the central entry point for platform messages and configuration
/// events from the platform.
class EnginePlatformDispatcher extends ui.PlatformDispatcher {
  /// Private constructor, since only dart:ui is supposed to create one of
  /// these.
  EnginePlatformDispatcher() {
    _addBrightnessMediaQueryListener();
    HighContrastSupport.instance.addListener(_updateHighContrast);
    _addFontSizeObserver();
    _addLocaleChangedListener();
    registerHotRestartListener(dispose);
    _appLifecycleState.addListener(_setAppLifecycleState);
    _viewFocusBinding.init();
    domDocument.body?.prepend(accessibilityPlaceholder);
    _onViewDisposedListener = viewManager.onViewDisposed.listen((_) {
      // Send a metrics changed event to the framework when a view is disposed.
      // View creation/resize is handled by the `_didResize` handler in the
      // EngineFlutterView itself.
      invokeOnMetricsChanged();
    });
  }

  late StreamSubscription<int> _onViewDisposedListener;

  /// The [EnginePlatformDispatcher] singleton.
  static EnginePlatformDispatcher get instance => _instance;
  static final EnginePlatformDispatcher _instance = EnginePlatformDispatcher();

  @visibleForTesting
  final DomElement accessibilityPlaceholder =
      EngineSemantics.instance.semanticsHelper.prepareAccessibilityPlaceholder();

  PlatformConfiguration configuration = PlatformConfiguration(
    locales: parseBrowserLanguages(),
    textScaleFactor: findBrowserTextScaleFactor(),
    accessibilityFeatures: computeAccessibilityFeatures(),
  );

  /// Compute accessibility features based on the current value of high contrast flag
  static EngineAccessibilityFeatures computeAccessibilityFeatures() {
    final EngineAccessibilityFeaturesBuilder builder = EngineAccessibilityFeaturesBuilder(0);
    if (HighContrastSupport.instance.isHighContrastEnabled) {
      builder.highContrast = true;
    }
    return builder.build();
  }

  void dispose() {
    _removeBrightnessMediaQueryListener();
    _disconnectFontSizeObserver();
    _removeLocaleChangedListener();
    HighContrastSupport.instance.removeListener(_updateHighContrast);
    _appLifecycleState.removeListener(_setAppLifecycleState);
    _viewFocusBinding.dispose();
    accessibilityPlaceholder.remove();
    _onViewDisposedListener.cancel();
    viewManager.dispose();
  }

  /// Receives all events related to platform configuration changes.
  @override
  ui.VoidCallback? get onPlatformConfigurationChanged => _onPlatformConfigurationChanged;
  ui.VoidCallback? _onPlatformConfigurationChanged;
  Zone? _onPlatformConfigurationChangedZone;
  @override
  set onPlatformConfigurationChanged(ui.VoidCallback? callback) {
    _onPlatformConfigurationChanged = callback;
    _onPlatformConfigurationChangedZone = Zone.current;
  }

  /// Engine code should use this method instead of the callback directly.
  /// Otherwise zones won't work properly.
  void invokeOnPlatformConfigurationChanged() {
    invoke(_onPlatformConfigurationChanged, _onPlatformConfigurationChangedZone);
  }

  @override
  Iterable<EngineFlutterDisplay> displays = <EngineFlutterDisplay>[EngineFlutterDisplay.instance];

  late final FlutterViewManager viewManager = FlutterViewManager(this);

  late final AppLifecycleState _appLifecycleState = AppLifecycleState.create(viewManager);

  /// The current list of windows.
  @override
  Iterable<EngineFlutterView> get views => viewManager.views;

  /// Returns the [EngineFlutterView] with the provided ID if one exists, or null
  /// otherwise.
  @override
  EngineFlutterView? view({required int id}) => viewManager[id];

  /// The [FlutterView] provided by the engine if the platform is unable to
  /// create windows, or, for backwards compatibility.
  ///
  /// If the platform provides an implicit view, it can be used to bootstrap
  /// the framework. This is common for platforms designed for single-view
  /// applications like mobile devices with a single display.
  ///
  /// Applications and libraries must not rely on this property being set
  /// as it may be null depending on the engine's configuration. Instead,
  /// consider using [View.of] to lookup the [FlutterView] the current
  /// [BuildContext] is drawing into.
  ///
  /// While the properties on the referenced [FlutterView] may change,
  /// the reference itself is guaranteed to never change over the lifetime
  /// of the application: if this property is null at startup, it will remain
  /// so throughout the entire lifetime of the application. If it points to a
  /// specific [FlutterView], it will continue to point to the same view until
  /// the application is shut down (although the engine may replace or remove
  /// the underlying backing surface of the view at its discretion).
  ///
  /// See also:
  ///
  /// * [View.of], for accessing the current view.
  /// * [PlatformDisptacher.views] for a list of all [FlutterView]s provided
  ///   by the platform.
  @override
  EngineFlutterWindow? get implicitView => viewManager[kImplicitViewId] as EngineFlutterWindow?;

  /// A callback that is invoked whenever the platform's [devicePixelRatio],
  /// [physicalSize], [padding], [viewInsets], or [systemGestureInsets]
  /// values change, for example when the device is rotated or when the
  /// application is resized (e.g. when showing applications side-by-side
  /// on Android).
  ///
  /// The engine invokes this callback in the same zone in which the callback
  /// was set.
  ///
  /// The framework registers with this callback and updates the layout
  /// appropriately.
  ///
  /// See also:
  ///
  ///  * [WidgetsBindingObserver], for a mechanism at the widgets layer to
  ///    register for notifications when this is called.
  ///  * [MediaQuery.of], a simpler mechanism for the same.
  @override
  ui.VoidCallback? get onMetricsChanged => _onMetricsChanged;
  ui.VoidCallback? _onMetricsChanged;
  Zone? _onMetricsChangedZone;
  @override
  set onMetricsChanged(ui.VoidCallback? callback) {
    _onMetricsChanged = callback;
    _onMetricsChangedZone = Zone.current;
  }

  /// Engine code should use this method instead of the callback directly.
  /// Otherwise zones won't work properly.
  void invokeOnMetricsChanged() {
    if (_onMetricsChanged != null) {
      invoke(_onMetricsChanged, _onMetricsChangedZone);
    }
  }

  late final ViewFocusBinding _viewFocusBinding = ViewFocusBinding(
    viewManager,
    invokeOnViewFocusChange,
  );

  @override
  ui.ViewFocusChangeCallback? get onViewFocusChange => _onViewFocusChange;
  ui.ViewFocusChangeCallback? _onViewFocusChange;
  Zone? _onViewFocusChangeZone;
  @override
  set onViewFocusChange(ui.ViewFocusChangeCallback? callback) {
    _onViewFocusChange = callback;
    _onViewFocusChangeZone = Zone.current;
  }

  // Engine code should use this method instead of the callback directly.
  // Otherwise zones won't work properly.
  void invokeOnViewFocusChange(ui.ViewFocusEvent viewFocusEvent) {
    invoke1<ui.ViewFocusEvent>(_onViewFocusChange, _onViewFocusChangeZone, viewFocusEvent);
  }

  @override
  void requestViewFocusChange({
    required int viewId,
    required ui.ViewFocusState state,
    required ui.ViewFocusDirection direction,
  }) {
    _viewFocusBinding.changeViewFocus(viewId, state);
  }

  /// A set of views which have rendered in the current `onBeginFrame` or
  /// `onDrawFrame` scope.
  Set<ui.FlutterView>? _viewsRenderedInCurrentFrame;

  /// A callback invoked when any window begins a frame.
  ///
  /// A callback that is invoked to notify the application that it is an
  /// appropriate time to provide a scene using the [SceneBuilder] API and the
  /// [PlatformWindow.render] method.
  /// When possible, this is driven by the hardware VSync signal of the attached
  /// screen with the highest VSync rate. This is only called if
  /// [PlatformWindow.scheduleFrame] has been called since the last time this
  /// callback was invoked.
  @override
  ui.FrameCallback? get onBeginFrame => _onBeginFrame;
  ui.FrameCallback? _onBeginFrame;
  Zone? _onBeginFrameZone;
  @override
  set onBeginFrame(ui.FrameCallback? callback) {
    _onBeginFrame = callback;
    _onBeginFrameZone = Zone.current;
  }

  /// Engine code should use this method instead of the callback directly.
  /// Otherwise zones won't work properly.
  void invokeOnBeginFrame(Duration duration) {
    _viewsRenderedInCurrentFrame = <ui.FlutterView>{};
    invoke1<Duration>(_onBeginFrame, _onBeginFrameZone, duration);
    _viewsRenderedInCurrentFrame = null;
  }

  /// A callback that is invoked for each frame after [onBeginFrame] has
  /// completed and after the microtask queue has been drained.
  ///
  /// This can be used to implement a second phase of frame rendering that
  /// happens after any deferred work queued by the [onBeginFrame] phase.
  @override
  ui.VoidCallback? get onDrawFrame => _onDrawFrame;
  ui.VoidCallback? _onDrawFrame;
  Zone? _onDrawFrameZone;
  @override
  set onDrawFrame(ui.VoidCallback? callback) {
    _onDrawFrame = callback;
    _onDrawFrameZone = Zone.current;
  }

  /// Engine code should use this method instead of the callback directly.
  /// Otherwise zones won't work properly.
  void invokeOnDrawFrame() {
    _viewsRenderedInCurrentFrame = <ui.FlutterView>{};
    invoke(_onDrawFrame, _onDrawFrameZone);
    _viewsRenderedInCurrentFrame = null;
  }

  /// A callback that is invoked when pointer data is available.
  ///
  /// The framework invokes this callback in the same zone in which the
  /// callback was set.
  ///
  /// See also:
  ///
  ///  * [GestureBinding], the Flutter framework class which manages pointer
  ///    events.
  @override
  ui.PointerDataPacketCallback? get onPointerDataPacket => _onPointerDataPacket;
  ui.PointerDataPacketCallback? _onPointerDataPacket;
  Zone? _onPointerDataPacketZone;
  @override
  set onPointerDataPacket(ui.PointerDataPacketCallback? callback) {
    _onPointerDataPacket = callback;
    _onPointerDataPacketZone = Zone.current;
  }

  /// Engine code should use this method instead of the callback directly.
  /// Otherwise zones won't work properly.
  void invokeOnPointerDataPacket(ui.PointerDataPacket dataPacket) {
    invoke1<ui.PointerDataPacket>(_onPointerDataPacket, _onPointerDataPacketZone, dataPacket);
  }

  /// A callback that is invoked when key data is available.
  ///
  /// The framework invokes this callback in the same zone in which the
  /// callback was set.
  ///
  /// See also:
  ///
  ///  * [GestureBinding], the Flutter framework class which manages pointer
  ///    events.
  @override
  ui.KeyDataCallback? get onKeyData => _onKeyData;
  ui.KeyDataCallback? _onKeyData;
  Zone? _onKeyDataZone;
  @override
  set onKeyData(ui.KeyDataCallback? callback) {
    _onKeyData = callback;
    _onKeyDataZone = Zone.current;
  }

  /// Engine code should use this method instead of the callback directly.
  /// Otherwise zones won't work properly.
  void invokeOnKeyData(ui.KeyData data, _KeyDataResponseCallback callback) {
    final ui.KeyDataCallback? onKeyData = _onKeyData;
    if (onKeyData != null) {
      invoke(() => callback(onKeyData(data)), _onKeyDataZone);
    } else {
      callback(false);
    }
  }

  /// A callback that is invoked to report the [FrameTiming] of recently
  /// rasterized frames.
  ///
  /// It's preferred to use [SchedulerBinding.addTimingsCallback] than to use
  /// [PlatformDispatcher.onReportTimings] directly because
  /// [SchedulerBinding.addTimingsCallback] allows multiple callbacks.
  ///
  /// This can be used to see if the application has missed frames (through
  /// [FrameTiming.buildDuration] and [FrameTiming.rasterDuration]), or high
  /// latencies (through [FrameTiming.totalSpan]).
  ///
  /// Unlike [Timeline], the timing information here is available in the release
  /// mode (additional to the profile and the debug mode). Hence this can be
  /// used to monitor the application's performance in the wild.
  ///
  /// {@macro dart.ui.TimingsCallback.list}
  ///
  /// If this is null, no additional work will be done. If this is not null,
  /// Flutter spends less than 0.1ms every 1 second to report the timings
  /// (measured on iPhone6S). The 0.1ms is about 0.6% of 16ms (frame budget for
  /// 60fps), or 0.01% CPU usage per second.
  @override
  ui.TimingsCallback? get onReportTimings => _onReportTimings;
  ui.TimingsCallback? _onReportTimings;
  Zone? _onReportTimingsZone;
  @override
  set onReportTimings(ui.TimingsCallback? callback) {
    _onReportTimings = callback;
    _onReportTimingsZone = Zone.current;
  }

  /// Engine code should use this method instead of the callback directly.
  /// Otherwise zones won't work properly.
  void invokeOnReportTimings(List<ui.FrameTiming> timings) {
    invoke1<List<ui.FrameTiming>>(_onReportTimings, _onReportTimingsZone, timings);
  }

  @override
  void sendPlatformMessage(
    String name,
    ByteData? data,
    ui.PlatformMessageResponseCallback? callback,
  ) {
    _sendPlatformMessage(name, data, _zonedPlatformMessageResponseCallback(callback));
  }

  @override
  void sendPortPlatformMessage(String name, ByteData? data, int identifier, Object port) {
    throw Exception("Isolates aren't supported in web.");
  }

  @override
  void registerBackgroundIsolate(ui.RootIsolateToken token) {
    throw Exception("Isolates aren't supported in web.");
  }

  // TODO(ianh): Deprecate onPlatformMessage once the framework is moved over
  // to using channel buffers exclusively.
  @override
  ui.PlatformMessageCallback? get onPlatformMessage => _onPlatformMessage;
  ui.PlatformMessageCallback? _onPlatformMessage;
  Zone? _onPlatformMessageZone;
  @override
  set onPlatformMessage(ui.PlatformMessageCallback? callback) {
    _onPlatformMessage = callback;
    _onPlatformMessageZone = Zone.current;
  }

  /// Engine code should use this method instead of the callback directly.
  /// Otherwise zones won't work properly.
  void invokeOnPlatformMessage(
    String name,
    ByteData? data,
    ui.PlatformMessageResponseCallback callback,
  ) {
    if (name == ui.ChannelBuffers.kControlChannelName) {
      // TODO(ianh): move this logic into ChannelBuffers once we remove onPlatformMessage
      try {
        ui.channelBuffers.handleMessage(data!);
      } finally {
        callback(null);
      }
    } else if (_onPlatformMessage != null) {
      invoke3<String, ByteData?, ui.PlatformMessageResponseCallback>(
        _onPlatformMessage,
        _onPlatformMessageZone,
        name,
        data,
        callback,
      );
    } else {
      ui.channelBuffers.push(name, data, callback);
    }
  }

  /// Wraps the given [callback] in another callback that ensures that the
  /// original callback is called in the zone it was registered in.
  static ui.PlatformMessageResponseCallback? _zonedPlatformMessageResponseCallback(
    ui.PlatformMessageResponseCallback? callback,
  ) {
    if (callback == null) {
      return null;
    }

    // Store the zone in which the callback is being registered.
    final Zone registrationZone = Zone.current;

    return (ByteData? data) {
      registrationZone.runUnaryGuarded(callback, data);
    };
  }

  void _sendPlatformMessage(
    String name,
    ByteData? data,
    ui.PlatformMessageResponseCallback? callback,
  ) {
    // In widget tests we want to bypass processing of platform messages.
    bool returnImmediately = false;
    assert(() {
      if (ui_web.debugEmulateFlutterTesterEnvironment) {
        returnImmediately = true;
      }
      return true;
    }());

    if (returnImmediately) {
      return;
    }

    if (debugPrintPlatformMessages) {
      print('Sent platform message on channel: "$name"');
    }

    bool allowDebugEcho = false;
    assert(() {
      allowDebugEcho = true;
      return true;
    }());

    if (allowDebugEcho && name == 'flutter/debug-echo') {
      // Echoes back the data unchanged. Used for testing purposes.
      replyToPlatformMessage(callback, data);
      return;
    }

    switch (name) {
      /// This should be in sync with shell/common/shell.cc
      case 'flutter/skia':
        final MethodCall decoded = jsonCodec.decodeMethodCall(data);
        switch (decoded.method) {
          case 'Skia.setResourceCacheMaxBytes':
            if (renderer is CanvasKitRenderer) {
              assert(
                decoded.arguments is int,
                'Argument to Skia.setResourceCacheMaxBytes must be an int, but was ${(decoded.arguments as Object?).runtimeType}',
              );
              final int cacheSizeInBytes = decoded.arguments as int;
              CanvasKitRenderer.instance.resourceCacheMaxBytes = cacheSizeInBytes;
            }

            // Also respond in HTML mode. Otherwise, apps would have to detect
            // CanvasKit vs HTML before invoking this method.
            replyToPlatformMessage(callback, jsonCodec.encodeSuccessEnvelope(<bool>[true]));
        }
        return;

      case 'flutter/assets':
        final String url = utf8.decode(data!.buffer.asUint8List());
        _handleFlutterAssetsMessage(url, callback);
        return;

      case 'flutter/platform':
        final MethodCall decoded = jsonCodec.decodeMethodCall(data);
        switch (decoded.method) {
          case 'SystemNavigator.pop':
            // TODO(a-wallen): As multi-window support expands, the pop call
            // will need to include the view ID. Right now only one view is
            // supported.
            //
            // TODO(mdebbar): What should we do in multi-view mode?
            //                https://github.com/flutter/flutter/issues/139174
            if (implicitView != null) {
              implicitView!.browserHistory.exit().then((_) {
                replyToPlatformMessage(callback, jsonCodec.encodeSuccessEnvelope(true));
              });
            } else {
              replyToPlatformMessage(callback, jsonCodec.encodeSuccessEnvelope(true));
            }
            return;
          case 'HapticFeedback.vibrate':
            final String? type = decoded.arguments as String?;
            vibrate(_getHapticFeedbackDuration(type));
            replyToPlatformMessage(callback, jsonCodec.encodeSuccessEnvelope(true));
            return;
          case 'SystemChrome.setApplicationSwitcherDescription':
            final Map<String, Object?> arguments = decoded.arguments as Map<String, Object?>;
            final String label = arguments['label'] as String? ?? '';
            // TODO(web): Stop setting the color from here, https://github.com/flutter/flutter/issues/123365
            final int primaryColor = arguments['primaryColor'] as int? ?? 0xFF000000;
            domDocument.title = label;
            setThemeColor(ui.Color(primaryColor));
            replyToPlatformMessage(callback, jsonCodec.encodeSuccessEnvelope(true));
            return;
          case 'SystemChrome.setSystemUIOverlayStyle':
            final Map<String, Object?> arguments = decoded.arguments as Map<String, Object?>;
            final int? statusBarColor = arguments['statusBarColor'] as int?;
            setThemeColor(statusBarColor == null ? null : ui.Color(statusBarColor));
            replyToPlatformMessage(callback, jsonCodec.encodeSuccessEnvelope(true));
            return;
          case 'SystemChrome.setPreferredOrientations':
            final List<dynamic> arguments = decoded.arguments as List<dynamic>;
            ScreenOrientation.instance.setPreferredOrientation(arguments).then((bool success) {
              replyToPlatformMessage(callback, jsonCodec.encodeSuccessEnvelope(success));
            });
            return;
          case 'SystemSound.play':
            // There are no default system sounds on web.
            replyToPlatformMessage(callback, jsonCodec.encodeSuccessEnvelope(true));
            return;
          case 'Clipboard.setData':
            ClipboardMessageHandler().setDataMethodCall(decoded, callback);
            return;
          case 'Clipboard.getData':
            ClipboardMessageHandler().getDataMethodCall(callback);
            return;
          case 'Clipboard.hasStrings':
            ClipboardMessageHandler().hasStringsMethodCall(callback);
            return;
        }

      // Dispatched by the bindings to delay service worker initialization.
      case 'flutter/service_worker':
        domWindow.dispatchEvent(createDomEvent('Event', 'flutter-first-frame'));
        return;

      case 'flutter/textinput':
        textEditing.channel.handleTextInput(data, callback);
        return;

      case 'flutter/contextmenu':
        final MethodCall decoded = jsonCodec.decodeMethodCall(data);
        switch (decoded.method) {
          case 'enableContextMenu':
            implicitView!.contextMenu.enable();
            replyToPlatformMessage(callback, jsonCodec.encodeSuccessEnvelope(true));
            return;
          case 'disableContextMenu':
            implicitView!.contextMenu.disable();
            replyToPlatformMessage(callback, jsonCodec.encodeSuccessEnvelope(true));
            return;
        }
        return;

      case 'flutter/mousecursor':
        final MethodCall decoded = standardCodec.decodeMethodCall(data);
        final Map<dynamic, dynamic> arguments = decoded.arguments as Map<dynamic, dynamic>;
        switch (decoded.method) {
          case 'activateSystemCursor':
            // TODO(mdebbar): Once the framework starts sending us a viewId, we
            //                should use it to grab the correct view.
            //                https://github.com/flutter/flutter/issues/140226
            views.firstOrNull?.mouseCursor.activateSystemCursor(arguments.tryString('kind'));
        }
        return;

      case 'flutter/web_test_e2e':
        replyToPlatformMessage(
          callback,
          jsonCodec.encodeSuccessEnvelope(_handleWebTestEnd2EndMessage(jsonCodec, data)),
        );
        return;

      case PlatformViewMessageHandler.channelName:
        // `arguments` can be a Map<String, Object> for `create`,
        // but an `int` for `dispose`, hence why `dynamic` everywhere.
        final MethodCall(:String method, :dynamic arguments) = standardCodec.decodeMethodCall(data);
        PlatformViewMessageHandler.instance.handlePlatformViewCall(method, arguments, callback!);
        return;

      case 'flutter/accessibility':
        // In widget tests we want to bypass processing of platform messages.
        const StandardMessageCodec codec = StandardMessageCodec();
        final EngineSemantics semantics = EngineSemantics.instance;
        if (semantics.semanticsEnabled) {
          semantics.accessibilityAnnouncements.handleMessage(codec, data);
        }
        replyToPlatformMessage(callback, codec.encodeMessage(true));
        return;

      case 'flutter/navigation':
        // TODO(a-wallen): As multi-window support expands, the navigation call
        // will need to include the view ID. Right now only one view is
        // supported.
        //
        // TODO(mdebbar): What should we do in multi-view mode?
        //                https://github.com/flutter/flutter/issues/139174
        if (implicitView != null) {
          implicitView!.handleNavigationMessage(data).then((bool handled) {
            if (handled) {
              replyToPlatformMessage(callback, jsonCodec.encodeSuccessEnvelope(true));
            } else {
              callback?.call(null);
            }
          });
        } else {
          callback?.call(null);
        }

        // As soon as Flutter starts taking control of the app navigation, we
        // should reset _defaultRouteName to "/" so it doesn't have any
        // further effect after this point.
        _defaultRouteName = '/';
        return;
    }

    if (pluginMessageCallHandler != null) {
      pluginMessageCallHandler!(name, data, callback);
      return;
    }

    // Passing [null] to [callback] indicates that the platform message isn't
    // implemented. Look at [MethodChannel.invokeMethod] to see how [null] is
    // handled.
    replyToPlatformMessage(callback, null);
  }

  Future<void> _handleFlutterAssetsMessage(
    String url,
    ui.PlatformMessageResponseCallback? callback,
  ) async {
    try {
      final HttpFetchResponse response =
          await ui_web.assetManager.loadAsset(url) as HttpFetchResponse;
      final ByteBuffer assetData = await response.asByteBuffer();
      replyToPlatformMessage(callback, assetData.asByteData());
    } catch (error) {
      printWarning('Error while trying to load an asset: $error');
      replyToPlatformMessage(callback, null);
    }
  }

  int _getHapticFeedbackDuration(String? type) {
    const int vibrateLongPress = 50;
    const int vibrateLightImpact = 10;
    const int vibrateMediumImpact = 20;
    const int vibrateHeavyImpact = 30;
    const int vibrateSelectionClick = 10;

    switch (type) {
      case 'HapticFeedbackType.lightImpact':
        return vibrateLightImpact;
      case 'HapticFeedbackType.mediumImpact':
        return vibrateMediumImpact;
      case 'HapticFeedbackType.heavyImpact':
        return vibrateHeavyImpact;
      case 'HapticFeedbackType.selectionClick':
        return vibrateSelectionClick;
      default:
        return vibrateLongPress;
    }
  }

  /// Requests that, at the next appropriate opportunity, the [onBeginFrame]
  /// and [onDrawFrame] callbacks be invoked.
  ///
  /// See also:
  ///
  ///  * [SchedulerBinding], the Flutter framework class which manages the
  ///    scheduling of frames.
  @override
  void scheduleFrame() {
    if (scheduleFrameCallback == null) {
      throw Exception('scheduleFrameCallback must be initialized first.');
    }
    scheduleFrameCallback!();
  }

  @override
  void scheduleWarmUpFrame({
    required ui.VoidCallback beginFrame,
    required ui.VoidCallback drawFrame,
  }) {
    Timer.run(beginFrame);
    // We use timers here to ensure that microtasks flush in between.
    //
    // TODO(dkwingsmt): This logic was moved from the framework and is different
    // from how Web renders a regular frame, which doesn't flush microtasks
    // between the callbacks at all (see `initializeEngineServices`). We might
    // want to change this. See the to-do in `initializeEngineServices` and
    // https://github.com/flutter/engine/pull/50570#discussion_r1496671676
    Timer.run(drawFrame);
  }

  /// Updates the application's rendering on the GPU with the newly provided
  /// [Scene]. This function must be called within the scope of the
  /// [onBeginFrame] or [onDrawFrame] callbacks being invoked. If this function
  /// is called a second time during a single [onBeginFrame]/[onDrawFrame]
  /// callback sequence or called outside the scope of those callbacks, the call
  /// will be ignored.
  ///
  /// To record graphical operations, first create a [PictureRecorder], then
  /// construct a [Canvas], passing that [PictureRecorder] to its constructor.
  /// After issuing all the graphical operations, call the
  /// [PictureRecorder.endRecording] function on the [PictureRecorder] to obtain
  /// the final [Picture] that represents the issued graphical operations.
  ///
  /// Next, create a [SceneBuilder], and add the [Picture] to it using
  /// [SceneBuilder.addPicture]. With the [SceneBuilder.build] method you can
  /// then obtain a [Scene] object, which you can display to the user via this
  /// [render] function.
  ///
  /// See also:
  ///
  ///  * [SchedulerBinding], the Flutter framework class which manages the
  ///    scheduling of frames.
  ///  * [RendererBinding], the Flutter framework class which manages layout and
  ///    painting.
  Future<void> render(ui.Scene scene, [ui.FlutterView? view]) async {
    final EngineFlutterView? target = (view ?? implicitView) as EngineFlutterView?;
    assert(target != null, 'Calling render without a FlutterView');
    if (target == null) {
      // If there is no view to render into, then this is a no-op.
      return;
    }

    // Only render in an `onDrawFrame` or `onBeginFrame` scope. This is checked
    // by checking if the `_viewsRenderedInCurrentFrame` is non-null and this
    // view hasn't been rendered already in this scope.
    final bool shouldRender = _viewsRenderedInCurrentFrame?.add(target) ?? false;
    // TODO(harryterkelsen): HTML renderer needs to violate the render rule in
    // order to perform golden tests in Flutter framework because on the HTML
    // renderer, golden tests render to DOM and then take a browser screenshot,
    // https://github.com/flutter/flutter/issues/137073.
    if (shouldRender || renderer.rendererTag == 'html') {
      await renderer.renderScene(scene, target);
    }
  }

  /// Additional accessibility features that may be enabled by the platform.
  @override
  ui.AccessibilityFeatures get accessibilityFeatures => configuration.accessibilityFeatures;

  /// A callback that is invoked when the value of [accessibilityFeatures] changes.
  ///
  /// The framework invokes this callback in the same zone in which the
  /// callback was set.
  @override
  ui.VoidCallback? get onAccessibilityFeaturesChanged => _onAccessibilityFeaturesChanged;
  ui.VoidCallback? _onAccessibilityFeaturesChanged;
  Zone? _onAccessibilityFeaturesChangedZone;
  @override
  set onAccessibilityFeaturesChanged(ui.VoidCallback? callback) {
    _onAccessibilityFeaturesChanged = callback;
    _onAccessibilityFeaturesChangedZone = Zone.current;
  }

  /// Engine code should use this method instead of the callback directly.
  /// Otherwise zones won't work properly.
  void invokeOnAccessibilityFeaturesChanged() {
    invoke(_onAccessibilityFeaturesChanged, _onAccessibilityFeaturesChangedZone);
  }

  /// Change the retained semantics data about this window.
  ///
  /// If [semanticsEnabled] is true, the user has requested that this function
  /// be called whenever the semantic content of this window changes.
  ///
  /// In either case, this function disposes the given update, which means the
  /// semantics update cannot be used further.
  @override
  @Deprecated('''
    In a multi-view world, the platform dispatcher can no longer provide apis
    to update semantics since each view will host its own semantics tree.

    Semantics updates must be passed to an individual [FlutterView]. To update
    semantics, use PlatformDispatcher.instance.views to get a [FlutterView] and
    call `updateSemantics`.
  ''')
  void updateSemantics(ui.SemanticsUpdate update) {
    implicitView?.semantics.updateSemantics(update);
  }

  /// This is equivalent to `locales.first`, except that it will provide an
  /// undefined (using the language tag "und") non-null locale if the [locales]
  /// list has not been set or is empty.
  ///
  /// We use the first locale in the [locales] list instead of the browser's
  /// built-in `navigator.language` because browsers do not agree on the
  /// implementation.
  ///
  /// See also:
  ///
  /// * https://developer.mozilla.org/en-US/docs/Web/API/NavigatorLanguage/languages,
  ///   which explains browser quirks in the implementation notes.
  @override
  ui.Locale get locale => locales.isEmpty ? const ui.Locale.fromSubtags() : locales.first;

  /// The full system-reported supported locales of the device.
  ///
  /// This establishes the language and formatting conventions that application
  /// should, if possible, use to render their user interface.
  ///
  /// The list is ordered in order of priority, with lower-indexed locales being
  /// preferred over higher-indexed ones. The first element is the primary [locale].
  ///
  /// The [onLocaleChanged] callback is called whenever this value changes.
  ///
  /// See also:
  ///
  ///  * [WidgetsBindingObserver], for a mechanism at the widgets layer to
  ///    observe when this value changes.
  @override
  List<ui.Locale> get locales => configuration.locales;

  // A subscription to the 'languagechange' event of 'window'.
  DomSubscription? _onLocaleChangedSubscription;

  /// Configures the [_onLocaleChangedSubscription].
  void _addLocaleChangedListener() {
    if (_onLocaleChangedSubscription != null) {
      return;
    }
    updateLocales(); // First time, for good measure.
    _onLocaleChangedSubscription = DomSubscription(domWindow, 'languagechange', (DomEvent _) {
      // Update internal config, then propagate the changes.
      updateLocales();
      invokeOnLocaleChanged();
    });
  }

  /// Removes the [_onLocaleChangedSubscription].
  void _removeLocaleChangedListener() {
    _onLocaleChangedSubscription?.cancel();
    _onLocaleChangedSubscription = null;
  }

  /// Performs the platform-native locale resolution.
  ///
  /// Each platform may return different results.
  ///
  /// If the platform fails to resolve a locale, then this will return null.
  ///
  /// This method returns synchronously and is a direct call to
  /// platform specific APIs without invoking method channels.
  @override
  ui.Locale? computePlatformResolvedLocale(List<ui.Locale> supportedLocales) {
    // TODO(garyq): Implement on web.
    return null;
  }

  /// A callback that is invoked whenever [locale] changes value.
  ///
  /// The framework invokes this callback in the same zone in which the
  /// callback was set.
  ///
  /// See also:
  ///
  ///  * [WidgetsBindingObserver], for a mechanism at the widgets layer to
  ///    observe when this callback is invoked.
  @override
  ui.VoidCallback? get onLocaleChanged => _onLocaleChanged;
  ui.VoidCallback? _onLocaleChanged;
  Zone? _onLocaleChangedZone;
  @override
  set onLocaleChanged(ui.VoidCallback? callback) {
    _onLocaleChanged = callback;
    _onLocaleChangedZone = Zone.current;
  }

  /// The locale used when we fail to get the list from the browser.
  static const ui.Locale _defaultLocale = ui.Locale('en', 'US');

  /// Sets locales to an empty list.
  ///
  /// The empty list is not a valid value for locales. This is only used for
  /// testing locale update logic.
  void debugResetLocales() {
    configuration = configuration.copyWith(locales: const <ui.Locale>[]);
  }

  // Called by `_onLocaleChangedSubscription` when browser languages change.
  void updateLocales() {
    configuration = configuration.copyWith(locales: parseBrowserLanguages());
  }

  static List<ui.Locale> parseBrowserLanguages() {
    // TODO(yjbanov): find a solution for IE
    final List<String>? languages = domWindow.navigator.languages;
    if (languages == null || languages.isEmpty) {
      // To make it easier for the app code, let's not leave the locales list
      // empty. This way there's fewer corner cases for apps to handle.
      return const <ui.Locale>[_defaultLocale];
    }

    final List<ui.Locale> locales = <ui.Locale>[];
    for (final String language in languages) {
      final List<String> parts = language.split('-');
      if (parts.length > 1) {
        locales.add(ui.Locale(parts.first, parts.last));
      } else {
        locales.add(ui.Locale(language));
      }
    }

    assert(locales.isNotEmpty);
    return locales;
  }

  /// Engine code should use this method instead of the callback directly.
  /// Otherwise zones won't work properly.
  void invokeOnLocaleChanged() {
    invoke(_onLocaleChanged, _onLocaleChangedZone);
  }

  /// The system-reported text scale.
  ///
  /// This establishes the text scaling factor to use when rendering text,
  /// according to the user's platform preferences.
  ///
  /// The [onTextScaleFactorChanged] callback is called whenever this value
  /// changes.
  ///
  /// See also:
  ///
  ///  * [WidgetsBindingObserver], for a mechanism at the widgets layer to
  ///    observe when this value changes.
  @override
  double get textScaleFactor => configuration.textScaleFactor;

  /// The setting indicating whether time should always be shown in the 24-hour
  /// format.
  ///
  /// This option is used by [showTimePicker].
  @override
  bool get alwaysUse24HourFormat => configuration.alwaysUse24HourFormat;

  /// Updates [textScaleFactor] and invokes [onTextScaleFactorChanged] and
  /// [onPlatformConfigurationChanged] callbacks if [textScaleFactor] changed.
  void _updateTextScaleFactor(double value) {
    if (configuration.textScaleFactor != value) {
      configuration = configuration.copyWith(textScaleFactor: value);
      invokeOnPlatformConfigurationChanged();
      invokeOnTextScaleFactorChanged();
    }
  }

  /// Watches for font-size changes in the browser's <html> element to
  /// recalculate [textScaleFactor].
  ///
  /// Updates [textScaleFactor] with the new value.
  DomMutationObserver? _fontSizeObserver;

  /// Set the callback function for updating [textScaleFactor] based on
  /// font-size changes in the browser's <html> element.
  void _addFontSizeObserver() {
    const String styleAttribute = 'style';

    _fontSizeObserver = createDomMutationObserver((
      JSArray<JSAny?> mutations,
      DomMutationObserver _,
    ) {
      for (final JSAny? mutation in mutations.toDart) {
        final DomMutationRecord record = mutation! as DomMutationRecord;
        if (record.type == 'attributes' && record.attributeName == styleAttribute) {
          final double newTextScaleFactor = findBrowserTextScaleFactor();
          _updateTextScaleFactor(newTextScaleFactor);
        }
      }
    });
    _fontSizeObserver!.observe(
      domDocument.documentElement!,
      attributes: true,
      attributeFilter: <String>[styleAttribute],
    );
  }

  /// Remove the observer for font-size changes in the browser's <html> element.
  void _disconnectFontSizeObserver() {
    _fontSizeObserver?.disconnect();
    _fontSizeObserver = null;
  }

  void _setAppLifecycleState(ui.AppLifecycleState state) {
    invokeOnPlatformMessage(
      'flutter/lifecycle',
      const StringCodec().encodeMessage(state.toString()),
      (_) {},
    );
  }

  /// A callback that is invoked whenever [textScaleFactor] changes value.
  ///
  /// The framework invokes this callback in the same zone in which the
  /// callback was set.
  ///
  /// See also:
  ///
  ///  * [WidgetsBindingObserver], for a mechanism at the widgets layer to
  ///    observe when this callback is invoked.
  @override
  ui.VoidCallback? get onTextScaleFactorChanged => _onTextScaleFactorChanged;
  ui.VoidCallback? _onTextScaleFactorChanged;
  Zone? _onTextScaleFactorChangedZone;
  @override
  set onTextScaleFactorChanged(ui.VoidCallback? callback) {
    _onTextScaleFactorChanged = callback;
    _onTextScaleFactorChangedZone = Zone.current;
  }

  /// Engine code should use this method instead of the callback directly.
  /// Otherwise zones won't work properly.
  void invokeOnTextScaleFactorChanged() {
    invoke(_onTextScaleFactorChanged, _onTextScaleFactorChangedZone);
  }

  void updateSemanticsEnabled(bool semanticsEnabled) {
    if (semanticsEnabled != this.semanticsEnabled) {
      configuration = configuration.copyWith(semanticsEnabled: semanticsEnabled);
      if (_onSemanticsEnabledChanged != null) {
        invokeOnSemanticsEnabledChanged();
      }
    }
  }

  /// The setting indicating the current brightness mode of the host platform.
  /// If the platform has no preference, [platformBrightness] defaults to [Brightness.light].
  @override
  ui.Brightness get platformBrightness => configuration.platformBrightness;

  /// Updates [_platformBrightness] and invokes [onPlatformBrightnessChanged]
  /// callback if [_platformBrightness] changed.
  void _updatePlatformBrightness(ui.Brightness value) {
    if (configuration.platformBrightness != value) {
      configuration = configuration.copyWith(platformBrightness: value);
      invokeOnPlatformConfigurationChanged();
      invokeOnPlatformBrightnessChanged();
    }
  }

  /// The setting indicating the current system font of the host platform.
  @override
  String? get systemFontFamily => configuration.systemFontFamily;

  /// Updates [_highContrast] and invokes [onHighContrastModeChanged]
  /// callback if [_highContrast] changed.
  void _updateHighContrast(bool value) {
    if (configuration.accessibilityFeatures.highContrast != value) {
      final EngineAccessibilityFeatures original =
          configuration.accessibilityFeatures as EngineAccessibilityFeatures;
      configuration = configuration.copyWith(
        accessibilityFeatures: original.copyWith(highContrast: value),
      );
      invokeOnPlatformConfigurationChanged();
    }
  }

  /// Reference to css media query that indicates the user theme preference on the web.
  final DomMediaQueryList _brightnessMediaQuery = domWindow.matchMedia(
    '(prefers-color-scheme: dark)',
  );

  /// A callback that is invoked whenever [_brightnessMediaQuery] changes value.
  ///
  /// Updates the [_platformBrightness] with the new user preference.
  DomEventListener? _brightnessMediaQueryListener;

  /// Set the callback function for listening changes in [_brightnessMediaQuery] value.
  void _addBrightnessMediaQueryListener() {
    _updatePlatformBrightness(
      _brightnessMediaQuery.matches ? ui.Brightness.dark : ui.Brightness.light,
    );

    _brightnessMediaQueryListener = createDomEventListener((DomEvent event) {
      final DomMediaQueryListEvent mqEvent = event as DomMediaQueryListEvent;
      _updatePlatformBrightness(mqEvent.matches! ? ui.Brightness.dark : ui.Brightness.light);
    });
    _brightnessMediaQuery.addListener(_brightnessMediaQueryListener);
  }

  /// Remove the callback function for listening changes in [_brightnessMediaQuery] value.
  void _removeBrightnessMediaQueryListener() {
    _brightnessMediaQuery.removeListener(_brightnessMediaQueryListener);
    _brightnessMediaQueryListener = null;
  }

  /// A callback that is invoked whenever [platformBrightness] changes value.
  ///
  /// The framework invokes this callback in the same zone in which the
  /// callback was set.
  ///
  /// See also:
  ///
  ///  * [WidgetsBindingObserver], for a mechanism at the widgets layer to
  ///    observe when this callback is invoked.
  @override
  ui.VoidCallback? get onPlatformBrightnessChanged => _onPlatformBrightnessChanged;
  ui.VoidCallback? _onPlatformBrightnessChanged;
  Zone? _onPlatformBrightnessChangedZone;
  @override
  set onPlatformBrightnessChanged(ui.VoidCallback? callback) {
    _onPlatformBrightnessChanged = callback;
    _onPlatformBrightnessChangedZone = Zone.current;
  }

  /// Engine code should use this method instead of the callback directly.
  /// Otherwise zones won't work properly.
  void invokeOnPlatformBrightnessChanged() {
    invoke(_onPlatformBrightnessChanged, _onPlatformBrightnessChangedZone);
  }

  /// A callback that is invoked whenever [systemFontFamily] changes value.
  ///
  /// The framework invokes this callback in the same zone in which the
  /// callback was set.
  ///
  /// See also:
  ///
  ///  * [WidgetsBindingObserver], for a mechanism at the widgets layer to
  ///    observe when this callback is invoked.
  @override
  ui.VoidCallback? get onSystemFontFamilyChanged => _onSystemFontFamilyChanged;
  ui.VoidCallback? _onSystemFontFamilyChanged;
  Zone? _onSystemFontFamilyChangedZone;
  @override
  set onSystemFontFamilyChanged(ui.VoidCallback? callback) {
    _onSystemFontFamilyChanged = callback;
    _onSystemFontFamilyChangedZone = Zone.current;
  }

  /// Engine code should use this method instead of the callback directly.
  /// Otherwise zones won't work properly.
  void invokeOnSystemFontFamilyChanged() {
    invoke(_onSystemFontFamilyChanged, _onSystemFontFamilyChangedZone);
  }

  /// Whether the user has requested that [updateSemantics] be called when
  /// the semantic contents of window changes.
  ///
  /// The [onSemanticsEnabledChanged] callback is called whenever this value
  /// changes.
  @override
  bool get semanticsEnabled => configuration.semanticsEnabled;

  /// A callback that is invoked when the value of [semanticsEnabled] changes.
  ///
  /// The framework invokes this callback in the same zone in which the
  /// callback was set.
  @override
  ui.VoidCallback? get onSemanticsEnabledChanged => _onSemanticsEnabledChanged;
  ui.VoidCallback? _onSemanticsEnabledChanged;
  Zone? _onSemanticsEnabledChangedZone;
  @override
  set onSemanticsEnabledChanged(ui.VoidCallback? callback) {
    _onSemanticsEnabledChanged = callback;
    _onSemanticsEnabledChangedZone = Zone.current;
  }

  /// Engine code should use this method instead of the callback directly.
  /// Otherwise zones won't work properly.
  void invokeOnSemanticsEnabledChanged() {
    invoke(_onSemanticsEnabledChanged, _onSemanticsEnabledChangedZone);
  }

  /// A callback that is invoked whenever the user requests an action to be
  /// performed on a semantics node.
  ///
  /// This callback is used when the user expresses the action they wish to
  /// perform based on the semantics node supplied by updateSemantics.
  ///
  /// The framework invokes this callback in the same zone in which the
  /// callback was set.
  @override
  ui.SemanticsActionEventCallback? get onSemanticsActionEvent => _onSemanticsActionEvent;
  ui.SemanticsActionEventCallback? _onSemanticsActionEvent;
  Zone _onSemanticsActionEventZone = Zone.root;
  @override
  set onSemanticsActionEvent(ui.SemanticsActionEventCallback? callback) {
    _onSemanticsActionEvent = callback;
    _onSemanticsActionEventZone = Zone.current;
  }

  /// Engine code should use this method instead of the callback directly.
  /// Otherwise zones won't work properly.
  void invokeOnSemanticsAction(int viewId, int nodeId, ui.SemanticsAction action, ByteData? args) {
    invoke1<ui.SemanticsActionEvent>(
      _onSemanticsActionEvent,
      _onSemanticsActionEventZone,
      ui.SemanticsActionEvent(type: action, nodeId: nodeId, viewId: viewId, arguments: args),
    );
  }

  // TODO(dnfield): make this work on web.
  // https://github.com/flutter/flutter/issues/100277
  ui.ErrorCallback? _onError;
  // ignore: unused_field
  late Zone _onErrorZone;
  @override
  ui.ErrorCallback? get onError => _onError;
  @override
  set onError(ui.ErrorCallback? callback) {
    _onError = callback;
    _onErrorZone = Zone.current;
  }

  /// The route or path that the embedder requested when the application was
  /// launched.
  ///
  /// This will be the string "`/`" if no particular route was requested.
  ///
  /// ## Android
  ///
  /// On Android, calling
  /// [`FlutterView.setInitialRoute`](/javadoc/io/flutter/view/FlutterView.html#setInitialRoute-java.lang.String-)
  /// will set this value. The value must be set sufficiently early, i.e. before
  /// the [runApp] call is executed in Dart, for this to have any effect on the
  /// framework. The `createFlutterView` method in your `FlutterActivity`
  /// subclass is a suitable time to set the value. The application's
  /// `AndroidManifest.xml` file must also be updated to have a suitable
  /// [`<intent-filter>`](https://developer.android.com/guide/topics/manifest/intent-filter-element.html).
  ///
  /// ## iOS
  ///
  /// On iOS, calling
  /// [`FlutterViewController.setInitialRoute`](/ios-embedder/interface_flutter_view_controller.html#a7f269c2da73312f856d42611cc12a33f)
  /// will set this value. The value must be set sufficiently early, i.e. before
  /// the [runApp] call is executed in Dart, for this to have any effect on the
  /// framework. The `application:didFinishLaunchingWithOptions:` method is a
  /// suitable time to set this value.
  ///
  /// See also:
  ///
  ///  * [Navigator], a widget that handles routing.
  ///  * [SystemChannels.navigation], which handles subsequent navigation
  ///    requests from the embedder.
  @override
  String get defaultRouteName {
    // TODO(mdebbar): What should we do in multi-view mode?
    //                https://github.com/flutter/flutter/issues/139174
    return _defaultRouteName ??= implicitView?.browserHistory.currentPath ?? '/';
  }

  /// Lazily initialized when the `defaultRouteName` getter is invoked.
  ///
  /// The reason for the lazy initialization is to give enough time for the app
  /// to set [locationStrategy] in `lib/initialization.dart`.
  String? _defaultRouteName;

  /// In Flutter, platform messages are exchanged between threads so the
  /// messages and responses have to be exchanged asynchronously. We simulate
  /// that by adding a zero-length delay to the reply.
  void replyToPlatformMessage(ui.PlatformMessageResponseCallback? callback, ByteData? data) {
    Future<void>.delayed(Duration.zero).then((_) {
      if (callback != null) {
        callback(data);
      }
    });
  }

  @override
  ui.FrameData get frameData => const ui.FrameData.webOnly();

  @override
  double scaleFontSize(double unscaledFontSize) => unscaledFontSize * textScaleFactor;
}

bool _handleWebTestEnd2EndMessage(MethodCodec codec, ByteData? data) {
  final MethodCall decoded = codec.decodeMethodCall(data);
  final double ratio = double.parse(decoded.arguments as String);
  switch (decoded.method) {
    case 'setDevicePixelRatio':
      EngineFlutterDisplay.instance.debugOverrideDevicePixelRatio(ratio);
      EnginePlatformDispatcher.instance.onMetricsChanged!();
      return true;
  }
  return false;
}

/// Invokes [callback] inside the given [zone].
void invoke(void Function()? callback, Zone? zone) {
  if (callback == null) {
    return;
  }

  assert(zone != null);

  if (identical(zone, Zone.current)) {
    callback();
  } else {
    zone!.runGuarded(callback);
  }
}

/// Invokes [callback] inside the given [zone] passing it [arg].
void invoke1<A>(void Function(A a)? callback, Zone? zone, A arg) {
  if (callback == null) {
    return;
  }

  assert(zone != null);

  if (identical(zone, Zone.current)) {
    callback(arg);
  } else {
    zone!.runUnaryGuarded<A>(callback, arg);
  }
}

/// Invokes [callback] inside the given [zone] passing it [arg1] and [arg2].
void invoke2<A1, A2>(void Function(A1 a1, A2 a2)? callback, Zone? zone, A1 arg1, A2 arg2) {
  if (callback == null) {
    return;
  }

  assert(zone != null);

  if (identical(zone, Zone.current)) {
    callback(arg1, arg2);
  } else {
    zone!.runGuarded(() {
      callback(arg1, arg2);
    });
  }
}

/// Invokes [callback] inside the given [zone] passing it [arg1], [arg2], and [arg3].
void invoke3<A1, A2, A3>(
  void Function(A1 a1, A2 a2, A3 a3)? callback,
  Zone? zone,
  A1 arg1,
  A2 arg2,
  A3 arg3,
) {
  if (callback == null) {
    return;
  }

  assert(zone != null);

  if (identical(zone, Zone.current)) {
    callback(arg1, arg2, arg3);
  } else {
    zone!.runGuarded(() {
      callback(arg1, arg2, arg3);
    });
  }
}

const double _defaultRootFontSize = 16.0;

/// Finds the text scale factor of the browser by looking at the computed style
/// of the browser's <html> element.
double findBrowserTextScaleFactor() {
  final num fontSize = parseFontSize(domDocument.documentElement!) ?? _defaultRootFontSize;
  return fontSize / _defaultRootFontSize;
}

class ViewConfiguration {
  const ViewConfiguration({
    this.view,
    this.devicePixelRatio = 1.0,
    this.visible = false,
    this.viewInsets = ui.ViewPadding.zero as ViewPadding,
    this.viewPadding = ui.ViewPadding.zero as ViewPadding,
    this.systemGestureInsets = ui.ViewPadding.zero as ViewPadding,
    this.padding = ui.ViewPadding.zero as ViewPadding,
    this.gestureSettings = const ui.GestureSettings(),
    this.displayFeatures = const <ui.DisplayFeature>[],
  });

  ViewConfiguration copyWith({
    EngineFlutterView? view,
    double? devicePixelRatio,
    bool? visible,
    ViewPadding? viewInsets,
    ViewPadding? viewPadding,
    ViewPadding? systemGestureInsets,
    ViewPadding? padding,
    ui.GestureSettings? gestureSettings,
    List<ui.DisplayFeature>? displayFeatures,
  }) {
    return ViewConfiguration(
      view: view ?? this.view,
      devicePixelRatio: devicePixelRatio ?? this.devicePixelRatio,
      visible: visible ?? this.visible,
      viewInsets: viewInsets ?? this.viewInsets,
      viewPadding: viewPadding ?? this.viewPadding,
      systemGestureInsets: systemGestureInsets ?? this.systemGestureInsets,
      padding: padding ?? this.padding,
      gestureSettings: gestureSettings ?? this.gestureSettings,
      displayFeatures: displayFeatures ?? this.displayFeatures,
    );
  }

  final EngineFlutterView? view;
  final double devicePixelRatio;
  final bool visible;
  final ViewPadding viewInsets;
  final ViewPadding viewPadding;
  final ViewPadding systemGestureInsets;
  final ViewPadding padding;
  final ui.GestureSettings gestureSettings;
  final List<ui.DisplayFeature> displayFeatures;

  @override
  String toString() {
    return '$runtimeType[view: $view]';
  }
}

class PlatformConfiguration {
  const PlatformConfiguration({
    this.accessibilityFeatures = const EngineAccessibilityFeatures(0),
    this.alwaysUse24HourFormat = false,
    this.semanticsEnabled = false,
    this.platformBrightness = ui.Brightness.light,
    this.textScaleFactor = 1.0,
    this.locales = const <ui.Locale>[],
    this.defaultRouteName = '/',
    this.systemFontFamily,
  });

  PlatformConfiguration copyWith({
    ui.AccessibilityFeatures? accessibilityFeatures,
    bool? alwaysUse24HourFormat,
    bool? semanticsEnabled,
    ui.Brightness? platformBrightness,
    double? textScaleFactor,
    List<ui.Locale>? locales,
    String? defaultRouteName,
    String? systemFontFamily,
  }) {
    return PlatformConfiguration(
      accessibilityFeatures: accessibilityFeatures ?? this.accessibilityFeatures,
      alwaysUse24HourFormat: alwaysUse24HourFormat ?? this.alwaysUse24HourFormat,
      semanticsEnabled: semanticsEnabled ?? this.semanticsEnabled,
      platformBrightness: platformBrightness ?? this.platformBrightness,
      textScaleFactor: textScaleFactor ?? this.textScaleFactor,
      locales: locales ?? this.locales,
      defaultRouteName: defaultRouteName ?? this.defaultRouteName,
      systemFontFamily: systemFontFamily ?? this.systemFontFamily,
    );
  }

  final ui.AccessibilityFeatures accessibilityFeatures;
  final bool alwaysUse24HourFormat;
  final bool semanticsEnabled;
  final ui.Brightness platformBrightness;
  final double textScaleFactor;
  final List<ui.Locale> locales;
  final String defaultRouteName;
  final String? systemFontFamily;
}
