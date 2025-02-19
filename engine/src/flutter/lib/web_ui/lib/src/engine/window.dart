// Copyright 2013 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

import 'dart:async';
import 'dart:typed_data';

import 'package:meta/meta.dart';
import 'package:ui/ui.dart' as ui;
import 'package:ui/ui_web/src/ui_web.dart' as ui_web;

import '../engine.dart' show DimensionsProvider, registerHotRestartListener, renderer;
import 'browser_detection.dart';
import 'display.dart';
import 'dom.dart';
import 'initialization.dart';
import 'js_interop/js_app.dart';
import 'mouse/context_menu.dart';
import 'mouse/cursor.dart';
import 'navigation/history.dart';
import 'platform_dispatcher.dart';
import 'pointer_binding.dart';
import 'semantics.dart';
import 'services.dart';
import 'text_editing/text_editing.dart';
import 'util.dart';
import 'view_embedder/dom_manager.dart';
import 'view_embedder/embedding_strategy/embedding_strategy.dart';
import 'view_embedder/global_html_attributes.dart';
import 'view_embedder/style_manager.dart';

typedef _HandleMessageCallBack = Future<bool> Function();

/// When set to true, all platform messages will be printed to the console.
const bool debugPrintPlatformMessages = false;

/// The view ID for the implicit flutter view provided by the platform.
const int kImplicitViewId = 0;

int _nextViewId = kImplicitViewId + 1;

/// Represents all views in the Flutter Web Engine.
///
/// In addition to everything defined in [ui.FlutterView], this class adds
/// a few web-specific properties.
class EngineFlutterView implements ui.FlutterView {
  /// Creates a [ui.FlutterView] that can be used in multi-view mode.
  ///
  /// The [hostElement] parameter specifies the container in the DOM into which
  /// the Flutter view will be rendered.
  factory EngineFlutterView(
    EnginePlatformDispatcher platformDispatcher,
    DomElement hostElement, {
    JsViewConstraints? viewConstraints,
  }) = _EngineFlutterViewImpl;

  EngineFlutterView._(
    this.viewId,
    this.platformDispatcher,
    // This is nullable to accommodate the legacy `EngineFlutterWindow`. In
    // multi-view mode, the host element is required for each view (as reflected
    // by the public `EngineFlutterView` constructor).
    DomElement? hostElement, {
    JsViewConstraints? viewConstraints,
  }) : _jsViewConstraints = viewConstraints,
       embeddingStrategy = EmbeddingStrategy.create(hostElement: hostElement),
       dimensionsProvider = DimensionsProvider.create(hostElement: hostElement) {
    // The embeddingStrategy will take care of cleaning up the rootElement on
    // hot restart.
    embeddingStrategy.attachViewRoot(dom.rootElement);
    pointerBinding = PointerBinding(this);
    _resizeSubscription = onResize.listen(_didResize);
    _globalHtmlAttributes.applyAttributes(
      viewId: viewId,
      rendererTag: renderer.rendererTag,
      buildMode: buildMode,
    );
    registerHotRestartListener(dispose);
  }

  static EngineFlutterWindow implicit(
    EnginePlatformDispatcher platformDispatcher,
    DomElement? hostElement,
  ) => EngineFlutterWindow._(platformDispatcher, hostElement);

  @override
  final int viewId;

  @override
  final EnginePlatformDispatcher platformDispatcher;

  /// Abstracts all the DOM manipulations required to embed a Flutter view in a user-supplied `hostElement`.
  final EmbeddingStrategy embeddingStrategy;

  late final StreamSubscription<ui.Size?> _resizeSubscription;

  final ViewConfiguration _viewConfiguration = const ViewConfiguration();

  /// Whether this [EngineFlutterView] has been disposed or not.
  bool isDisposed = false;

  /// Disposes of the [EngineFlutterView] instance and undoes all of its DOM
  /// tree and any event listeners.
  @mustCallSuper
  void dispose() {
    if (isDisposed) {
      return;
    }
    isDisposed = true;
    _resizeSubscription.cancel();
    dimensionsProvider.close();
    pointerBinding.dispose();
    dom.rootElement.remove();
    // TODO(harryterkelsen): What should we do about this in multi-view?
    renderer.clearFragmentProgramCache();
    semantics.reset();
  }

  @override
  void render(ui.Scene scene, {ui.Size? size}) {
    assert(!isDisposed, 'Trying to render a disposed EngineFlutterView.');
    if (size != null) {
      resize(size);
    }
    platformDispatcher.render(scene, this);
  }

  @override
  void updateSemantics(ui.SemanticsUpdate update) {
    assert(!isDisposed, 'Trying to update semantics on a disposed EngineFlutterView.');
    semantics.updateSemantics(update);
  }

  late final GlobalHtmlAttributes _globalHtmlAttributes = GlobalHtmlAttributes(
    rootElement: dom.rootElement,
    hostElement: embeddingStrategy.hostElement,
  );

  late final MouseCursor mouseCursor = MouseCursor(dom.rootElement);

  late final ContextMenu contextMenu = ContextMenu(dom.rootElement);

  late final DomManager dom = DomManager(devicePixelRatio: devicePixelRatio);

  late final PointerBinding pointerBinding;

  @override
  ViewConstraints get physicalConstraints {
    final double dpr = devicePixelRatio;
    final ui.Size currentLogicalSize = physicalSize / dpr;
    return ViewConstraints.fromJs(_jsViewConstraints, currentLogicalSize) * dpr;
  }

  final JsViewConstraints? _jsViewConstraints;

  late final EngineSemanticsOwner semantics = EngineSemanticsOwner(viewId, dom.semanticsHost);

  @override
  ui.Size get physicalSize {
    return _physicalSize ??= _computePhysicalSize();
  }

  /// Resizes the `rootElement` to `newPhysicalSize` by changing its CSS style.
  ///
  /// This is used by the [render] method, when the framework sends new dimensions
  /// for the current Flutter View.
  ///
  /// Dimensions from the framework are constrained by the [physicalConstraints]
  /// that can be configured by the user when adding a view to the app.
  ///
  /// In practice, this method changes the size of the `rootElement` of the app
  /// so it can push/shrink inside its `hostElement`. That way, a Flutter app
  /// can change the layout of the container page.
  ///
  /// ```none
  /// <p>Some HTML content...</p>
  /// +--- (div) hostElement ------------------------------------+
  /// | +--- rootElement ---------------------+                  |
  /// | |                                     |                  |
  /// | |                                     |    container     |
  /// | |    size applied to *this*           |    must be able  |
  /// | |                                     |    to reflow     |
  /// | |                                     |                  |
  /// | +-------------------------------------+                  |
  /// +----------------------------------------------------------+
  /// <p>More HTML content...</p>
  /// ```
  ///
  /// The `hostElement` needs to be styled in a way that allows its size to flow
  /// with its contents. Things like `max-height: 100px; overflow: hidden` will
  /// work as expected (by hiding the overflowing part of the flutter app), but
  /// if in that case flutter is not made aware of that max-height with
  /// `physicalConstraints`, it will end up rendering more pixels that are visible
  /// on the screen, with a possible hit to performance.
  ///
  /// TL;DR: The `viewConstraints` of a Flutter view, must take into consideration
  /// the CSS box-model restrictions imposed on its `hostElement` (especially when
  /// hiding `overflow`). Flutter does not attempt to interpret the styles of
  /// `hostElement` to compute its `physicalConstraints`, only its current size.
  void resize(ui.Size newPhysicalSize) {
    // The browser uses CSS, and CSS operates in logical sizes.
    final ui.Size logicalSize = newPhysicalSize / devicePixelRatio;
    dom.rootElement.style
      ..width = '${logicalSize.width}px'
      ..height = '${logicalSize.height}px';

    // Force an update of the physicalSize so it's ready for the renderer.
    _computePhysicalSize();
  }

  /// Lazily populated and cleared at the end of the frame.
  ui.Size? _physicalSize;

  ui.Size? debugPhysicalSizeOverride;

  /// Computes the physical size of the view.
  ///
  /// This function is expensive. It triggers browser layout if there are
  /// pending DOM writes.
  ui.Size _computePhysicalSize() {
    ui.Size? physicalSizeOverride;

    assert(() {
      physicalSizeOverride = debugPhysicalSizeOverride;
      return true;
    }());

    return physicalSizeOverride ?? dimensionsProvider.computePhysicalSize();
  }

  /// Forces the view to recompute its physical size. Useful for tests.
  void debugForceResize() {
    _physicalSize = _computePhysicalSize();
  }

  @override
  ViewPadding get viewInsets => _viewInsets;
  ViewPadding _viewInsets = ui.ViewPadding.zero as ViewPadding;

  @override
  ViewPadding get viewPadding => _viewConfiguration.viewPadding;

  @override
  ViewPadding get systemGestureInsets => _viewConfiguration.systemGestureInsets;

  @override
  ViewPadding get padding => _viewConfiguration.padding;

  @override
  ui.GestureSettings get gestureSettings => _viewConfiguration.gestureSettings;

  @override
  List<ui.DisplayFeature> get displayFeatures => _viewConfiguration.displayFeatures;

  @override
  EngineFlutterDisplay get display => EngineFlutterDisplay.instance;

  @override
  double get devicePixelRatio => display.devicePixelRatio;

  @visibleForTesting
  final DimensionsProvider dimensionsProvider;

  Stream<ui.Size?> get onResize => dimensionsProvider.onResize;

  /// Called immediately after the view has been resized.
  ///
  /// When there is a text editing going on in mobile devices, do not change
  /// the physicalSize, change the [window.viewInsets]. See:
  /// https://api.flutter.dev/flutter/dart-ui/FlutterView/viewInsets.html
  /// https://api.flutter.dev/flutter/dart-ui/FlutterView/physicalSize.html
  ///
  /// Note: always check for rotations for a mobile device. Update the physical
  /// size if the change is caused by a rotation.
  void _didResize(ui.Size? newSize) {
    StyleManager.scaleSemanticsHost(dom.semanticsHost, devicePixelRatio);
    final ui.Size newPhysicalSize = _computePhysicalSize();
    final bool isEditingOnMobile =
        isMobile && !_isRotation(newPhysicalSize) && textEditing.isEditing;
    if (isEditingOnMobile) {
      _computeOnScreenKeyboardInsets(true);
    } else {
      _physicalSize = newPhysicalSize;
      // When physical size changes this value has to be recalculated.
      _computeOnScreenKeyboardInsets(false);
    }
    platformDispatcher.invokeOnMetricsChanged();
  }

  /// Uses the previous physical size and current innerHeight/innerWidth
  /// values to decide if a device is rotating.
  ///
  /// During a rotation the height and width values will (almost) swap place.
  /// Values can slightly differ due to space occupied by the browser header.
  /// For example the following values are collected for Pixel 3 rotation:
  ///
  /// height: 658 width: 393
  /// new height: 313 new width: 738
  ///
  /// The following values are from a changed caused by virtual keyboard.
  ///
  /// height: 658 width: 393
  /// height: 368 width: 393
  bool _isRotation(ui.Size newPhysicalSize) {
    // This method compares the new dimensions with the previous ones.
    // Return false if the previous dimensions are not set.
    if (_physicalSize != null) {
      // First confirm both height and width are effected.
      if (_physicalSize!.height != newPhysicalSize.height &&
          _physicalSize!.width != newPhysicalSize.width) {
        // If prior to rotation height is bigger than width it should be the
        // opposite after the rotation and vice versa.
        if ((_physicalSize!.height > _physicalSize!.width &&
                newPhysicalSize.height < newPhysicalSize.width) ||
            (_physicalSize!.width > _physicalSize!.height &&
                newPhysicalSize.width < newPhysicalSize.height)) {
          // Rotation detected
          return true;
        }
      }
    }
    return false;
  }

  void _computeOnScreenKeyboardInsets(bool isEditingOnMobile) {
    _viewInsets = dimensionsProvider.computeKeyboardInsets(
      _physicalSize!.height,
      isEditingOnMobile,
    );
  }
}

final class _EngineFlutterViewImpl extends EngineFlutterView {
  _EngineFlutterViewImpl(
    EnginePlatformDispatcher platformDispatcher,
    DomElement hostElement, {
    JsViewConstraints? viewConstraints,
  }) : super._(_nextViewId++, platformDispatcher, hostElement, viewConstraints: viewConstraints);
}

/// The Web implementation of [ui.SingletonFlutterWindow].
final class EngineFlutterWindow extends EngineFlutterView implements ui.SingletonFlutterWindow {
  EngineFlutterWindow._(EnginePlatformDispatcher platformDispatcher, DomElement? hostElement)
    : super._(kImplicitViewId, platformDispatcher, hostElement) {
    if (ui_web.isCustomUrlStrategySet) {
      _browserHistory = createHistoryForExistingState(ui_web.urlStrategy);
    }
  }

  @override
  void dispose() {
    super.dispose();
    _browserHistory?.dispose();
  }

  @override
  ui.VoidCallback? get onMetricsChanged => platformDispatcher.onMetricsChanged;
  @override
  set onMetricsChanged(ui.VoidCallback? callback) {
    platformDispatcher.onMetricsChanged = callback;
  }

  @override
  ui.Locale get locale => platformDispatcher.locale;
  @override
  List<ui.Locale> get locales => platformDispatcher.locales;

  @override
  ui.Locale? computePlatformResolvedLocale(List<ui.Locale> supportedLocales) {
    return platformDispatcher.computePlatformResolvedLocale(supportedLocales);
  }

  @override
  ui.VoidCallback? get onLocaleChanged => platformDispatcher.onLocaleChanged;
  @override
  set onLocaleChanged(ui.VoidCallback? callback) {
    platformDispatcher.onLocaleChanged = callback;
  }

  @override
  String get initialLifecycleState => platformDispatcher.initialLifecycleState;

  @override
  double get textScaleFactor => platformDispatcher.textScaleFactor;

  @override
  bool get nativeSpellCheckServiceDefined => platformDispatcher.nativeSpellCheckServiceDefined;

  @override
  bool get supportsShowingSystemContextMenu => platformDispatcher.supportsShowingSystemContextMenu;

  @override
  bool get brieflyShowPassword => platformDispatcher.brieflyShowPassword;

  @override
  bool get alwaysUse24HourFormat => platformDispatcher.alwaysUse24HourFormat;

  @override
  ui.VoidCallback? get onTextScaleFactorChanged => platformDispatcher.onTextScaleFactorChanged;
  @override
  set onTextScaleFactorChanged(ui.VoidCallback? callback) {
    platformDispatcher.onTextScaleFactorChanged = callback;
  }

  @override
  ui.Brightness get platformBrightness => platformDispatcher.platformBrightness;

  @override
  ui.VoidCallback? get onPlatformBrightnessChanged =>
      platformDispatcher.onPlatformBrightnessChanged;
  @override
  set onPlatformBrightnessChanged(ui.VoidCallback? callback) {
    platformDispatcher.onPlatformBrightnessChanged = callback;
  }

  @override
  String? get systemFontFamily => platformDispatcher.systemFontFamily;

  @override
  ui.VoidCallback? get onSystemFontFamilyChanged => platformDispatcher.onSystemFontFamilyChanged;
  @override
  set onSystemFontFamilyChanged(ui.VoidCallback? callback) {
    platformDispatcher.onSystemFontFamilyChanged = callback;
  }

  @override
  ui.FrameCallback? get onBeginFrame => platformDispatcher.onBeginFrame;
  @override
  set onBeginFrame(ui.FrameCallback? callback) {
    platformDispatcher.onBeginFrame = callback;
  }

  @override
  ui.VoidCallback? get onDrawFrame => platformDispatcher.onDrawFrame;
  @override
  set onDrawFrame(ui.VoidCallback? callback) {
    platformDispatcher.onDrawFrame = callback;
  }

  @override
  ui.TimingsCallback? get onReportTimings => platformDispatcher.onReportTimings;
  @override
  set onReportTimings(ui.TimingsCallback? callback) {
    platformDispatcher.onReportTimings = callback;
  }

  @override
  ui.PointerDataPacketCallback? get onPointerDataPacket => platformDispatcher.onPointerDataPacket;
  @override
  set onPointerDataPacket(ui.PointerDataPacketCallback? callback) {
    platformDispatcher.onPointerDataPacket = callback;
  }

  @override
  ui.KeyDataCallback? get onKeyData => platformDispatcher.onKeyData;
  @override
  set onKeyData(ui.KeyDataCallback? callback) {
    platformDispatcher.onKeyData = callback;
  }

  @override
  String get defaultRouteName => platformDispatcher.defaultRouteName;

  @override
  void scheduleFrame() => platformDispatcher.scheduleFrame();

  @override
  bool get semanticsEnabled => platformDispatcher.semanticsEnabled;

  @override
  ui.VoidCallback? get onSemanticsEnabledChanged => platformDispatcher.onSemanticsEnabledChanged;
  @override
  set onSemanticsEnabledChanged(ui.VoidCallback? callback) {
    platformDispatcher.onSemanticsEnabledChanged = callback;
  }

  @override
  ui.FrameData get frameData => const ui.FrameData.webOnly();

  @override
  ui.VoidCallback? get onFrameDataChanged => null;
  @override
  set onFrameDataChanged(ui.VoidCallback? callback) {}

  @override
  ui.AccessibilityFeatures get accessibilityFeatures => platformDispatcher.accessibilityFeatures;

  @override
  ui.VoidCallback? get onAccessibilityFeaturesChanged =>
      platformDispatcher.onAccessibilityFeaturesChanged;
  @override
  set onAccessibilityFeaturesChanged(ui.VoidCallback? callback) {
    platformDispatcher.onAccessibilityFeaturesChanged = callback;
  }

  @override
  void sendPlatformMessage(
    String name,
    ByteData? data,
    ui.PlatformMessageResponseCallback? callback,
  ) {
    platformDispatcher.sendPlatformMessage(name, data, callback);
  }

  @override
  ui.PlatformMessageCallback? get onPlatformMessage => platformDispatcher.onPlatformMessage;
  @override
  set onPlatformMessage(ui.PlatformMessageCallback? callback) {
    platformDispatcher.onPlatformMessage = callback;
  }

  @override
  void setIsolateDebugName(String name) => ui.PlatformDispatcher.instance.setIsolateDebugName(name);

  /// Handles the browser history integration to allow users to use the back
  /// button, etc.
  BrowserHistory get browserHistory {
    return _browserHistory ??= createHistoryForExistingState(_urlStrategyForInitialization);
  }

  ui_web.UrlStrategy? get _urlStrategyForInitialization {
    // Prevent any further customization of URL strategy.
    ui_web.preventCustomUrlStrategy();
    return ui_web.urlStrategy;
  }

  BrowserHistory?
  _browserHistory; // Must be either SingleEntryBrowserHistory or MultiEntriesBrowserHistory.

  Future<void> _useSingleEntryBrowserHistory() async {
    // Recreate the browser history mode that's appropriate for the existing
    // history state.
    //
    // If it happens to be a single-entry one, then there's nothing further to do.
    //
    // But if it's a multi-entry one, it will be torn down below and replaced
    // with a single-entry history.
    //
    // See: https://github.com/flutter/flutter/issues/79241
    _browserHistory ??= createHistoryForExistingState(_urlStrategyForInitialization);

    if (_browserHistory is SingleEntryBrowserHistory) {
      return;
    }

    // At this point, we know that `_browserHistory` is a non-null
    // `MultiEntriesBrowserHistory` instance.
    final ui_web.UrlStrategy? strategy = _browserHistory?.urlStrategy;
    await _browserHistory?.tearDown();
    _browserHistory = SingleEntryBrowserHistory(urlStrategy: strategy);
  }

  Future<void> _useMultiEntryBrowserHistory() async {
    // Recreate the browser history mode that's appropriate for the existing
    // history state.
    //
    // If it happens to be a multi-entry one, then there's nothing further to do.
    //
    // But if it's a single-entry one, it will be torn down below and replaced
    // with a multi-entry history.
    //
    // See: https://github.com/flutter/flutter/issues/79241
    _browserHistory ??= createHistoryForExistingState(_urlStrategyForInitialization);

    if (_browserHistory is MultiEntriesBrowserHistory) {
      return;
    }

    // At this point, we know that `_browserHistory` is a non-null
    // `SingleEntryBrowserHistory` instance.
    final ui_web.UrlStrategy? strategy = _browserHistory?.urlStrategy;
    await _browserHistory?.tearDown();
    _browserHistory = MultiEntriesBrowserHistory(urlStrategy: strategy);
  }

  @visibleForTesting
  Future<void> debugInitializeHistory(
    ui_web.UrlStrategy? strategy, {
    required bool useSingle,
  }) async {
    await _browserHistory?.tearDown();

    ui_web.urlStrategy = strategy;
    if (useSingle) {
      _browserHistory = SingleEntryBrowserHistory(urlStrategy: strategy);
    } else {
      _browserHistory = MultiEntriesBrowserHistory(urlStrategy: strategy);
    }
  }

  Future<void> resetHistory() async {
    await _browserHistory?.tearDown();
    _browserHistory = null;
    ui_web.debugResetCustomUrlStrategy();
  }

  Future<void> _endOfTheLine = Future<void>.value();

  Future<bool> _waitInTheLine(_HandleMessageCallBack callback) async {
    final Future<void> currentPosition = _endOfTheLine;
    final Completer<void> completer = Completer<void>();
    _endOfTheLine = completer.future;
    await currentPosition;
    bool result = false;
    try {
      result = await callback();
    } finally {
      completer.complete();
    }
    return result;
  }

  Future<bool> handleNavigationMessage(ByteData? data) async {
    return _waitInTheLine(() async {
      final MethodCall decoded = const JSONMethodCodec().decodeMethodCall(data);
      final Map<String, dynamic>? arguments = decoded.arguments as Map<String, dynamic>?;
      switch (decoded.method) {
        case 'selectMultiEntryHistory':
          await _useMultiEntryBrowserHistory();
          return true;
        case 'selectSingleEntryHistory':
          await _useSingleEntryBrowserHistory();
          return true;
        // the following cases assert that arguments are not null
        case 'routeUpdated': // deprecated
          assert(arguments != null);
          await _useSingleEntryBrowserHistory();
          browserHistory.setRouteName(arguments!.tryString('routeName'));
          return true;
        case 'routeInformationUpdated':
          assert(arguments != null);
          final String? uriString = arguments!.tryString('uri');
          final String path;
          if (uriString != null) {
            final Uri uri = Uri.parse(uriString);
            // Need to remove scheme and authority.
            path = Uri.decodeComponent(
              Uri(
                path: uri.path.isEmpty ? '/' : uri.path,
                queryParameters: uri.queryParametersAll.isEmpty ? null : uri.queryParametersAll,
                fragment: uri.fragment.isEmpty ? null : uri.fragment,
              ).toString(),
            );
          } else {
            path = arguments.tryString('location')!;
          }
          browserHistory.setRouteName(
            path,
            state: arguments['state'],
            replace: arguments.tryBool('replace') ?? false,
          );
          return true;
      }
      return false;
    });
  }
}

/// The window singleton.
///
/// `dart:ui` window delegates to this value. However, this value has a wider
/// API surface, providing Web-specific functionality that the standard
/// `dart:ui` version does not.
EngineFlutterWindow get window {
  assert(
    _window != null,
    'Trying to access the implicit FlutterView, but it is not available.\n'
    'Note: the implicit FlutterView is not available in multi-view mode.',
  );
  return _window!;
}

EngineFlutterWindow? _window;

/// Initializes the [window] (aka the implicit view), if it's not already
/// initialized.
EngineFlutterWindow ensureImplicitViewInitialized({DomElement? hostElement}) {
  if (_window == null) {
    _window = EngineFlutterView.implicit(EnginePlatformDispatcher.instance, hostElement);
    EnginePlatformDispatcher.instance.viewManager.registerView(_window!);
  }
  return _window!;
}

/// The Web implementation of [ui.ViewPadding].
class ViewPadding implements ui.ViewPadding {
  const ViewPadding({
    required this.left,
    required this.top,
    required this.right,
    required this.bottom,
  });

  @override
  final double left;
  @override
  final double top;
  @override
  final double right;
  @override
  final double bottom;
}

class ViewConstraints implements ui.ViewConstraints {
  const ViewConstraints({
    this.minWidth = 0.0,
    this.maxWidth = double.infinity,
    this.minHeight = 0.0,
    this.maxHeight = double.infinity,
  });

  ViewConstraints.tight(ui.Size size)
    : minWidth = size.width,
      maxWidth = size.width,
      minHeight = size.height,
      maxHeight = size.height;

  /// Converts JsViewConstraints into ViewConstraints.
  ///
  /// Since JsViewConstraints are expressed by the user, in logical pixels, this
  /// conversion uses logical pixels for the current size as well.
  ///
  /// The resulting ViewConstraints object will be multiplied by devicePixelRatio
  /// later to compute the physicalViewConstraints, which is what the framework
  /// uses.
  factory ViewConstraints.fromJs(JsViewConstraints? constraints, ui.Size currentLogicalSize) {
    if (constraints == null) {
      return ViewConstraints.tight(currentLogicalSize);
    }
    return ViewConstraints(
      minWidth: _computeMinConstraintValue(constraints.minWidth, currentLogicalSize.width),
      minHeight: _computeMinConstraintValue(constraints.minHeight, currentLogicalSize.height),
      maxWidth: _computeMaxConstraintValue(constraints.maxWidth, currentLogicalSize.width),
      maxHeight: _computeMaxConstraintValue(constraints.maxHeight, currentLogicalSize.height),
    );
  }

  @override
  final double minWidth;
  @override
  final double maxWidth;
  @override
  final double minHeight;
  @override
  final double maxHeight;

  @override
  bool isSatisfiedBy(ui.Size size) {
    return (minWidth <= size.width) &&
        (size.width <= maxWidth) &&
        (minHeight <= size.height) &&
        (size.height <= maxHeight);
  }

  @override
  bool get isTight => minWidth >= maxWidth && minHeight >= maxHeight;

  ViewConstraints operator *(double factor) {
    return ViewConstraints(
      minWidth: minWidth * factor,
      maxWidth: maxWidth * factor,
      minHeight: minHeight * factor,
      maxHeight: maxHeight * factor,
    );
  }

  @override
  ViewConstraints operator /(double factor) {
    return ViewConstraints(
      minWidth: minWidth / factor,
      maxWidth: maxWidth / factor,
      minHeight: minHeight / factor,
      maxHeight: maxHeight / factor,
    );
  }

  @override
  bool operator ==(Object other) {
    if (identical(this, other)) {
      return true;
    }
    if (other.runtimeType != runtimeType) {
      return false;
    }
    return other is ViewConstraints &&
        other.minWidth == minWidth &&
        other.maxWidth == maxWidth &&
        other.minHeight == minHeight &&
        other.maxHeight == maxHeight;
  }

  @override
  int get hashCode => Object.hash(minWidth, maxWidth, minHeight, maxHeight);

  @override
  String toString() {
    if (minWidth == double.infinity && minHeight == double.infinity) {
      return 'ViewConstraints(biggest)';
    }
    if (minWidth == 0 &&
        maxWidth == double.infinity &&
        minHeight == 0 &&
        maxHeight == double.infinity) {
      return 'ViewConstraints(unconstrained)';
    }
    String describe(double min, double max, String dim) {
      if (min == max) {
        return '$dim=${min.toStringAsFixed(1)}';
      }
      return '${min.toStringAsFixed(1)}<=$dim<=${max.toStringAsFixed(1)}';
    }

    final String width = describe(minWidth, maxWidth, 'w');
    final String height = describe(minHeight, maxHeight, 'h');
    return 'ViewConstraints($width, $height)';
  }
}

// Computes the "min" value for a constraint that takes into account user `desired`
// configuration and the actual available value.
//
// Returns the `desired` value unless it is `null`, in which case it returns the
// `available` value.
double _computeMinConstraintValue(double? desired, double available) {
  assert(desired == null || desired >= 0, 'Minimum constraint must be >= 0 if set.');
  assert(desired == null || desired.isFinite, 'Minimum constraint must be finite.');
  return desired ?? available;
}

// Computes the "max" value for a constraint that takes into account user `desired`
// configuration and the `available` size.
//
// Returns the `desired` value unless it is `null`, in which case it returns the
// `available` value.
//
// A `desired` value of `Infinity` or `Number.POSITIVE_INFINITY` (from JS) means
// "unconstrained".
//
// This method allows returning values larger than `available`, so the Flutter
// app is able to stretch its container up to a certain value, without being
// fully unconstrained.
double _computeMaxConstraintValue(double? desired, double available) {
  assert(desired == null || desired >= 0, 'Maximum constraint must be >= 0 if set.');
  return desired ?? available;
}
