// Copyright 2014 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

/// @docImport 'package:flutter/material.dart';
///
/// @docImport 'binding.dart';
/// @docImport 'widget_tester.dart';
library;

import 'dart:ui' hide window;

import 'package:flutter/foundation.dart';
import 'package:flutter/widgets.dart';

/// Test version of [AccessibilityFeatures] in which specific features may
/// be set to arbitrary values.
///
/// By default, all features are disabled. For an instance where all the
/// features are enabled, consider the [FakeAccessibilityFeatures.allOn]
/// constant.
@immutable
class FakeAccessibilityFeatures implements AccessibilityFeatures {
  /// Creates a test instance of [AccessibilityFeatures].
  ///
  /// By default, all features are disabled.
  const FakeAccessibilityFeatures({
    this.accessibleNavigation = false,
    this.invertColors = false,
    this.disableAnimations = false,
    this.boldText = false,
    this.reduceMotion = false,
    this.highContrast = false,
    this.onOffSwitchLabels = false,
  });

  /// An instance of [AccessibilityFeatures] where all the features are enabled.
  static const FakeAccessibilityFeatures allOn = FakeAccessibilityFeatures(
    accessibleNavigation: true,
    invertColors: true,
    disableAnimations: true,
    boldText: true,
    reduceMotion: true,
    highContrast: true,
    onOffSwitchLabels: true,
  );

  @override
  final bool accessibleNavigation;

  @override
  final bool invertColors;

  @override
  final bool disableAnimations;

  @override
  final bool boldText;

  @override
  final bool reduceMotion;

  @override
  final bool highContrast;

  @override
  final bool onOffSwitchLabels;

  @override
  bool operator ==(Object other) {
    if (other.runtimeType != runtimeType) {
      return false;
    }
    return other is FakeAccessibilityFeatures &&
        other.accessibleNavigation == accessibleNavigation &&
        other.invertColors == invertColors &&
        other.disableAnimations == disableAnimations &&
        other.boldText == boldText &&
        other.reduceMotion == reduceMotion &&
        other.highContrast == highContrast &&
        other.onOffSwitchLabels == onOffSwitchLabels;
  }

  @override
  int get hashCode {
    return Object.hash(
      accessibleNavigation,
      invertColors,
      disableAnimations,
      boldText,
      reduceMotion,
      highContrast,
      onOffSwitchLabels,
    );
  }

  /// This gives us some grace time when the dart:ui side adds something to
  /// [AccessibilityFeatures], and makes things easier when we do rolls to
  /// give us time to catch up.
  ///
  /// If you would like to add to this class, changes must first be made in the
  /// engine, followed by the framework.
  @override
  dynamic noSuchMethod(Invocation invocation) {
    return null;
  }
}

/// Used to fake insets and padding for [TestFlutterView]s.
///
/// See also:
///
///   * [TestFlutterView.padding], [TestFlutterView.systemGestureInsets],
///     [TestFlutterView.viewInsets], and [TestFlutterView.viewPadding] for test
///     properties that make use of [FakeViewPadding].
@immutable
class FakeViewPadding implements ViewPadding {
  /// Instantiates a new [FakeViewPadding] object for faking insets and padding
  /// during tests.
  const FakeViewPadding({this.left = 0.0, this.top = 0.0, this.right = 0.0, this.bottom = 0.0});

  FakeViewPadding._wrap(ViewPadding base)
    : left = base.left,
      top = base.top,
      right = base.right,
      bottom = base.bottom;

  /// A view padding that has zeros for each edge.
  static const FakeViewPadding zero = FakeViewPadding();

  @override
  final double left;

  @override
  final double top;

  @override
  final double right;

  @override
  final double bottom;
}

/// [PlatformDispatcher] that wraps another [PlatformDispatcher] and
/// allows faking of some properties for testing purposes.
///
/// See also:
///
///   * [TestFlutterView], which wraps a [FlutterView] for testing and
///     mocking purposes.
class TestPlatformDispatcher implements PlatformDispatcher {
  /// Constructs a [TestPlatformDispatcher] that defers all behavior to the given
  /// [PlatformDispatcher] unless explicitly overridden for test purposes.
  TestPlatformDispatcher({required PlatformDispatcher platformDispatcher})
    : _platformDispatcher = platformDispatcher {
    _updateViewsAndDisplays();
    _platformDispatcher.onMetricsChanged = _handleMetricsChanged;
    _platformDispatcher.onViewFocusChange = _handleViewFocusChanged;
  }

  /// The [PlatformDispatcher] that is wrapped by this [TestPlatformDispatcher].
  final PlatformDispatcher _platformDispatcher;

  @override
  TestFlutterView? get implicitView {
    return _platformDispatcher.implicitView != null
        ? _testViews[_platformDispatcher.implicitView!.viewId]!
        : null;
  }

  final Map<int, TestFlutterView> _testViews = <int, TestFlutterView>{};
  final Map<int, TestDisplay> _testDisplays = <int, TestDisplay>{};

  @override
  VoidCallback? get onMetricsChanged => _platformDispatcher.onMetricsChanged;
  VoidCallback? _onMetricsChanged;
  @override
  set onMetricsChanged(VoidCallback? callback) {
    _onMetricsChanged = callback;
  }

  void _handleMetricsChanged() {
    _updateViewsAndDisplays();
    _onMetricsChanged?.call();
  }

  @override
  ViewFocusChangeCallback? get onViewFocusChange => _platformDispatcher.onViewFocusChange;
  ViewFocusChangeCallback? _onViewFocusChange;
  @override
  set onViewFocusChange(ViewFocusChangeCallback? callback) {
    _onViewFocusChange = callback;
  }

  void _handleViewFocusChanged(ViewFocusEvent event) {
    _updateViewsAndDisplays();
    _currentlyFocusedViewId = switch (event.state) {
      ViewFocusState.focused => event.viewId,
      ViewFocusState.unfocused => null,
    };
    _onViewFocusChange?.call(event);
  }

  /// Returns the list of [ViewFocusEvent]s that have been received by
  /// [requestViewFocusChange] since the last call to
  /// [resetFocusedViewTestValues].
  ///
  /// Clearing or modifying the returned list will do nothing (it's a copy).
  /// Call [resetFocusedViewTestValues] to clear.
  List<ViewFocusEvent> get testFocusEvents => _testFocusEvents.toList();
  final List<ViewFocusEvent> _testFocusEvents = <ViewFocusEvent>[];

  /// Returns the last view ID to be focused by [onViewFocusChange].
  /// Returns null if no views are focused.
  ///
  /// Can be reset to null with [resetFocusedViewTestValues].
  int? get currentlyFocusedViewIdTestValue => _currentlyFocusedViewId;
  int? _currentlyFocusedViewId;

  /// Clears [testFocusEvents] and sets [currentlyFocusedViewIdTestValue] to
  /// null.
  void resetFocusedViewTestValues() {
    if (_currentlyFocusedViewId != null) {
      // If there is a focused view, then tell everyone who still cares that
      // it's unfocusing.
      _platformDispatcher.onViewFocusChange?.call(
        ViewFocusEvent(
          viewId: _currentlyFocusedViewId!,
          state: ViewFocusState.unfocused,
          direction: ViewFocusDirection.undefined,
        ),
      );
      _currentlyFocusedViewId = null;
    }
    _testFocusEvents.clear();
  }

  @override
  void requestViewFocusChange({
    required int viewId,
    required ViewFocusState state,
    required ViewFocusDirection direction,
  }) {
    _testFocusEvents.add(ViewFocusEvent(viewId: viewId, state: state, direction: direction));
    _platformDispatcher.requestViewFocusChange(viewId: viewId, state: state, direction: direction);
  }

  @override
  Locale get locale => _localeTestValue ?? _platformDispatcher.locale;
  Locale? _localeTestValue;

  /// Hides the real locale and reports the given [localeTestValue] instead.
  // ignore: avoid_setters_without_getters
  set localeTestValue(Locale localeTestValue) {
    _localeTestValue = localeTestValue;
    onLocaleChanged?.call();
  }

  /// Deletes any existing test locale and returns to using the real locale.
  void clearLocaleTestValue() {
    _localeTestValue = null;
    onLocaleChanged?.call();
  }

  @override
  List<Locale> get locales => _localesTestValue ?? _platformDispatcher.locales;
  List<Locale>? _localesTestValue;

  /// Hides the real locales and reports the given [localesTestValue] instead.
  // ignore: avoid_setters_without_getters
  set localesTestValue(List<Locale> localesTestValue) {
    _localesTestValue = localesTestValue;
    onLocaleChanged?.call();
  }

  /// Deletes any existing test locales and returns to using the real locales.
  void clearLocalesTestValue() {
    _localesTestValue = null;
    onLocaleChanged?.call();
  }

  @override
  VoidCallback? get onLocaleChanged => _platformDispatcher.onLocaleChanged;
  @override
  set onLocaleChanged(VoidCallback? callback) {
    _platformDispatcher.onLocaleChanged = callback;
  }

  @override
  String get initialLifecycleState => _initialLifecycleStateTestValue;
  String _initialLifecycleStateTestValue = '';

  /// Sets a faked initialLifecycleState for testing.
  // ignore: avoid_setters_without_getters
  set initialLifecycleStateTestValue(String state) {
    _initialLifecycleStateTestValue = state;
  }

  /// Resets [initialLifecycleState] to the default value for the platform.
  void resetInitialLifecycleState() {
    _initialLifecycleStateTestValue = '';
  }

  @override
  double get textScaleFactor => _textScaleFactorTestValue ?? _platformDispatcher.textScaleFactor;
  double? _textScaleFactorTestValue;

  /// Hides the real text scale factor and reports the given
  /// [textScaleFactorTestValue] instead.
  // ignore: avoid_setters_without_getters
  set textScaleFactorTestValue(double textScaleFactorTestValue) {
    _textScaleFactorTestValue = textScaleFactorTestValue;
    onTextScaleFactorChanged?.call();
  }

  /// Deletes any existing test text scale factor and returns to using the real
  /// text scale factor.
  void clearTextScaleFactorTestValue() {
    _textScaleFactorTestValue = null;
    onTextScaleFactorChanged?.call();
  }

  @override
  double scaleFontSize(double unscaledFontSize) => textScaleFactor * unscaledFontSize;

  @override
  Brightness get platformBrightness =>
      _platformBrightnessTestValue ?? _platformDispatcher.platformBrightness;
  Brightness? _platformBrightnessTestValue;
  @override
  VoidCallback? get onPlatformBrightnessChanged => _platformDispatcher.onPlatformBrightnessChanged;
  @override
  set onPlatformBrightnessChanged(VoidCallback? callback) {
    _platformDispatcher.onPlatformBrightnessChanged = callback;
  }

  /// Hides the real platform brightness and reports the given
  /// [platformBrightnessTestValue] instead.
  // ignore: avoid_setters_without_getters
  set platformBrightnessTestValue(Brightness platformBrightnessTestValue) {
    _platformBrightnessTestValue = platformBrightnessTestValue;
    onPlatformBrightnessChanged?.call();
  }

  /// Deletes any existing test platform brightness and returns to using the
  /// real platform brightness.
  void clearPlatformBrightnessTestValue() {
    _platformBrightnessTestValue = null;
    onPlatformBrightnessChanged?.call();
  }

  @override
  bool get alwaysUse24HourFormat =>
      _alwaysUse24HourFormatTestValue ?? _platformDispatcher.alwaysUse24HourFormat;
  bool? _alwaysUse24HourFormatTestValue;

  /// Hides the real clock format and reports the given
  /// [alwaysUse24HourFormatTestValue] instead.
  // ignore: avoid_setters_without_getters
  set alwaysUse24HourFormatTestValue(bool alwaysUse24HourFormatTestValue) {
    _alwaysUse24HourFormatTestValue = alwaysUse24HourFormatTestValue;
  }

  /// Deletes any existing test clock format and returns to using the real clock
  /// format.
  void clearAlwaysUse24HourTestValue() {
    _alwaysUse24HourFormatTestValue = null;
  }

  @override
  VoidCallback? get onTextScaleFactorChanged => _platformDispatcher.onTextScaleFactorChanged;
  @override
  set onTextScaleFactorChanged(VoidCallback? callback) {
    _platformDispatcher.onTextScaleFactorChanged = callback;
  }

  @override
  bool get nativeSpellCheckServiceDefined =>
      _nativeSpellCheckServiceDefinedTestValue ??
      _platformDispatcher.nativeSpellCheckServiceDefined;
  bool? _nativeSpellCheckServiceDefinedTestValue;
  // ignore: avoid_setters_without_getters
  set nativeSpellCheckServiceDefinedTestValue(bool nativeSpellCheckServiceDefinedTestValue) {
    _nativeSpellCheckServiceDefinedTestValue = nativeSpellCheckServiceDefinedTestValue;
  }

  /// Deletes existing value that determines whether or not a native spell check
  /// service is defined and returns to the real value.
  void clearNativeSpellCheckServiceDefined() {
    _nativeSpellCheckServiceDefinedTestValue = null;
  }

  @override
  bool get supportsShowingSystemContextMenu =>
      _supportsShowingSystemContextMenu ?? _platformDispatcher.supportsShowingSystemContextMenu;
  bool? _supportsShowingSystemContextMenu;
  set supportsShowingSystemContextMenu(bool value) {
    _supportsShowingSystemContextMenu = value;
  }

  /// Resets [supportsShowingSystemContextMenu] to the default value.
  void resetSupportsShowingSystemContextMenu() {
    _supportsShowingSystemContextMenu = null;
  }

  @override
  bool get brieflyShowPassword =>
      _brieflyShowPasswordTestValue ?? _platformDispatcher.brieflyShowPassword;
  bool? _brieflyShowPasswordTestValue;

  /// Hides the real [brieflyShowPassword] and reports the given
  /// `brieflyShowPasswordTestValue` instead.
  // ignore: avoid_setters_without_getters
  set brieflyShowPasswordTestValue(bool brieflyShowPasswordTestValue) {
    _brieflyShowPasswordTestValue = brieflyShowPasswordTestValue;
  }

  /// Resets [brieflyShowPassword] to the default value for the platform.
  void resetBrieflyShowPassword() {
    _brieflyShowPasswordTestValue = null;
  }

  @override
  FrameCallback? get onBeginFrame => _platformDispatcher.onBeginFrame;
  @override
  set onBeginFrame(FrameCallback? callback) {
    _platformDispatcher.onBeginFrame = callback;
  }

  @override
  VoidCallback? get onDrawFrame => _platformDispatcher.onDrawFrame;
  @override
  set onDrawFrame(VoidCallback? callback) {
    _platformDispatcher.onDrawFrame = callback;
  }

  @override
  TimingsCallback? get onReportTimings => _platformDispatcher.onReportTimings;
  @override
  set onReportTimings(TimingsCallback? callback) {
    _platformDispatcher.onReportTimings = callback;
  }

  @override
  PointerDataPacketCallback? get onPointerDataPacket => _platformDispatcher.onPointerDataPacket;
  @override
  set onPointerDataPacket(PointerDataPacketCallback? callback) {
    _platformDispatcher.onPointerDataPacket = callback;
  }

  @override
  String get defaultRouteName => _defaultRouteNameTestValue ?? _platformDispatcher.defaultRouteName;
  String? _defaultRouteNameTestValue;

  /// Hides the real default route name and reports the given
  /// [defaultRouteNameTestValue] instead.
  // ignore: avoid_setters_without_getters
  set defaultRouteNameTestValue(String defaultRouteNameTestValue) {
    _defaultRouteNameTestValue = defaultRouteNameTestValue;
  }

  /// Deletes any existing test default route name and returns to using the real
  /// default route name.
  void clearDefaultRouteNameTestValue() {
    _defaultRouteNameTestValue = null;
  }

  @override
  void scheduleFrame() {
    _platformDispatcher.scheduleFrame();
  }

  @override
  bool get semanticsEnabled => _semanticsEnabledTestValue ?? _platformDispatcher.semanticsEnabled;
  bool? _semanticsEnabledTestValue;

  /// Hides the real semantics enabled and reports the given
  /// [semanticsEnabledTestValue] instead.
  // ignore: avoid_setters_without_getters
  set semanticsEnabledTestValue(bool semanticsEnabledTestValue) {
    _semanticsEnabledTestValue = semanticsEnabledTestValue;
    onSemanticsEnabledChanged?.call();
  }

  /// Deletes any existing test semantics enabled and returns to using the real
  /// semantics enabled.
  void clearSemanticsEnabledTestValue() {
    _semanticsEnabledTestValue = null;
    onSemanticsEnabledChanged?.call();
  }

  @override
  VoidCallback? get onSemanticsEnabledChanged => _platformDispatcher.onSemanticsEnabledChanged;
  @override
  set onSemanticsEnabledChanged(VoidCallback? callback) {
    _platformDispatcher.onSemanticsEnabledChanged = callback;
  }

  @override
  SemanticsActionEventCallback? get onSemanticsActionEvent =>
      _platformDispatcher.onSemanticsActionEvent;
  @override
  set onSemanticsActionEvent(SemanticsActionEventCallback? callback) {
    _platformDispatcher.onSemanticsActionEvent = callback;
  }

  @override
  AccessibilityFeatures get accessibilityFeatures =>
      _accessibilityFeaturesTestValue ?? _platformDispatcher.accessibilityFeatures;
  AccessibilityFeatures? _accessibilityFeaturesTestValue;

  /// Hides the real accessibility features and reports the given
  /// [accessibilityFeaturesTestValue] instead.
  ///
  /// Consider using [FakeAccessibilityFeatures] to provide specific
  /// values for the various accessibility features under test.
  // ignore: avoid_setters_without_getters
  set accessibilityFeaturesTestValue(AccessibilityFeatures accessibilityFeaturesTestValue) {
    _accessibilityFeaturesTestValue = accessibilityFeaturesTestValue;
    onAccessibilityFeaturesChanged?.call();
  }

  /// Deletes any existing test accessibility features and returns to using the
  /// real accessibility features.
  void clearAccessibilityFeaturesTestValue() {
    _accessibilityFeaturesTestValue = null;
    onAccessibilityFeaturesChanged?.call();
  }

  @override
  VoidCallback? get onAccessibilityFeaturesChanged =>
      _platformDispatcher.onAccessibilityFeaturesChanged;
  @override
  set onAccessibilityFeaturesChanged(VoidCallback? callback) {
    _platformDispatcher.onAccessibilityFeaturesChanged = callback;
  }

  @override
  void setIsolateDebugName(String name) {
    _platformDispatcher.setIsolateDebugName(name);
  }

  @override
  void sendPlatformMessage(String name, ByteData? data, PlatformMessageResponseCallback? callback) {
    _platformDispatcher.sendPlatformMessage(name, data, callback);
  }

  /// Delete any test value properties that have been set on this [TestPlatformDispatcher]
  /// and return to reporting the real [PlatformDispatcher] values for all
  /// [PlatformDispatcher] properties.
  ///
  /// If desired, clearing of properties can be done on an individual basis,
  /// e.g., [clearLocaleTestValue].
  void clearAllTestValues() {
    clearAccessibilityFeaturesTestValue();
    clearAlwaysUse24HourTestValue();
    clearDefaultRouteNameTestValue();
    clearPlatformBrightnessTestValue();
    clearLocaleTestValue();
    clearLocalesTestValue();
    clearSemanticsEnabledTestValue();
    clearTextScaleFactorTestValue();
    clearNativeSpellCheckServiceDefined();
    resetBrieflyShowPassword();
    resetSupportsShowingSystemContextMenu();
    resetInitialLifecycleState();
    resetSystemFontFamily();
  }

  @override
  VoidCallback? get onFrameDataChanged => _platformDispatcher.onFrameDataChanged;
  @override
  set onFrameDataChanged(VoidCallback? value) {
    _platformDispatcher.onFrameDataChanged = value;
  }

  @override
  KeyDataCallback? get onKeyData => _platformDispatcher.onKeyData;

  @override
  set onKeyData(KeyDataCallback? onKeyData) {
    _platformDispatcher.onKeyData = onKeyData;
  }

  @override
  VoidCallback? get onPlatformConfigurationChanged =>
      _platformDispatcher.onPlatformConfigurationChanged;

  @override
  set onPlatformConfigurationChanged(VoidCallback? onPlatformConfigurationChanged) {
    _platformDispatcher.onPlatformConfigurationChanged = onPlatformConfigurationChanged;
  }

  @override
  Locale? computePlatformResolvedLocale(List<Locale> supportedLocales) =>
      _platformDispatcher.computePlatformResolvedLocale(supportedLocales);

  @override
  ByteData? getPersistentIsolateData() => _platformDispatcher.getPersistentIsolateData();

  @override
  Iterable<TestFlutterView> get views => _testViews.values;

  @override
  FlutterView? view({required int id}) => _testViews[id];

  @override
  Iterable<TestDisplay> get displays => _testDisplays.values;

  void _updateViewsAndDisplays() {
    final List<Object> extraDisplayKeys = <Object>[..._testDisplays.keys];
    for (final Display display in _platformDispatcher.displays) {
      extraDisplayKeys.remove(display.id);
      if (!_testDisplays.containsKey(display.id)) {
        _testDisplays[display.id] = TestDisplay(this, display);
      }
    }
    extraDisplayKeys.forEach(_testDisplays.remove);

    final List<Object> extraViewKeys = <Object>[..._testViews.keys];
    for (final FlutterView view in _platformDispatcher.views) {
      // TODO(pdblasi-google): Remove this try-catch once the Display API is stable and supported on all platforms
      late final TestDisplay display;
      try {
        final Display realDisplay = view.display;
        if (_testDisplays.containsKey(realDisplay.id)) {
          display = _testDisplays[view.display.id]!;
        } else {
          display = _UnsupportedDisplay(
            this,
            view,
            'PlatformDispatcher did not contain a Display with id ${realDisplay.id}, '
            'which was expected by FlutterView ($view)',
          );
        }
      } catch (error) {
        display = _UnsupportedDisplay(this, view, error);
      }

      extraViewKeys.remove(view.viewId);
      if (!_testViews.containsKey(view.viewId)) {
        _testViews[view.viewId] = TestFlutterView(
          view: view,
          platformDispatcher: this,
          display: display,
        );
      }
    }

    extraViewKeys.forEach(_testViews.remove);
  }

  @override
  ErrorCallback? get onError => _platformDispatcher.onError;
  @override
  set onError(ErrorCallback? value) {
    _platformDispatcher.onError;
  }

  @override
  VoidCallback? get onSystemFontFamilyChanged => _platformDispatcher.onSystemFontFamilyChanged;
  @override
  set onSystemFontFamilyChanged(VoidCallback? value) {
    _platformDispatcher.onSystemFontFamilyChanged = value;
  }

  @override
  FrameData get frameData => _platformDispatcher.frameData;

  @override
  void registerBackgroundIsolate(RootIsolateToken token) {
    _platformDispatcher.registerBackgroundIsolate(token);
  }

  @override
  void requestDartPerformanceMode(DartPerformanceMode mode) {
    _platformDispatcher.requestDartPerformanceMode(mode);
  }

  /// The system font family to use for this test.
  ///
  /// Defaults to the value provided by [PlatformDispatcher.systemFontFamily].
  /// This can only be set in a test environment to emulate different platform
  /// configurations. A standard [PlatformDispatcher] is not mutable from the
  /// framework.
  ///
  /// Setting this value to `null` will force [systemFontFamily] to return
  /// `null`. If you want to have the value default to the platform
  /// [systemFontFamily], use [resetSystemFontFamily].
  ///
  /// See also:
  ///
  ///   * [PlatformDispatcher.systemFontFamily] for the standard implementation
  ///   * [resetSystemFontFamily] to reset this value specifically
  ///   * [clearAllTestValues] to reset all test values for this view
  @override
  String? get systemFontFamily {
    return _forceSystemFontFamilyToBeNull
        ? null
        : _systemFontFamily ?? _platformDispatcher.systemFontFamily;
  }

  String? _systemFontFamily;
  bool _forceSystemFontFamilyToBeNull = false;
  set systemFontFamily(String? value) {
    _systemFontFamily = value;
    if (value == null) {
      _forceSystemFontFamilyToBeNull = true;
    }
    onSystemFontFamilyChanged?.call();
  }

  /// Resets [systemFontFamily] to the default for the platform.
  void resetSystemFontFamily() {
    _systemFontFamily = null;
    _forceSystemFontFamilyToBeNull = false;
    onSystemFontFamilyChanged?.call();
  }

  @override
  void updateSemantics(SemanticsUpdate update) {
    _platformDispatcher.updateSemantics(update);
  }

  /// This gives us some grace time when the dart:ui side adds something to
  /// [PlatformDispatcher], and makes things easier when we do rolls to give
  /// us time to catch up.
  @override
  dynamic noSuchMethod(Invocation invocation) {
    return null;
  }
}

/// A [FlutterView] that wraps another [FlutterView] and allows faking of
/// some properties for testing purposes.
///
/// This class should not be instantiated manually, as
/// it requires a backing [FlutterView] that must be produced from
/// a [PlatformDispatcher].
///
/// See also:
///
///   * [WidgetTester.view] which allows for accessing the [TestFlutterView]
///     for single view applications or widget testing.
///   * [WidgetTester.viewOf] which allows for accessing the appropriate
///     [TestFlutterView] in a given situation for multi-view applications.
///   * [TestPlatformDispatcher], which allows for faking of platform specific
///     functionality.
class TestFlutterView implements FlutterView {
  /// Constructs a [TestFlutterView] that defers all behavior to the given
  /// [FlutterView] unless explicitly overridden for testing.
  TestFlutterView({
    required FlutterView view,
    required TestPlatformDispatcher platformDispatcher,
    required TestDisplay display,
  }) : _view = view,
       _platformDispatcher = platformDispatcher,
       _display = display;

  /// The [FlutterView] backing this [TestFlutterView].
  final FlutterView _view;

  @override
  TestPlatformDispatcher get platformDispatcher => _platformDispatcher;
  final TestPlatformDispatcher _platformDispatcher;

  @override
  TestDisplay get display => _display;
  final TestDisplay _display;

  @override
  int get viewId => _view.viewId;

  /// The device pixel ratio to use for this test.
  ///
  /// Defaults to the value provided by [FlutterView.devicePixelRatio]. This
  /// can only be set in a test environment to emulate different view
  /// configurations. A standard [FlutterView] is not mutable from the framework.
  ///
  /// See also:
  ///
  ///   * [FlutterView.devicePixelRatio] for the standard implementation
  ///   * [TestDisplay.devicePixelRatio] which will stay in sync with this value
  ///   * [resetDevicePixelRatio] to reset this value specifically
  ///   * [reset] to reset all test values for this view
  @override
  double get devicePixelRatio => _display._devicePixelRatio ?? _view.devicePixelRatio;
  set devicePixelRatio(double value) {
    _display.devicePixelRatio = value;
  }

  /// Resets [devicePixelRatio] for this test view to the default value for this view.
  ///
  /// This will also reset the [devicePixelRatio] for the [TestDisplay]
  /// that is related to this view.
  void resetDevicePixelRatio() {
    _display.resetDevicePixelRatio();
  }

  /// The display features to use for this test.
  ///
  /// Defaults to the value provided by [FlutterView.displayFeatures]. This
  /// can only be set in a test environment to emulate different view
  /// configurations. A standard [FlutterView] is not mutable from the framework.
  ///
  /// See also:
  ///
  ///   * [FlutterView.displayFeatures] for the standard implementation
  ///   * [resetDisplayFeatures] to reset this value specifically
  ///   * [reset] to reset all test values for this view
  @override
  List<DisplayFeature> get displayFeatures => _displayFeatures ?? _view.displayFeatures;
  List<DisplayFeature>? _displayFeatures;
  set displayFeatures(List<DisplayFeature> value) {
    _displayFeatures = value;
    platformDispatcher.onMetricsChanged?.call();
  }

  /// Resets [displayFeatures] to the default values for this view.
  void resetDisplayFeatures() {
    _displayFeatures = null;
    platformDispatcher.onMetricsChanged?.call();
  }

  /// The padding to use for this test.
  ///
  /// Defaults to the value provided by [FlutterView.padding]. This
  /// can only be set in a test environment to emulate different view
  /// configurations. A standard [FlutterView] is not mutable from the framework.
  ///
  /// See also:
  ///
  ///   * [FakeViewPadding] which is used to set this value for testing
  ///   * [FlutterView.padding] for the standard implementation.
  ///   * [resetPadding] to reset this value specifically.
  ///   * [reset] to reset all test values for this view.
  @override
  FakeViewPadding get padding => _padding ?? FakeViewPadding._wrap(_view.padding);
  FakeViewPadding? _padding;
  set padding(FakeViewPadding value) {
    _padding = value;
    platformDispatcher.onMetricsChanged?.call();
  }

  /// Resets [padding] to the default value for this view.
  void resetPadding() {
    _padding = null;
    platformDispatcher.onMetricsChanged?.call();
  }

  /// The physical size to use for this test.
  ///
  /// Defaults to the value provided by [FlutterView.physicalSize]. This
  /// can only be set in a test environment to emulate different view
  /// configurations. A standard [FlutterView] is not mutable from the framework.
  ///
  /// Setting this value also sets [physicalConstraints] to tight constraints
  /// based on the given size.
  ///
  /// See also:
  ///
  ///   * [FlutterView.physicalSize] for the standard implementation
  ///   * [resetPhysicalSize] to reset this value specifically
  ///   * [reset] to reset all test values for this view
  @override
  Size get physicalSize => _physicalSize ?? _view.physicalSize;
  Size? _physicalSize;
  set physicalSize(Size value) {
    _physicalSize = value;
    // For backwards compatibility the constraints are set based on the provided size.
    physicalConstraints = ViewConstraints.tight(value);
  }

  /// Resets [physicalSize] (and implicitly also the [physicalConstraints]) to
  /// the default value for this view.
  void resetPhysicalSize() {
    _physicalSize = null;
    resetPhysicalConstraints();
  }

  /// The physical constraints to use for this test.
  ///
  /// Defaults to the value provided by [FlutterView.physicalConstraints]. This
  /// can only be set in a test environment to emulate different view
  /// configurations. A standard [FlutterView] is not mutable from the framework.
  ///
  /// See also:
  ///
  ///   * [FlutterView.physicalConstraints] for the standard implementation
  ///   * [physicalConstraints] to reset this value specifically
  ///   * [reset] to reset all test values for this view
  @override
  ViewConstraints get physicalConstraints => _physicalConstraints ?? _view.physicalConstraints;
  ViewConstraints? _physicalConstraints;
  set physicalConstraints(ViewConstraints value) {
    _physicalConstraints = value;
    platformDispatcher.onMetricsChanged?.call();
  }

  /// Resets [physicalConstraints] to the default value for this view.
  void resetPhysicalConstraints() {
    _physicalConstraints = null;
    platformDispatcher.onMetricsChanged?.call();
  }

  /// The system gesture insets to use for this test.
  ///
  /// Defaults to the value provided by [FlutterView.systemGestureInsets].
  /// This can only be set in a test environment to emulate different view
  /// configurations. A standard [FlutterView] is not mutable from the framework.
  ///
  /// See also:
  ///
  ///   * [FakeViewPadding] which is used to set this value for testing
  ///   * [FlutterView.systemGestureInsets] for the standard implementation
  ///   * [resetSystemGestureInsets] to reset this value specifically
  ///   * [reset] to reset all test values for this view
  @override
  FakeViewPadding get systemGestureInsets =>
      _systemGestureInsets ?? FakeViewPadding._wrap(_view.systemGestureInsets);
  FakeViewPadding? _systemGestureInsets;
  set systemGestureInsets(FakeViewPadding value) {
    _systemGestureInsets = value;
    platformDispatcher.onMetricsChanged?.call();
  }

  /// Resets [systemGestureInsets] to the default value for this view.
  void resetSystemGestureInsets() {
    _systemGestureInsets = null;
    platformDispatcher.onMetricsChanged?.call();
  }

  /// The view insets to use for this test.
  ///
  /// Defaults to the value provided by [FlutterView.viewInsets]. This
  /// can only be set in a test environment to emulate different view
  /// configurations. A standard [FlutterView] is not mutable from the framework.
  ///
  /// See also:
  ///
  ///   * [FakeViewPadding] which is used to set this value for testing
  ///   * [FlutterView.viewInsets] for the standard implementation
  ///   * [resetViewInsets] to reset this value specifically
  ///   * [reset] to reset all test values for this view
  @override
  FakeViewPadding get viewInsets => _viewInsets ?? FakeViewPadding._wrap(_view.viewInsets);
  FakeViewPadding? _viewInsets;
  set viewInsets(FakeViewPadding value) {
    _viewInsets = value;
    platformDispatcher.onMetricsChanged?.call();
  }

  /// Resets [viewInsets] to the default value for this view.
  void resetViewInsets() {
    _viewInsets = null;
    platformDispatcher.onMetricsChanged?.call();
  }

  /// The view padding to use for this test.
  ///
  /// Defaults to the value provided by [FlutterView.viewPadding]. This
  /// can only be set in a test environment to emulate different view
  /// configurations. A standard [FlutterView] is not mutable from the framework.
  ///
  /// See also:
  ///
  ///   * [FakeViewPadding] which is used to set this value for testing
  ///   * [FlutterView.viewPadding] for the standard implementation
  ///   * [resetViewPadding] to reset this value specifically
  ///   * [reset] to reset all test values for this view
  @override
  FakeViewPadding get viewPadding => _viewPadding ?? FakeViewPadding._wrap(_view.viewPadding);
  FakeViewPadding? _viewPadding;
  set viewPadding(FakeViewPadding value) {
    _viewPadding = value;
    platformDispatcher.onMetricsChanged?.call();
  }

  /// Resets [viewPadding] to the default value for this view.
  void resetViewPadding() {
    _viewPadding = null;
    platformDispatcher.onMetricsChanged?.call();
  }

  /// The gesture settings to use for this test.
  ///
  /// Defaults to the value provided by [FlutterView.gestureSettings]. This
  /// can only be set in a test environment to emulate different view
  /// configurations. A standard [FlutterView] is not mutable from the framework.
  ///
  /// See also:
  ///
  ///   * [FlutterView.gestureSettings] for the standard implementation
  ///   * [resetGestureSettings] to reset this value specifically
  ///   * [reset] to reset all test values for this view
  @override
  GestureSettings get gestureSettings => _gestureSettings ?? _view.gestureSettings;
  GestureSettings? _gestureSettings;
  set gestureSettings(GestureSettings value) {
    _gestureSettings = value;
    platformDispatcher.onMetricsChanged?.call();
  }

  /// Resets [gestureSettings] to the default value for this view.
  void resetGestureSettings() {
    _gestureSettings = null;
    platformDispatcher.onMetricsChanged?.call();
  }

  @override
  void render(Scene scene, {Size? size}) {
    _view.render(scene, size: size);
  }

  @override
  void updateSemantics(SemanticsUpdate update) {
    _view.updateSemantics(update);
  }

  /// Resets all test values to the defaults for this view.
  ///
  /// See also:
  ///
  ///   * [resetDevicePixelRatio] to reset [devicePixelRatio] specifically
  ///   * [resetDisplayFeatures]  to reset [displayFeatures] specifically
  ///   * [resetPadding] to reset [padding] specifically
  ///   * [resetPhysicalSize] to reset [physicalSize] specifically
  ///   * [resetSystemGestureInsets] to reset [systemGestureInsets] specifically
  ///   * [resetViewInsets] to reset [viewInsets] specifically
  ///   * [resetViewPadding] to reset [viewPadding] specifically
  ///   * [resetGestureSettings] to reset [gestureSettings] specifically
  void reset() {
    resetDevicePixelRatio();
    resetDisplayFeatures();
    resetPadding();
    resetPhysicalSize();
    // resetPhysicalConstraints is implicitly called by resetPhysicalSize.
    resetSystemGestureInsets();
    resetViewInsets();
    resetViewPadding();
    resetGestureSettings();
  }

  /// This gives us some grace time when the dart:ui side adds something to
  /// [FlutterView], and makes things easier when we do rolls to give
  /// us time to catch up.
  @override
  dynamic noSuchMethod(Invocation invocation) {
    return null;
  }
}

/// A version of [Display] that can be modified to allow for testing various
/// use cases.
///
/// Updates to the [TestDisplay] will be surfaced through
/// [PlatformDispatcher.onMetricsChanged].
class TestDisplay implements Display {
  /// Creates a new [TestDisplay] backed by the given [Display].
  TestDisplay(TestPlatformDispatcher platformDispatcher, Display display)
    : _platformDispatcher = platformDispatcher,
      _display = display;

  final Display _display;
  final TestPlatformDispatcher _platformDispatcher;

  @override
  int get id => _display.id;

  /// The device pixel ratio to use for this test.
  ///
  /// Defaults to the value provided by [Display.devicePixelRatio]. This
  /// can only be set in a test environment to emulate different display
  /// configurations. A standard [Display] is not mutable from the framework.
  ///
  /// See also:
  ///
  ///   * [Display.devicePixelRatio] for the standard implementation
  ///   * [TestFlutterView.devicePixelRatio] which will stay in sync with this value
  ///   * [resetDevicePixelRatio] to reset this value specifically
  ///   * [reset] to reset all test values for this display
  @override
  double get devicePixelRatio => _devicePixelRatio ?? _display.devicePixelRatio;
  double? _devicePixelRatio;
  set devicePixelRatio(double value) {
    _devicePixelRatio = value;
    _platformDispatcher.onMetricsChanged?.call();
  }

  /// Resets [devicePixelRatio] to the default value for this display.
  ///
  /// This will also reset the [devicePixelRatio] for any [TestFlutterView]s
  /// that are related to this display.
  void resetDevicePixelRatio() {
    _devicePixelRatio = null;
    _platformDispatcher.onMetricsChanged?.call();
  }

  /// The refresh rate to use for this test.
  ///
  /// Defaults to the value provided by [Display.refreshRate]. This
  /// can only be set in a test environment to emulate different display
  /// configurations. A standard [Display] is not mutable from the framework.
  ///
  /// See also:
  ///
  ///   * [Display.refreshRate] for the standard implementation
  ///   * [resetRefreshRate] to reset this value specifically
  ///   * [reset] to reset all test values for this display
  @override
  double get refreshRate => _refreshRate ?? _display.refreshRate;
  double? _refreshRate;
  set refreshRate(double value) {
    _refreshRate = value;
    _platformDispatcher.onMetricsChanged?.call();
  }

  /// Resets [refreshRate] to the default value for this display.
  void resetRefreshRate() {
    _refreshRate = null;
    _platformDispatcher.onMetricsChanged?.call();
  }

  /// The size of the [Display] to use for this test.
  ///
  /// Defaults to the value provided by [Display.refreshRate]. This
  /// can only be set in a test environment to emulate different display
  /// configurations. A standard [Display] is not mutable from the framework.
  ///
  /// See also:
  ///
  ///   * [Display.refreshRate] for the standard implementation
  ///   * [resetRefreshRate] to reset this value specifically
  ///   * [reset] to reset all test values for this display
  @override
  Size get size => _size ?? _display.size;
  Size? _size;
  set size(Size value) {
    _size = value;
    _platformDispatcher.onMetricsChanged?.call();
  }

  /// Resets [size] to the default value for this display.
  void resetSize() {
    _size = null;
    _platformDispatcher.onMetricsChanged?.call();
  }

  /// Resets all values on this [TestDisplay].
  ///
  /// See also:
  ///   * [resetDevicePixelRatio] to reset [devicePixelRatio] specifically
  ///   * [resetRefreshRate] to reset [refreshRate] specifically
  ///   * [resetSize] to reset [size] specifically
  void reset() {
    resetDevicePixelRatio();
    resetRefreshRate();
    resetSize();
  }

  /// This gives us some grace time when the dart:ui side adds something to
  /// [Display], and makes things easier when we do rolls to give
  /// us time to catch up.
  @override
  dynamic noSuchMethod(Invocation invocation) {
    return null;
  }
}

// TODO(pdblasi-google): Remove this once the Display API is stable and supported on all platforms
class _UnsupportedDisplay implements TestDisplay {
  _UnsupportedDisplay(this._platformDispatcher, this._view, this.error);

  final FlutterView _view;
  final Object? error;

  @override
  final TestPlatformDispatcher _platformDispatcher;

  @override
  double get devicePixelRatio => _devicePixelRatio ?? _view.devicePixelRatio;
  @override
  double? _devicePixelRatio;
  @override
  set devicePixelRatio(double value) {
    _devicePixelRatio = value;
    _platformDispatcher.onMetricsChanged?.call();
  }

  @override
  void resetDevicePixelRatio() {
    _devicePixelRatio = null;
    _platformDispatcher.onMetricsChanged?.call();
  }

  @override
  dynamic noSuchMethod(Invocation invocation) {
    throw UnsupportedError(
      'The Display API is unsupported in this context. '
      'As of the last metrics change on PlatformDispatcher, this was the error '
      'given when trying to prepare the display for testing: $error',
    );
  }
}

/// Deprecated. Will be removed in a future version of Flutter.
///
/// This class has been deprecated to prepare for Flutter's upcoming support
/// for multiple views and multiple windows.
///
/// [SingletonFlutterWindow] that wraps another [SingletonFlutterWindow] and
/// allows faking of some properties for testing purposes.
///
/// Tests for certain widgets, e.g., [MaterialApp], might require faking certain
/// properties of a [SingletonFlutterWindow]. [TestWindow] facilitates the
/// faking of these properties by overriding the properties of a real
/// [SingletonFlutterWindow] with desired fake values. The binding used within
/// tests, [TestWidgetsFlutterBinding], contains a [TestWindow] that is used by
/// all tests.
///
/// ## Sample Code
///
/// A test can utilize a [TestWindow] in the following way:
///
/// ```dart
/// testWidgets('your test name here', (WidgetTester tester) async {
///   // Retrieve the TestWidgetsFlutterBinding.
///   final TestWidgetsFlutterBinding testBinding = tester.binding;
///
///   // Fake the desired properties of the TestWindow. All code running
///   // within this test will perceive the following fake text scale
///   // factor as the real text scale factor of the window.
///   testBinding.platformDispatcher.textScaleFactorTestValue = 2.5;
///
///   // Test code that depends on text scale factor here.
/// });
/// ```
///
/// The [TestWidgetsFlutterBinding] is recreated for each test and
/// therefore any fake values defined in one test will not persist
/// to the next.
///
/// If a test needs to override a real [SingletonFlutterWindow] property and
/// then later return to using the real [SingletonFlutterWindow] property,
/// [TestWindow] provides methods to clear each individual test value, e.g.,
/// [clearDevicePixelRatioTestValue].
///
/// To clear all fake test values in a [TestWindow], consider using
/// [clearAllTestValues].
///
/// See also:
///
///   * [TestPlatformDispatcher], which wraps a [PlatformDispatcher] for
///     testing purposes and is used by the [platformDispatcher] property of
///     this class.
@Deprecated(
  'Use TestPlatformDispatcher (via WidgetTester.platformDispatcher) or TestFlutterView (via WidgetTester.view) instead. '
  'Deprecated to prepare for the upcoming multi-window support. '
  'This feature was deprecated after v3.9.0-0.1.pre.',
)
class TestWindow implements SingletonFlutterWindow {
  /// Constructs a [TestWindow] that defers all behavior to the given
  /// [SingletonFlutterWindow] unless explicitly overridden for test purposes.
  @Deprecated(
    'Use TestPlatformDispatcher (via WidgetTester.platformDispatcher) or TestFlutterView (via WidgetTester.view) instead. '
    'Deprecated to prepare for the upcoming multi-window support. '
    'This feature was deprecated after v3.9.0-0.1.pre.',
  )
  TestWindow({required SingletonFlutterWindow window})
    : platformDispatcher = TestPlatformDispatcher(platformDispatcher: window.platformDispatcher);

  /// Constructs a [TestWindow] that defers all behavior to the given
  /// [TestPlatformDispatcher] and its [TestPlatformDispatcher.implicitView].
  ///
  /// This class will not work when multiple views are present. If multiple view
  /// support is needed use [WidgetTester.platformDispatcher] and
  /// [WidgetTester.viewOf].
  ///
  /// See also:
  ///
  ///   * [TestPlatformDispatcher] which allows faking of platform-wide values for
  ///     testing purposes.
  ///   * [TestFlutterView] which allows faking of view-specific values for
  ///     testing purposes.
  @Deprecated(
    'Use TestPlatformDispatcher (via WidgetTester.platformDispatcher) or TestFlutterView (via WidgetTester.view) instead. '
    'Deprecated to prepare for the upcoming multi-window support. '
    'This feature was deprecated after v3.9.0-0.1.pre.',
  )
  TestWindow.fromPlatformDispatcher({
    @Deprecated(
      'Use WidgetTester.platformDispatcher instead. '
      'Deprecated to prepare for the upcoming multi-window support. '
      'This feature was deprecated after v3.9.0-0.1.pre.',
    )
    required this.platformDispatcher,
  });

  @Deprecated(
    'Use WidgetTester.platformDispatcher instead. '
    'Deprecated to prepare for the upcoming multi-window support. '
    'This feature was deprecated after v3.9.0-0.1.pre.',
  )
  @override
  final TestPlatformDispatcher platformDispatcher;

  TestFlutterView get _view => platformDispatcher.implicitView!;

  @Deprecated(
    'Use WidgetTester.view.devicePixelRatio instead. '
    'Deprecated to prepare for the upcoming multi-window support. '
    'This feature was deprecated after v3.9.0-0.1.pre.',
  )
  @override
  double get devicePixelRatio => _view.devicePixelRatio;

  /// Hides the real device pixel ratio and reports the given [devicePixelRatio]
  /// instead.
  @Deprecated(
    'Use WidgetTester.view.devicePixelRatio instead. '
    'Deprecated to prepare for the upcoming multi-window support. '
    'This feature was deprecated after v3.9.0-0.1.pre.',
  )
  // ignore: avoid_setters_without_getters
  set devicePixelRatioTestValue(double devicePixelRatio) {
    _view.devicePixelRatio = devicePixelRatio;
  }

  /// Deletes any existing test device pixel ratio and returns to using the real
  /// device pixel ratio.
  @Deprecated(
    'Use WidgetTester.view.resetDevicePixelRatio() instead. '
    'Deprecated to prepare for the upcoming multi-window support. '
    'This feature was deprecated after v3.9.0-0.1.pre.',
  )
  void clearDevicePixelRatioTestValue() {
    _view.resetDevicePixelRatio();
  }

  @Deprecated(
    'Use WidgetTester.view.physicalSize instead. '
    'Deprecated to prepare for the upcoming multi-window support. '
    'This feature was deprecated after v3.9.0-0.1.pre.',
  )
  @override
  Size get physicalSize => _view.physicalSize;

  /// Hides the real physical size and reports the given [physicalSizeTestValue]
  /// instead.
  @Deprecated(
    'Use WidgetTester.view.physicalSize instead. '
    'Deprecated to prepare for the upcoming multi-window support. '
    'This feature was deprecated after v3.9.0-0.1.pre.',
  )
  // ignore: avoid_setters_without_getters
  set physicalSizeTestValue(Size physicalSizeTestValue) {
    _view.physicalSize = physicalSizeTestValue;
  }

  /// Deletes any existing test physical size and returns to using the real
  /// physical size.
  @Deprecated(
    'Use WidgetTester.view.resetPhysicalSize() instead. '
    'Deprecated to prepare for the upcoming multi-window support. '
    'This feature was deprecated after v3.9.0-0.1.pre.',
  )
  void clearPhysicalSizeTestValue() {
    _view.resetPhysicalSize();
  }

  @Deprecated(
    'Use WidgetTester.view.viewInsets instead. '
    'Deprecated to prepare for the upcoming multi-window support. '
    'This feature was deprecated after v3.9.0-0.1.pre.',
  )
  @override
  ViewPadding get viewInsets => _view.viewInsets;

  /// Hides the real view insets and reports the given [viewInsetsTestValue]
  /// instead.
  ///
  /// Use [FakeViewPadding] to set this value for testing.
  @Deprecated(
    'Use WidgetTester.view.viewInsets instead. '
    'Deprecated to prepare for the upcoming multi-window support. '
    'This feature was deprecated after v3.9.0-0.1.pre.',
  )
  // ignore: avoid_setters_without_getters
  set viewInsetsTestValue(ViewPadding value) {
    _view.viewInsets = value is FakeViewPadding ? value : FakeViewPadding._wrap(value);
  }

  /// Deletes any existing test view insets and returns to using the real view
  /// insets.
  @Deprecated(
    'Use WidgetTester.view.resetViewInsets() instead. '
    'Deprecated to prepare for the upcoming multi-window support. '
    'This feature was deprecated after v3.9.0-0.1.pre.',
  )
  void clearViewInsetsTestValue() {
    _view.resetViewInsets();
  }

  @Deprecated(
    'Use WidgetTester.view.viewPadding instead. '
    'Deprecated to prepare for the upcoming multi-window support. '
    'This feature was deprecated after v3.9.0-0.1.pre.',
  )
  @override
  ViewPadding get viewPadding => _view.viewPadding;

  /// Hides the real view padding and reports the given [paddingTestValue]
  /// instead.
  ///
  /// Use [FakeViewPadding] to set this value for testing.
  @Deprecated(
    'Use WidgetTester.view.viewPadding instead. '
    'Deprecated to prepare for the upcoming multi-window support. '
    'This feature was deprecated after v3.9.0-0.1.pre.',
  )
  // ignore: avoid_setters_without_getters
  set viewPaddingTestValue(ViewPadding value) {
    _view.viewPadding = value is FakeViewPadding ? value : FakeViewPadding._wrap(value);
  }

  /// Deletes any existing test view padding and returns to using the real
  /// viewPadding.
  @Deprecated(
    'Use WidgetTester.view.resetViewPadding() instead. '
    'Deprecated to prepare for the upcoming multi-window support. '
    'This feature was deprecated after v3.9.0-0.1.pre.',
  )
  void clearViewPaddingTestValue() {
    _view.resetViewPadding();
  }

  @Deprecated(
    'Use WidgetTester.view.padding instead. '
    'Deprecated to prepare for the upcoming multi-window support. '
    'This feature was deprecated after v3.9.0-0.1.pre.',
  )
  @override
  ViewPadding get padding => _view.padding;

  /// Hides the real padding and reports the given [paddingTestValue] instead.
  ///
  /// Use [FakeViewPadding] to set this value for testing.
  @Deprecated(
    'Use WidgetTester.view.padding instead. '
    'Deprecated to prepare for the upcoming multi-window support. '
    'This feature was deprecated after v3.9.0-0.1.pre.',
  )
  // ignore: avoid_setters_without_getters
  set paddingTestValue(ViewPadding value) {
    _view.padding = value is FakeViewPadding ? value : FakeViewPadding._wrap(value);
  }

  /// Deletes any existing test padding and returns to using the real padding.
  @Deprecated(
    'Use WidgetTester.view.resetPadding() instead. '
    'Deprecated to prepare for the upcoming multi-window support. '
    'This feature was deprecated after v3.9.0-0.1.pre.',
  )
  void clearPaddingTestValue() {
    _view.resetPadding();
  }

  @Deprecated(
    'Use WidgetTester.view.gestureSettings instead. '
    'Deprecated to prepare for the upcoming multi-window support. '
    'This feature was deprecated after v3.9.0-0.1.pre.',
  )
  @override
  GestureSettings get gestureSettings => _view.gestureSettings;

  /// Hides the real gesture settings and reports the given [gestureSettingsTestValue] instead.
  @Deprecated(
    'Use WidgetTester.view.gestureSettings instead. '
    'Deprecated to prepare for the upcoming multi-window support. '
    'This feature was deprecated after v3.9.0-0.1.pre.',
  )
  // ignore: avoid_setters_without_getters
  set gestureSettingsTestValue(GestureSettings gestureSettingsTestValue) {
    _view.gestureSettings = gestureSettingsTestValue;
  }

  /// Deletes any existing test gesture settings and returns to using the real gesture settings.
  @Deprecated(
    'Use WidgetTester.view.resetGestureSettings() instead. '
    'Deprecated to prepare for the upcoming multi-window support. '
    'This feature was deprecated after v3.9.0-0.1.pre.',
  )
  void clearGestureSettingsTestValue() {
    _view.resetGestureSettings();
  }

  @Deprecated(
    'Use WidgetTester.view.displayFeatures instead. '
    'Deprecated to prepare for the upcoming multi-window support. '
    'This feature was deprecated after v3.9.0-0.1.pre.',
  )
  @override
  List<DisplayFeature> get displayFeatures => _view.displayFeatures;

  /// Hides the real displayFeatures and reports the given [displayFeaturesTestValue] instead.
  @Deprecated(
    'Use WidgetTester.view.displayFeatures instead. '
    'Deprecated to prepare for the upcoming multi-window support. '
    'This feature was deprecated after v3.9.0-0.1.pre.',
  )
  // ignore: avoid_setters_without_getters
  set displayFeaturesTestValue(List<DisplayFeature> displayFeaturesTestValue) {
    _view.displayFeatures = displayFeaturesTestValue;
  }

  /// Deletes any existing test padding and returns to using the real padding.
  @Deprecated(
    'Use WidgetTester.view.resetDisplayFeatures() instead. '
    'Deprecated to prepare for the upcoming multi-window support. '
    'This feature was deprecated after v3.9.0-0.1.pre.',
  )
  void clearDisplayFeaturesTestValue() {
    _view.resetDisplayFeatures();
  }

  @Deprecated(
    'Use WidgetTester.view.systemGestureInsets instead. '
    'Deprecated to prepare for the upcoming multi-window support. '
    'This feature was deprecated after v3.9.0-0.1.pre.',
  )
  @override
  ViewPadding get systemGestureInsets => _view.systemGestureInsets;

  /// Hides the real system gesture insets and reports the given
  /// [systemGestureInsetsTestValue] instead.
  ///
  /// Use [FakeViewPadding] to set this value for testing.
  @Deprecated(
    'Use WidgetTester.view.systemGestureInsets instead. '
    'Deprecated to prepare for the upcoming multi-window support. '
    'This feature was deprecated after v3.9.0-0.1.pre.',
  )
  // ignore: avoid_setters_without_getters
  set systemGestureInsetsTestValue(ViewPadding value) {
    _view.systemGestureInsets = value is FakeViewPadding ? value : FakeViewPadding._wrap(value);
  }

  /// Deletes any existing test system gesture insets and returns to using the real system gesture insets.
  @Deprecated(
    'Use WidgetTester.view.resetSystemGestureInsets() instead. '
    'Deprecated to prepare for the upcoming multi-window support. '
    'This feature was deprecated after v3.9.0-0.1.pre.',
  )
  void clearSystemGestureInsetsTestValue() {
    _view.resetSystemGestureInsets();
  }

  @Deprecated(
    'Use WidgetTester.platformDispatcher.onMetricsChanged instead. '
    'Deprecated to prepare for the upcoming multi-window support. '
    'This feature was deprecated after v3.9.0-0.1.pre.',
  )
  @override
  VoidCallback? get onMetricsChanged => platformDispatcher.onMetricsChanged;
  @Deprecated(
    'Use WidgetTester.platformDispatcher.onMetricsChanged instead. '
    'Deprecated to prepare for the upcoming multi-window support. '
    'This feature was deprecated after v3.9.0-0.1.pre.',
  )
  @override
  set onMetricsChanged(VoidCallback? callback) {
    platformDispatcher.onMetricsChanged = callback;
  }

  @Deprecated(
    'Use WidgetTester.platformDispatcher.locale instead. '
    'Deprecated to prepare for the upcoming multi-window support. '
    'This feature was deprecated after v3.9.0-0.1.pre.',
  )
  @override
  Locale get locale => platformDispatcher.locale;

  @Deprecated(
    'Use WidgetTester.platformDispatcher.locales instead. '
    'Deprecated to prepare for the upcoming multi-window support. '
    'This feature was deprecated after v3.9.0-0.1.pre.',
  )
  @override
  List<Locale> get locales => platformDispatcher.locales;

  @Deprecated(
    'Use WidgetTester.platformDispatcher.onLocaleChanged instead. '
    'Deprecated to prepare for the upcoming multi-window support. '
    'This feature was deprecated after v3.9.0-0.1.pre.',
  )
  @override
  VoidCallback? get onLocaleChanged => platformDispatcher.onLocaleChanged;
  @Deprecated(
    'Use WidgetTester.platformDispatcher.onLocaleChanged instead. '
    'Deprecated to prepare for the upcoming multi-window support. '
    'This feature was deprecated after v3.9.0-0.1.pre.',
  )
  @override
  set onLocaleChanged(VoidCallback? callback) {
    platformDispatcher.onLocaleChanged = callback;
  }

  @Deprecated(
    'Use WidgetTester.platformDispatcher.initialLifecycleState instead. '
    'Deprecated to prepare for the upcoming multi-window support. '
    'This feature was deprecated after v3.9.0-0.1.pre.',
  )
  @override
  String get initialLifecycleState => platformDispatcher.initialLifecycleState;

  @Deprecated(
    'Use WidgetTester.platformDispatcher.textScaleFactor instead. '
    'Deprecated to prepare for the upcoming multi-window support. '
    'This feature was deprecated after v3.9.0-0.1.pre.',
  )
  @override
  double get textScaleFactor => platformDispatcher.textScaleFactor;

  @Deprecated(
    'Use WidgetTester.platformDispatcher.platformBrightness instead. '
    'Deprecated to prepare for the upcoming multi-window support. '
    'This feature was deprecated after v3.9.0-0.1.pre.',
  )
  @override
  Brightness get platformBrightness => platformDispatcher.platformBrightness;
  @Deprecated(
    'Use WidgetTester.platformDispatcher.onPlatformBrightnessChanged instead. '
    'Deprecated to prepare for the upcoming multi-window support. '
    'This feature was deprecated after v3.9.0-0.1.pre.',
  )
  @override
  VoidCallback? get onPlatformBrightnessChanged => platformDispatcher.onPlatformBrightnessChanged;
  @Deprecated(
    'Use WidgetTester.platformDispatcher.onPlatformBrightnessChanged instead. '
    'Deprecated to prepare for the upcoming multi-window support. '
    'This feature was deprecated after v3.9.0-0.1.pre.',
  )
  @override
  set onPlatformBrightnessChanged(VoidCallback? callback) {
    platformDispatcher.onPlatformBrightnessChanged = callback;
  }

  @Deprecated(
    'Use WidgetTester.platformDispatcher.alwaysUse24HourFormat instead. '
    'Deprecated to prepare for the upcoming multi-window support. '
    'This feature was deprecated after v3.9.0-0.1.pre.',
  )
  @override
  bool get alwaysUse24HourFormat => platformDispatcher.alwaysUse24HourFormat;

  @Deprecated(
    'Use WidgetTester.platformDispatcher.onTextScaleFactorChanged instead. '
    'Deprecated to prepare for the upcoming multi-window support. '
    'This feature was deprecated after v3.9.0-0.1.pre.',
  )
  @override
  VoidCallback? get onTextScaleFactorChanged => platformDispatcher.onTextScaleFactorChanged;
  @Deprecated(
    'Use WidgetTester.platformDispatcher.onTextScaleFactorChanged instead. '
    'Deprecated to prepare for the upcoming multi-window support. '
    'This feature was deprecated after v3.9.0-0.1.pre.',
  )
  @override
  set onTextScaleFactorChanged(VoidCallback? callback) {
    platformDispatcher.onTextScaleFactorChanged = callback;
  }

  @Deprecated(
    'Use WidgetTester.platformDispatcher.nativeSpellCheckServiceDefined instead. '
    'Deprecated to prepare for the upcoming multi-window support. '
    'This feature was deprecated after v3.9.0-0.1.pre.',
  )
  @override
  bool get nativeSpellCheckServiceDefined => platformDispatcher.nativeSpellCheckServiceDefined;
  @Deprecated(
    'Use WidgetTester.platformDispatcher.nativeSpellCheckServiceDefinedTestValue instead. '
    'Deprecated to prepare for the upcoming multi-window support. '
    'This feature was deprecated after v3.9.0-0.1.pre.',
  )
  // ignore: avoid_setters_without_getters
  set nativeSpellCheckServiceDefinedTestValue(bool nativeSpellCheckServiceDefinedTestValue) {
    platformDispatcher.nativeSpellCheckServiceDefinedTestValue =
        nativeSpellCheckServiceDefinedTestValue;
  }

  @Deprecated(
    'Use WidgetTester.platformDispatcher.brieflyShowPassword instead. '
    'Deprecated to prepare for the upcoming multi-window support. '
    'This feature was deprecated after v3.9.0-0.1.pre.',
  )
  @override
  bool get brieflyShowPassword => platformDispatcher.brieflyShowPassword;

  @Deprecated(
    'Use WidgetTester.platformDispatcher.onBeginFrame instead. '
    'Deprecated to prepare for the upcoming multi-window support. '
    'This feature was deprecated after v3.9.0-0.1.pre.',
  )
  @override
  FrameCallback? get onBeginFrame => platformDispatcher.onBeginFrame;
  @Deprecated(
    'Use WidgetTester.platformDispatcher.onBeginFrame instead. '
    'Deprecated to prepare for the upcoming multi-window support. '
    'This feature was deprecated after v3.9.0-0.1.pre.',
  )
  @override
  set onBeginFrame(FrameCallback? callback) {
    platformDispatcher.onBeginFrame = callback;
  }

  @Deprecated(
    'Use WidgetTester.platformDispatcher.onDrawFrame instead. '
    'Deprecated to prepare for the upcoming multi-window support. '
    'This feature was deprecated after v3.9.0-0.1.pre.',
  )
  @override
  VoidCallback? get onDrawFrame => platformDispatcher.onDrawFrame;
  @Deprecated(
    'Use WidgetTester.platformDispatcher.onDrawFrame instead. '
    'Deprecated to prepare for the upcoming multi-window support. '
    'This feature was deprecated after v3.9.0-0.1.pre.',
  )
  @override
  set onDrawFrame(VoidCallback? callback) {
    platformDispatcher.onDrawFrame = callback;
  }

  @Deprecated(
    'Use WidgetTester.platformDispatcher.onReportTimings instead. '
    'Deprecated to prepare for the upcoming multi-window support. '
    'This feature was deprecated after v3.9.0-0.1.pre.',
  )
  @override
  TimingsCallback? get onReportTimings => platformDispatcher.onReportTimings;
  @Deprecated(
    'Use WidgetTester.platformDispatcher.onReportTimings instead. '
    'Deprecated to prepare for the upcoming multi-window support. '
    'This feature was deprecated after v3.9.0-0.1.pre.',
  )
  @override
  set onReportTimings(TimingsCallback? callback) {
    platformDispatcher.onReportTimings = callback;
  }

  @Deprecated(
    'Use WidgetTester.platformDispatcher.onPointerDataPacket instead. '
    'Deprecated to prepare for the upcoming multi-window support. '
    'This feature was deprecated after v3.9.0-0.1.pre.',
  )
  @override
  PointerDataPacketCallback? get onPointerDataPacket => platformDispatcher.onPointerDataPacket;
  @Deprecated(
    'Use WidgetTester.platformDispatcher.onPointerDataPacket instead. '
    'Deprecated to prepare for the upcoming multi-window support. '
    'This feature was deprecated after v3.9.0-0.1.pre.',
  )
  @override
  set onPointerDataPacket(PointerDataPacketCallback? callback) {
    platformDispatcher.onPointerDataPacket = callback;
  }

  @Deprecated(
    'Use WidgetTester.platformDispatcher.defaultRouteName instead. '
    'Deprecated to prepare for the upcoming multi-window support. '
    'This feature was deprecated after v3.9.0-0.1.pre.',
  )
  @override
  String get defaultRouteName => platformDispatcher.defaultRouteName;

  @Deprecated(
    'Use WidgetTester.platformDispatcher.scheduleFrame() instead. '
    'Deprecated to prepare for the upcoming multi-window support. '
    'This feature was deprecated after v3.9.0-0.1.pre.',
  )
  @override
  void scheduleFrame() {
    platformDispatcher.scheduleFrame();
  }

  @Deprecated(
    'Use WidgetTester.view.render(scene) instead. '
    'Deprecated to prepare for the upcoming multi-window support. '
    'This feature was deprecated after v3.9.0-0.1.pre.',
  )
  @override
  void render(Scene scene, {Size? size}) {
    _view.render(scene, size: size);
  }

  @Deprecated(
    'Use WidgetTester.platformDispatcher.semanticsEnabled instead. '
    'Deprecated to prepare for the upcoming multi-window support. '
    'This feature was deprecated after v3.9.0-0.1.pre.',
  )
  @override
  bool get semanticsEnabled => platformDispatcher.semanticsEnabled;

  @Deprecated(
    'Use WidgetTester.platformDispatcher.onSemanticsEnabledChanged instead. '
    'Deprecated to prepare for the upcoming multi-window support. '
    'This feature was deprecated after v3.9.0-0.1.pre.',
  )
  @override
  VoidCallback? get onSemanticsEnabledChanged => platformDispatcher.onSemanticsEnabledChanged;
  @Deprecated(
    'Use WidgetTester.platformDispatcher.onSemanticsEnabledChanged instead. '
    'Deprecated to prepare for the upcoming multi-window support. '
    'This feature was deprecated after v3.9.0-0.1.pre.',
  )
  @override
  set onSemanticsEnabledChanged(VoidCallback? callback) {
    platformDispatcher.onSemanticsEnabledChanged = callback;
  }

  @Deprecated(
    'Use WidgetTester.platformDispatcher.accessibilityFeatures instead. '
    'Deprecated to prepare for the upcoming multi-window support. '
    'This feature was deprecated after v3.9.0-0.1.pre.',
  )
  @override
  AccessibilityFeatures get accessibilityFeatures => platformDispatcher.accessibilityFeatures;

  @Deprecated(
    'Use WidgetTester.platformDispatcher.onAccessibilityFeaturesChanged instead. '
    'Deprecated to prepare for the upcoming multi-window support. '
    'This feature was deprecated after v3.9.0-0.1.pre.',
  )
  @override
  VoidCallback? get onAccessibilityFeaturesChanged =>
      platformDispatcher.onAccessibilityFeaturesChanged;
  @Deprecated(
    'Use WidgetTester.platformDispatcher.onAccessibilityFeaturesChanged instead. '
    'Deprecated to prepare for the upcoming multi-window support. '
    'This feature was deprecated after v3.9.0-0.1.pre.',
  )
  @override
  set onAccessibilityFeaturesChanged(VoidCallback? callback) {
    platformDispatcher.onAccessibilityFeaturesChanged = callback;
  }

  @Deprecated(
    'Use WidgetTester.view.updateSemantics(update) instead. '
    'Deprecated to prepare for the upcoming multi-window support. '
    'This feature was deprecated after v3.9.0-0.1.pre.',
  )
  @override
  void updateSemantics(SemanticsUpdate update) {
    _view.updateSemantics(update);
  }

  @Deprecated(
    'Use WidgetTester.platformDispatcher.setIsolateDebugName(name) instead. '
    'Deprecated to prepare for the upcoming multi-window support. '
    'This feature was deprecated after v3.9.0-0.1.pre.',
  )
  @override
  void setIsolateDebugName(String name) {
    platformDispatcher.setIsolateDebugName(name);
  }

  @Deprecated(
    'Use WidgetTester.platformDispatcher.sendPlatformMessage(name, data, callback) instead. '
    'Deprecated to prepare for the upcoming multi-window support. '
    'This feature was deprecated after v3.9.0-0.1.pre.',
  )
  @override
  void sendPlatformMessage(String name, ByteData? data, PlatformMessageResponseCallback? callback) {
    platformDispatcher.sendPlatformMessage(name, data, callback);
  }

  /// Delete any test value properties that have been set on this [TestWindow]
  /// as well as its [platformDispatcher].
  ///
  /// After calling this, the real [SingletonFlutterWindow] and
  /// [PlatformDispatcher] values are reported again.
  ///
  /// If desired, clearing of properties can be done on an individual basis,
  /// e.g., [clearDevicePixelRatioTestValue].
  @Deprecated(
    'Use WidgetTester.platformDispatcher.clearAllTestValues() and WidgetTester.view.reset() instead. '
    'Deprecated to prepare for the upcoming multi-window support. '
    'This feature was deprecated after v3.9.0-0.1.pre.',
  )
  void clearAllTestValues() {
    clearDevicePixelRatioTestValue();
    clearPaddingTestValue();
    clearGestureSettingsTestValue();
    clearDisplayFeaturesTestValue();
    clearPhysicalSizeTestValue();
    clearViewInsetsTestValue();
    platformDispatcher.clearAllTestValues();
  }

  @override
  @Deprecated(
    'Use WidgetTester.platformDispatcher.onFrameDataChanged instead. '
    'Deprecated to prepare for the upcoming multi-window support. '
    'This feature was deprecated after v3.9.0-0.1.pre.',
  )
  VoidCallback? get onFrameDataChanged => platformDispatcher.onFrameDataChanged;
  @Deprecated(
    'Use WidgetTester.platformDispatcher.onFrameDataChanged instead. '
    'Deprecated to prepare for the upcoming multi-window support. '
    'This feature was deprecated after v3.9.0-0.1.pre.',
  )
  @override
  set onFrameDataChanged(VoidCallback? value) {
    platformDispatcher.onFrameDataChanged = value;
  }

  @Deprecated(
    'Use WidgetTester.platformDispatcher.onKeyData instead. '
    'Deprecated to prepare for the upcoming multi-window support. '
    'This feature was deprecated after v3.9.0-0.1.pre.',
  )
  @override
  KeyDataCallback? get onKeyData => platformDispatcher.onKeyData;
  @Deprecated(
    'Use WidgetTester.platformDispatcher.onKeyData instead. '
    'Deprecated to prepare for the upcoming multi-window support. '
    'This feature was deprecated after v3.9.0-0.1.pre.',
  )
  @override
  set onKeyData(KeyDataCallback? value) {
    platformDispatcher.onKeyData = value;
  }

  @Deprecated(
    'Use WidgetTester.platformDispatcher.onSystemFontFamilyChanged instead. '
    'Deprecated to prepare for the upcoming multi-window support. '
    'This feature was deprecated after v3.9.0-0.1.pre.',
  )
  @override
  VoidCallback? get onSystemFontFamilyChanged => platformDispatcher.onSystemFontFamilyChanged;
  @Deprecated(
    'Use WidgetTester.platformDispatcher.onSystemFontFamilyChanged instead. '
    'Deprecated to prepare for the upcoming multi-window support. '
    'This feature was deprecated after v3.9.0-0.1.pre.',
  )
  @override
  set onSystemFontFamilyChanged(VoidCallback? value) {
    platformDispatcher.onSystemFontFamilyChanged = value;
  }

  @Deprecated(
    'Use WidgetTester.platformDispatcher.computePlatformResolvedLocale(supportedLocales) instead. '
    'Deprecated to prepare for the upcoming multi-window support. '
    'This feature was deprecated after v3.9.0-0.1.pre.',
  )
  @override
  Locale? computePlatformResolvedLocale(List<Locale> supportedLocales) {
    return platformDispatcher.computePlatformResolvedLocale(supportedLocales);
  }

  @Deprecated(
    'Use WidgetTester.platformDispatcher.frameData instead. '
    'Deprecated to prepare for the upcoming multi-window support. '
    'This feature was deprecated after v3.9.0-0.1.pre.',
  )
  @override
  FrameData get frameData => platformDispatcher.frameData;

  @Deprecated(
    'Use WidgetTester.platformDispatcher.systemFontFamily instead. '
    'Deprecated to prepare for the upcoming multi-window support. '
    'This feature was deprecated after v3.9.0-0.1.pre.',
  )
  @override
  String? get systemFontFamily => platformDispatcher.systemFontFamily;

  @Deprecated(
    'Use WidgetTester.view.viewId instead. '
    'Deprecated to prepare for the upcoming multi-window support. '
    'This feature was deprecated after v3.9.0-0.1.pre.',
  )
  @override
  int get viewId => _view.viewId;

  /// This gives us some grace time when the dart:ui side adds something to
  /// [SingletonFlutterWindow], and makes things easier when we do rolls to give
  /// us time to catch up.
  @override
  dynamic noSuchMethod(Invocation invocation) {
    return null;
  }
}
