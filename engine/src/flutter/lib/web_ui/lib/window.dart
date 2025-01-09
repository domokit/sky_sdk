// Copyright 2013 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

part of ui;

abstract class Display {
  int get id;
  double get devicePixelRatio;
  Size get size;
  double get refreshRate;
}

abstract class FlutterView {
  PlatformDispatcher get platformDispatcher;
  int get viewId;
  double get devicePixelRatio;
  ViewConstraints get physicalConstraints;
  Size get physicalSize;
  ViewPadding get viewInsets;
  ViewPadding get viewPadding;
  ViewPadding get systemGestureInsets;
  ViewPadding get padding;
  GestureSettings get gestureSettings;
  List<DisplayFeature> get displayFeatures;
  Display get display;
  void render(Scene scene, {Size? size});
  void updateSemantics(SemanticsUpdate update) => platformDispatcher.updateSemantics(update);
}

abstract class SingletonFlutterWindow extends FlutterView {
  VoidCallback? get onMetricsChanged;
  set onMetricsChanged(VoidCallback? callback);

  Locale get locale;
  List<Locale> get locales;

  Locale? computePlatformResolvedLocale(List<Locale> supportedLocales);

  VoidCallback? get onLocaleChanged;
  set onLocaleChanged(VoidCallback? callback);

  String get initialLifecycleState;

  double get textScaleFactor;

  bool get nativeSpellCheckServiceDefined;

  bool get supportsShowingSystemContextMenu;

  bool get brieflyShowPassword;

  bool get alwaysUse24HourFormat;

  VoidCallback? get onTextScaleFactorChanged;
  set onTextScaleFactorChanged(VoidCallback? callback);

  Brightness get platformBrightness;

  VoidCallback? get onPlatformBrightnessChanged;
  set onPlatformBrightnessChanged(VoidCallback? callback);

  String? get systemFontFamily;

  VoidCallback? get onSystemFontFamilyChanged;
  set onSystemFontFamilyChanged(VoidCallback? callback);

  FrameCallback? get onBeginFrame;
  set onBeginFrame(FrameCallback? callback);

  VoidCallback? get onDrawFrame;
  set onDrawFrame(VoidCallback? callback);

  TimingsCallback? get onReportTimings;
  set onReportTimings(TimingsCallback? callback);

  PointerDataPacketCallback? get onPointerDataPacket;
  set onPointerDataPacket(PointerDataPacketCallback? callback);

  KeyDataCallback? get onKeyData;
  set onKeyData(KeyDataCallback? callback);

  String get defaultRouteName;

  void scheduleFrame();

  bool get semanticsEnabled;

  VoidCallback? get onSemanticsEnabledChanged;
  set onSemanticsEnabledChanged(VoidCallback? callback);

  FrameData get frameData;

  VoidCallback? get onFrameDataChanged;
  set onFrameDataChanged(VoidCallback? callback);

  AccessibilityFeatures get accessibilityFeatures;

  VoidCallback? get onAccessibilityFeaturesChanged;
  set onAccessibilityFeaturesChanged(VoidCallback? callback);

  void sendPlatformMessage(String name, ByteData? data, PlatformMessageResponseCallback? callback);

  PlatformMessageCallback? get onPlatformMessage;
  set onPlatformMessage(PlatformMessageCallback? callback);

  void setIsolateDebugName(String name);
}

abstract class AccessibilityFeatures {
  bool get accessibleNavigation;
  bool get invertColors;
  bool get disableAnimations;
  bool get boldText;
  bool get reduceMotion;
  bool get highContrast;
  bool get onOffSwitchLabels;
}

enum Brightness { dark, light }

// Unimplemented classes.
// TODO(dit): see https://github.com/flutter/flutter/issues/33614.
class CallbackHandle {
  CallbackHandle.fromRawHandle(this._handle);

  final int _handle;

  int toRawHandle() => _handle;

  @override
  bool operator ==(Object other) => identical(this, other);

  @override
  // ignore: unnecessary_overrides
  int get hashCode => super.hashCode;
}

// TODO(dit): see https://github.com/flutter/flutter/issues/33615.
abstract final class PluginUtilities {
  static CallbackHandle? getCallbackHandle(Function callback) {
    throw UnimplementedError();
  }

  static Function? getCallbackFromHandle(CallbackHandle handle) {
    throw UnimplementedError();
  }
}

abstract final class IsolateNameServer {
  static dynamic lookupPortByName(String name) {
    throw UnimplementedError();
  }

  static bool registerPortWithName(dynamic port, String name) {
    throw UnimplementedError();
  }

  static bool removePortNameMapping(String name) {
    throw UnimplementedError();
  }
}

SingletonFlutterWindow get window => engine.window;

class FrameData {
  const FrameData.webOnly();

  int get frameNumber => -1;
}

class GestureSettings {
  const GestureSettings({this.physicalTouchSlop, this.physicalDoubleTapSlop});

  final double? physicalTouchSlop;

  final double? physicalDoubleTapSlop;

  GestureSettings copyWith({double? physicalTouchSlop, double? physicalDoubleTapSlop}) {
    return GestureSettings(
      physicalTouchSlop: physicalTouchSlop ?? this.physicalTouchSlop,
      physicalDoubleTapSlop: physicalDoubleTapSlop ?? this.physicalDoubleTapSlop,
    );
  }

  @override
  bool operator ==(Object other) {
    if (other.runtimeType != runtimeType) {
      return false;
    }
    return other is GestureSettings &&
        other.physicalTouchSlop == physicalTouchSlop &&
        other.physicalDoubleTapSlop == physicalDoubleTapSlop;
  }

  @override
  int get hashCode => Object.hash(physicalTouchSlop, physicalDoubleTapSlop);

  @override
  String toString() =>
      'GestureSettings(physicalTouchSlop: $physicalTouchSlop, physicalDoubleTapSlop: $physicalDoubleTapSlop)';
}
