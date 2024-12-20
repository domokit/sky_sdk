// Copyright 2014 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

/// @docImport 'package:flutter/cupertino.dart';
/// @docImport 'package:flutter/material.dart';
library;

import 'dart:async';
import 'dart:ui';

import 'package:flutter/foundation.dart';

import 'binding.dart';
import 'system_channels.dart';

export 'dart:ui' show Brightness, Color;

export 'binding.dart' show SystemUiChangeCallback;

/// Specifies a particular device orientation.
///
/// To determine which values correspond to which orientations, first position
/// the device in its default orientation (this is the orientation that the
/// system first uses for its boot logo, or the orientation in which the
/// hardware logos or markings are upright, or the orientation in which the
/// cameras are at the top). If this is a portrait orientation, then this is
/// [portraitUp]. Otherwise, it's [landscapeLeft]. As you rotate the device by
/// 90 degrees in a counter-clockwise direction around the axis that pierces the
/// screen, you step through each value in this enum in the order given.
///
/// For a device with a landscape default orientation, the orientation obtained
/// by rotating the device 90 degrees clockwise from its default orientation is
/// [portraitUp].
///
/// Used by [SystemChrome.setPreferredOrientations].
enum DeviceOrientation {
  /// If the device shows its boot logo in portrait, then the boot logo is shown
  /// in [portraitUp]. Otherwise, the device shows its boot logo in landscape
  /// and this orientation is obtained by rotating the device 90 degrees
  /// clockwise from its boot orientation.
  portraitUp,

  /// The orientation that is 90 degrees counterclockwise from [portraitUp].
  ///
  /// If the device shows its boot logo in landscape, then the boot logo is
  /// shown in [landscapeLeft].
  landscapeLeft,

  /// The orientation that is 180 degrees from [portraitUp].
  portraitDown,

  /// The orientation that is 90 degrees clockwise from [portraitUp].
  landscapeRight,
}

/// Specifies a description of the application that is pertinent to the
/// embedder's application switcher (also known as "recent tasks") user
/// interface.
///
/// Used by [SystemChrome.setApplicationSwitcherDescription].
@immutable
class ApplicationSwitcherDescription {
  /// Creates an ApplicationSwitcherDescription.
  const ApplicationSwitcherDescription({this.label, this.primaryColor});

  /// A label and description of the current state of the application.
  final String? label;

  /// The application's primary color.
  ///
  /// This may influence the color that the operating system uses to represent
  /// the application.
  final int? primaryColor;
}

/// Specifies a system overlay at a particular location.
///
/// Used by [SystemChrome.setEnabledSystemUIMode].
enum SystemUiOverlay {
  /// The status bar provided by the embedder on the top of the application
  /// surface, if any.
  top,

  /// The status bar provided by the embedder on the bottom of the application
  /// surface, if any.
  bottom,
}

/// Describes different display configurations for both Android and iOS.
///
/// These modes mimic Android-specific display setups.
///
/// Used by [SystemChrome.setEnabledSystemUIMode].
///
/// Flutter apps use [SystemUiMode.edgeToEdge] by default and setting any
/// of the other [SystemUiMode]s will NOT work unless you perform the migration
/// detailed in
/// https://docs.flutter.dev/release/breaking-changes/default-systemuimode-edge-to-edge.
enum SystemUiMode {
  /// Fullscreen display with status and navigation bars presentable by tapping
  /// anywhere on the display.
  ///
  /// Available starting at Android SDK 4.1 (API 16). Earlier versions of Android
  /// will not be affected by this setting. However, if your app targets Android
  /// SDK 15 (API 35) or later (Flutter does this by default), then you must
  /// migrate using the instructions in
  /// https://docs.flutter.dev/release/breaking-changes/default-systemuimode-edge-to-edge
  /// to use this mode.
  ///
  /// For applications running on iOS, the status bar and home indicator will be
  /// hidden for a similar fullscreen experience.
  ///
  /// Tapping on the screen displays overlays, this gesture is not received by
  /// the application.
  ///
  /// See also:
  ///
  ///   * [SystemUiChangeCallback], used to listen and respond to the change in
  ///     system overlays.
  leanBack,

  /// Fullscreen display with status and navigation bars presentable through a
  /// swipe gesture at the edges of the display.
  ///
  /// Available starting at Android SDK 4.4 (API 19). Earlier versions of
  /// Android will not be affected by this setting. However, if your app targets
  /// Android SDK 15 (API 35) or later (Flutter does this by default), then you
  /// must migrate using the instructions in
  /// https://docs.flutter.dev/release/breaking-changes/default-systemuimode-edge-to-edge
  /// to use this mode.
  ///
  /// For applications running on iOS, the status bar and home indicator will be
  /// hidden for a similar fullscreen experience.
  ///
  /// A swipe gesture from the edge of the screen displays overlays. In contrast
  /// to [SystemUiMode.immersiveSticky], this gesture is not received by the
  /// application.
  ///
  /// See also:
  ///
  ///   * [SystemUiChangeCallback], used to listen and respond to the change in
  ///     system overlays.
  immersive,

  /// Fullscreen display with status and navigation bars presentable through a
  /// swipe gesture at the edges of the display.
  ///
  /// Available starting at Android SDK 4.4 (API 19). Earlier versions of
  /// Android will not be affected by this setting. However, if your app targets
  /// Android SDK 15 (API 35) or later (Flutter does this by default), then you
  /// must migrate using the instructions in
  /// https://docs.flutter.dev/release/breaking-changes/default-systemuimode-edge-to-edge
  /// to use this mode.
  ///
  /// For applications running on iOS, the status bar and home indicator will be
  /// hidden for a similar fullscreen experience.
  ///
  /// A swipe gesture from the edge of the screen displays overlays. In contrast
  /// to [SystemUiMode.immersive], this gesture is received by the application.
  ///
  /// See also:
  ///
  ///   * [SystemUiChangeCallback], used to listen and respond to the change in
  ///     system overlays.
  immersiveSticky,

  /// Fullscreen display with status and navigation elements rendered over the
  /// application.
  ///
  /// Available starting at Android SDK 10 (API 29). Earlier versions of Android
  /// will not be affected by this setting.
  ///
  /// If your app targets Android SDK 15 (API 35) or later (Flutter does this by
  /// default), then this mode is used by default on Android. This mode is also
  /// used by default on iOS.
  ///
  /// For applications running on iOS, the status bar and home indicator will be
  /// visible.
  ///
  /// The system overlays will not disappear or reappear in this mode as they
  /// are permanently displayed on top of the application.
  ///
  /// See also:
  ///
  ///   * [SystemUiOverlayStyle], can be used to configure transparent status and
  ///     navigation bars with or without a contrast scrim.
  edgeToEdge,

  /// Declares manually configured [SystemUiOverlay]s.
  ///
  /// When using this mode with [SystemChrome.setEnabledSystemUIMode], the
  /// preferred overlays must be set by the developer.
  ///
  /// When [SystemUiOverlay.top] is enabled, the status bar will remain visible
  /// on all platforms. Omitting this overlay will hide the status bar on iOS &
  /// Android.
  ///
  /// When [SystemUiOverlay.bottom] is enabled, the navigation bar and home
  /// indicator of Android and iOS applications will remain visible. Omitting this
  /// overlay will hide them.
  ///
  /// Omitting both overlays will result in the same configuration as
  /// [SystemUiMode.leanBack].
  ///
  /// If your app targets Android SDK 15 (API 35) or later, then you must
  /// migrate using the instructions in
  /// https://docs.flutter.dev/release/breaking-changes/default-systemuimode-edge-to-edge
  /// to use this mode.
  manual,
}

/// Specifies a preference for the style of the system overlays.
///
/// Used by [AppBar.systemOverlayStyle] for declaratively setting the style of
/// the system overlays, and by [SystemChrome.setSystemUIOverlayStyle] for
/// imperatively setting the style of the system overlays.
@immutable
class SystemUiOverlayStyle {
  /// Creates a new [SystemUiOverlayStyle].
  const SystemUiOverlayStyle({
    this.systemNavigationBarColor,
    this.systemNavigationBarDividerColor,
    this.systemNavigationBarIconBrightness,
    this.systemNavigationBarContrastEnforced,
    this.statusBarColor,
    this.statusBarBrightness,
    this.statusBarIconBrightness,
    this.systemStatusBarContrastEnforced,
  });

  /// The color of the system bottom navigation bar.
  ///
  /// Only honored in Android versions O and greater.
  final Color? systemNavigationBarColor;

  /// The color of the divider between the system's bottom navigation bar and the app's content.
  ///
  /// Only honored in Android versions P and greater.
  final Color? systemNavigationBarDividerColor;

  /// The brightness of the system navigation bar icons.
  ///
  /// Only honored in Android versions O and greater.
  /// When set to [Brightness.light], the system navigation bar icons are light.
  /// When set to [Brightness.dark], the system navigation bar icons are dark.
  final Brightness? systemNavigationBarIconBrightness;

  /// Overrides the contrast enforcement when setting a transparent navigation
  /// bar.
  ///
  /// When setting a transparent navigation bar in SDK 29+, or Android 10 and up,
  /// a translucent body scrim may be applied behind the button navigation bar
  /// to ensure contrast with buttons and the background of the application.
  ///
  /// SDK 28-, or Android P and lower, will not apply this body scrim.
  ///
  /// Setting this to false overrides the default body scrim.
  ///
  /// See also:
  ///
  ///   * [SystemUiOverlayStyle.systemNavigationBarColor], which is overridden
  ///   when transparent to enforce this contrast policy.
  final bool? systemNavigationBarContrastEnforced;

  /// The color of top status bar.
  ///
  /// Only honored in Android version M and greater.
  final Color? statusBarColor;

  /// The brightness of top status bar.
  ///
  /// Only honored in iOS.
  final Brightness? statusBarBrightness;

  /// The brightness of the top status bar icons.
  ///
  /// Only honored in Android version M and greater.
  final Brightness? statusBarIconBrightness;

  /// Overrides the contrast enforcement when setting a transparent status
  /// bar.
  ///
  /// When setting a transparent status bar in SDK 29+, or Android 10 and up,
  /// a translucent body scrim may be applied to ensure contrast with icons and
  /// the background of the application.
  ///
  /// SDK 28-, or Android P and lower, will not apply this body scrim.
  ///
  /// Setting this to false overrides the default body scrim.
  ///
  /// See also:
  ///
  ///   * [SystemUiOverlayStyle.statusBarColor], which is overridden
  ///   when transparent to enforce this contrast policy.
  final bool? systemStatusBarContrastEnforced;

  /// System overlays should be drawn with a light color. Intended for
  /// applications with a dark background.
  static const SystemUiOverlayStyle light = SystemUiOverlayStyle(
    systemNavigationBarColor: Color(0xFF000000),
    systemNavigationBarIconBrightness: Brightness.light,
    statusBarIconBrightness: Brightness.light,
    statusBarBrightness: Brightness.dark,
  );

  /// System overlays should be drawn with a dark color. Intended for
  /// applications with a light background.
  static const SystemUiOverlayStyle dark = SystemUiOverlayStyle(
    systemNavigationBarColor: Color(0xFF000000),
    systemNavigationBarIconBrightness: Brightness.light,
    statusBarIconBrightness: Brightness.dark,
    statusBarBrightness: Brightness.light,
  );

  /// Convert this event to a map for serialization.
  Map<String, dynamic> _toMap() {
    return <String, dynamic>{
      'systemNavigationBarColor': systemNavigationBarColor?.value,
      'systemNavigationBarDividerColor': systemNavigationBarDividerColor?.value,
      'systemStatusBarContrastEnforced': systemStatusBarContrastEnforced,
      'statusBarColor': statusBarColor?.value,
      'statusBarBrightness': statusBarBrightness?.toString(),
      'statusBarIconBrightness': statusBarIconBrightness?.toString(),
      'systemNavigationBarIconBrightness': systemNavigationBarIconBrightness?.toString(),
      'systemNavigationBarContrastEnforced': systemNavigationBarContrastEnforced,
    };
  }

  @override
  String toString() => '${objectRuntimeType(this, 'SystemUiOverlayStyle')}(${_toMap()})';

  /// Creates a copy of this theme with the given fields replaced with new values.
  SystemUiOverlayStyle copyWith({
    Color? systemNavigationBarColor,
    Color? systemNavigationBarDividerColor,
    bool? systemNavigationBarContrastEnforced,
    Color? statusBarColor,
    Brightness? statusBarBrightness,
    Brightness? statusBarIconBrightness,
    bool? systemStatusBarContrastEnforced,
    Brightness? systemNavigationBarIconBrightness,
  }) {
    return SystemUiOverlayStyle(
      systemNavigationBarColor: systemNavigationBarColor ?? this.systemNavigationBarColor,
      systemNavigationBarDividerColor:
          systemNavigationBarDividerColor ?? this.systemNavigationBarDividerColor,
      systemNavigationBarContrastEnforced:
          systemNavigationBarContrastEnforced ?? this.systemNavigationBarContrastEnforced,
      statusBarColor: statusBarColor ?? this.statusBarColor,
      statusBarIconBrightness: statusBarIconBrightness ?? this.statusBarIconBrightness,
      statusBarBrightness: statusBarBrightness ?? this.statusBarBrightness,
      systemStatusBarContrastEnforced:
          systemStatusBarContrastEnforced ?? this.systemStatusBarContrastEnforced,
      systemNavigationBarIconBrightness:
          systemNavigationBarIconBrightness ?? this.systemNavigationBarIconBrightness,
    );
  }

  @override
  int get hashCode => Object.hash(
    systemNavigationBarColor,
    systemNavigationBarDividerColor,
    systemNavigationBarContrastEnforced,
    statusBarColor,
    statusBarBrightness,
    statusBarIconBrightness,
    systemStatusBarContrastEnforced,
    systemNavigationBarIconBrightness,
  );

  @override
  bool operator ==(Object other) {
    if (other.runtimeType != runtimeType) {
      return false;
    }
    return other is SystemUiOverlayStyle &&
        other.systemNavigationBarColor == systemNavigationBarColor &&
        other.systemNavigationBarDividerColor == systemNavigationBarDividerColor &&
        other.systemNavigationBarContrastEnforced == systemNavigationBarContrastEnforced &&
        other.statusBarColor == statusBarColor &&
        other.statusBarIconBrightness == statusBarIconBrightness &&
        other.statusBarBrightness == statusBarBrightness &&
        other.systemStatusBarContrastEnforced == systemStatusBarContrastEnforced &&
        other.systemNavigationBarIconBrightness == systemNavigationBarIconBrightness;
  }
}

List<String> _stringify(List<dynamic> list) => <String>[
  for (final dynamic item in list) item.toString(),
];

/// Controls specific aspects of the operating system's graphical interface and
/// how it interacts with the application.
abstract final class SystemChrome {
  /// Specifies the set of orientations the application interface can
  /// be displayed in.
  ///
  /// The `orientation` argument is a list of [DeviceOrientation] enum values.
  /// The empty list causes the application to defer to the operating system
  /// default.
  ///
  /// ## Limitations
  ///
  /// ### Android
  ///
  /// Android screens may choose to [letterbox](https://developer.android.com/guide/practices/enhanced-letterboxing)
  /// applications that lock orientation, particularly on larger screens. When
  /// letterboxing occurs on Android, the [MediaQueryData.size] reports the
  /// letterboxed size, not the full screen size. Applications that make
  /// decisions about whether to lock orientation based on the screen size
  /// must use the `display` property of the current [FlutterView].
  ///
  /// ```dart
  /// // A widget that locks the screen to portrait if it is less than 600
  /// // logical pixels wide.
  /// class MyApp extends StatefulWidget {
  ///   const MyApp({ super.key });
  ///
  ///   @override
  ///   State<MyApp> createState() => _MyAppState();
  /// }
  ///
  /// class _MyAppState extends State<MyApp> with WidgetsBindingObserver {
  ///   ui.FlutterView? _view;
  ///   static const double kOrientationLockBreakpoint = 600;
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
  ///     _view = View.maybeOf(context);
  ///   }
  ///
  ///   @override
  ///   void dispose() {
  ///     WidgetsBinding.instance.removeObserver(this);
  ///     _view = null;
  ///     super.dispose();
  ///   }
  ///
  ///   @override
  ///   void didChangeMetrics() {
  ///     final ui.Display? display = _view?.display;
  ///     if (display == null) {
  ///       return;
  ///     }
  ///     if (display.size.width / display.devicePixelRatio < kOrientationLockBreakpoint) {
  ///       SystemChrome.setPreferredOrientations(<DeviceOrientation>[
  ///         DeviceOrientation.portraitUp,
  ///       ]);
  ///     } else {
  ///       SystemChrome.setPreferredOrientations(<DeviceOrientation>[]);
  ///     }
  ///   }
  ///
  ///   @override
  ///   Widget build(BuildContext context) {
  ///     return const MaterialApp(
  ///       home: Placeholder(),
  ///     );
  ///   }
  /// }
  /// ```
  ///
  /// ### iOS
  ///
  /// This setting will only be respected on iPad if multitasking is disabled.
  ///
  /// You can decide to opt out of multitasking on iPad, then
  /// setPreferredOrientations will work but your app will not
  /// support Slide Over and Split View multitasking anymore.
  ///
  /// Should you decide to opt out of multitasking you can do this by
  /// setting "Requires full screen" to true in the Xcode Deployment Info.
  static Future<void> setPreferredOrientations(List<DeviceOrientation> orientations) async {
    await SystemChannels.platform.invokeMethod<void>(
      'SystemChrome.setPreferredOrientations',
      _stringify(orientations),
    );
  }

  /// Specifies the description of the current state of the application as it
  /// pertains to the application switcher (also known as "recent tasks").
  ///
  /// Any part of the description that is unsupported on the current platform
  /// will be ignored.
  static Future<void> setApplicationSwitcherDescription(
    ApplicationSwitcherDescription description,
  ) async {
    await SystemChannels.platform.invokeMethod<void>(
      'SystemChrome.setApplicationSwitcherDescription',
      <String, dynamic>{'label': description.label, 'primaryColor': description.primaryColor},
    );
  }

  /// Specifies the [SystemUiMode] to have visible when the application
  /// is running.
  ///
  /// The `overlays` argument is a list of [SystemUiOverlay] enum values
  /// denoting the overlays to show when configured with [SystemUiMode.manual].
  ///
  /// If a particular mode is unsupported on the platform, enabling or
  /// disabling that mode will be ignored.
  ///
  /// The settings here can be overridden by the platform when System UI becomes
  /// necessary for functionality.
  ///
  /// For example, on Android, when the keyboard becomes visible, it will enable the
  /// navigation bar and status bar system UI overlays. When the keyboard is closed,
  /// Android will not restore the previous UI visibility settings, and the UI
  /// visibility cannot be changed until 1 second after the keyboard is closed to
  /// prevent malware locking users from navigation buttons.
  ///
  /// To regain "fullscreen" after text entry, the UI overlays can be set again
  /// after a delay of at least 1 second through [restoreSystemUIOverlays] or
  /// calling this again. Otherwise, the original UI overlay settings will be
  /// automatically restored only when the application loses and regains focus.
  ///
  /// Alternatively, a [SystemUiChangeCallback] can be provided to respond to
  /// changes in the System UI. This will be called, for example, when in
  /// [SystemUiMode.leanBack] and the user taps the screen to bring up the
  /// system overlays. The callback provides a boolean to represent if the
  /// application is currently in a fullscreen mode or not, so that the
  /// application can respond to these changes. When `systemOverlaysAreVisible`
  /// is true, the application is not fullscreen. See
  /// [SystemChrome.setSystemUIChangeCallback] to respond to these changes in a
  /// fullscreen application.
  ///
  /// If your app targets Android SDK 15 (API 35) or later (Flutter does this by
  /// default), then your Flutter app uses [SystemUiMode.edgeToEdge] by default
  /// on Android and setting any of the other [SystemUiMode]s will NOT work
  /// unless you perform the migration detailed in
  /// https://docs.flutter.dev/release/breaking-changes/default-systemuimode-edge-to-edge.
  static Future<void> setEnabledSystemUIMode(
    SystemUiMode mode, {
    List<SystemUiOverlay>? overlays,
  }) async {
    if (mode != SystemUiMode.manual) {
      await SystemChannels.platform.invokeMethod<void>(
        'SystemChrome.setEnabledSystemUIMode',
        mode.toString(),
      );
    } else {
      assert(mode == SystemUiMode.manual && overlays != null);
      await SystemChannels.platform.invokeMethod<void>(
        'SystemChrome.setEnabledSystemUIOverlays',
        _stringify(overlays!),
      );
    }
  }

  /// Sets the callback method for responding to changes in the system UI.
  ///
  /// This is relevant when using [SystemUiMode.leanBack]
  /// and [SystemUiMode.immersive] and [SystemUiMode.immersiveSticky] on Android
  /// platforms, where the [SystemUiOverlay]s can appear and disappear based on
  /// user interaction.
  ///
  /// This will be called, for example, when in [SystemUiMode.leanBack] and the
  /// user taps the screen to bring up the system overlays. The callback
  /// provides a boolean to represent if the application is currently in a
  /// fullscreen mode or not, so that the application can respond to these
  /// changes. When `systemOverlaysAreVisible` is true, the application is not
  /// fullscreen.
  ///
  /// When using [SystemUiMode.edgeToEdge], system overlays are always visible
  /// and do not change. When manually configuring [SystemUiOverlay]s with
  /// [SystemUiMode.manual], this callback will only be triggered when all
  /// overlays have been disabled. This results in the same behavior as
  /// [SystemUiMode.leanBack].
  ///
  static Future<void> setSystemUIChangeCallback(SystemUiChangeCallback? callback) async {
    ServicesBinding.instance.setSystemUiChangeCallback(callback);
    // Skip setting up the listener if there is no callback.
    if (callback != null) {
      await SystemChannels.platform.invokeMethod<void>('SystemChrome.setSystemUIChangeListener');
    }
  }

  /// Restores the system overlays to the last settings provided via
  /// [setEnabledSystemUIMode]. May be used when the platform force enables/disables
  /// UI elements.
  ///
  /// For example, when the Android keyboard disables hidden status and navigation bars,
  /// this can be called to re-disable the bars when the keyboard is closed.
  ///
  /// On Android, the system UI cannot be changed until 1 second after the previous
  /// change. This is to prevent malware from permanently hiding navigation buttons.
  static Future<void> restoreSystemUIOverlays() async {
    await SystemChannels.platform.invokeMethod<void>('SystemChrome.restoreSystemUIOverlays');
  }

  /// Specifies the style to use for the system overlays (e.g. the status bar on
  /// Android or iOS, the system navigation bar on Android) that are visible (if any).
  ///
  /// This method will schedule the embedder update to be run in a microtask.
  /// Any subsequent calls to this method during the current event loop will
  /// overwrite the pending value, such that only the last specified value takes
  /// effect.
  ///
  /// Call this API in code whose lifecycle matches that of the desired
  /// system UI styles. For instance, to change the system UI style on a new
  /// page, consider calling when pushing/popping a new [PageRoute].
  ///
  /// The [AppBar] widget automatically sets the system overlay style based on
  /// its [AppBar.systemOverlayStyle], so configure that instead of calling this
  /// method directly. Likewise, do the same for [CupertinoNavigationBar] via
  /// [CupertinoNavigationBar.backgroundColor].
  ///
  /// If a particular style is not supported on the platform, selecting it will
  /// have no effect.
  ///
  /// {@tool sample}
  /// The following example uses an `AppBar` to set the system status bar color and
  /// the system navigation bar color.
  ///
  /// ** See code in examples/api/lib/services/system_chrome/system_chrome.set_system_u_i_overlay_style.0.dart **
  /// {@end-tool}
  ///
  /// For more complex control of the system overlay styles, consider using
  /// an [AnnotatedRegion] widget instead of calling [setSystemUIOverlayStyle]
  /// directly. This widget places a value directly into the layer tree where
  /// it can be hit-tested by the framework. On every frame, the framework will
  /// hit-test and select the annotated region it finds under the status and
  /// navigation bar and synthesize them into a single style. This can be used
  /// to configure the system styles when an app bar is not used. When an app
  /// bar is used, apps should not enclose the app bar in an annotated region
  /// because one is automatically created. If an app bar is used and the app
  /// bar is enclosed in an annotated region, the app bar overlay style supersedes
  /// the status bar properties defined in the enclosing annotated region overlay
  /// style and the enclosing annotated region overlay style supersedes the app bar
  /// overlay style navigation bar properties.
  ///
  /// {@tool sample}
  /// The following example uses an `AnnotatedRegion<SystemUiOverlayStyle>` to set
  /// the system status bar color and the system navigation bar color.
  ///
  /// ** See code in examples/api/lib/services/system_chrome/system_chrome.set_system_u_i_overlay_style.1.dart **
  /// {@end-tool}
  ///
  /// See also:
  ///
  ///  * [AppBar.systemOverlayStyle], a convenient property for declaratively setting
  ///    the style of the system overlays.
  ///  * [AnnotatedRegion], the widget used to place a `SystemUiOverlayStyle` into
  ///    the layer tree.
  static void setSystemUIOverlayStyle(SystemUiOverlayStyle style) {
    if (_pendingStyle != null) {
      // The microtask has already been queued; just update the pending value.
      _pendingStyle = style;
      return;
    }
    if (style == _latestStyle) {
      // Trivial success: no microtask has been queued and the given style is
      // already in effect, so no need to queue a microtask.
      return;
    }
    _pendingStyle = style;
    scheduleMicrotask(() {
      assert(_pendingStyle != null);
      if (_pendingStyle != _latestStyle) {
        SystemChannels.platform.invokeMethod<void>(
          'SystemChrome.setSystemUIOverlayStyle',
          _pendingStyle!._toMap(),
        );
        _latestStyle = _pendingStyle;
      }
      _pendingStyle = null;
    });
  }

  /// Called by the binding during a transition to a new app lifecycle state.
  static void handleAppLifecycleStateChanged(AppLifecycleState state) {
    // When the app is detached, clear the record of the style sent to the host
    // so that it will be sent again when the app is reattached.
    if (state == AppLifecycleState.detached) {
      scheduleMicrotask(() {
        _latestStyle = null;
      });
    }
  }

  static SystemUiOverlayStyle? _pendingStyle;

  /// The last style that was set using [SystemChrome.setSystemUIOverlayStyle].
  @visibleForTesting
  static SystemUiOverlayStyle? get latestStyle => _latestStyle;
  static SystemUiOverlayStyle? _latestStyle;
}
