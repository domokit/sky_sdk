// Copyright 2013 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

import 'dart:async';
import 'dart:developer' as developer;

import 'package:ui/src/engine.dart';
import 'package:ui/ui.dart' as ui;
import 'package:ui/ui_web/src/ui_web.dart' as ui_web;
import 'package:web_test_fonts/web_test_fonts.dart';

/// The mode the app is running in.
/// Keep these in sync with the same constants on the framework-side under foundation/constants.dart.
const bool kReleaseMode = bool.fromEnvironment('dart.vm.product');

/// A constant that is true if the application was compiled in profile mode.
const bool kProfileMode = bool.fromEnvironment('dart.vm.profile');

/// A constant that is true if the application was compiled in debug mode.
const bool kDebugMode = !kReleaseMode && !kProfileMode;

/// Returns mode of the app is running in as a string.
String get buildMode =>
    kReleaseMode
        ? 'release'
        : kProfileMode
        ? 'profile'
        : 'debug';

/// A benchmark metric that includes frame-related computations prior to
/// submitting layer and picture operations to the underlying renderer, such as
/// HTML and CanvasKit. During this phase we compute transforms, clips, and
/// other information needed for rendering.
const String kProfilePrerollFrame = 'preroll_frame';

/// A benchmark metric that includes submitting layer and picture information
/// to the renderer.
const String kProfileApplyFrame = 'apply_frame';

final List<ui.VoidCallback> _hotRestartListeners = <ui.VoidCallback>[];

/// Requests that [listener] is called just before hot restarting the app.
void registerHotRestartListener(ui.VoidCallback listener) {
  _hotRestartListeners.add(listener);
}

/// Pretends that hot restart is about to happen.
///
/// Useful in tests to check that the engine performs appropriate clean-ups,
/// such as removing static DOM listeners, prior to allowing the Dart runtime
/// to re-initialize the program.
void debugEmulateHotRestart() {
  // While hot restart listeners are executing, more listeners may be added. To
  // avoid concurrent modification, the listeners are copies and emptied. If new
  // listeners are added in the process, the loop will pick them up.
  while (_hotRestartListeners.isNotEmpty) {
    final List<ui.VoidCallback> copyOfListeners = _hotRestartListeners.toList();
    _hotRestartListeners.clear();
    for (final ui.VoidCallback listener in copyOfListeners) {
      listener();
    }
  }
}

/// How far along the initialization process the engine is currently is.
///
/// The initialization process starts with [none] and proceeds in increasing
/// `index` number until [initialized].
enum DebugEngineInitializationState {
  /// Initialization hasn't started yet.
  uninitialized,

  /// The engine is initializing its non-UI services.
  initializingServices,

  /// The engine has initialized its non-UI services, but hasn't started
  /// initializing the UI.
  initializedServices,

  /// The engine started attaching UI surfaces to the web page.
  initializingUi,

  /// The engine has fully completed initialization.
  ///
  /// At this point the framework can start using the engine for I/O, rendering,
  /// etc.
  ///
  /// This is the final state of the engine.
  initialized,
}

/// The current initialization state of the engine.
///
/// See [DebugEngineInitializationState] for possible states.
DebugEngineInitializationState get initializationState => _initializationState;
DebugEngineInitializationState _initializationState = DebugEngineInitializationState.uninitialized;

/// Resets the state back to [DebugEngineInitializationState.uninitialized].
///
/// This is for testing only.
void debugResetEngineInitializationState() {
  _initializationState = DebugEngineInitializationState.uninitialized;
}

/// Initializes non-UI engine services.
///
/// Does not put any UI onto the page. It is therefore safe to call this
/// function while the page is showing non-Flutter UI, such as a loading
/// indicator, a splash screen, or in an add-to-app scenario where the host page
/// is written using a different web framework.
///
/// See also:
///
///  * [initializeEngineUi], which is typically called after this function, and
///    puts UI elements on the page.
Future<void> initializeEngineServices({
  ui_web.AssetManager? assetManager,
  JsFlutterConfiguration? jsConfiguration,
}) async {
  if (_initializationState != DebugEngineInitializationState.uninitialized) {
    assert(() {
      throw StateError(
        'Invalid engine initialization state. `initializeEngineServices` was '
        'called, but the engine has already started initialization and is '
        'currently in state "$_initializationState".',
      );
    }());
    return;
  }
  _initializationState = DebugEngineInitializationState.initializingServices;

  // Store `jsConfiguration` so user settings are available to the engine.
  configuration.setUserConfiguration(jsConfiguration);

  // Called by the Web runtime just before hot restarting the app.
  //
  // This extension cleans up resources that are registered with browser's
  // global singletons that Dart compiler is unable to clean-up automatically.
  //
  // This extension does not need to clean-up Dart statics. Those are cleaned
  // up by the compiler.
  developer.registerExtension('ext.flutter.disassemble', (_, _) {
    for (final ui.VoidCallback listener in _hotRestartListeners) {
      listener();
    }
    return Future<developer.ServiceExtensionResponse>.value(
      developer.ServiceExtensionResponse.result('OK'),
    );
  });

  if (Profiler.isBenchmarkMode) {
    Profiler.ensureInitialized();
  }

  assetManager ??= ui_web.AssetManager(assetBase: configuration.assetBase);
  _setAssetManager(assetManager);

  Future<void> initializeRendererCallback() async => renderer.initialize();
  await Future.wait<void>(<Future<void>>[initializeRendererCallback(), _downloadAssetFonts()]);
  _initializationState = DebugEngineInitializationState.initializedServices;
}

/// Initializes the UI surfaces for the Flutter framework to render to.
///
/// Must be called after [initializeEngineServices].
///
/// This function will start altering the HTML structure of the page. If used
/// in an add-to-app scenario, the host page is expected to prepare for Flutter
/// UI appearing on screen prior to calling this function.
Future<void> initializeEngineUi() async {
  if (_initializationState != DebugEngineInitializationState.initializedServices) {
    assert(() {
      throw StateError(
        'Invalid engine initialization state. `initializeEngineUi` was '
        'called while the engine initialization state was '
        '"$_initializationState". `initializeEngineUi` can only be called '
        'when the engine is in state '
        '"${DebugEngineInitializationState.initializedServices}".',
      );
    }());
    return;
  }
  _initializationState = DebugEngineInitializationState.initializingUi;

  RawKeyboard.initialize(onMacOs: ui_web.browser.operatingSystem == ui_web.OperatingSystem.macOs);
  KeyboardBinding.initInstance();

  // Ensures Flutter renders a global "generator" meta-tag.
  ensureMetaTag('generator', 'Flutter');

  if (!configuration.multiViewEnabled) {
    ensureImplicitViewInitialized(hostElement: configuration.hostElement);
  }
  _initializationState = DebugEngineInitializationState.initialized;
}

ui_web.AssetManager get engineAssetManager => _debugAssetManager ?? _assetManager!;
ui_web.AssetManager? _assetManager;
ui_web.AssetManager? _debugAssetManager;

set debugOnlyAssetManager(ui_web.AssetManager? manager) => _debugAssetManager = manager;

void _setAssetManager(ui_web.AssetManager assetManager) {
  if (assetManager == _assetManager) {
    return;
  }

  _assetManager = assetManager;
}

Future<void> _downloadAssetFonts() async {
  renderer.fontCollection.clear();

  if (ui_web.debugEmulateFlutterTesterEnvironment) {
    // Load the embedded test font before loading fonts from the assets so that
    // the embedded test font is the default (first) font.
    await renderer.fontCollection.loadFontFromList(
      EmbeddedTestFont.flutterTest.data,
      fontFamily: EmbeddedTestFont.flutterTest.fontFamily,
    );
  }

  if (_debugAssetManager != null || _assetManager != null) {
    await renderer.fontCollection.loadAssetFonts(await fetchFontManifest(ui_web.assetManager));
  }
}

/// Whether to disable the font fallback system.
///
/// We need to disable font fallbacks for some framework tests because
/// Flutter error messages may contain an arrow symbol which is not
/// covered by ASCII fonts. This causes us to try to download the
/// Noto Sans Symbols font, which kicks off a `Timer` which doesn't
/// complete before the Widget tree is disposed (this is by design).
bool get debugDisableFontFallbacks => _debugDisableFontFallbacks;
set debugDisableFontFallbacks(bool value) {
  _debugDisableFontFallbacks = value;
}

bool _debugDisableFontFallbacks = false;
