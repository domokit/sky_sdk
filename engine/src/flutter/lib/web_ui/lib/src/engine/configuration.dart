// Copyright 2013 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

/// JavaScript API a Flutter Web application can use to configure the Web
/// Engine.
///
/// The configuration is passed from JavaScript to the engine as part of the
/// bootstrap process, through the `FlutterEngineInitializer.initializeEngine`
/// JS method, with an (optional) object of type [JsFlutterConfiguration].
///
/// This library also supports the legacy method of setting a plain JavaScript
/// object set as the `flutterConfiguration` property of the top-level `window`
/// object, but that approach is now deprecated and will warn users.
///
/// Both methods are **disallowed** to be used at the same time.
///
/// Example:
///
///     _flutter.loader.loadEntrypoint({
///       // ...
///       onEntrypointLoaded: async function(engineInitializer) {
///         let appRunner = await engineInitializer.initializeEngine({
///           // JsFlutterConfiguration goes here...
///           canvasKitBaseUrl: "https://example.com/my-custom-canvaskit/",
///         });
///         appRunner.runApp();
///       }
///     });
///
/// Example of the **deprecated** style (this will issue a JS console warning!):
///
///     <script>
///       window.flutterConfiguration = {
///         canvasKitBaseUrl: "https://example.com/my-custom-canvaskit/"
///       };
///     </script>
///
/// Configuration properties supplied via this object override those supplied
/// using the corresponding environment variables. For example, if both the
/// `canvasKitBaseUrl` config entry and the `FLUTTER_WEB_CANVASKIT_URL`
/// environment variables are provided, the `canvasKitBaseUrl` entry is used.

@JS()
library configuration;

import 'dart:js_interop';

import 'package:meta/meta.dart';
import 'canvaskit/renderer.dart';
import 'dom.dart';

/// The Web Engine configuration for the current application.
FlutterConfiguration get configuration {
  if (_debugConfiguration != null) {
    return _debugConfiguration!;
  }
  return _configuration ??= FlutterConfiguration.legacy(_jsConfiguration);
}

FlutterConfiguration? _configuration;

FlutterConfiguration? _debugConfiguration;

/// Overrides the initial test configuration with new values coming from `newConfig`.
///
/// The initial test configuration (AKA `_jsConfiguration`) is set in the
/// `test_platform.dart` file. See: `window.flutterConfiguration` in `_testBootstrapHandler`.
///
/// The result of calling this method each time is:
///
///     [configuration] = _jsConfiguration + newConfig
///
/// Subsequent calls to this method don't *add* more to an already overridden
/// configuration; this method always starts from an original `_jsConfiguration`,
/// and adds `newConfig` to it.
///
/// If `newConfig` is null, [configuration] resets to the initial `_jsConfiguration`.
///
/// This must be called before the engine is initialized. Calling it after the
/// engine is initialized will result in some of the properties not taking
/// effect because they are consumed during initialization.
@visibleForTesting
void debugOverrideJsConfiguration(JsFlutterConfiguration? newConfig) {
  if (newConfig != null) {
    _debugConfiguration = configuration.withOverrides(newConfig);
  } else {
    _debugConfiguration = null;
  }
}

/// Supplies Web Engine configuration properties.
class FlutterConfiguration {
  /// Constructs an unitialized configuration object.
  @visibleForTesting
  FlutterConfiguration();

  /// Constucts a "tainted by JS globals" configuration object.
  ///
  /// This configuration style is deprecated. It will warn the user about the
  /// new API (if used)
  FlutterConfiguration.legacy(JsFlutterConfiguration? config) {
    if (config != null) {
      _usedLegacyConfigStyle = true;
      _configuration = config;
    }
    // Warn the user of the deprecated behavior.
    assert(() {
      if (config != null) {
        domWindow.console.warn(
          'window.flutterConfiguration is now deprecated.\n'
          'Use engineInitializer.initializeEngine(config) instead.\n'
          'See: https://docs.flutter.dev/development/platform-integration/web/initialization',
        );
      }
      if (_requestedRendererType != null) {
        domWindow.console.warn(
          'window.flutterWebRenderer is now deprecated.\n'
          'Use engineInitializer.initializeEngine(config) instead.\n'
          'See: https://docs.flutter.dev/development/platform-integration/web/initialization',
        );
      }
      return true;
    }());
  }

  FlutterConfiguration withOverrides(JsFlutterConfiguration? overrides) {
    final JsFlutterConfiguration newJsConfig =
        objectConstructor.assign(
              <String, Object>{}.jsify(),
              _configuration.jsify(),
              overrides.jsify(),
            )
            as JsFlutterConfiguration;
    final FlutterConfiguration newConfig = FlutterConfiguration();
    newConfig._configuration = newJsConfig;
    return newConfig;
  }

  bool _usedLegacyConfigStyle = false;
  JsFlutterConfiguration? _configuration;

  /// Sets a value for [_configuration].
  ///
  /// This method is called by the engine initialization process, through the
  /// [initEngineServices] method.
  ///
  /// This method throws an AssertionError, if the _configuration object has
  /// been set to anything non-null through the [FlutterConfiguration.legacy]
  /// constructor.
  void setUserConfiguration(JsFlutterConfiguration? configuration) {
    if (configuration != null) {
      assert(
        !_usedLegacyConfigStyle,
        'Use engineInitializer.initializeEngine(config) only. '
        'Using the (deprecated) window.flutterConfiguration and initializeEngine '
        'configuration simultaneously is not supported.',
      );
      assert(
        _requestedRendererType == null || configuration.renderer == null,
        'Use engineInitializer.initializeEngine(config) only. '
        'Using the (deprecated) window.flutterWebRenderer and initializeEngine '
        'configuration simultaneously is not supported.',
      );
      _configuration = configuration;
    }
  }

  // Static constant parameters.
  //
  // These properties affect tree shaking and therefore cannot be supplied at
  // runtime. They must be static constants for the compiler to remove dead code
  // effectively.

  /// Auto detect which rendering backend to use.
  ///
  /// Using flutter tools option "--web-renderer=auto" would set the value to
  /// true. Otherwise, it would be false.
  static const bool flutterWebAutoDetect = bool.fromEnvironment('FLUTTER_WEB_AUTO_DETECT');

  static const bool flutterWebUseSkwasm = bool.fromEnvironment('FLUTTER_WEB_USE_SKWASM');

  /// Enable the Skia-based rendering backend.
  ///
  /// Using flutter tools option "--web-renderer=canvaskit" would set the value to
  /// true.
  ///
  /// Using flutter tools option "--web-renderer=html" would set the value to false.
  static const bool useSkia = bool.fromEnvironment('FLUTTER_WEB_USE_SKIA');

  // Runtime parameters.
  //
  // These parameters can be supplied either as environment variables, or at
  // runtime. Runtime-supplied values take precedence over environment
  // variables.

  /// The absolute base URL of the location of the `assets` directory of the app.
  ///
  /// This value is useful when Flutter web assets are deployed to a separate
  /// domain (or subdirectory) from which the index.html is served, for example:
  ///
  /// * Application: https://www.my-app.com/
  /// * Flutter Assets: https://cdn.example.com/my-app/build-hash/assets/
  ///
  /// The `assetBase` value would be set to:
  ///
  /// * `'https://cdn.example.com/my-app/build-hash/'`
  ///
  /// It is also useful in the case that a Flutter web application is embedded
  /// into another web app, in a way that the `<base>` tag of the index.html
  /// cannot be set (because it'd break the host app), for example:
  ///
  /// * Application: https://www.my-app.com/
  /// * Flutter Assets: https://www.my-app.com/static/companion/flutter/assets/
  ///
  /// The `assetBase` would be set to:
  ///
  /// * `'/static/companion/flutter/'`
  ///
  /// Do not confuse this configuration value with [canvasKitBaseUrl].
  String? get assetBase => _configuration?.assetBase;

  /// The base URL to use when downloading the CanvasKit script and associated
  /// wasm.
  ///
  /// The expected directory structure nested under this URL is as follows:
  ///
  ///     /canvaskit.js              - the build of CanvasKit JS API bindings
  ///     /canvaskit.wasm            - the build of CanvasKit WASM module
  ///
  /// The base URL can be overridden using the `FLUTTER_WEB_CANVASKIT_URL`
  /// environment variable or using the configuration API for JavaScript.
  ///
  /// When specifying using the environment variable set it in the Flutter tool
  /// using the `--dart-define` option. The value must end with a `/`.
  ///
  /// Example:
  ///
  /// ```bash
  /// flutter run \
  ///   -d chrome \
  ///   --web-renderer=canvaskit \
  ///   --dart-define=FLUTTER_WEB_CANVASKIT_URL=https://example.com/custom-canvaskit-build/
  /// ```
  String get canvasKitBaseUrl => _configuration?.canvasKitBaseUrl ?? _defaultCanvasKitBaseUrl;
  static const String _defaultCanvasKitBaseUrl = String.fromEnvironment(
    'FLUTTER_WEB_CANVASKIT_URL',
    defaultValue: 'canvaskit/',
  );

  /// The variant of CanvasKit to download.
  ///
  /// Available values are:
  ///
  /// * `auto` - the default value. The engine will automatically detect the
  /// best variant to use based on the browser.
  ///
  /// * `full` - the full variant of CanvasKit that can be used in any browser.
  ///
  /// * `chromium` - the lite variant of CanvasKit that can be used in
  /// Chromium-based browsers.
  CanvasKitVariant get canvasKitVariant {
    final String variant = _configuration?.canvasKitVariant ?? 'auto';
    return CanvasKitVariant.values.byName(variant);
  }

  /// If set to true, forces CPU-only rendering in CanvasKit (i.e. the engine
  /// won't use WebGL).
  ///
  /// This is mainly used for testing or for apps that want to ensure they
  /// run on devices which don't support WebGL.
  bool get canvasKitForceCpuOnly =>
      _configuration?.canvasKitForceCpuOnly ?? _defaultCanvasKitForceCpuOnly;
  static const bool _defaultCanvasKitForceCpuOnly = bool.fromEnvironment(
    'FLUTTER_WEB_CANVASKIT_FORCE_CPU_ONLY',
  );

  /// The maximum number of canvases to use when rendering in CanvasKit.
  ///
  /// Limits the amount of overlays that can be created.
  int get canvasKitMaximumSurfaces {
    final int maxSurfaces = _configuration?.canvasKitMaximumSurfaces?.toInt() ?? 8;
    if (maxSurfaces < 1) {
      return 1;
    }
    return maxSurfaces;
  }

  /// Set this flag to `true` to cause the engine to visualize the semantics tree
  /// on the screen for debugging.
  ///
  /// This only works in profile and release modes. Debug mode does not support
  /// passing compile-time constants.
  ///
  /// Example:
  ///
  /// ```bash
  /// flutter run -d chrome --profile --dart-define=FLUTTER_WEB_DEBUG_SHOW_SEMANTICS=true
  /// ```
  bool get debugShowSemanticsNodes =>
      _configuration?.debugShowSemanticsNodes ?? _defaultDebugShowSemanticsNodes;
  static const bool _defaultDebugShowSemanticsNodes = bool.fromEnvironment(
    'FLUTTER_WEB_DEBUG_SHOW_SEMANTICS',
  );

  /// Returns the [hostElement] in which the Flutter Application is supposed
  /// to render, or `null` if the user hasn't specified anything.
  DomElement? get hostElement => _configuration?.hostElement;

  /// Sets Flutter Web in "multi-view" mode.
  ///
  /// Multi-view mode allows apps to:
  ///
  ///  * Start without a `hostElement`.
  ///  * Add/remove views (`hostElements`) from JS while the application is running.
  ///  * ...
  ///  * PROFIT?
  bool get multiViewEnabled => _configuration?.multiViewEnabled ?? false;

  /// Returns a `nonce` to allowlist the inline styles that Flutter web needs.
  ///
  /// See: https://developer.mozilla.org/en-US/docs/Web/HTML/Global_attributes/nonce
  String? get nonce => _configuration?.nonce;

  /// Returns the [requestedRendererType] to be used with the current Flutter
  /// application, normally 'canvaskit' or 'auto'.
  ///
  /// This value may come from the JS configuration, but also a specific JS value:
  /// `window.flutterWebRenderer`.
  ///
  /// This is used by the Renderer class to decide how to initialize the engine.
  String? get requestedRendererType => _configuration?.renderer ?? _requestedRendererType;

  /// Returns the base URL to load fallback fonts from. Fallback fonts are
  /// downloaded automatically when there is no font bundled with the app that
  /// can show a glyph that is being rendered.
  ///
  /// Defaults to 'https://fonts.gstatic.com/s/'.
  String get fontFallbackBaseUrl =>
      _configuration?.fontFallbackBaseUrl ?? 'https://fonts.gstatic.com/s/';

  bool get forceSingleThreadedSkwasm => _configuration?.forceSingleThreadedSkwasm ?? false;
}

@JS('window.flutterConfiguration')
external JsFlutterConfiguration? get _jsConfiguration;

/// The JS bindings for the object that's set as `window.flutterConfiguration`.
@JS()
@anonymous
@staticInterop
class JsFlutterConfiguration {
  external factory JsFlutterConfiguration();
}

extension JsFlutterConfigurationExtension on JsFlutterConfiguration {
  @JS('assetBase')
  external JSString? get _assetBase;
  String? get assetBase => _assetBase?.toDart;

  @JS('canvasKitBaseUrl')
  external JSString? get _canvasKitBaseUrl;
  String? get canvasKitBaseUrl => _canvasKitBaseUrl?.toDart;

  @JS('canvasKitVariant')
  external JSString? get _canvasKitVariant;
  String? get canvasKitVariant => _canvasKitVariant?.toDart;

  @JS('canvasKitForceCpuOnly')
  external JSBoolean? get _canvasKitForceCpuOnly;
  bool? get canvasKitForceCpuOnly => _canvasKitForceCpuOnly?.toDart;

  @JS('canvasKitMaximumSurfaces')
  external JSNumber? get _canvasKitMaximumSurfaces;
  double? get canvasKitMaximumSurfaces => _canvasKitMaximumSurfaces?.toDartDouble;

  @JS('debugShowSemanticsNodes')
  external JSBoolean? get _debugShowSemanticsNodes;
  bool? get debugShowSemanticsNodes => _debugShowSemanticsNodes?.toDart;

  external DomElement? get hostElement;

  @JS('multiViewEnabled')
  external JSBoolean? get _multiViewEnabled;
  bool? get multiViewEnabled => _multiViewEnabled?.toDart;

  @JS('nonce')
  external JSString? get _nonce;
  String? get nonce => _nonce?.toDart;

  @JS('renderer')
  external JSString? get _renderer;
  String? get renderer => _renderer?.toDart;

  @JS('fontFallbackBaseUrl')
  external JSString? get _fontFallbackBaseUrl;
  String? get fontFallbackBaseUrl => _fontFallbackBaseUrl?.toDart;

  @JS('forceSingleThreadedSkwasm')
  external JSBoolean? get _forceSingleThreadedSkwasm;
  bool? get forceSingleThreadedSkwasm => _forceSingleThreadedSkwasm?.toDart;
}

/// A JavaScript entrypoint that allows developer to set rendering backend
/// at runtime before launching the application.
@JS('window.flutterWebRenderer')
external JSString? get __requestedRendererType;
String? get _requestedRendererType => __requestedRendererType?.toDart;
