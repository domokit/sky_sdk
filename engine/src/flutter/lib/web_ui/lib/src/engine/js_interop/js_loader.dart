// Copyright 2013 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

@JS()
library js_loader;

import 'dart:js_interop';

import 'package:js/js_util.dart' as js_util;
import 'package:ui/src/engine.dart';

@JS()
@staticInterop
class FlutterJS {}

extension FlutterJSExtension on FlutterJS {
  external FlutterLoader? get loader;
}

// Both `flutter`, `loader`(_flutter.loader), must be checked for null before
// `didCreateEngineInitializer` can be safely accessed.
@JS('_flutter')
external FlutterJS? get flutter;

@JS()
@staticInterop
class FlutterLoader {}

extension FlutterLoaderExtension on FlutterLoader {
  external void didCreateEngineInitializer(FlutterEngineInitializer initializer);
  bool get isAutoStart => !js_util.hasProperty(this, 'didCreateEngineInitializer');
}

/// Typedef for the function that initializes the flutter engine.
/// ///
/// [JsFlutterConfiguration] comes from `../configuration.dart`. It is the same
/// object that can be used to configure flutter "inline", through the
/// (to be deprecated) `window.flutterConfiguration` object.
typedef InitializeEngineFn = Future<FlutterAppRunner> Function([JsFlutterConfiguration?]);

/// Typedef for the `autoStart` function that can be called straight from an engine initializer instance.
/// (Similar to [RunAppFn], but taking no specific "runApp" parameters).
typedef ImmediateRunAppFn = Future<FlutterApp> Function();

// FlutterEngineInitializer

/// An object that allows the user to initialize the Engine of a Flutter App.
///
/// As a convenience method, [autoStart] allows the user to immediately initialize
/// and run a Flutter Web app, from JavaScript.
@JS()
@anonymous
@staticInterop
abstract class FlutterEngineInitializer {
  factory FlutterEngineInitializer({
    required InitializeEngineFn initializeEngine,
    required ImmediateRunAppFn autoStart,
  }) => FlutterEngineInitializer._(
    initializeEngine:
        (([JsFlutterConfiguration? config]) =>
                (initializeEngine(config) as Future<JSObject>).toPromise)
            .toJS,
    autoStart: (() => (autoStart() as Future<JSObject>).toPromise).toJS,
  );
  external factory FlutterEngineInitializer._({
    required JSFunction initializeEngine,
    required JSFunction autoStart,
  });
}

// FlutterAppRunner

/// A class that exposes a function that runs the Flutter app,
/// and returns a promise of a FlutterAppCleaner.
@JS()
@anonymous
@staticInterop
abstract class FlutterAppRunner {
  factory FlutterAppRunner({required RunAppFn runApp}) => FlutterAppRunner._(
    runApp: (([RunAppFnParameters? args]) => (runApp(args) as Future<JSObject>).toPromise).toJS,
  );

  /// Runs a flutter app
  external factory FlutterAppRunner._({
    required JSFunction runApp, // Returns an App
  });
}

/// The shape of the object that can be passed as parameter to the
/// runApp function of the FlutterAppRunner object (from JS).
@JS()
@anonymous
@staticInterop
abstract class RunAppFnParameters {}

/// Typedef for the function that runs the flutter app main entrypoint.
typedef RunAppFn = Future<FlutterApp> Function([RunAppFnParameters?]);
