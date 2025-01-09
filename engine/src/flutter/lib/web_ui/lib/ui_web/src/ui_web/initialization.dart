// Copyright 2013 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

import 'package:ui/src/engine.dart';
import 'package:ui/ui.dart' as ui;

/// Bootstraps the Flutter Web engine and app.
///
/// If the app uses plugins, then the [registerPlugins] callback can be provided
/// to register those plugins. This is done typically by calling
/// `registerPlugins` from the auto-generated `web_plugin_registrant.dart` file.
///
/// The [runApp] callback is invoked to run the app after the engine is fully
/// initialized.
///
/// For more information, see what the `flutter_tools` does in the entrypoint
/// that it generates around the app's main method:
///
/// * https://github.com/flutter/flutter/blob/95be76ab7e3dca2def54454313e97f94f4ac4582/packages/flutter_tools/lib/src/web/file_generators/main_dart.dart#L14-L43
///
/// By default, engine initialization and app startup occur immediately and back
/// to back. They can be programmatically controlled by setting
/// `FlutterLoader.didCreateEngineInitializer`. For more information, see how
/// `flutter.js` does it:
///
/// * https://github.com/flutter/flutter/blob/95be76ab7e3dca2def54454313e97f94f4ac4582/packages/flutter_tools/lib/src/web/file_generators/js/flutter.js
Future<void> bootstrapEngine({ui.VoidCallback? registerPlugins, ui.VoidCallback? runApp}) async {
  // Create the object that knows how to bootstrap an app from JS and Dart.
  final AppBootstrap bootstrap = AppBootstrap(
    initializeEngine: ([JsFlutterConfiguration? configuration]) async {
      await initializeEngineServices(jsConfiguration: configuration);
    },
    runApp: () async {
      if (registerPlugins != null) {
        registerPlugins();
      }
      await initializeEngineUi();
      if (runApp != null) {
        runApp();
      }
    },
  );

  final FlutterLoader? loader = flutter?.loader;
  if (loader == null || loader.isAutoStart) {
    // The user does not want control of the app, bootstrap immediately.
    await bootstrap.autoStart();
  } else {
    // Yield control of the bootstrap procedure to the user.
    loader.didCreateEngineInitializer(bootstrap.prepareEngineInitializer());
  }
}
