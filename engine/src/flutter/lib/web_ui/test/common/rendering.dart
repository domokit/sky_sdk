// Copyright 2013 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

import 'dart:async';

import 'package:ui/src/engine.dart';
import 'package:ui/ui.dart' as ui;

// The scene that will be rendered in the next call to `onDrawFrame`.
ui.Scene? _sceneToRender;

// Completer that will complete when the call to render completes.
Completer<void>? _sceneCompleter;

/// Sets up rendering so that `onDrawFrame` will render the last requested
/// scene.
void setUpRenderingForTests() {
  // Set `onDrawFrame` to call `renderer.renderScene`.
  EnginePlatformDispatcher.instance.onDrawFrame = () {
    if (_sceneToRender != null) {
      EnginePlatformDispatcher.instance
          .render(_sceneToRender!)
          .then<void>((_) {
            _sceneCompleter?.complete();
          })
          .catchError((Object error) {
            _sceneCompleter?.completeError(error);
          });
      _sceneToRender = null;
    }
  };
}

/// Render the given [scene] in an `onDrawFrame` scope.
Future<void> renderScene(ui.Scene scene) {
  _sceneToRender = scene;
  _sceneCompleter = Completer<void>();
  EnginePlatformDispatcher.instance.invokeOnDrawFrame();
  return _sceneCompleter!.future;
}
