// Copyright 2013 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

part of dart.ui;

@pragma('vm:entry-point')
void _setupHooks() {
  assert(() {
    // In debug mode, register the schedule frame extension.
    developer.registerExtension('ext.ui.window.scheduleFrame', _scheduleFrame);

    // In debug mode, allow shaders to be reinitialized.
    developer.registerExtension('ext.ui.window.reinitializeShader', _reinitializeShader);

    return true;
  }());

  // In debug and profile mode, allow tools to display the current rendering backend.
  if (!_kReleaseMode) {
    developer.registerExtension('ext.ui.window.impellerEnabled', _getImpellerEnabled);
  }
}
