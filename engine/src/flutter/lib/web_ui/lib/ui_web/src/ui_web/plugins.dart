// Copyright 2013 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

import 'package:ui/src/engine.dart';
import 'package:ui/ui.dart' as ui;

/// Sets the handler that forwards platform messages to web plugins.
///
/// This function exists because unlike mobile, on the web plugins are also
/// implemented using Dart code, and that code needs a way to receive messages.
void setPluginHandler(ui.PlatformMessageCallback handler) {
  pluginMessageCallHandler = handler;
}
