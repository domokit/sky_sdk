// Copyright 2013 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

import 'dart:async';

import 'package:ui/src/engine.dart';
import 'package:ui/src/engine/skwasm/skwasm_impl.dart'
    if (dart.library.html) 'package:ui/src/engine/skwasm/skwasm_stub.dart';
import 'package:ui/ui.dart';

import '../common/rendering.dart';

Picture drawPicture(void Function(Canvas) drawCommands) {
  final PictureRecorder recorder = PictureRecorder();
  final Canvas canvas = Canvas(recorder);
  drawCommands(canvas);
  return recorder.endRecording();
}

/// Draws the [Picture]. This is in preparation for a golden test.
Future<void> drawPictureUsingCurrentRenderer(Picture picture) async {
  final SceneBuilder sb = SceneBuilder();
  sb.pushOffset(0, 0);
  sb.addPicture(Offset.zero, picture);
  await renderScene(sb.build());
}

/// Convenience getter for the implicit view.
FlutterView get implicitView => EnginePlatformDispatcher.instance.implicitView!;

/// Returns [true] if this test is running in the CanvasKit renderer.
bool get isCanvasKit => renderer is CanvasKitRenderer;

bool get isSkwasm => renderer is SkwasmRenderer;

bool get isMultiThreaded => isSkwasm && (renderer as SkwasmRenderer).isMultiThreaded;
