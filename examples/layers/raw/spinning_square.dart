// Copyright 2014 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

// This example shows how to perform a simple animation using the raw interface
// to the engine.

import 'dart:math' as math;
import 'dart:typed_data';
import 'dart:ui' as ui;

// The FlutterView into which this example will draw; set in the main method.
late final ui.FlutterView view;

void beginFrame(Duration timeStamp) {
  // The timeStamp argument to beginFrame indicates the timing information we
  // should use to clock our animations. It's important to use timeStamp rather
  // than reading the system time because we want all the parts of the system to
  // coordinate the timings of their animations. If each component read the
  // system clock independently, the animations that we processed later would be
  // slightly ahead of the animations we processed earlier.

  // PAINT

  final ui.Rect paintBounds = ui.Offset.zero & (view.physicalSize / view.devicePixelRatio);
  final ui.PictureRecorder recorder = ui.PictureRecorder();
  final ui.Canvas canvas = ui.Canvas(recorder, paintBounds);
  canvas.translate(paintBounds.width / 2.0, paintBounds.height / 2.0);

  // Here we determine the rotation according to the timeStamp given to us by
  // the engine.
  final double t = timeStamp.inMicroseconds / Duration.microsecondsPerMillisecond / 1800.0;
  canvas.rotate(math.pi * (t % 1.0));

  canvas.drawRect(
    const ui.Rect.fromLTRB(-100.0, -100.0, 100.0, 100.0),
    ui.Paint()..color = const ui.Color.fromARGB(255, 0, 255, 0),
  );
  final ui.Picture picture = recorder.endRecording();

  // COMPOSITE

  final double devicePixelRatio = view.devicePixelRatio;
  final Float64List deviceTransform =
      Float64List(16)
        ..[0] = devicePixelRatio
        ..[5] = devicePixelRatio
        ..[10] = 1.0
        ..[15] = 1.0;
  final ui.SceneBuilder sceneBuilder =
      ui.SceneBuilder()
        ..pushTransform(deviceTransform)
        ..addPicture(ui.Offset.zero, picture)
        ..pop();
  view.render(sceneBuilder.build());

  // After rendering the current frame of the animation, we ask the engine to
  // schedule another frame. The engine will call beginFrame again when its time
  // to produce the next frame.
  ui.PlatformDispatcher.instance.scheduleFrame();
}

void main() {
  // TODO(goderbauer): Create a window if embedder doesn't provide an implicit view to draw into.
  assert(ui.PlatformDispatcher.instance.implicitView != null);
  view = ui.PlatformDispatcher.instance.implicitView!;

  ui.PlatformDispatcher.instance
    ..onBeginFrame = beginFrame
    ..scheduleFrame();
}
