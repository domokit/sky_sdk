// Copyright 2014 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

// This example shows how to put some pixels on the screen using the raw
// interface to the engine.

import 'dart:typed_data';
import 'dart:ui' as ui;

// The FlutterView into which this example will draw; set in the main method.
late final ui.FlutterView view;

late ui.Color color;

ui.Picture paint(ui.Rect paintBounds) {
  // First we create a PictureRecorder to record the commands we're going to
  // feed in the canvas. The PictureRecorder will eventually produce a Picture,
  // which is an immutable record of those commands.
  final ui.PictureRecorder recorder = ui.PictureRecorder();

  // Next, we create a canvas from the recorder. The canvas is an interface
  // which can receive drawing commands. The canvas interface is modeled after
  // the SkCanvas interface from Skia. The paintBounds establishes a "cull rect"
  // for the canvas, which lets the implementation discard any commands that
  // are entirely outside this rectangle.
  final ui.Canvas canvas = ui.Canvas(recorder, paintBounds);

  // The commands draw a circle in the center of the screen.
  final ui.Size size = paintBounds.size;
  canvas.drawCircle(
    size.center(ui.Offset.zero),
    size.shortestSide * 0.45,
    ui.Paint()..color = color,
  );

  // When we're done issuing painting commands, we end the recording an receive
  // a Picture, which is an immutable record of the commands we've issued. You
  // can draw a Picture into another canvas or include it as part of a
  // composited scene.
  return recorder.endRecording();
}

ui.Scene composite(ui.Picture picture, ui.Rect paintBounds) {
  // The device pixel ratio gives an approximate ratio of the size of pixels on
  // the device's screen to "normal" sized pixels. We commonly work in logical
  // pixels, which are then scaled by the device pixel ratio before being drawn
  // on the screen.
  final double devicePixelRatio = view.devicePixelRatio;

  // This transform scales the x and y coordinates by the devicePixelRatio.
  final Float64List deviceTransform =
      Float64List(16)
        ..[0] = devicePixelRatio
        ..[5] = devicePixelRatio
        ..[10] = 1.0
        ..[15] = 1.0;

  // We build a very simple scene graph with two nodes. The root node is a
  // transform that scale its children by the device pixel ratio. This transform
  // lets us paint in "logical" pixels which are converted to device pixels by
  // this scaling operation.
  final ui.SceneBuilder sceneBuilder =
      ui.SceneBuilder()
        ..pushTransform(deviceTransform)
        ..addPicture(ui.Offset.zero, picture)
        ..pop();

  // When we're done recording the scene, we call build() to obtain an immutable
  // record of the scene we've recorded.
  return sceneBuilder.build();
}

void beginFrame(Duration timeStamp) {
  final ui.Rect paintBounds = ui.Offset.zero & (view.physicalSize / view.devicePixelRatio);
  // First, record a picture with our painting commands.
  final ui.Picture picture = paint(paintBounds);
  // Second, include that picture in a scene graph.
  final ui.Scene scene = composite(picture, paintBounds);
  // Third, instruct the engine to render that scene graph.
  view.render(scene);
}

void handlePointerDataPacket(ui.PointerDataPacket packet) {
  // The pointer packet contains a number of pointer movements, which we iterate
  // through and process.
  for (final ui.PointerData datum in packet.data) {
    if (datum.change == ui.PointerChange.down) {
      // If the pointer went down, we change the color of the circle to blue.
      color = const ui.Color(0xFF0000FF);
      // Rather than calling paint() synchronously, we ask the engine to
      // schedule a frame. The engine will call onBeginFrame when it is actually
      // time to produce the frame.
      ui.PlatformDispatcher.instance.scheduleFrame();
    } else if (datum.change == ui.PointerChange.up) {
      // Similarly, if the pointer went up, we change the color of the circle to
      // green and schedule a frame. It's harmless to call scheduleFrame many
      // times because the engine will ignore redundant requests up until the
      // point where the engine calls onBeginFrame, which signals the boundary
      // between one frame and another.
      color = const ui.Color(0xFF00FF00);
      ui.PlatformDispatcher.instance.scheduleFrame();
    }
  }
}

// This function is the primary entry point to your application. The engine
// calls main() as soon as it has loaded your code.
void main() {
  // TODO(goderbauer): Create a window if embedder doesn't provide an implicit view to draw into.
  assert(ui.PlatformDispatcher.instance.implicitView != null);
  view = ui.PlatformDispatcher.instance.implicitView!;

  color = const ui.Color(0xFF00FF00);
  // The engine calls onBeginFrame whenever it wants us to produce a frame.
  ui.PlatformDispatcher.instance.onBeginFrame = beginFrame;
  // The engine calls onPointerDataPacket whenever it had updated information
  // about the pointers directed at our app.
  ui.PlatformDispatcher.instance.onPointerDataPacket = handlePointerDataPacket;
  // Here we kick off the whole process by asking the engine to schedule a new
  // frame. The engine will eventually call onBeginFrame when it is time for us
  // to actually produce the frame.
  ui.PlatformDispatcher.instance.scheduleFrame();
}
