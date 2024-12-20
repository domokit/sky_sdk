// Copyright 2014 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

// This example shows how to draw some bi-directional text using the raw
// interface to the engine.

import 'dart:typed_data';
import 'dart:ui' as ui;

// The FlutterView into which this example will draw; set in the main method.
late final ui.FlutterView view;

// A paragraph represents a rectangular region that contains some text.
late ui.Paragraph paragraph;

ui.Picture paint(ui.Rect paintBounds) {
  final ui.PictureRecorder recorder = ui.PictureRecorder();
  final ui.Canvas canvas = ui.Canvas(recorder, paintBounds);

  final double devicePixelRatio = view.devicePixelRatio;
  final ui.Size logicalSize = view.physicalSize / devicePixelRatio;

  canvas.translate(logicalSize.width / 2.0, logicalSize.height / 2.0);
  canvas.drawRect(
    const ui.Rect.fromLTRB(-100.0, -100.0, 100.0, 100.0),
    ui.Paint()..color = const ui.Color.fromARGB(255, 0, 255, 0),
  );

  // The paint method of Paragraph draws the contents of the paragraph onto the
  // given canvas.
  canvas.drawParagraph(
    paragraph,
    ui.Offset(-paragraph.width / 2.0, (paragraph.width / 2.0) - 125.0),
  );

  return recorder.endRecording();
}

ui.Scene composite(ui.Picture picture, ui.Rect paintBounds) {
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
  return sceneBuilder.build();
}

void beginFrame(Duration timeStamp) {
  final ui.Rect paintBounds = ui.Offset.zero & (view.physicalSize / view.devicePixelRatio);
  final ui.Picture picture = paint(paintBounds);
  final ui.Scene scene = composite(picture, paintBounds);
  view.render(scene);
}

void main() {
  // TODO(goderbauer): Create a window if embedder doesn't provide an implicit view to draw into.
  assert(ui.PlatformDispatcher.instance.implicitView != null);
  view = ui.PlatformDispatcher.instance.implicitView!;

  // To create a paragraph of text, we use ParagraphBuilder.
  final ui.ParagraphBuilder builder =
      ui.ParagraphBuilder(
          // The text below has a primary direction of left-to-right.
          // The embedded text has other directions.
          // If this was TextDirection.rtl, the "Hello, world" text would end up on
          // the other side of the right-to-left text.
          ui.ParagraphStyle(textDirection: ui.TextDirection.ltr),
        )
        // We first push a style that turns the text blue.
        ..pushStyle(ui.TextStyle(color: const ui.Color(0xFF0000FF)))
        ..addText('Hello, ')
        // The next run of text will be bold.
        ..pushStyle(ui.TextStyle(fontWeight: ui.FontWeight.bold))
        ..addText('world. ')
        // The pop() command signals the end of the bold styling.
        ..pop()
        // We add text to the paragraph in logical order. The paragraph object
        // understands bi-directional text and will compute the visual ordering
        // during layout.
        ..addText('هذا هو قليلا طويلة من النص الذي يجب التفاف .')
        // The second pop() removes the blue color.
        ..pop()
        // We can add more text with the default styling.
        ..addText(' و أكثر قليلا لجعله أطول. ')
        ..addText('สวัสดี');
  // When we're done adding styles and text, we build the Paragraph object, at
  // which time we can apply styling that affects the entire paragraph, such as
  // left, right, or center alignment. Once built, the contents of the paragraph
  // cannot be altered, but sizing and positioning information can be updated.
  paragraph =
      builder.build()
        // Next, we supply a width that the text is permitted to occupy and we ask
        // the paragraph to the visual position of each its glyphs as well as its
        // overall size, subject to its sizing constraints.
        ..layout(const ui.ParagraphConstraints(width: 180.0));

  // Finally, we register our beginFrame callback and kick off the first frame.
  ui.PlatformDispatcher.instance
    ..onBeginFrame = beginFrame
    ..scheduleFrame();
}
