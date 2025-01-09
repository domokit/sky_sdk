// Copyright 2014 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

import 'dart:async';
import 'dart:typed_data';
import 'dart:ui' as ui;

import 'package:flutter_driver/driver_extension.dart';

import 'windows.dart';

void drawHelloWorld(ui.FlutterView view) {
  final ui.ParagraphStyle style = ui.ParagraphStyle();
  final ui.ParagraphBuilder paragraphBuilder = ui.ParagraphBuilder(style)..addText('Hello world');
  final ui.Paragraph paragraph = paragraphBuilder.build();

  paragraph.layout(const ui.ParagraphConstraints(width: 100.0));

  final ui.PictureRecorder recorder = ui.PictureRecorder();
  final ui.Canvas canvas = ui.Canvas(recorder);

  canvas.drawParagraph(paragraph, ui.Offset.zero);

  final ui.Picture picture = recorder.endRecording();
  final ui.SceneBuilder sceneBuilder =
      ui.SceneBuilder()
        ..addPicture(ui.Offset.zero, picture)
        ..pop();

  view.render(sceneBuilder.build());
}

Future<void> _waitUntilWindowVisible() async {
  while (!await isWindowVisible()) {
    await Future<void>.delayed(const Duration(milliseconds: 100));
  }
}

void _expectVisible(bool current, bool expect, Completer<String> completer, int frameCount) {
  if (current != expect) {
    try {
      throw 'Window should be ${expect ? 'visible' : 'hidden'} on frame $frameCount';
    } catch (e) {
      if (!completer.isCompleted) {
        completer.completeError(e);
      }
      rethrow;
    }
  }
}

void main() async {
  // TODO(goderbauer): Create a window if embedder doesn't provide an implicit view to draw into.
  assert(ui.PlatformDispatcher.instance.implicitView != null);
  final ui.FlutterView view = ui.PlatformDispatcher.instance.implicitView!;

  // Create a completer to send the window visibility result back to the
  // integration test.
  final Completer<String> visibilityCompleter = Completer<String>();
  enableFlutterDriverExtension(
    handler: (String? message) async {
      if (message == 'verifyWindowVisibility') {
        return visibilityCompleter.future;
      } else if (message == 'verifyTheme') {
        final bool app = await isAppDarkModeEnabled();
        final bool system = await isSystemDarkModeEnabled();

        return (app == system)
            ? 'success'
            : 'error: app dark mode ($app) does not match system dark mode ($system)';
      } else if (message == 'verifyStringConversion') {
        // Use a test string that contains code points that fit in both 8 and 16 bits.
        // The code points are passed a list of integers through the method channel,
        // which will use the UTF16 to UTF8 utility function to convert them to a
        // std::string, which should equate to the original expected string.
        const String expected = 'ABCℵ';
        final Int32List codePoints = Int32List.fromList(expected.codeUnits);
        final String converted = await testStringConversion(codePoints);
        return (converted == expected)
            ? 'success'
            : 'error: conversion of UTF16 string to UTF8 failed, expected "${expected.codeUnits}" but got "${converted.codeUnits}"';
      }

      throw 'Unrecognized message: $message';
    },
  );

  try {
    if (await isWindowVisible()) {
      throw 'Window should be hidden at startup';
    }

    int frameCount = 0;
    ui.PlatformDispatcher.instance.onBeginFrame = (Duration duration) {
      // Our goal is to verify that it's `drawHelloWorld` that makes the window
      // appear, not anything else. This requires checking the visibility right
      // before drawing, but since `isWindowVisible` has to be async, and
      // `FlutterView.render` (in `drawHelloWorld`) forbids async before it,
      // this can not be done during a single onBeginFrame. However, we can
      // verify in separate frames to indirectly prove it, by ensuring that
      // no other mechanism can affect isWindowVisible in the first frame at all.
      frameCount += 1;
      switch (frameCount) {
        // The 1st frame: render nothing, just verify that the window is hidden.
        case 1:
          isWindowVisible().then((bool visible) {
            _expectVisible(visible, false, visibilityCompleter, frameCount);
            ui.PlatformDispatcher.instance.scheduleFrame();
          });
        // The 2nd frame: render, which makes the window appear.
        case 2:
          drawHelloWorld(view);
          _waitUntilWindowVisible().then((_) {
            if (!visibilityCompleter.isCompleted) {
              visibilityCompleter.complete('success');
            }
          });
        // Others, in case requested to render.
        default:
          drawHelloWorld(view);
      }
    };
  } catch (e) {
    visibilityCompleter.completeError(e);
    rethrow;
  }
  ui.PlatformDispatcher.instance.scheduleFrame();
}
