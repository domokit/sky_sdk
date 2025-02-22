// Copyright 2013 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

// ignore_for_file: avoid_print

import 'dart:io';
import 'dart:typed_data';
import 'dart:ui';

@pragma('vm:external-name', 'SignalNativeTest')
external void signalNativeTest();

void main() {}

@pragma('vm:entry-point')
void empty() {}

/// Notifies the test of a string value.
///
/// This is used to notify the native side of the test of a string value from
/// the Dart fixture under test.
@pragma('vm:external-name', 'NotifyStringValue')
external void notifyStringValue(String s);

@pragma('vm:entry-point')
void executableNameNotNull() {
  notifyStringValue(Platform.executable);
}

@pragma('vm:entry-point')
void canLogToStdout() {
  // Emit hello world message to output then signal the test.
  print('Hello logging');
  signalNativeTest();
}

@pragma('vm:entry-point')
void canCompositePlatformViews() {
  PlatformDispatcher.instance.onBeginFrame = (Duration duration) {
    final builder = SceneBuilder();
    builder.addPicture(const Offset(1.0, 1.0), _createSimplePicture());
    builder.pushOffset(1.0, 2.0);
    builder.addPlatformView(42, width: 123.0, height: 456.0);
    builder.addPicture(const Offset(1.0, 1.0), _createSimplePicture());
    builder.pop(); // offset
    PlatformDispatcher.instance.views.first.render(builder.build());
  };
  PlatformDispatcher.instance.scheduleFrame();
}

@pragma('vm:entry-point')
void drawIntoAllViews() {
  PlatformDispatcher.instance.onBeginFrame = (Duration duration) {
    final builder = SceneBuilder();
    builder.addPicture(const Offset(1.0, 1.0), _createSimplePicture());
    for (final FlutterView view in PlatformDispatcher.instance.views) {
      view.render(builder.build());
    }
  };
  PlatformDispatcher.instance.scheduleFrame();
}

/// Returns a [Picture] of a simple black square.
Picture _createSimplePicture() {
  final blackPaint = Paint();
  final baseRecorder = PictureRecorder();
  final canvas = Canvas(baseRecorder);
  canvas.drawRect(const Rect.fromLTRB(0.0, 0.0, 1000.0, 1000.0), blackPaint);
  return baseRecorder.endRecording();
}

@pragma('vm:entry-point')
void nativeCallback() {
  signalNativeTest();
}

@pragma('vm:entry-point')
void backgroundTest() {
  PlatformDispatcher.instance.views.first.render(SceneBuilder().build());
  signalNativeTest(); // should look black
}

@pragma('vm:entry-point')
void sendFooMessage() {
  PlatformDispatcher.instance.sendPlatformMessage('foo', null, (ByteData? result) {});
}

@pragma('vm:external-name', 'NotifyEngineId')
external void notifyEngineId(int? engineId);

@pragma('vm:entry-point')
void testEngineId() {
  notifyEngineId(PlatformDispatcher.instance.engineId);
}
