// Copyright 2013 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

import 'dart:ui';

void main(List<String> args) {
  print('child-view: starting');

  TestApp app = TestApp();
  app.run();
}

class TestApp {
  static const _yellow = Color.fromARGB(255, 255, 255, 0);
  static const _pink = Color.fromARGB(255, 255, 0, 255);

  Color _backgroundColor = _pink;

  void run() {
    window.onPointerDataPacket = (PointerDataPacket packet) {
      this.pointerDataPacket(packet);
    };
    window.onMetricsChanged = () {
      window.scheduleFrame();
    };
    window.onBeginFrame = (Duration duration) {
      this.beginFrame(duration);
    };

    window.scheduleFrame();
  }

  void beginFrame(Duration duration) {
    final pixelRatio = window.devicePixelRatio;
    final size = window.physicalSize / pixelRatio;
    final physicalBounds = Offset.zero & size * pixelRatio;
    final recorder = PictureRecorder();
    final canvas = Canvas(recorder, physicalBounds);
    canvas.scale(pixelRatio, pixelRatio);
    final paint = Paint()..color = this._backgroundColor;
    canvas.drawRect(Rect.fromLTWH(0, 0, size.width, size.height), paint);
    final picture = recorder.endRecording();
    final sceneBuilder =
        SceneBuilder()
          ..pushClipRect(physicalBounds)
          ..addPicture(Offset.zero, picture)
          ..pop();
    window.render(sceneBuilder.build());
  }

  void pointerDataPacket(PointerDataPacket packet) {
    for (final data in packet.data) {
      if (data.change == PointerChange.down) {
        this._backgroundColor = _yellow;
      }
    }
    window.scheduleFrame();
  }
}
