// Copyright 2020 The Fuchsia Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

import 'dart:convert';
import 'dart:typed_data';
import 'dart:ui';
import 'dart:zircon';

void main() {
  print('Launching mouse-input-view');
  MyApp app = MyApp();
  app.run();
}

class MyApp {
  static const _red = Color.fromARGB(255, 244, 67, 54);
  static const _orange = Color.fromARGB(255, 255, 152, 0);
  static const _yellow = Color.fromARGB(255, 255, 235, 59);
  static const _green = Color.fromARGB(255, 76, 175, 80);
  static const _blue = Color.fromARGB(255, 33, 150, 143);
  static const _purple = Color.fromARGB(255, 156, 39, 176);

  final List<Color> _colors = <Color>[_red, _orange, _yellow, _green, _blue, _purple];

  // Each tap will increment the counter, we then determine what color to choose
  int _touchCounter = 0;

  void run() {
    // Set up window callbacks.
    window.onPointerDataPacket = (PointerDataPacket packet) {
      this.pointerDataPacket(packet);
    };
    window.onMetricsChanged = () {
      window.scheduleFrame();
    };
    window.onBeginFrame = (Duration duration) {
      this.beginFrame(duration);
    };

    // The child view should be attached to Scenic now.
    // Ready to build the scene.
    window.scheduleFrame();
  }

  void beginFrame(Duration duration) {
    // Convert physical screen size of device to values
    final pixelRatio = window.devicePixelRatio;
    final size = window.physicalSize / pixelRatio;
    final physicalBounds = Offset.zero & size * pixelRatio;
    // Set up Canvas that uses the screen size
    final recorder = PictureRecorder();
    final canvas = Canvas(recorder, physicalBounds);
    canvas.scale(pixelRatio, pixelRatio);
    // Draw something
    // Color of the screen is set initially to the first value in _colors
    // Incrementing _touchCounter will change screen color
    final paint = Paint()..color = _colors[_touchCounter % _colors.length];
    canvas.drawRect(Rect.fromLTWH(0, 0, size.width, size.height), paint);
    // Build the scene
    final picture = recorder.endRecording();
    final sceneBuilder =
        SceneBuilder()
          ..pushClipRect(physicalBounds)
          ..addPicture(Offset.zero, picture)
          ..pop();
    window.render(sceneBuilder.build());
  }

  void pointerDataPacket(PointerDataPacket packet) async {
    int nowNanos = System.clockGetMonotonic();

    for (PointerData data in packet.data) {
      print('mouse-input-view received input: ${data.toStringFull()}');

      if (data.kind == PointerDeviceKind.mouse) {
        if (data.change == PointerChange.down) {
          _touchCounter++;
        }

        _reportMouseInput(
          localX: data.physicalX,
          localY: data.physicalY,
          buttons: data.buttons,
          phase: data.change.name,
          timeReceived: nowNanos,
          wheelXPhysicalPixel: data.scrollDeltaX,
          wheelYPhysicalPixel: data.scrollDeltaY,
        );
      }
    }

    window.scheduleFrame();
  }

  void _reportMouseInput({
    required double localX,
    required double localY,
    required int timeReceived,
    required int buttons,
    required String phase,
    required double wheelXPhysicalPixel,
    required double wheelYPhysicalPixel,
  }) {
    print('mouse-input-view reporting mouse input to MouseInputListener');
    final message = ByteData.sublistView(
      utf8.encode(
        json.encode({
          'method': 'MouseInputListener.ReportMouseInput',
          'local_x': localX,
          'local_y': localY,
          'time_received': timeReceived,
          'component_name': 'touch-input-view',
          'buttons': buttons,
          'phase': phase,
          'wheel_x_physical_pixel': wheelXPhysicalPixel,
          'wheel_y_physical_pixel': wheelYPhysicalPixel,
        }),
      ),
    );
    PlatformDispatcher.instance.sendPlatformMessage('fuchsia/input_test', message, null);
  }
}
