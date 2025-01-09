// Copyright 2013 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

import 'dart:ui';

import 'package:test/test.dart';

void main() {
  test('MaskFilter - NOP blur does not crash', () async {
    final PictureRecorder recorder = PictureRecorder();
    final Canvas canvas = Canvas(recorder);
    final Paint paint =
        Paint()
          ..color = const Color(0xff00AA00)
          ..maskFilter = const MaskFilter.blur(BlurStyle.normal, 0);
    canvas.saveLayer(const Rect.fromLTRB(-100, -100, 200, 200), paint);
    canvas.drawRect(const Rect.fromLTRB(0, 0, 100, 100), Paint());
    canvas.restore();
    final Picture picture = recorder.endRecording();

    final SceneBuilder builder = SceneBuilder();
    builder.addPicture(Offset.zero, picture);

    final Scene scene = builder.build();
    await scene.toImage(100, 100);
  });
}
