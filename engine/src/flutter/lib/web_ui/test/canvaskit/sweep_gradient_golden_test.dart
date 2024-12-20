// Copyright 2013 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

import 'dart:math' as math;

import 'package:test/bootstrap/browser.dart';
import 'package:test/test.dart';
import 'package:ui/src/engine.dart';
import 'package:ui/ui.dart' as ui;

import 'common.dart';

void main() {
  internalBootstrapBrowserTest(() => testMain);
}

const ui.Rect region = ui.Rect.fromLTRB(0, 0, 500, 250);

void testMain() {
  group('SweepGradient', () {
    setUpCanvasKitTest(withImplicitView: true);

    test('is correctly rendered', () async {
      final CkPictureRecorder recorder = CkPictureRecorder();
      final CkCanvas canvas = recorder.beginRecording(region);

      final CkGradientSweep gradient = CkGradientSweep(
        const ui.Offset(250, 125),
        const <ui.Color>[
          ui.Color(0xFF4285F4),
          ui.Color(0xFF34A853),
          ui.Color(0xFFFBBC05),
          ui.Color(0xFFEA4335),
          ui.Color(0xFF4285F4),
        ],
        const <double>[0.0, 0.25, 0.5, 0.75, 1.0],
        ui.TileMode.clamp,
        -(math.pi / 2),
        math.pi * 2 - (math.pi / 2),
        null,
      );

      final CkPaint paint = CkPaint()..shader = gradient;

      canvas.drawRect(region, paint);

      await matchPictureGolden(
        'canvaskit_sweep_gradient.png',
        recorder.endRecording(),
        region: region,
      );
    });
    // TODO(hterkelsen): https://github.com/flutter/flutter/issues/71520
  }, skip: isSafari || isFirefox);
}
