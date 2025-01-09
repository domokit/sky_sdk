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
  group('Linear', () {
    setUpCanvasKitTest(withImplicitView: true);

    test('is correctly rendered', () async {
      final CkPictureRecorder recorder = CkPictureRecorder();
      final CkCanvas canvas = recorder.beginRecording(region);

      final CkGradientLinear gradient = CkGradientLinear(
        ui.Offset(region.left + region.width / 4, region.height / 2),
        ui.Offset(region.right - region.width / 8, region.height / 2),
        const <ui.Color>[
          ui.Color(0xFF4285F4),
          ui.Color(0xFF34A853),
          ui.Color(0xFFFBBC05),
          ui.Color(0xFFEA4335),
          ui.Color(0xFF4285F4),
        ],
        const <double>[0.0, 0.25, 0.5, 0.75, 1.0],
        ui.TileMode.clamp,
        null,
      );

      final CkPaint paint = CkPaint()..shader = gradient;

      canvas.drawRect(region, paint);

      await matchPictureGolden(
        'canvaskit_linear_gradient.png',
        recorder.endRecording(),
        region: region,
      );
    });

    test('is correctly rendered when rotated', () async {
      final CkPictureRecorder recorder = CkPictureRecorder();
      final CkCanvas canvas = recorder.beginRecording(region);

      final CkGradientLinear gradient = CkGradientLinear(
        ui.Offset(region.left + region.width / 4, region.height / 2),
        ui.Offset(region.right - region.width / 8, region.height / 2),
        const <ui.Color>[
          ui.Color(0xFF4285F4),
          ui.Color(0xFF34A853),
          ui.Color(0xFFFBBC05),
          ui.Color(0xFFEA4335),
          ui.Color(0xFF4285F4),
        ],
        const <double>[0.0, 0.25, 0.5, 0.75, 1.0],
        ui.TileMode.clamp,
        Matrix4.rotationZ(math.pi / 6.0).storage,
      );

      final CkPaint paint = CkPaint()..shader = gradient;

      canvas.drawRect(region, paint);

      await matchPictureGolden(
        'canvaskit_linear_gradient_rotated.png',
        recorder.endRecording(),
        region: region,
      );
    });
    // TODO(hterkelsen): https://github.com/flutter/flutter/issues/71520
  }, skip: isSafari || isFirefox);
}
