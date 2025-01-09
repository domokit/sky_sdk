// Copyright 2013 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.
import 'package:test/bootstrap/browser.dart';
import 'package:test/test.dart';
import 'package:ui/src/engine.dart';
import 'package:ui/ui.dart' as ui;

import 'common.dart';

void main() {
  internalBootstrapBrowserTest(() => testMain);
}

const ui.Rect region = ui.Rect.fromLTRB(0, 0, 500, 250);

/// Test that we can render even if `createImageBitmap` is not supported.
void testMain() {
  group('CanvasKit', () {
    setUpCanvasKitTest(withImplicitView: true);
    setUp(() async {
      EngineFlutterDisplay.instance.debugOverrideDevicePixelRatio(1.0);
    });

    tearDown(() {
      debugDisableCreateImageBitmapSupport = false;
      debugIsChrome110OrOlder = null;
    });

    test('can render without createImageBitmap', () async {
      debugDisableCreateImageBitmapSupport = true;

      expect(browserSupportsCreateImageBitmap, isFalse);

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
        'canvaskit_linear_gradient_no_create_image_bitmap.png',
        recorder.endRecording(),
        region: region,
      );
    });

    test('createImageBitmap support is disabled on '
        'Windows on Chrome version 110 or older', () async {
      debugIsChrome110OrOlder = true;
      debugDisableCreateImageBitmapSupport = false;

      expect(browserSupportsCreateImageBitmap, isFalse);
    });
  });
}
