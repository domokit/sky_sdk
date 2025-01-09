// Copyright 2013 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

import 'package:test/bootstrap/browser.dart';
import 'package:test/test.dart';
import 'package:ui/ui.dart' as ui;
import 'package:web_engine_tester/golden_tester.dart';

import '../common/test_initialization.dart';
import 'utils.dart';

void main() {
  internalBootstrapBrowserTest(() => testMain);
}

Future<void> testMain() async {
  const ui.Rect region = ui.Rect.fromLTWH(0, 0, 300, 300);

  setUpUnitTests(
    withImplicitView: true,
    emulateTesterEnvironment: false,
    setUpTestViewDimensions: false,
  );

  test('Test drawing a shadow of an opaque object', () async {
    final ui.Picture picture = drawPicture((ui.Canvas canvas) {
      final ui.Path path = ui.Path();
      path.moveTo(50, 150);
      path.cubicTo(100, 50, 200, 250, 250, 150);

      canvas.drawShadow(path, const ui.Color(0xFF000000), 5, false);
      canvas.drawPath(path, ui.Paint()..color = const ui.Color(0xFFFF00FF));
    });
    await drawPictureUsingCurrentRenderer(picture);
    await matchGoldenFile('shadow_opaque_object.png', region: region);
  });

  test('Test drawing a shadow of a translucent object', () async {
    final ui.Picture picture = drawPicture((ui.Canvas canvas) {
      final ui.Path path = ui.Path();
      path.moveTo(50, 150);
      path.cubicTo(100, 250, 200, 50, 250, 150);

      canvas.drawShadow(path, const ui.Color(0xFF000000), 5, true);
      canvas.drawPath(path, ui.Paint()..color = const ui.Color(0x8F00FFFF));
    });
    await drawPictureUsingCurrentRenderer(picture);
    await matchGoldenFile('shadow_translucent_object.png', region: region);
  });
}
