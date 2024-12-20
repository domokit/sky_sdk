// Copyright 2013 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

import 'package:test/bootstrap/browser.dart';
import 'package:test/test.dart';
import 'package:ui/src/engine.dart';
import 'package:ui/ui.dart' hide TextStyle;
import '../../common/test_initialization.dart';
import '../screenshot.dart';

void main() {
  internalBootstrapBrowserTest(() => testMain);
}

Future<void> testMain() async {
  setUpUnitTests(setUpTestViewDimensions: false);

  Future<void> testGradient(
    String fileName,
    Shader shader, {
    Rect paintRect = const Rect.fromLTRB(50, 50, 300, 300),
    Rect shaderRect = const Rect.fromLTRB(50, 50, 300, 300),
    Rect region = const Rect.fromLTWH(0, 0, 500, 500),
  }) async {
    final RecordingCanvas rc = RecordingCanvas(region);
    final SurfacePaint paint = SurfacePaint()..shader = shader;
    final Path path = Path();
    path.addRect(paintRect);
    rc.drawPath(path, paint);
    await canvasScreenshot(rc, fileName, region: region);
  }

  test('Should draw centered radial gradient.', () async {
    const Rect shaderRect = Rect.fromLTRB(50, 50, 300, 300);
    await testGradient(
      'radial_gradient_centered',
      Gradient.radial(
        Offset((shaderRect.left + shaderRect.right) / 2, (shaderRect.top + shaderRect.bottom) / 2),
        shaderRect.width / 2,
        <Color>[const Color.fromARGB(255, 0, 0, 0), const Color.fromARGB(255, 0, 0, 255)],
      ),
    );
  });

  test('Should draw right bottom centered radial gradient.', () async {
    const Rect shaderRect = Rect.fromLTRB(50, 50, 300, 300);
    await testGradient(
      'radial_gradient_right_bottom',
      Gradient.radial(Offset(shaderRect.right, shaderRect.bottom), shaderRect.width / 2, <Color>[
        const Color.fromARGB(255, 0, 0, 0),
        const Color.fromARGB(255, 0, 0, 255),
      ]),
    );
  });

  test('Should draw with radial gradient with TileMode.clamp.', () async {
    const Rect shaderRect = Rect.fromLTRB(50, 50, 100, 100);
    await testGradient(
      'radial_gradient_tilemode_clamp',
      Gradient.radial(
        Offset((shaderRect.left + shaderRect.right) / 2, (shaderRect.top + shaderRect.bottom) / 2),
        shaderRect.width / 2,
        <Color>[const Color.fromARGB(255, 0, 0, 0), const Color.fromARGB(255, 0, 0, 255)],
        <double>[0.0, 1.0],
      ),
      shaderRect: shaderRect,
    );
  });

  const List<Color> colors = <Color>[
    Color(0xFF000000),
    Color(0xFFFF3C38),
    Color(0xFFFF8C42),
    Color(0xFFFFF275),
    Color(0xFF6699CC),
    Color(0xFF656D78),
  ];
  const List<double> colorStops = <double>[0.0, 0.05, 0.4, 0.6, 0.9, 1.0];

  test(
    'Should draw with radial gradient with TileMode.repeated.',
    () async {
      const Rect shaderRect = Rect.fromLTRB(50, 50, 100, 100);
      await testGradient(
        'radial_gradient_tilemode_repeated',
        Gradient.radial(
          Offset(
            (shaderRect.left + shaderRect.right) / 2,
            (shaderRect.top + shaderRect.bottom) / 2,
          ),
          shaderRect.width / 2,
          colors,
          colorStops,
          TileMode.repeated,
        ),
        shaderRect: shaderRect,
        region: const Rect.fromLTWH(0, 0, 600, 800),
      );
    },
    // TODO(yjbanov): https://github.com/flutter/flutter/issues/86623
    skip: isFirefox,
  );

  test(
    'Should draw with radial gradient with TileMode.mirrored.',
    () async {
      const Rect shaderRect = Rect.fromLTRB(50, 50, 100, 100);
      await testGradient(
        'radial_gradient_tilemode_mirror',
        Gradient.radial(
          Offset(
            (shaderRect.left + shaderRect.right) / 2,
            (shaderRect.top + shaderRect.bottom) / 2,
          ),
          shaderRect.width / 2,
          colors,
          colorStops,
          TileMode.mirror,
        ),
        shaderRect: shaderRect,
        region: const Rect.fromLTWH(0, 0, 600, 800),
      );
    },
    // TODO(yjbanov): https://github.com/flutter/flutter/issues/86623
    skip: isFirefox,
  );
}
