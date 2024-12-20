// Copyright 2013 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

import 'package:test/bootstrap/browser.dart';
import 'package:test/test.dart';
import 'package:ui/ui.dart' as ui;

import '../common/test_initialization.dart';

void main() {
  internalBootstrapBrowserTest(() => testMain);
}

Future<void> testMain() async {
  setUpUnitTests(emulateTesterEnvironment: false, setUpTestViewDimensions: false);

  test('default field values are as documented on api.flutter.dev', () {
    final paint = ui.Paint();
    expect(paint.blendMode, ui.BlendMode.srcOver);
    expect(paint.color, const ui.Color(0xFF000000));
    expect(paint.colorFilter, null);
    expect(paint.filterQuality, ui.FilterQuality.none);
    expect(paint.imageFilter, null);
    expect(paint.invertColors, false);
    expect(paint.isAntiAlias, true);
    expect(paint.maskFilter, null);
    expect(paint.shader, null);
    expect(paint.strokeCap, ui.StrokeCap.butt);
    expect(paint.strokeJoin, ui.StrokeJoin.miter);
    expect(paint.strokeMiterLimit, 4.0);
    expect(paint.strokeWidth, 0.0);
    expect(paint.style, ui.PaintingStyle.fill);
  });

  test('toString()', () {
    final ui.Paint paint = ui.Paint();
    paint.blendMode = ui.BlendMode.darken;
    paint.style = ui.PaintingStyle.fill;
    paint.strokeWidth = 1.2;
    paint.strokeCap = ui.StrokeCap.square;
    paint.strokeJoin = ui.StrokeJoin.bevel;
    paint.isAntiAlias = true;
    paint.color = const ui.Color(0xaabbccdd);
    paint.invertColors = true;
    paint.shader = ui.Gradient.linear(
      const ui.Offset(0.1, 0.2),
      const ui.Offset(1.5, 1.6),
      const <ui.Color>[ui.Color(0xaabbccdd), ui.Color(0xbbccddee)],
      <double>[0.3, 0.4],
      ui.TileMode.decal,
    );
    paint.maskFilter = const ui.MaskFilter.blur(ui.BlurStyle.normal, 1.7);
    paint.filterQuality = ui.FilterQuality.high;
    paint.colorFilter = const ui.ColorFilter.linearToSrgbGamma();
    paint.strokeMiterLimit = 1.8;
    paint.imageFilter = ui.ImageFilter.blur(sigmaX: 1.9, sigmaY: 2.1, tileMode: ui.TileMode.mirror);

    expect(
      paint.toString(),
      'Paint('
      '${const ui.Color(0xaabbccdd)}; '
      'BlendMode.darken; '
      'colorFilter: ColorFilter.linearToSrgbGamma(); '
      'maskFilter: MaskFilter.blur(BlurStyle.normal, 1.7); '
      'filterQuality: FilterQuality.high; '
      'shader: Gradient(); '
      'imageFilter: ImageFilter.blur(1.9, 2.1, mirror); '
      'invert: true'
      ')',
    );
  });

  test('.from copies every field', () {
    final ui.Paint paint = ui.Paint();
    paint.blendMode = ui.BlendMode.darken;
    paint.style = ui.PaintingStyle.fill;
    paint.strokeWidth = 1.2;
    paint.strokeCap = ui.StrokeCap.square;
    paint.strokeJoin = ui.StrokeJoin.bevel;
    paint.isAntiAlias = true;
    paint.color = const ui.Color(0xaabbccdd);
    paint.invertColors = true;
    paint.shader = ui.Gradient.linear(
      const ui.Offset(0.1, 0.2),
      const ui.Offset(1.5, 1.6),
      const <ui.Color>[ui.Color(0xaabbccdd), ui.Color(0xbbccddee)],
      <double>[0.3, 0.4],
      ui.TileMode.decal,
    );
    paint.maskFilter = const ui.MaskFilter.blur(ui.BlurStyle.normal, 1.7);
    paint.filterQuality = ui.FilterQuality.high;
    paint.colorFilter = const ui.ColorFilter.linearToSrgbGamma();
    paint.strokeMiterLimit = 1.8;
    paint.imageFilter = ui.ImageFilter.blur(sigmaX: 1.9, sigmaY: 2.1, tileMode: ui.TileMode.mirror);

    final ui.Paint copy = ui.Paint.from(paint);

    expect(copy.blendMode, paint.blendMode);
    expect(copy.style, paint.style);
    expect(copy.strokeWidth, paint.strokeWidth);
    expect(copy.strokeCap, paint.strokeCap);
    expect(copy.strokeJoin, paint.strokeJoin);
    expect(copy.isAntiAlias, paint.isAntiAlias);
    expect(copy.color, paint.color);
    expect(copy.invertColors, paint.invertColors);
    expect(copy.shader, paint.shader);
    expect(copy.maskFilter, paint.maskFilter);
    expect(copy.filterQuality, paint.filterQuality);
    expect(copy.colorFilter, paint.colorFilter);
    expect(copy.strokeMiterLimit, paint.strokeMiterLimit);
    expect(copy.imageFilter, paint.imageFilter);
  });
}
