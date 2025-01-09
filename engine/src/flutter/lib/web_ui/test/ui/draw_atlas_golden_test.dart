// Copyright 2013 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

import 'dart:math' as math;
import 'dart:typed_data';

import 'package:test/bootstrap/browser.dart';
import 'package:test/test.dart';
import 'package:ui/ui.dart' as ui;
import 'package:web_engine_tester/golden_tester.dart';

import '../common/test_initialization.dart';
import 'utils.dart';

void main() {
  internalBootstrapBrowserTest(() => testMain);
}

const ui.Rect kBlueSquareRegion = ui.Rect.fromLTRB(0, 0, 25, 25);
const ui.Rect kRedCircleRegion = ui.Rect.fromLTRB(25, 0, 50, 25);
const ui.Rect kMagentaStarRegion = ui.Rect.fromLTRB(0, 25, 30, 55);
const ui.Rect kGreenSquiggleRegion = ui.Rect.fromLTRB(30, 25, 50, 55);
const ui.Rect kTotalAtlasRegion = ui.Rect.fromLTRB(0, 0, 55, 55);

ui.Image generateAtlas() {
  final ui.PictureRecorder recorder = ui.PictureRecorder();
  final ui.Canvas canvas = ui.Canvas(recorder);

  canvas.drawColor(const ui.Color(0), ui.BlendMode.src);

  // Draw the square
  canvas.save();
  canvas.clipRect(kBlueSquareRegion);
  canvas.drawRect(
    // Inset the square by one pixel so it doesn't bleed into the other sprites.
    kBlueSquareRegion.deflate(1.0),
    ui.Paint()..color = const ui.Color(0xFF0000FF),
  );
  canvas.restore();

  // Draw the circle
  canvas.save();
  canvas.clipRect(kRedCircleRegion);
  canvas.drawCircle(
    kRedCircleRegion.center,
    // Make the circle one pixel smaller than the bounds to it doesn't bleed
    // into the other shapes.
    (kRedCircleRegion.width / 2.0) - 1.0,
    ui.Paint()..color = const ui.Color(0xFFFF0000),
  );
  canvas.restore();

  // Draw the star
  canvas.save();
  canvas.clipRect(kMagentaStarRegion);
  final ui.Offset starCenter = kMagentaStarRegion.center;

  // Make the star one pixel smaller than the bounds so that it doesn't bleed
  // into the other shapes.
  final double radius = (kMagentaStarRegion.height / 2.0) - 1.0;

  // Start at the top (rotated 90 degrees)
  double theta = -math.pi / 2.0;

  // Rotate two fifths of the circle each time
  const double rotation = 4.0 * math.pi / 5.0;
  final ui.Path starPath = ui.Path();
  starPath.moveTo(
    starCenter.dx + radius * math.cos(theta),
    starCenter.dy + radius * math.sin(theta),
  );
  for (int i = 0; i < 5; i++) {
    theta += rotation;
    starPath.lineTo(
      starCenter.dx + radius * math.cos(theta),
      starCenter.dy + radius * math.sin(theta),
    );
  }
  canvas.drawPath(
    starPath,
    ui.Paint()
      ..color = const ui.Color(0xFFFF00FF)
      ..style = ui.PaintingStyle.fill,
  );
  canvas.restore();

  // Draw the Squiggle
  canvas.save();
  canvas.clipRect(kGreenSquiggleRegion);
  final ui.Path squigglePath = ui.Path();
  squigglePath.moveTo(kGreenSquiggleRegion.topCenter.dx, kGreenSquiggleRegion.topCenter.dy + 2.0);
  squigglePath.cubicTo(
    kGreenSquiggleRegion.left - 10.0,
    kGreenSquiggleRegion.top + kGreenSquiggleRegion.height * 0.33,
    kGreenSquiggleRegion.right + 10.0,
    kGreenSquiggleRegion.top + kGreenSquiggleRegion.height * 0.66,
    kGreenSquiggleRegion.bottomCenter.dx,
    kGreenSquiggleRegion.bottomCenter.dy - 2.0,
  );
  canvas.drawPath(
    squigglePath,
    ui.Paint()
      ..color = const ui.Color(0xFF00FF00)
      ..style = ui.PaintingStyle.stroke
      ..strokeWidth = 5.0,
  );
  canvas.restore();

  final ui.Picture picture = recorder.endRecording();
  return picture.toImageSync(kTotalAtlasRegion.width.toInt(), kTotalAtlasRegion.height.toInt());
}

Future<void> testMain() async {
  setUpUnitTests(withImplicitView: true, setUpTestViewDimensions: false);

  const ui.Rect region = ui.Rect.fromLTWH(0, 0, 300, 300);

  test('render atlas', () async {
    final ui.PictureRecorder recorder = ui.PictureRecorder();
    final ui.Canvas canvas = ui.Canvas(recorder, region);
    final ui.Image atlas = generateAtlas();
    canvas.drawImage(atlas, ui.Offset.zero, ui.Paint());
    await drawPictureUsingCurrentRenderer(recorder.endRecording());
    await matchGoldenFile('ui_atlas.png', region: region);
  }, skip: isHtml); // HTML renderer doesn't support drawAtlas

  test('drawAtlas', () async {
    final ui.PictureRecorder recorder = ui.PictureRecorder();
    final ui.Canvas canvas = ui.Canvas(recorder, region);
    final ui.Image atlas = generateAtlas();
    final List<ui.RSTransform> transforms = List<ui.RSTransform>.generate(12, (int index) {
      const double radius = 100;
      const double rotation = math.pi / 6.0;
      final double angle = rotation * index;
      final double scos = math.sin(angle);
      final double ssin = math.cos(angle);
      return ui.RSTransform(
        scos,
        ssin,
        region.center.dx + radius * scos,
        region.center.dy + radius * ssin,
      );
    });
    final List<ui.Rect> rects = <ui.Rect>[
      kBlueSquareRegion,
      kRedCircleRegion,
      kMagentaStarRegion,
      kGreenSquiggleRegion,
      kBlueSquareRegion,
      kRedCircleRegion,
      kMagentaStarRegion,
      kGreenSquiggleRegion,
      kBlueSquareRegion,
      kRedCircleRegion,
      kMagentaStarRegion,
      kGreenSquiggleRegion,
    ];
    canvas.drawAtlas(atlas, transforms, rects, null, null, null, ui.Paint());

    await drawPictureUsingCurrentRenderer(recorder.endRecording());
    await matchGoldenFile('ui_draw_atlas.png', region: region);
  }, skip: isHtml); // HTML renderer doesn't support drawAtlas

  test('drawAtlasRaw', () async {
    final ui.PictureRecorder recorder = ui.PictureRecorder();
    final ui.Canvas canvas = ui.Canvas(recorder, region);
    final ui.Image atlas = generateAtlas();
    final Float32List transforms = Float32List(12 * 4);
    for (int i = 0; i < 12; i++) {
      const double radius = 100;
      const double rotation = math.pi / 6.0;
      final double angle = rotation * i;
      final double scos = math.sin(angle);
      final double ssin = math.cos(angle);
      transforms[i * 4] = scos;
      transforms[i * 4 + 1] = ssin;
      transforms[i * 4 + 2] = region.center.dx + radius * scos;
      transforms[i * 4 + 3] = region.center.dy + radius * ssin;
    }
    final List<ui.Rect> rects = <ui.Rect>[
      kBlueSquareRegion,
      kRedCircleRegion,
      kMagentaStarRegion,
      kGreenSquiggleRegion,
      kBlueSquareRegion,
      kRedCircleRegion,
      kMagentaStarRegion,
      kGreenSquiggleRegion,
      kBlueSquareRegion,
      kRedCircleRegion,
      kMagentaStarRegion,
      kGreenSquiggleRegion,
    ];
    final Float32List rawRects = Float32List(rects.length * 4);
    for (int i = 0; i < rects.length; i++) {
      rawRects[i * 4] = rects[i].left;
      rawRects[i * 4 + 1] = rects[i].top;
      rawRects[i * 4 + 2] = rects[i].right;
      rawRects[i * 4 + 3] = rects[i].bottom;
    }

    final Int32List colors = Int32List(12);
    for (int i = 0; i < 12; i++) {
      final int rgb = 0xFF << (8 * (i % 3));
      colors[i] = 0xFF000000 | rgb;
    }
    canvas.drawRawAtlas(atlas, transforms, rawRects, colors, ui.BlendMode.dstIn, null, ui.Paint());

    await drawPictureUsingCurrentRenderer(recorder.endRecording());
    await matchGoldenFile('ui_draw_atlas_raw.png', region: region);
  }, skip: isHtml); // HTML renderer doesn't support drawAtlas
}
