// Copyright 2013 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

import 'dart:math' as math;

import 'package:test/bootstrap/browser.dart';
import 'package:test/test.dart';
import 'package:ui/src/engine.dart';
import 'package:ui/ui.dart' hide TextStyle;
import '../../common/test_initialization.dart';
import '../screenshot.dart';

// TODO(yjbanov): unskip Firefox tests when Firefox implements WebGL in headless mode.
// https://github.com/flutter/flutter/issues/86623

void main() {
  internalBootstrapBrowserTest(() => testMain);
}

Future<void> testMain() async {
  setUpUnitTests(setUpTestViewDimensions: false);

  test('Should draw linear gradient using rectangle.', () async {
    final RecordingCanvas rc = RecordingCanvas(const Rect.fromLTRB(0, 0, 500, 500));
    const Rect shaderRect = Rect.fromLTRB(50, 50, 300, 300);
    final SurfacePaint paint =
        SurfacePaint()
          ..shader = Gradient.linear(
            Offset(shaderRect.left, shaderRect.top),
            Offset(shaderRect.right, shaderRect.bottom),
            const <Color>[Color(0xFFcfdfd2), Color(0xFF042a85)],
          );
    rc.drawRect(shaderRect, paint);
    expect(rc.renderStrategy.hasArbitraryPaint, isTrue);
    await canvasScreenshot(rc, 'linear_gradient_rect');
  });

  test('Should blend linear gradient with alpha channel correctly.', () async {
    const Rect canvasRect = Rect.fromLTRB(0, 0, 500, 500);
    final RecordingCanvas rc = RecordingCanvas(canvasRect);
    final SurfacePaint backgroundPaint =
        SurfacePaint()
          ..style = PaintingStyle.fill
          ..color = const Color(0xFFFF0000);
    rc.drawRect(canvasRect, backgroundPaint);

    const Rect shaderRect = Rect.fromLTRB(50, 50, 300, 300);
    final SurfacePaint paint =
        SurfacePaint()
          ..shader = Gradient.linear(
            Offset(shaderRect.left, shaderRect.top),
            Offset(shaderRect.right, shaderRect.bottom),
            const <Color>[Color(0x00000000), Color(0xFF0000FF)],
          );
    rc.drawRect(shaderRect, paint);
    expect(rc.renderStrategy.hasArbitraryPaint, isTrue);
    await canvasScreenshot(rc, 'linear_gradient_rect_alpha');
  });

  test('Should draw linear gradient with transform.', () async {
    final RecordingCanvas rc = RecordingCanvas(const Rect.fromLTRB(0, 0, 500, 500));
    final List<double> angles = <double>[0.0, 90.0, 180.0];
    double yOffset = 0;
    for (final double angle in angles) {
      final Rect shaderRect = Rect.fromLTWH(50, 50 + yOffset, 100, 100);
      final Matrix4 matrix = Matrix4.identity();
      matrix.translate(shaderRect.left, shaderRect.top);
      matrix.multiply(Matrix4.rotationZ((angle / 180) * math.pi));
      final Matrix4 post = Matrix4.identity();
      post.translate(-shaderRect.left, -shaderRect.top);
      matrix.multiply(post);
      final SurfacePaint paint =
          SurfacePaint()
            ..shader = Gradient.linear(
              Offset(shaderRect.left, shaderRect.top),
              Offset(shaderRect.right, shaderRect.bottom),
              const <Color>[Color(0xFFFF0000), Color(0xFF042a85)],
              null,
              TileMode.clamp,
              matrix.toFloat64(),
            );
      rc.drawRect(shaderRect, SurfacePaint()..color = const Color(0xFF000000));
      rc.drawOval(shaderRect, paint);
      yOffset += 120;
    }
    expect(rc.renderStrategy.hasArbitraryPaint, isTrue);
    await canvasScreenshot(rc, 'linear_gradient_oval_matrix');
  }, skip: isFirefox);

  // Regression test for https://github.com/flutter/flutter/issues/50010
  test('Should draw linear gradient using rounded rect.', () async {
    final RecordingCanvas rc = RecordingCanvas(const Rect.fromLTRB(0, 0, 500, 500));
    const Rect shaderRect = Rect.fromLTRB(50, 50, 300, 300);
    final SurfacePaint paint =
        SurfacePaint()
          ..shader = Gradient.linear(
            Offset(shaderRect.left, shaderRect.top),
            Offset(shaderRect.right, shaderRect.bottom),
            const <Color>[Color(0xFFcfdfd2), Color(0xFF042a85)],
          );
    rc.drawRRect(RRect.fromRectAndRadius(shaderRect, const Radius.circular(16)), paint);
    expect(rc.renderStrategy.hasArbitraryPaint, isTrue);
    await canvasScreenshot(rc, 'linear_gradient_rounded_rect');
  });

  test('Should draw tiled repeated linear gradient with transform.', () async {
    final RecordingCanvas rc = RecordingCanvas(const Rect.fromLTRB(0, 0, 500, 500));
    final List<double> angles = <double>[0.0, 30.0, 210.0];
    double yOffset = 0;
    for (final double angle in angles) {
      final Rect shaderRect = Rect.fromLTWH(50, 50 + yOffset, 100, 100);
      final SurfacePaint paint =
          SurfacePaint()
            ..shader = Gradient.linear(
              Offset(shaderRect.left, shaderRect.top),
              Offset(shaderRect.left + shaderRect.width / 2, shaderRect.top),
              const <Color>[Color(0xFFFF0000), Color(0xFF042a85)],
              null,
              TileMode.repeated,
              Matrix4.rotationZ((angle / 180) * math.pi).toFloat64(),
            );
      rc.drawRect(shaderRect, SurfacePaint()..color = const Color(0xFF000000));
      rc.drawOval(shaderRect, paint);
      yOffset += 120;
    }
    expect(rc.renderStrategy.hasArbitraryPaint, isTrue);
    await canvasScreenshot(rc, 'linear_gradient_tiled_repeated_rect');
  }, skip: isFirefox);

  test('Should draw tiled mirrored linear gradient with transform.', () async {
    final RecordingCanvas rc = RecordingCanvas(const Rect.fromLTRB(0, 0, 500, 500));
    final List<double> angles = <double>[0.0, 30.0, 210.0];
    double yOffset = 0;
    for (final double angle in angles) {
      final Rect shaderRect = Rect.fromLTWH(50, 50 + yOffset, 100, 100);
      final SurfacePaint paint =
          SurfacePaint()
            ..shader = Gradient.linear(
              Offset(shaderRect.left, shaderRect.top),
              Offset(shaderRect.left + shaderRect.width / 2, shaderRect.top),
              const <Color>[Color(0xFFFF0000), Color(0xFF042a85)],
              null,
              TileMode.mirror,
              Matrix4.rotationZ((angle / 180) * math.pi).toFloat64(),
            );
      rc.drawRect(shaderRect, SurfacePaint()..color = const Color(0xFF000000));
      rc.drawOval(shaderRect, paint);
      yOffset += 120;
    }
    expect(rc.renderStrategy.hasArbitraryPaint, isTrue);
    await canvasScreenshot(rc, 'linear_gradient_tiled_mirrored_rect');
  }, skip: isFirefox);
}
