// Copyright 2013 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

import 'dart:math' as math;

import 'package:ui/ui.dart' as ui;

import 'dom.dart';
import 'util.dart';

/// Renders an RRect using path primitives.
abstract class RRectRenderer {
  // To draw the rounded rectangle, perform the following steps:
  //   0. Ensure border radius don't overlap
  //   1. Flip left,right top,bottom since web doesn't support flipped
  //      coordinates with negative radii.
  //   2. draw the line for the top
  //   3. draw the arc for the top-right corner
  //   4. draw the line for the right side
  //   5. draw the arc for the bottom-right corner
  //   6. draw the line for the bottom of the rectangle
  //   7. draw the arc for the bottom-left corner
  //   8. draw the line for the left side
  //   9. draw the arc for the top-left corner
  //
  // After drawing, the current point will be the left side of the top of the
  // rounded rectangle (after the corner).
  // TODO(het): Confirm that this is the end point in Flutter for RRect

  void render(ui.RRect inputRRect, {bool startNewPath = true, bool reverse = false}) {
    // Ensure border radius curves never overlap
    final ui.RRect rrect = inputRRect.scaleRadii();

    double left = rrect.left;
    double right = rrect.right;
    double top = rrect.top;
    double bottom = rrect.bottom;
    if (left > right) {
      left = right;
      right = rrect.left;
    }
    if (top > bottom) {
      top = bottom;
      bottom = rrect.top;
    }
    final double trRadiusX = rrect.trRadiusX.abs();
    final double tlRadiusX = rrect.tlRadiusX.abs();
    final double trRadiusY = rrect.trRadiusY.abs();
    final double tlRadiusY = rrect.tlRadiusY.abs();
    final double blRadiusX = rrect.blRadiusX.abs();
    final double brRadiusX = rrect.brRadiusX.abs();
    final double blRadiusY = rrect.blRadiusY.abs();
    final double brRadiusY = rrect.brRadiusY.abs();

    if (!reverse) {
      if (startNewPath) {
        beginPath();
      }

      moveTo(left + trRadiusX, top);

      // Top side and top-right corner
      lineTo(right - trRadiusX, top);
      ellipse(
        right - trRadiusX,
        top + trRadiusY,
        trRadiusX,
        trRadiusY,
        0,
        1.5 * math.pi,
        2.0 * math.pi,
        false,
      );

      // Right side and bottom-right corner
      lineTo(right, bottom - brRadiusY);
      ellipse(
        right - brRadiusX,
        bottom - brRadiusY,
        brRadiusX,
        brRadiusY,
        0,
        0,
        0.5 * math.pi,
        false,
      );

      // Bottom side and bottom-left corner
      lineTo(left + blRadiusX, bottom);
      ellipse(
        left + blRadiusX,
        bottom - blRadiusY,
        blRadiusX,
        blRadiusY,
        0,
        0.5 * math.pi,
        math.pi,
        false,
      );

      // Left side and top-left corner
      lineTo(left, top + tlRadiusY);
      ellipse(
        left + tlRadiusX,
        top + tlRadiusY,
        tlRadiusX,
        tlRadiusY,
        0,
        math.pi,
        1.5 * math.pi,
        false,
      );
    } else {
      // Draw the rounded rectangle, counterclockwise.
      moveTo(right - trRadiusX, top);

      if (startNewPath) {
        beginPath();
      }

      // Top side and top-left corner
      lineTo(left + tlRadiusX, top);
      ellipse(
        left + tlRadiusX,
        top + tlRadiusY,
        tlRadiusX,
        tlRadiusY,
        0,
        1.5 * math.pi,
        1 * math.pi,
        true,
      );

      // Left side and bottom-left corner
      lineTo(left, bottom - blRadiusY);
      ellipse(
        left + blRadiusX,
        bottom - blRadiusY,
        blRadiusX,
        blRadiusY,
        0,
        1 * math.pi,
        0.5 * math.pi,
        true,
      );

      // Bottom side and bottom-right corner
      lineTo(right - brRadiusX, bottom);
      ellipse(
        right - brRadiusX,
        bottom - brRadiusY,
        brRadiusX,
        brRadiusY,
        0,
        0.5 * math.pi,
        0 * math.pi,
        true,
      );

      // Right side and top-right corner
      lineTo(right, top + trRadiusY);
      ellipse(
        right - trRadiusX,
        top + trRadiusY,
        trRadiusX,
        trRadiusY,
        0,
        0 * math.pi,
        1.5 * math.pi,
        true,
      );
    }
  }

  void beginPath();
  void moveTo(double x, double y);
  void lineTo(double x, double y);
  void ellipse(
    double centerX,
    double centerY,
    double radiusX,
    double radiusY,
    double rotation,
    double startAngle,
    double endAngle,
    bool antiClockwise,
  );
}

/// Renders RRect to a 2d canvas.
class RRectToCanvasRenderer extends RRectRenderer {
  RRectToCanvasRenderer(this.context);
  final DomCanvasRenderingContext2D context;
  @override
  void beginPath() {
    context.beginPath();
  }

  @override
  void moveTo(double x, double y) {
    context.moveTo(x, y);
  }

  @override
  void lineTo(double x, double y) {
    context.lineTo(x, y);
  }

  @override
  void ellipse(
    double centerX,
    double centerY,
    double radiusX,
    double radiusY,
    double rotation,
    double startAngle,
    double endAngle,
    bool antiClockwise,
  ) {
    drawEllipse(
      context,
      centerX,
      centerY,
      radiusX,
      radiusY,
      rotation,
      startAngle,
      endAngle,
      antiClockwise,
    );
  }
}

/// Renders RRect to a path.
class RRectToPathRenderer extends RRectRenderer {
  RRectToPathRenderer(this.path);
  final ui.Path path;
  @override
  void beginPath() {}

  @override
  void moveTo(double x, double y) {
    path.moveTo(x, y);
  }

  @override
  void lineTo(double x, double y) {
    path.lineTo(x, y);
  }

  @override
  void ellipse(
    double centerX,
    double centerY,
    double radiusX,
    double radiusY,
    double rotation,
    double startAngle,
    double endAngle,
    bool antiClockwise,
  ) {
    path.addArc(
      ui.Rect.fromLTRB(centerX - radiusX, centerY - radiusY, centerX + radiusX, centerY + radiusY),
      startAngle,
      antiClockwise ? startAngle - endAngle : endAngle - startAngle,
    );
  }
}

typedef RRectRendererEllipseCallback =
    void Function(
      double centerX,
      double centerY,
      double radiusX,
      double radiusY,
      double rotation,
      double startAngle,
      double endAngle,
      bool antiClockwise,
    );
typedef RRectRendererCallback = void Function(double x, double y);

/// Converts RRect to path primitives with callbacks.
class RRectMetricsRenderer extends RRectRenderer {
  RRectMetricsRenderer({this.moveToCallback, this.lineToCallback, this.ellipseCallback});

  final RRectRendererEllipseCallback? ellipseCallback;
  final RRectRendererCallback? lineToCallback;
  final RRectRendererCallback? moveToCallback;
  @override
  void beginPath() {}

  @override
  void ellipse(
    double centerX,
    double centerY,
    double radiusX,
    double radiusY,
    double rotation,
    double startAngle,
    double endAngle,
    bool antiClockwise,
  ) => ellipseCallback!(
    centerX,
    centerY,
    radiusX,
    radiusY,
    rotation,
    startAngle,
    endAngle,
    antiClockwise,
  );

  @override
  void lineTo(double x, double y) => lineToCallback!(x, y);

  @override
  void moveTo(double x, double y) => moveToCallback!(x, y);
}
