// Copyright 2013 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

import 'dart:async';

import '../display.dart';
import '../dom.dart';
import 'bitmap_canvas.dart';
import 'dom_canvas.dart';
import 'picture.dart';
import 'scene.dart';
import 'surface.dart';

/// Surfaces that were retained this frame.
///
/// Surfaces should be added to this list directly. Instead, if a surface needs
/// to be retained call [_retainSurface].
List<PersistedSurface> retainedSurfaces = <PersistedSurface>[];

/// Maps every surface currently active on the screen to debug statistics.
Map<PersistedSurface, DebugSurfaceStats> surfaceStats = <PersistedSurface, DebugSurfaceStats>{};

List<Map<PersistedSurface, DebugSurfaceStats>> _surfaceStatsTimeline =
    <Map<PersistedSurface, DebugSurfaceStats>>[];

/// Returns debug statistics for the given [surface].
DebugSurfaceStats surfaceStatsFor(PersistedSurface surface) {
  if (!debugExplainSurfaceStats) {
    throw Exception(
      'surfaceStatsFor is only available when debugExplainSurfaceStats is set to true.',
    );
  }
  return surfaceStats.putIfAbsent(surface, () => DebugSurfaceStats(surface));
}

/// Compositor information collected for one frame useful for assessing the
/// efficiency of constructing the frame.
///
/// This information is only available in debug mode.
///
/// For stats pertaining to a single surface the numeric counter fields are
/// typically either 0 or 1. For aggregated stats, the numbers can be >1.
class DebugSurfaceStats {
  DebugSurfaceStats(this.surface);

  /// The surface these stats are for, or `null` if these are aggregated stats.
  final PersistedSurface? surface;

  /// How many times a surface was retained from a previously rendered frame.
  int retainSurfaceCount = 0;

  /// How many times a surface reused an HTML element from a previously rendered
  /// surface.
  int reuseElementCount = 0;

  /// If a surface is a [PersistedPicture], how many times it painted.
  int paintCount = 0;

  /// If a surface is a [PersistedPicture], how many pixels it painted.
  int paintPixelCount = 0;

  /// If a surface is a [PersistedPicture], how many times it reused a
  /// previously allocated `<canvas>` element when it painted.
  int reuseCanvasCount = 0;

  /// If a surface is a [PersistedPicture], how many times it allocated a new
  /// bitmap canvas.
  int allocateBitmapCanvasCount = 0;

  /// If a surface is a [PersistedPicture], how many pixels it allocated for
  /// the bitmap.
  ///
  /// For aggregated stats, this is the total sum of all pixels across all
  /// canvases.
  int allocatedBitmapSizeInPixels = 0;

  /// The number of HTML DOM nodes a surface allocated.
  ///
  /// For aggregated stats, this is the total sum of all DOM nodes across all
  /// surfaces.
  int allocatedDomNodeCount = 0;

  /// Adds all counters of [oneSurfaceStats] into this object.
  void aggregate(DebugSurfaceStats oneSurfaceStats) {
    retainSurfaceCount += oneSurfaceStats.retainSurfaceCount;
    reuseElementCount += oneSurfaceStats.reuseElementCount;
    paintCount += oneSurfaceStats.paintCount;
    paintPixelCount += oneSurfaceStats.paintPixelCount;
    reuseCanvasCount += oneSurfaceStats.reuseCanvasCount;
    allocateBitmapCanvasCount += oneSurfaceStats.allocateBitmapCanvasCount;
    allocatedBitmapSizeInPixels += oneSurfaceStats.allocatedBitmapSizeInPixels;
    allocatedDomNodeCount += oneSurfaceStats.allocatedDomNodeCount;
  }
}

DomCanvasRenderingContext2D? _debugSurfaceStatsOverlayCtx;

void debugRepaintSurfaceStatsOverlay(PersistedScene scene) {
  final int overlayWidth = domWindow.innerWidth!.toInt();
  const int rowHeight = 30;
  const int rowCount = 4;
  const int overlayHeight = rowHeight * rowCount;
  const int strokeWidth = 2;

  _surfaceStatsTimeline.add(surfaceStats);

  while (_surfaceStatsTimeline.length > (overlayWidth / strokeWidth)) {
    _surfaceStatsTimeline.removeAt(0);
  }

  if (_debugSurfaceStatsOverlayCtx == null) {
    final DomCanvasElement debugSurfaceStatsOverlay = createDomCanvasElement(
      width: overlayWidth,
      height: overlayHeight,
    );
    debugSurfaceStatsOverlay.style
      ..position = 'fixed'
      ..left = '0'
      ..top = '0'
      ..zIndex = '1000'
      ..opacity = '0.8';
    _debugSurfaceStatsOverlayCtx = debugSurfaceStatsOverlay.context2D;
    domDocument.body!.append(debugSurfaceStatsOverlay);
  }

  _debugSurfaceStatsOverlayCtx!
    ..fillStyle = 'black'
    ..beginPath()
    ..rect(0, 0, overlayWidth, overlayHeight)
    ..fill();

  final double physicalScreenWidth =
      domWindow.innerWidth! * EngineFlutterDisplay.instance.browserDevicePixelRatio;
  final double physicalScreenHeight =
      domWindow.innerHeight! * EngineFlutterDisplay.instance.browserDevicePixelRatio;
  final double physicsScreenPixelCount = physicalScreenWidth * physicalScreenHeight;

  final int totalDomNodeCount = scene.rootElement!.querySelectorAll('*').length;

  for (int i = 0; i < _surfaceStatsTimeline.length; i++) {
    final Map<PersistedSurface, DebugSurfaceStats> statsMap = _surfaceStatsTimeline[i];
    final DebugSurfaceStats totals = DebugSurfaceStats(null);
    int pixelCount = 0;
    for (final DebugSurfaceStats oneSurfaceStats in statsMap.values) {
      totals.aggregate(oneSurfaceStats);
      if (oneSurfaceStats.surface is PersistedPicture) {
        final PersistedPicture picture = oneSurfaceStats.surface! as PersistedPicture;
        pixelCount += picture.bitmapPixelCount;
      }
    }

    final double repaintRate = totals.paintPixelCount / pixelCount;
    final double domAllocationRate = totals.allocatedDomNodeCount / totalDomNodeCount;
    final double bitmapAllocationRate =
        totals.allocatedBitmapSizeInPixels / physicsScreenPixelCount;
    final double surfaceRetainRate = totals.retainSurfaceCount / _surfaceStatsTimeline[i].length;

    // Repaints
    _debugSurfaceStatsOverlayCtx!
      ..lineWidth = strokeWidth
      ..strokeStyle = 'red'
      ..beginPath()
      ..moveTo(strokeWidth * i, rowHeight)
      ..lineTo(strokeWidth * i, rowHeight * (1 - repaintRate))
      ..stroke();

    // DOM allocations
    _debugSurfaceStatsOverlayCtx!
      ..lineWidth = strokeWidth
      ..strokeStyle = 'red'
      ..beginPath()
      ..moveTo(strokeWidth * i, 2 * rowHeight)
      ..lineTo(strokeWidth * i, rowHeight * (2 - domAllocationRate))
      ..stroke();

    // Bitmap allocations
    _debugSurfaceStatsOverlayCtx!
      ..lineWidth = strokeWidth
      ..strokeStyle = 'red'
      ..beginPath()
      ..moveTo(strokeWidth * i, 3 * rowHeight)
      ..lineTo(strokeWidth * i, rowHeight * (3 - bitmapAllocationRate))
      ..stroke();

    // Surface retentions
    _debugSurfaceStatsOverlayCtx!
      ..lineWidth = strokeWidth
      ..strokeStyle = 'green'
      ..beginPath()
      ..moveTo(strokeWidth * i, 4 * rowHeight)
      ..lineTo(strokeWidth * i, rowHeight * (4 - surfaceRetainRate))
      ..stroke();
  }

  _debugSurfaceStatsOverlayCtx!
    ..font = 'normal normal 14px sans-serif'
    ..fillStyle = 'white'
    ..fillText('Repaint rate', 5, rowHeight - 5)
    ..fillText('DOM alloc rate', 5, 2 * rowHeight - 5)
    ..fillText('Bitmap alloc rate', 5, 3 * rowHeight - 5)
    ..fillText('Retain rate', 5, 4 * rowHeight - 5);

  for (int i = 1; i <= rowCount; i++) {
    _debugSurfaceStatsOverlayCtx!
      ..lineWidth = 1
      ..strokeStyle = 'blue'
      ..beginPath()
      ..moveTo(0, overlayHeight - rowHeight * i)
      ..lineTo(overlayWidth, overlayHeight - rowHeight * i)
      ..stroke();
  }
}

/// Prints debug statistics for the current frame to the console.
void debugPrintSurfaceStats(PersistedScene scene, int frameNumber) {
  int pictureCount = 0;
  int paintCount = 0;

  int bitmapCanvasCount = 0;
  int bitmapReuseCount = 0;
  int bitmapAllocationCount = 0;
  int bitmapPaintCount = 0;
  int bitmapPixelsAllocated = 0;

  int domCanvasCount = 0;
  int domPaintCount = 0;

  int surfaceRetainCount = 0;
  int elementReuseCount = 0;

  int totalAllocatedDomNodeCount = 0;

  void countReusesRecursively(PersistedSurface surface) {
    final DebugSurfaceStats stats = surfaceStatsFor(surface);

    surfaceRetainCount += stats.retainSurfaceCount;
    elementReuseCount += stats.reuseElementCount;
    totalAllocatedDomNodeCount += stats.allocatedDomNodeCount;

    if (surface is PersistedPicture) {
      pictureCount += 1;
      paintCount += stats.paintCount;

      if (surface.canvas is DomCanvas) {
        domCanvasCount++;
        domPaintCount += stats.paintCount;
      }

      if (surface.canvas is BitmapCanvas) {
        bitmapCanvasCount++;
        bitmapPaintCount += stats.paintCount;
      }

      bitmapReuseCount += stats.reuseCanvasCount;
      bitmapAllocationCount += stats.allocateBitmapCanvasCount;
      bitmapPixelsAllocated += stats.allocatedBitmapSizeInPixels;
    }

    surface.visitChildren(countReusesRecursively);
  }

  scene.visitChildren(countReusesRecursively);

  final StringBuffer buf = StringBuffer();
  buf
    ..writeln('---------------------- FRAME #$frameNumber -------------------------')
    ..writeln('Surfaces retained: $surfaceRetainCount')
    ..writeln('Elements reused: $elementReuseCount')
    ..writeln('Elements allocated: $totalAllocatedDomNodeCount')
    ..writeln('Pictures: $pictureCount')
    ..writeln('  Painted: $paintCount')
    ..writeln('  Skipped painting: ${pictureCount - paintCount}')
    ..writeln('DOM canvases:')
    ..writeln('  Painted: $domPaintCount')
    ..writeln('  Skipped painting: ${domCanvasCount - domPaintCount}')
    ..writeln('Bitmap canvases: $bitmapCanvasCount')
    ..writeln('  Painted: $bitmapPaintCount')
    ..writeln('  Skipped painting: ${bitmapCanvasCount - bitmapPaintCount}')
    ..writeln('  Reused: $bitmapReuseCount')
    ..writeln('  Allocated: $bitmapAllocationCount')
    ..writeln('  Allocated pixels: $bitmapPixelsAllocated')
    ..writeln('  Available for reuse: ${recycledCanvases.length}');

  // A microtask will fire after the DOM is flushed, letting us probe into
  // actual <canvas> tags.
  scheduleMicrotask(() {
    final Iterable<DomElement> canvasElements = domDocument.querySelectorAll('canvas');
    final StringBuffer canvasInfo = StringBuffer();
    final int pixelCount = canvasElements
        .cast<DomCanvasElement>()
        .map<int>((DomCanvasElement e) {
          final int pixels = (e.width! * e.height!).toInt();
          canvasInfo.writeln('    - ${e.width!} x ${e.height!} = $pixels pixels');
          return pixels;
        })
        .fold(0, (int total, int pixels) => total + pixels);
    final double physicalScreenWidth =
        domWindow.innerWidth! * EngineFlutterDisplay.instance.browserDevicePixelRatio;
    final double physicalScreenHeight =
        domWindow.innerHeight! * EngineFlutterDisplay.instance.browserDevicePixelRatio;
    final double physicsScreenPixelCount = physicalScreenWidth * physicalScreenHeight;
    final double screenPixelRatio = pixelCount / physicsScreenPixelCount;
    final String screenDescription =
        '1 screen is $physicalScreenWidth x $physicalScreenHeight = $physicsScreenPixelCount pixels';
    final String canvasPixelDescription =
        '$pixelCount (${screenPixelRatio.toStringAsFixed(2)} x screens';
    buf
      ..writeln('  Elements: ${canvasElements.length}')
      ..writeln(canvasInfo)
      ..writeln('  Pixels: $canvasPixelDescription; $screenDescription)')
      ..writeln('-----------------------------------------------------------');
    final bool screenPixelRatioTooHigh = screenPixelRatio > kScreenPixelRatioWarningThreshold;
    if (screenPixelRatioTooHigh) {
      print('WARNING: pixel/screen ratio too high (${screenPixelRatio.toStringAsFixed(2)}x)');
    }
    print(buf);
  });
}
