// Copyright 2014 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

/// @docImport 'package:flutter/material.dart';
///
/// @docImport 'app.dart';
library;

import 'package:flutter/rendering.dart';

import 'framework.dart';

/// Displays performance statistics.
///
/// The overlay shows two time series. The first shows how much time was
/// required on this thread to produce each frame. The second shows how much
/// time was required on the raster thread (formerly known as the GPU thread)
/// to produce each frame. Ideally, both these values would be less than
/// the total frame budget for the hardware on which the app is running.
/// For example, if the hardware has a screen that updates at 60 Hz, each
/// thread should ideally spend less than 16ms producing each frame.
/// This ideal condition is indicated by a green vertical line for each thread.
/// Otherwise, the performance overlay shows a red vertical line.
///
/// The simplest way to show the performance overlay is to set
/// [MaterialApp.showPerformanceOverlay] or [WidgetsApp.showPerformanceOverlay]
/// to true.
class PerformanceOverlay extends LeafRenderObjectWidget {
  /// Create a performance overlay that only displays specific statistics. The
  /// mask is created by shifting 1 by the index of the specific
  /// [PerformanceOverlayOption] to enable.
  const PerformanceOverlay({super.key, this.optionsMask = 0});

  /// Create a performance overlay that displays all available statistics.
  PerformanceOverlay.allEnabled({super.key})
    : optionsMask =
          1 << PerformanceOverlayOption.displayRasterizerStatistics.index |
          1 << PerformanceOverlayOption.visualizeRasterizerStatistics.index |
          1 << PerformanceOverlayOption.displayEngineStatistics.index |
          1 << PerformanceOverlayOption.visualizeEngineStatistics.index;

  /// The mask is created by shifting 1 by the index of the specific
  /// [PerformanceOverlayOption] to enable.
  final int optionsMask;

  @override
  RenderPerformanceOverlay createRenderObject(BuildContext context) =>
      RenderPerformanceOverlay(optionsMask: optionsMask);

  @override
  void updateRenderObject(BuildContext context, RenderPerformanceOverlay renderObject) {
    renderObject.optionsMask = optionsMask;
  }
}
