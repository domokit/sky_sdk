// Copyright 2014 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

import 'dart:ui';

import 'recorder.dart';

/// Repeatedly paints a grid of rectangles where each rectangle is drawn in its
/// own [Picture].
///
/// Measures the performance of updating many layers.
///
/// See also `bench_draw_rect.dart`, which draws nearly identical UI but puts all
/// rectangles into the same picture.
class BenchUpdateManyChildLayers extends SceneBuilderRecorder {
  BenchUpdateManyChildLayers() : super(name: benchmarkName);

  static const String benchmarkName = 'bench_update_many_child_layers';

  /// Number of rows in the grid.
  static const int kRows = 32;

  /// Number of columns in the grid.
  static const int kColumns = 32;

  /// Counter used to offset the rendered rects to make them wobble.
  ///
  /// The wobbling is there so a human could visually verify that the benchmark
  /// is correctly pumping frames.
  double wobbleCounter = 0;

  late List<Picture> _pictures;
  late Size viewSize;
  late Size cellSize;
  late Size rectSize;

  @override
  Future<void> setUpAll() async {
    _pictures = <Picture>[];
    viewSize = view.physicalSize;
    cellSize = Size(viewSize.width / kColumns, viewSize.height / kRows);
    rectSize = cellSize * 0.8;

    final Paint paint = Paint()..color = const Color.fromARGB(255, 255, 0, 0);
    for (int i = 0; i < kRows * kColumns; i++) {
      final PictureRecorder pictureRecorder = PictureRecorder();
      final Canvas canvas = Canvas(pictureRecorder);
      canvas.drawRect(Offset.zero & rectSize, paint);
      _pictures.add(pictureRecorder.endRecording());
    }
  }

  OffsetEngineLayer? _rootLayer;
  final Map<int, OffsetEngineLayer> _layers = <int, OffsetEngineLayer>{};

  @override
  void onDrawFrame(SceneBuilder sceneBuilder) {
    _rootLayer = sceneBuilder.pushOffset(0, 0, oldLayer: _rootLayer);
    for (int row = 0; row < kRows; row++) {
      for (int col = 0; col < kColumns; col++) {
        final int layerId = 1000000 * row + col;
        final OffsetEngineLayer? oldLayer = _layers[layerId];
        final double wobbleOffsetX = col * cellSize.width + (wobbleCounter - 5).abs();
        final double offsetY = row * cellSize.height;
        // Retain every other layer, so we exercise the update path 50% of the
        // time and the retain path the other 50%.
        final bool shouldRetain = oldLayer != null && (row + col).isEven;
        if (shouldRetain) {
          sceneBuilder.addRetained(oldLayer);
        } else {
          _layers[layerId] = sceneBuilder.pushOffset(wobbleOffsetX, offsetY, oldLayer: oldLayer);
          sceneBuilder.addPicture(Offset.zero, _pictures[row * kColumns + col]);
          sceneBuilder.pop();
        }
      }
    }
    sceneBuilder.pop();
    wobbleCounter += 1;
    wobbleCounter = wobbleCounter % 10;
  }
}
