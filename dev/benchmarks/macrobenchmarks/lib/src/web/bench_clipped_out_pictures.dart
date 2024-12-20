// Copyright 2014 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

import 'dart:math' as math;
import 'dart:ui';

import 'recorder.dart';

/// Draws many pictures in a grid with only the middle picture visible on the
/// screen, all others are clipped out, for example:
///
///     +-------------+-------------+-------------+---...
///     |             |             |             |
///     |  invisible  |  invisible  |  invisible  |
///     |             |             |             |
///     +-----------------------------------------+---...
///     |             |             |             |
///     |  invisible  |  invisible  |  invisible  |
///     |             |             |             |
///     +-----------------------------------------+---...
///     |             |             |             |
///     |  invisible  |  invisible  |   VISIBLE   |
///     |             |             |             |
///     +-------------+-------------+-------------+---...
///     |             |             |             |
///     :             :             :             :
///
/// We used to unnecessarily allocate DOM nodes, consuming memory and CPU time.
class BenchClippedOutPictures extends SceneBuilderRecorder {
  BenchClippedOutPictures() : super(name: benchmarkName);

  static const String benchmarkName = 'clipped_out_pictures';

  static final Paint paint = Paint();

  double angle = 0.0;

  static const int kRows = 20;
  static const int kColumns = 20;

  @override
  void onDrawFrame(SceneBuilder sceneBuilder) {
    final Size viewSize = view.physicalSize / view.devicePixelRatio;
    final Size pictureSize = Size(viewSize.width / kColumns, viewSize.height / kRows);

    // Fills a single cell with random text.
    void fillCell(int row, int column) {
      sceneBuilder.pushOffset(column * pictureSize.width, row * pictureSize.height);

      final PictureRecorder pictureRecorder = PictureRecorder();
      final Canvas canvas = Canvas(pictureRecorder);
      canvas.save();
      canvas.drawCircle(Offset(pictureSize.width / 2, pictureSize.height / 2), 5.0, paint);
      canvas.drawRect(
        Rect.fromCenter(
          center: Offset(pictureSize.width / 2, pictureSize.height / 2),
          width: pictureSize.width / 6,
          height: pictureSize.height / 6,
        ),
        paint,
      );
      canvas.restore();
      final Picture picture = pictureRecorder.endRecording();
      sceneBuilder.addPicture(Offset.zero, picture);
      sceneBuilder.pop();
    }

    // Starting with the top-left cell, fill every cell.
    sceneBuilder.pushClipRect(
      Rect.fromCircle(
        center: Offset(viewSize.width / 2, viewSize.height / 2),
        radius: math.min(viewSize.width, viewSize.height) / 6,
      ),
    );
    sceneBuilder.pushOffset(5.0 * math.cos(angle), 5.0 * math.sin(angle));
    angle += math.pi / 20;
    for (int row = 0; row < 10; row++) {
      for (int column = 0; column < 10; column++) {
        fillCell(row, column);
      }
    }
    sceneBuilder.pop();
    sceneBuilder.pop();
  }
}
