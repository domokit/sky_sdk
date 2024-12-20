// Copyright 2014 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

import 'dart:ui';

import 'recorder.dart';

/// Measure the performance of paint bounds estimation by recording a picture
/// without actually rendering it.
///
/// Bounds estimation is done in two phases:
///
/// * As we call drawing methods on `Canvas` we grow bounds with every paint op.
/// * When we're done recording a picture we call `PictureRecorder.endRecording`
///   at which point we compute the overall picture bounds and cache the result.
///
/// This benchmarks puts emphasis on paint operations that trigger expensive
/// math such as `transformLTRB`. To do that we push non-identity transforms
/// and rotations before calling drawing methods.
class BenchPictureRecording extends RawRecorder {
  BenchPictureRecording() : super(name: benchmarkName);

  static const String benchmarkName = 'bench_picture_recording';

  /// Cached paint used for drawing.
  ///
  /// We want to avoid polluting the results with paint initialization logic.
  late Paint paint;

  /// A prelaid out and cached paragraph.
  ///
  /// This is cached to remove text layout time from the benchmark time.
  late Paragraph paragraph;

  @override
  Future<void> setUpAll() async {
    paint = Paint();
    paragraph =
        (ParagraphBuilder(ParagraphStyle())..addText('abcd edfh ijkl mnop qrst uvwx yz')).build()
          ..layout(const ParagraphConstraints(width: 50));
  }

  @override
  void body(Profile profile) {
    final PictureRecorder recorder = PictureRecorder();
    final Canvas canvas = Canvas(recorder);
    profile.record('recordPaintCommands', () {
      for (int i = 1; i <= 100; i++) {
        canvas.translate((10 + i).toDouble(), (10 + i).toDouble());

        canvas.save();
        for (int j = 0; j < 10; j++) {
          canvas.drawRect(const Rect.fromLTWH(10, 10, 10, 10), paint);
          canvas.drawCircle(const Offset(50, 50), 50, paint);
          canvas.rotate(1.0);
        }
        canvas.restore();

        canvas.save();
        for (int j = 0; j < 10; j++) {
          canvas.translate(1, 1);
          canvas.clipRect(Rect.fromLTWH(20, 20, 40 / i, 40));
          canvas.drawRRect(
            RRect.fromRectAndRadius(const Rect.fromLTWH(10, 10, 10, 10), const Radius.circular(2)),
            paint,
          );
          canvas.drawParagraph(paragraph, Offset.zero);
        }
        canvas.restore();
      }
    }, reported: true);
    profile.record('estimatePaintBounds', () {
      recorder.endRecording();
    }, reported: true);
  }
}
