// Copyright 2013 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

import 'dart:typed_data';

import 'package:ui/ui.dart' as ui;

import 'conic.dart';
import 'path_ref.dart';
import 'path_utils.dart';

/// Converts [path] to SVG path syntax to be used as "d" attribute in path
/// element.
String pathToSvg(PathRef pathRef, {double offsetX = 0, double offsetY = 0}) {
  final StringBuffer buffer = StringBuffer();
  final PathRefIterator iter = PathRefIterator(pathRef);
  int verb = 0;
  final Float32List outPts = Float32List(PathRefIterator.kMaxBufferSize);
  while ((verb = iter.next(outPts)) != SPath.kDoneVerb) {
    switch (verb) {
      case SPath.kMoveVerb:
        buffer.write('M ${outPts[0] + offsetX} ${outPts[1] + offsetY}');
      case SPath.kLineVerb:
        buffer.write('L ${outPts[2] + offsetX} ${outPts[3] + offsetY}');
      case SPath.kCubicVerb:
        buffer.write(
          'C ${outPts[2] + offsetX} ${outPts[3] + offsetY} '
          '${outPts[4] + offsetX} ${outPts[5] + offsetY} ${outPts[6] + offsetX} ${outPts[7] + offsetY}',
        );
      case SPath.kQuadVerb:
        buffer.write(
          'Q ${outPts[2] + offsetX} ${outPts[3] + offsetY} '
          '${outPts[4] + offsetX} ${outPts[5] + offsetY}',
        );
      case SPath.kConicVerb:
        final double w = iter.conicWeight;
        final Conic conic = Conic(
          outPts[0],
          outPts[1],
          outPts[2],
          outPts[3],
          outPts[4],
          outPts[5],
          w,
        );
        final List<ui.Offset> points = conic.toQuads();
        final int len = points.length;
        for (int i = 1; i < len; i += 2) {
          final double p1x = points[i].dx;
          final double p1y = points[i].dy;
          final double p2x = points[i + 1].dx;
          final double p2y = points[i + 1].dy;
          buffer.write(
            'Q ${p1x + offsetX} ${p1y + offsetY} '
            '${p2x + offsetX} ${p2y + offsetY}',
          );
        }
      case SPath.kCloseVerb:
        buffer.write('Z');
      default:
        throw UnimplementedError('Unknown path verb $verb');
    }
  }
  return buffer.toString();
}
