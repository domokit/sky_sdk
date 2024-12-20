// Copyright 2013 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

import 'package:ui/ui.dart' as ui;

import '../vector_math.dart';
import 'canvas.dart';

/// A cache of [Picture]s that have already been rasterized.
///
/// In the case of a [Picture] with a lot of complex drawing commands, it can
/// be faster to rasterize that [Picture] into it's own canvas and composite
/// that canvas into the scene rather than replay the drawing commands for that
/// picture into the overall scene.
///
/// This class is responsible for deciding if a [Picture] should be cached and
/// for creating the cached [Picture]s that can be drawn directly into the
/// canvas.
class RasterCache {
  /// Make a decision on whether to cache [picture] under transform [matrix].
  ///
  /// This is based on heuristics such as how many times [picture] has been
  /// drawn before and the complexity of the drawing commands in [picture].
  ///
  /// We also take into account the current transform [matrix], because, for
  /// example, a picture may be rasterized with the identity transform, but
  /// when it is used, the transform is a 3x scale. In this case, compositing
  /// the rendered picture would result in pixelation. So, we must use both
  /// the picture and the transform as a cache key.
  ///
  /// The flag [isComplex] is a hint to the raster cache that this picture
  /// contains complex drawing commands, and as such should be more strongly
  /// considered for caching.
  ///
  /// The flag [willChange] is a hint to the raster cache that this picture
  /// will change, and so should be less likely to be cached.
  void prepare(ui.Picture picture, Matrix4 matrix, bool isComplex, bool willChange) {}

  /// Gets a raster cache result for the [picture] at transform [matrix].
  RasterCacheResult get(ui.Picture picture, Matrix4 matrix) => RasterCacheResult();
}

/// A cache entry for a given picture and matrix.
class RasterCacheResult {
  /// Whether or not this result represents a rasterized picture that can be
  /// drawn into the scene.
  bool get isValid => false;

  /// Draws the rasterized picture into the [canvas].
  void draw(CkCanvas canvas) {}
}
