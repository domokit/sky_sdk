// Copyright 2013 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

import 'package:ui/ui.dart' as ui;

import 'vector_math.dart';

// These are additional APIs that are not part of the `dart:ui` interface that
// are needed internally to properly implement a `SceneBuilder` on top of the
// generic Canvas/Picture api.
abstract class SceneCanvas implements ui.Canvas {
  // This is the same as a normal `saveLayer` call, but we can pass a backdrop image filter.
  void saveLayerWithFilter(ui.Rect? bounds, ui.Paint paint, ui.ImageFilter backdropFilter);
}

abstract class ScenePicture implements ui.Picture {
  // This is a conservative bounding box of all the drawing primitives in this picture.
  ui.Rect get cullRect;
}

abstract class SceneImageFilter implements ui.ImageFilter {
  // Since some image filters affect the actual drawing bounds of a given picture, this
  // gives the maximum draw boundary for a picture with the given input bounds after it
  // has been processed by the filter.
  ui.Rect filterBounds(ui.Rect inputBounds);

  // The matrix image filter changes the position of the content, so when positioning
  // platform views and calculating occlusion we need to take its transform into account.
  Matrix4? get transform;
}

abstract class ScenePath implements ui.Path {
  // In order to properly clip platform views with paths, we need to be able to get a
  // string representation of them.
  String toSvgString();
}
