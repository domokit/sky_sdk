// Copyright 2013 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

import 'package:ui/src/engine.dart';
import 'package:ui/src/engine/skwasm/skwasm_impl.dart';
import 'package:ui/ui.dart' as ui;

class SkwasmPicture extends SkwasmObjectWrapper<RawPicture> implements ScenePicture {
  SkwasmPicture.fromHandle(PictureHandle handle) : super(handle, _registry);

  static final SkwasmFinalizationRegistry<RawPicture> _registry =
      SkwasmFinalizationRegistry<RawPicture>(pictureDispose);

  @override
  Future<ui.Image> toImage(int width, int height) async => toImageSync(width, height);

  @override
  int get approximateBytesUsed => pictureApproximateBytesUsed(handle);

  @override
  void dispose() {
    super.dispose();
    ui.Picture.onDispose?.call(this);
  }

  @override
  ui.Image toImageSync(int width, int height) =>
      SkwasmImage(imageCreateFromPicture(handle, width, height));

  @override
  ui.Rect get cullRect {
    return withStackScope((StackScope s) {
      final RawRect rect = s.allocFloatArray(4);
      pictureGetCullRect(handle, rect);
      return s.convertRectFromNative(rect);
    });
  }
}

class SkwasmPictureRecorder extends SkwasmObjectWrapper<RawPictureRecorder>
    implements ui.PictureRecorder {
  SkwasmPictureRecorder() : super(pictureRecorderCreate(), _registry);

  static final SkwasmFinalizationRegistry<RawPictureRecorder> _registry =
      SkwasmFinalizationRegistry<RawPictureRecorder>(pictureRecorderDispose);

  @override
  SkwasmPicture endRecording() {
    isRecording = false;

    final SkwasmPicture picture = SkwasmPicture.fromHandle(pictureRecorderEndRecording(handle));
    ui.Picture.onCreate?.call(picture);
    return picture;
  }

  @override
  bool isRecording = true;
}
