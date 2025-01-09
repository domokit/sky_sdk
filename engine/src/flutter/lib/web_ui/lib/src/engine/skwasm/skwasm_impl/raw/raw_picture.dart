// Copyright 2013 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

@DefaultAsset('skwasm')
library skwasm_impl;

import 'dart:ffi';

import 'package:ui/src/engine/skwasm/skwasm_impl.dart';

final class RawPictureRecorder extends Opaque {}

typedef PictureRecorderHandle = Pointer<RawPictureRecorder>;

final class RawPicture extends Opaque {}

typedef PictureHandle = Pointer<RawPicture>;

@Native<PictureRecorderHandle Function()>(symbol: 'pictureRecorder_create', isLeaf: true)
external PictureRecorderHandle pictureRecorderCreate();

@Native<Void Function(PictureRecorderHandle)>(symbol: 'pictureRecorder_dispose', isLeaf: true)
external void pictureRecorderDispose(PictureRecorderHandle picture);

@Native<CanvasHandle Function(PictureRecorderHandle, RawRect)>(
  symbol: 'pictureRecorder_beginRecording',
  isLeaf: true,
)
external CanvasHandle pictureRecorderBeginRecording(
  PictureRecorderHandle picture,
  RawRect cullRect,
);

@Native<PictureHandle Function(PictureRecorderHandle)>(
  symbol: 'pictureRecorder_endRecording',
  isLeaf: true,
)
external PictureHandle pictureRecorderEndRecording(PictureRecorderHandle picture);

@Native<Void Function(PictureHandle)>(symbol: 'picture_dispose', isLeaf: true)
external void pictureDispose(PictureHandle handle);

@Native<Uint32 Function(PictureHandle)>(symbol: 'picture_approximateBytesUsed', isLeaf: true)
external int pictureApproximateBytesUsed(PictureHandle handle);

@Native<Void Function(PictureHandle, RawRect)>(symbol: 'picture_getCullRect', isLeaf: true)
external void pictureGetCullRect(PictureHandle handle, RawRect outRect);
