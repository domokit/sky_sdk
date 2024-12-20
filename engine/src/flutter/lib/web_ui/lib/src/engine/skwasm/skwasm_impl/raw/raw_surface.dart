// Copyright 2013 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

@DefaultAsset('skwasm')
library skwasm_impl;

import 'dart:ffi';
import 'package:ui/src/engine/skwasm/skwasm_impl.dart';

final class RawSurface extends Opaque {}

typedef SurfaceHandle = Pointer<RawSurface>;

final class RawRenderCallback extends Opaque {}

typedef OnRenderCallbackHandle = Pointer<RawRenderCallback>;

typedef CallbackId = int;

@Native<SurfaceHandle Function()>(symbol: 'surface_create', isLeaf: true)
external SurfaceHandle surfaceCreate();

@Native<UnsignedLong Function(SurfaceHandle)>(symbol: 'surface_getThreadId', isLeaf: true)
external int surfaceGetThreadId(SurfaceHandle handle);

@Native<Void Function(SurfaceHandle, OnRenderCallbackHandle)>(
  symbol: 'surface_setCallbackHandler',
  isLeaf: true,
)
external void surfaceSetCallbackHandler(SurfaceHandle surface, OnRenderCallbackHandle callback);

@Native<Void Function(SurfaceHandle)>(symbol: 'surface_destroy', isLeaf: true)
external void surfaceDestroy(SurfaceHandle surface);

@Native<Int32 Function(SurfaceHandle, Pointer<PictureHandle>, Int)>(
  symbol: 'surface_renderPictures',
  isLeaf: true,
)
external CallbackId surfaceRenderPictures(
  SurfaceHandle surface,
  Pointer<PictureHandle> picture,
  int count,
);

@Native<Int32 Function(SurfaceHandle, ImageHandle, Int)>(
  symbol: 'surface_rasterizeImage',
  isLeaf: true,
)
external CallbackId surfaceRasterizeImage(SurfaceHandle handle, ImageHandle image, int format);
