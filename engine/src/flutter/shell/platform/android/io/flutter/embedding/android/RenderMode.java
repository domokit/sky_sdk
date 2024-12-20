// Copyright 2013 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

package io.flutter.embedding.android;

/** Render modes for a Flutter UI. */
public enum RenderMode {
  /**
   * {@code RenderMode}, which paints a Flutter UI to a {@link android.view.SurfaceView}. This mode
   * has the best performance, but a Flutter UI in this mode cannot be positioned between 2 other
   * Android {@code View}s in the z-index, nor can it be animated/transformed. Unless the special
   * capabilities of a {@link android.graphics.SurfaceTexture} are required, developers should
   * strongly prefer this render mode.
   */
  surface,
  /**
   * {@code RenderMode}, which paints a Flutter UI to a {@link android.graphics.SurfaceTexture}.
   * This mode is not as performant as {@link #surface}, but a Flutter UI in this mode can be
   * animated and transformed, as well as positioned in the z-index between 2+ other Android {@code
   * Views}. Unless the special capabilities of a {@link android.graphics.SurfaceTexture} are
   * required, developers should strongly prefer the {@link #surface} render mode.
   */
  texture,
  /**
   * {@code RenderMode}, which paints Paints a Flutter UI provided by an {@link
   * android.media.ImageReader} onto a {@link android.graphics.Canvas}. This mode is not as
   * performant as {@link RenderMode#surface}, but a {@code FlutterView} in this mode can handle
   * full interactivity with a {@link io.flutter.plugin.platform.PlatformView}. Unless {@link
   * io.flutter.plugin.platform.PlatformView}s are required developers should strongly prefer the
   * {@link RenderMode#surface} render mode.
   */
  image
}
