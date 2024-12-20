// Copyright 2013 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

package io.flutter.embedding.android;

/**
 * Transparency mode for a Flutter UI.
 *
 * <p>{@code TransparencyMode} impacts the visual behavior and performance of a {@link
 * FlutterSurfaceView}, which is displayed when a Flutter UI uses {@link RenderMode#surface}.
 *
 * <p>{@code TransparencyMode} does not impact {@link FlutterTextureView}, which is displayed when a
 * Flutter UI uses {@link RenderMode#texture}, because a {@link FlutterTextureView} automatically
 * comes with transparency.
 */
public enum TransparencyMode {
  /**
   * Renders a Flutter UI without any transparency. This affects Flutter UI's with {@link
   * RenderMode#surface} by introducing a base color of black, and places the {@link
   * FlutterSurfaceView}'s {@code Window} behind all other content.
   *
   * <p>In {@link RenderMode#surface}, this mode is the most performant and is a good choice for
   * fullscreen Flutter UIs that will not undergo {@code Fragment} transactions. If this mode is
   * used within a {@code Fragment}, and that {@code Fragment} is replaced by another one, a brief
   * black flicker may be visible during the switch.
   */
  opaque,
  /**
   * Renders a Flutter UI with transparency. This affects Flutter UI's in {@link RenderMode#surface}
   * by allowing background transparency, and places the {@link FlutterSurfaceView}'s {@code Window}
   * on top of all other content.
   *
   * <p>In {@link RenderMode#surface}, this mode is less performant than {@link #opaque}, but this
   * mode avoids the black flicker problem that {@link #opaque} has when going through {@code
   * Fragment} transactions. Consider using this {@code TransparencyMode} if you intend to switch
   * {@code Fragment}s at runtime that contain a Flutter UI.
   */
  transparent
}
