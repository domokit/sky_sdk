// Copyright 2013 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

package io.flutter.plugin.platform;

import android.view.Surface;

/**
 * A PlatformViewRenderTarget interface allows an Android Platform View to be rendered into an
 * offscreen buffer (usually a texture is involved) that the engine can compose into the
 * FlutterView.
 */
public interface PlatformViewRenderTarget {
  // Called when the render target should be resized.
  public void resize(int width, int height);

  // Returns the currently specified width.
  public int getWidth();

  // Returns the currently specified height.
  public int getHeight();

  // The id of this render target.
  public long getId();

  // Releases backing resources.
  public void release();

  // Returns true in the case that backing resource have been released.
  public boolean isReleased();

  // Returns the Surface to be rendered on to.
  public Surface getSurface();

  // Schedules a frame to be drawn.
  public default void scheduleFrame() {}
}
