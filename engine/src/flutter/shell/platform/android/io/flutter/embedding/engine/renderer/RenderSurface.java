// Copyright 2013 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

package io.flutter.embedding.engine.renderer;

import android.view.Surface;
import androidx.annotation.NonNull;
import androidx.annotation.Nullable;

/**
 * Owns a {@code Surface} that {@code FlutterRenderer} would like to paint.
 *
 * <p>{@code RenderSurface} is responsible for providing a {@code Surface} to a given {@code
 * FlutterRenderer} when requested, and then notify that {@code FlutterRenderer} when the {@code
 * Surface} changes, or is destroyed.
 *
 * <p>The behavior of providing a {@code Surface} is delegated to this interface because the timing
 * of a {@code Surface}'s availability is determined by Android. Therefore, an accessor method would
 * not fulfill the requirements. Therefore, a {@code RenderSurface} is given a {@code
 * FlutterRenderer}, which the {@code RenderSurface} is expected to notify as a {@code Surface}
 * becomes available, changes, or is destroyed.
 */
public interface RenderSurface {
  /**
   * Returns the {@code FlutterRenderer} that is attached to this {@code RenderSurface}, or null if
   * no {@code FlutterRenderer} is currently attached.
   */
  @Nullable
  FlutterRenderer getAttachedRenderer();

  /**
   * Instructs this {@code RenderSurface} to give its {@code Surface} to the given {@code
   * FlutterRenderer} so that Flutter can paint pixels on it.
   *
   * <p>After this call, {@code RenderSurface} is expected to invoke the following methods on {@link
   * FlutterRenderer} at the appropriate times:
   *
   * <ol>
   *   <li>{@link FlutterRenderer#startRenderingToSurface(Surface, boolean)}
   *   <li>{@link FlutterRenderer#surfaceChanged(int, int)}}
   *   <li>{@link FlutterRenderer#stopRenderingToSurface()}
   * </ol>
   */
  void attachToRenderer(@NonNull FlutterRenderer renderer);

  /**
   * Instructs this {@code RenderSurface} to stop forwarding {@code Surface} notifications to the
   * {@code FlutterRenderer} that was previously connected with {@link
   * #attachToRenderer(FlutterRenderer)}.
   *
   * <p>This {@code RenderSurface} should also clean up any references related to the previously
   * connected {@code FlutterRenderer}.
   */
  void detachFromRenderer();

  /**
   * Instructs this {@code RenderSurface} to stop forwarding {@code Surface} notifications to the
   * {@code FlutterRenderer} that was previously connected with {@link
   * #attachToRenderer(FlutterRenderer)}.
   */
  void pause();

  /**
   * Instructs this {@code RenderSurface} to resume forwarding {@code Surface} notifications to the
   * {@code FlutterRenderer} that was previously connected with {@link
   * #attachToRenderer(FlutterRenderer)}.
   */
  void resume();
}
