// Copyright 2013 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

package io.flutter.view;

import android.graphics.SurfaceTexture;
import android.media.Image;
import android.view.Surface;
import androidx.annotation.Keep;
import androidx.annotation.NonNull;
import androidx.annotation.Nullable;

// TODO(mattcarroll): re-evalute docs in this class and add nullability annotations.
/**
 * Registry of backend textures used with a single {@link io.flutter.embedding.android.FlutterView}
 * instance. Entries may be embedded into the Flutter view using the <a
 * href="https://api.flutter.dev/flutter/widgets/Texture-class.html">Texture</a> widget.
 */
public interface TextureRegistry {
  /**
   * Creates and registers a SurfaceProducer texture managed by the Flutter engine.
   *
   * @return A SurfaceProducer.
   */
  @NonNull
  SurfaceProducer createSurfaceProducer();

  /**
   * Creates and registers a SurfaceTexture managed by the Flutter engine.
   *
   * @return A SurfaceTextureEntry.
   */
  @NonNull
  SurfaceTextureEntry createSurfaceTexture();

  /**
   * Registers a SurfaceTexture managed by the Flutter engine.
   *
   * @return A SurfaceTextureEntry.
   */
  @NonNull
  SurfaceTextureEntry registerSurfaceTexture(@NonNull SurfaceTexture surfaceTexture);

  /**
   * Creates and registers a texture managed by the Flutter engine.
   *
   * @return a ImageTextureEntry.
   */
  @NonNull
  ImageTextureEntry createImageTexture();

  /**
   * Callback invoked when memory is low.
   *
   * <p>Invoke this from {@link android.app.Activity#onTrimMemory(int)}.
   */
  default void onTrimMemory(int level) {}

  /** An entry in the texture registry. */
  interface TextureEntry {
    /** @return The identity of this texture. */
    long id();

    /** De-registers and releases all resources . */
    void release();
  }

  /** Uses a Surface to populate the texture. */
  @Keep
  interface SurfaceProducer extends TextureEntry {
    /** Specify the size of this texture in physical pixels */
    void setSize(int width, int height);

    /** @return The currently specified width (physical pixels) */
    int getWidth();

    /** @return The currently specified height (physical pixels) */
    int getHeight();

    /**
     * Direct access to the surface object.
     *
     * <p>When using this API, you will usually need to implement {@link SurfaceProducer.Callback}
     * and provide it to {@link #setCallback(Callback)} in order to be notified when an existing
     * surface has been destroyed (such as when the application goes to the background) or a new
     * surface has been created (such as when the application is resumed back to the foreground).
     *
     * <p>NOTE: You should not cache the returned surface but instead invoke {@code getSurface} each
     * time you need to draw. The surface may change when the texture is resized or has its format
     * changed.
     *
     * @return a Surface to use for a drawing target for various APIs.
     */
    Surface getSurface();

    /**
     * Sets a callback that is notified when a previously created {@link Surface} returned by {@link
     * SurfaceProducer#getSurface()} is no longer valid due to being destroyed, or a new surface is
     * now available (after the previous one was destroyed) for rendering.
     *
     * @param callback The callback to notify, or null to remove the callback.
     */
    void setCallback(Callback callback);

    /** Callback invoked by {@link #setCallback(Callback)}. */
    interface Callback {
      /**
       * An alias for {@link Callback#onSurfaceAvailable()} with a less accurate name.
       *
       * @deprecated Override and use {@link Callback#onSurfaceAvailable()} instead.
       */
      @Deprecated(since = "Flutter 3.27", forRemoval = true)
      default void onSurfaceCreated() {}

      /**
       * Invoked when an Android application is resumed after {@link Callback#onSurfaceDestroyed()}.
       *
       * <p>Applications should now call {@link SurfaceProducer#getSurface()} to get a new
       * {@link Surface}, as the previous one was destroyed and released as a result of a low memory
       * event from the Android OS.
       *
       * <pre>
       * {@code
       * void example(SurfaceProducer producer) {
       *   producer.setCallback(new SurfaceProducer.Callback() {
       *     @override
       *     public void onSurfaceAvailable() {
       *       Surface surface = producer.getSurface();
       *       redrawOrUse(surface);
       *     }
       *
       *     // ...
       *   });
       * }
       * }
       * </pre>
       */
      default void onSurfaceAvailable() {
        this.onSurfaceCreated();
      }

      /**
       * Invoked when a {@link Surface} returned by {@link SurfaceProducer#getSurface()} is invalid.
       *
       * <p>In a low memory environment, the Android OS will signal to Flutter to release resources,
       * such as surfaces, that are not currently in use, such as when the application is in the
       * background, and this method is subsequently called to notify a plugin author to stop
       * using or rendering to the last surface.
       *
       * <p>Use {@link Callback#onSurfaceAvailable()} to be notified to resume rendering.
       *
       * <pre>
       * {@code
       * void example(SurfaceProducer producer) {
       *   producer.setCallback(new SurfaceProducer.Callback() {
       *     @override
       *     public void onSurfaceDestroyed() {
       *       // Store information about the last frame, if necessary.
       *       // Potentially release other dependent resources.
       *     }
       *
       *     // ...
       *   });
       * }
       * }
       * </pre>
       */
      void onSurfaceDestroyed();
    }

    /** This method is not officially part of the public API surface and will be deprecated. */
    void scheduleFrame();

    /**
     * Returns whether the current rendering path handles crop and rotation metadata.
     *
     * <p>On most newer Android devices (API 29+), a {@link android.media.ImageReader} backend is
     * used, which has more features, works in new graphic backends directly (such as Impeller's
     * Vulkan backend), and is the Android recommended solution. However, crop and rotation metadata
     * are <strong>not</strong> handled automatically, and require plugin authors to make
     * appropriate changes ({@see https://github.com/flutter/flutter/issues/144407}).
     *
     * <pre>{@code
     * void example(SurfaceProducer producer) {
     *   bool supported = producer.handlesCropAndRotation();
     *   if (!supported) {
     *       // Manually rotate/crop, either in the Android plugin or in the Dart framework layer.
     *   }
     * }
     * }</pre>
     *
     * @return {@code true} if crop and rotation is handled automatically, {@code false} otherwise.
     */
    boolean handlesCropAndRotation();
  }

  /** A registry entry for a managed SurfaceTexture. */
  @Keep
  interface SurfaceTextureEntry extends TextureEntry {
    /** @return The managed SurfaceTexture. */
    @NonNull
    SurfaceTexture surfaceTexture();

    /** Set a listener that will be notified when the most recent image has been consumed. */
    default void setOnFrameConsumedListener(@Nullable OnFrameConsumedListener listener) {}

    /** Set a listener that will be notified when a memory pressure warning was forward. */
    default void setOnTrimMemoryListener(@Nullable OnTrimMemoryListener listener) {}
  }

  @Keep
  interface ImageTextureEntry extends TextureEntry {
    /**
     * Next paint will update texture to use the contents of image.
     *
     * <p>NOTE: Caller should not call Image.close() on the pushed image.
     *
     * <p>NOTE: In the case that multiple calls to PushFrame occur before the next paint only the
     * last frame pushed will be used (dropping the missed frames).
     */
    void pushImage(Image image);
  }

  /** Listener invoked when the most recent image has been consumed. */
  interface OnFrameConsumedListener {
    /**
     * This method will to be invoked when the most recent image from the image stream has been
     * consumed.
     */
    void onFrameConsumed();
  }

  /** Listener invoked when a memory pressure warning was forward. */
  interface OnTrimMemoryListener {
    /** This method will be invoked when a memory pressure warning was forward. */
    void onTrimMemory(int level);
  }

  @Keep
  interface ImageConsumer {
    /**
     * Retrieve the last Image produced. Drops all previously produced images.
     *
     * <p>NOTE: Caller must call Image.close() on returned image.
     *
     * @return Image or null.
     */
    @Nullable
    Image acquireLatestImage();
  }

  @Keep
  interface GLTextureConsumer {
    /**
     * Retrieve the last GL texture produced.
     *
     * @return SurfaceTexture.
     */
    @NonNull
    SurfaceTexture getSurfaceTexture();
  }
}
