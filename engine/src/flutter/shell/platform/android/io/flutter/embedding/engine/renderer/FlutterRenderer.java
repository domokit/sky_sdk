// Copyright 2013 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

package io.flutter.embedding.engine.renderer;

import static io.flutter.Build.API_LEVELS;

import android.annotation.TargetApi;
import android.content.ComponentCallbacks2;
import android.graphics.Bitmap;
import android.graphics.ImageFormat;
import android.graphics.Rect;
import android.graphics.SurfaceTexture;
import android.hardware.HardwareBuffer;
import android.hardware.SyncFence;
import android.media.Image;
import android.media.ImageReader;
import android.os.Build;
import android.os.Handler;
import android.os.Looper;
import android.view.Surface;
import androidx.annotation.Keep;
import androidx.annotation.NonNull;
import androidx.annotation.Nullable;
import androidx.annotation.VisibleForTesting;
import androidx.lifecycle.DefaultLifecycleObserver;
import androidx.lifecycle.LifecycleOwner;
import androidx.lifecycle.ProcessLifecycleOwner;
import io.flutter.Log;
import io.flutter.embedding.engine.FlutterJNI;
import io.flutter.view.TextureRegistry;
import java.io.IOException;
import java.lang.ref.WeakReference;
import java.nio.ByteBuffer;
import java.util.ArrayDeque;
import java.util.ArrayList;
import java.util.HashMap;
import java.util.HashSet;
import java.util.Iterator;
import java.util.List;
import java.util.Set;
import java.util.concurrent.atomic.AtomicLong;

/**
 * Represents the rendering responsibilities of a {@code FlutterEngine}.
 *
 * <p>{@code FlutterRenderer} works in tandem with a provided {@link RenderSurface} to paint Flutter
 * pixels to an Android {@code View} hierarchy.
 *
 * <p>{@code FlutterRenderer} manages textures for rendering, and forwards some Java calls to native
 * Flutter code via JNI. The corresponding {@link RenderSurface} provides the Android {@link
 * Surface} that this renderer paints.
 *
 * <p>{@link io.flutter.embedding.android.FlutterSurfaceView} and {@link
 * io.flutter.embedding.android.FlutterTextureView} are implementations of {@link RenderSurface}.
 */
public class FlutterRenderer implements TextureRegistry {
  /**
   * Whether to always use GL textures for {@link FlutterRenderer#createSurfaceProducer()}.
   *
   * <p>This is a debug-only API intended for local development. For example, when using a newer
   * Android device (that normally would use {@link ImageReaderSurfaceProducer}, but wanting to test
   * the OpenGLES/{@link SurfaceTextureSurfaceProducer} code branch. This flag has undefined
   * behavior if set to true while running in a Vulkan (Impeller) context.
   */
  @VisibleForTesting public static boolean debugForceSurfaceProducerGlTextures = false;

  /** Whether to disable clearing of the Surface used to render platform views. */
  @VisibleForTesting public static boolean debugDisableSurfaceClear = false;

  private static final String TAG = "FlutterRenderer";

  @NonNull private final FlutterJNI flutterJNI;
  @NonNull private final AtomicLong nextTextureId = new AtomicLong(0L);
  @Nullable private Surface surface;
  private boolean isDisplayingFlutterUi = false;
  private final Handler handler = new Handler();

  @NonNull
  private final Set<WeakReference<TextureRegistry.OnTrimMemoryListener>> onTrimMemoryListeners =
      new HashSet<>();

  @NonNull private final List<ImageReaderSurfaceProducer> imageReaderProducers = new ArrayList<>();

  @NonNull
  private final FlutterUiDisplayListener flutterUiDisplayListener =
      new FlutterUiDisplayListener() {
        @Override
        public void onFlutterUiDisplayed() {
          isDisplayingFlutterUi = true;
        }

        @Override
        public void onFlutterUiNoLongerDisplayed() {
          isDisplayingFlutterUi = false;
        }
      };

  public FlutterRenderer(@NonNull FlutterJNI flutterJNI) {
    this.flutterJNI = flutterJNI;
    this.flutterJNI.addIsDisplayingFlutterUiListener(flutterUiDisplayListener);
    ProcessLifecycleOwner.get()
        .getLifecycle()
        .addObserver(
            new DefaultLifecycleObserver() {
              @Override
              public void onResume(@NonNull LifecycleOwner owner) {
                Log.v(TAG, "onResume called; notifying SurfaceProducers");
                for (ImageReaderSurfaceProducer producer : imageReaderProducers) {
                  if (producer.callback != null && producer.notifiedDestroy) {
                    producer.notifiedDestroy = false;
                    producer.callback.onSurfaceAvailable();
                  }
                }
              }
            });
  }

  /**
   * Returns true if this {@code FlutterRenderer} is painting pixels to an Android {@code View}
   * hierarchy, false otherwise.
   */
  public boolean isDisplayingFlutterUi() {
    return isDisplayingFlutterUi;
  }

  /**
   * Adds a listener that is invoked whenever this {@code FlutterRenderer} starts and stops painting
   * pixels to an Android {@code View} hierarchy.
   */
  public void addIsDisplayingFlutterUiListener(@NonNull FlutterUiDisplayListener listener) {
    flutterJNI.addIsDisplayingFlutterUiListener(listener);

    if (isDisplayingFlutterUi) {
      listener.onFlutterUiDisplayed();
    }
  }

  /**
   * Removes a listener added via {@link
   * #addIsDisplayingFlutterUiListener(FlutterUiDisplayListener)}.
   */
  public void removeIsDisplayingFlutterUiListener(@NonNull FlutterUiDisplayListener listener) {
    flutterJNI.removeIsDisplayingFlutterUiListener(listener);
  }

  private void clearDeadListeners() {
    final Iterator<WeakReference<OnTrimMemoryListener>> iterator = onTrimMemoryListeners.iterator();
    while (iterator.hasNext()) {
      WeakReference<OnTrimMemoryListener> listenerRef = iterator.next();
      final OnTrimMemoryListener listener = listenerRef.get();
      if (listener == null) {
        iterator.remove();
      }
    }
  }

  /** Adds a listener that is invoked when a memory pressure warning was forward. */
  @VisibleForTesting
  /* package */ void addOnTrimMemoryListener(@NonNull OnTrimMemoryListener listener) {
    // Purge dead listener to avoid accumulating.
    clearDeadListeners();
    onTrimMemoryListeners.add(new WeakReference<>(listener));
  }

  /**
   * Removes a {@link OnTrimMemoryListener} that was added with {@link
   * #addOnTrimMemoryListener(OnTrimMemoryListener)}.
   */
  @VisibleForTesting
  /* package */ void removeOnTrimMemoryListener(@NonNull OnTrimMemoryListener listener) {
    for (WeakReference<OnTrimMemoryListener> listenerRef : onTrimMemoryListeners) {
      if (listenerRef.get() == listener) {
        onTrimMemoryListeners.remove(listenerRef);
        break;
      }
    }
  }

  // ------ START TextureRegistry IMPLEMENTATION -----

  /**
   * Creates and returns a new external texture {@link SurfaceProducer} managed by the Flutter
   * engine that is also made available to Flutter code.
   */
  @NonNull
  @Override
  public SurfaceProducer createSurfaceProducer() {
    // Prior to Impeller, Flutter on Android *only* ran on OpenGLES (via Skia). That
    // meant that
    // plugins (i.e. end-users) either explicitly created a SurfaceTexture (via
    // createX/registerX) or an ImageTexture (via createX/registerX).
    //
    // In an Impeller world, which for the first time uses (if available) a Vulkan
    // rendering
    // backend, it is no longer possible (at least not trivially) to render an
    // OpenGLES-provided
    // texture (SurfaceTexture) in a Vulkan context.
    //
    // This function picks the "best" rendering surface based on the Android
    // runtime, and
    // provides a consumer-agnostic SurfaceProducer (which in turn vends a Surface),
    // and has
    // plugins (i.e. end-users) use the Surface instead, letting us "hide" the
    // consumer-side
    // of the implementation.
    //
    // tl;dr: If ImageTexture is available, we use it, otherwise we use a
    // SurfaceTexture.
    // Coincidentally, if ImageTexture is available, we are also on an Android
    // version that is
    // running Vulkan, so we don't have to worry about it not being supported.
    final SurfaceProducer entry;
    if (!debugForceSurfaceProducerGlTextures
        && Build.VERSION.SDK_INT >= API_LEVELS.API_29
        && !flutterJNI.ShouldDisableAHB()) {
      final long id = nextTextureId.getAndIncrement();
      final ImageReaderSurfaceProducer producer = new ImageReaderSurfaceProducer(id);
      registerImageTexture(id, producer);
      addOnTrimMemoryListener(producer);
      imageReaderProducers.add(producer);
      Log.v(TAG, "New ImageReaderSurfaceProducer ID: " + id);
      entry = producer;
    } else {
      // TODO(matanlurey): Actually have the class named "*Producer" to well, produce
      // something. This is a code smell, but does guarantee the paths for both
      // createSurfaceTexture and createSurfaceProducer doesn't diverge. As we get more
      // confident in this API and any possible bugs (and have tests to check we don't
      // regress), reconsider this pattern.
      final SurfaceTextureEntry texture = createSurfaceTexture();
      final SurfaceTextureSurfaceProducer producer =
          new SurfaceTextureSurfaceProducer(texture.id(), handler, flutterJNI, texture);
      Log.v(TAG, "New SurfaceTextureSurfaceProducer ID: " + texture.id());
      entry = producer;
    }
    return entry;
  }

  /**
   * Creates and returns a new {@link SurfaceTexture} managed by the Flutter engine that is also
   * made available to Flutter code.
   */
  @NonNull
  @Override
  public SurfaceTextureEntry createSurfaceTexture() {
    Log.v(TAG, "Creating a SurfaceTexture.");
    final SurfaceTexture surfaceTexture = new SurfaceTexture(0);
    return registerSurfaceTexture(surfaceTexture);
  }

  /**
   * Registers and returns a {@link SurfaceTexture} managed by the Flutter engine that is also made
   * available to Flutter code.
   */
  @NonNull
  @Override
  public SurfaceTextureEntry registerSurfaceTexture(@NonNull SurfaceTexture surfaceTexture) {
    return registerSurfaceTexture(nextTextureId.getAndIncrement(), surfaceTexture);
  }

  /**
   * Similar to {@link FlutterRenderer#registerSurfaceTexture} but with an existing @{code
   * textureId}.
   *
   * @param surfaceTexture Surface texture to wrap.
   * @param textureId A texture ID already created that should be assigned to the surface texture.
   */
  @NonNull
  private SurfaceTextureEntry registerSurfaceTexture(
      long textureId, @NonNull SurfaceTexture surfaceTexture) {
    surfaceTexture.detachFromGLContext();
    final SurfaceTextureRegistryEntry entry =
        new SurfaceTextureRegistryEntry(textureId, surfaceTexture);
    Log.v(TAG, "New SurfaceTexture ID: " + entry.id());
    registerTexture(entry.id(), entry.textureWrapper());
    addOnTrimMemoryListener(entry);
    return entry;
  }

  @NonNull
  @Override
  public ImageTextureEntry createImageTexture() {
    final ImageTextureRegistryEntry entry =
        new ImageTextureRegistryEntry(nextTextureId.getAndIncrement());
    Log.v(TAG, "New ImageTextureEntry ID: " + entry.id());
    registerImageTexture(entry.id(), entry);
    return entry;
  }

  @Override
  public void onTrimMemory(int level) {
    final Iterator<WeakReference<OnTrimMemoryListener>> iterator = onTrimMemoryListeners.iterator();
    while (iterator.hasNext()) {
      WeakReference<OnTrimMemoryListener> listenerRef = iterator.next();
      final OnTrimMemoryListener listener = listenerRef.get();
      if (listener != null) {
        listener.onTrimMemory(level);
      } else {
        // Purge cleared refs to avoid accumulating a lot of dead listener
        iterator.remove();
      }
    }
  }

  final class SurfaceTextureRegistryEntry
      implements TextureRegistry.SurfaceTextureEntry, TextureRegistry.OnTrimMemoryListener {
    private final long id;
    @NonNull private final SurfaceTextureWrapper textureWrapper;
    private boolean released;
    @Nullable private OnTrimMemoryListener trimMemoryListener;
    @Nullable private OnFrameConsumedListener frameConsumedListener;

    SurfaceTextureRegistryEntry(long id, @NonNull SurfaceTexture surfaceTexture) {
      this.id = id;
      Runnable onFrameConsumed =
          () -> {
            if (frameConsumedListener != null) {
              frameConsumedListener.onFrameConsumed();
            }
          };
      this.textureWrapper = new SurfaceTextureWrapper(surfaceTexture, onFrameConsumed);

      // Even though we make sure to unregister the callback before releasing, as of
      // Android O, SurfaceTexture has a data race when accessing the callback, so the
      // callback may still be called by a stale reference after released==true and
      // mNativeView==null.
      SurfaceTexture.OnFrameAvailableListener onFrameListener =
          texture -> {
            if (released || !flutterJNI.isAttached()) {
              // Even though we make sure to unregister the callback before releasing, as of
              // Android O, SurfaceTexture has a data race when accessing the callback, so the
              // callback may still be called by a stale reference after released==true and
              // mNativeView==null.
              return;
            }
            textureWrapper.markDirty();
            scheduleEngineFrame();
          };
      // The callback relies on being executed on the UI thread (un-synchronised read of
      // mNativeView and also the engine code check for platform thread in
      // Shell::OnPlatformViewMarkTextureFrameAvailable), so we explicitly pass a Handler for the
      // current thread.
      this.surfaceTexture().setOnFrameAvailableListener(onFrameListener, new Handler());
    }

    @Override
    public void onTrimMemory(int level) {
      if (trimMemoryListener != null) {
        trimMemoryListener.onTrimMemory(level);
      }
    }

    private void removeListener() {
      removeOnTrimMemoryListener(this);
    }

    @NonNull
    public SurfaceTextureWrapper textureWrapper() {
      return textureWrapper;
    }

    @Override
    @NonNull
    public SurfaceTexture surfaceTexture() {
      return textureWrapper.surfaceTexture();
    }

    @Override
    public long id() {
      return id;
    }

    @Override
    public void release() {
      if (released) {
        return;
      }
      Log.v(TAG, "Releasing a SurfaceTexture (" + id + ").");
      textureWrapper.release();
      unregisterTexture(id);
      removeListener();
      released = true;
    }

    @Override
    protected void finalize() throws Throwable {
      try {
        if (released) {
          return;
        }

        handler.post(new TextureFinalizerRunnable(id, flutterJNI));
      } finally {
        super.finalize();
      }
    }

    @Override
    public void setOnFrameConsumedListener(@Nullable OnFrameConsumedListener listener) {
      frameConsumedListener = listener;
    }

    @Override
    public void setOnTrimMemoryListener(@Nullable OnTrimMemoryListener listener) {
      trimMemoryListener = listener;
    }
  }

  static final class TextureFinalizerRunnable implements Runnable {
    private final long id;
    private final FlutterJNI flutterJNI;

    TextureFinalizerRunnable(long id, @NonNull FlutterJNI flutterJNI) {
      this.id = id;
      this.flutterJNI = flutterJNI;
    }

    @Override
    public void run() {
      if (!flutterJNI.isAttached()) {
        return;
      }
      Log.v(TAG, "Releasing a Texture (" + id + ").");
      flutterJNI.unregisterTexture(id);
    }
  }

  // Keep a queue of ImageReaders.
  // Each ImageReader holds acquired Images.
  // When we acquire the next image, close any ImageReaders that don't have any
  // more pending images.
  @Keep
  @TargetApi(API_LEVELS.API_29)
  final class ImageReaderSurfaceProducer
      implements TextureRegistry.SurfaceProducer,
          TextureRegistry.ImageConsumer,
          TextureRegistry.OnTrimMemoryListener {
    private static final String TAG = "ImageReaderSurfaceProducer";
    private static final int MAX_IMAGES = 5;

    // Flip when debugging to see verbose logs.
    private static final boolean VERBOSE_LOGS = false;

    // We must always cleanup on memory pressure on Android 14 due to a bug in Android.
    // It is safe to do on all versions so we unconditionally have this set to true.
    private static final boolean CLEANUP_ON_MEMORY_PRESSURE = true;

    private final long id;

    private boolean released;
    // Will be true in tests and on Android API < 33.
    private boolean ignoringFence = false;

    private static final boolean trimOnMemoryPressure = CLEANUP_ON_MEMORY_PRESSURE;

    // The requested width and height are updated by setSize.
    private int requestedWidth = 1;
    private int requestedHeight = 1;
    // Whenever the requested width and height change we set this to be true so we
    // create a new ImageReader (inside getSurface) with the correct width and height.
    // We use this flag so that we lazily create the ImageReader only when a frame
    // will be produced at that size.
    private boolean createNewReader = true;

    /**
     * Stores whether {@link Callback#onSurfaceDestroyed()} was previously invoked.
     *
     * <p>Used to avoid signaling {@link Callback#onSurfaceAvailable()} unnecessarily.
     */
    private boolean notifiedDestroy = false;

    // State held to track latency of various stages.
    private long lastDequeueTime = 0;
    private long lastQueueTime = 0;
    private long lastScheduleTime = 0;
    private int numTrims = 0;

    private final Object lock = new Object();
    // REQUIRED: The following fields must only be accessed when lock is held.
    private final ArrayDeque<PerImageReader> imageReaderQueue = new ArrayDeque<>();
    private final HashMap<ImageReader, PerImageReader> perImageReaders = new HashMap<>();
    private PerImage lastDequeuedImage = null;
    private PerImageReader lastReaderDequeuedFrom = null;
    private Callback callback = null;

    /** Internal class: state held per Image produced by ImageReaders. */
    private class PerImage {
      public final Image image;
      public final long queuedTime;

      public PerImage(Image image, long queuedTime) {
        this.image = image;
        this.queuedTime = queuedTime;
      }
    }

    /** Internal class: state held per ImageReader. */
    private class PerImageReader {
      public final ImageReader reader;
      private final ArrayDeque<PerImage> imageQueue = new ArrayDeque<>();
      private boolean closed = false;

      public PerImageReader(ImageReader reader) {
        this.reader = reader;
        ImageReader.OnImageAvailableListener onImageAvailableListener =
            r -> {
              Image image = null;
              try {
                image = r.acquireLatestImage();
              } catch (IllegalStateException e) {
                Log.e(TAG, "onImageAvailable acquireLatestImage failed: " + e);
              }
              if (image == null) {
                return;
              }
              if (released || closed) {
                image.close();
                return;
              }
              onImage(r, image);
            };
        reader.setOnImageAvailableListener(
            onImageAvailableListener, new Handler(Looper.getMainLooper()));
      }

      PerImage queueImage(Image image) {
        if (closed) {
          return null;
        }
        PerImage perImage = new PerImage(image, System.nanoTime());
        imageQueue.add(perImage);
        // If we fall too far behind we will skip some frames.
        while (imageQueue.size() > 2) {
          PerImage r = imageQueue.removeFirst();
          if (VERBOSE_LOGS) {
            Log.i(TAG, reader.hashCode() + " force closed image=" + r.image.hashCode());
          }
          r.image.close();
        }
        return perImage;
      }

      PerImage dequeueImage() {
        if (imageQueue.isEmpty()) {
          return null;
        }
        return imageQueue.removeFirst();
      }

      /** returns true if we can prune this reader */
      boolean canPrune() {
        return imageQueue.isEmpty() && lastReaderDequeuedFrom != this;
      }

      boolean imageQueueIsEmpty() {
        return imageQueue.isEmpty();
      }

      void close() {
        closed = true;
        if (VERBOSE_LOGS) {
          Log.i(TAG, "Closing reader=" + reader.hashCode());
        }
        reader.close();
        imageQueue.clear();
      }
    }

    double deltaMillis(long deltaNanos) {
      return (double) deltaNanos / 1000000.0;
    }

    PerImageReader getOrCreatePerImageReader(ImageReader reader) {
      PerImageReader r = perImageReaders.get(reader);
      if (r == null) {
        r = new PerImageReader(reader);
        perImageReaders.put(reader, r);
        imageReaderQueue.add(r);
        if (VERBOSE_LOGS) {
          Log.i(TAG, "imageReaderQueue#=" + imageReaderQueue.size());
        }
      }
      return r;
    }

    void pruneImageReaderQueue() {
      boolean change = false;
      // Prune nodes from the head of the ImageReader queue.
      while (imageReaderQueue.size() > 1) {
        PerImageReader r = imageReaderQueue.peekFirst();
        if (r == null || !r.canPrune()) {
          // No more ImageReaders can be pruned this round.
          break;
        }
        imageReaderQueue.removeFirst();
        perImageReaders.remove(r.reader);
        r.close();
        change = true;
      }
      if (change && VERBOSE_LOGS) {
        Log.i(TAG, "Pruned image reader queue length=" + imageReaderQueue.size());
      }
    }

    void onImage(ImageReader reader, Image image) {
      PerImage queuedImage;
      synchronized (lock) {
        PerImageReader perReader = getOrCreatePerImageReader(reader);
        queuedImage = perReader.queueImage(image);
      }
      if (queuedImage == null) {
        // We got a late image.
        return;
      }
      if (VERBOSE_LOGS) {
        if (lastQueueTime != 0) {
          long now = System.nanoTime();
          long queueDelta = now - lastQueueTime;
          Log.i(
              TAG,
              reader.hashCode()
                  + " enqueued image="
                  + queuedImage.image.hashCode()
                  + " queueDelta="
                  + deltaMillis(queueDelta));
          lastQueueTime = now;
        } else {
          lastQueueTime = System.nanoTime();
        }
      }
      scheduleEngineFrame();
    }

    PerImage dequeueImage() {
      PerImage r = null;
      boolean hasPendingImages = false;
      synchronized (lock) {
        for (PerImageReader reader : imageReaderQueue) {
          r = reader.dequeueImage();
          if (r == null) {
            // This reader is probably about to get pruned.
            continue;
          }
          if (VERBOSE_LOGS) {
            if (lastDequeueTime != 0) {
              long now = System.nanoTime();
              long dequeueDelta = now - lastDequeueTime;
              long queuedFor = now - r.queuedTime;
              long scheduleDelay = now - lastScheduleTime;
              Log.i(
                  TAG,
                  reader.reader.hashCode()
                      + " dequeued image="
                      + r.image.hashCode()
                      + " queuedFor= "
                      + deltaMillis(queuedFor)
                      + " dequeueDelta="
                      + deltaMillis(dequeueDelta)
                      + " scheduleDelay="
                      + deltaMillis(scheduleDelay));
              lastDequeueTime = now;
            } else {
              lastDequeueTime = System.nanoTime();
            }
          }
          if (lastDequeuedImage != null) {
            if (VERBOSE_LOGS) {
              Log.i(
                  TAG,
                  lastReaderDequeuedFrom.reader.hashCode()
                      + " closing image="
                      + lastDequeuedImage.image.hashCode());
            }
            // We must keep the last image dequeued open until we are done presenting
            // it. We have just dequeued a new image (r). Close the previously dequeued
            // image.
            lastDequeuedImage.image.close();
          }
          // Remember the last image and reader dequeued from. We do this because we must
          // keep both of these alive until we are done presenting the image.
          lastDequeuedImage = r;
          lastReaderDequeuedFrom = reader;
          break;
        }
        pruneImageReaderQueue();
        for (PerImageReader reader : imageReaderQueue) {
          if (!reader.imageQueueIsEmpty()) {
            hasPendingImages = true;
            break;
          }
        }
      }
      if (hasPendingImages) {
        // Request another frame to ensure that images are consumed until the queue is empty.
        handler.post(
            () -> {
              if (!released) {
                scheduleEngineFrame();
              }
            });
      }
      return r;
    }

    @Override
    public void onTrimMemory(int level) {
      if (!trimOnMemoryPressure) {
        return;
      }
      if (level < ComponentCallbacks2.TRIM_MEMORY_BACKGROUND) {
        return;
      }
      synchronized (lock) {
        numTrims++;
      }
      cleanup();
      createNewReader = true;
      if (this.callback != null) {
        notifiedDestroy = true;
        this.callback.onSurfaceDestroyed();
      }
    }

    private void releaseInternal() {
      cleanup();
      released = true;
      removeOnTrimMemoryListener(this);
      imageReaderProducers.remove(this);
    }

    private void cleanup() {
      synchronized (lock) {
        for (PerImageReader pir : perImageReaders.values()) {
          if (lastReaderDequeuedFrom == pir) {
            lastReaderDequeuedFrom = null;
          }
          pir.close();
        }
        perImageReaders.clear();
        if (lastDequeuedImage != null) {
          lastDequeuedImage.image.close();
          lastDequeuedImage = null;
        }
        if (lastReaderDequeuedFrom != null) {
          lastReaderDequeuedFrom.close();
          lastReaderDequeuedFrom = null;
        }
        imageReaderQueue.clear();
      }
    }

    @TargetApi(API_LEVELS.API_33)
    private void waitOnFence(Image image) {
      try {
        SyncFence fence = image.getFence();
        fence.awaitForever();
      } catch (IOException e) {
        // Drop.
      }
    }

    private void maybeWaitOnFence(Image image) {
      if (image == null) {
        return;
      }
      if (ignoringFence) {
        return;
      }
      if (Build.VERSION.SDK_INT >= API_LEVELS.API_33) {
        // The fence API is only available on Android >= 33.
        waitOnFence(image);
        return;
      }
      // Log once per ImageTextureEntry.
      ignoringFence = true;
      Log.d(TAG, "ImageTextureEntry can't wait on the fence on Android < 33");
    }

    ImageReaderSurfaceProducer(long id) {
      this.id = id;
    }

    @Override
    public void setCallback(Callback callback) {
      this.callback = callback;
    }

    @Override
    public boolean handlesCropAndRotation() {
      return false;
    }

    @Override
    public long id() {
      return id;
    }

    @Override
    public void release() {
      if (released) {
        return;
      }
      releaseInternal();
      unregisterTexture(id);
    }

    @Override
    public void setSize(int width, int height) {
      // Clamp to a minimum of 1. A 0x0 texture is a runtime exception in ImageReader.
      width = Math.max(1, width);
      height = Math.max(1, height);

      if (requestedWidth == width && requestedHeight == height) {
        // No size change.
        return;
      }
      this.createNewReader = true;
      this.requestedHeight = height;
      this.requestedWidth = width;
    }

    @Override
    public int getWidth() {
      return this.requestedWidth;
    }

    @Override
    public int getHeight() {
      return this.requestedHeight;
    }

    @Override
    public Surface getSurface() {
      PerImageReader pir = getActiveReader();
      if (VERBOSE_LOGS) {
        Log.i(TAG, pir.reader.hashCode() + " returning surface to render a new frame.");
      }
      return pir.reader.getSurface();
    }

    @Override
    public void scheduleFrame() {
      if (VERBOSE_LOGS) {
        long now = System.nanoTime();
        if (lastScheduleTime != 0) {
          long delta = now - lastScheduleTime;
          Log.v(TAG, "scheduleFrame delta=" + deltaMillis(delta));
        }
        lastScheduleTime = now;
      }
      scheduleEngineFrame();
    }

    @Override
    @TargetApi(API_LEVELS.API_29)
    public Image acquireLatestImage() {
      PerImage r = dequeueImage();
      if (r == null) {
        return null;
      }
      maybeWaitOnFence(r.image);
      return r.image;
    }

    private PerImageReader getActiveReader() {
      synchronized (lock) {
        if (createNewReader) {
          createNewReader = false;
          // Create a new ImageReader and add it to the queue.
          ImageReader reader = createImageReader();
          if (VERBOSE_LOGS) {
            Log.i(
                TAG, reader.hashCode() + " created w=" + requestedWidth + " h=" + requestedHeight);
          }
          return getOrCreatePerImageReader(reader);
        }
        return imageReaderQueue.peekLast();
      }
    }

    @Override
    protected void finalize() throws Throwable {
      try {
        if (released) {
          return;
        }
        releaseInternal();
        handler.post(new TextureFinalizerRunnable(id, flutterJNI));
      } finally {
        super.finalize();
      }
    }

    @TargetApi(API_LEVELS.API_33)
    private ImageReader createImageReader33() {
      final ImageReader.Builder builder = new ImageReader.Builder(requestedWidth, requestedHeight);
      // Allow for double buffering.
      builder.setMaxImages(MAX_IMAGES);
      // Use PRIVATE image format so that we can support video decoding.
      // TODO(johnmccutchan): Should we always use PRIVATE here? It may impact our ability to
      // read back texture data. If we don't always want to use it, how do we decide when to
      // use it or not? Perhaps PlatformViews can indicate if they may contain DRM'd content.
      // I need to investigate how PRIVATE impacts our ability to take screenshots or capture
      // the output of Flutter application.
      builder.setImageFormat(ImageFormat.PRIVATE);
      // Hint that consumed images will only be read by GPU.
      builder.setUsage(HardwareBuffer.USAGE_GPU_SAMPLED_IMAGE);
      return builder.build();
    }

    @TargetApi(API_LEVELS.API_29)
    private ImageReader createImageReader29() {
      return ImageReader.newInstance(
          requestedWidth,
          requestedHeight,
          ImageFormat.PRIVATE,
          MAX_IMAGES,
          HardwareBuffer.USAGE_GPU_SAMPLED_IMAGE);
    }

    private ImageReader createImageReader() {
      if (Build.VERSION.SDK_INT >= API_LEVELS.API_33) {
        return createImageReader33();
      } else if (Build.VERSION.SDK_INT >= API_LEVELS.API_29) {
        return createImageReader29();
      }
      throw new UnsupportedOperationException(
          "ImageReaderPlatformViewRenderTarget requires API version 29+");
    }

    @VisibleForTesting
    public void disableFenceForTest() {
      // Roboelectric's implementation of SyncFence is borked.
      ignoringFence = true;
    }

    @VisibleForTesting
    public int numImageReaders() {
      synchronized (lock) {
        return imageReaderQueue.size();
      }
    }

    @VisibleForTesting
    public int numTrims() {
      synchronized (lock) {
        return numTrims;
      }
    }

    @VisibleForTesting
    public int numImages() {
      int r = 0;
      synchronized (lock) {
        for (PerImageReader reader : imageReaderQueue) {
          r += reader.imageQueue.size();
        }
      }
      return r;
    }
  }

  @Keep
  final class ImageTextureRegistryEntry
      implements TextureRegistry.ImageTextureEntry, TextureRegistry.ImageConsumer {
    private static final String TAG = "ImageTextureRegistryEntry";
    private final long id;
    private boolean released;
    private boolean ignoringFence = false;
    private Image image;

    ImageTextureRegistryEntry(long id) {
      this.id = id;
    }

    @Override
    public long id() {
      return id;
    }

    @Override
    public void release() {
      if (released) {
        return;
      }
      released = true;
      if (image != null) {
        image.close();
        image = null;
      }
      unregisterTexture(id);
    }

    @Override
    public void pushImage(Image image) {
      if (released) {
        return;
      }
      Image toClose;
      synchronized (this) {
        toClose = this.image;
        this.image = image;
      }
      // Close the previously pushed buffer.
      if (toClose != null) {
        Log.e(TAG, "Dropping PlatformView Frame");
        toClose.close();
      }
      if (image != null) {
        scheduleEngineFrame();
      }
    }

    @TargetApi(API_LEVELS.API_33)
    private void waitOnFence(Image image) {
      try {
        SyncFence fence = image.getFence();
        fence.awaitForever();
      } catch (IOException e) {
        // Drop.
      }
    }

    @TargetApi(API_LEVELS.API_29)
    private void maybeWaitOnFence(Image image) {
      if (image == null) {
        return;
      }
      if (ignoringFence) {
        return;
      }
      if (Build.VERSION.SDK_INT >= API_LEVELS.API_33) {
        // The fence API is only available on Android >= 33.
        waitOnFence(image);
        return;
      }
      // Log once per ImageTextureEntry.
      ignoringFence = true;
      Log.d(TAG, "ImageTextureEntry can't wait on the fence on Android < 33");
    }

    @Override
    @TargetApi(API_LEVELS.API_29)
    public Image acquireLatestImage() {
      Image r;
      synchronized (this) {
        r = this.image;
        this.image = null;
      }
      maybeWaitOnFence(r);
      return r;
    }

    @Override
    protected void finalize() throws Throwable {
      try {
        if (released) {
          return;
        }
        if (image != null) {
          // Be sure to finalize any cached image.
          image.close();
          image = null;
        }
        released = true;
        handler.post(new TextureFinalizerRunnable(id, flutterJNI));
      } finally {
        super.finalize();
      }
    }
  }
  // ------ END TextureRegistry IMPLEMENTATION ----

  /**
   * Notifies Flutter that the given {@code surface} was created and is available for Flutter
   * rendering.
   *
   * <p>If called more than once, the current native resources are released. This can be undesired
   * if the Engine expects to reuse this surface later. For example, this is true when platform
   * views are displayed in a frame, and then removed in the next frame.
   *
   * <p>To avoid releasing the current surface resources, set {@code keepCurrentSurface} to true.
   *
   * <p>See {@link android.view.SurfaceHolder.Callback} and {@link
   * android.view.TextureView.SurfaceTextureListener}
   *
   * @param surface The render surface.
   * @param onlySwap True if the current active surface should not be detached.
   */
  public void startRenderingToSurface(@NonNull Surface surface, boolean onlySwap) {
    if (!onlySwap) {
      // Stop rendering to the surface releases the associated native resources, which causes
      // a glitch when toggling between rendering to an image view (hybrid composition) and
      // rendering directly to a Surface or Texture view. For more,
      // https://github.com/flutter/flutter/issues/95343
      stopRenderingToSurface();
    }

    this.surface = surface;

    if (onlySwap) {
      // In the swap case we are just swapping the surface that we render to.
      flutterJNI.onSurfaceWindowChanged(surface);
    } else {
      // In the non-swap case we are creating a new surface to render to.
      flutterJNI.onSurfaceCreated(surface);
    }
  }

  /**
   * Swaps the {@link Surface} used to render the current frame.
   *
   * <p>In hybrid composition, the root surfaces changes from {@link
   * android.view.SurfaceHolder#getSurface()} to {@link android.media.ImageReader#getSurface()} when
   * a platform view is in the current frame.
   */
  public void swapSurface(@NonNull Surface surface) {
    this.surface = surface;
    flutterJNI.onSurfaceWindowChanged(surface);
  }

  /**
   * Notifies Flutter that a {@code surface} previously registered with {@link
   * #startRenderingToSurface(Surface, boolean)} has changed size to the given {@code width} and
   * {@code height}.
   *
   * <p>See {@link android.view.SurfaceHolder.Callback} and {@link
   * android.view.TextureView.SurfaceTextureListener}
   */
  public void surfaceChanged(int width, int height) {
    flutterJNI.onSurfaceChanged(width, height);
  }

  /**
   * Notifies Flutter that a {@code surface} previously registered with {@link
   * #startRenderingToSurface(Surface, boolean)} has been destroyed and needs to be released and
   * cleaned up on the Flutter side.
   *
   * <p>See {@link android.view.SurfaceHolder.Callback} and {@link
   * android.view.TextureView.SurfaceTextureListener}
   */
  public void stopRenderingToSurface() {
    if (surface != null) {
      flutterJNI.onSurfaceDestroyed();

      // TODO(mattcarroll): the source of truth for this call should be FlutterJNI, which is
      // where the call to onFlutterUiDisplayed() comes from. However, no such native callback
      // exists yet, so until the engine and FlutterJNI are configured to call us back when
      // rendering stops, we will manually monitor that change here.
      if (isDisplayingFlutterUi) {
        flutterUiDisplayListener.onFlutterUiNoLongerDisplayed();
      }

      isDisplayingFlutterUi = false;
      surface = null;
    }
  }

  private void translateFeatureBounds(int[] displayFeatureBounds, int offset, Rect bounds) {
    displayFeatureBounds[offset] = bounds.left;
    displayFeatureBounds[offset + 1] = bounds.top;
    displayFeatureBounds[offset + 2] = bounds.right;
    displayFeatureBounds[offset + 3] = bounds.bottom;
  }

  /**
   * Notifies Flutter that the viewport metrics, e.g. window height and width, have changed.
   *
   * <p>If the width, height, or devicePixelRatio are less than or equal to 0, this update is
   * ignored.
   *
   * @param viewportMetrics The metrics to send to the Dart application.
   */
  public void setViewportMetrics(@NonNull ViewportMetrics viewportMetrics) {
    // We might get called with just the DPR if width/height aren't available yet.
    // Just ignore, as it will get called again when width/height are set.
    if (!viewportMetrics.validate()) {
      return;
    }
    Log.v(
        TAG,
        "Setting viewport metrics\n"
            + "Size: "
            + viewportMetrics.width
            + " x "
            + viewportMetrics.height
            + "\n"
            + "Padding - L: "
            + viewportMetrics.viewPaddingLeft
            + ", T: "
            + viewportMetrics.viewPaddingTop
            + ", R: "
            + viewportMetrics.viewPaddingRight
            + ", B: "
            + viewportMetrics.viewPaddingBottom
            + "\n"
            + "Insets - L: "
            + viewportMetrics.viewInsetLeft
            + ", T: "
            + viewportMetrics.viewInsetTop
            + ", R: "
            + viewportMetrics.viewInsetRight
            + ", B: "
            + viewportMetrics.viewInsetBottom
            + "\n"
            + "System Gesture Insets - L: "
            + viewportMetrics.systemGestureInsetLeft
            + ", T: "
            + viewportMetrics.systemGestureInsetTop
            + ", R: "
            + viewportMetrics.systemGestureInsetRight
            + ", B: "
            + viewportMetrics.systemGestureInsetRight
            + "\n"
            + "Display Features: "
            + viewportMetrics.displayFeatures.size()
            + "\n"
            + "Display Cutouts: "
            + viewportMetrics.displayCutouts.size());

    int totalFeaturesAndCutouts =
        viewportMetrics.displayFeatures.size() + viewportMetrics.displayCutouts.size();
    int[] displayFeaturesBounds = new int[totalFeaturesAndCutouts * 4];
    int[] displayFeaturesType = new int[totalFeaturesAndCutouts];
    int[] displayFeaturesState = new int[totalFeaturesAndCutouts];
    for (int i = 0; i < viewportMetrics.displayFeatures.size(); i++) {
      DisplayFeature displayFeature = viewportMetrics.displayFeatures.get(i);
      translateFeatureBounds(displayFeaturesBounds, 4 * i, displayFeature.bounds);
      displayFeaturesType[i] = displayFeature.type.encodedValue;
      displayFeaturesState[i] = displayFeature.state.encodedValue;
    }
    int cutoutOffset = viewportMetrics.displayFeatures.size() * 4;
    for (int i = 0; i < viewportMetrics.displayCutouts.size(); i++) {
      DisplayFeature displayCutout = viewportMetrics.displayCutouts.get(i);
      translateFeatureBounds(displayFeaturesBounds, cutoutOffset + 4 * i, displayCutout.bounds);
      displayFeaturesType[viewportMetrics.displayFeatures.size() + i] =
          displayCutout.type.encodedValue;
      displayFeaturesState[viewportMetrics.displayFeatures.size() + i] =
          displayCutout.state.encodedValue;
    }

    flutterJNI.setViewportMetrics(
        viewportMetrics.devicePixelRatio,
        viewportMetrics.width,
        viewportMetrics.height,
        viewportMetrics.viewPaddingTop,
        viewportMetrics.viewPaddingRight,
        viewportMetrics.viewPaddingBottom,
        viewportMetrics.viewPaddingLeft,
        viewportMetrics.viewInsetTop,
        viewportMetrics.viewInsetRight,
        viewportMetrics.viewInsetBottom,
        viewportMetrics.viewInsetLeft,
        viewportMetrics.systemGestureInsetTop,
        viewportMetrics.systemGestureInsetRight,
        viewportMetrics.systemGestureInsetBottom,
        viewportMetrics.systemGestureInsetLeft,
        viewportMetrics.physicalTouchSlop,
        displayFeaturesBounds,
        displayFeaturesType,
        displayFeaturesState);
  }

  // TODO(mattcarroll): describe the native behavior that this invokes
  // TODO(mattcarroll): determine if this is nullable or nonnull
  public Bitmap getBitmap() {
    return flutterJNI.getBitmap();
  }

  // TODO(mattcarroll): describe the native behavior that this invokes
  public void dispatchPointerDataPacket(@NonNull ByteBuffer buffer, int position) {
    flutterJNI.dispatchPointerDataPacket(buffer, position);
  }

  // TODO(mattcarroll): describe the native behavior that this invokes
  private void registerTexture(long textureId, @NonNull SurfaceTextureWrapper textureWrapper) {
    flutterJNI.registerTexture(textureId, textureWrapper);
  }

  private void registerImageTexture(
      long textureId, @NonNull TextureRegistry.ImageConsumer imageTexture) {
    flutterJNI.registerImageTexture(textureId, imageTexture);
  }

  @VisibleForTesting
  /* package */ void scheduleEngineFrame() {
    flutterJNI.scheduleFrame();
  }

  // TODO(mattcarroll): describe the native behavior that this invokes
  private void unregisterTexture(long textureId) {
    flutterJNI.unregisterTexture(textureId);
  }

  // TODO(mattcarroll): describe the native behavior that this invokes
  public boolean isSoftwareRenderingEnabled() {
    return flutterJNI.getIsSoftwareRenderingEnabled();
  }

  // TODO(mattcarroll): describe the native behavior that this invokes
  public void setAccessibilityFeatures(int flags) {
    flutterJNI.setAccessibilityFeatures(flags);
  }

  // TODO(mattcarroll): describe the native behavior that this invokes
  public void setSemanticsEnabled(boolean enabled) {
    flutterJNI.setSemanticsEnabled(enabled);
  }

  // TODO(mattcarroll): describe the native behavior that this invokes
  public void dispatchSemanticsAction(
      int nodeId, int action, @Nullable ByteBuffer args, int argsPosition) {
    flutterJNI.dispatchSemanticsAction(nodeId, action, args, argsPosition);
  }

  /**
   * Mutable data structure that holds all viewport metrics properties that Flutter cares about.
   *
   * <p>All distance measurements, e.g., width, height, padding, viewInsets, are measured in device
   * pixels, not logical pixels.
   */
  public static final class ViewportMetrics {
    /** A value that indicates the setting has not been set. */
    public static final int unsetValue = -1;

    public float devicePixelRatio = 1.0f;
    public int width = 0;
    public int height = 0;
    // The fields prefixed with viewPadding and viewInset are used to calculate the padding,
    // viewPadding, and viewInsets of ViewConfiguration in Dart. This calculation is performed at
    // https://github.com/flutter/engine/blob/main/lib/ui/hooks.dart#L139-L155.
    public int viewPaddingTop = 0;
    public int viewPaddingRight = 0;
    public int viewPaddingBottom = 0;
    public int viewPaddingLeft = 0;
    public int viewInsetTop = 0;
    public int viewInsetRight = 0;
    public int viewInsetBottom = 0;
    public int viewInsetLeft = 0;
    public int systemGestureInsetTop = 0;
    public int systemGestureInsetRight = 0;
    public int systemGestureInsetBottom = 0;
    public int systemGestureInsetLeft = 0;
    public int physicalTouchSlop = unsetValue;

    /**
     * Whether this instance contains valid metrics for the Flutter application.
     *
     * @return True if width, height, and devicePixelRatio are > 0; false otherwise.
     */
    boolean validate() {
      return width > 0 && height > 0 && devicePixelRatio > 0;
    }

    // Features
    private final List<DisplayFeature> displayFeatures = new ArrayList<>();

    // Specifically display cutouts.
    private final List<DisplayFeature> displayCutouts = new ArrayList<>();

    public List<DisplayFeature> getDisplayFeatures() {
      return displayFeatures;
    }

    public List<DisplayFeature> getDisplayCutouts() {
      return displayCutouts;
    }

    public void setDisplayFeatures(List<DisplayFeature> newFeatures) {
      displayFeatures.clear();
      displayFeatures.addAll(newFeatures);
    }

    public void setDisplayCutouts(List<DisplayFeature> newCutouts) {
      displayCutouts.clear();
      displayCutouts.addAll(newCutouts);
    }
  }

  /**
   * Description of a physical feature on the display.
   *
   * <p>A display feature is a distinctive physical attribute located within the display panel of
   * the device. It can intrude into the application window space and create a visual distortion,
   * visual or touch discontinuity, make some area invisible or create a logical divider or
   * separation in the screen space.
   *
   * <p>Based on {@link androidx.window.layout.DisplayFeature}, with added support for cutouts.
   */
  public static final class DisplayFeature {
    public final Rect bounds;
    public final DisplayFeatureType type;
    public final DisplayFeatureState state;

    public DisplayFeature(Rect bounds, DisplayFeatureType type, DisplayFeatureState state) {
      this.bounds = bounds;
      this.type = type;
      this.state = state;
    }
  }

  /**
   * Types of display features that can appear on the viewport.
   *
   * <p>Some, like {@link #FOLD}, can be reported without actually occluding the screen. They are
   * useful for knowing where the display is bent or has a crease. The {@link DisplayFeature#bounds}
   * can be 0-width in such cases.
   */
  public enum DisplayFeatureType {
    /**
     * Type of display feature not yet known to Flutter. This can happen if WindowManager is updated
     * with new types. The {@link DisplayFeature#bounds} is the only known property.
     */
    UNKNOWN(0),

    /**
     * A fold in the flexible display that does not occlude the screen. Corresponds to {@link
     * androidx.window.layout.FoldingFeature.OcclusionType#NONE}
     */
    FOLD(1),

    /**
     * Splits the display in two separate panels that can fold. Occludes the screen. Corresponds to
     * {@link androidx.window.layout.FoldingFeature.OcclusionType#FULL}
     */
    HINGE(2),

    /**
     * Area of the screen that usually houses cameras or sensors. Occludes the screen. Corresponds
     * to {@link android.view.DisplayCutout}
     */
    CUTOUT(3);

    public final int encodedValue;

    DisplayFeatureType(int encodedValue) {
      this.encodedValue = encodedValue;
    }
  }

  /**
   * State of the display feature.
   *
   * <p>For foldables, the state is the posture. For cutouts, this property is {@link #UNKNOWN}
   */
  public enum DisplayFeatureState {
    /** The display feature is a cutout or this state is new and not yet known to Flutter. */
    UNKNOWN(0),

    /**
     * The foldable device is completely open. The screen space that is presented to the user is
     * flat. Corresponds to {@link androidx.window.layout.FoldingFeature.State#FLAT}
     */
    POSTURE_FLAT(1),

    /**
     * The foldable device's hinge is in an intermediate position between opened and closed state.
     * There is a non-flat angle between parts of the flexible screen or between physical display
     * panels. Corresponds to {@link androidx.window.layout.FoldingFeature.State#HALF_OPENED}
     */
    POSTURE_HALF_OPENED(2);

    public final int encodedValue;

    DisplayFeatureState(int encodedValue) {
      this.encodedValue = encodedValue;
    }
  }
}
