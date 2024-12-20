package io.flutter.embedding.engine.renderer;

import android.graphics.SurfaceTexture;
import android.os.Handler;
import android.view.Surface;
import androidx.annotation.NonNull;
import androidx.annotation.Nullable;
import io.flutter.embedding.engine.FlutterJNI;
import io.flutter.view.TextureRegistry;

/** Uses a {@link android.graphics.SurfaceTexture} to populate the texture registry. */
final class SurfaceTextureSurfaceProducer
    implements TextureRegistry.SurfaceProducer, TextureRegistry.GLTextureConsumer {
  private final long id;
  private int requestBufferWidth;
  private int requestedBufferHeight;
  private boolean released;
  @Nullable private Surface surface;
  @NonNull private final TextureRegistry.SurfaceTextureEntry texture;
  @NonNull private final Handler handler;
  @NonNull private final FlutterJNI flutterJNI;

  SurfaceTextureSurfaceProducer(
      long id,
      @NonNull Handler handler,
      @NonNull FlutterJNI flutterJNI,
      @NonNull TextureRegistry.SurfaceTextureEntry texture) {
    this.id = id;
    this.handler = handler;
    this.flutterJNI = flutterJNI;
    this.texture = texture;
  }

  @Override
  protected void finalize() throws Throwable {
    try {
      if (released) {
        return;
      }
      release();
      handler.post(new FlutterRenderer.TextureFinalizerRunnable(id, flutterJNI));
    } finally {
      super.finalize();
    }
  }

  @Override
  public long id() {
    return id;
  }

  @Override
  public void release() {
    texture.release();
    released = true;
  }

  @Override
  public void setCallback(Callback callback) {
    // Intentionally blank: SurfaceTextures don't get platform notifications or cleanup.
  }

  @Override
  public boolean handlesCropAndRotation() {
    return true;
  }

  @Override
  @NonNull
  public SurfaceTexture getSurfaceTexture() {
    return texture.surfaceTexture();
  }

  @Override
  public void setSize(int width, int height) {
    requestBufferWidth = width;
    requestedBufferHeight = height;
    getSurfaceTexture().setDefaultBufferSize(width, height);
  }

  @Override
  public int getWidth() {
    return requestBufferWidth;
  }

  @Override
  public int getHeight() {
    return requestedBufferHeight;
  }

  @Override
  public Surface getSurface() {
    if (surface == null) {
      surface = new Surface(texture.surfaceTexture());
    }
    return surface;
  }

  @Override
  public void scheduleFrame() {
    flutterJNI.markTextureFrameAvailable(id);
  }
}
