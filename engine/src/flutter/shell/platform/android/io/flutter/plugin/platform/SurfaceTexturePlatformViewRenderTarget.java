package io.flutter.plugin.platform;

import static android.content.ComponentCallbacks2.TRIM_MEMORY_COMPLETE;
import static io.flutter.Build.API_LEVELS;

import android.annotation.TargetApi;
import android.graphics.SurfaceTexture;
import android.os.Build;
import android.view.Surface;
import io.flutter.view.TextureRegistry;
import io.flutter.view.TextureRegistry.SurfaceTextureEntry;

@TargetApi(API_LEVELS.API_26)
public class SurfaceTexturePlatformViewRenderTarget implements PlatformViewRenderTarget {
  private static final String TAG = "SurfaceTexturePlatformViewRenderTarget";

  private final SurfaceTextureEntry surfaceTextureEntry;

  private SurfaceTexture surfaceTexture;
  private Surface surface;
  private int bufferWidth = 0;
  private int bufferHeight = 0;

  private boolean shouldRecreateSurfaceForLowMemory = false;
  private final TextureRegistry.OnTrimMemoryListener trimMemoryListener =
      new TextureRegistry.OnTrimMemoryListener() {
        @Override
        public void onTrimMemory(int level) {
          // When a memory pressure warning is received and the level equal {@code
          // ComponentCallbacks2.TRIM_MEMORY_COMPLETE}, the Android system releases the underlying
          // surface. If we continue to use the surface (e.g., call lockHardwareCanvas), a crash
          // occurs, and we found that this crash appeared on Android 10 and above.
          // See https://github.com/flutter/flutter/issues/103870 for more details.
          //
          // Here our workaround is to recreate the surface before using it.
          if (level == TRIM_MEMORY_COMPLETE && Build.VERSION.SDK_INT >= API_LEVELS.API_29) {
            shouldRecreateSurfaceForLowMemory = true;
          }
        }
      };

  private void recreateSurfaceIfNeeded() {
    if (surface != null && !shouldRecreateSurfaceForLowMemory) {
      // No need to recreate the surface.
      return;
    }
    if (surface != null) {
      surface.release();
      surface = null;
    }
    surface = createSurface();
    shouldRecreateSurfaceForLowMemory = false;
  }

  protected Surface createSurface() {
    return new Surface(surfaceTexture);
  }

  /** Implementation of PlatformViewRenderTarget */
  public SurfaceTexturePlatformViewRenderTarget(SurfaceTextureEntry surfaceTextureEntry) {
    if (Build.VERSION.SDK_INT < API_LEVELS.API_23) {
      throw new UnsupportedOperationException(
          "Platform views cannot be displayed below API level 23"
              + "You can prevent this issue by setting `minSdkVersion: 23` in build.gradle.");
    }
    this.surfaceTextureEntry = surfaceTextureEntry;
    this.surfaceTexture = surfaceTextureEntry.surfaceTexture();
    surfaceTextureEntry.setOnTrimMemoryListener(trimMemoryListener);
  }

  public void resize(int width, int height) {
    bufferWidth = width;
    bufferHeight = height;
    if (surfaceTexture != null) {
      surfaceTexture.setDefaultBufferSize(bufferWidth, bufferHeight);
    }
  }

  public int getWidth() {
    return bufferWidth;
  }

  public int getHeight() {
    return bufferHeight;
  }

  public long getId() {
    return this.surfaceTextureEntry.id();
  }

  public boolean isReleased() {
    return surfaceTexture == null;
  }

  public void release() {
    // Don't release the texture, let the GC finalize it.
    surfaceTexture = null;
    if (surface != null) {
      surface.release();
      surface = null;
    }
  }

  public Surface getSurface() {
    recreateSurfaceIfNeeded();
    if (surfaceTexture == null || surfaceTexture.isReleased()) {
      return null;
    }
    return surface;
  }
}
