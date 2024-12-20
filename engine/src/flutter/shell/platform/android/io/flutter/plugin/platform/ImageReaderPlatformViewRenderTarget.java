package io.flutter.plugin.platform;

import static io.flutter.Build.API_LEVELS;

import android.annotation.TargetApi;
import android.graphics.ImageFormat;
import android.hardware.HardwareBuffer;
import android.media.Image;
import android.media.ImageReader;
import android.os.Build;
import android.os.Handler;
import android.view.Surface;
import io.flutter.Log;
import io.flutter.view.TextureRegistry.ImageTextureEntry;

@TargetApi(API_LEVELS.API_29)
public class ImageReaderPlatformViewRenderTarget implements PlatformViewRenderTarget {
  private ImageTextureEntry textureEntry;
  private ImageReader reader;
  private int bufferWidth = 0;
  private int bufferHeight = 0;
  private static final String TAG = "ImageReaderPlatformViewRenderTarget";
  private static final int MAX_IMAGES = 4;

  private void closeReader() {
    if (this.reader != null) {
      // Push a null image, forcing the texture entry to close any cached images.
      textureEntry.pushImage(null);
      // Close the reader, which also closes any produced images.
      this.reader.close();
      this.reader = null;
    }
  }

  private final Handler onImageAvailableHandler = new Handler();
  private final ImageReader.OnImageAvailableListener onImageAvailableListener =
      new ImageReader.OnImageAvailableListener() {
        @Override
        public void onImageAvailable(ImageReader reader) {
          Image image = null;
          try {
            image = reader.acquireLatestImage();
          } catch (IllegalStateException e) {
            Log.e(TAG, "onImageAvailable acquireLatestImage failed: " + e.toString());
          }
          if (image == null) {
            return;
          }
          textureEntry.pushImage(image);
        }
      };

  @TargetApi(API_LEVELS.API_33)
  protected ImageReader createImageReader33() {
    final ImageReader.Builder builder = new ImageReader.Builder(bufferWidth, bufferHeight);
    // Allow for double buffering.
    builder.setMaxImages(MAX_IMAGES);
    // Use PRIVATE image format so that we can support video decoding.
    // TODO(johnmccutchan): Should we always use PRIVATE here? It may impact our ability to read
    // back texture data. If we don't always want to use it, how do we decide when to use it or not?
    // Perhaps PlatformViews can indicate if they may contain DRM'd content. I need to investigate
    // how PRIVATE impacts our ability to take screenshots or capture the output of Flutter
    // application.
    builder.setImageFormat(ImageFormat.PRIVATE);
    // Hint that consumed images will only be read by GPU.
    builder.setUsage(HardwareBuffer.USAGE_GPU_SAMPLED_IMAGE);
    final ImageReader reader = builder.build();
    reader.setOnImageAvailableListener(this.onImageAvailableListener, onImageAvailableHandler);
    return reader;
  }

  @TargetApi(API_LEVELS.API_29)
  protected ImageReader createImageReader29() {
    final ImageReader reader =
        ImageReader.newInstance(
            bufferWidth,
            bufferHeight,
            ImageFormat.PRIVATE,
            MAX_IMAGES,
            HardwareBuffer.USAGE_GPU_SAMPLED_IMAGE);
    reader.setOnImageAvailableListener(this.onImageAvailableListener, onImageAvailableHandler);
    return reader;
  }

  protected ImageReader createImageReader() {
    if (Build.VERSION.SDK_INT >= API_LEVELS.API_33) {
      return createImageReader33();
    } else if (Build.VERSION.SDK_INT >= API_LEVELS.API_29) {
      return createImageReader29();
    }
    throw new UnsupportedOperationException(
        "ImageReaderPlatformViewRenderTarget requires API version 29+");
  }

  public ImageReaderPlatformViewRenderTarget(ImageTextureEntry textureEntry) {
    if (Build.VERSION.SDK_INT < API_LEVELS.API_29) {
      throw new UnsupportedOperationException(
          "ImageReaderPlatformViewRenderTarget requires API version 29+");
    }
    this.textureEntry = textureEntry;
  }

  public void resize(int width, int height) {
    if (this.reader != null && bufferWidth == width && bufferHeight == height) {
      // No size change.
      return;
    }
    closeReader();
    bufferWidth = width;
    bufferHeight = height;
    this.reader = createImageReader();
  }

  public int getWidth() {
    return this.bufferWidth;
  }

  public int getHeight() {
    return this.bufferHeight;
  }

  public long getId() {
    return this.textureEntry.id();
  }

  public void release() {
    closeReader();
    // textureEntry has a finalizer attached.
    textureEntry = null;
  }

  public boolean isReleased() {
    return this.textureEntry == null;
  }

  public Surface getSurface() {
    return this.reader.getSurface();
  }
}
