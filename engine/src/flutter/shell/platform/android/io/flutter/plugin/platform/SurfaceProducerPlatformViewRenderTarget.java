package io.flutter.plugin.platform;

import static io.flutter.Build.API_LEVELS;

import android.annotation.TargetApi;
import android.view.Surface;
import io.flutter.view.TextureRegistry.SurfaceProducer;

@TargetApi(API_LEVELS.API_29)
public class SurfaceProducerPlatformViewRenderTarget implements PlatformViewRenderTarget {
  private static final String TAG = "SurfaceProducerRenderTarget";
  private SurfaceProducer producer;

  public SurfaceProducerPlatformViewRenderTarget(SurfaceProducer producer) {
    this.producer = producer;
  }

  // Called when the render target should be resized.
  public void resize(int width, int height) {
    this.producer.setSize(width, height);
  }

  // Returns the currently specified width.
  public int getWidth() {
    return this.producer.getWidth();
  }

  // Returns the currently specified height.
  public int getHeight() {
    return this.producer.getHeight();
  }

  // The id of this render target.
  public long getId() {
    return this.producer.id();
  }

  // Releases backing resources.
  public void release() {
    this.producer.release();
    this.producer = null;
  }

  // Returns true in the case that backing resource have been released.
  public boolean isReleased() {
    return this.producer == null;
  }

  // Returns the Surface to be rendered on to.
  public Surface getSurface() {
    return this.producer.getSurface();
  }

  public void scheduleFrame() {
    this.producer.scheduleFrame();
  }
}
