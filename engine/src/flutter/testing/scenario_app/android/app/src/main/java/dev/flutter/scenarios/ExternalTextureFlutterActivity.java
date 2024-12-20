// Copyright 2013 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

package dev.flutter.scenarios;

import static io.flutter.Build.API_LEVELS;

import android.content.res.AssetFileDescriptor;
import android.graphics.Canvas;
import android.graphics.ImageFormat;
import android.graphics.LinearGradient;
import android.graphics.Paint;
import android.graphics.Shader.TileMode;
import android.hardware.HardwareBuffer;
import android.media.Image;
import android.media.ImageReader;
import android.media.ImageWriter;
import android.media.MediaCodec;
import android.media.MediaExtractor;
import android.media.MediaFormat;
import android.os.Build.VERSION;
import android.os.Bundle;
import android.os.Handler;
import android.os.HandlerThread;
import android.util.Log;
import android.view.Gravity;
import android.view.Surface;
import android.view.SurfaceHolder;
import android.view.SurfaceView;
import android.view.ViewGroup;
import android.widget.FrameLayout;
import android.widget.FrameLayout.LayoutParams;
import androidx.annotation.NonNull;
import androidx.annotation.Nullable;
import androidx.annotation.RequiresApi;
import androidx.core.util.Supplier;
import io.flutter.view.TextureRegistry;
import java.io.IOException;
import java.nio.ByteBuffer;
import java.util.Map;
import java.util.Objects;
import java.util.concurrent.CountDownLatch;

public class ExternalTextureFlutterActivity extends TestActivity {
  static final String TAG = "Scenarios";
  private static final int SURFACE_WIDTH = 192;
  private static final int SURFACE_HEIGHT = 256;

  private SurfaceRenderer flutterRenderer;

  // Latch used to ensure both SurfaceRenderers produce a frame before taking a screenshot.
  private final CountDownLatch firstFrameLatch = new CountDownLatch(1);

  private long textureId = 0;
  private TextureRegistry.SurfaceProducer surfaceProducer;

  @Override
  protected void onCreate(@Nullable Bundle savedInstanceState) {
    super.onCreate(savedInstanceState);

    String surfaceRenderer = getIntent().getStringExtra("surface_renderer");
    assert surfaceRenderer != null;
    flutterRenderer = selectSurfaceRenderer(surfaceRenderer);

    // Create and place a SurfaceView above the Flutter content.
    SurfaceView surfaceView = new SurfaceView(getContext());
    surfaceView.setZOrderMediaOverlay(true);
    surfaceView.setMinimumWidth(SURFACE_WIDTH);
    surfaceView.setMinimumHeight(SURFACE_HEIGHT);

    FrameLayout frameLayout = new FrameLayout(getContext());
    frameLayout.addView(
        surfaceView,
        new LayoutParams(
            ViewGroup.LayoutParams.WRAP_CONTENT,
            ViewGroup.LayoutParams.WRAP_CONTENT,
            Gravity.BOTTOM | Gravity.CENTER_HORIZONTAL));

    addContentView(
        frameLayout,
        new ViewGroup.LayoutParams(
            ViewGroup.LayoutParams.MATCH_PARENT, ViewGroup.LayoutParams.MATCH_PARENT));

    SurfaceHolder surfaceHolder = surfaceView.getHolder();
    surfaceHolder.setFixedSize(SURFACE_WIDTH, SURFACE_HEIGHT);
  }

  @Override
  public void waitUntilFlutterRendered() {
    super.waitUntilFlutterRendered();

    try {
      firstFrameLatch.await();
    } catch (InterruptedException e) {
      throw new RuntimeException(e);
    }
  }

  private SurfaceRenderer selectSurfaceRenderer(String surfaceRenderer) {
    switch (surfaceRenderer) {
      case "image":
        if (VERSION.SDK_INT >= API_LEVELS.API_23) {
          // CanvasSurfaceRenderer doesn't work correctly when used with ImageSurfaceRenderer.
          // Use MediaSurfaceRenderer for now.
          return new ImageSurfaceRenderer(selectSurfaceRenderer("media"));
        } else {
          throw new RuntimeException("ImageSurfaceRenderer not supported");
        }
      case "media":
        return new MediaSurfaceRenderer(this::createMediaExtractor);
      case "canvas":
      default:
        return new CanvasSurfaceRenderer();
    }
  }

  private MediaExtractor createMediaExtractor() {
    // Sample Video generated with FFMPEG.
    // ffmpeg -loop 1 -i ~/engine/src/flutter/lib/ui/fixtures/DashInNooglerHat.jpg -c:v libx264
    // -profile:v main -level:v 5.2 -t 1 -r 1 -vf scale=192:256 -b:v 1M sample.mp4
    try {
      MediaExtractor extractor = new MediaExtractor();
      try (AssetFileDescriptor afd = getAssets().openFd("sample.mp4")) {
        extractor.setDataSource(afd.getFileDescriptor(), afd.getStartOffset(), afd.getLength());
      }
      return extractor;
    } catch (IOException e) {
      e.printStackTrace();
      throw new RuntimeException(e);
    }
  }

  @Override
  public void onPause() {
    flutterRenderer.destroy();
    surfaceProducer.release();
    super.onPause();
  }

  @Override
  public void onFlutterUiDisplayed() {
    surfaceProducer =
        Objects.requireNonNull(getFlutterEngine()).getRenderer().createSurfaceProducer();
    surfaceProducer.setSize(SURFACE_WIDTH, SURFACE_HEIGHT);
    flutterRenderer.attach(surfaceProducer.getSurface(), firstFrameLatch);
    flutterRenderer.repaint();
    textureId = surfaceProducer.id();

    super.onFlutterUiDisplayed();
  }

  @Override
  protected void getScenarioParams(@NonNull Map<String, Object> args) {
    super.getScenarioParams(args);
    args.put("texture_id", textureId);
    args.put("texture_width", SURFACE_WIDTH);
    args.put("texture_height", SURFACE_HEIGHT);
  }

  private interface SurfaceRenderer {
    void attach(Surface surface, CountDownLatch onFirstFrame);

    void repaint();

    void destroy();
  }

  /** Paints a simple gradient onto the attached Surface. */
  private static class CanvasSurfaceRenderer implements SurfaceRenderer {
    private Surface surface;
    private CountDownLatch onFirstFrame;

    protected CanvasSurfaceRenderer() {}

    @Override
    public void attach(Surface surface, CountDownLatch onFirstFrame) {
      this.surface = surface;
      this.onFirstFrame = onFirstFrame;
    }

    @Override
    public void repaint() {
      Canvas canvas =
          VERSION.SDK_INT >= API_LEVELS.API_23
              ? surface.lockHardwareCanvas()
              : surface.lockCanvas(null);
      Paint paint = new Paint();
      paint.setShader(
          new LinearGradient(
              0.0f,
              0.0f,
              canvas.getWidth(),
              canvas.getHeight(),
              new int[] {
                // Cyan (#00FFFF)
                0xFF00FFFF,
                // Magenta (#FF00FF)
                0xFFFF00FF,
                // Yellow (#FFFF00)
                0xFFFFFF00,
              },
              null,
              TileMode.REPEAT));
      canvas.drawPaint(paint);
      surface.unlockCanvasAndPost(canvas);

      if (onFirstFrame != null) {
        onFirstFrame.countDown();
        onFirstFrame = null;
      }
    }

    @Override
    public void destroy() {}
  }

  /** Decodes a sample video into the attached Surface. */
  private static class MediaSurfaceRenderer implements SurfaceRenderer {
    private final Supplier<MediaExtractor> extractorSupplier;
    private CountDownLatch onFirstFrame;

    private Surface surface;
    private MediaExtractor extractor;
    private MediaFormat format;
    private Thread decodeThread;

    protected MediaSurfaceRenderer(Supplier<MediaExtractor> extractorSupplier) {
      this.extractorSupplier = extractorSupplier;
    }

    @Override
    public void attach(Surface surface, CountDownLatch onFirstFrame) {
      this.surface = surface;
      this.onFirstFrame = onFirstFrame;

      extractor = extractorSupplier.get();
      format = extractor.getTrackFormat(0);

      decodeThread = new Thread(this::decodeThreadMain);
      decodeThread.start();
    }

    private void decodeThreadMain() {
      try {
        MediaCodec codec =
            MediaCodec.createDecoderByType(
                Objects.requireNonNull(format.getString(MediaFormat.KEY_MIME)));
        codec.configure(format, surface, null, 0);
        codec.start();

        // Track 0 is always the video track, since the sample video doesn't contain audio.
        extractor.selectTrack(0);

        MediaCodec.BufferInfo bufferInfo = new MediaCodec.BufferInfo();
        boolean seenEOS = false;
        long startTimeNs = System.nanoTime();
        int frameCount = 0;

        while (true) {
          // Move samples (video frames) from the extractor into the decoder, as long as we haven't
          // consumed all samples.
          if (!seenEOS) {
            int inputBufferIndex = codec.dequeueInputBuffer(-1);
            ByteBuffer inputBuffer = codec.getInputBuffer(inputBufferIndex);
            assert inputBuffer != null;
            int sampleSize = extractor.readSampleData(inputBuffer, 0);
            if (sampleSize >= 0) {
              long presentationTimeUs = extractor.getSampleTime();
              codec.queueInputBuffer(inputBufferIndex, 0, sampleSize, presentationTimeUs, 0);
              extractor.advance();
            } else {
              codec.queueInputBuffer(
                  inputBufferIndex, 0, 0, 0, MediaCodec.BUFFER_FLAG_END_OF_STREAM);
              seenEOS = true;
            }
          }

          // Then consume decoded video frames from the decoder. These frames are automatically
          // pushed to the attached Surface, so this schedules them for present.
          int outputBufferIndex = codec.dequeueOutputBuffer(bufferInfo, 10000);
          boolean lastBuffer = (bufferInfo.flags & MediaCodec.BUFFER_FLAG_END_OF_STREAM) != 0;
          if (outputBufferIndex >= 0) {
            if (bufferInfo.size > 0) {
              if (onFirstFrame != null) {
                onFirstFrame.countDown();
                onFirstFrame = null;
              }
              Log.w(TAG, "Presenting frame " + frameCount);
              frameCount++;

              codec.releaseOutputBuffer(
                  outputBufferIndex, startTimeNs + (bufferInfo.presentationTimeUs * 1000));
            }
          }

          // Exit the loop if there are no more frames available.
          if (lastBuffer) {
            break;
          }
        }

        codec.stop();
        codec.release();
        extractor.release();
      } catch (IOException e) {
        e.printStackTrace();
        throw new RuntimeException(e);
      }
    }

    @Override
    public void repaint() {}

    @Override
    public void destroy() {
      try {
        decodeThread.join();
      } catch (InterruptedException e) {
        e.printStackTrace();
        throw new RuntimeException(e);
      }
    }
  }

  /**
   * Takes frames from the inner SurfaceRenderer and feeds it through an ImageReader and ImageWriter
   * pair.
   */
  @RequiresApi(API_LEVELS.API_23)
  private static class ImageSurfaceRenderer implements SurfaceRenderer {
    private final SurfaceRenderer inner;
    private CountDownLatch onFirstFrame;
    private ImageReader reader;
    private ImageWriter writer;

    private Handler handler;
    private HandlerThread handlerThread;

    private boolean canReadImage = true;
    private boolean canWriteImage = true;

    protected ImageSurfaceRenderer(SurfaceRenderer inner) {
      this.inner = inner;
    }

    @Override
    public void attach(Surface surface, CountDownLatch onFirstFrame) {
      this.onFirstFrame = onFirstFrame;
      if (VERSION.SDK_INT >= API_LEVELS.API_29) {
        // On Android Q+, use PRIVATE image format.
        // Also let the frame producer know the images will
        // be sampled from by the GPU.
        writer = ImageWriter.newInstance(surface, 3, ImageFormat.PRIVATE);
        reader =
            ImageReader.newInstance(
                SURFACE_WIDTH,
                SURFACE_HEIGHT,
                ImageFormat.PRIVATE,
                2,
                HardwareBuffer.USAGE_GPU_SAMPLED_IMAGE);
      } else {
        // Before Android Q, this will change the format of the surface to match the images.
        writer = ImageWriter.newInstance(surface, 3);
        reader = ImageReader.newInstance(SURFACE_WIDTH, SURFACE_HEIGHT, writer.getFormat(), 2);
      }
      inner.attach(reader.getSurface(), null);

      handlerThread = new HandlerThread("image reader/writer thread");
      handlerThread.start();

      handler = new Handler(handlerThread.getLooper());
      reader.setOnImageAvailableListener(this::onImageAvailable, handler);
      writer.setOnImageReleasedListener(this::onImageReleased, handler);
    }

    private void onImageAvailable(ImageReader reader) {
      Log.v(TAG, "Image available");

      if (!canWriteImage) {
        // If the ImageWriter hasn't released the latest image, don't attempt to enqueue another
        // image.
        // Otherwise the reader writer pair locks up if the writer runs behind, as the reader runs
        // out of images and the writer has no more space for images.
        canReadImage = true;
        return;
      }

      canReadImage = false;
      Image image = reader.acquireLatestImage();
      try {
        canWriteImage = false;
        writer.queueInputImage(image);
      } catch (IllegalStateException e) {
        // If the output surface disconnects, this method will be interrupted with an
        // IllegalStateException.
        // Simply log and return.
        Log.i(TAG, "Surface disconnected from ImageWriter", e);
        image.close();
      }

      Log.v(TAG, "Output image");

      if (onFirstFrame != null) {
        onFirstFrame.countDown();
        onFirstFrame = null;
      }
    }

    private void tryAcquireImage() {
      if (canReadImage) {
        onImageAvailable(reader);
      }
    }

    private void onImageReleased(ImageWriter imageWriter) {
      Log.v(TAG, "Image released");

      if (!canWriteImage) {
        canWriteImage = true;
        if (canReadImage) {
          // Try acquire the image in a handler message, as we may have another call to
          // onImageAvailable in the thread's message queue.
          handler.post(this::tryAcquireImage);
        }
      }
    }

    @Override
    public void repaint() {
      inner.repaint();
    }

    @Override
    public void destroy() {
      Log.i(TAG, "Destroying ImageSurfaceRenderer");
      inner.destroy();
      handler.post(this::destroyReaderWriter);
    }

    private void destroyReaderWriter() {
      writer.close();
      Log.i(TAG, "ImageWriter destroyed");
      reader.close();
      Log.i(TAG, "ImageReader destroyed");
      handlerThread.quitSafely();
    }
  }
}
