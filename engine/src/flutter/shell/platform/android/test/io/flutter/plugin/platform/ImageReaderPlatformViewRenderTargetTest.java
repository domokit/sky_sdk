// Copyright 2013 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

package io.flutter.plugin.platform;

import static android.os.Looper.getMainLooper;
import static io.flutter.Build.API_LEVELS;
import static org.junit.Assert.*;
import static org.mockito.ArgumentMatchers.*;
import static org.mockito.Mockito.*;
import static org.robolectric.Shadows.shadowOf;

import android.annotation.TargetApi;
import android.content.Context;
import android.graphics.Canvas;
import android.graphics.Color;
import android.graphics.PorterDuff;
import android.media.Image;
import android.view.Surface;
import android.view.View;
import androidx.test.core.app.ApplicationProvider;
import androidx.test.ext.junit.runners.AndroidJUnit4;
import io.flutter.view.TextureRegistry.ImageTextureEntry;
import org.junit.Test;
import org.junit.runner.RunWith;

@TargetApi(API_LEVELS.API_29)
@RunWith(AndroidJUnit4.class)
public class ImageReaderPlatformViewRenderTargetTest {
  private final Context ctx = ApplicationProvider.getApplicationContext();

  class TestImageTextureEntry implements ImageTextureEntry {
    private Image lastPushedImage;

    public long id() {
      return 1;
    }

    public void release() {
      if (this.lastPushedImage != null) {
        this.lastPushedImage.close();
      }
    }

    public void pushImage(Image image) {
      if (this.lastPushedImage != null) {
        this.lastPushedImage.close();
      }
      this.lastPushedImage = image;
    }

    public Image acquireLatestImage() {
      Image r = this.lastPushedImage;
      this.lastPushedImage = null;
      return r;
    }
  }

  @Test
  public void viewDraw_writesToBuffer() {
    final TestImageTextureEntry textureEntry = new TestImageTextureEntry();
    final ImageReaderPlatformViewRenderTarget renderTarget =
        new ImageReaderPlatformViewRenderTarget(textureEntry);
    // Custom view.
    final View platformView =
        new View(ctx) {
          @Override
          public void draw(Canvas canvas) {
            super.draw(canvas);
            canvas.drawColor(Color.RED);
          }
        };
    final int size = 100;
    platformView.measure(size, size);
    platformView.layout(0, 0, size, size);
    renderTarget.resize(size, size);

    // We don't have an image in the texture entry.
    assertNull(textureEntry.acquireLatestImage());

    // Start rendering a frame.
    final Surface s = renderTarget.getSurface();
    assertNotNull(s);
    final Canvas targetCanvas = s.lockHardwareCanvas();
    assertNotNull(targetCanvas);

    try {
      // Fill the render target with transparent pixels. This is needed for platform views that
      // expect a transparent background.
      targetCanvas.drawColor(Color.TRANSPARENT, PorterDuff.Mode.CLEAR);
      // Override the canvas that this subtree of views will use to draw.
      platformView.draw(targetCanvas);
    } finally {
      // Finish rendering a frame.
      s.unlockCanvasAndPost(targetCanvas);
    }

    // Pump the UI thread task loop. This is needed so that the OnImageAvailable callback
    // gets invoked (resulting in textureEntry.pushImage being invoked).
    shadowOf(getMainLooper()).idle();

    // An image was pushed into the texture entry and it has the correct dimensions.
    Image pushedImage = textureEntry.acquireLatestImage();
    assertNotNull(pushedImage);
    assertEquals(pushedImage.getWidth(), size);
    assertEquals(pushedImage.getHeight(), size);
  }
}
