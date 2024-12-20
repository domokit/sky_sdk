// Copyright 2013 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

package io.flutter.plugin.platform;

import static io.flutter.Build.API_LEVELS;
import static org.junit.Assert.*;
import static org.mockito.ArgumentMatchers.*;
import static org.mockito.Mockito.*;

import android.annotation.TargetApi;
import android.content.Context;
import android.graphics.Canvas;
import android.graphics.Color;
import android.graphics.SurfaceTexture;
import android.view.Surface;
import android.view.View;
import androidx.test.core.app.ApplicationProvider;
import androidx.test.ext.junit.runners.AndroidJUnit4;
import io.flutter.view.TextureRegistry.SurfaceTextureEntry;
import org.junit.Test;
import org.junit.runner.RunWith;

@TargetApi(API_LEVELS.API_31)
@RunWith(AndroidJUnit4.class)
public class SurfaceTexturePlatformViewRenderTargetTest {
  private final Context ctx = ApplicationProvider.getApplicationContext();

  @Test
  public void viewDraw_writesToBuffer() {
    final Canvas canvas = mock(Canvas.class);
    final Surface surface = mock(Surface.class);
    when(surface.lockHardwareCanvas()).thenReturn(canvas);
    when(surface.isValid()).thenReturn(true);
    final SurfaceTexture surfaceTexture = mock(SurfaceTexture.class);
    final SurfaceTextureEntry surfaceTextureEntry = mock(SurfaceTextureEntry.class);
    when(surfaceTextureEntry.surfaceTexture()).thenReturn(surfaceTexture);
    when(surfaceTexture.isReleased()).thenReturn(false);

    final SurfaceTexturePlatformViewRenderTarget renderTarget =
        new SurfaceTexturePlatformViewRenderTarget(surfaceTextureEntry) {
          @Override
          protected Surface createSurface() {
            return surface;
          }
        };

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

    // Test.
    final Surface s = renderTarget.getSurface();
    final Canvas c = s.lockHardwareCanvas();
    platformView.draw(c);
    s.unlockCanvasAndPost(c);

    // Verify.
    verify(canvas, times(1)).drawColor(Color.RED);
    verify(surface, times(1)).lockHardwareCanvas();
    verify(surface, times(1)).unlockCanvasAndPost(canvas);
    verifyNoMoreInteractions(surface);
  }

  @Test
  public void release() {
    final Canvas canvas = mock(Canvas.class);
    final Surface surface = mock(Surface.class);
    when(surface.lockHardwareCanvas()).thenReturn(canvas);
    when(surface.isValid()).thenReturn(true);
    final SurfaceTexture surfaceTexture = mock(SurfaceTexture.class);
    final SurfaceTextureEntry surfaceTextureEntry = mock(SurfaceTextureEntry.class);
    when(surfaceTextureEntry.surfaceTexture()).thenReturn(surfaceTexture);
    when(surfaceTexture.isReleased()).thenReturn(false);
    final SurfaceTexturePlatformViewRenderTarget renderTarget =
        new SurfaceTexturePlatformViewRenderTarget(surfaceTextureEntry) {
          @Override
          protected Surface createSurface() {
            return surface;
          }
        };

    final Surface s = renderTarget.getSurface();
    final Canvas c = s.lockHardwareCanvas();
    s.unlockCanvasAndPost(c);

    reset(surface);
    reset(surfaceTexture);

    // Test.
    renderTarget.release();

    // Verify.
    verify(surface, times(1)).release();
    verifyNoMoreInteractions(surface);
    verifyNoMoreInteractions(surfaceTexture);
  }
}
