// Copyright 2013 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

package dev.flutter.scenarios;

import static io.flutter.Build.API_LEVELS;

import android.annotation.TargetApi;
import android.content.Context;
import android.content.ContextWrapper;
import android.graphics.Canvas;
import android.graphics.Color;
import android.graphics.Paint;
import android.view.Surface;
import android.view.SurfaceHolder;
import android.view.SurfaceView;
import android.view.View;
import androidx.annotation.NonNull;
import androidx.annotation.Nullable;
import io.flutter.Log;
import io.flutter.plugin.common.MessageCodec;
import io.flutter.plugin.common.StringCodec;
import io.flutter.plugin.platform.PlatformView;
import io.flutter.plugin.platform.PlatformViewFactory;
import java.nio.ByteBuffer;

@TargetApi(API_LEVELS.API_23)
public final class SurfacePlatformViewFactory extends PlatformViewFactory {
  private boolean preserveContext;

  SurfacePlatformViewFactory(boolean preserveContext) {
    super(
        new MessageCodec<Object>() {
          @Nullable
          @Override
          public ByteBuffer encodeMessage(@Nullable Object o) {
            if (o instanceof String) {
              return StringCodec.INSTANCE.encodeMessage((String) o);
            }
            return null;
          }

          @Nullable
          @Override
          public Object decodeMessage(@Nullable ByteBuffer byteBuffer) {
            return StringCodec.INSTANCE.decodeMessage(byteBuffer);
          }
        });
    this.preserveContext = preserveContext;
  }

  @SuppressWarnings("unchecked")
  @Override
  @NonNull
  public PlatformView create(@NonNull Context context, int id, @Nullable Object args) {
    if (preserveContext) {
      return new SurfacePlatformView(context);
    } else {
      final Context differentContext = new ContextWrapper(context);
      return new SurfacePlatformView(differentContext);
    }
  }

  private static class SurfacePlatformView implements PlatformView {
    static String TAG = "SurfacePlatformView";

    final SurfaceView surfaceView;

    @SuppressWarnings("unchecked")
    SurfacePlatformView(@NonNull final Context context) {
      surfaceView = new SurfaceView(context);
      surfaceView
          .getHolder()
          .addCallback(
              new SurfaceHolder.Callback() {
                @Override
                public void surfaceCreated(SurfaceHolder holder) {
                  Log.i(TAG, "surfaceCreated");
                  final Surface surface = holder.getSurface();
                  final Canvas canvas = surface.lockHardwareCanvas();
                  canvas.drawColor(Color.WHITE);

                  final Paint paint = new Paint();
                  paint.setColor(Color.RED);
                  canvas.drawCircle(canvas.getWidth() / 2, canvas.getHeight() / 2, 20, paint);
                  surface.unlockCanvasAndPost(canvas);
                }

                @Override
                public void surfaceChanged(
                    SurfaceHolder holder, int format, int width, int height) {
                  Log.i(TAG, "surfaceChanged");
                }

                @Override
                public void surfaceDestroyed(SurfaceHolder holder) {
                  Log.i(TAG, "surfaceDestroyed");
                }
              });
    }

    @Override
    @NonNull
    public View getView() {
      return surfaceView;
    }

    @Override
    public void dispose() {}
  }
}
